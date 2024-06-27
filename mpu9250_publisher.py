import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import smbus2
import math
import time
import numpy as np
#from mpu9250_jmdev.registers import *
#from mpu9250_jmdev.mpu_9250 import MPU9250

class MPU9250:
    def __init__(self, bus_num=1, address=0x68, address_ak=0x0C):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.address_ak = address_ak  # Magnetometer I2C address

        # Power up the MPU9250
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)

        # Accelerometer configuration
        '''
        Accel Range(2G)     : 0x00
        Accel Range(4G)     : 0x08
        Accel Range(8G)     : 0x10
        Accel Range(16G)    : 0x18
        '''
        
        afs_sel = 0x08  # AFS_4G
        self.bus.write_byte_data(self.address, 0x1C, afs_sel)
        time.sleep(0.1)

        # Gyroscope configuration
        '''
        Gyro Range(250DPS)  : 0x00
        Gyro Range(500DPS)  : 0x08
        Gyro Range(1000DPS) : 0x10
        Gyro Range(2000DPS) : 0x18
        '''

        gfs_sel = 0x08  # GFS_500DPS
        self.bus.write_byte_data(self.address, 0x1B, gfs_sel)
        time.sleep(0.1)

        # Magnetometer configuration
        self.configure_magnetometer()

    def configure_magnetometer(self):
        # Power down magnetometer
        self.bus.write_byte_data(self.address_ak, 0x0A, 0x00)
        time.sleep(0.1)

        # Enter fuse ROM access mode
        self.bus.write_byte_data(self.address_ak, 0x0A, 0x0F)
        time.sleep(0.1)

        # Set magnetometer resolution
        mfs_sel = 0x10  # MFS_16BITS
        self.bus.write_byte_data(self.address_ak, 0x0A, mfs_sel)
        time.sleep(0.1)

        # Set continuous measurement mode
        self.bus.write_byte_data(self.address_ak, 0x0A, 0x16)
        time.sleep(0.1)

    def read_i2c_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def get_motion_data(self):
        # Scale value
        '''
        AFS_2G  : 16384.0
        AFS_4G  : 8192.0
        AFS_8G  : 4096.0
        AFS_16G : 2048.0

        GFS_250DPS  : 131.0
        GFS_500DPS  : 65.5
        GFS_1000DPS : 32.8
        GFS_2000DPS : 16.4
        '''

        accel_x = self.read_i2c_word(0x3B) / 8192.0  # AFS_4G
        accel_y = self.read_i2c_word(0x3D) / 8192.0
        accel_z = self.read_i2c_word(0x3F) / 8192.0

        gyro_x = self.read_i2c_word(0x43) / 65.5  # GFS_500DPS
        gyro_y = self.read_i2c_word(0x45) / 65.5
        gyro_z = self.read_i2c_word(0x47) / 65.5

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    def madgwick_filter_update(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.q
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        norm = 1 / norm
        ax *= norm
        ay *= norm
        az *= norm

        f1 = 2 * (q2 * q4 - q1 * q3) - ax
        f2 = 2 * (q1 * q2 + q3 * q4) - ay
        f3 = 1 - 2 * (q2 * q2 + q3 * q3) - az
        J_11or24 = 2 * q3
        J_12or23 = 2 * q4
        J_13or22 = 2 * q1
        J_14or21 = 2 * q2
        J_32 = 2 * J_14or21
        J_33 = 2 * J_11or24

        step = np.array([
            J_14or21 * f2 - J_11or24 * f1,
            J_12or23 * f1 + J_13or22 * f2 - J_32 * f3,
            J_12or23 * f2 - J_33 * f3 - J_14or21 * f1,
            J_13or22 * f1 + J_11or24 * f2
        ])
        step = step / np.linalg.norm(step)
        q_dot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
            q1 * gx + q3 * gz - q4 * gy,
            q1 * gy - q2 * gz + q4 * gx,
            q1 * gz + q2 * gy - q3 * gx
        ]) - self.beta * step

        self.q += q_dot * self.timer_period
        self.q = self.q / np.linalg.norm(self.q)

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.mpu = MPU9250()
        self.timer_period = 0.1  # 10Hz(origin: 0.1)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.last_time = self.get_clock().now()
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.beta = 0.1
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.get_motion_data()
        self.mpu.madgwick_filter_update(np.radians(gyro_x), np.radians(gyro_y), np.radians(gyro_z),
                                 accel_x, accel_y, accel_z)
        q = self.q

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        self.publisher_.publish(imu_msg)

        # 디버깅 로그 추가
        self.get_logger().info(f'Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f}')
        self.get_logger().info(f'Gyro: x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}')

        # 가속도계를 사용하여 회전 변환(roll, pitch) 계산
        #accel_roll = math.atan2(accel_y, accel_z)
        #accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    
        # 자이로스코프를 사용하여 각도 계산 (적분)
        #gyro_roll = self.roll + gyro_x * dt
        #gyro_pitch = self.pitch + gyro_y * dt

        # Complementary Filter 적용
        #self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        #self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        # Yaw 보정 (이 단순 보정은 실제 환경에 따라 조정이 필요할 수 있습니다)
        #self.yaw += gyro_z * dt
        #self.yaw = self.yaw % (2 * math.pi)  # -pi to pi 범위 유지

        # 쿼터니언 계산
        #cy = math.cos(self.yaw * 0.5)
        #sy = math.sin(self.yaw * 0.5)
        #cp = math.cos(self.pitch * 0.5)
        #sp = math.sin(self.pitch * 0.5)
        #cr = math.cos(self.roll * 0.5)
        #sr = math.sin(self.roll * 0.5)

        #qx = sr * cp * cy - cr * sp * sy
        #qy = cr * sp * cy + sr * cp * sy
        #qz = cr * cp * sy - sr * sp * cy
        #qw = cr * cp * cy + sr * sp * sy

        # Madgwick 필터 업데이트
        #self.madgwick_filter_update(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
        #q = self.q

        # 속도 및 위치 업데이트
        acceleration = np.array([accel_x, accel_y, accel_z])
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        scaling_factor = 0.1
        self.position *= scaling_factor

         # 디버깅 로그 추가
        #self.get_logger().info(f'Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}')
        #self.get_logger().info(f'Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}')
        self.get_logger().info(f'')    # 빈줄

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        
        #t.transform.translation.x = 0.0
        #t.transform.translation.y = 0.0
        #t.transform.translation.z = 0.0
        
        #t.transform.rotation.x = qx
        #t.transform.rotation.y = qy
        #t.transform.rotation.z = qz
        #t.transform.rotation.w = qw

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    mpu9250_publisher = MPU9250Publisher()
    rclpy.spin(mpu9250_publisher)
    mpu9250_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
