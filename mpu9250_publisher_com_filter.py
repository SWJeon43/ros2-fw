import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
#import smbus2
#import math
import numpy as np
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # 10Hz(origin: 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.bus = smbus2.SMBus(1)
        #self.address = 0x68
        #self.bus.write_byte_data(self.address, 0x6B, 0)

        # MPU9250 초기화
        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_250,   # 자이로 풀 스케일 설정: ±250dps
            afs=AFS_2G,    # 가속도 풀 스케일 설정: ±2g
            mfs=AK8963_BIT_16,  # 자력계 풀 스케일 설정: 16-bit
            mode=AK8963_MODE_C100HZ
        )

        self.mpu.calibrate()  # 보정
        self.mpu.configure()  # 설정
        self.madgwick = MadgwickAHRS(sampleperiod=timer_period)

        self.last_time = self.get_clock().now()
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

        # 센서 회전 데이터 초기화
        #self.alpha = 0.98   # Complementary filter(상보필터 상수)
        #self.roll = 0.0
        #self.pitch = 0.0
        #self.yaw = 0.0

        # 속도, 위치 데이터 초기화
        #self.position = np.array([0.0, 0.0, 0.0])
        #self.velocity = np.array([0.0, 0.0, 0.0])

    #def read_i2c_word(self, reg):
    #    high = self.bus.read_byte_data(self.address, reg)
    #    low = self.bus.read_byte_data(self.address, reg + 1)
    #    value = (high << 8) + low
    #    if value >= 0x8000:
    #        value = -((65535 - value) + 1)
    #    return value

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

#########################################################

        # MPU9250 데이터 읽기
        self.mpu.read_all()
        accel = self.mpu.accel
        gyro = self.mpu.gyro

        accel_x, accel_y, accel_z = accel[0], accel[1], accel[2] - 1.0
        gyro_x, gyro_y, gyro_z = np.deg2rad(gyro[0]), np.deg2rad(gyro[1]), np.deg2rad(gyro[2])  # 각속도 단위를 rad/s로 변환

#########################################################

        # 센서 raw 데이터 추출
        #accel_x = self.read_i2c_word(0x3B) / 16384.0
        #accel_y = self.read_i2c_word(0x3D) / 16384.0
        #accel_z = self.read_i2c_word(0x3F) / 16384.0 - 1    # 중력 보정

        #gyro_x = self.read_i2c_word(0x43) / 131.0
        #gyro_y = self.read_i2c_word(0x45) / 131.0
        #gyro_z = self.read_i2c_word(0x47) / 131.0

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

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
        self.madgwick.update_imu(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
        q = self.madgwick.q

        # 속도 및 위치 업데이트
        acceleration = np.array([accel_x, accel_y, accel_z])
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        #scaling_factor = 0.1
        #self.position *= scaling_factor

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
