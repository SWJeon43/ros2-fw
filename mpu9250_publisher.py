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

class MadgwickAHRS:
    def __init__(self, sample_period=1/256, beta=0.1):
        self.sample_period = sample_period
        self.beta = beta
        self.q = np.array([1, 0, 0, 0], dtype=np.float64)  # initial quaternion

    def update_imu(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.q
        
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        norm = 1 / norm
        ax *= norm
        ay *= norm
        az *= norm

        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay

        norm = 1 / np.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        q1 += q_dot1 * self.sample_period
        q2 += q_dot2 * self.sample_period
        q3 += q_dot3 * self.sample_period
        q4 += q_dot4 * self.sample_period

        norm = 1 / np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = np.array([q1 * norm, q2 * norm, q3 * norm, q4 * norm])

    @property
    def quaternion(self):
        return self.q

class MPU9250:
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.address_ak = 0x0C  # Magnetometer I2C address

        #for addr in range(0x03, 0x78):
        #    try:
        #        self.bus.write_quick(addr)
        #        print(f"Found device at address: 0x{addr:02X}")
        #    except OSError:
        #        pass

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
        
        afs_sel = 0x00  # AFS_4G
        self.bus.write_byte_data(self.address, 0x1C, afs_sel)
        time.sleep(0.1)

        # Gyroscope configuration
        '''
        Gyro Range(250DPS)  : 0x00
        Gyro Range(500DPS)  : 0x08
        Gyro Range(1000DPS) : 0x10
        Gyro Range(2000DPS) : 0x18
        '''

        gfs_sel = 0x00  # GFS_500DPS
        self.bus.write_byte_data(self.address, 0x1B, gfs_sel)
        time.sleep(0.1)

        # Magnetometer configuration
        #self.configure_magnetometer()

    def configure_magnetometer(self):
        try:
            # Power down magnetometer
            #print("address: %x, address_ak: %x" % (self.address, self.address_ak))
            self.bus.write_byte_data(self.address_ak, 0x0A, 0x00)
            time.sleep(0.1)

            # Enter fuse ROM access mode
            self.bus.write_byte_data(self.address_ak, 0x0A, 0x0F)
            time.sleep(0.1)

            # Read sensitivity adjustment values from fuse ROM
            asax = self.bus.read_byte_data(self.address_ak, 0x10)
            asay = self.bus.read_byte_data(self.address_ak, 0x11)
            asaz = self.bus.read_byte_data(self.address_ak, 0x12)

            # Calculate sensitivity adjustment values
            self.mag_sensitivity = np.array([(asax - 128) / 256.0 + 1.0,
                                             (asay - 128) / 256.0 + 1.0,
                                             (asaz - 128) / 256.0 + 1.0])

            # Set magnetometer resolution
            mfs_sel = 0x10  # MFS_16BITS
            self.bus.write_byte_data(self.address_ak, 0x0A, mfs_sel)
            time.sleep(0.1)

            # Set continuous measurement mode
            self.bus.write_byte_data(self.address_ak, 0x0A, 0x16)
            time.sleep(0.1)

        except OSError as e:
            print(f"Failed to configure magnetometer: {e}")
            # Optional: Re-raise the exception or handle it as needed
            raise e

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

        accel_x = self.read_i2c_word(0x3B) / 16384.0  # AFS_4G
        accel_y = self.read_i2c_word(0x3D) / 16384.0
        accel_z = self.read_i2c_word(0x3F) / 16384.0 - 1

        gyro_x = self.read_i2c_word(0x43) / 131.0  # GFS_500DPS
        gyro_y = self.read_i2c_word(0x45) / 131.0
        gyro_z = self.read_i2c_word(0x47) / 131.0

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.madgwick = MadgwickAHRS(sample_period=0.1)
        self.mpu = MPU9250()

        timer_period = 0.1  # 10Hz(origin: 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_time = self.get_clock().now()

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 가속도, 각속도 데이터 업데이트
        accel_x, accel_y, accel_z, \
            gyro_x, gyro_y, gyro_z = self.mpu.get_motion_data()

        # madgwick 필터 적용
        self.madgwick.update_imu(np.radians(gyro_x), np.radians(gyro_y), np.radians(gyro_z),
                                 accel_x, accel_y, accel_z)
        q = self.madgwick.quaternion

        # 쿼터니언 적용 및 중력 보정
        gravity = np.array([0, 0, 1.0])
        rotation_matrix = self.quaternion_to_rotation_matrix(q)
        corrected_acceleration = \
            rotation_matrix.dot(np.array \
                                ([accel_x, accel_y, accel_z]) - gravity)

        # 위치정보 업데이트
        self.velocity += corrected_acceleration * dt
        self.position += self.velocity * dt

        # 디버깅 로그 추가
        self.get_logger().info(f'Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f}')
        self.get_logger().info(f'Gyro: x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}')

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        scaling_factor = 0.1
        self.position *= scaling_factor

         # 디버깅 로그 추가
        #self.get_logger().info(f'Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}')
        #self.get_logger().info(f'Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}')
        self.get_logger().info(f'')    # 빈줄

        # imu 퍼블리셔에 데이터 셋팅
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]
        imu_msg.orientation.w = q[0]

        self.publisher_.publish(imu_msg)

        # tf 브로드캐스터에 데이터 셋팅
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
def main(args=None):
    rclpy.init(args=args)
    mpu9250_publisher = MPU9250Publisher()
    rclpy.spin(mpu9250_publisher)
    mpu9250_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
