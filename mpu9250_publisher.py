import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import smbus2
import math
import numpy as np
#import time

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 1  # 10Hz(origin: 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = smbus2.SMBus(1)
        self.address = 0x68
        self.bus.write_byte_data(self.address, 0x6B, 0)
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_time = self.get_clock().now()
        self.yaw = 0.0

    def read_i2c_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def timer_callback(self):
        current_time = self.get_clock().now()
        #current_time2 = time.time()

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 시간값 디버그
        #self.get_logger().info(f'current_time: {current_time}')
        #self.get_logger().info(f'current_time2: {current_time2}')
        #self.get_logger().info(f'dt: {dt}')

        accel_x = self.read_i2c_word(0x3B) / 16384.0
        accel_y = self.read_i2c_word(0x3D) / 16384.0
        accel_z = self.read_i2c_word(0x3F) / 16384.0 - 1    # 중력 보정
        gyro_x = self.read_i2c_word(0x43) / 131.0
        gyro_y = self.read_i2c_word(0x45) / 131.0
        gyro_z = self.read_i2c_word(0x47) / 131.0

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

        # 회전 변환 계산
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        self.yaw += gyro_z * dt
        #self.yaw = 0

        # 쿼터니언 계산
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # 속도 및 위치 업데이트
        acceleration = np.array([accel_x, accel_y, accel_z])
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        #scaling_factor = 0.1
        #self.position *= scaling_factor

         # 디버깅 로그 추가
        self.get_logger().info(f'Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}')
        self.get_logger().info(f'Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}')
        self.get_logger().info()    # 빈줄

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
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    mpu9250_publisher = MPU9250Publisher()
    rclpy.spin(mpu9250_publisher)
    mpu9250_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
