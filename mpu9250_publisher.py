"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import smbus2
import math

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = smbus2.SMBus(1)
        self.address = 0x68
        self.bus.write_byte_data(self.address, 0x6B, 0)

    def read_i2c_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def timer_callback(self):
        accel_x = self.read_i2c_word(0x3B)
        accel_y = self.read_i2c_word(0x3D)
        accel_z = self.read_i2c_word(0x3F)
        gyro_x = self.read_i2c_word(0x43)
        gyro_y = self.read_i2c_word(0x45)
        gyro_z = self.read_i2c_word(0x47)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = accel_x / 16384.0
        imu_msg.linear_acceleration.y = accel_y / 16384.0
        imu_msg.linear_acceleration.z = accel_z / 16384.0

        imu_msg.angular_velocity.x = gyro_x / 131.0
        imu_msg.angular_velocity.y = gyro_y / 131.0
        imu_msg.angular_velocity.z = gyro_z / 131.0

        self.publisher_.publish(imu_msg)

        # IMU 데이터를 사용하여 회전 변환 계산
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))
        yaw = 0.0  # Yaw는 자이로스코프 데이터를 적분하여 계산해야 하지만 여기서는 생략합니다.

        # 쿼터니언 계산
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
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
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
from time import sleep

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68

        self.mpu_init()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_time = self.get_clock().now()

    def mpu_init(self):
        self.bus.write_byte_data(self.device_address, 0x6B, 0x00)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def timer_callback(self):
        msg = Imu()

        accel_x = self.read_raw_data(0x3B) / 16384.0
        accel_y = self.read_raw_data(0x3D) / 16384.0
        accel_z = self.read_raw_data(0x3F) / 16384.0
        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
