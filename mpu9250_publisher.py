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
import MPU9250
import MadgwickAHRS

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.madgwick = MadgwickAHRS(sampleperiod=0.1)
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
            gyro_x, gyro_y, gyro_z = self.get_motion_data()

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

        # 디버깅 로그 추가
        self.get_logger().info(f'Accel: x={accel_x:.3f}, y={accel_y:.3f}, z={accel_z:.3f}')
        self.get_logger().info(f'Gyro: x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}')

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        #scaling_factor = 0.1
        #self.position *= scaling_factor

         # 디버깅 로그 추가
        #self.get_logger().info(f'Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}')
        #self.get_logger().info(f'Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}')
        self.get_logger().info(f'')    # 빈줄

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
