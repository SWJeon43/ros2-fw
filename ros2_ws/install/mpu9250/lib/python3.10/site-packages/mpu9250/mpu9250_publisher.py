import os
import sys
import time
import smbus

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.mpu = MPU9250()
        timer_period = 0.01  # 100Hz(origin: 0.1/10Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_time = self.get_clock().now()
        #self.madgwick = MadgwickAHRS(sample_period=timer_period)

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

    def timer_callback(self):
        # 오프셋 지속 보정 코드
       # if self.mpu.sample_count % self.mpu.offset_update_period == 0:
       #     self.mpu.update_offsets()
       # self.mpu.sample_count += 1

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 가속도, 각속도 데이터 업데이트
        #accel_x, accel_y, accel_z, \
        #    gyro_x, gyro_y, gyro_z = self.mpu.get_motion_data()

        # madgwick 필터 적용
        #self.madgwick.update_imu(np.radians(gyro_x), np.radians(gyro_y), np.radians(gyro_z),
        #                         accel_x, accel_y, accel_z)
        #q = self.madgwick.quaternion

        # 쿼터니언 적용 및 중력 보정
        #gravity = np.array([0, 0, 1.0])
        #rotation_matrix = self.quaternion_to_rotation_matrix(q)
        #corrected_acceleration = \
        #    rotation_matrix.dot(np.array \
        #                        ([accel_x, accel_y, accel_z]) - gravity)

        # 위치정보 업데이트
        #self.velocity += corrected_acceleration * dt
        #self.velocity += [accel_x, accel_y, accel_z]
        #self.position = self.velocity * dt

        # 디버깅 로그 추가
        #self.get_logger().info(f'Accel:    x={accel_x:7.3f}, y={accel_y:7.3f}, z={accel_z:7.3f}')
        #self.get_logger().info(f'Gyro:     x={gyro_x:7.3f}, y={gyro_y:7.3f}, z={gyro_z:7.3f}')

        # 위치 값의 스케일링 확인
        # 너무 큰 값이 나오지 않도록 적절한 스케일링 적용 필요
        #scaling_factor = 0.1
        #self.position *= scaling_factor

         # 디버깅 로그 추가
        #self.get_logger().info(f'Position: x={self.position[0]:7.3f}, y={self.position[1]:7.3f}, z={self.position[2]:7.3f}')
        #self.get_logger().info(f'Velocity: x={self.velocity[0]:7.3f}, y={self.velocity[1]:7.3f}, z={self.velocity[2]:7.3f}')
        #self.get_logger().info(f'')    # 빈줄
        #os.system("clear")

        ###
        sensorfusion = madgwick.Madgwick(0.5)

        address = 0x68
        bus = smbus.SMBus(1)
        imu = MPU9250.MPU9250(bus, address)
        imu.begin()

        imu.readSensor()
        sensorfusion.updateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], \
            imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
        
        ###

        # imu 퍼블리셔에 데이터 셋팅
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = imu.AccelVals[0]
        imu_msg.linear_acceleration.y = imu.AccelVals[1]
        imu_msg.linear_acceleration.z = imu.AccelVals[2]

        imu_msg.angular_velocity.x = imu.GyroVals[0]
        imu_msg.angular_velocity.y = imu.GyroVals[1]
        imu_msg.angular_velocity.z = imu.GyroVals[2]

        imu_msg.orientation.x = imu.q[1]
        imu_msg.orientation.y = imu.q[2]
        imu_msg.orientation.z = imu.q[3]
        imu_msg.orientation.w = imu.q[0]

        self.publisher_.publish(imu_msg)

        # tf 브로드캐스터에 데이터 셋팅
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'
        
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        t.transform.rotation.x = imu.q[1]
        t.transform.rotation.y = imu.q[2]
        t.transform.rotation.z = imu.q[3]
        t.transform.rotation.w = imu.q[0]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    mpu9250_node = MPU9250Publisher()
    rclpy.spin(mpu9250_node)
    mpu9250_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
