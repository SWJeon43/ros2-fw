import os
import sys
import time
import smbus

import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Vector3

from mpu9250.imusensor.MPU9250 import MPU9250
from mpu9250.imusensor.filters import madgwick

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_time = self.get_clock().now()

        self.sensorfusion = madgwick.Madgwick(0.5)  # Adjusted beta value

        address = 0x68
        bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(bus, address)
        self.imu.begin()

        # Additional sensor configuration
        self.imu.setAccelRange("AccelRangeSelect8G")
        self.imu.setGyroRange("GyroRangeSelect500DPS")
        self.imu.setSampleRate(100)  # 100Hz

        # Load calibration data if exists, otherwise perform calibration
        calib_file = "/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_data.json"
        if os.path.exists(calib_file):
            self.imu.loadCalibDataFromFile(calib_file)
        else:
            self.calibrate_sensors()
            self.imu.saveCalibDataToFile(calib_file)

    def calibrate_sensors(self):
        self.get_logger().info("Starting sensor calibration...")
        self.get_logger().info("Keep the sensor still for gyro calibration")
        self.imu.caliberateGyro()
        self.get_logger().info("Gyro calibration done")

        self.get_logger().info("Rotate the sensor in all directions for magnetometer calibration")
        time.sleep(3)
        self.imu.caliberateMagApprox()
        self.get_logger().info("Magnetometer calibration done")

        self.get_logger().info("Sensor calibration completed")

    def timer_callback(self):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            self.imu.readSensor()

            ax, ay, az = self.imu.AccelVals
            gx, gy, gz = self.imu.GyroVals
            mx, my, mz = self.imu.MagVals

            # Apply coordinate system transformation if needed
            # ax, ay, az = ay, ax, -az
            # gx, gy, gz = gy, gx, -gz
            # mx, my, mz = my, mx, -mz

            self.sensorfusion.updateRollPitchYaw(ax, ay, az, gx, gy, gz, mx, my, mz, dt)

            # Log raw and processed data
            self.get_logger().info(f'Raw Accel: x={ax:7.3f}, y={ay:7.3f}, z={az:7.3f}')
            self.get_logger().info(f'Raw Gyro: x={gx:7.3f}, y={gy:7.3f}, z={gz:7.3f}')
            self.get_logger().info(f'Raw Mag: x={mx:7.3f}, y={my:7.3f}, z={mz:7.3f}')
            self.get_logger().info(f'Orientation: roll={self.sensorfusion.roll:7.3f}, pitch={self.sensorfusion.pitch:7.3f}, yaw={self.sensorfusion.yaw:7.3f}')

            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
            imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
            imu_msg.orientation.x = self.sensorfusion.q[1]
            imu_msg.orientation.y = self.sensorfusion.q[2]
            imu_msg.orientation.z = self.sensorfusion.q[3]
            imu_msg.orientation.w = self.sensorfusion.q[0]

            self.publisher_.publish(imu_msg)

            # Broadcast transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'imu_link'
            t.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
            t.transform.rotation = imu_msg.orientation

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    mpu9250_node = MPU9250Publisher()
    rclpy.spin(mpu9250_node)
    mpu9250_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()