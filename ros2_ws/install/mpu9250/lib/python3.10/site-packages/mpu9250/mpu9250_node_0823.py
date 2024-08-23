import os
import sys
import time
import smbus

import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

from mpu9250.imusensor.MPU9250 import MPU9250
from mpu9250.imusensor.filters import madgwick

from scipy.integrate import cumtrapz

class MPU9250Publisher(Node):
    def __init__(self):
        super().__init__('mpu9250_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.01  # 100Hz(origin: 0.1/10Hz)
        #timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_time = self.get_clock().now()

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

        self.sensorfusion = madgwick.Madgwick(0.1)

        address = 0x68
        bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(bus, address)
        self.imu.begin()

        # calibrations 적용
        print("Gyro calibration start...")
        self.imu.caliberateGyro()
        print("Accel calibration start...")
        self.imu.caliberateAccelerometer()
        print("Mag calibration ready...")
        time.sleep(10)
        print("Mag calibration start...")
        #self.imu.caliberateMagPrecise()
        self.imu.caliberateMagApprox()
        
        self.imu.saveCalibDataToFile("/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_real4.json")
        #self.imu.loadCalibDataFromFile("/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_real4.json")

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        imu = self.imu
        sensorfusion = self.sensorfusion

        imu.readSensor()

        ax = imu.AccelVals[0]
        ay = imu.AccelVals[1]
        az = imu.AccelVals[2]

        gx = imu.GyroVals[0]
        gy = imu.GyroVals[1]
        gz = imu.GyroVals[2]

        #ax = imu.AccelVals[1]
        #ay = imu.AccelVals[0]
        #az = -imu.AccelVals[2]

        #gx = imu.GyroVals[1]
        #gy = imu.GyroVals[0]
        #gz = -imu.GyroVals[2]

        mx = imu.MagVals[0]
        my = imu.MagVals[1]
        mz = imu.MagVals[2]

        #if az < -9.0:
        #    az = -az
        #az -= 9.81

        sensorfusion.updateRollPitchYaw(ax, ay, az, gx, gy, gz, mx, my, mz, dt)
        #imu.computeOrientation()
        
        #self.get_logger().info(f'Accel: x={ax:7.3f}, y={ay:7.3f}, z={az:7.3f}')
        #self.get_logger().info(f'Gyro: x={gx:7.3f}, y={gy:7.3f}, z={gz:7.3f}')
        #self.get_logger().info(f'Mag: x={mx:7.3f}, y={my:7.3f}, z={mz:7.3f}')
        self.get_logger().info(f'roll={sensorfusion.roll:7.3f}, pitch={sensorfusion.pitch:7.3f}, yaw={sensorfusion.yaw:7.3f}')

##########
        # Correct the acceleration by removing gravity
        #acc = np.array([ax, ay, az])
        #acc -= np.array([0, 0, 9.81])

        # Convert acceleration to the world frame
        #q = sensorfusion.q
        #R = self.quaternion_to_rotation_matrix(q)
        #acc_world = R.dot(acc)

        # Integrate acceleration to get velocity
        #self.velocity += acc_world * dt

        # Integrate velocity to get position
        #self.position += self.velocity * dt

        # Optionally apply a simple drift correction or limit integration duration
        ##self.velocity *= 0.99

        # 스케일링
        #self.position *= 0.1
        ###
############

        a_local = np.array([ax, ay, az])
        q = sensorfusion.q

        R = self.quaternion_to_rotation_matrix(q)
        a_world = np.dot(R, a_local)

        g = np.array([0, 0, 9.81])
        a_linear = a_world - g

        self.velocity = cumtrapz(a_linear, dx=dt, initial=0)
        self.position = cumtrapz(self.velocity, dx=dt, initial=0)

        #print("position : ", self.position)

        # imu 퍼블리셔에 데이터 셋팅
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.orientation.x = sensorfusion.q[1]
        imu_msg.orientation.y = sensorfusion.q[2]
        imu_msg.orientation.z = sensorfusion.q[3]
        imu_msg.orientation.w = sensorfusion.q[0]

        self.publisher_.publish(imu_msg)

        # tf 브로드캐스터에 데이터 셋팅
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_link'
        
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        t.transform.rotation.x = sensorfusion.q[1]
        t.transform.rotation.y = sensorfusion.q[2]
        t.transform.rotation.z = sensorfusion.q[3]
        t.transform.rotation.w = sensorfusion.q[0]

        self.tf_broadcaster.sendTransform(t)

    def quaternion_to_rotation_matrix(self, q):
        """
        Converts a quaternion into a rotation matrix.
        """
        
        w, x, y, z = q
        return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])

def main(args=None):
    rclpy.init(args=args)
    mpu9250_node = MPU9250Publisher()
    rclpy.spin(mpu9250_node)
    mpu9250_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
