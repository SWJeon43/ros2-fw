import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

imu.loadCalibDataFromFile("/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_real4.json")

while True:
    imu.readSensor()

    # roll, pitch, yaw 값 구하기
    imu.computeOrientation()

    print ("Accel2 x: {:7.3f}; Accel y: {:7.3f}; Accel z: {:7.3f}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
    print ("roll: {:7.3f}; pitch: {:7.3f}; yaw: {:7.3f}".format(imu.roll, imu.pitch, imu.yaw))
    time.sleep(0.1)
