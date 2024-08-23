import os
import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)

def reset_mpu9250(imu):
    PWR_MGMT_1 = 0x6B
    #imu.__writeRegister(PWR_MGMT_1, 0x80)
    bus.write_byte_data(address, PWR_MGMT_1, 0x80)
    time.sleep(0.1)

    #imu.__writeRegister(PWR_MGMT_1, 0x01)
    bus.write_byte_data(address, PWR_MGMT_1, 0x01)
    time.sleep(0.1)

def initialize_mpu9250(imu):
    reset_mpu9250(imu)
    imu.begin()


def normalize_angle_mod(angle):
    angle = angle % 360

    if angle > 180:
        angle -= 360
    return angle

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

initialize_mpu9250(imu)

while True:
    imu.readSensor()
    imu.computeOrientation()

    #roll = normalize_angle_mod(imu.roll)
    #pitch = normalize_angle_mod(imu.pitch)
    #yaw = normalize_angle_mod(imu.yaw)

    roll = normalize_angle(imu.roll)
    pitch = normalize_angle(imu.pitch)
    yaw = normalize_angle(imu.yaw)

    #print("Accel x: {0} ; Accel y: {1} ; Accel z: {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
    #print("Gyro x: {0} ; Gyro y: {1} ; Gyro z: {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
    #print("Mag x: {0} ; Mag y: {1} ; Mag z: {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
    print("roll: {0} ; pitch: {1} ; y: {2}".format(roll, pitch, yaw))

    #print(f'gx={imu.GyroVals[0]:7.3f}, gy={imu.GyroVals[1]:7.3f}, gz={imu.GyroVals[2]:7.3f}')
    time.sleep(0.1)
