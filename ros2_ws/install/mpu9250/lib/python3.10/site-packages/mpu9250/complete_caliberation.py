import smbus
import numpy as np
import sys
import os
import time

from imusensor.MPU9250 import MPU9250


address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

print ("Gyro calibration starting")
imu.caliberateGyro()
print ("Gyro calibration finished")

print ("Accel calibration starting")
imu.caliberateAccelerometer()
print ("Accel calibration finished")
print ("imu.AccelBias")
print (imu.AccelBias)
print ("imu.Accels")
print (imu.Accels)

print ("Mag calibration starting")
time.sleep(2)
imu.caliberateMagApprox()
#imu.caliberateMagPrecise()
print ("Mag calibration finished")
print ("imu.MagBias")
print (imu.MagBias)
print ("imu.Magtransform")
print (imu.Magtransform)
print ("imu.Mags")
print (imu.Mags)

accelscale = imu.Accels
accelBias = imu.AccelBias
gyroBias = imu.GyroBias
mags = imu.Mags
magBias = imu.MagBias

imu.saveCalibDataToFile("/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_real4.json")
print ("calib data saved")

imu.loadCalibDataFromFile("/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/calib_real4.json")
print ("calib data loaded")

if np.array_equal(accelscale, imu.Accels) & np.array_equal(accelBias, imu.AccelBias) & \
        np.array_equal(mags, imu.Mags) & np.array_equal(magBias, imu.MagBias) & \
        np.array_equal(gyroBias, imu.GyroBias):
            print ("calib loaded properly")
