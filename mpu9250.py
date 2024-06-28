import smbus2
import time
import numpy as np

class MPU9250:
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.address_ak = 0x0C  # Magnetometer I2C address

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
        
        afs_sel = 0x08  # AFS_4G
        self.bus.write_byte_data(self.address, 0x1C, afs_sel)
        time.sleep(0.1)

        # Gyroscope configuration
        '''
        Gyro Range(250DPS)  : 0x00
        Gyro Range(500DPS)  : 0x08
        Gyro Range(1000DPS) : 0x10
        Gyro Range(2000DPS) : 0x18
        '''

        gfs_sel = 0x08  # GFS_500DPS
        self.bus.write_byte_data(self.address, 0x1B, gfs_sel)
        time.sleep(0.1)

        # Magnetometer configuration
        self.configure_magnetometer()

    def configure_magnetometer(self):
        try:
            # Power down magnetometer
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

        accel_x = self.read_i2c_word(0x3B) / 8192.0  # AFS_4G
        accel_y = self.read_i2c_word(0x3D) / 8192.0
        accel_z = self.read_i2c_word(0x3F) / 8192.0

        gyro_x = self.read_i2c_word(0x43) / 65.5  # GFS_500DPS
        gyro_y = self.read_i2c_word(0x45) / 65.5
        gyro_z = self.read_i2c_word(0x47) / 65.5

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    '''
    def madgwick_filter_update(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.q
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        norm = 1 / norm
        ax *= norm
        ay *= norm
        az *= norm

        f1 = 2 * (q2 * q4 - q1 * q3) - ax
        f2 = 2 * (q1 * q2 + q3 * q4) - ay
        f3 = 1 - 2 * (q2 * q2 + q3 * q3) - az
        J_11or24 = 2 * q3
        J_12or23 = 2 * q4
        J_13or22 = 2 * q1
        J_14or21 = 2 * q2
        J_32 = 2 * J_14or21
        J_33 = 2 * J_11or24

        step = np.array([
            J_14or21 * f2 - J_11or24 * f1,
            J_12or23 * f1 + J_13or22 * f2 - J_32 * f3,
            J_12or23 * f2 - J_33 * f3 - J_14or21 * f1,
            J_13or22 * f1 + J_11or24 * f2
        ])
        step = step / np.linalg.norm(step)
        q_dot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
            q1 * gx + q3 * gz - q4 * gy,
            q1 * gy - q2 * gz + q4 * gx,
            q1 * gz + q2 * gy - q3 * gx
        ]) - self.beta * step

        self.q += q_dot * self.timer_period
        self.q = self.q / np.linalg.norm(self.q)
    '''