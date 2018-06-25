#Sourced from https://github.com/popedison/LSM9DS1

#import mraa as m
import numpy as np
from smbus import SMBus
from config import XM, MAG

""" Function to parse the data bytes
Inputs:
    b: byte array of 2-axis data, odd = high byte, even = low byte
Outputs:
    x, y, z: signed 16-bit integer of x, y, and z values
"""
def parsedata(b, cal):
    x = np.int16(b[0] | (b[1] << 8))*cal
    y = np.int16(b[2] | (b[3] << 8))*cal
    z = np.int16(b[4] | (b[5] << 8))*cal
    return x, y, z


""" Function to read the uncalibrated data from a 3-axis sensor
Inputs:
    x: I2C object from mraa
    address: address of sensor - accel/mag or gyro
    reg: register to read for specific data and config
Outputs:
    x, y, z: signed 16-bit integer of x, y, and z values
"""
def read3axis(x, address, reg, cal):
    x.address(address)
    data = x.readBytesReg(0x80 | reg, 6)
    x, y, z = parsedata(data, cal)
    return x, y, z 


# IMU Class 
class IMU:

    # Mag and Gyro Configs loaded
    XM = XM
    MA = MAG

    # Default mag, gyro, and accel ranges loaded
    selected_a_range = '2G'
    selected_m_range = '4GAUSS'
    selected_g_range = '245DPS'

    # Initialize I2C port for 9-axis IMU
    def __init__(self, I2CPort=1):
        self.bus = SMBus(I2CPort)
        #self.x = m.I2c(I2CPort)

    # Initialize - checking gyro and mag are properly connected
    def initialize(self):
        #self.x.address(self.MA.ADDRESS)
        resp = self.bus.read_i2c_block_data(self.MA.ADDRESS, self.MA.WHO_AM_I)
        #resp = self.x.readReg(self.MA.WHO_AM_I)
        if resp == self.MA.WHO_AM_I_OK:
            print "Magnetometer init success!"
        else:
            print "Magnetometer init failed"
        # Check accel/mag - expect back 0x49 = 73L if connected to 9dof breakout
        #self.x.address(self.XM.ADDRESS)
        resp = self.bus.read_i2c_block_data(self.XM.ADDRESS, self.XM.WHO_AM_I)
        #resp = self.x.readReg(self.XM.WHO_AM_I)
        if resp == self.XM.WHO_AM_I_OK:
            print "Accel/giro init success!"
        else:
            print "Accel/giro init failed"

    # Enables the accelerometer, 100 Hz continuous in X, Y, and Z
    def enable_accel(self):
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG5_XL, 0x38)
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG6_XL, 0xC0)
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG8, 0x04)
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG5_XL, 0x38)  # 3 axis enable 
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG6_XL, 0xC0) # 408 Hz, 952 ODR
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG8, 0x04) #multiple reads enable

    # Enables the gyro in normal mode on all axes
    def enable_gyro(self):
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG1_G, 0xC3)
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG4, 0x38)
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG8, 0x04)
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG1_G, 0xC3) # 100Hz 952 ODR
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG4, 0x38) # 3 axis enable
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG8, 0x04) #multiple reads enable

    # Enables the mag continuously on all axes
    def enable_mag(self):
        self.bus.write_i2c_block_data(self.MA.ADDRESS, self.XM.CTRL_REG1_M, 0x7C)
        self.bus.write_i2c_block_data(self.MA.ADDRESS, self.XM.CTRL_REG3_M, 0x00)
        #self.x.address(self.MA.ADDRESS)
        #self.x.writeReg(self.MA.CTRL_REG1_M, 0x7C)  
        #self.x.address(self.MA.ADDRESS)
        #self.x.writeReg(self.MA.CTRL_REG3_M, 0x00) # continous

    # Enables temperature measurement at the same frequency as mag  
    """def enable_temp(self):
        self.bus.read_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG5_XM)
        self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG5_XM, (rate | (1<<7)))
        #self.x.address(self.XM.ADDRESS)
        #rate = self.x.readReg(self.XM.CTRL_REG5_XM)  
        #self.x.address(self.XM.ADDRESS)
        #self.x.writeReg(self.XM.CTRL_REG5_XM, (rate | (1<<7)))"""  

    # Sets the range on the accelerometer, default is +/- 2Gs
    def accel_range(self,AR="2G"):
        try:
            Arange = self.XM.RANGE_A[AR]
            #self.x.address(self.XM.ADDRESS)
            #accelReg = self.x.readReg(self.XM.CTRL_REG6_XL)
            accelReg = self.bus.read_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG6_XL)
            accelReg |= Arange
            #self.x.address(self.XM.ADDRESS)
            #self.x.writeReg(self.XM.CTRL_REG6_XL, accelReg)
            self.bus.write_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG6_XL, accelReg)
            self.selected_a_range = AR
        except(KeyError):
            print("Invalid range. Valid range keys are '2G', '4G', '8G', '16G'")

    # Sets the range on the mag - default is +/- 2 Gauss
    def mag_range(self,MR="2GAUSS"):
        try:
            Mrange = self.MA.RANGE_M[MR]
            #self.x.address(self.MA.ADDRESS)
            #magReg = self.x.readReg(self.MA.CTRL_REG2_M)
            magReg = self.bus.read_i2c_block_data(self.MA.ADDRESS, self.MA.CTRL_REG2_M)
            magReg &= ~(0b01100000)
            magReg |= Mrange
            #self.x.address(self.MA.ADDRESS)
            #self.x.writeReg(self.MA.CTRL_REG2_M, magReg)
            self.bus.write_i2c_block_data(self.MA.ADDRESS, self.MA.CTRL_REG2_M, magReg)
            self.selected_m_range = MR
        except(KeyError):
            print("Invalid range. Valid range keys are '4GAUSS', '8GAUSS', or '12GAUSS' '16GAUSS'")

    # Sets the range on the gyro - default is +/- 245 degrees per second
    def gyro_range(self,GR="245DPS"):
        try:
            Grange = self.XM.RANGE_G[GR]
            #self.x.address(self.XM.ADDRESS)
            #gyroReg = self.x.readReg(self.XM.CTRL_REG1_G)
            gyroReg = self.bus.read_i2c_block_data(self.XM.ADDRESS, self.XM.CTRL_REG1_G)
            gyroReg &= ~(0b00011000)
            gyroReg |= Grange;
            #self.x.address(self.G.ADDRESS)
            #self.x.writeReg(self.G.CTRL_REG1_G, gyroReg)
            self.bus.write_i2c_block_data(self.G.ADDRESS, self.G.CTRL_REG1_G, gyroReg)
            self.selected_g_range = GR
        except(KeyError):
            print("Invalid range. Valid range keys are '245DPS', '500DPS', or '2000DPS'")

    # Reads and calibrates the accelerometer values into Gs
    def read_accel(self):
        cal = self.XM.CAL_A[self.selected_a_range]
        self.ax, self.ay, self.az = read3axis(self.x, self.XM.ADDRESS, self.XM.OUT_X_L_XL, cal)
    
    # Reads and calibrates the mag values into Gauss
    def read_mag(self):
        cal = self.MA.CAL_M[self.selected_m_range]
        self.mx, self.my, self.mz = read3axis(self.x, self.MA.ADDRESS, self.MA.OUT_X_L_M, cal) 
    
    # Reads and calibrates the gyro values into degrees per second
    def read_gyro(self):
        cal = self.XM.CAL_G[self.selected_g_range]
        self.gx, self.gy, self.gz = read3axis(self.x, self.XM.ADDRESS, self.XM.OUT_X_L_G, cal)  
    
    # Reads and calibrates the temperature in degrees C
    def readTemp(self):
        #self.x.address(self.XM.ADDRESS)
        #tempdata = self.x.readBytesReg(0x80 | self.XM.OUT_TEMP_L, 2)
        gyroReg = self.bus.read_i2c_block_data(self.XM.ADDRESS, 0x80 | self.XM.OUT_TEMP_L)
        temp = np.int16(((tempdata[1] >> 4) << 8) | tempdata[0])
        self.temp = temp * self.XM.CAL_TEMP

