# Adafruit BNO080 Absolute Orientation Sensor Library
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import binascii
import logging
import struct
import time
import Adafruit_GPIO.I2C as I2C


# I2C addresses
BNO080_ADDRESS_A                     = 0x4A
BNO080_ADDRESS_B                     = 0x4B
BNO080_ID                            = 0xF8

# Page id register definition
BNO080_PAGE_ID_ADDR                  = 0X07

# PAGE0 REGISTER DEFINITION START
BNO080_CHIP_ID_ADDR                  = 0xF9
BNO080_ACCEL_REV_ID_ADDR             = 0x01
BNO080_MAG_REV_ID_ADDR               = 0x02
BNO080_GYRO_REV_ID_ADDR              = 0x03
BNO080_SW_REV_ID_LSB_ADDR            = 0x04
BNO080_SW_REV_ID_MSB_ADDR            = 0x05
BNO080_BL_REV_ID_ADDR                = 0X06

# Accel data register
BNO080_ACCEL_DATA_X_LSB_ADDR         = 0X08
BNO080_ACCEL_DATA_X_MSB_ADDR         = 0X09
BNO080_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
BNO080_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
BNO080_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
BNO080_ACCEL_DATA_Z_MSB_ADDR         = 0X0D

# Mag data register
BNO080_MAG_DATA_X_LSB_ADDR           = 0X0E
BNO080_MAG_DATA_X_MSB_ADDR           = 0X0F
BNO080_MAG_DATA_Y_LSB_ADDR           = 0X10
BNO080_MAG_DATA_Y_MSB_ADDR           = 0X11
BNO080_MAG_DATA_Z_LSB_ADDR           = 0X12
BNO080_MAG_DATA_Z_MSB_ADDR           = 0X13

# Gyro data registers
BNO080_GYRO_DATA_X_LSB_ADDR          = 0X14
BNO080_GYRO_DATA_X_MSB_ADDR          = 0X15
BNO080_GYRO_DATA_Y_LSB_ADDR          = 0X16
BNO080_GYRO_DATA_Y_MSB_ADDR          = 0X17
BNO080_GYRO_DATA_Z_LSB_ADDR          = 0X18
BNO080_GYRO_DATA_Z_MSB_ADDR          = 0X19

# Euler data registers
BNO080_EULER_H_LSB_ADDR              = 0X1A  # yaw   = x = heading
BNO080_EULER_H_MSB_ADDR              = 0X1B
BNO080_EULER_R_LSB_ADDR              = 0X1C  # roll  = y
BNO080_EULER_R_MSB_ADDR              = 0X1D
BNO080_EULER_P_LSB_ADDR              = 0X1E  # pitch = z
BNO080_EULER_P_MSB_ADDR              = 0X1F

# Quaternion data registers
BNO080_QUATERNION_DATA_W_LSB_ADDR    = 0X20
BNO080_QUATERNION_DATA_W_MSB_ADDR    = 0X21
BNO080_QUATERNION_DATA_X_LSB_ADDR    = 0X22
BNO080_QUATERNION_DATA_X_MSB_ADDR    = 0X23
BNO080_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
BNO080_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
BNO080_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
BNO080_QUATERNION_DATA_Z_MSB_ADDR    = 0X27

# Linear acceleration data registers
BNO080_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
BNO080_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
BNO080_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
BNO080_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
BNO080_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
BNO080_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D

# Gravity data registers
BNO080_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
BNO080_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
BNO080_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
BNO080_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
BNO080_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
BNO080_GRAVITY_DATA_Z_MSB_ADDR       = 0X33

# Temperature data register
BNO080_TEMP_ADDR                     = 0X34

# Status registers
BNO080_CALIB_STAT_ADDR               = 0X35
BNO080_SELFTEST_RESULT_ADDR          = 0X36
BNO080_INTR_STAT_ADDR                = 0X37

BNO080_SYS_CLK_STAT_ADDR             = 0X38
BNO080_SYS_STAT_ADDR                 = 0X39
BNO080_SYS_ERR_ADDR                  = 0X3A

# Unit selection register
BNO080_UNIT_SEL_ADDR                 = 0X3B
BNO080_DATA_SELECT_ADDR              = 0X3C

# Mode registers
BNO080_OPR_MODE_ADDR                 = 0X3D
BNO080_PWR_MODE_ADDR                 = 0X3E

BNO080_SYS_TRIGGER_ADDR              = 0X3F
BNO080_TEMP_SOURCE_ADDR              = 0X40

# Axis remap registers
BNO080_AXIS_MAP_CONFIG_ADDR          = 0X41
BNO080_AXIS_MAP_SIGN_ADDR            = 0X42

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

# SIC registers
BNO080_SIC_MATRIX_0_LSB_ADDR         = 0X43
BNO080_SIC_MATRIX_0_MSB_ADDR         = 0X44
BNO080_SIC_MATRIX_1_LSB_ADDR         = 0X45
BNO080_SIC_MATRIX_1_MSB_ADDR         = 0X46
BNO080_SIC_MATRIX_2_LSB_ADDR         = 0X47
BNO080_SIC_MATRIX_2_MSB_ADDR         = 0X48
BNO080_SIC_MATRIX_3_LSB_ADDR         = 0X49
BNO080_SIC_MATRIX_3_MSB_ADDR         = 0X4A
BNO080_SIC_MATRIX_4_LSB_ADDR         = 0X4B
BNO080_SIC_MATRIX_4_MSB_ADDR         = 0X4C
BNO080_SIC_MATRIX_5_LSB_ADDR         = 0X4D
BNO080_SIC_MATRIX_5_MSB_ADDR         = 0X4E
BNO080_SIC_MATRIX_6_LSB_ADDR         = 0X4F
BNO080_SIC_MATRIX_6_MSB_ADDR         = 0X50
BNO080_SIC_MATRIX_7_LSB_ADDR         = 0X51
BNO080_SIC_MATRIX_7_MSB_ADDR         = 0X52
BNO080_SIC_MATRIX_8_LSB_ADDR         = 0X53
BNO080_SIC_MATRIX_8_MSB_ADDR         = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR              = 0X55
ACCEL_OFFSET_X_MSB_ADDR              = 0X56
ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR                = 0X5B
MAG_OFFSET_X_MSB_ADDR                = 0X5C
MAG_OFFSET_Y_LSB_ADDR                = 0X5D
MAG_OFFSET_Y_MSB_ADDR                = 0X5E
MAG_OFFSET_Z_LSB_ADDR                = 0X5F
MAG_OFFSET_Z_MSB_ADDR                = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR               = 0X61
GYRO_OFFSET_X_MSB_ADDR               = 0X62
GYRO_OFFSET_Y_LSB_ADDR               = 0X63
GYRO_OFFSET_Y_MSB_ADDR               = 0X64
GYRO_OFFSET_Z_LSB_ADDR               = 0X65
GYRO_OFFSET_Z_MSB_ADDR               = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR                = 0X67
ACCEL_RADIUS_MSB_ADDR                = 0X68
MAG_RADIUS_LSB_ADDR                  = 0X69
MAG_RADIUS_MSB_ADDR                  = 0X6A

# Power modes
POWER_MODE_NORMAL                    = 0X00
POWER_MODE_LOWPOWER                  = 0X01
POWER_MODE_SUSPEND                   = 0X02

# Operation mode settings
OPERATION_MODE_CONFIG                = 0X00
OPERATION_MODE_ACCONLY               = 0X01
OPERATION_MODE_MAGONLY               = 0X02
OPERATION_MODE_GYRONLY               = 0X03
OPERATION_MODE_ACCMAG                = 0X04
OPERATION_MODE_ACCGYRO               = 0X05
OPERATION_MODE_MAGGYRO               = 0X06
OPERATION_MODE_AMG                   = 0X07
OPERATION_MODE_IMUPLUS               = 0X08
OPERATION_MODE_COMPASS               = 0X09
OPERATION_MODE_M4G                   = 0X0A
OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
OPERATION_MODE_NDOF                  = 0X0C


# SHTP = Sensor Hub Transport Protocol

# All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
# These are used for low level communication with the sensor, on channel 2

CHANNEL_COMMAND = 0
CHANNEL_EXECUTABLE = 1
CHANNEL_CONTROL = 2
CHANNEL_REPORTS = 3
CHANNEL_WAKE_REPORTS = 4
CHANNEL_GYRO = 5

SHTP_REPORT_COMMAND_RESPONSE = 0xF1
SHTP_REPORT_COMMAND_REQUEST = 0xF2
SHTP_REPORT_FRS_READ_RESPONSE = 0xF3
SHTP_REPORT_FRS_READ_REQUEST = 0xF4
SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8
SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9
SHTP_REPORT_BASE_TIMESTAMP = 0xFB
SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD

# All the different sensors and features we can get reports from
# These are used when enabling a given sensor
SENSOR_REPORTID_ACCELEROMETER = 0x01
SENSOR_REPORTID_GYROSCOPE = 0x02
SENSOR_REPORTID_MAGNETIC_FIELD = 0x03
SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04
SENSOR_REPORTID_ROTATION_VECTOR = 0x05
SENSOR_REPORTID_GRAVITY = 0x06
SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08
SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = 0x09
SENSOR_REPORTID_TAP_DETECTOR = 0x10
SENSOR_REPORTID_STEP_COUNTER = 0x11
SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x13
SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E

# Record IDs from figure 29, page 29 reference manual
# These are used to read the metadata for each sensor type
FRS_RECORDID_ACCELEROMETER = 0xE302
FRS_RECORDID_GYROSCOPE_CALIBRATED = 0xE306
FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED = 0xE309
FRS_RECORDID_ROTATION_VECTOR = 0xE30B

# Command IDs from section 6.4, page 42
# These are used to calibrate, initialize, set orientation, tare etc the sensor
COMMAND_ERRORS = 1
COMMAND_COUNTER = 2
COMMAND_TARE = 3
COMMAND_INITIALIZE = 4
COMMAND_DCD = 6
COMMAND_ME_CALIBRATE = 7
COMMAND_DCD_PERIOD_SAVE = 9
COMMAND_OSCILLATOR = 10
COMMAND_CLEAR_DCD = 11

CALIBRATE_ACCEL = 0
CALIBRATE_GYRO = 1
CALIBRATE_MAG = 2
CALIBRATE_PLANAR_ACCEL = 3
CALIBRATE_ACCEL_GYRO_MAG = 4
CALIBRATE_STOP = 5


class BNO080(object):
    receivedData = []

    def __init__(self, address=BNO080_ADDRESS_A, gpio=None, **kwargs):
        self._rst = None
        self._serial = None
        self._i2c_device = None

        i2c = I2C
        # Save a reference to the I2C device instance for later communication.
        self._i2c_device = i2c.get_i2c_device(address, **kwargs)

        self.sequenceNumber = []
        self.sequenceNumber[0] = 0
        self.sequenceNumber[1] = 0
        self.sequenceNumber[2] = 0
        self.sequenceNumber[3] = 0
        self.sequenceNumber[4] = 0
        self.sequenceNumber[5] = 0

    def _send_shtp_command(self, channelNumber, dataLength, data):
        packetLength = dataLength + 4
        self.sequenceNumber[channelNumber] = self.sequenceNumber[channelNumber] + 1
        self._i2c_device.writeRaw8(packetLength & 0xFF)  # Packet length LSB
        self._i2c_device.writeRaw8(packetLength >> 8)    # Packet length MSB
        self._i2c_device.writeRaw8(channelNumber)
        self._i2c_device.writeRaw8(self.sequenceNumber[channelNumber])  # Send the sequence number, increments with each packet sent, different counter for each channel
        i = 0
        while i < dataLength:
            self._i2c_device.writeRaw8(data[i])

    def _read_data(self, numberOfBytesToRead):
        data = []
        i = 0
        while i < numberOfBytesToRead + 4:
            data[i] = self._i2c_device.readRaw8()
        return data

    def _receive_packet(self):
        data = self._i2c_device.readList(BNO080_ADDRESS_A, 4)
        # Store the header info.
        shtpHeader = []
        shtpHeader[0] = data[0]
        shtpHeader[1] = data[1]
        shtpHeader[2] = data[2]
        shtpHeader[3] = data[3]

        # Calculate the number of data bytes in this packet
        dataLength = shtpHeader[1] << 8 | shtpHeader[0]
        dataLength &= ~(1 << 15)  # Clear the MSbit.

        if dataLength == 0:
            return False
        else:
            dataLength = dataLength - 4
            self.receivedData = self._read_data(dataLength)
            return True

    def _set_feature_command(self, reportID, specificConfig):

        timeBetweenReports = 500

        data = []
        data[0] = SHTP_REPORT_SET_FEATURE_COMMAND
        data[1] = reportID
        data[2] = 0  # Feature flags
        data[3] = 0  # Change sensitivity (LSB)
        data[4] = 0  # Change sensitivity (MSB)
        data[5] = (timeBetweenReports >> 0) & 0xFF   # Report interval (LSB) in microseconds. 0x7A120 = 500ms
        data[6] = (timeBetweenReports >> 8) & 0xFF   # Report interval
        data[7] = (timeBetweenReports >> 16) & 0xFF  # Report interval
        data[8] = (timeBetweenReports >> 24) & 0xFF  # Report interval (MSB)
        data[9] = 0  # Batch Interval (LSB)
        data[10] = 0  # Batch Interval
        data[11] = 0  # Batch Interval
        data[12] = 0  # Batch Interval (MSB)
        data[13] = (specificConfig >> 0) & 0xFF   # Sensor-specific config (LSB)
        data[14] = (specificConfig >> 8) & 0xFF   # Sensor-specific config
        data[15] = (specificConfig >> 16) & 0xFF  # Sensor-specific config
        data[16] = (specificConfig >> 24) & 0xFF  # Sensor-specific config (MSB)
        self._send_shtp_command(CHANNEL_CONTROL, 17, data)

    def begin(self):
        # reset

        # Check communication with device
        data = []
        data[0] = SHTP_REPORT_PRODUCT_ID_REQUEST
        data[1] = 0
        # Transmit packet on channel 2, 2 bytes
        self._send_shtp_command(CHANNEL_CONTROL, 2, data)

        # Now we wait for response
        if self._receive_packet():
            if self.receivedData[0] is SHTP_REPORT_PRODUCT_ID_RESPONSE:
                return True

        return False

    def enable_rotation_vector(self, update_time):
        # Enter operation mode to read sensor data.
        self._set_feature_command(SENSOR_REPORTID_ROTATION_VECTOR, update_time * 1000)

    def data_available(self):
        response = self._receive_packet()
        if response is False:
            return False
        else:
            return response

