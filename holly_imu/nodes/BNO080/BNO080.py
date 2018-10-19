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
import pigpio
import Adafruit_GPIO.I2C as I2C


# I2C addresses
BNO080_ADDRESS_A                     = 0x4A
BNO080_ADDRESS_B                     = 0x4B
BNO080_ID                            = 0xF8


# SHTP = Sensor Hub Transport Protocol

# All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
# These are used for low level communication with the sensor, on channel 2

# Channel names
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
    sequenceNumber = [0, 0, 0, 0, 0, 0]

    def __init__(self, address=BNO080_ADDRESS_B, gpio=None, **kwargs):
        # self._rst = None
        # self._serial = None
        # self._i2c_device = None

        self.pi = pigpio.pi()
        self.h = self.pi.i2c_open(1, address)

        # i2c = I2C
        # Save a reference to the I2C device instance for later communication.
        # self._i2c_device = i2c.get_i2c_device(address, **kwargs)

    def _send_shtp_command(self, channelNumber, dataLength, data):

        packetLength = dataLength + 4

        data = [packetLength & 0xFF, packetLength >> 8, channelNumber, self.sequenceNumber[channelNumber]] + data

        # print ''
        # print 'Sending packet'
        # print ' '.join('{:02x}'.format(x) for x in data)
        # self._i2c_device.writeList(0, data)
        self.pi.i2c_write_device(self.h, data)

        # Increment the sequence counter
        self.sequenceNumber[channelNumber] = self.sequenceNumber[channelNumber] + 1

        # self._i2c_device.writeRaw8(packetLength & 0xFF)  # Packet length LSB
        # self._i2c_device.writeRaw8(packetLength >> 8)    # Packet length MSB
        # self._i2c_device.writeRaw8(channelNumber)
        # self._i2c_device.writeRaw8(self.sequenceNumber[channelNumber])  # Send the sequence number, increments with each packet sent, different counter for each channel
        # i = 0
        # while i < dataLength:
        #     self._i2c_device.write8(i + 4, data[i])
        #     i += 1

    def _receive_packet(self):
        (count, data) = self.pi.i2c_read_device(self.h, 4)
        if count <= 0:
            return False
        # data = self._i2c_device.readList(0, 4)

        # Store the header info.
        shtpHeader = [data[0], data[1], data[2], data[3]]

        # Calculate the number of data bytes in this packet
        dataLength = shtpHeader[1] << 8 | shtpHeader[0]
        dataLength &= ~(1 << 15)  # Clear the MSbit.

        if dataLength == 0:
            return False
        else:
            print ''
            print 'Received packet:'
            # print ' '.join('{:02x}'.format(x) for x in shtpHeader)
            print('Length: {0}'.format(dataLength - 4))
            print('Channel: {0}'.format(shtpHeader[2]))

            # receivedData = self._i2c_device.readList(0, dataLength)
            (count, receivedData) = self.pi.i2c_read_device(self.h, dataLength)
            self.receivedHeader = receivedData[0:4]
            self.receivedData = receivedData[4:dataLength]
            print ' '.join('{:02x}'.format(x) for x in self.receivedData)
            return True

    def _set_feature_command(self, reportID, specificConfig):

        timeBetweenReports = 500

        data = []
        data.append(SHTP_REPORT_SET_FEATURE_COMMAND)
        data.append(reportID)
        data.append(0)  # Feature flags
        data.append(0)  # Change sensitivity (LSB)
        data.append(0)  # Change sensitivity (MSB)
        data.append((timeBetweenReports >> 0) & 0xFF)   # Report interval (LSB) in microseconds. 0x7A120 = 500ms
        data.append((timeBetweenReports >> 8) & 0xFF)   # Report interval
        data.append((timeBetweenReports >> 16) & 0xFF)  # Report interval
        data.append((timeBetweenReports >> 24) & 0xFF)  # Report interval (MSB)
        data.append(0)  # Batch Interval (LSB)
        data.append(0)  # Batch Interval
        data.append(0)  # Batch Interval
        data.append(0)  # Batch Interval (MSB)
        data.append((specificConfig >> 0) & 0xFF)   # Sensor-specific config (LSB)
        data.append((specificConfig >> 8) & 0xFF)   # Sensor-specific config
        data.append((specificConfig >> 16) & 0xFF)  # Sensor-specific config
        data.append((specificConfig >> 24) & 0xFF)  # Sensor-specific config (MSB)
        self._send_shtp_command(CHANNEL_CONTROL, 17, data)

    def soft_reset(self):
        self._send_shtp_command(CHANNEL_EXECUTABLE, 1, [1])
        time.sleep(0.5)
        new_data = True
        while new_data:
            new_data = self._receive_packet()
            time.sleep(0.1)

    def begin(self):
        # reset
        print 'Resetting'
        self.soft_reset()
        print 'Reset complete'

        # Check communication with device
        data = [SHTP_REPORT_PRODUCT_ID_REQUEST, 0]
        # Transmit packet on channel 2, 2 bytes
        self._send_shtp_command(CHANNEL_CONTROL, 2, data)

        # Now we wait for response
        if self._receive_packet():
            if self.receivedData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE:
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
            if self.receivedHeader == CHANNEL_REPORTS:
                self._parse_input_report()
            elif self.receivedHeader == CHANNEL_CONTROL:
                self._parse_command_report()
            return True

    def get_data_array(self):
        return self.receivedData

    def _parse_input_report(self):
        report_id = self.receivedData[5]
        sequence_number = self.receivedData[6]
        status = self.receivedData[7] & 0x03
        data1 = self.receivedData[10] << 8 | self.receivedData[9]
        data2 = self.receivedData[12] << 8 | self.receivedData[11]
        data3 = self.receivedData[14] << 8 | self.receivedData[13]
        data4 = 0
        data5 = 0
        if len(self.receivedData) > 14:
            data4 = self.receivedData[16] << 8 | self.receivedData[15]
        if len(self.receivedData) > 16:
            data5 = self.receivedData[18] << 8 | self.receivedData[17]

        if report_id == SENSOR_REPORTID_ACCELEROMETER:
            self.accelAccuracy = status
            self.rawAccelX = data1
            self.rawAccelY = data2
            self.rawAccelZ = data3
        elif report_id == SENSOR_REPORTID_LINEAR_ACCELERATION:
            self.accelLinAccuracy = status
            self.rawLinAccelX = data1
            self.rawLinAccelY = data2
            self.rawLinAccelZ = data3
        elif report_id == SENSOR_REPORTID_GYROSCOPE:
            self.gyroAccuracy = status
            self.rawGyroX = data1
            self.rawGyroY = data2
            self.rawGyroZ = data3
        elif report_id == SENSOR_REPORTID_MAGNETIC_FIELD:
            self.magAccuracy = status
            self.rawMagX = data1
            self.rawMagY = data2
            self.rawMagZ = data3
        elif report_id == SENSOR_REPORTID_ROTATION_VECTOR or report_id == SENSOR_REPORTID_GAME_ROTATION_VECTOR:
            self.quatAccuracy = status
            self.rawQuatI = data1
            self.rawQuatJ = data2
            self.rawQuatK = data3
            self.rawQuatReal = data4
            self.rawQuatRadianAccuracy = data5  # Only available on rotation vector, not game rot vector
        elif report_id == SENSOR_REPORTID_STEP_COUNTER:
            self.stepCount = data3
        elif report_id == SENSOR_REPORTID_STABILITY_CLASSIFIER:
            self.stabilityClassifier = self.receivedData[9]
        elif report_id == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
            self.activityClassifier = self.receivedData[10]
        elif report_id == SHTP_REPORT_COMMAND_RESPONSE:
            # The BNO080 responds with this report to command requests.
            # It's up to use to remember which command we issued.
            command = self.receivedData[7]  # This is the Command byte of the response
            if command == COMMAND_ME_CALIBRATE:
                # Calibration report found
                self.calibrationStatus = self.receivedData[10]  # R0 - Status (0 = success, non-zero = fail)

    def _parse_command_report(self):
        if self.receivedData[0] == SHTP_REPORT_COMMAND_RESPONSE:
            command = self.receivedData[2]
            if command == COMMAND_ME_CALIBRATE:
                self.calibrationStatus = self.receivedData[4]