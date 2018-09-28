#!/usr/bin/env python
import rospy
import tf
import time
import traceback
import math
import sys, getopt
from BNO055 import BNO055

sys.path.append('.')
import RTIMU
import os.path
import time
import math

from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from tf.transformations import quaternion_from_euler


imu = BNO055.BNO055()

rospy.init_node('holly_imu')
rate = rospy.Rate(25)

# setup publisher and classes
imuPub = rospy.Publisher('imu/data', Imu, queue_size=10)
msg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
magMsg = MagneticField()

statusPub = rospy.Publisher('imu/debug', UInt8MultiArray, queue_size=10)
statusMsg = UInt8MultiArray()


seq = 1

sensorSetupNeeded = True
sensorCalibrationSaved = False
sensorCalibrationLoaded = False
sensorCalibrationFetched = False
cal_data = []

if os.path.exists("imu-cal.txt"):
    sensorCalibrationSaved = True
    sensorCalibrationFetched = True
    with open("imu-cal.txt", "r") as f:
        for line in f:
            cal_data.append(int(line.strip()))
    print "Calibration data loaded"


def get_data():
    global seq, sensorSetupNeeded, sensorCalibrationSaved, sensorCalibrationLoaded, cal_data

    if not sensorSetupNeeded:
        try:

            # Read the Euler angles for heading, roll, pitch (all in degrees).
            heading, roll, pitch = imu.read_euler()

            # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
            system_status, gyro_status, accel_status, mag_status = imu.get_calibration_status()

            # Print everything out.
            print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
                heading, roll, pitch, system_status, gyro_status, accel_status, mag_status))

            # Publish the status flags so we can see whats going on
            statusMsg.data = system_status, gyro_status, accel_status, mag_status
            statusPub.publish(statusMsg)

            # If we have a successful calibration mark the data as loaded
            if not sensorCalibrationLoaded and system_status == 3:
                sensorCalibrationLoaded = True

            # If we have calibration data and it hasn't been loaded then load it
            if sensorCalibrationFetched and not sensorCalibrationLoaded:
                imu.set_calibration(cal_data)

            if not sensorCalibrationSaved and system_status == 3:
                sensorCalibrationSaved = True
                cal_data = imu.get_calibration()
                print cal_data
                with open("imu-cal.txt", "w") as f:
                    for cal in cal_data:
                        f.write(str(cal) + "\n")
                print "Calibration data saved"


            seq += 1

            # Only publish sensor data if its a reasonable quality
            if mag_status > 1:

                # Publish the mag data

                magMsg.header.seq = seq
                magMsg.header.stamp = rospy.Time.now()
                magMsg.header.frame_id = "base_link"

                # Magnetometer data (in micro-Teslas):
                x, y, z = imu.read_magnetometer()
                magMsg.magnetic_field.x = x / 1000000  # Convert to Teslas
                magMsg.magnetic_field.y = y / 1000000
                magMsg.magnetic_field.z = z / 1000000
                magMsg.magnetic_field_covariance = [0.1] * 9

                magPub.publish(magMsg)

            if system_status > 0 and gyro_status > 1:

                # Publish the gyro and accel data

                msg.header.seq = seq
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"

                # x, y, z, w = imu.read_quaternion()
                # Translate the heading, angle needs to be reversed
                # Roll and pitch are also swapped for some reason
                quaternion = quaternion_from_euler((pitch * -1) * 1000 / 57296, (roll * -1) * 1000 / 57296, (360 - heading) * 1000 / 57296)
                msg.orientation.x = quaternion[0]  # x
                msg.orientation.y = quaternion[1]  # y
                msg.orientation.z = quaternion[2]  # x
                msg.orientation.w = quaternion[3]  # w
                msg.orientation_covariance = [0.001] * 9
                # print('Orientation: X={0:0.8F} Y={1:0.8F} Z={2:0.8F} W={2:0.8F}'.format(x, y, z, w))

                # Gyroscope data (in degrees per second):
                x, y, z = imu.read_gyroscope()
                msg.angular_velocity.x = x * 1000 / 57296  # Convert to rad/s
                msg.angular_velocity.y = y * 1000 / 57296
                msg.angular_velocity.z = z * 1000 / 57296
                msg.angular_velocity_covariance = [1] * 9
                print('Gyro: X={0:0.2F} Y={1:0.2F} Z={2:0.2F}'.format(x, y, z))

                # Accelerometer data (in meters per second squared):
                x, y, z = imu.read_accelerometer()
                # if accel_status > 1:
                msg.linear_acceleration.x = x
                msg.linear_acceleration.y = y
                msg.linear_acceleration.z = z
                msg.linear_acceleration_covariance = [10] * 9

                print('Accelerometer: X={0:0.2F} Y={1:0.2F} Z={2:0.2F}'.format(x, y, z))

                imuPub.publish(msg)

            else:
                rospy.logwarn('Sensor accuracy to low')

        except IOError:
            # print 'Error reading uv sensor'
            rospy.logwarn('Error reading the imu sensor')
            sensorSetupNeeded = True

    rate.sleep()


while not rospy.is_shutdown():
    try:
        if sensorSetupNeeded:
            try:
                if not imu.begin():
                    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

                imu.set_external_crystal(True)

                # Print system status and self test result.
                status, self_test, error = imu.get_system_status()
                print('System status: {0}'.format(status))
                print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
                # Print out an error if system status is in error mode.
                if status == 0x01:
                    print('System error: {0}'.format(error))
                    print('See datasheet section 4.3.59 for the meaning.')

                # Print BNO055 software revision and other diagnostic data.
                sw, bl, accel, mag, gyro = imu.get_revision()
                print('Software version:   {0}'.format(sw))
                print('Bootloader version: {0}'.format(bl))
                print('Accelerometer ID:   0x{0:02X}'.format(accel))
                print('Magnetometer ID:    0x{0:02X}'.format(mag))
                print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
                sensorSetupNeeded = False
                firstReading = 1
                sensorCalibrationLoaded = False
            except IOError:
                # print 'Error setting up uv sensor'
                rospy.logwarn('Error setting up the imu sensor')
                sensorSetupNeeded = True

        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
