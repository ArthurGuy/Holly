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

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


imu = BNO055.BNO055(rst='P9_12')

rospy.init_node('holly_imu')
rate = rospy.Rate(10) # 10hz

# setup publisher and classes
imuPub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
msg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
magMsg = MagneticField()

seq = 1

if not imu.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

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


def get_data():
    global seq

    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = imu.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = imu.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
        heading, roll, pitch, sys, gyro, accel, mag))

    seq += 1

    # Publish the mag data

    magMsg.header.seq = seq
    magMsg.header.stamp = rospy.Time.now()
    magMsg.header.frame_id = "base_link"

    # Magnetometer data (in micro-Teslas):
    x, y, z = imu.read_magnetometer()
    magMsg.magnetic_field.x = x
    magMsg.magnetic_field.y = y
    magMsg.magnetic_field.z = z

    magPub.publish(magMsg)

    # Publish the gyro and accel data

    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    quaternion = tf.transformations.quaternion_from_euler(heading, roll, pitch)

    msg.orientation.x = quaternion[0]
    msg.orientation.y = quaternion[1]
    msg.orientation.z = quaternion[2]
    msg.orientation.w = quaternion[3]

    # Gyroscope data (in degrees per second):
    x,y,z = imu.read_gyroscope()

    msg.angular_velocity.x = x
    msg.angular_velocity.y = y
    msg.angular_velocity.z = z

    # Accelerometer data (in meters per second squared):
    x,y,z = imu.read_accelerometer()

    #msg.linear_acceleration.x = accelData[0] * 9.80665
    #msg.linear_acceleration.y = accelData[1] * 9.80665
    #msg.linear_acceleration.z = accelData[2] * 9.80665

    imuPub.publish(msg)


    #print "Published new data " + str(seq)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
