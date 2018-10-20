#!/usr/bin/env python
import rospy
import tf
import time
import traceback
import math
import sys, getopt
from BNO080 import BNO080

sys.path.append('.')
import RTIMU
import os.path
import time
import math

from std_msgs.msg import Int8MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from tf.transformations import euler_from_quaternion


imu = BNO080.BNO080()

rospy.init_node('holly_imu2')
rate = rospy.Rate(20)


# setup publisher and classes
imuPub = rospy.Publisher('imu/data', Imu, queue_size=5)
msg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=5)
magMsg = MagneticField()

statusPub = rospy.Publisher('imu/debug', Int8MultiArray, queue_size=1)
statusMsg = Int8MultiArray()


seq = 1

updateCalibration = False
sensorSetupNeeded = True
sensorCalibrationSaved = False
sensorCalibrationLoaded = False
sensorCalibrationFetched = False
cal_data = []

rospy.loginfo("IMU2 starting")

if not imu.begin():
    raise RuntimeError('Failed to initialize BNO080. Is the sensor connected?')

imu.enable_rotation_vector(100)
imu.enable_linear_acceleration(100)
# imu.enable_magnetometer(200)

imu.calibrate_all()


while not rospy.is_shutdown():
    try:
        if imu.data_available():
            # print('IMU data available')
            print ''
            mag_accuracy = imu.get_mag_accuracy()
            sensor_accuracy = imu.get_quat_accuracy()
            print('Sys_cal={0} Mag_cal={1}'.format(sensor_accuracy, mag_accuracy))
            i, j, k, real = imu.get_rotation_quaternion()
            rotation_accuracy = imu.get_rotation_accuracy()
            print('Orientation: I={0:0.8F} J={1:0.8F} K={2:0.8F} Real={3:0.8F} Accuracy={4}'.format(i, j, k, real, rotation_accuracy))
            # angles = euler_from_quaternion([i, j, k, real])
            # print('Roll={0:0.2F} Pitch={1:0.2F} Heading={2:0.2F} '.format(angles[0], angles[1], angles[2]))

            x, y, z = imu.get_linear_acceleration()
            linear_accuracy = imu.get_linear_accuracy()
            print('Acceleration: X={0:0.8F} Y={1:0.8F} Z={2:0.8F} Accuracy={3}'.format(x, y, z, linear_accuracy))
        else:
            print('No IMU data available')

        rate.sleep()
    except:
        traceback.print_exc()

if rospy.is_shutdown():
    imu.stop()
    print 'Finished'
