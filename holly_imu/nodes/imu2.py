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

from tf.transformations import quaternion_from_euler


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

imu.enable_rotation_vector(100)  # Send data update every 100ms
imu.enable_magnetometer(100)

imu.calibrate_all()


while not rospy.is_shutdown():
    try:
        if imu.data_available():
            print('IMU data available')
            mag_accuracy = imu.get_mag_accuracy()
            sensor_accuracy = imu.get_quat_accuracy()
            print('Sys_cal={0} Mag_cal={1}'.format(sensor_accuracy, mag_accuracy))
            i, j, k, real = imu.get_rotation_quaternion()
            print('Orientation: I={0:0.8F} J={1:0.8F} K={2:0.8F} Real={2:0.8F}'.format(i, j, k, real))
        else:
            print('No IMU data available')

        rate.sleep()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
