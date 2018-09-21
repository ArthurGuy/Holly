#!/usr/bin/env python
import rospy
import tf
import time
import traceback
import math
import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imuSensor = RTIMU.RTIMU(s)

print("IMU Name: " + imuSensor.IMUName())

if (not imuSensor.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

imuSensor.setSlerpPower(0.02)
imuSensor.setGyroEnable(True)
imuSensor.setAccelEnable(True)
imuSensor.setCompassEnable(True)

poll_interval = imuSensor.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

rospy.init_node('holly_imu') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

# setup publisher and classes
imuPub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
msg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
magMsg = MagneticField()

seq = 1


def get_data():
    global seq

    if imuSensor.IMURead():

        seq += 1

        data = imuSensor.getIMUData()
        fusionPose = data["fusionPose"]

        gyroData = data["gyro"]
        accelData = data["accel"]
        compassData = data["compass"]

        # Publish the mag data

        magMsg.header.seq = seq
        magMsg.header.stamp = rospy.Time.now()
        magMsg.header.frame_id = "base_link"

        magMsg.magnetic_field.x = compassData[0]
        magMsg.magnetic_field.y = compassData[1]
        magMsg.magnetic_field.z = compassData[2]

        magPub.publish(magMsg)


        # Publish the gyro and accel data

        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        quaternion = tf.transformations.quaternion_from_euler(fusionPose[0], fusionPose[1], fusionPose[2])

        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        msg.angular_velocity.x = gyroData[0]
        msg.angular_velocity.y = gyroData[1]
        msg.angular_velocity.z = gyroData[2]

        msg.linear_acceleration.x = accelData[0] * 9.80665
        msg.linear_acceleration.y = accelData[1] * 9.80665
        msg.linear_acceleration.z = accelData[2] * 9.80665

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
