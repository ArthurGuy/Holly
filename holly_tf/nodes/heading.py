#!/usr/bin/env python
import math
import rospy
import traceback
import sys
import time
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8MultiArray, Bool, Float32
from tf.transformations import euler_from_quaternion

sys.path.append('.')

rospy.init_node('holly_heading') #public display name of the publisher
rate = rospy.Rate(10) # 10hz


def odom_callback(data):
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    print('Orientation ODOM: Roll={0:0.8F} Pitch={1:0.8F} Yaw={2:0.8F}'.format(roll, pitch, yaw))


def imu_callback(data):
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    print('Orientation  IMU: Roll={0:0.8F} Pitch={1:0.8F} Yaw={2:0.8F}'.format(roll, pitch, yaw))


def status_callback(imu_status):
    system_status, gyro_status, accel_status, mag_status = imu_status.data
    print('Status: Sys={0} Mag={1} Linear_accel={2} Gyro={3}'.format(system_status, mag_status, accel_status, gyro_status))


# odomMsg = Odometry()
# rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)

imuMsg = Imu()
rospy.Subscriber("/imu/data", Imu, imu_callback)

statusMessage = Int8MultiArray()
rospy.Subscriber("/imu/debug", Imu, status_callback)


def update_position():

    rate.sleep()


while not rospy.is_shutdown():
    try:
        update_position()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
