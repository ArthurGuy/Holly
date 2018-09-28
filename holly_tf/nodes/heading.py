#!/usr/bin/env python
import math
import rospy
import traceback
import sys
import time
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
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
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print('Orientation: Roll={0:0.8F} Pitch={1:0.8F} Yaw={2:0.8F}'.format(roll, pitch, yaw))


odomMsg = Odometry()
rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)


def update_position():

    rate.sleep()


while not rospy.is_shutdown():
    try:
        update_position()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
