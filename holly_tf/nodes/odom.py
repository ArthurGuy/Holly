#!/usr/bin/env python
import math
import rospy
import traceback
import sys
import time
from geometry_msgs.msg import Twist, Pose, Point, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray

sys.path.append('.')

rospy.init_node('holly_odom_calculator') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

wheel_travel_1 = 0
wheel_travel_2 = 0
wheel_travel_3 = 0
wheel_travel_4 = 0
wheel_travel_5 = 0
wheel_travel_6 = 0
wheel_angle = 0


def odom_callback(data):
    global wheel_travel_1, wheel_travel_2, wheel_travel_3, wheel_travel_4, wheel_travel_5, wheel_travel_6
    wheel_travel_1 = data.data[0]
    wheel_travel_2 = data.data[1]
    wheel_travel_3 = data.data[2]
    wheel_travel_4 = data.data[3]
    wheel_travel_5 = data.data[4]
    wheel_travel_6 = data.data[5]

#
# def odom_callback_1(data):
#     global wheel_travel_1
#     wheel_travel_1 = data.data
#
#
# def odom_callback_2(data):
#     global wheel_travel_2
#     wheel_travel_2 = data.data
#
#
# def odom_callback_3(data):
#     global wheel_travel_3
#     wheel_travel_3 = data.data
#
#
# def odom_callback_4(data):
#     global wheel_travel_4
#     wheel_travel_4 = data.data
#
#
# def odom_callback_5(data):
#     global wheel_travel_5
#     wheel_travel_5 = data.data
#
#
# def odom_callback_6(data):
#     global wheel_travel_6
#     wheel_travel_6 = data.data


def wheel_angle_callback(data):
    global wheel_angle
    wheel_angle = data.data


odomMsg = Odometry()
odomPublisher = rospy.Publisher('/odom', Odometry, queue_size=10)
# rospy.Subscriber("/imu", Imu, imu_callback)
rospy.Subscriber("/holly/encoders", Float64MultiArray, odom_callback)
# rospy.Subscriber("/holly/encoder1", Float64, odom_callback_1)
# rospy.Subscriber("/holly/encoder2", Float64, odom_callback_2)
# rospy.Subscriber("/holly/encoder3", Float64, odom_callback_3)
# rospy.Subscriber("/holly/encoder4", Float64, odom_callback_4)
# rospy.Subscriber("/holly/encoder5", Float64, odom_callback_5)
# rospy.Subscriber("/holly/encoder6", Float64, odom_callback_6)
rospy.Subscriber("/holly/wheel_angle", Float64, wheel_angle_callback)

last_update_time = time.time()
time_delta = 0

seq = 1

# Running totals
abs_x_m = 0
abs_y_m = 0
average_distance_traveled = 0


def update_position():
    global seq, last_update_time, time_delta, average_distance_traveled, abs_y_m, abs_x_m

    # How long since the last update
    time_delta = time.time() - last_update_time

    # Average the two center wheels
    _average_distance_traveled = ((wheel_travel_2 + wheel_travel_4) / 2)
    average_distance_traveled_delta = _average_distance_traveled - average_distance_traveled
    _speed = average_distance_traveled_delta / time_delta

    x_delta = average_distance_traveled_delta * math.cos(wheel_angle)
    y_delta = average_distance_traveled_delta * math.sin(wheel_angle)

    x_speed = x_delta / time_delta
    y_speed = y_delta / time_delta

    abs_x_m = abs_x_m + x_delta
    abs_y_m = abs_y_m + y_delta


    # Update stored totals
    last_update_time = time.time()
    average_distance_traveled = _average_distance_traveled


    seq += 1

    odomMsg.header.seq = seq
    odomMsg.header.stamp = rospy.Time.now()
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"

    pose = Pose()
    pose.position.x = abs_x_m
    pose.position.y = abs_y_m
    odomMsg.pose.pose = pose
    odomMsg.pose.covariance = [0.001] * 36

    twist = Twist()
    twist.linear.x = x_speed
    twist.linear.y = y_speed
    odomMsg.twist.twist = twist
    odomMsg.twist.covariance = [0.1] * 36

    odomPublisher.publish(odomMsg)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        update_position()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
