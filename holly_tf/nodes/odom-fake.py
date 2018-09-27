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

sys.path.append('.')

rospy.init_node('holly_odom_fake')
rate = rospy.Rate(10) # 10hz

odomMsg = Odometry()
odomPublisher = rospy.Publisher('/odom', Odometry, queue_size=10)

seq = 1


def update_position():
    global seq

    seq += 1

    odomMsg.header.seq = seq
    odomMsg.header.stamp = rospy.Time.now()
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"

    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    odomMsg.pose.pose = pose

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    odomMsg.twist.twist = twist

    odomPublisher.publish(odomMsg)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        update_position()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
