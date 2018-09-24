#!/usr/bin/env python
import rospy
import traceback
import sys
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

sys.path.append('.')

rospy.init_node('holly_odom_calculator') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

encoder_count_1 = 0
encoder_count_2 = 0
encoder_count_3 = 0
encoder_count_4 = 0
encoder_count_5 = 0
encoder_count_6 = 0


def odom_callback_1(self, data):
    self.encoder_count_1 = data.data


def odom_callback_2(self, data):
    self.encoder_count_2 = data.data


def odom_callback_3(self, data):
    self.encoder_count_3 = data.data


def odom_callback_4(self, data):
    self.encoder_count_4 = data.data


def odom_callback_5(self, data):
    self.encoder_count_5 = data.data


def odom_callback_6(self, data):
    self.encoder_count_6 = data.data


odomMsg = Odometry()
odomPublisher = rospy.Publisher('/odom', Odometry, queue_size=10)
rospy.Subscriber("/holly/encoder1", Float64, odom_callback_1)
rospy.Subscriber("/holly/encoder2", Float64, odom_callback_2)
rospy.Subscriber("/holly/encoder3", Float64, odom_callback_3)
rospy.Subscriber("/holly/encoder4", Float64, odom_callback_4)
rospy.Subscriber("/holly/encoder5", Float64, odom_callback_5)
rospy.Subscriber("/holly/encoder6", Float64, odom_callback_6)

seq = 1

# Running totals
abs_x_m = 0
abs_y_m = 0


def get_data():
    global seq

    seq += 1

    odomMsg.header.seq = seq
    odomMsg.header.stamp = rospy.Time.now()
    odomMsg.header.frame_id = "odom"
    odomMsg.child_frame_id = "base_link"

    pose = Pose()
    pose.position.x = encoder_count_1
    pose.position.y = encoder_count_2
    odomMsg.pose.pose = pose

    odomPublisher.publish(odomMsg)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
