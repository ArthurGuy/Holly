#!/usr/bin/env python
import rospy
import traceback
import sys, getopt
import random
from sensor_msgs.msg import JointState
sys.path.append('.')

rospy.init_node('holly_bogie_position') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

jointPublisher = rospy.Publisher('/holly/joint_states', JointState, queue_size=10)
jointMessage = JointState()

seq = 1


def get_data():
    global seq

    # Read sensor data

    seq += 1

    jointMessage.header.seq = seq
    jointMessage.header.stamp = rospy.Time.now()
    jointMessage.header.frame_id = "base_link"

    jointMessage.name = "base_to_bogie_rear"
    jointMessage.position = random.uniform(-0.3, 0.3)

    jointPublisher.publish(jointMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
