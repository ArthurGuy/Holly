#!/usr/bin/env python
import rospy
import traceback
import sys, getopt
import random
import SI1145.SI1145 as SI1145
import time
sys.path.append('.')

rospy.init_node('holly_light_sensor') #public display name of the publisher
rate = rospy.Rate(1) # 1hz

#jointPublisher = rospy.Publisher('/holly/light_sensor', JointState, queue_size=10)
#jointMessage = JointState()

seq = 1

sensor = SI1145.SI1145()


def get_data():
    global seq

    vis = sensor.readVisible()
    IR = sensor.readIR()
    UV = sensor.readUV()
    uvIndex = UV / 100.0
    print 'Vis:             ' + str(vis)
    print 'IR:              ' + str(IR)
    print 'UV Index:        ' + str(uvIndex)

    seq += 1

    # jointMessage.header.seq = seq
    # jointMessage.header.stamp = rospy.Time.now()
    # jointMessage.header.frame_id = "base_link"
    #
    # jointMessage.name = ["base_to_bogie_rear", "base_to_bogie_left", "base_to_bogie_right"]
    # jointMessage.position = [rear, left, right]
    #
    # jointPublisher.publish(jointMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
