#!/usr/bin/env python
import rospy
import traceback
import sys
import SRF08.SRF08 as SRF08
from sensor_msgs.msg import Range
sys.path.append('.')

rospy.init_node('holly_chassis_height') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

rangePublisher = rospy.Publisher('/holly/chassis_height', Range, queue_size=10)
rangeMessage = Range()

seq = 1

sensor = SRF08.SRF08(0xE0)

def get_data():
    global seq

    seq += 1

    rangeMessage.header.seq = seq
    rangeMessage.header.stamp = rospy.Time.now()
    rangeMessage.header.frame_id = "base_link"

    rangeMessage.range = sensor.distance()

    rangePublisher.publish(rangeMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
