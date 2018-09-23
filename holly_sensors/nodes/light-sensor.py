#!/usr/bin/env python
import rospy
import traceback
import sys, getopt
import random
import SI1145.SI1145 as SI1145
import time
from holly_sensors.msg import LightReading
sys.path.append('.')

rospy.init_node('holly_light_sensor') #public display name of the publisher
rate = rospy.Rate(1) # 1hz

lightPublisher = rospy.Publisher('/holly/light_sensor', LightReading, queue_size=10)
lightMessage = LightReading()

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

    lightMessage.header.seq = seq
    lightMessage.header.stamp = rospy.Time.now()
    lightMessage.header.frame_id = "base_link"

    lightMessage.vis = vis
    lightMessage.ir = IR
    lightMessage.uv = uvIndex

    lightPublisher.publish(lightMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
