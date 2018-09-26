#!/usr/bin/env python
import rospy
import traceback
import sys
from APDS9301.APDS9301 import *
from std_msgs.msg import Float32
sys.path.append('.')

rospy.init_node('holly_light_sensor')  # public display name of the publisher
# rate = rospy.Rate(1/30)  # every 30 seconds

lightPublisher = rospy.Publisher('/environment/uv_index', Float32, queue_size=10)
lightMessage = Float32()

firstReading = 1
sensorSetupNeeded = 1


def get_data():
    global sensorSetupNeeded, firstReading

    if not sensorSetupNeeded:
        try:
            lux = sensor.acquire()

            if firstReading:
                # The first reading isn't accurate so ignore that one
                firstReading = 0
                return

            print 'Vis:             ' + str(lux)

            lightMessage.data = round(lux)

            lightPublisher.publish(lightMessage)
        except IOError:
            # print 'Error reading uv sensor'
            rospy.logwarn('Error reading the lux sensor')
            sensorSetupNeeded = 1

    rospy.sleep(30)


while not rospy.is_shutdown():
    try:
        if sensorSetupNeeded:
            try:
                sensor = APDS9301()
                sensor.init(0x39, GAIN_LOW, TIMING_13_7)
                sensorSetupNeeded = 0
                firstReading = 1
            except IOError:
                # print 'Error setting up uv sensor'
                rospy.logwarn('Error setting up the lux sensor')
                sensorSetupNeeded = 1

        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
