#!/usr/bin/env python
import rospy
import traceback
import sys
import SI1145.SI1145 as SI1145
from std_msgs.msg import Float32
sys.path.append('.')

rospy.init_node('holly_light_sensor')  # public display name of the publisher
rate = rospy.Rate(1/30)  # every 30 seconds

lightPublisher = rospy.Publisher('/environment/uv_index', Float32, queue_size=10)
lightMessage = Float32()

firstReading = 1
sensorSetupNeeded = 1


def get_data():
    global sensorSetupNeeded, firstReading

    if not sensorSetupNeeded:
        try:
            vis = sensor.readVisible()
            IR = sensor.readIR()
            UV = sensor.readUV()
            uvIndex = UV / 100.0

            if firstReading:
                # The first reading isn't accurate so ignore that one
                firstReading = 0
                return

            # print 'Vis:             ' + str(vis)
            # print 'IR:              ' + str(IR)
            # print 'UV Index:        ' + str(uvIndex)

            # lightMessage.vis = vis
            # lightMessage.ir = IR
            lightMessage.data = round(uvIndex, 2)

            lightPublisher.publish(lightMessage)
        except IOError:
            # print 'Error reading uv sensor'
            rospy.logwarn('Error reading the uv sensor')
            sensorSetupNeeded = 1

    rate.sleep()


while not rospy.is_shutdown():
    try:
        if sensorSetupNeeded:
            try:
                sensor = SI1145.SI1145()
                sensorSetupNeeded = 0
                firstReading = 1
            except IOError:
                # print 'Error setting up uv sensor'
                rospy.logwarn('Error setting up the uv sensor')
                sensorSetupNeeded = 1

        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
