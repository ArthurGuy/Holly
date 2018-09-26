#!/usr/bin/env python
import rospy
import traceback
import sys
from VL53L0X.python.VL53L0X import VL53L0X
from VL53L0X.python.VL53L0X import VL53L0X_BETTER_ACCURACY_MODE
from sensor_msgs.msg import Range
from time import sleep
sys.path.append('.')

rospy.init_node('holly_chassis_height') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

rangePublisher = rospy.Publisher('/holly/chassis_height', Range, queue_size=10)
rangeMessage = Range()

seq = 1

device_setup = 0


def setup_sensor():
    global device_setup, tof

    print "Setting up sensor"

    if device_setup:
        return

    tof = VL53L0X()
    tof.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
    device_setup = 1

    print "Device setup"
    sleep(1)


def get_data():
    global seq, device_setup

    try:
        if not device_setup:
            setup_sensor()
    except IOError:
        print "Error setting up device, waiting"
        sleep(5)
        return

    seq += 1

    distance_in_mm = tof.get_distance()  # Grab the range in mm
    print float(distance_in_mm) / 1000


    rangeMessage.header.seq = seq
    rangeMessage.header.stamp = rospy.Time.now()
    rangeMessage.header.frame_id = "base_link"

    rangeMessage.radiation_type = 1
    rangeMessage.min_range = 0.05
    rangeMessage.max_range = 8
    rangeMessage.field_of_view = 0.01
    rangeMessage.range = float(distance_in_mm) / 1000

    rangePublisher.publish(rangeMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        print "Exiting, shutting down sensor"
        tof.stop_ranging()  # Stop ranging
        raise
    except:
        # Dump a stacktrace and throw the exception again
        traceback.print_exc()
        raise
