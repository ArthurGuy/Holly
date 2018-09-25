#!/usr/bin/env python
import rospy
import traceback
import sys
import VL53L1X
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

    if device_setup:
        return
    try:
        tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof.open() # Initialise the i2c bus and configure the sensor
        tof.start_ranging(1)  # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
        device_setup = 1
        sleep(1)
    except IOError:
        sleep(5)
        raise


def get_data():
    global seq

    seq += 1

    distance_in_mm = tof.get_distance()  # Grab the range in mm
    print distance_in_mm


    rangeMessage.header.seq = seq
    rangeMessage.header.stamp = rospy.Time.now()
    rangeMessage.header.frame_id = "base_link"

    rangeMessage.range = float(distance_in_mm) / 100

    rangePublisher.publish(rangeMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        setup_sensor()
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        tof.stop_ranging()  # Stop ranging
        traceback.print_exc()
