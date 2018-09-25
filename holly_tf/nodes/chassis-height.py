#!/usr/bin/env python
import rospy
import traceback
import sys
import VL53L1X
from sensor_msgs.msg import Range
sys.path.append('.')

rospy.init_node('holly_chassis_height') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

rangePublisher = rospy.Publisher('/holly/chassis_height', Range, queue_size=10)
rangeMessage = Range()

seq = 1

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open() # Initialise the i2c bus and configure the sensor

def get_data():
    global seq

    seq += 1

    tof.start_ranging(1)  # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

    distance_in_mm = tof.get_distance()  # Grab the range in mm

    tof.stop_ranging()  # Stop ranging

    rangeMessage.header.seq = seq
    rangeMessage.header.stamp = rospy.Time.now()
    rangeMessage.header.frame_id = "base_link"

    rangeMessage.range = float(distance_in_mm) / 100

    rangePublisher.publish(rangeMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
