#!/usr/bin/env python
import rospy
import traceback
import sys, getopt
import random
import Adafruit_ADS1x15
import time
from sensor_msgs.msg import JointState
sys.path.append('.')

rospy.init_node('holly_bogie_position') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

jointPublisher = rospy.Publisher('/holly/joint_states', JointState, queue_size=10)
jointMessage = JointState()

seq = 1

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)
adc = Adafruit_ADS1x15.ADS1115()

# Gain
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V

# The full range is 32767
# Using the gai we can work out the volt reading per bit
v_per_bit = 4.096/32767

# TODO - Workout how many bits per radian
bits_per_rad = 10000.0

# On startup assume the rover is level and take readings for the midpoints
center_left = adc.read_adc(0, gain=1) / bits_per_rad
center_right = adc.read_adc(1, gain=1) / bits_per_rad
center_rear = adc.read_adc(2, gain=1) / bits_per_rad

def get_data():
    global seq

    # Read sensor data
    try:
        left = round((adc.read_adc(0, gain=1) / bits_per_rad) - center_left, 3)
        right = round((adc.read_adc(1, gain=1) / bits_per_rad) - center_right, 3)
        rear = round((adc.read_adc(2, gain=1) / bits_per_rad) - center_rear, 3)

    except IOError:
        rear = 0
        left = 0
        right = 0

    seq += 1

    jointMessage.header.seq = seq
    jointMessage.header.stamp = rospy.Time.now()
    jointMessage.header.frame_id = "base_link"

    jointMessage.name = ["base_to_bogie_rear", "base_to_bogie_left", "base_to_bogie_right"]
    jointMessage.position = [rear, left, right]

    jointPublisher.publish(jointMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
