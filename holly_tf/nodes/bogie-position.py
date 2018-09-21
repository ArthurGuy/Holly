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

def get_data():
    global seq

    # Read sensor data
    rear = adc.read_adc(0, gain=1)
    left = adc.read_adc(1, gain=1)
    right = adc.read_adc(2, gain=1)

    seq += 1

    jointMessage.header.seq = seq
    jointMessage.header.stamp = rospy.Time.now()
    jointMessage.header.frame_id = "base_link"

    jointMessage.name = ["base_to_bogie_rear"]
    jointMessage.position = [random.uniform(-0.3, 0.3)]

    jointPublisher.publish(jointMessage)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
