#!/usr/bin/env python
import rospy
import tf
import time
import traceback
import math
import sys, getopt
import gpiozero
from time import sleep


from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


PIN_MOUSECAM_CS    = 5
PIN_MOUSECAM_RESET = 6


ADNS3080_PIXELS_X                = 30
ADNS3080_PIXELS_Y                = 30

ADNS3080_PRODUCT_ID           = 0x00
ADNS3080_REVISION_ID          = 0x01
ADNS3080_MOTION               = 0x02
ADNS3080_DELTA_X              = 0x03
ADNS3080_DELTA_Y              = 0x04
ADNS3080_SQUAL                = 0x05
ADNS3080_PIXEL_SUM            = 0x06
ADNS3080_MAXIMUM_PIXEL        = 0x07
ADNS3080_CONFIGURATION_BITS   = 0x0a
ADNS3080_EXTENDED_CONFIG      = 0x0b
ADNS3080_DATA_OUT_LOWER       = 0x0c
ADNS3080_DATA_OUT_UPPER       = 0x0d
ADNS3080_SHUTTER_LOWER        = 0x0e
ADNS3080_SHUTTER_UPPER        = 0x0f
ADNS3080_FRAME_PERIOD_LOWER   = 0x10
ADNS3080_FRAME_PERIOD_UPPER   = 0x11
ADNS3080_MOTION_CLEAR         = 0x12
ADNS3080_FRAME_CAPTURE        = 0x13
ADNS3080_SROM_ENABLE          = 0x14
ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER     = 0x19
ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER     = 0x1a
ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER     = 0x1b
ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER     = 0x1c
ADNS3080_SHUTTER_MAX_BOUND_LOWER          = 0x1e
ADNS3080_SHUTTER_MAX_BOUND_UPPER          = 0x1e
ADNS3080_SROM_ID              = 0x1f
ADNS3080_OBSERVATION          = 0x3d
ADNS3080_INVERSE_PRODUCT_ID   = 0x3f
ADNS3080_PIXEL_BURST          = 0x40
ADNS3080_MOTION_BURST         = 0x50
ADNS3080_SROM_LOAD            = 0x60
ADNS3080_PRODUCT_ID_VAL       = 0x17


# Setup the connection to the optical flow sensor
opti_flow_sensor = gpiozero.SPIDevice(port=0, device=0)
opti_flow_reset = gpiozero.LED(PIN_MOUSECAM_RESET)
opti_flow_cs = gpiozero.LED(PIN_MOUSECAM_CS)

# Setup the ROS publisher
rospy.init_node('holly_optical_flow') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

# setup publisher and classes
odomPub = rospy.Publisher('optical_flow/odom', Odometry, queue_size=10)
msg = Odometry()

seq = 1

def sensor_reset():
    opti_flow_reset.on()
    sleep(0.001) # reset pulse >10us
    opti_flow_reset.off()
    sleep(0.035) # 35ms from reset to functional

def sensor_init():
    opti_flow_cs.on()

    sensor_reset()

    pid = mousecam_read_reg(ADNS3080_PRODUCT_ID)
    if pid != ADNS3080_PRODUCT_ID_VAL:
        return -1

    #turn on sensitive mode
    mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19)
    return 0


def mousecam_write_reg(reg, val):
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([reg | 0x80])
    opti_flow_sensor._spi.transfer([val])
    opti_flow_cs.on()
    sleep(0.00005)

def mousecam_read_reg(reg):
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([reg])
    sleep(0.000075)
    ret = opti_flow_sensor._spi.transfer([0xff]);
    opti_flow_cs.on()
    sleep(0.000005)
    return ret;

class Move:
    def __init__(self):
        self.motion = 0
        self.dx = 0
        self.dy = 0
        self.squal = 0
        self.shutter = 0
        self.max_pix = 0

def mousecam_read_motion():
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([ADNS3080_MOTION_BURST])
    sleep(0.000075)

    m = Move()

    m.motion =  opti_flow_sensor._spi.transfer([0xff])
    m.dx =  opti_flow_sensor._spi.transfer([0xff])
    m.dy =  opti_flow_sensor._spi.transfer([0xff])
    m.squal =  opti_flow_sensor._spi.transfer([0xff])
    m.shutter =  opti_flow_sensor._spi.transfer([0xff])<<8
    m.shutter |=  opti_flow_sensor._spi.transfer([0xff])
    m.max_pix =  opti_flow_sensor._spi.transfer([0xff])

    opti_flow_cs.on()
    sleep(0.000005)


def get_data():
    global seq

    seq += 1


    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"

    msg.pose.pose = Pose(Point(x, y, 0))

    odomPub.publish(msg)


    rate.sleep()


# Setup the sensor
sensor_init()

while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        spi.close() 
        raise
    except:
        traceback.print_exc()
