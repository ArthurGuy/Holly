#!/usr/bin/env python
import rospy
import traceback
import gpiozero
from time import sleep

from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

PIN_SENSOR_CS = 5
PIN_SENSOR_RESET = 6
PIN_LIGHTING = 12

ADNS3080_PIXELS_X = 30
ADNS3080_PIXELS_Y = 30
ADNS3080_COUNTS_PER_INCH = 1600
# This will need trial and error as its dependent on the mounting height
ADNS3080_COUNTS_PER_METER = 1620

ADNS3080_PRODUCT_ID = 0x00
ADNS3080_REVISION_ID = 0x01
ADNS3080_MOTION = 0x02
ADNS3080_DELTA_X = 0x03
ADNS3080_DELTA_Y = 0x04
ADNS3080_SQUAL = 0x05
ADNS3080_PIXEL_SUM = 0x06
ADNS3080_MAXIMUM_PIXEL = 0x07
ADNS3080_CONFIGURATION_BITS = 0x0a
ADNS3080_EXTENDED_CONFIG = 0x0b
ADNS3080_DATA_OUT_LOWER = 0x0c
ADNS3080_DATA_OUT_UPPER = 0x0d
ADNS3080_SHUTTER_LOWER = 0x0e
ADNS3080_SHUTTER_UPPER = 0x0f
ADNS3080_FRAME_PERIOD_LOWER = 0x10
ADNS3080_FRAME_PERIOD_UPPER = 0x11
ADNS3080_MOTION_CLEAR = 0x12
ADNS3080_FRAME_CAPTURE = 0x13
ADNS3080_SROM_ENABLE = 0x14
ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER = 0x19
ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER = 0x1a
ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER = 0x1b
ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER = 0x1c
ADNS3080_SHUTTER_MAX_BOUND_LOWER = 0x1e
ADNS3080_SHUTTER_MAX_BOUND_UPPER = 0x1e
ADNS3080_SROM_ID = 0x1f
ADNS3080_OBSERVATION = 0x3d
ADNS3080_INVERSE_PRODUCT_ID = 0x3f
ADNS3080_PIXEL_BURST = 0x40
ADNS3080_MOTION_BURST = 0x50
ADNS3080_SROM_LOAD = 0x60
ADNS3080_PRODUCT_ID_VAL = 0x17

# Setup the connection to the optical flow sensor
opti_flow_sensor = gpiozero.SPIDevice(port=0, device=0)
opti_flow_reset = gpiozero.LED(PIN_SENSOR_RESET)
opti_flow_cs = gpiozero.LED(PIN_SENSOR_CS)
led_lighting = gpiozero.LED(pin=PIN_LIGHTING, active_high=False)

# Setup the ROS publisher
rospy.init_node('holly_optical_flow')  # public display name of the publisher
rate = rospy.Rate(10)  # 10hz

# setup publisher and classes
odomPub = rospy.Publisher('optical_flow/odom', Odometry, queue_size=10)
msg = Odometry()

qualityPub = rospy.Publisher('optical_flow/quality', Float32, queue_size=1)
qualityMsg = Float32()

seq = 1

# Cumulative offsets
abs_x = 0
abs_y = 0
abs_x_m = 0
abs_y_m = 0

last_time = rospy.get_time()


def sensor_reset():
    opti_flow_reset.on()
    sleep(0.020)  # reset pulse >10us
    opti_flow_reset.off()
    sleep(0.050)  # 35ms from reset to functional


def sensor_init():
    opti_flow_cs.on()

    sensor_reset()
    sleep(0.1)
    sensor_reset()

    pid = sensor_read_reg(ADNS3080_PRODUCT_ID)
    if pid != ADNS3080_PRODUCT_ID_VAL:
        return -1

    # turn on sensitive mode, 1600 counts per inch
    sensor_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19)
    return 0


def sensor_write_reg(reg, val):
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([reg | 0x80])
    opti_flow_sensor._spi.transfer([val])
    opti_flow_cs.on()
    sleep(0.00005)


def sensor_read_reg(reg):
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([reg])
    sleep(0.000075)
    ret = opti_flow_sensor._spi.transfer([0xff]);
    opti_flow_cs.on()
    sleep(0.000005)
    return ret[0]


class Move:
    def __init__(self):
        self.motion = 0
        self.dx = 0
        self.dy = 0
        self.squal = 0
        self.shutter = 0
        self.max_pix = 0


def sensor_read_motion():
    opti_flow_cs.off()
    opti_flow_sensor._spi.transfer([ADNS3080_MOTION_BURST])
    sleep(0.0001)  # > 75us

    m = Move()

    m.motion = opti_flow_sensor._spi.transfer([0xff])[0]

    dx = opti_flow_sensor._spi.transfer([0xff])[0]
    if dx > 127:
        dx = (255 - dx) * -1

    dy = opti_flow_sensor._spi.transfer([0xff])[0]
    if dy > 127:
        dy = (255 - dy) * -1

    m.dx = dx
    m.dy = dy
    m.squal = opti_flow_sensor._spi.transfer([0xff])[0]
    m.shutter = opti_flow_sensor._spi.transfer([0xff])[0] << 8
    m.shutter |= opti_flow_sensor._spi.transfer([0xff])[0]
    m.max_pix = opti_flow_sensor._spi.transfer([0xff])[0]

    opti_flow_cs.on()
    sleep(0.000005)

    return m


def get_data():
    global seq, abs_x, abs_y, abs_x_m, abs_y_m, last_time

    seq += 1

    m = sensor_read_motion()
    if m.motion & 0x10:
        print "Overflow"

    qualityMsg.data = m.squal
    qualityPub.publish(qualityMsg)

    if m.squal < 50:
        led_lighting.on()
    if m.squal > 80:
        led_lighting.off()

    abs_x += m.dx
    abs_y += m.dy

    time_diff = rospy.get_time() - last_time
    last_time = rospy.get_time()

    # Convert the counts reading into metres
    abs_x_m = (float(abs_x) / ADNS3080_COUNTS_PER_METER)
    abs_y_m = (float(abs_y) / ADNS3080_COUNTS_PER_METER)

    speed_x_m = (float(m.dx) / ADNS3080_COUNTS_PER_METER) / time_diff
    speed_y_m = (float(m.dy) / ADNS3080_COUNTS_PER_METER) / time_diff

    # print str(speed_x_m) + ", " + str(speed_y_m)
    # print m.squal

    # Y forward and back

    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"

    pose = Pose()
    pose.position.x = abs_x_m
    pose.position.y = abs_y_m

    poseC = PoseWithCovariance()
    poseC.pose = pose
    poseC.covariance = [-1] * 36
    msg.pose = poseC

    twist = Twist()
    twist.linear.x = speed_y_m
    twist.linear.y = 0
    msg.twist.twist = twist
    msg.twist.covariance = [0.1] * 36



    odomPub.publish(msg)

    rate.sleep()


# Setup the sensor
if sensor_init() == -1:
    print "Error setting up the motion sensor"
    opti_flow_sensor.close()
    led_lighting.off()
    exit()

while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()

if rospy.is_shutdown():
    opti_flow_sensor.close()
    led_lighting.off()
    print 'Finished'