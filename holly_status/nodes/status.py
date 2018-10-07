#!/usr/bin/env python
import rospy
import traceback
import gpiozero
from time import sleep
from std_msgs.msg import UInt8MultiArray

PIN_LED = 21

imuProblem = False


# Setup the connection to the optical flow sensor
led = gpiozero.LED(PIN_LED)


def toggle_led():
    global imuProblem

    led.on()

    if imuProblem:
        sleep(0.2)
    else:
        sleep(1)

    led.off()

    if imuProblem:
        sleep(0.2)
    else:
        sleep(1)


def imu_status_callback(data):
    global imuProblem
    print(data)
    system_status, gyro_status, accel_status, mag_status = data.data
    # print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(system_status, gyro_status, accel_status, mag_status))
    if system_status == 3:
        imuProblem = False
    else:
        imuProblem = True


rospy.init_node('holly_status_output')

rospy.Subscriber("/imu/debug", UInt8MultiArray, imu_status_callback)


while not rospy.is_shutdown():
    try:
        toggle_led()

    except (KeyboardInterrupt, SystemExit):
        led.off()
        raise
    except:
        led.off()
        traceback.print_exc()
