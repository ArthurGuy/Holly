#!/usr/bin/env python
import rospy
import traceback
import gpiozero
from time import sleep
from std_msgs.msg import Int8MultiArray, Float64MultiArray

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


def imu_status_callback(imu_status):
    global imuProblem

    sensor_accuracy, gyro_accuracy, accel_accuracy, mag_accuracy, cal_status, cal_accel, cal_gyro, cal_mag = imu_status.data
    # print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sensor_accuracy, gyro_accuracy, accel_accuracy, mag_accuracy))

    if sensor_accuracy == 0:
        imuProblem = True
    else:
        imuProblem = False


rospy.init_node('holly_status_output')
rospy.Subscriber("/imu/debug", Int8MultiArray, imu_status_callback)


while not rospy.is_shutdown():
    try:
        toggle_led()

    except (KeyboardInterrupt, SystemExit):
        led.off()
        raise
    except:
        led.off()
        traceback.print_exc()
