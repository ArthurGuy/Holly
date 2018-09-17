#!/usr/bin/env python
import rospy
import traceback
import gpiozero
from time import sleep

PIN_LED = 21


# Setup the connection to the optical flow sensor
led = gpiozero.LED(PIN_LED)


def toggle_led():
    led.on()
    sleep(1)
    led.off()
    sleep(1)


while not rospy.is_shutdown():
    try:
        toggle_led()

    except (KeyboardInterrupt, SystemExit):
        led.off()
        raise
    except:
        led.off()
        traceback.print_exc()
