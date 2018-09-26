#!/usr/bin/env python
import rospy
import traceback
import sys
from BME280.BME280 import *
from std_msgs.msg import Float64, Float32
from time import sleep
sys.path.append('.')

rospy.init_node('holly_pressure_sensor')  # public display name of the publisher
rate = rospy.Rate(0.5)  # 0.1hz

pressurePublisher = rospy.Publisher('/holly/pressure_sensor', Float32, queue_size=10)
pressureMessage = Float32()

try:
    sensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
    sensorSetupNeeded = 0
except IOError:
    print 'Error setting up sensor'
    sensorSetupNeeded = 1


def get_data():
    global sensorSetupNeeded

    if not sensorSetupNeeded:
        try:
            degrees = sensor.read_temperature()
            pascals = sensor.read_pressure()
            hectopascals = pascals / 100
            humidity = sensor.read_humidity()

            print 'Temp      = {0:0.3f} deg C'.format(degrees)
            print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
            print 'Humidity  = {0:0.2f} %'.format(humidity)

            pressureMessage.data = round(hectopascals, 2)

            pressurePublisher.publish(pressureMessage)
        except IOError:
            print 'Error reading sensor'
            sensorSetupNeeded = 1
            # sleep(5)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        if sensorSetupNeeded:
            try:
                sensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
                sensorSetupNeeded = 0
            except IOError:
                print 'Error setting up sensor'
                sensorSetupNeeded = 1

        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
