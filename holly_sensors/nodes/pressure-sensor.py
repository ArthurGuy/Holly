#!/usr/bin/env python
import rospy
import traceback
import sys
from BME280.BME280 import *
from std_msgs.msg import Float32
sys.path.append('.')

rospy.init_node('holly_pressure_sensor')  # public display name of the publisher
rate = rospy.Rate(0.1)  # 0.1hz

pressurePublisher = rospy.Publisher('/environment/air_pressure', Float32, queue_size=10)
pressureMessage = Float32()
tempPublisher = rospy.Publisher('/environment/air_temp', Float32, queue_size=10)
tempMessage = Float32()
humidityPublisher = rospy.Publisher('/environment/air_humidity', Float32, queue_size=10)
humidityMessage = Float32()

firstReading = 1
sensorSetupNeeded = 1


def get_data():
    global sensorSetupNeeded, firstReading

    if not sensorSetupNeeded:
        try:
            degrees = sensor.read_temperature()
            pascals = sensor.read_pressure()
            hectopascals = pascals / 100
            humidity = sensor.read_humidity()

            if firstReading:
                # The first reading isn't accurate so ignore that one
                firstReading = 0
                return

            print 'Temp      = {0:0.2f} deg C'.format(degrees)
            print 'Pressure  = {0:0.2f} hPa'.format(hectopascals)
            print 'Humidity  = {0:0.2f} %'.format(humidity)
            print '-'

            pressureMessage.data = round(hectopascals, 2)
            pressurePublisher.publish(pressureMessage)

            tempMessage.data = round(degrees, 2)
            tempPublisher.publish(tempMessage)

            humidityMessage.data = round(humidity, 2)
            humidityPublisher.publish(humidityMessage)
        except IOError:
            print 'Error reading sensor'
            sensorSetupNeeded = 1
            # sleep(5)

    rate.sleep()


while not rospy.is_shutdown():
    try:
        if sensorSetupNeeded:
            try:
                sensor = BME280(t_mode=BME280_OSAMPLE_16, p_mode=BME280_OSAMPLE_16, h_mode=BME280_OSAMPLE_16)
                sensorSetupNeeded = 0
                firstReading = 1
            except IOError:
                print 'Error setting up sensor'
                sensorSetupNeeded = 1

        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
