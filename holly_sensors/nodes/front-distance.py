#!/usr/bin/env python
import rospy
import traceback
import sys
import signal
from VL53L0X.python.VL53L0X import VL53L0X
from VL53L0X.python.VL53L0X import VL53L0X_BETTER_ACCURACY_MODE
from sensor_msgs.msg import Range
sys.path.append('.')

rospy.init_node('front_distance_sensor') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

rangePublisher = rospy.Publisher('/holly/range/front', Range, queue_size=10)
rangeMessage = Range()

seq = 1

sensorSetupNeeded = 1


def handler(signum, frame):
    global sensorSetupNeeded
    print 'Error setting up the range sensor'
    rospy.logwarn('Error setting up range sensor')
    sensorSetupNeeded = 1
    raise Exception("timeout")


signal.signal(signal.SIGALRM, handler)


def get_data():
    global seq, sensorSetupNeeded

    if not sensorSetupNeeded:
        try:
            distance_in_mm = tof.get_distance()  # Grab the range in mm

            if distance_in_mm > 0:
                seq += 1

                print float(distance_in_mm) / 1000

                rangeMessage.header.seq = seq
                rangeMessage.header.stamp = rospy.Time.now()
                rangeMessage.header.frame_id = "front_distance_sensor"

                rangeMessage.radiation_type = 1
                rangeMessage.min_range = 0.05
                rangeMessage.max_range = 2
                rangeMessage.field_of_view = 0.436 # 25 degrees
                rangeMessage.range = float(distance_in_mm) / 1000

                rangePublisher.publish(rangeMessage)
        except:
            rospy.logwarn('Error reading the range sensor')
            sensorSetupNeeded = 1
            # sleep(5)

    rate.sleep()


while not rospy.is_shutdown():
    # Reset the watchdog to 2 seconds
    signal.alarm(2)

    try:
        if sensorSetupNeeded:
            # try:
            tof = VL53L0X(0x29, 0, 0x70)
            tof_rear = VL53L0X(0x29, 1, 0x70)
            tof.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
            tof_rear.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
            sensorSetupNeeded = 0
            # except:
            #     print 'Error setting up the range sensor'
            #     rospy.logwarn('Error setting up range sensor')
            #     sensorSetupNeeded = 1

        get_data()

    except (KeyboardInterrupt, SystemExit):
        print "Exiting, shutting down sensor"
        # tof.stop_ranging()  # Stop ranging
        raise
    except:
        # Dump a stacktrace and throw the exception again
        traceback.print_exc()
        raise
