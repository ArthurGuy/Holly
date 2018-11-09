#!/usr/bin/env python
import rospy
import traceback
import sys
import signal
from VL53L0X.python.VL53L0X import VL53L0X
from VL53L0X.python.VL53L0X import VL53L0X_BETTER_ACCURACY_MODE
from sensor_msgs.msg import Range
sys.path.append('.')

rospy.init_node('distance_sensors')  # public display name of the publisher
rate = rospy.Rate(10)  # 10hz

range1Publisher = rospy.Publisher('/holly/range/front_left', Range, queue_size=5)
range1Message = Range()

range5Publisher = rospy.Publisher('/holly/range/front_right', Range, queue_size=5)
range5Message = Range()

range2Publisher = rospy.Publisher('/holly/range/rear', Range, queue_size=5)
range2Message = Range()

seq1 = 1
seq2 = 1
seq5 = 1

sensorSetupNeeded = 1


def handler(signum, frame):
    global sensorSetupNeeded
    print 'Error setting up the range sensor'
    rospy.logwarn('Error setting up range sensor')
    sensorSetupNeeded = 1
    raise Exception("timeout")


signal.signal(signal.SIGALRM, handler)


def get_data():
    global seq1, seq2, seq5, sensorSetupNeeded

    if not sensorSetupNeeded:
        try:
            sensor_1_distance = sensor_1.get_distance()  # Grab the range in mm
            sensor_2_distance = sensor_2.get_distance()  # Grab the range in mm
            sensor_5_distance = sensor_5.get_distance()  # Grab the range in mm

            if sensor_1_distance > 0:
                seq1 += 1

                range1Message.header.seq = seq1
                range1Message.header.stamp = rospy.Time.now()
                range1Message.header.frame_id = "front_left_distance_sensor"

                range1Message.radiation_type = 1
                range1Message.min_range = 0.05
                range1Message.max_range = 2
                range1Message.field_of_view = 0.436  # 25 degrees
                range1Message.range = float(sensor_1_distance) / 1000

                range1Publisher.publish(range1Message)

            if sensor_5_distance > 0:
                seq5 += 1

                range5Message.header.seq = seq5
                range5Message.header.stamp = rospy.Time.now()
                range5Message.header.frame_id = "front_right_distance_sensor"

                range5Message.radiation_type = 1
                range5Message.min_range = 0.05
                range5Message.max_range = 2
                range5Message.field_of_view = 0.436  # 25 degrees
                range5Message.range = float(sensor_5_distance) / 1000

                range5Publisher.publish(range5Message)

            if sensor_2_distance > 0:
                seq2 += 1

                range2Message.header.seq = seq2
                range2Message.header.stamp = rospy.Time.now()
                range2Message.header.frame_id = "rear_distance_sensor"

                range2Message.radiation_type = 1
                range2Message.min_range = 0.05
                range2Message.max_range = 2
                range2Message.field_of_view = 0.436  # 25 degrees
                range2Message.range = float(sensor_2_distance) / 1000

                range2Publisher.publish(range2Message)
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
            sensor_1 = VL53L0X(0x29, 0, 0x70)
            sensor_2 = VL53L0X(0x29, 1, 0x70)
            sensor_5 = VL53L0X(0x29, 4, 0x70)
            sensor_1.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
            sensor_2.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
            sensor_5.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)
            sensorSetupNeeded = 0

        get_data()

    except (KeyboardInterrupt, SystemExit):
        print "Exiting, shutting down sensor"
        # tof.stop_ranging()  # Stop ranging
        raise
    except:
        # Dump a stacktrace and throw the exception again
        traceback.print_exc()
        raise
