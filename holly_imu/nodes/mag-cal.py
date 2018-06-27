#!/usr/bin/env python
import rospy

from LSM9DS1.LSM9DS1 import IMU
import time
import traceback
import math

# Setup the IMU
imu = IMU()

# Initialize IMU
imu.initialize()
imu.enable_mag()
imu.mag_range("4GAUSS")

mag_max = [0, 0, 0]
mag_min = [0, 0, 0]
mag_temp = [0, 0, 0]
mag_bias = [0, 0, 0]


def get_readings():
    for i in range(1, 100):
        imu.read_mag()

        mag_temp[0] = imu.mx
        mag_temp[1] = imu.my
        mag_temp[2] = imu.mz

        for x in range(0, 3):

            if mag_temp[x] > mag_max[x]:
                mag_max[x] = mag_temp[x]
            if mag_temp[x] < mag_min[x]:
                mag_min[x] = mag_temp[x];

    for x in range(0, 3):
        mag_bias[x] = (mag_max[x] + mag_min[x]) / 2

    print "Offsets " + str(mag_bias[0]) + ", " + str(mag_bias[1]) + ", " + str(mag_bias[2])


while not rospy.is_shutdown():
    try:
        get_readings()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()