#!/usr/bin/env python
import rospy
import tf
import time
import traceback
import math
import sys, getopt
from BNO080 import BNO080

sys.path.append('.')
import os.path
import time
import math

from std_msgs.msg import Int8MultiArray, Bool, Float32, MultiArrayLayout
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from tf.transformations import euler_from_quaternion


imu = BNO080.BNO080()

rospy.init_node('holly_imu')
rate = rospy.Rate(20)


# setup publisher and classes
imuPub = rospy.Publisher('imu/data', Imu, queue_size=5)
imuMsg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=5)
magMsg = MagneticField()

statusPub = rospy.Publisher('imu/debug', Int8MultiArray, queue_size=1)
statusMsg = Int8MultiArray()

directionAccuracyPub = rospy.Publisher('imu/direction_accuracy', Float32, queue_size=1)
directionAccuracyMsg = Float32()

no_data_count = 0
restart_count = 0

seq = 1

updateCalibration = False
sensorSetupNeeded = True
sensorCalibrationSaved = False
sensorCalibrationLoaded = False
sensorCalibrationFetched = False
cal_data = []

rospy.loginfo("IMU starting")


def setup_imu():
    global restart_count
    if not imu.begin():
        raise RuntimeError('Failed to initialize BNO080. Is the sensor connected?')

    imu.enable_rotation_vector(50)
    imu.enable_accelerometer(100)
    imu.enable_gyro(100)
    imu.enable_magnetometer(100)

    imu.calibrate_all()

    imu.save_dcd_periodically_command()

    restart_count = restart_count + 1


setup_imu()

while not rospy.is_shutdown():
    try:
        if seq == 100:
            # Stop dynamically calibrating the gyro to avoid unwanted drift
            rospy.loginfo("Stopping dynamic gyro calibration")
            imu.calibrate_main()

        if imu.data_available():
            no_data_count = 0

            # print('IMU data available')
            # print ''
            mag_accuracy = imu.get_mag_accuracy()
            sensor_accuracy = imu.get_quat_accuracy()
            accel_accuracy = imu.get_accelerometer_accuracy()
            gyro_accuracy = imu.get_gyro_accuracy()
            gyro_data_delay = imu.get_gyro_data_delay()  # delay in (us)
            mag_data_delay = imu.get_gyro_data_delay()  # delay in (us)

            # cal_status, cal_accel, cal_gyro, cal_mag = imu.get_calibration_status()
            # print('Calibration status, Status: {0}, Accel: {1}, Gyro: {2}, Mag: {3}'.format(cal_status, cal_accel, cal_gyro, cal_mag))

            # print('Delay: {0}'.format(gyro_data_delay))

            # print('Calibration: Sys={0} Mag={1} Accel={2} Gyro={3}'.format(sensor_accuracy, mag_accuracy, accel_accuracy, gyro_accuracy))
            i, j, k, real = imu.get_rotation_quaternion()
            rotation_accuracy = imu.get_rotation_accuracy()
            # print('Orientation: I={0:0.8F} J={1:0.8F} K={2:0.8F} Real={3:0.8F} Accuracy={4}'.format(i, j, k, real, rotation_accuracy))
            # angles = euler_from_quaternion([i, j, k, real])
            # print('Roll={0:0.2F} Pitch={1:0.2F} Heading={2:0.2F} '.format(angles[0], angles[1], angles[2]))

            accelX, accelY, accelZ = imu.get_acceleration()
            # print('Acceleration: X={0:0.8F} Y={1:0.8F} Z={2:0.8F}'.format(accelX, accelY, accelZ))

            gyroX, gyroY, gyroZ = imu.get_gyro()
            # print('Gyro: X={0:0.8F} Y={1:0.8F} Z={2:0.8F}'.format(gyroX, gyroY, gyroZ))

            magX, magY, magZ = imu.get_mag()
            # print('Mag: X={0:0.8F} Y={1:0.8F} Z={2:0.8F}'.format(magX, magY, magZ))

            # Publish the status flags so we can see whats going on
            statusLayout = MultiArrayLayout()
            statusLayout.dim[0].label = 'overall'
            statusLayout.dim[1].label = 'gyro'
            statusLayout.dim[2].label = 'accel'
            statusLayout.dim[3].label = 'mag'
            statusMsg.layout = statusLayout
            statusMsg.data = sensor_accuracy, gyro_accuracy, accel_accuracy, mag_accuracy
            statusPub.publish(statusMsg)

            directionAccuracyMsg.data = rotation_accuracy
            directionAccuracyPub.publish(directionAccuracyMsg)

            seq += 1

            # Publish the mag data #
            magMsg.header.seq = seq
            magMsg.header.stamp = rospy.Time.now() - rospy.Duration(mag_data_delay / 1000000)
            magMsg.header.frame_id = "base_link"

            # Magnetometer data (in micro-Teslas):
            magMsg.magnetic_field.x = magX / 1000000  # Convert to Teslas
            magMsg.magnetic_field.y = magY / 1000000
            magMsg.magnetic_field.z = magZ / 1000000
            magMsg.magnetic_field_covariance = [0.01, 0.00, 0.00,
                                                0.00, 0.01, 0.00,
                                                0.00, 0.00, 0.01]

            magPub.publish(magMsg)


            # Publish the gyro and accel data #

            imuMsg.header.seq = seq
            imuMsg.header.stamp = rospy.Time.now() - rospy.Duration(gyro_data_delay / 1000000)
            imuMsg.header.frame_id = "base_link"

            imuMsg.orientation.x = i
            imuMsg.orientation.y = j
            imuMsg.orientation.z = k
            imuMsg.orientation.w = real
            imuMsg.orientation_covariance = [0.01, 0.00, 0.00,
                                             0.00, 0.01, 0.00,
                                             0.00, 0.00, 0.01]

            # Gyroscope data (in degrees per second):
            imuMsg.angular_velocity.x = gyroX
            imuMsg.angular_velocity.y = gyroY
            imuMsg.angular_velocity.z = gyroZ
            imuMsg.angular_velocity_covariance = [0.01, 0.00, 0.00,
                                                  0.00, 0.01, 0.00,
                                                  0.00, 0.00, 0.01]

            # Accelerometer data (in meters per second squared):
            imuMsg.linear_acceleration.x = accelX
            imuMsg.linear_acceleration.y = accelY
            imuMsg.linear_acceleration.z = accelZ
            imuMsg.linear_acceleration_covariance = [0.01, 0.00, 0.00,
                                                     0.00, 0.01, 0.00,
                                                     0.00, 0.00, 0.01]

            imuPub.publish(imuMsg)

        else:
            print('No IMU data available')
            no_data_count = no_data_count + 1
            if no_data_count == 10:
                setup_imu()

        rate.sleep()
    except:
        traceback.print_exc()

if rospy.is_shutdown():
    imu.stop()
    print 'Finished'
