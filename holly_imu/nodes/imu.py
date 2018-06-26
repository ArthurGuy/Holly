#!/usr/bin/env python
import rospy

from LSM9DS1.LSM9DS1 import IMU
import time
import traceback
import math

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

rospy.init_node('holly_imu') #public display name of the publisher
rate = rospy.Rate(10) # 10hz

# setup publisher and classes
imuPub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
msg = Imu()

magPub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
magMsg = MagneticField()

seq = 1

# Setup the IMU
imu = IMU()

# Initialize IMU
imu.initialize()

# Enable accel, mag, gyro, and temperature
imu.enable_accel()
imu.enable_mag()
imu.enable_gyro()
imu.enable_temp()

# Set range on accel, mag, and gyro

# Specify Options: "2G", "4G", "6G", "8G", "16G"
imu.accel_range("2G")       # leave blank for default of "2G"

# Specify Options: "4GAUSS", "8GAUSS", "12GAUSS"
imu.mag_range("4GAUSS")     # leave blank for default of "4GAUSS"

# Specify Options: "245DPS", "500DPS", "2000DPS"
imu.gyro_range("245DPS")    # leave blank for default of "245DPS"

def get_data():
    global seq

    imu.read_accel()
    imu.read_mag()
    imu.read_gyro()
    imu.readTemp()

    seq += 1

    # Publish the mag data

    magMsg.header.seq = seq
    magMsg.header.stamp = rospy.Time.now()
    magMsg.header.frame_id = "base_link"

    magMsg.magnetic_field.x = imu.mx
    magMsg.magnetic_field.y = imu.my
    magMsg.magnetic_field.z = imu.mz

    magPub.publish(magMsg)


    # Publish the gyro and accel data

    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.orientation.x = 0
    msg.orientation.y = 0
    msg.orientation.z = 0
    msg.orientation.w = 0

    msg.angular_velocity.x = math.radians(imu.gx)
    msg.angular_velocity.y = math.radians(imu.gy)
    msg.angular_velocity.z = math.radians(imu.gz)

    msg.linear_acceleration.x = imu.ax * 9.80665
    msg.linear_acceleration.y = imu.ay * 9.80665
    msg.linear_acceleration.z = imu.az * 9.80665

    imuPub.publish(msg)


    #print "Published new data"

    rate.sleep()


while not rospy.is_shutdown():
    try:
        get_data()

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()
