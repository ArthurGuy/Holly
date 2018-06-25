#!/usr/bin/env python
import rospy

from LSM9DS1 import IMU
import time

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

# setup publisher and classes
pub = rospy.Publisher('imu', Imu, queue_size=10)
rospy.init_node('get_data', anonymous=True)
rate = rospy.Rate(10) # 10hz
msg = Imu()

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

    imu.read_accel()
    imu.read_mag()
    imu.read_gyro()
    imu.readTemp()

    data = imu.getIMUData()
    seq += 1
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()

    msg.orientation.x = imu.mx
    msg.orientation.y = imu.my
    msg.orientation.z = imu.mz
    msg.orientation.w = 0
    
    msg.angular_velocity.x = imu.gx
    msg.angular_velocity.y = imu.gy
    msg.angular_velocity.z = imu.gz

    msg.linear_acceleration.x = imu.ax
    msg.linear_acceleration.y = imu.ay
    msg.linear_acceleration.z = imu.az
    
    pub.publish(msg)
    
    rate.sleep()

    
while not rospy.is_shutdown():
    try:
        get_data()
    
    except (KeyboardInterrupt, SystemExit):
        raise 
    except:
        traceback.print_exc()
