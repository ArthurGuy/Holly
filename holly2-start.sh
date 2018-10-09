#!/bin/bash -e

export ROS_IP=192.168.1.61
export ROS_MASTER_URI=http://192.168.1.60:11311
source /home/holly/catkin_ws/devel/setup.bash
roslaunch holly start2.launch