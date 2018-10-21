#!/bin/bash -e

export ROS_IP=192.168.1.60
source /home/holly/catkin_ws/devel/setup.bash
pigpiod
roslaunch holly start.launch
