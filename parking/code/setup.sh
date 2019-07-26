#!/bin/bash

# ROS MASTER NODE
# REDROVER Camera IP: 192.168.21.102
cd catkin_workspace
export ROS_MASTER_URI=http://192.168.1.143:11311
export ROS_HOSTNAME=192.168.1.143
source devel/setup.bash
