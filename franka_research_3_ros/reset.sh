#!/bin/bash

# set -e

# echo "Resetting the robot to the initial state..."

# source /catkin_ws/devel/setup.bash

# rosparam set /target_joint_positions '[0, -1.3, 0, -2.5, 0, 1, 0]'
# roslaunch serl_franka_controllers joint.launch robot_ip:=169.254.67.230 load_gripper:=true

set -e
. /catkin_ws/devel/setup.bash
# roscore

# export ROS_MASTER_URI=http://172.16.0.1:11311
# export ROS_MASTER_URI=http://localhost:11311
# export ROS_MASTER_URI=http://172.16.0.1:33019
# export ROS_MASTER_URI=http://73fe7e63163b:38975
python3 reset.py $@