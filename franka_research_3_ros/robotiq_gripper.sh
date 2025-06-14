#!/bin/bash
set -e
. /catkin_ws/devel/setup.bash

# Check if a device is specified as an argument, otherwise use default
DEVICE=${1:-/dev/ttyUSB0}

# Start the Robotiq RTU node
echo "Starting Robotiq2FGripperRtuNode on device $DEVICE"
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py $DEVICE &
PID_RTU=$!

# Wait for the RTU node to start up
sleep 2

# Start the Robotiq action server
echo "Starting Robotiq action server"
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch

# Set up a trap to kill the RTU node when this script is terminated
trap "kill $PID_RTU" EXIT

# Keep the script running
wait