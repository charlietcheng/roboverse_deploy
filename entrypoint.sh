#!/bin/bash
set -e
. /catkin_ws/devel/setup.bash

echo $1

if [ "$1" = "main" ]; then
    python3 main.py $@

elif [ "$1" = "gripper" ]; then
    python3 open_gripper.py $@

elif [ "$1" = "camera" ]; then
    python3 src/camera.py $@
fi