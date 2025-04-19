#!/bin/bash
set -e
. /catkin_ws/devel/setup.bash

python3 open_gripper.py $@