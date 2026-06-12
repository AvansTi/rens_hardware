#!/bin/bash

source /opt/ros/jazzy/setup.bash
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=a1
source /home/rens/uros_ws/install/setup.bash
source /home/rens/rens_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch linorobot2_bringup bringup.launch.py joy:=true > /dev/null 2>&1