#!/bin/bash
source /opt/ros/humble/setup.bash
source /fleet-management-system/ros_ws/install/local_setup.sh

ros2 launch nav2_bringup bringup_launch.py map:=/home/piros/fleet-management-system/ros_ws/src/AMR/maps/map_1750065869.yaml params_file:=/home/piros/fleet-management-system/ros_ws/src/AMR/config/nav2_params1.yaml use_sim_time:=False