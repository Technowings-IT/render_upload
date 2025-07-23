#!/bin/bash
set -e  # Exit on any error

echo "Starting Robot Control..."
source /opt/ros/jazzy/setup.bash
source /fleet-management-system/ros_ws/install/local_setup.bash

# Start robot control
ros2 launch amr ros2_control_robot.launch.py

echo "Robot control script completed"
