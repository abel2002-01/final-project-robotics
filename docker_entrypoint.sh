#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Use FastDDS with UDP transport (no shared memory)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/delivery_robot_project/fastdds_profile.xml

# Execute command
exec "$@"

