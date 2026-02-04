#!/bin/bash
# Complete Test Script for Increment 01
# This script launches the complete system step by step

set -e

echo "=========================================="
echo "Increment 01 - Complete System Test"
echo "=========================================="

PROJECT_DIR="/home/abel/robo/Final-Project/increment_01_basic_navigation"
cd "$PROJECT_DIR"

# Source ROS 2
echo "[1/6] Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

# Check if package is built
if ! ros2 pkg list | grep -q "^delivery_robot$"; then
    echo "Package not found. Building..."
    colcon build --packages-select delivery_robot
    source install/setup.bash
fi

echo "[2/6] Launching Gazebo world..."
ros2 launch delivery_robot gazebo_world.launch.py &
GAZEBO_PID=$!
sleep 8  # Wait for Gazebo to fully load

echo "[3/6] Spawning robot..."
# Spawn robot using gz service directly
gz service -s /world/office_world_professional/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Turtlebot3 Waffle", name: "turtlebot3_waffle", pose: {position: {x: 2.0, y: 1.0, z: 0.01}}' 2>/dev/null || echo "Robot spawn attempted"

sleep 3

echo "[4/6] Starting ros_gz_bridge for robot topics..."
# Bridge essential topics
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    --ros-args -p use_sim_time:=true &
BRIDGE_PID=$!
sleep 3

echo "[5/6] Checking topics..."
echo "Available ROS 2 topics:"
ros2 topic list

echo ""
echo "[6/6] System Status"
echo "==================="
echo "Gazebo PID: $GAZEBO_PID"
echo "Bridge PID: $BRIDGE_PID"
echo ""
echo "To teleoperate the robot:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "To stop, press Ctrl+C or run:"
echo "  kill $GAZEBO_PID $BRIDGE_PID"
echo ""
echo "=========================================="
echo "System is running! Check Gazebo window."
echo "=========================================="

# Keep script running
wait $GAZEBO_PID

