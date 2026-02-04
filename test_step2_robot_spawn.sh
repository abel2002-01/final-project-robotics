#!/bin/bash
# ============================================
# STEP 2: Test Robot Spawning
# ============================================
# This tests the robot spawn into the world
#
# Run this AFTER step 1 confirms world works
# This script starts Gazebo, then spawns the robot
#
# SUCCESS: Robot appears at Reception waypoint (green circle)
# FAILURE: Robot doesn't appear, or appears in wrong location
#
# Press Ctrl+C to stop when done testing
# ============================================

set -e

echo "=========================================="
echo "STEP 2: Testing Robot Spawn"
echo "=========================================="
echo ""
echo "What to check:"
echo "  1. Gazebo opens with office world"
echo "  2. After ~5 seconds, robot appears at green marker (Reception)"
echo "  3. Robot is a differential drive robot with LiDAR"
echo ""
echo "Press Ctrl+C when done testing."
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Get paths
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PKG_PATH="$SCRIPT_DIR/delivery_robot"

# Source workspace if built
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# Find files
WORLD_FILE="$PKG_PATH/worlds/office_world_simple.sdf"
ROBOT_FILE="$PKG_PATH/models/delivery_robot.sdf"

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found at $WORLD_FILE"
    exit 1
fi

if [ ! -f "$ROBOT_FILE" ]; then
    echo "ERROR: Robot file not found at $ROBOT_FILE"
    exit 1
fi

echo "World file: $WORLD_FILE"
echo "Robot file: $ROBOT_FILE"
echo ""

# Start Gazebo in background
echo "Starting Gazebo..."
gz sim -r "$WORLD_FILE" &
GZ_PID=$!

# Wait for Gazebo to initialize
echo "Waiting 5 seconds for Gazebo to initialize..."
sleep 5

# Spawn robot
echo "Spawning robot at Reception (2.0, 1.0)..."
gz service -s /world/office_world_professional/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"$ROBOT_FILE\" name: \"delivery_robot\" pose: {position: {x: 2.0 y: 1.0 z: 0.05}}"

echo ""
echo "Robot should now be visible at the green Reception marker."
echo "Press Ctrl+C to stop."
echo ""

# Wait for Gazebo
wait $GZ_PID

