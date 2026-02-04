#!/bin/bash
# ============================================
# STEP 1: Test if Gazebo World Loads
# ============================================
# This tests ONLY the world loading - no robot, no nav2
# 
# SUCCESS: Gazebo opens with the office world visible
# FAILURE: Gazebo crashes or doesn't load
#
# Press Ctrl+C to stop when done testing
# ============================================

set -e

echo "=========================================="
echo "STEP 1: Testing Gazebo World Load"
echo "=========================================="
echo ""
echo "What to check:"
echo "  1. Gazebo window opens"
echo "  2. Office world appears (walls, desks, colored waypoint markers)"
echo "  3. You see 3 colored circles on the floor:"
echo "     - GREEN at Reception (2, 1)"
echo "     - ORANGE at Storage (4, 7.5)"  
echo "     - BLUE at Office (12.5, 3)"
echo ""
echo "Press Ctrl+C when done testing."
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Get package path
PKG_PATH=$(dirname "$(readlink -f "$0")")/delivery_robot

# Source workspace if built
if [ -f "$(dirname "$0")/install/setup.bash" ]; then
    source "$(dirname "$0")/install/setup.bash"
fi

# Find world file
WORLD_FILE="$PKG_PATH/worlds/office_world_simple.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    WORLD_FILE="$PKG_PATH/worlds/office_world.sdf"
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found!"
    exit 1
fi

echo "Using world: $WORLD_FILE"
echo ""

# Launch Gazebo directly with the world
gz sim -r "$WORLD_FILE"

