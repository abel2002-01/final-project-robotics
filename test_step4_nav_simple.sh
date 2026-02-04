#!/bin/bash
# ============================================
# STEP 4: Test Simple Navigation (one goal)
# ============================================
# This tests if robot can navigate to ONE goal
#
# SUCCESS: Robot navigates to Storage waypoint (orange circle)
# FAILURE: Navigation fails or robot gets stuck
# ============================================

set -e

echo "=========================================="
echo "STEP 4: Testing Simple Navigation"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start Gazebo + Robot + Bridge + Nav2"
echo "  2. Set initial pose at Reception"
echo "  3. Send ONE navigation goal to Storage"
echo ""
echo "Press Ctrl+C to stop when done."
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Get paths
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Build and source workspace
cd "$SCRIPT_DIR"
if [ ! -d "install" ]; then
    echo "Building workspace..."
    colcon build --packages-select delivery_robot
fi
source "$SCRIPT_DIR/install/setup.bash"

# Use simulation_bringup which has proper setup
echo "Launching simulation (this may take 30+ seconds to fully initialize)..."
echo ""
echo "Watch for:"
echo "  - Gazebo window opens"
echo "  - Robot appears at Reception (green circle)"
echo "  - RViz shows map and robot"
echo ""
echo "Once Nav2 is ready, open another terminal and run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "    '{pose: {header: {frame_id: \"map\"}, pose: {position: {x: 4.0, y: 7.5}}}}'"
echo ""

ros2 launch delivery_robot simulation_bringup.launch.py planner:=smac

