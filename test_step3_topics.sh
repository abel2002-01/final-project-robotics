#!/bin/bash
# ============================================
# STEP 3: Test ROS2 Topics
# ============================================
# This tests if robot topics are publishing properly
#
# Run this WHILE step 2 (robot spawn) is running in another terminal
#
# SUCCESS: You see data from /odom, /scan, /cmd_vel topics
# FAILURE: Topics missing or no data
# ============================================

set -e

echo "=========================================="
echo "STEP 3: Testing ROS2 Topics"
echo "=========================================="
echo ""
echo "PREREQUISITE: Run test_step2_robot_spawn.sh in another terminal first!"
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace if built
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "=========================================="
echo "Part A: Listing all topics..."
echo "=========================================="
ros2 topic list
echo ""

echo "=========================================="
echo "Part B: Checking critical topics..."
echo "=========================================="

# Check for odom
echo ""
echo "--- /odom topic ---"
if ros2 topic list | grep -q "/odom"; then
    echo "✓ /odom topic EXISTS"
    echo "Type: $(ros2 topic type /odom 2>/dev/null || echo 'unknown')"
    echo ""
    echo "Checking if publishing (5 second timeout)..."
    timeout 5s ros2 topic hz /odom 2>/dev/null | head -3 || echo "⚠ No data received (topic may not be publishing)"
else
    echo "✗ /odom topic NOT FOUND"
fi

echo ""
echo "--- /scan topic ---"
if ros2 topic list | grep -q "/scan"; then
    echo "✓ /scan topic EXISTS"
    echo "Type: $(ros2 topic type /scan 2>/dev/null || echo 'unknown')"
    echo ""
    echo "Checking if publishing (5 second timeout)..."
    timeout 5s ros2 topic hz /scan 2>/dev/null | head -3 || echo "⚠ No data received (topic may not be publishing)"
else
    echo "✗ /scan topic NOT FOUND"
fi

echo ""
echo "--- /cmd_vel topic ---"
if ros2 topic list | grep -q "/cmd_vel"; then
    echo "✓ /cmd_vel topic EXISTS"
    echo "Type: $(ros2 topic type /cmd_vel 2>/dev/null || echo 'unknown')"
else
    echo "✗ /cmd_vel topic NOT FOUND"
fi

echo ""
echo "=========================================="
echo "Part C: Check TF tree..."
echo "=========================================="
echo "Active TF frames:"
ros2 run tf2_tools tf2_echo odom base_link 2>/dev/null &
TF_PID=$!
sleep 3
kill $TF_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "SUMMARY"
echo "=========================================="
echo "For navigation to work, you need:"
echo "  ✓ /odom - robot odometry (must be publishing)"
echo "  ✓ /scan - LiDAR data (must be publishing)"
echo "  ✓ /cmd_vel - velocity commands (must exist)"
echo "  ✓ TF: odom -> base_link transform"
echo ""
echo "If any are missing, check:"
echo "  - Robot model has correct plugins"
echo "  - ros_gz_bridge is running"
echo "  - Topic names match (may need remapping)"

