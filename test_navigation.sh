#!/bin/bash
# Test script to run navigation demo and verify robot movement

set -e

cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║          🤖 TESTING SMACPLANNER2D NAVIGATION 🤖                     ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

# Clean up any existing processes
pkill -f gazebo || true
pkill -f nav2 || true
pkill -f waypoint_navigator || true
sleep 2

# Run the demo
echo "Starting navigation demo with SmacPlanner2D..."
echo "This will take approximately 2-3 minutes..."
echo ""

./run_demo.sh smac

echo ""
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                    ✅ DEMO COMPLETED! ✅                             ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
