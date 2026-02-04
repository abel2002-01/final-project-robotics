#!/bin/bash
# Quick Start Guide - Copy and paste these commands

echo "=========================================="
echo "  DELIVERY ROBOT - QUICK START"
echo "=========================================="
echo ""
echo "Choose your method:"
echo ""
echo "METHOD 1: Automated Script (Easiest)"
echo "  ./run_demo.sh smac"
echo ""
echo "METHOD 2: Manual (3 terminals)"
echo "  See RUN_DEMO_GUIDE.md for details"
echo ""
echo "=========================================="
echo ""
echo "Current status:"
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

echo "ROS_DISTRO: $ROS_DISTRO"
echo "Package: $(ros2 pkg list | grep delivery_robot 2>/dev/null || echo 'Not found - need to build')"
echo "Executables: $(ros2 pkg executables delivery_robot 2>/dev/null | wc -l)"
echo ""
echo "✅ Ready to run!"

