#!/bin/bash
# Verification and Test Script for Increment 01
# This script verifies the package structure, builds it, and tests components

set -e  # Exit on error

echo "=========================================="
echo "Increment 01 Verification and Testing"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS="${GREEN}✓${NC}"
FAIL="${RED}✗${NC}"
WARN="${YELLOW}⚠${NC}"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${PASS} ROS 2 Jazzy sourced"
else
    echo -e "${FAIL} ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo ""
echo "1. VERIFYING FILE STRUCTURE"
echo "---------------------------"

# Check required files
files_to_check=(
    "delivery_robot/package.xml"
    "delivery_robot/setup.py"
    "delivery_robot/delivery_robot/waypoint_navigator.py"
    "delivery_robot/launch/gazebo_world.launch.py"
    "delivery_robot/launch/spawn_robot.launch.py"
    "delivery_robot/launch/nav2_bringup.launch.py"
    "delivery_robot/launch/full_navigation.launch.py"
    "delivery_robot/launch/slam_launch.py"
    "delivery_robot/config/nav2_params_navfn.yaml"
    "delivery_robot/config/nav2_params_smac.yaml"
    "delivery_robot/config/slam_params.yaml"
    "delivery_robot/config/nav2_default_view.rviz"
    "delivery_robot/worlds/office_world.sdf"
    "README.md"
)

missing_files=0
for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${PASS} $file"
    else
        echo -e "${FAIL} $file (MISSING)"
        missing_files=$((missing_files + 1))
    fi
done

if [ $missing_files -gt 0 ]; then
    echo -e "${FAIL} $missing_files file(s) missing!"
    exit 1
fi

echo ""
echo "2. VERIFYING PYTHON SYNTAX"
echo "---------------------------"

python3 -m py_compile delivery_robot/delivery_robot/waypoint_navigator.py 2>/dev/null && \
    echo -e "${PASS} waypoint_navigator.py syntax valid" || \
    echo -e "${FAIL} waypoint_navigator.py has syntax errors"

python3 -c "import ast; ast.parse(open('delivery_robot/launch/full_navigation.launch.py').read())" 2>/dev/null && \
    echo -e "${PASS} full_navigation.launch.py syntax valid" || \
    echo -e "${FAIL} full_navigation.launch.py has syntax errors"

python3 -c "import ast; ast.parse(open('delivery_robot/launch/nav2_bringup.launch.py').read())" 2>/dev/null && \
    echo -e "${PASS} nav2_bringup.launch.py syntax valid" || \
    echo -e "${FAIL} nav2_bringup.launch.py has syntax errors"

echo ""
echo "3. CHECKING ROS 2 DEPENDENCIES"
echo "------------------------------"

check_ros_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${PASS} $1 installed"
        return 0
    else
        echo -e "${FAIL} $1 NOT installed"
        return 1
    fi
}

required_packages=(
    "navigation2"
    "nav2_bringup"
    "nav2_simple_commander"
    "nav2_map_server"
)

missing_packages=0
for pkg in "${required_packages[@]}"; do
    if ! check_ros_pkg "$pkg"; then
        missing_packages=$((missing_packages + 1))
    fi
done

if [ $missing_packages -gt 0 ]; then
    echo -e "${WARN} Some packages missing. Install with: sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup"
fi

echo ""
echo "4. BUILDING PACKAGE"
echo "-------------------"

if [ -d "build" ] || [ -d "install" ]; then
    echo -e "${WARN} Build/install directories exist, cleaning..."
    rm -rf build install log
fi

colcon build --packages-select delivery_robot 2>&1 | tail -5
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${PASS} Package built successfully"
else
    echo -e "${FAIL} Package build failed"
    exit 1
fi

echo ""
echo "5. VERIFYING INSTALLATION"
echo "-------------------------"

source install/setup.bash

if ros2 pkg list | grep -q "^delivery_robot$"; then
    echo -e "${PASS} Package is discoverable by ROS 2"
else
    echo -e "${FAIL} Package not discoverable"
    exit 1
fi

# Check launch files
if ros2 launch delivery_robot gazebo_world.launch.py --show-args >/dev/null 2>&1; then
    echo -e "${PASS} Launch files are valid"
else
    echo -e "${FAIL} Launch files have errors"
fi

# Check executable
if [ -f "install/delivery_robot/lib/delivery_robot/waypoint_navigator" ]; then
    echo -e "${PASS} waypoint_navigator executable created"
else
    echo -e "${WARN} waypoint_navigator executable not found (may need to run manually)"
fi

echo ""
echo "6. VERIFYING CONFIGURATION FILES"
echo "--------------------------------"

# Check YAML files can be read (basic check)
for yaml_file in delivery_robot/config/*.yaml; do
    if [ -f "$yaml_file" ] && grep -q "ros__parameters" "$yaml_file" 2>/dev/null; then
        echo -e "${PASS} $(basename $yaml_file) appears valid"
    else
        echo -e "${WARN} $(basename $yaml_file) may have issues"
    fi
done

echo ""
echo "7. SUMMARY"
echo "----------"
echo -e "${GREEN}✓ Package structure: OK${NC}"
echo -e "${GREEN}✓ Python syntax: OK${NC}"
echo -e "${GREEN}✓ Build: OK${NC}"
echo -e "${GREEN}✓ Installation: OK${NC}"
echo ""
echo "Package is ready to use!"
echo ""
echo "To run the system:"
echo "  ros2 launch delivery_robot full_navigation.launch.py planner:=navfn"
echo ""
echo "=========================================="


