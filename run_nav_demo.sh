#!/bin/bash
#===============================================================================
# Navigation Demo with Proper Simulation Time Synchronization
#
# This script runs the navigation demo with proper clock synchronization
# between Gazebo and Nav2. All nodes use simulation time.
#
# Usage: ./run_nav_demo.sh [planner]
#   planner: 'navfn' or 'smac' (default: smac)
#===============================================================================

set -e

PLANNER=${1:-smac}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/test_results/demo_$(date +%Y%m%d_%H%M%S)"
RUN_ID="demo_$(date +%H%M%S)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT INT TERM

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║          🤖 DELIVERY ROBOT NAVIGATION DEMO 🤖                        ║"
echo "║                                                                      ║"
echo "║  Using SmacPlanner2D algorithm for path planning                     ║"
echo "║  Route: Reception → Storage → Office                                 ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${GREEN}Planner: $PLANNER${NC}"
echo ""

# Setup
mkdir -p "$OUTPUT_DIR"
source /opt/ros/jazzy/setup.bash
cd "$SCRIPT_DIR"

if [[ ! -d "install" ]]; then
    echo -e "${YELLOW}Building workspace...${NC}"
    colcon build --packages-select delivery_robot
fi
source install/setup.bash

echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 1: Starting Gazebo Simulation...${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"

# Start Gazebo
ros2 launch delivery_robot gazebo_world.launch.py &
GAZEBO_PID=$!
sleep 12
echo -e "${GREEN}✓ Gazebo running${NC}"

# Spawn robot  
echo -e "${YELLOW}Step 2: Spawning Robot...${NC}"
ros2 launch delivery_robot spawn_robot.launch.py &
sleep 8
echo -e "${GREEN}✓ Robot spawned${NC}"

echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 3: Starting ROS-Gazebo Bridges with Simulation Time...${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"

# CRITICAL: Start clock bridge FIRST
echo "  Starting clock bridge..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    --ros-args -p use_sim_time:=true &
sleep 3

# Verify clock is working
echo "  Verifying clock..."
if timeout 5 ros2 topic echo /clock --once > /dev/null 2>&1; then
    echo -e "${GREEN}  ✓ Clock bridge working${NC}"
else
    echo -e "${YELLOW}  ⚠ Clock may not be ready yet, continuing...${NC}"
fi

# Start other bridges with use_sim_time
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    --ros-args -p use_sim_time:=true &
sleep 1
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    --ros-args -p use_sim_time:=true &
sleep 1
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
    --ros-args -p use_sim_time:=true &
sleep 1

# Start TF publisher with sim time
ros2 run delivery_robot odom_to_tf --ros-args -p use_sim_time:=true &
sleep 2

echo -e "${GREEN}✓ Bridges active with simulation time${NC}"

echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 4: Starting Nav2 with ${PLANNER} planner...${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"

ros2 launch delivery_robot nav2_bringup.launch.py planner:=$PLANNER &
NAV2_PID=$!
sleep 25
echo -e "${GREEN}✓ Nav2 active${NC}"

echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}Step 5: Starting Navigation Mission!${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"

echo -e "${GREEN}Watch Gazebo - robot will navigate:${NC}"
echo "  1. Reception (2, 1) → Start"
echo "  2. Storage (4, 7.5)"
echo "  3. Office (12.5, 3) → End"
echo ""

# Run navigation with sim time
ros2 run delivery_robot waypoint_navigator \
    --ros-args \
    -p planner_type:=$PLANNER \
    -p output_dir:=$OUTPUT_DIR \
    -p run_id:=$RUN_ID \
    -p use_sim_time:=true

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    🎉 DEMO COMPLETED! 🎉                             ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "Results saved to: $OUTPUT_DIR"

read -p "Press Enter to exit..."



