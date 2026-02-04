#!/bin/bash
#
# Automated Navigation Test Runner
# Tests both NavFn and SmacPlanner2D and generates comparison report
#
# Usage:
#   ./automated_test.sh [num_runs]
#
# Example:
#   ./automated_test.sh 3    # Run 3 tests per planner
#

set -e

# Configuration
NUM_RUNS=${1:-1}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/test_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="$OUTPUT_DIR/$TIMESTAMP"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Autonomous Delivery Robot Test Suite${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "  Planners: NavFn, SmacPlanner2D"
echo -e "  Runs per planner: $NUM_RUNS"
echo -e "  Output directory: $RESULTS_DIR"
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
export TURTLEBOT3_MODEL=waffle

# Create results directory
mkdir -p "$RESULTS_DIR/navfn"
mkdir -p "$RESULTS_DIR/smac"

# Function to cleanup background processes
cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    sleep 2
}

# Function to wait for a topic to be available
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    
    echo -n "  Waiting for $topic..."
    while ! ros2 topic list 2>/dev/null | grep -q "$topic"; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
    done
    echo -e " ${GREEN}OK${NC}"
    return 0
}

# Function to run a single test
run_test() {
    local planner=$1
    local run_id=$2
    local output_dir=$3
    
    echo -e "\n${BLUE}----------------------------------------${NC}"
    echo -e "${BLUE}Testing: $planner (Run $run_id)${NC}"
    echo -e "${BLUE}----------------------------------------${NC}"
    
    # Cleanup any previous runs
    cleanup
    
    # Start Gazebo world (background)
    echo -e "\n${YELLOW}Starting Gazebo simulation...${NC}"
    ros2 launch delivery_robot gazebo_world.launch.py &
    GAZEBO_PID=$!
    sleep 10
    
    # Spawn robot (background)
    echo -e "${YELLOW}Spawning TurtleBot3 robot...${NC}"
    ros2 launch delivery_robot spawn_robot.launch.py &
    SPAWN_PID=$!
    sleep 5
    
    # Start sensor bridges
    echo -e "${YELLOW}Starting sensor bridges...${NC}"
    ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
    ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &
    sleep 2
    
    # Start TF publisher
    echo -e "${YELLOW}Starting TF publishers...${NC}"
    ros2 run delivery_robot odom_to_tf &
    sleep 2
    
    # Verify topics
    echo -e "${YELLOW}Verifying topics...${NC}"
    if ! wait_for_topic "/scan" 20; then
        echo -e "${RED}ERROR: /scan topic not available${NC}"
        cleanup
        return 1
    fi
    if ! wait_for_topic "/odom" 20; then
        echo -e "${RED}ERROR: /odom topic not available${NC}"
        cleanup
        return 1
    fi
    
    # Start Nav2 with specified planner
    echo -e "${YELLOW}Starting Nav2 with $planner planner...${NC}"
    ros2 launch delivery_robot nav2_bringup.launch.py planner:=$planner &
    NAV2_PID=$!
    sleep 15  # Nav2 needs time to initialize
    
    # Run waypoint navigator with metrics
    echo -e "${YELLOW}Running waypoint navigation...${NC}"
    ros2 run delivery_robot waypoint_navigator \
        --ros-args \
        -p planner_type:=$planner \
        -p output_dir:=$output_dir \
        -p run_id:=run_$run_id \
        -p use_sim_time:=true
    
    local nav_result=$?
    
    # Cleanup
    cleanup
    
    if [ $nav_result -eq 0 ]; then
        echo -e "${GREEN}Test $planner run $run_id: SUCCESS${NC}"
        return 0
    else
        echo -e "${RED}Test $planner run $run_id: FAILED${NC}"
        return 1
    fi
}

# Function to generate summary report
generate_report() {
    local report_file="$RESULTS_DIR/comparison_report.txt"
    
    echo -e "\n${BLUE}============================================${NC}"
    echo -e "${BLUE}  Generating Comparison Report${NC}"
    echo -e "${BLUE}============================================${NC}"
    
    cat > "$report_file" << EOF
================================================================================
          NAVIGATION PLANNER COMPARISON REPORT
================================================================================
Generated: $(date)
Number of runs per planner: $NUM_RUNS

================================================================================
NavFn Results:
================================================================================
EOF
    
    # Aggregate NavFn results
    if ls "$RESULTS_DIR/navfn/"*.csv 1> /dev/null 2>&1; then
        cat "$RESULTS_DIR/navfn/"*.csv >> "$report_file"
    else
        echo "No NavFn results found" >> "$report_file"
    fi
    
    cat >> "$report_file" << EOF

================================================================================
SmacPlanner2D Results:
================================================================================
EOF
    
    # Aggregate SmacPlanner2D results
    if ls "$RESULTS_DIR/smac/"*.csv 1> /dev/null 2>&1; then
        cat "$RESULTS_DIR/smac/"*.csv >> "$report_file"
    else
        echo "No SmacPlanner2D results found" >> "$report_file"
    fi
    
    cat >> "$report_file" << EOF

================================================================================
                         END OF REPORT
================================================================================
EOF
    
    echo -e "${GREEN}Report saved to: $report_file${NC}"
    
    # Display summary
    echo ""
    echo "=========================================="
    echo "           TEST SUMMARY"
    echo "=========================================="
    cat "$report_file"
}

# Trap for cleanup on exit
trap cleanup EXIT

# Initial cleanup
cleanup

# Build the package
echo -e "${YELLOW}Building package...${NC}"
cd "$SCRIPT_DIR"
colcon build --packages-select delivery_robot 2>&1 | tail -5
source install/setup.bash

# Run tests for each planner
NAVFN_SUCCESS=0
SMAC_SUCCESS=0

# Test NavFn
echo -e "\n${BLUE}============================================${NC}"
echo -e "${BLUE}  Testing NavFn Planner${NC}"
echo -e "${BLUE}============================================${NC}"

for i in $(seq 1 $NUM_RUNS); do
    if run_test "navfn" "$i" "$RESULTS_DIR/navfn"; then
        NAVFN_SUCCESS=$((NAVFN_SUCCESS + 1))
    fi
done

# Test SmacPlanner2D
echo -e "\n${BLUE}============================================${NC}"
echo -e "${BLUE}  Testing SmacPlanner2D${NC}"
echo -e "${BLUE}============================================${NC}"

for i in $(seq 1 $NUM_RUNS); do
    if run_test "smac" "$i" "$RESULTS_DIR/smac"; then
        SMAC_SUCCESS=$((SMAC_SUCCESS + 1))
    fi
done

# Generate comparison report
generate_report

# Final summary
echo -e "\n${BLUE}============================================${NC}"
echo -e "${BLUE}  FINAL RESULTS${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  NavFn:         $NAVFN_SUCCESS / $NUM_RUNS successful"
echo -e "  SmacPlanner2D: $SMAC_SUCCESS / $NUM_RUNS successful"
echo -e "  Results saved: $RESULTS_DIR"
echo ""

if [ $NAVFN_SUCCESS -eq $NUM_RUNS ] && [ $SMAC_SUCCESS -eq $NUM_RUNS ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${YELLOW}Some tests failed. Check the report for details.${NC}"
    exit 1
fi

