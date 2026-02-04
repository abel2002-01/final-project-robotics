#!/bin/bash
#===============================================================================
# Automated Navigation Planner Comparison Tests
# 
# Runs multiple test iterations for both NavFn and SmacPlanner2D planners,
# then generates comprehensive comparison reports.
#
# Usage:
#   ./run_comparison_tests.sh [num_runs_per_planner]
#
# Default: 3 runs per planner
#===============================================================================

set -e

# Configuration
NUM_RUNS=${1:-3}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_DIR="$SCRIPT_DIR/test_results/comparison_$TIMESTAMP"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║         📊 NAVIGATION PLANNER COMPARISON TEST SUITE 📊              ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BLUE}Configuration:${NC}"
echo "   Runs per planner: $NUM_RUNS"
echo "   Total runs: $((NUM_RUNS * 2))"
echo "   Results directory: $RESULTS_DIR"
echo ""

# Create results directory
mkdir -p "$RESULTS_DIR"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_no_shm.xml"

# Initialize counters
NAVFN_SUCCESS=0
NAVFN_FAIL=0
SMAC_SUCCESS=0
SMAC_FAIL=0

# Function to run a single test
run_test() {
    local PLANNER=$1
    local RUN_NUM=$2
    local RUN_ID="${PLANNER}_run$(printf '%02d' $RUN_NUM)"
    
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}  Running: $PLANNER test #$RUN_NUM${NC}"
    echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Use run_full_test.sh if available
    if [[ -f "$SCRIPT_DIR/run_full_test.sh" ]]; then
        bash "$SCRIPT_DIR/run_full_test.sh" "$PLANNER" "$RESULTS_DIR" "$RUN_ID"
        return $?
    fi
    
    # Fallback: manual test execution
    source /opt/ros/jazzy/setup.bash
    cd "$SCRIPT_DIR"
    source install/setup.bash
    
    # Clean up any previous processes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    sleep 3
    
    # Start Gazebo
    ros2 launch delivery_robot gazebo_world.launch.py &
    sleep 10
    
    # Spawn robot
    ros2 launch delivery_robot spawn_robot.launch.py &
    sleep 5
    
    # Start bridges
    ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
    ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &
    ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
    ros2 run delivery_robot odom_to_tf --ros-args -p use_sim_time:=true &
    sleep 3
    
    # Start Nav2
    ros2 launch delivery_robot nav2_bringup.launch.py planner:=$PLANNER &
    sleep 15
    
    # Run navigation test
    ros2 run delivery_robot waypoint_navigator \
        --ros-args \
        -p planner_type:=$PLANNER \
        -p output_dir:=$RESULTS_DIR \
        -p run_id:=$RUN_ID
    
    local RESULT=$?
    
    # Cleanup
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    sleep 5
    
    return $RESULT
}

# Run NavFn tests
echo -e "\n${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  PHASE 1: NavFn Planner Tests ($NUM_RUNS runs)${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}\n"

for i in $(seq 1 $NUM_RUNS); do
    if run_test "navfn" $i; then
        NAVFN_SUCCESS=$((NAVFN_SUCCESS + 1))
        echo -e "${GREEN}✓ NavFn run #$i completed successfully${NC}"
    else
        NAVFN_FAIL=$((NAVFN_FAIL + 1))
        echo -e "${RED}✗ NavFn run #$i failed${NC}"
    fi
    sleep 5
done

# Run SmacPlanner2D tests
echo -e "\n${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  PHASE 2: SmacPlanner2D Tests ($NUM_RUNS runs)${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}\n"

for i in $(seq 1 $NUM_RUNS); do
    if run_test "smac" $i; then
        SMAC_SUCCESS=$((SMAC_SUCCESS + 1))
        echo -e "${GREEN}✓ SmacPlanner2D run #$i completed successfully${NC}"
    else
        SMAC_FAIL=$((SMAC_FAIL + 1))
        echo -e "${RED}✗ SmacPlanner2D run #$i failed${NC}"
    fi
    sleep 5
done

# Generate analysis report
echo -e "\n${CYAN}═══════════════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}  PHASE 3: Generating Comparison Reports${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════════════${NC}\n"

python3 "$SCRIPT_DIR/analyze_and_report.py" "$RESULTS_DIR"

# Print summary
echo -e "\n${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                    📊 TEST SUITE COMPLETE 📊                         ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BLUE}Test Results Summary:${NC}"
echo "┌────────────────────┬──────────┬──────────┐"
echo "│ Planner            │ Success  │ Failed   │"
echo "├────────────────────┼──────────┼──────────┤"
printf "│ NavFn              │ %8d │ %8d │\n" $NAVFN_SUCCESS $NAVFN_FAIL
printf "│ SmacPlanner2D      │ %8d │ %8d │\n" $SMAC_SUCCESS $SMAC_FAIL
echo "└────────────────────┴──────────┴──────────┘"
echo ""

echo -e "${GREEN}📁 Results saved to: $RESULTS_DIR${NC}"
echo ""
echo "Files generated:"
ls -la "$RESULTS_DIR"/*.csv "$RESULTS_DIR"/*.json "$RESULTS_DIR"/*.md 2>/dev/null || echo "   (checking for files...)"

echo ""
echo -e "${YELLOW}View the comparison report:${NC}"
echo "   cat $RESULTS_DIR/comparison_report_*.md | less"

