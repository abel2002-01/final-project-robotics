#!/bin/bash
#
# Complete End-to-End Navigation Test
# Tests the full delivery robot navigation system
#
# Usage: ./run_full_test.sh [navfn|smac]
#

PLANNER=${1:-navfn}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/test_results/$(date +%Y%m%d_%H%M%S)"

echo "============================================"
echo "  Full Navigation Test - $PLANNER Planner"
echo "============================================"
echo "  Output: $OUTPUT_DIR"
echo ""

# Setup environment
source /opt/ros/jazzy/setup.bash
cd "$SCRIPT_DIR"
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_no_shm.xml"
mkdir -p "$OUTPUT_DIR"

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up all processes..."
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    sleep 3
    echo "Done."
}

trap cleanup EXIT

# Initial cleanup
cleanup

# Step 1: Start Gazebo
echo "[1/6] Starting Gazebo simulation..."
(source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch delivery_robot gazebo_world.launch.py) &
GAZEBO_PID=$!
sleep 30
echo "      Gazebo PID: $GAZEBO_PID"

# Step 2: Spawn robot
echo "[2/6] Spawning delivery robot..."
(source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch delivery_robot spawn_robot.launch.py) &
sleep 20
echo "      Robot spawned"

# Step 3: Setup bridges
echo "[3/6] Setting up sensor bridges..."
(source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan) &
(source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry) &
(source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist) &
(source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock) &
sleep 10
echo "      Bridges running"

# Step 4: TF publisher
echo "[4/6] Starting TF publisher..."
(source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run delivery_robot odom_to_tf --ros-args -p use_sim_time:=true) &
sleep 3
echo "      TF publisher running"

# Step 5: Nav2
echo "[5/6] Starting Nav2 with $PLANNER planner..."
(source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch delivery_robot nav2_bringup.launch.py planner:=$PLANNER) &
NAV2_PID=$!
sleep 60
echo "      Nav2 PID: $NAV2_PID"

# Verify topics
echo ""
echo "Verifying system state..."
source /opt/ros/jazzy/setup.bash
echo "  Topics:"
ros2 topic list 2>/dev/null | grep -E "^/(scan|odom|cmd_vel|map|plan|tf)$" | head -10

echo ""
echo "[6/6] Running waypoint navigation..."
echo "      Route: Reception -> Storage -> Office"
echo ""
echo "============================================"

source /opt/ros/jazzy/setup.bash
source $SCRIPT_DIR/install/setup.bash
ros2 run delivery_robot waypoint_navigator --ros-args -p planner_type:=$PLANNER -p output_dir:=$OUTPUT_DIR -p run_id:=test_001 -p use_sim_time:=true

echo ""
echo "============================================"
echo "  Test Complete!"
echo "============================================"
echo "  Results: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR" 2>/dev/null || echo "  (no results files yet)"
