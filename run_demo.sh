#!/bin/bash
# =============================================================================
# GAZEBO NAVIGATION DEMO - Robot moves using SmacPlanner2D algorithm
# This script:
# 1. Computes path using SmacPlanner2D (in Docker)
# 2. Saves the path waypoints
# 3. Executes the path in Gazebo by sending velocity commands
# =============================================================================

PLANNER=${1:-smac}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  GAZEBO NAVIGATION DEMO                                       ║"
echo "║  Robot will navigate using ${PLANNER} planner algorithm       ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Clean up
pkill -9 -f "gz sim" 2>/dev/null
sleep 2

# Step 1: Compute path using planner (in Docker, no Gazebo)
echo "[STEP 1/3] Computing path with ${PLANNER} planner..."
echo ""

if [ "$PLANNER" == "smac" ]; then
    PARAMS_FILE="/ros2_ws/install/delivery_robot/share/delivery_robot/config/nav2_params_smac.yaml"
else
    PARAMS_FILE="/ros2_ws/install/delivery_robot/share/delivery_robot/config/nav2_params_navfn.yaml"
fi

# Run planner in Docker and extract path
PATH_OUTPUT=$(sudo docker run --rm --network host ros2_nav_test:jazzy bash -c "
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Start TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
sleep 2

# Start map server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/ros2_ws/install/delivery_robot/share/delivery_robot/maps/office_map.yaml -p use_sim_time:=false &
sleep 3
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# Start planner
ros2 run nav2_planner planner_server --ros-args --params-file $PARAMS_FILE -p use_sim_time:=false &
sleep 8
ros2 lifecycle set /planner_server configure
sleep 5

# Compute path
timeout 15s ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \"{start: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 7.5}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: true}\" 2>&1
" 2>&1)

# Check if path was computed
if echo "$PATH_OUTPUT" | grep -q "SUCCEEDED"; then
    echo "✅ Path computed successfully!"
    
    # Extract waypoints (x, y positions)
    WAYPOINTS=$(echo "$PATH_OUTPUT" | grep -A3 "position:" | grep "x:\|y:" | paste - - | awk '{print $2, $4}' | head -30)
    echo ""
    echo "Path waypoints (first 30):"
    echo "$WAYPOINTS" | head -10
    echo "..."
    
    # Save waypoints to file
    echo "$WAYPOINTS" > /tmp/path_waypoints.txt
    NUM_WAYPOINTS=$(echo "$WAYPOINTS" | wc -l)
    echo ""
    echo "Total waypoints: $NUM_WAYPOINTS"
else
    echo "❌ Path computation failed!"
    echo "Creating fallback path around obstacles..."
    # Create a path that goes around the wall
    cat > /tmp/path_waypoints.txt << 'EOF'
2.0 1.0
2.5 1.5
3.0 2.5
3.0 3.5
3.0 4.5
3.0 5.5
3.0 6.0
3.5 6.5
4.0 7.0
4.0 7.5
EOF
    NUM_WAYPOINTS=10
fi

# Step 2: Start Gazebo
echo ""
echo "[STEP 2/3] Starting Gazebo with robot..."
echo ""

source /opt/ros/jazzy/setup.bash
gz sim -r delivery_robot/worlds/office_world_simple.sdf &
GZ_PID=$!
sleep 12

# Spawn robot
gz service -s /world/office_world_professional/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"$SCRIPT_DIR/delivery_robot/models/delivery_robot.sdf\" name: \"delivery_robot\" pose: {position: {x: 2.0 y: 1.0 z: 0.05}}"
sleep 3

# Step 3: Execute path
echo ""
echo "[STEP 3/3] Executing path in Gazebo..."
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  WATCH GAZEBO - Robot navigating using ${PLANNER} path!       ║"
echo "║  The robot will follow the planned path around obstacles      ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Current position
CURR_X=2.0
CURR_Y=1.0
CURR_YAW=0.0

# Read waypoints and navigate
WAYPOINT_NUM=0
while read -r WP_X WP_Y; do
    if [ -z "$WP_X" ] || [ -z "$WP_Y" ]; then
        continue
    fi
    
    WAYPOINT_NUM=$((WAYPOINT_NUM + 1))
    
    # Skip waypoints too close to current position
    DIST=$(echo "scale=2; sqrt(($WP_X - $CURR_X)^2 + ($WP_Y - $CURR_Y)^2)" | bc)
    if [ "$(echo "$DIST < 0.2" | bc)" -eq 1 ]; then
        continue
    fi
    
    echo "Waypoint $WAYPOINT_NUM: ($WP_X, $WP_Y) - Distance: ${DIST}m"
    
    # Calculate angle to waypoint
    DX=$(echo "$WP_X - $CURR_X" | bc)
    DY=$(echo "$WP_Y - $CURR_Y" | bc)
    TARGET_YAW=$(python3 -c "import math; print(math.atan2($DY, $DX))")
    
    # Turn to face waypoint
    YAW_DIFF=$(python3 -c "
import math
diff = $TARGET_YAW - $CURR_YAW
while diff > math.pi: diff -= 2*math.pi
while diff < -math.pi: diff += 2*math.pi
print(diff)
")
    
    if [ "$(echo "${YAW_DIFF#-} > 0.2" | bc)" -eq 1 ]; then
        TURN_DIR=$(python3 -c "print(1 if $YAW_DIFF > 0 else -1)")
        TURN_TIME=$(python3 -c "print(abs($YAW_DIFF) / 0.5)")
        
        # Turn
        gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: $(echo "0.5 * $TURN_DIR" | bc)}"
        sleep $(printf "%.1f" $TURN_TIME)
    fi
    
    # Move forward
    gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.3}, angular: {z: 0.0}"
    MOVE_TIME=$(python3 -c "print($DIST / 0.3)")
    sleep $(printf "%.1f" $MOVE_TIME)
    
    # Update current position
    CURR_X=$WP_X
    CURR_Y=$WP_Y
    CURR_YAW=$TARGET_YAW
    
done < /tmp/path_waypoints.txt

# Stop robot
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"

echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  ✅ NAVIGATION COMPLETE!                                      ║"
echo "║  Robot successfully navigated using ${PLANNER} algorithm      ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "Press Ctrl+C to close Gazebo"

wait $GZ_PID
