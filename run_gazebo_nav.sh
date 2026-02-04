#!/bin/bash
# =============================================================================
# Complete Gazebo Navigation Demo with SmacPlanner2D or NavFn
# This script runs the robot in Gazebo and navigates using the planner
# =============================================================================

PLANNER=${1:-smac}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  Gazebo Navigation Demo                                       ║"
echo "║  Planner: $PLANNER                                            ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

# Kill any existing processes
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
sleep 2

# Set environment
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profile.xml"
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || colcon build --packages-select delivery_robot && source install/setup.bash

echo "[1/6] Starting Gazebo..."
gz sim -r delivery_robot/worlds/office_world_simple.sdf &
GZ_PID=$!
sleep 10

echo "[2/6] Spawning robot..."
gz service -s /world/office_world_professional/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"$SCRIPT_DIR/delivery_robot/models/delivery_robot.sdf\" name: \"delivery_robot\" pose: {position: {x: 2.0 y: 1.0 z: 0.05}}"
sleep 3

echo "[3/6] Starting ROS-Gazebo bridges..."
# Clock bridge (Gazebo -> ROS2)
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
sleep 1

# Cmd_vel bridge (ROS2 -> Gazebo) - THIS IS KEY FOR ROBOT MOVEMENT
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
sleep 1

# Odom bridge (Gazebo -> ROS2)
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &
sleep 1

# Scan bridge (Gazebo -> ROS2)  
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
sleep 2

echo "[4/6] Starting TF publisher..."
# Publish static TF for map->odom (identity transform)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
sleep 1

# Dynamic TF from odom (odom->base_link)
ros2 run delivery_robot odom_to_tf --ros-args -p use_sim_time:=false &
sleep 2

echo "[5/6] Starting Nav2 with $PLANNER planner..."
if [ "$PLANNER" == "smac" ]; then
    PARAMS_FILE="delivery_robot/config/nav2_params_smac.yaml"
else
    PARAMS_FILE="delivery_robot/config/nav2_params_navfn.yaml"
fi

ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$PARAMS_FILE \
    use_sim_time:=false &
NAV2_PID=$!
sleep 15

echo "[6/6] Sending navigation goal to Storage (4.0, 7.5)..."
echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  WATCH GAZEBO - Robot should start moving!                    ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: 7.5, z: 0.0}, orientation: {w: 1.0}}}}" \
    --feedback &

echo ""
echo "Navigation in progress... Press Ctrl+C to stop."
echo ""

# Wait for user to stop
wait $GZ_PID

