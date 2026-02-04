#!/bin/bash
# =============================================================================
# Docker Gazebo Navigation - See Robot Moving with Planner Algorithm
# =============================================================================

PLANNER=${1:-smac}

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  Docker Gazebo Navigation Demo                                ║"
echo "║  Planner: $PLANNER                                            ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

# Allow X11
xhost +local:docker 2>/dev/null || true

cd /home/abel/robo/Final-Project/increment_01_basic_navigation

# Run in Docker with display
sudo docker run --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/ws:rw \
    ros2_nav_test:jazzy \
    bash -c "
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

echo '=== Starting Gazebo + Nav2 Demo ==='

# Start Gazebo in background
echo '[1/5] Starting Gazebo...'
gz sim -r /ros2_ws/src/delivery_robot_project/delivery_robot/worlds/office_world_simple.sdf &
sleep 12

# Spawn robot
echo '[2/5] Spawning robot...'
gz service -s /world/office_world_professional/create \\
    --reqtype gz.msgs.EntityFactory \\
    --reptype gz.msgs.Boolean \\
    --timeout 5000 \\
    --req 'sdf_filename: \"/ros2_ws/src/delivery_robot_project/delivery_robot/models/delivery_robot.sdf\" name: \"delivery_robot\" pose: {position: {x: 2.0 y: 1.0 z: 0.05}}'
sleep 3

# Start bridges
echo '[3/5] Starting bridges...'
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
sleep 3

# Start TF
echo '[4/5] Starting TF...'
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 2 1 0 0 0 0 odom base_link &
sleep 2

# Start Nav2
echo '[5/5] Starting Nav2 with ${PLANNER} planner...'
if [ '${PLANNER}' == 'smac' ]; then
    ros2 launch nav2_bringup navigation_launch.py \\
        params_file:=/ros2_ws/install/delivery_robot/share/delivery_robot/config/nav2_params_smac.yaml \\
        use_sim_time:=false &
else
    ros2 launch nav2_bringup navigation_launch.py \\
        params_file:=/ros2_ws/install/delivery_robot/share/delivery_robot/config/nav2_params_navfn.yaml \\
        use_sim_time:=false &
fi
sleep 20

echo ''
echo '╔═══════════════════════════════════════════════════════════════╗'
echo '║  SENDING NAVIGATION GOAL - Watch Gazebo!                      ║'
echo '║  Robot will navigate from Reception to Storage                ║'
echo '╚═══════════════════════════════════════════════════════════════╝'
echo ''

# Send navigation goal with feedback
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: 7.5, z: 0.0}, orientation: {w: 1.0}}}}\" \\
    --feedback

echo 'Navigation complete!'
sleep 5
"

