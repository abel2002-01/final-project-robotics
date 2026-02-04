# Incremental Testing Guide

This guide helps you test the navigation system step-by-step to identify what works and what doesn't.

## Prerequisites

```bash
# Ensure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Build the workspace (first time or after changes)
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
colcon build --packages-select delivery_robot
source install/setup.bash
```

---

## Test 1: Gazebo World Loads

**Goal**: Verify the simulation environment loads correctly.

```bash
./test_step1_world.sh
```

**Expected Result**:
- ✅ Gazebo window opens
- ✅ Office world appears with walls, desks
- ✅ Three colored waypoint markers visible:
  - GREEN circle at (2, 1) = Reception
  - ORANGE circle at (4, 7.5) = Storage  
  - BLUE circle at (12.5, 3) = Office

**If this fails**: Check Gazebo installation and world file exists.

---

## Test 2: Robot Spawns

**Goal**: Verify the robot spawns correctly in the world.

```bash
./test_step2_robot_spawn.sh
```

**Expected Result**:
- ✅ Robot appears at the green Reception marker
- ✅ Robot has a blue body with brown "package" on top
- ✅ Robot has wheels and red LiDAR sensor

**If this fails**: Check robot model file and spawn command.

---

## Test 3: ROS Topics Working

**Goal**: Verify sensor data is being published to ROS2.

**Terminal 1**: Run step 2 (robot spawn)
```bash
./test_step2_robot_spawn.sh
```

**Terminal 2**: Check topics
```bash
./test_step3_topics.sh
```

**Expected Result**:
- ✅ `/odom` topic exists and publishing
- ✅ `/scan` topic exists and publishing
- ✅ `/cmd_vel` topic exists
- ✅ TF tree has odom → base_link

**If this fails**: The ros_gz_bridge isn't running. This is the key issue!

---

## Test 4: Full Simulation with Bridges

**Goal**: Test the complete simulation stack with proper bridge setup.

```bash
./test_step4_nav_simple.sh
```

This uses `simulation_bringup.launch.py` which includes:
- Clock bridge
- Sensor bridges (/odom, /scan, /cmd_vel)
- odom_to_tf node
- Nav2 stack

**Expected Result**:
- ✅ Gazebo opens with robot at Reception
- ✅ After ~20 seconds, Nav2 should be active
- ✅ Robot should be localized on the map

**To send a test navigation goal** (in another terminal):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Navigate to Storage (x=4, y=7.5)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 4.0, y: 7.5}}}}'
```

---

## Test 5: Waypoint Navigation

**Goal**: Run the full waypoint sequence.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Use simulation_bringup instead of full_navigation
ros2 launch delivery_robot simulation_bringup.launch.py planner:=smac
```

**In another terminal**, after Nav2 is ready (~20 seconds):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run delivery_robot waypoint_navigator --ros-args -p use_sim_time:=true
```

---

## Understanding the Issues

### Issue 1: full_navigation.launch.py vs simulation_bringup.launch.py

**`full_navigation.launch.py`** is MISSING:
- Clock bridge (needed for sim time)
- Sensor bridges (ros_gz_bridge)
- odom_to_tf node

**`simulation_bringup.launch.py`** has all these, so it's more complete.

### Issue 2: Topic Naming

The bridge config (`ros_gz_bridge.yaml`) maps:
- Gazebo `/odom` → ROS `/odom_gz`
- Gazebo `/scan` → ROS `/scan_gz`

But the code expects `/odom` and `/scan`!

The `simulation_bringup.launch.py` correctly bridges without renaming.

### Issue 3: Metrics Not Captured

The `distance_traveled_m: 0.0` in test results means the odometry callback isn't receiving data. This happens because:
1. No bridge running, OR
2. Bridge uses wrong topic names

---

## Quick Fix for full_navigation.launch.py

To fix `full_navigation.launch.py`, add these nodes after the robot spawn:

```python
# Clock bridge
clock_bridge = TimerAction(
    period=4.0,
    actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    ]
)

# Sensor bridges
sensor_bridges = TimerAction(
    period=4.5,
    actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scan_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge', 
            arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        ),
    ]
)

# TF publisher
odom_tf = TimerAction(
    period=4.5,
    actions=[
        Node(
            package='delivery_robot',
            executable='odom_to_tf',
            name='odom_to_tf',
            parameters=[{'use_sim_time': True}],
        )
    ]
)
```

---

## Recommended Testing Order

1. ✅ Test 1: World loads
2. ✅ Test 2: Robot spawns
3. ✅ Test 3: Topics work (with manual bridge)
4. ✅ Test 4: Nav2 with simulation_bringup.launch.py
5. ✅ Test 5: Full waypoint navigation

