# Run Instructions - Increment 01

## VERIFIED WORKING SYSTEM

The system has been tested and verified. Follow these steps:

## Quick Test (Verify Everything Works)

### Step 1: Build and Source
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
colcon build --packages-select delivery_robot
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

### Step 2: Launch World + Robot + Bridge
```bash
ros2 launch delivery_robot test_world_robot.launch.py
```

**What happens:**
1. Gazebo opens with office world (5 seconds)
2. TurtleBot3 robot spawns at Reception (5 more seconds)
3. ROS-Gazebo bridge starts (3 more seconds)
4. Odom-to-TF node starts (2 more seconds)
5. Static TF for LiDAR starts

**Verify in another terminal:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
# Should see: /scan, /odom, /cmd_vel, /clock, /tf, /tf_static
```

### Step 3: Activate SLAM (Required for Mapping)
```bash
# In the same terminal:
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

Now you should see `/map` topic available:
```bash
ros2 topic list | grep map
# Should show: /map, /map_metadata
```

### Step 4: Auto-Explore to Create Map
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run delivery_robot auto_explore
```

This moves the robot automatically to create a map.

### Step 5: Save Map
```bash
# Using SLAM service:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'delivery_robot/maps/office_map'}}"
```

Or manually teleoperate:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Step 6: Verify Map Files
```bash
ls -la delivery_robot/maps/
# Should see: office_map.pgm and office_map.yaml
```

## Troubleshooting

### SLAM not activating
```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### No /map topic
- Ensure SLAM is activated (see above)
- Check /scan topic is publishing
- Verify TF transforms are working

### Robot doesn't move
- Check /cmd_vel topic
- Verify Gazebo is running
- Wait for robot to fully spawn (10+ seconds)

### TF not publishing
- The odom_to_tf node converts /odom to TF
- Ensure bridge is running

## Available Nodes

| Node | Purpose |
|------|---------|
| `waypoint_navigator` | Navigate through waypoints |
| `auto_explore` | Automatic exploration for mapping |
| `odom_to_tf` | Convert odometry to TF transforms |
| `metrics_logger` | Log navigation metrics (placeholder) |

## Launch Files

| File | Purpose |
|------|---------|
| `test_world_robot.launch.py` | Complete test: world + robot + bridge + TF |
| `gazebo_world.launch.py` | Just the world |
| `spawn_robot.launch.py` | Just robot spawn |
| `slam_launch.py` | SLAM for mapping |
| `nav2_bringup.launch.py` | Nav2 stack |
| `full_navigation.launch.py` | Complete navigation system |

## Quick Reference Commands

```bash
# Build
colcon build --packages-select delivery_robot

# Source
source install/setup.bash

# Test system
ros2 launch delivery_robot test_world_robot.launch.py

# Activate SLAM
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate

# Auto explore
ros2 run delivery_robot auto_explore

# Manual teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'delivery_robot/maps/office_map'}}"

# Check topics
ros2 topic list

# Check nodes
ros2 node list
```

