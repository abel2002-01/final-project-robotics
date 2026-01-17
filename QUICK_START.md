# Quick Start Guide - Increment 01

## 🚀 Fast Track Setup (5 Minutes)

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-smac-planner
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
export TURTLEBOT3_MODEL=waffle
```

### 2. Build Package
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
colcon build --packages-select delivery_robot
source install/setup.bash
```

### 3. Create Map (First Time Only)
```bash
# Terminal 1: Start simulation
ros2 launch delivery_robot gazebo_world.launch.py
ros2 launch delivery_robot spawn_robot.launch.py
ros2 launch delivery_robot slam_launch.py

# Terminal 2: Teleoperate robot to map world
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Save map when done
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix delivery_robot)/share/delivery_robot/maps/office_map
```

### 4. Run Navigation
```bash
# With NavFn planner
ros2 launch delivery_robot full_navigation.launch.py planner:=navfn

# OR with SmacPlanner2D planner
ros2 launch delivery_robot full_navigation.launch.py planner:=smac
```

## 📋 What Happens

1. Gazebo loads office world (3 seconds)
2. TurtleBot3 spawns at Reception (3 seconds)
3. Nav2 initializes with selected planner (5 seconds)
4. Robot automatically navigates: Reception → Storage → Office
5. RViz shows visualization (optional)

## 🔍 Verify It's Working

- **Gazebo:** Robot visible in office world
- **RViz:** Map, robot, path plan visible
- **Console:** Waypoint progress messages
- **Robot:** Moves autonomously through waypoints

## ❗ Common Issues

**Map not found?**
- Run Step 3 (Create Map) first

**Robot doesn't spawn?**
- Check: `echo $TURTLEBOT3_MODEL` (should be "waffle")
- Wait 3-5 seconds after Gazebo starts

**Nav2 doesn't initialize?**
- Ensure map files exist in `maps/` directory
- Check initial pose matches robot spawn location

## 📚 More Information

- **Full Documentation:** See `README.md`
- **Technical Details:** See `IMPLEMENTATION_NOTES.md`
- **Summary:** See `INCREMENT_SUMMARY.md`

## 🎯 Next Steps

After successful navigation:
1. Compare paths in RViz (NavFn vs SmacPlanner2D)
2. Observe path smoothness differences
3. Note any navigation failures or recoveries
4. Ready for Increment 02: Metrics Collection

