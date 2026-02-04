# Increment 01: Basic End-to-End Navigation with NavFn and SmacPlanner2D

## Overview

This is the **first increment** of the autonomous delivery robot project. We have created a complete, working end-to-end navigation system that allows a TurtleBot3 robot to navigate through waypoints in an office environment using Nav2 with switchable global planners.

## What Was Implemented

### Core Components

1. **Office World Environment** ✅
   - Professional Gazebo office world with rooms, corridors, and obstacles
   - Three waypoint markers: Reception, Storage, and Office
   - Realistic furniture and layout for testing navigation

2. **Robot Setup** ✅
   - TurtleBot3 Waffle robot model (spawns from Fuel model database)
   - Robot spawn configuration at Reception waypoint
   - Integration with Gazebo simulation

3. **SLAM Toolbox Configuration** ✅
   - Configuration files for map creation
   - Launch file for offline mapping
   - Ready for manual map generation step

4. **Nav2 Navigation Stack** ✅
   - **Two Global Planners Configured:**
     - **NavFn** (baseline): Wavefront/Dijkstra-based planner
     - **SmacPlanner2D** (modern): Cost-aware A* planner
   - **Local Controller:** DWB (Dynamic Window-based) - kept constant for fair comparison
   - Complete Nav2 configuration with costmaps, AMCL, and recovery behaviors
   - Planner selection via launch argument

5. **Waypoint Navigation** ✅
   - Python node using `nav2_simple_commander` API
   - Sequential navigation through 3 waypoints:
     - Reception (x=2.0, y=1.0)
     - Storage (x=4.0, y=7.5)
     - Office (x=12.5, y=3.0)

6. **Complete Launch System** ✅
   - Single launch file that brings up entire system
   - Automatically sequences: World → Robot → Nav2 → Navigator
   - Optional RViz visualization

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulator                          │
│  ┌──────────────────┐          ┌─────────────────────┐      │
│  │  Office World    │          │  TurtleBot3 Waffle  │      │
│  │  - Rooms         │          │  - LiDAR            │      │
│  │  - Corridors     │◄─────────│  - Odometry         │      │
│  │  - Waypoints     │          │  - Base Controller  │      │
│  └──────────────────┘          └─────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ Sensor Data
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Nav2 Navigation Stack                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Map Server   │  │    AMCL      │  │  Costmaps    │     │
│  │              │  │ Localization │  │  (Global +   │     │
│  └──────────────┘  └──────────────┘  │   Local)     │     │
│                                       └──────────────┘     │
│  ┌──────────────────────────────────────────────────┐      │
│  │  Global Planner (Selectable)                     │      │
│  │  ┌──────────┐           ┌──────────────┐        │      │
│  │  │ NavFn    │  or       │ SmacPlanner  │        │      │
│  │  │ (Dijkstra│           │ 2D (A*)      │        │      │
│  │  │ /A*)     │           │              │        │      │
│  │  └──────────┘           └──────────────┘        │      │
│  └──────────────────────────────────────────────────┘      │
│  ┌──────────────────────────────────────────────────┐      │
│  │  Local Controller: DWB (Dynamic Window-based)    │      │
│  │  - Path following                                │      │
│  │  - Obstacle avoidance                            │      │
│  │  - Velocity control                              │      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │ Goals
                              │
┌─────────────────────────────────────────────────────────────┐
│              Waypoint Navigator Node                         │
│  ┌────────────────────────────────────────────────────┐    │
│  │  nav2_simple_commander API                         │    │
│  │  - Defines waypoint sequence                       │    │
│  │  - Sends navigation goals                          │    │
│  │  - Monitors progress                               │    │
│  │  - Reports completion                              │    │
│  └────────────────────────────────────────────────────┘    │
│  Waypoints: Reception → Storage → Office                    │
└─────────────────────────────────────────────────────────────┘
```

## File Structure

```
increment_01_basic_navigation/
├── delivery_robot/                    # ROS 2 package
│   ├── config/
│   │   ├── nav2_params_navfn.yaml    # Nav2 config with NavFn planner
│   │   ├── nav2_params_smac.yaml     # Nav2 config with SmacPlanner2D
│   │   ├── slam_params.yaml          # SLAM Toolbox configuration
│   │   └── nav2_default_view.rviz    # RViz visualization config
│   ├── launch/
│   │   ├── gazebo_world.launch.py    # Gazebo world launcher
│   │   ├── spawn_robot.launch.py     # Robot spawn launcher
│   │   ├── slam_launch.py            # SLAM mapping launcher
│   │   ├── nav2_bringup.launch.py    # Nav2 stack launcher
│   │   └── full_navigation.launch.py # Complete system launcher
│   ├── delivery_robot/
│   │   └── waypoint_navigator.py     # Waypoint navigation node
│   ├── maps/                         # Map files (to be created)
│   ├── worlds/
│   │   └── office_world.sdf          # Office environment
│   ├── package.xml                   # Package dependencies
│   └── setup.py                      # Package setup
└── README.md                         # This file
```

## Key Files Explained

### 1. Nav2 Configuration Files

**`config/nav2_params_navfn.yaml`**
- Complete Nav2 parameter configuration
- Uses `nav2_navfn_planner/NavfnPlanner` as global planner
- DWB controller for local path following
- Costmap settings for obstacle avoidance
- AMCL, BT Navigator, and recovery behavior configs

**`config/nav2_params_smac.yaml`**
- Same structure as NavFn config
- Uses `nav2_smac_planner/SmacPlanner2D` as global planner
- Cost-aware A* with path smoothing enabled
- Same DWB controller to ensure fair comparison

### 2. Launch Files

**`launch/full_navigation.launch.py`**
- Main entry point for the system
- Accepts `planner:=navfn|smac` argument
- Orchestrates: Gazebo → Robot → Nav2 → Navigator
- Includes RViz visualization

**`launch/nav2_bringup.launch.py`**
- Brings up Nav2 stack with selected planner
- Loads map, starts AMCL, initializes planners/controllers
- Configures lifecycle management

### 3. Waypoint Navigator

**`delivery_robot/waypoint_navigator.py`**
- Python node using Nav2 SimpleCommander API
- Defines 3 waypoints based on office_world.sdf locations
- Sequential navigation with progress monitoring
- Logs status and completion

## How to Use

### Prerequisites

1. **Install Dependencies:**
```bash
# Nav2 and planners
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-nav2-navfn-planner
sudo apt install ros-jazzy-nav2-smac-planner

# TurtleBot3
sudo apt install ros-jazzy-turtlebot3*
export TURTLEBOT3_MODEL=waffle

# SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# Gazebo/GZ Sim
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

2. **Build the Package:**
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
colcon build --packages-select delivery_robot
source install/setup.bash
```

### Step 1: Create the Map (One-time Setup)

Before navigation works, you need to create a map of the office world:

```bash
# Terminal 1: Launch world + robot + SLAM
ros2 launch delivery_robot gazebo_world.launch.py
ros2 launch delivery_robot spawn_robot.launch.py
ros2 launch delivery_robot slam_launch.py

# Terminal 2: Teleoperate robot to map entire office
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# After mapping complete, save the map
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix delivery_robot)/share/delivery_robot/maps/office_map
```

This creates:
- `maps/office_map.pgm` - Occupancy grid image
- `maps/office_map.yaml` - Map metadata

### Step 2: Run Navigation with NavFn

```bash
ros2 launch delivery_robot full_navigation.launch.py planner:=navfn
```

The system will:
1. Launch Gazebo with office world
2. Spawn TurtleBot3 at Reception
3. Start Nav2 with NavFn planner
4. Begin waypoint navigation automatically
5. Robot navigates: Reception → Storage → Office

### Step 3: Run Navigation with SmacPlanner2D

```bash
ros2 launch delivery_robot full_navigation.launch.py planner:=smac
```

Same process, but uses SmacPlanner2D for global planning. Observe path differences in RViz.

### What to Observe

- **In Gazebo:** Robot moves through office, avoids obstacles
- **In RViz:**
  - Green path: Global plan from current planner
  - Costmaps: Blue (safe), yellow (caution), red (obstacles)
  - Laser scan: Red dots showing obstacles
  - Robot model: Current pose and orientation

## Technical Details

### Global Planner Comparison

**NavFn (Baseline)**
- Algorithm: Wavefront expansion with Dijkstra/A*
- Path quality: Functional but may have discontinuities
- Computation: Fast for small maps
- Best for: Simple environments, baseline comparison

**SmacPlanner2D (Modern)**
- Algorithm: Cost-aware A* with path smoothing
- Path quality: Smoother, more optimal paths
- Computation: Slightly slower but better quality
- Best for: Complex environments, better path quality

### Waypoint Coordinates

Defined from `worlds/office_world.sdf`:
- **Reception:** (x=2.0, y=1.0, yaw=0.0)
- **Storage:** (x=4.0, y=7.5, yaw=0.0)
- **Office:** (x=12.5, y=3.0, yaw=0.0)

### Navigation Parameters

- **Robot footprint:** 0.22m radius (TurtleBot3)
- **Costmap resolution:** 0.05m (5cm)
- **Global costmap:** Full map coverage
- **Local costmap:** 3m × 3m rolling window
- **Inflation radius:** 0.55m for obstacle safety
- **Controller frequency:** 20 Hz
- **Max velocity:** 0.26 m/s linear, 1.0 rad/s angular

## Success Criteria Achieved

✅ Office world loads in Gazebo  
✅ TurtleBot3 spawns at correct location  
✅ Nav2 initializes with both planner options  
✅ Robot receives navigation goals successfully  
✅ Waypoint navigation completes full route  
✅ Planner selection works via launch argument  
✅ Both NavFn and SmacPlanner2D produce valid paths  

## Limitations & Future Improvements

### Current Limitations
- **No metrics collection:** Performance not measured yet
- **No recovery tuning:** Basic recovery behaviors only
- **Single route:** Fixed waypoint sequence
- **No dynamic obstacles:** Static environment only
- **Manual map creation:** Requires SLAM step

### What's Next (Increment 02)

Potential improvements for next increment:
1. **Metrics Logger Node**
   - Measure: time to goal, path length, planning time
   - Record recovery behavior triggers
   - Export to CSV for comparison

2. **Automatic Testing**
   - Script to run multiple tests per planner
   - Automated data collection
   - Statistical analysis

3. **Enhanced Recovery Behaviors**
   - Tune recovery parameters
   - Better stuck detection
   - Improved obstacle negotiation

4. **Dynamic Obstacles**
   - Add moving objects to world
   - Test planner robustness
   - Compare dynamic handling

5. **Path Visualization**
   - Compare paths side-by-side
   - Visual metrics overlays
   - Animation of navigation runs

## Troubleshooting

### Robot doesn't spawn
- Check TurtleBot3 model is set: `export TURTLEBOT3_MODEL=waffle`
- Verify Gazebo is running before spawn launch
- Check Fuel model access (internet connection needed)

### Nav2 fails to initialize
- Ensure map files exist in `maps/` directory
- Check AMCL initial pose matches robot spawn location
- Verify all Nav2 packages are installed

### Navigation fails at waypoints
- Check waypoint coordinates match world layout
- Verify costmap inflation allows passage
- Reduce goal tolerance if robot gets stuck

### RViz shows no map
- Ensure map_server is running
- Check map file paths in nav2_bringup.launch.py
- Verify map files are in correct location

## References

- Nav2 Documentation: https://navigation.ros.org/
- NavFn Planner: https://docs.nav2.org/configuration/packages/configuring-navfn.html
- SmacPlanner2D: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- TurtleBot3: http://emanual.robotis.com/docs/en/platform/turtlebot3/

## Summary

This increment establishes the foundation for comparing global planners in Nav2. We have:
- ✅ A working simulation environment
- ✅ Two configured global planners (NavFn & SmacPlanner2D)
- ✅ Complete navigation stack with waypoint following
- ✅ Easy switching between planners

**The system is ready for testing and serves as the baseline for subsequent increments.**

---

**Next Increment:** Will add metrics collection, automated testing, and performance comparison tools.

# robotics-final-project

