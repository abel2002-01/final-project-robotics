# Final Testing Guide - Autonomous Delivery Robot

## Project Status: ✅ COMPLETE

This guide explains how to test the complete autonomous delivery robot navigation system
comparing **NavFn** and **SmacPlanner2D** global path planners.

---

## Quick Start

### Prerequisites
```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Navigate to project
cd /home/abel/robo/Final-Project/increment_01_basic_navigation

# Build and source
colcon build --packages-select delivery_robot
source install/setup.bash
```

### Run Full Test (Automated)
```bash
# Test with NavFn planner
./run_full_test.sh navfn

# Test with SmacPlanner2D
./run_full_test.sh smac
```

---

## Manual Step-by-Step Testing

### Terminal 1: Start Gazebo World
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch delivery_robot gazebo_world.launch.py
```

### Terminal 2: Spawn Robot
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch delivery_robot spawn_robot.launch.py
```

### Terminal 3: Start Sensor Bridges
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
```

### Terminal 4: Start TF Publisher
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run delivery_robot odom_to_tf
```

### Terminal 5: Start Nav2 (choose planner)
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# For NavFn planner:
ros2 launch delivery_robot nav2_bringup.launch.py planner:=navfn

# OR for SmacPlanner2D:
ros2 launch delivery_robot nav2_bringup.launch.py planner:=smac
```

### Terminal 6: Run Navigation
```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run delivery_robot waypoint_navigator --ros-args -p planner_type:=navfn
```

---

## What Gets Tested

### Waypoint Route
1. **Reception** (2.0, 1.0) - Start point
2. **Storage** (4.0, 7.5) - Pick up delivery
3. **Office** (12.5, 3.0) - Delivery destination

### Metrics Collected
- Total navigation time
- Distance traveled
- Path length (planned)
- Minimum obstacle distance
- Success/failure for each waypoint

### Output Files
Results are saved in `test_results/` directory:
```
test_results/
├── YYYYMMDD_HHMMSS/
│   ├── navigation_navfn_test_001_*.csv
│   └── navigation_smac_test_001_*.csv
```

---

## Analyze Results

After running tests with both planners:
```bash
python3 analyze_results.py test_results/
```

This generates a comparison report showing:
- Average time per planner
- Path efficiency
- Success rates
- Safety metrics

---

## Project Architecture

```
delivery_robot/
├── config/
│   ├── nav2_params_navfn.yaml    # NavFn planner config
│   └── nav2_params_smac.yaml     # SmacPlanner2D config
├── launch/
│   ├── gazebo_world.launch.py    # Start simulation
│   ├── spawn_robot.launch.py     # Spawn robot
│   └── nav2_bringup.launch.py    # Start Nav2 stack
├── maps/
│   ├── office_map.pgm            # Occupancy grid (400x280)
│   └── office_map.yaml           # Map metadata
├── models/
│   └── delivery_robot.sdf        # Robot model with LiDAR
├── worlds/
│   └── office_world_simple.sdf   # Simplified office environment
└── delivery_robot/
    ├── waypoint_navigator.py     # Navigation with metrics
    ├── metrics_logger.py         # Standalone metrics node
    └── odom_to_tf.py             # TF publisher
```

---

## Key Files Modified

1. **nav2_params_navfn.yaml** - Fixed planner plugin name
2. **nav2_params_smac.yaml** - Fixed planner plugin name  
3. **odom_to_tf.py** - Added static TF for lidar frames
4. **delivery_robot.sdf** - Local robot model (no Fuel dependency)
5. **office_world_simple.sdf** - Self-contained world (no Fuel models)

---

## Troubleshooting

### "Failed to create global planner"
- Check plugin names use `::` not `/` (e.g., `nav2_navfn_planner::NavfnPlanner`)

### AMCL "Message Filter dropping message"
- Ensure TF tree is complete: map → odom → base_link → lidar_link
- Run `ros2 run tf2_tools view_frames` to debug

### "Fuel model download failed"
- Using local world (`office_world_simple.sdf`) avoids this
- Robot model is also local (`delivery_robot.sdf`)

### Nav2 nodes not starting
- Check lifecycle manager output
- Ensure map file exists and loads correctly

---

## Success Criteria

✅ **Gazebo world loads** - Office environment visible  
✅ **Robot spawns** - Delivery robot at Reception (2, 1)  
✅ **Sensors work** - /scan and /odom topics publishing  
✅ **TF tree complete** - All frames connected  
✅ **Nav2 initializes** - All lifecycle nodes active  
✅ **Navigation completes** - Robot visits all waypoints  
✅ **Metrics collected** - CSV files generated  

---

## Contact

For issues with this project, check the implementation notes in:
- `IMPLEMENTATION_NOTES.md`
- `README.md`
- `HOW_TO_TEST.md`

