# How to Test and Verify Increment 01

## ✅ Verification Results

**All checks PASSED!** The package has been verified and is ready to use.

```
✓ Package structure: OK
✓ Python syntax: OK  
✓ Build: OK
✓ Installation: OK
✓ All required files present
✓ All dependencies installed
✓ Launch files valid
```

## 🚀 Quick Test Instructions

### Step 1: Verify Package (Automated)

Run the verification script:

```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
bash verify_and_test.sh
```

**Expected Output:**
- All file structure checks: ✓
- Python syntax checks: ✓
- ROS 2 dependencies: ✓
- Package build: ✓
- Installation: ✓

### Step 2: Build the Package

```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build package
colcon build --packages-select delivery_robot

# Source workspace
source install/setup.bash
```

**Verify Build:**
```bash
# Check package is discoverable
ros2 pkg list | grep delivery_robot

# Check launch file
ros2 launch delivery_robot gazebo_world.launch.py --show-args
```

### Step 3: Test Individual Components

#### Test 1: Verify World File
```bash
# Check world file exists and is valid
ls -lh delivery_robot/worlds/office_world.sdf
file delivery_robot/worlds/office_world.sdf
```

#### Test 2: Verify Launch Files Syntax
```bash
# Test each launch file
python3 -c "import ast; ast.parse(open('delivery_robot/launch/gazebo_world.launch.py').read())" && echo "✓ Valid"
python3 -c "import ast; ast.parse(open('delivery_robot/launch/full_navigation.launch.py').read())" && echo "✓ Valid"
```

#### Test 3: Verify Waypoint Navigator Code
```bash
# Check Python syntax
python3 -m py_compile delivery_robot/delivery_robot/waypoint_navigator.py && echo "✓ Syntax valid"
```

#### Test 4: Check Configuration Files
```bash
# List all config files
ls -1 delivery_robot/config/

# Verify YAML structure (basic check)
grep -q "ros__parameters" delivery_robot/config/nav2_params_navfn.yaml && echo "✓ NavFn config OK"
grep -q "ros__parameters" delivery_robot/config/nav2_params_smac.yaml && echo "✓ SmacPlanner config OK"
```

### Step 4: Test Launch File Discovery

```bash
# Source workspace first
source install/setup.bash

# List available launch files
ros2 launch delivery_robot

# Check specific launch file arguments
ros2 launch delivery_robot full_navigation.launch.py --show-args
```

**Expected Arguments:**
```
Arguments:
  'planner': Global planner (navfn or smac) [default: 'navfn']
  'use_sim_time': Use simulation time [default: 'true']
  'use_rviz': Launch RViz [default: 'true']
```

### Step 5: Test Navigation Node (Dry Run)

**Note:** This requires Nav2 to be running, but we can check the node exists:

```bash
# Check executable exists
ls -l install/delivery_robot/lib/delivery_robot/waypoint_navigator

# Check it's in ROS 2 path
which waypoint_navigator  # May not work if not sourced

# Test import (if rclpy available)
python3 -c "import sys; sys.path.insert(0, 'install/delivery_robot/lib'); import delivery_robot.waypoint_navigator" && echo "✓ Module importable"
```

## 🎯 Full System Test (Requires Gazebo)

### Prerequisites

1. **Install Dependencies:**
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-smac-planner
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-ros-gz-sim ros-gz-bridge
export TURTLEBOT3_MODEL=waffle
```

2. **Create Map (One-time):**
```bash
# Terminal 1: Launch world
ros2 launch delivery_robot gazebo_world.launch.py

# Terminal 2: Spawn robot
ros2 launch delivery_robot spawn_robot.launch.py

# Terminal 3: Start SLAM
ros2 launch delivery_robot slam_launch.py

# Terminal 4: Teleoperate to map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# After mapping, save map
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix delivery_robot)/share/delivery_robot/maps/office_map
```

### Test Navigation with NavFn

```bash
# Build and source (if not done)
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
colcon build --packages-select delivery_robot
source install/setup.bash

# Launch full system
ros2 launch delivery_robot full_navigation.launch.py planner:=navfn
```

**What to Observe:**
- ✓ Gazebo launches with office world
- ✓ Robot spawns at Reception waypoint
- ✓ Nav2 initializes
- ✓ RViz shows map and robot
- ✓ Robot navigates: Reception → Storage → Office
- ✓ Console shows waypoint progress

### Test Navigation with SmacPlanner2D

```bash
ros2 launch delivery_robot full_navigation.launch.py planner:=smac
```

**Compare:**
- Path smoothness differences
- Navigation behavior
- Path planning time (in console)

## 📊 Verification Checklist

Use this checklist to verify everything works:

### Structure & Files
- [ ] All 14 required files present
- [ ] Package.xml valid
- [ ] Setup.py valid
- [ ] World file exists (30KB+)

### Code Quality
- [ ] Python syntax valid (waypoint_navigator.py)
- [ ] Launch files syntax valid
- [ ] No import errors
- [ ] Code follows ROS 2 conventions

### Build & Installation
- [ ] Package builds without errors
- [ ] Package is discoverable (`ros2 pkg list`)
- [ ] Launch files discoverable
- [ ] Executables created in install/

### Configuration
- [ ] NavFn config file valid
- [ ] SmacPlanner config file valid
- [ ] SLAM config file valid
- [ ] RViz config file exists

### Dependencies
- [ ] ROS 2 Jazzy installed
- [ ] Nav2 packages installed
- [ ] TurtleBot3 packages available
- [ ] SLAM Toolbox available

## 🔍 Debugging Tips

### If Package Doesn't Build
```bash
# Clean and rebuild
rm -rf build install log
colcon build --packages-select delivery_robot
```

### If Launch File Not Found
```bash
# Ensure workspace is sourced
source install/setup.bash
echo $ROS_PACKAGE_PATH | grep delivery_robot
```

### If Dependencies Missing
```bash
# Check what's installed
ros2 pkg list | grep nav2
ros2 pkg list | grep turtlebot3

# Install missing packages
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### If Node Doesn't Run
```bash
# Check executable exists
ls -l install/delivery_robot/lib/delivery_robot/

# Check Python path
python3 -c "import sys; print(sys.path)"
```

## ✅ Success Criteria

**Package is verified when:**
1. ✓ Verification script passes all checks
2. ✓ Package builds without errors
3. ✓ All files are present and valid
4. ✓ Launch files can be discovered
5. ✓ Dependencies are installed

**System is ready when:**
1. ✓ Map file exists (for full testing)
2. ✓ Gazebo can launch world
3. ✓ Robot can spawn
4. ✓ Nav2 can initialize
5. ✓ Navigation can complete waypoint sequence

## 📝 Test Results Template

```markdown
## Test Date: [DATE]

### Environment
- ROS 2 Version: Jazzy
- OS: Ubuntu 24.04
- Gazebo: [Version]

### Build Test
- [ ] Package builds: PASS/FAIL
- [ ] Install successful: PASS/FAIL

### Component Tests
- [ ] Launch file discovery: PASS/FAIL
- [ ] Config file validation: PASS/FAIL
- [ ] Code syntax: PASS/FAIL

### Runtime Tests
- [ ] World launches: PASS/FAIL
- [ ] Robot spawns: PASS/FAIL
- [ ] Nav2 initializes: PASS/FAIL
- [ ] Waypoint navigation: PASS/FAIL

### Notes
[Any issues or observations]
```

## 🎓 Learning Outcomes

After running these tests, you should:
- Understand the package structure
- Know how to build ROS 2 packages
- Be able to verify launch files
- Know how to test navigation components
- Understand the testing workflow

---

**Next:** Once verified, proceed to create the map and test full navigation!


