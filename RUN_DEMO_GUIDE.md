# 🤖 Running the SmacPlanner2D Navigation Demo

## Quick Start

To see the robot navigate using **SmacPlanner2D**, run:

```bash
cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_demo.sh smac
```

## What You'll See

### 1. **Gazebo Window Opens** (Step 1)
- Office environment with walls, furniture, and obstacles
- Robot spawns at **Reception** (x=2, y=1) with a red delivery box on top

### 2. **Navigation Stack Starts** (Step 4)
- Map loads successfully
- AMCL localizes the robot
- Nav2 becomes active with SmacPlanner2D planner

### 3. **Robot Navigation** (Step 5)
Watch the robot navigate through 3 waypoints:

1. **Reception** → Robot starts here (already at waypoint)
2. **Storage** (x=4, y=7.5) → Robot navigates through corridors
3. **Office** (x=12.5, y=3) → Final destination

### 4. **What to Observe**

**In Gazebo:**
- Robot moves smoothly through the office
- Red delivery box stays on top of the robot
- Robot avoids obstacles (walls, furniture)
- Robot follows the planned path

**In Terminal:**
- Progress updates every 2 seconds
- Distance remaining decreases as robot approaches goal
- Success/failure messages for each waypoint
- Final metrics summary

## Expected Output

```
[INFO] [waypoint_navigator]: Starting navigation: Reception -> Storage -> Office
[INFO] [waypoint_navigator]: [Waypoint 1/3] Navigating to Reception...
[INFO] [waypoint_navigator]:   ✓ Reception reached in 1.0s
[INFO] [waypoint_navigator]: [Waypoint 2/3] Navigating to Storage...
[INFO] [waypoint_navigator]:   Distance remaining: 8.38m
[INFO] [waypoint_navigator]:   Distance remaining: 7.96m
...
[INFO] [waypoint_navigator]:   ✓ Storage reached in 45.2s
[INFO] [waypoint_navigator]: [Waypoint 3/3] Navigating to Office...
[INFO] [waypoint_navigator]:   ✓ Office reached in 38.7s
```

## Results Location

After completion, results are saved to:
```
test_results/demo_YYYYMMDD_HHMMSS/
├── navigation_smac_demo_HHMMSS_YYYYMMDD_HHMMSS.csv
└── navigation_smac_demo_HHMMSS_YYYYMMDD_HHMMSS.json
```

## Troubleshooting

### If robot doesn't move:
1. Wait 15-20 seconds for AMCL to localize
2. Check Gazebo window is visible
3. Verify no errors in terminal (warnings are OK)

### If navigation fails:
- Check that map loaded successfully
- Verify robot spawned correctly
- Ensure all bridges are active

### Common Warnings (Non-Fatal):
- `TF_OLD_DATA` warnings → Normal, will clear as system syncs
- `Message Filter dropping message` → Normal, old messages being filtered

## Key Features of SmacPlanner2D

- **Cost-aware planning**: Considers obstacle costs, not just distance
- **Optimized A* search**: More efficient than NavFn
- **Smoother paths**: Better path quality for differential-drive robots
- **Faster planning**: Typically faster than NavFn for complex environments

## Demo Duration

- **Total time**: ~2-3 minutes
- **Waypoint 1 (Reception)**: ~1 second (already there)
- **Waypoint 2 (Storage)**: ~30-60 seconds
- **Waypoint 3 (Office)**: ~30-60 seconds

## Next Steps

After the demo completes:
1. Check the JSON/CSV files for detailed metrics
2. Compare with NavFn results using `analyze_and_report.py`
3. Run multiple tests for statistical analysis

---

**Enjoy watching your delivery robot navigate! 🚀**
