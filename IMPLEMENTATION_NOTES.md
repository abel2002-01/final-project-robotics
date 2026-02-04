# Implementation Notes - Increment 01

## Design Decisions

### Why Two Separate Config Files?

We created separate Nav2 config files (`nav2_params_navfn.yaml` and `nav2_params_smac.yaml`) instead of using a single file with conditional parameters because:
- **Clarity:** Each planner has specific parameters that differ
- **Maintainability:** Easier to tune each planner independently
- **Comparison:** Ensures identical settings except for the planner plugin
- **ROS 2 Patterns:** Aligns with standard Nav2 configuration practices

### Why DWB Controller?

The DWB (Dynamic Window-based) controller is used for both planner configs because:
- **Fair Comparison:** Same local controller ensures differences are from global planning
- **Standard Default:** DWB is Nav2's default and well-tested
- **Sufficient for Now:** Meets requirements for basic navigation
- **Future Extensibility:** Can be swapped for MPPI in later increments

### Waypoint Definition Approach

Waypoints are hardcoded in the navigator node based on world file coordinates:
- **Simplicity:** No need for parameter files or services yet
- **Reliability:** Direct coordinate mapping from world to navigation
- **Clarity:** Easy to see and modify waypoint sequence
- **Future:** Can be parameterized in next increments

### Launch File Sequencing

The `full_navigation.launch.py` uses TimerActions to sequence startup:
- **Gazebo World:** Starts immediately
- **Robot Spawn:** 3 second delay (lets Gazebo initialize)
- **Nav2:** 5 second delay (lets robot sensors initialize)
- **Navigator:** 8 second delay (lets Nav2 fully initialize)

**Why?** Prevents race conditions where nodes start before dependencies are ready.

## Implementation Challenges & Solutions

### Challenge 1: Robot Spawn Method

**Problem:** ros_gz_sim uses different spawn mechanisms than classic Gazebo.

**Solution:** Used `gz service` command with EntityFactory service. This works with Fuel models directly from URLs.

**Future Improvement:** Could use ros_gz_sim's create node if better integration is needed.

### Challenge 2: Nav2 Config Complexity

**Problem:** Nav2 requires extensive parameter configuration across many components.

**Solution:** Used complete parameter sets based on Nav2 documentation. Both configs share identical parameters except for the planner plugin.

**Key Parameters:**
- Costmaps: Same inflation and resolution for both
- AMCL: Same localization settings
- Controller: Same DWB settings for fair comparison
- BT Navigator: Same behavior tree configuration

### Challenge 3: Waypoint Navigator API

**Problem:** nav2_simple_commander API requires proper pose initialization and feedback handling.

**Solution:**
- Set initial pose before navigation starts
- Use `followWaypoints()` for sequential navigation
- Monitor feedback for progress reporting
- Handle all TaskResult states (SUCCESS, FAILED, CANCELED)

### Challenge 4: Map Dependency

**Problem:** Navigation requires a map, but we don't have one yet.

**Solution:** 
- Created SLAM launch files for map generation
- Documented the mapping process as a prerequisite step
- Map creation is a one-time manual operation

**Note:** This is expected - maps are created through SLAM or manual drawing before navigation.

## Code Organization

### Package Structure

```
delivery_robot/              # ROS 2 ament_python package
├── config/                  # Configuration files (YAML, RViz)
├── launch/                  # Launch files (.launch.py)
├── delivery_robot/          # Python package (for entry points)
│   └── waypoint_navigator.py  # Main navigation node
├── maps/                    # Map files (created via SLAM)
├── worlds/                  # Gazebo world files
└── package.xml              # Package metadata
```

### Entry Points

Defined in `setup.py`:
- `waypoint_navigator` → `delivery_robot.waypoint_navigator:main`
- `metrics_logger` → (placeholder for future increment)

### Launch File Hierarchy

1. **gazebo_world.launch.py**: Lowest level, just world
2. **spawn_robot.launch.py**: Adds robot to world
3. **slam_launch.py**: Independent, for mapping
4. **nav2_bringup.launch.py**: Nav2 stack (requires map)
5. **full_navigation.launch.py**: Top level, orchestrates everything

This hierarchy allows:
- Running components independently for testing
- Reusing launch files in different combinations
- Easy debugging by isolating components

## Testing Strategy

### Manual Testing Steps

1. **World Loading:** Launch Gazebo, verify office world appears
2. **Robot Spawn:** Verify TurtleBot3 appears at Reception waypoint
3. **SLAM:** Test map creation with teleop
4. **Nav2:** Test with RViz "2D Goal Pose" tool
5. **Waypoint Navigation:** Full automated run

### Expected Behaviors

**NavFn:**
- Creates paths that follow grid structure
- May have sharp corners
- Faster computation
- Reliable but less optimal

**SmacPlanner2D:**
- Creates smoother paths
- Better cost-aware routing
- Slightly slower but better quality
- More optimal trajectory

### Debugging Tips

**Check Topics:**
```bash
ros2 topic list
ros2 topic echo /map
ros2 topic echo /odom
ros2 topic echo /plan
```

**Check Services:**
```bash
ros2 service list
ros2 service call /plan_compute_path_to_pose ...
```

**Check TF Tree:**
```bash
ros2 run tf2_tools view_frames
```

## Performance Considerations

### Computational Requirements

- **NavFn:** Lower computational overhead, suitable for embedded systems
- **SmacPlanner2D:** Higher quality but more computation, good for powerful hardware

### Memory Usage

- **Costmaps:** Global (~30m × 30m × 0.05m resolution = ~360KB)
- **Local costmap:** Rolling window (3m × 3m × 0.05m = ~36KB)
- **AMCL:** Particle filter (default: 2000 particles)

### Real-time Performance

- **Controller:** 20 Hz update rate (50ms per cycle)
- **Planner:** On-demand (triggered by goal requests)
- **Costmap updates:** 5 Hz local, 1 Hz global

## Known Issues & Workarounds

### Issue 1: RViz May Not Show Map Initially

**Symptom:** RViz opens but map display is empty.

**Workaround:** 
- Wait a few seconds for map_server to publish
- Or manually set Fixed Frame to "map" in RViz

### Issue 2: Robot May Not Reach Exact Waypoint

**Symptom:** Robot stops near but not exactly at waypoint.

**Explanation:** Goal tolerance is set to 0.25m (default). This is intentional for safety and controller behavior.

**Adjustment:** Can reduce `xy_goal_tolerance` in Nav2 config if needed (but may cause oscillations).

### Issue 3: Spawn Timing Can Be Finicky

**Symptom:** Robot doesn't appear or appears at wrong location.

**Workaround:** 
- Ensure Gazebo is fully loaded before spawning
- Increase TimerAction delay if needed
- Check robot name doesn't conflict with existing entities

## Future Enhancements (Next Increments)

### Increment 02 Priorities

1. **Metrics Collection**
   - Time to goal measurement
   - Path length calculation
   - Planning time logging
   - Success rate tracking

2. **Automated Testing**
   - Script to run N tests per planner
   - Data collection automation
   - Statistical analysis

3. **Recovery Behavior Tuning**
   - Better stuck detection
   - Improved recovery sequences
   - Obstacle negotiation refinement

4. **Visualization Improvements**
   - Side-by-side path comparison
   - Metrics overlays in RViz
   - Run animation/replay

### Increment 03+ Ideas

- Dynamic obstacles
- Multiple route testing
- Controller comparison (MPPI vs DWB)
- Multi-robot scenarios
- Advanced path smoothing
- Predictive planning

## Lessons Learned

1. **Nav2 Configuration is Complex:** Requires understanding of many interconnected components
2. **Timing Matters:** Launch file sequencing is critical for reliable startup
3. **Planner Differences:** NavFn and SmacPlanner2D behave differently even with same settings
4. **Testing Requires Patience:** Full navigation runs take time, need proper test setup
5. **Documentation is Key:** Clear README prevents confusion in future increments

## Dependencies Summary

**ROS 2 Packages:**
- navigation2
- nav2_bringup
- nav2_navfn_planner
- nav2_smac_planner
- nav2_simple_commander
- slam_toolbox
- ros_gz_sim
- turtlebot3 packages

**Python Libraries:**
- rclpy
- nav2_simple_commander
- geometry_msgs

**System Requirements:**
- ROS 2 Jazzy
- Gazebo Garden/Ignition
- Python 3.10+

## Conclusion

Increment 01 establishes a solid foundation with:
- ✅ Working end-to-end navigation
- ✅ Pluggable global planner architecture
- ✅ Clean code organization
- ✅ Comprehensive documentation

**Ready for Increment 02: Metrics and Analysis**


