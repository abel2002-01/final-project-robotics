#!/bin/bash
# Verification and Demo Runner Script
# Verifies setup and provides instructions for running the demo

set -e

cd /home/abel/robo/Final-Project/increment_01_basic_navigation
source /opt/ros/jazzy/setup.bash 2>/dev/null || { echo "ERROR: ROS 2 not found. Please install ROS 2 Jazzy."; exit 1; }
source install/setup.bash 2>/dev/null || { echo "ERROR: Workspace not built. Run 'colcon build' first."; exit 1; }

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║     🤖 SMACPLANNER2D NAVIGATION DEMO - VERIFICATION & RUNNER 🤖    ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

# Verify components
echo "🔍 Verifying setup..."
echo ""

# Check map file
if [ -f "delivery_robot/maps/office_map.pgm" ]; then
    MAP_SIZE=$(stat -c%s "delivery_robot/maps/office_map.pgm")
    if [ "$MAP_SIZE" -gt 100000 ]; then
        echo "  ✓ Map file exists and is valid (${MAP_SIZE} bytes)"
    else
        echo "  ⚠ Map file is too small. Regenerating..."
        python3 generate_map.py
        colcon build --packages-select delivery_robot
        source install/setup.bash
    fi
else
    echo "  ⚠ Map file missing. Generating..."
    python3 generate_map.py
    colcon build --packages-select delivery_robot
    source install/setup.bash
fi

# Check launch files
if [ -f "delivery_robot/launch/nav2_bringup.launch.py" ]; then
    echo "  ✓ Nav2 launch files found"
else
    echo "  ✗ Nav2 launch files missing!"
    exit 1
fi

# Check waypoint navigator
if command -v ros2 &> /dev/null && ros2 pkg executables delivery_robot | grep -q waypoint_navigator; then
    echo "  ✓ Waypoint navigator executable found"
else
    echo "  ✗ Waypoint navigator not found. Rebuilding..."
    colcon build --packages-select delivery_robot
    source install/setup.bash
fi

echo ""
echo "✅ All components verified!"
echo ""

# Instructions
echo "═══════════════════════════════════════════════════════════════════════"
echo "📋 TO RUN THE DEMO:"
echo "═══════════════════════════════════════════════════════════════════════"
echo ""
echo "1. Open a terminal and run:"
echo "   cd /home/abel/robo/Final-Project/increment_01_basic_navigation"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source install/setup.bash"
echo "   ./run_demo.sh smac"
echo ""
echo "2. Watch the Gazebo window - you'll see:"
echo "   • Office environment with walls and furniture"
echo "   • Robot with red delivery box on top"
echo "   • Robot navigating: Reception → Storage → Office"
echo ""
echo "3. In the terminal, you'll see:"
echo "   • Progress updates every 2 seconds"
echo "   • Distance remaining to each waypoint"
echo "   • Success/failure for each waypoint"
echo "   • Final metrics summary"
echo ""
echo "4. Demo duration: ~2-3 minutes"
echo ""
echo "═══════════════════════════════════════════════════════════════════════"
echo "🎯 WHAT TO OBSERVE:"
echo "═══════════════════════════════════════════════════════════════════════"
echo ""
echo "In Gazebo Window:"
echo "  • Robot starts at Reception (bottom-left area)"
echo "  • Moves through corridors to Storage (top area)"
echo "  • Then navigates to Office (right side)"
echo "  • Red delivery box stays on robot throughout"
echo "  • Robot avoids obstacles and follows smooth paths"
echo ""
echo "In Terminal:"
echo "  • 'Distance remaining' decreases as robot approaches goal"
echo "  • '✓ waypoint reached' messages when successful"
echo "  • Final summary with total time, distance, success rate"
echo ""
echo "═══════════════════════════════════════════════════════════════════════"
echo "📊 RESULTS:"
echo "═══════════════════════════════════════════════════════════════════════"
echo ""
echo "After completion, results saved to:"
echo "  test_results/demo_YYYYMMDD_HHMMSS/"
echo "  ├── navigation_smac_demo_*.csv"
echo "  └── navigation_smac_demo_*.json"
echo ""
echo "═══════════════════════════════════════════════════════════════════════"
echo ""

# Ask if user wants to run now
read -p "Would you like to run the demo now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "🚀 Starting demo in 3 seconds..."
    echo "   (Press Ctrl+C to stop)"
    sleep 3
    ./run_demo.sh smac
else
    echo ""
    echo "To run later, execute: ./run_demo.sh smac"
    echo ""
fi




