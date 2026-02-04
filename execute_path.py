#!/usr/bin/env python3
"""
Execute a pre-computed path in Gazebo.
Reads waypoints from JSON and sends velocity commands.
"""
import json
import subprocess
import time
import math
import sys

def send_cmd(linear_x, angular_z):
    """Send velocity command to Gazebo."""
    cmd = f'gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}"'
    subprocess.run(cmd, shell=True, capture_output=True)

def execute_path(json_file):
    """Execute path from JSON file."""
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    planner = data['planner']
    path = data['path']
    
    print("=" * 60)
    print(f"  EXECUTING {planner} PATH")
    print(f"  {len(path)} waypoints")
    print("=" * 60)
    print()
    
    # Current assumed position (start)
    curr_x, curr_y = 0.0, 0.0
    curr_yaw = 0.0  # Facing +X direction
    
    # Speed settings
    LINEAR_SPEED = 0.35
    ANGULAR_SPEED = 0.6
    
    for i, wp in enumerate(path):
        target_x = wp['x']
        target_y = wp['y']
        
        # Skip if too close
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.2:
            continue
        
        print(f"Waypoint {i+1}/{len(path)}: ({target_x:.2f}, {target_y:.2f}) - Distance: {distance:.2f}m")
        
        # Calculate required heading
        target_yaw = math.atan2(dy, dx)
        yaw_diff = target_yaw - curr_yaw
        
        # Normalize to [-pi, pi]
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Turn to face waypoint
        if abs(yaw_diff) > 0.2:
            turn_time = abs(yaw_diff) / ANGULAR_SPEED
            turn_dir = 1 if yaw_diff > 0 else -1
            
            print(f"  Turning {'LEFT' if turn_dir > 0 else 'RIGHT'} ({math.degrees(yaw_diff):.0f}°)...")
            
            steps = int(turn_time / 0.3) + 1
            for _ in range(steps):
                send_cmd(0.05, ANGULAR_SPEED * turn_dir)
                time.sleep(0.3)
        
        # Move forward
        move_time = distance / LINEAR_SPEED
        print(f"  Moving forward {distance:.2f}m...")
        
        steps = int(move_time / 0.3) + 1
        for _ in range(steps):
            send_cmd(LINEAR_SPEED, 0.0)
            time.sleep(0.3)
        
        # Update position estimate
        curr_x = target_x
        curr_y = target_y
        curr_yaw = target_yaw
    
    # Stop
    send_cmd(0.0, 0.0)
    
    print()
    print("=" * 60)
    print(f"  ✅ NAVIGATION COMPLETE!")
    print(f"  Robot navigated using {planner}")
    print("=" * 60)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: execute_path.py <path.json>")
        sys.exit(1)
    
    execute_path(sys.argv[1])

