#!/usr/bin/env python3
"""Extract path waypoints from Nav2 planner output."""
import json
import re
import sys

def extract_path(input_file, output_file, planner_name="SmacPlanner2D"):
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Find all position blocks
    positions = re.findall(r'position:\s*\n\s*x:\s*([\d.e+-]+)\s*\n\s*y:\s*([\d.e+-]+)', content)
    
    waypoints = []
    for x_str, y_str in positions:
        x, y = float(x_str), float(y_str)
        # Skip near-zero values (orientation data)
        if abs(x) > 0.001 or abs(y) > 0.001:
            waypoints.append({'x': round(x, 4), 'y': round(y, 4)})
    
    # Sample every 10th waypoint
    sampled = waypoints[::10]
    if waypoints and (not sampled or sampled[-1] != waypoints[-1]):
        sampled.append(waypoints[-1])
    
    path_data = {
        'planner': planner_name,
        'start': {'x': 0.0, 'y': 0.0},
        'goal': {'x': 5.0, 'y': 5.0},
        'total_waypoints': len(waypoints),
        'sampled_waypoints': len(sampled),
        'path': sampled
    }
    
    with open(output_file, 'w') as f:
        json.dump(path_data, f, indent=2)
    
    print(f"Planner: {planner_name}")
    print(f"Total waypoints: {len(waypoints)}")
    print(f"Sampled waypoints: {len(sampled)}")
    print()
    print("Path preview:")
    for i, wp in enumerate(sampled[:15]):
        print(f"  {i+1}: ({wp['x']}, {wp['y']})")
    if len(sampled) > 15:
        print(f"  ... and {len(sampled)-15} more waypoints")
    
    return path_data

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        extract_path(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "SmacPlanner2D")
    else:
        print("Usage: extract_path.py <input_file> <output_file> [planner_name]")

