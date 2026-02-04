#!/usr/bin/env python3
"""
Simple Navigation Test Script

This script demonstrates direct robot navigation without the full Nav2 stack.
It measures navigation metrics and can be used to compare different approaches.

Usage:
  1. Start Gazebo and bridge first (see instructions below)
  2. Run this script: python3 simple_nav_test.py

Prerequisites:
  # Terminal 1: Start Gazebo
  gz sim -r delivery_robot/worlds/office_world_simple.sdf
  
  # Terminal 2: Spawn robot
  gz service -s /world/office_world_professional/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req 'sdf_filename: "delivery_robot/models/delivery_robot.sdf" name: "delivery_robot" pose: {position: {x: 2.0 y: 1.0 z: 0.05}}'
  
  # Terminal 3: Start bridge
  ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
  
  # Terminal 4: Run this script
  python3 simple_nav_test.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import json
from datetime import datetime


class SimpleNavigator(Node):
    """Simple P-controller based navigation to waypoints."""
    
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        # Current robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.min_obstacle = float('inf')
        self.odom_received = False
        
        # Navigation parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.goal_tolerance = 0.3  # meters
        
        # Metrics
        self.distance_traveled = 0.0
        self.last_x = None
        self.last_y = None
        
        self.get_logger().info('Simple Navigator initialized')
    
    def odom_cb(self, msg):
        """Update robot position from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Track distance traveled
        if self.last_x is not None:
            dx = self.x - self.last_x
            dy = self.y - self.last_y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_x = self.x
        self.last_y = self.y
        
        self.odom_received = True
    
    def scan_cb(self, msg):
        """Track minimum obstacle distance."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.min_obstacle:
                self.min_obstacle = min_dist
    
    def wait_for_odom(self, timeout=10.0):
        """Wait for odometry to be received."""
        start = time.time()
        while not self.odom_received and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_received
    
    def navigate_to(self, goal_x, goal_y, timeout=60.0):
        """Navigate to a goal position using simple P-controller."""
        start_time = time.time()
        start_dist = self.distance_traveled
        
        self.get_logger().info(f'Navigating to ({goal_x:.1f}, {goal_y:.1f})...')
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Calculate error
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if we reached the goal
            if distance < self.goal_tolerance:
                self.stop()
                elapsed = time.time() - start_time
                dist = self.distance_traveled - start_dist
                self.get_logger().info(f'  ✓ Reached goal in {elapsed:.1f}s, traveled {dist:.2f}m')
                return True, elapsed, dist
            
            # Calculate desired heading
            desired_yaw = math.atan2(dy, dx)
            yaw_error = desired_yaw - self.yaw
            
            # Normalize yaw error to [-pi, pi]
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            # Create velocity command
            cmd = Twist()
            
            # If we need to turn a lot, turn in place first
            if abs(yaw_error) > 0.3:
                cmd.angular.z = self.angular_speed * (1.0 if yaw_error > 0 else -1.0)
                cmd.linear.x = 0.05  # Slow forward while turning
            else:
                # Drive towards goal
                cmd.linear.x = min(self.linear_speed, distance * 0.5)
                cmd.angular.z = yaw_error * 1.5  # Proportional steering
            
            self.cmd_pub.publish(cmd)
        
        # Timeout
        self.stop()
        elapsed = time.time() - start_time
        dist = self.distance_traveled - start_dist
        self.get_logger().error(f'  ✗ Navigation timeout after {elapsed:.1f}s')
        return False, elapsed, dist
    
    def stop(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    nav = SimpleNavigator()
    
    # Waypoints
    waypoints = [
        ('Reception', 2.0, 1.0),
        ('Storage', 4.0, 7.5),
        ('Office', 12.5, 3.0),
    ]
    
    # Wait for odometry
    print("\n" + "="*60)
    print("SIMPLE NAVIGATION TEST")
    print("="*60)
    print("\nWaiting for odometry...")
    
    if not nav.wait_for_odom():
        print("ERROR: No odometry received! Make sure the bridge is running.")
        nav.destroy_node()
        rclpy.shutdown()
        return
    
    print(f"Robot position: ({nav.x:.2f}, {nav.y:.2f})")
    print("\nStarting navigation through waypoints...")
    print("-"*60)
    
    # Navigate through waypoints
    results = []
    total_time = 0.0
    total_distance = 0.0
    success_count = 0
    
    for name, x, y in waypoints:
        print(f"\n[Waypoint] {name} ({x}, {y})")
        nav.min_obstacle = float('inf')  # Reset for this leg
        
        success, elapsed, distance = nav.navigate_to(x, y)
        
        results.append({
            'name': name,
            'goal': {'x': x, 'y': y},
            'success': success,
            'time_sec': round(elapsed, 2),
            'distance_m': round(distance, 2),
            'min_obstacle_m': round(nav.min_obstacle, 2) if nav.min_obstacle != float('inf') else None,
        })
        
        total_time += elapsed
        total_distance += distance
        if success:
            success_count += 1
    
    # Print summary
    print("\n" + "="*60)
    print("NAVIGATION COMPLETE")
    print("="*60)
    print(f"\nResults:")
    print(f"  Waypoints: {success_count}/{len(waypoints)} ({100*success_count/len(waypoints):.0f}% success)")
    print(f"  Total Time: {total_time:.1f}s")
    print(f"  Total Distance: {total_distance:.2f}m")
    if nav.min_obstacle != float('inf'):
        print(f"  Minimum Obstacle Distance: {nav.min_obstacle:.2f}m")
    
    # Save results
    output = {
        'test_type': 'simple_navigation',
        'timestamp': datetime.now().isoformat(),
        'summary': {
            'success_count': success_count,
            'total_waypoints': len(waypoints),
            'success_rate': round(100*success_count/len(waypoints), 1),
            'total_time_sec': round(total_time, 2),
            'total_distance_m': round(total_distance, 2),
        },
        'waypoints': results,
    }
    
    output_file = f'/tmp/simple_nav_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)
    print(f"\nResults saved to: {output_file}")
    
    # Cleanup
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

