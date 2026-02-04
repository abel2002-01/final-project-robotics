#!/usr/bin/env python3
"""
Waypoint Navigator Node with Metrics Collection
Navigates robot through a sequence of waypoints using Nav2 SimpleCommander.
Waypoints: Reception -> Storage -> Office

Features:
- Sequential waypoint navigation
- Integrated metrics collection
- CSV and JSON export of results
- Configurable planner selection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math
import os
import csv
import json
from datetime import datetime


class WaypointNavigator(Node):
    """
    Navigates robot through delivery waypoints with metrics collection.
    """
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare parameters
        self.declare_parameter('planner_type', 'navfn')
        self.declare_parameter('output_dir', '/tmp/nav_metrics')
        self.declare_parameter('run_id', 'run_001')
        self.planner_type = self.get_parameter('planner_type').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.run_id = self.get_parameter('run_id').get_parameter_value().string_value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create Nav2 navigator
        self.navigator = BasicNavigator()
        
        # Waypoint poses (from office_world.sdf)
        # Reception: x=2, y=1 (start)
        # Corridor: x=2.2, y=3.5 (Align with gap, avoid Couch/Desk)
        # Gap: x=2.5, y=6.0 (Pass between Bookshelf at 2.0 and Wall at 3.0)
        # Storage: x=2, y=7.5
        # Office: x=12, y=6.5
        self.waypoint_names = ['Reception', 'Corridor', 'Gap', 'Storage', 'Office']
        self.waypoints = [
            self.create_waypoint(2.0, 1.0, 0.0),
            self.create_waypoint(2.2, 3.5, 0.0),
            self.create_waypoint(2.5, 6.0, 0.0),
            self.create_waypoint(2.0, 7.5, 0.0),
            self.create_waypoint(12.0, 6.5, 0.0),
        ]
        
        # Metrics storage
        self.metrics = {
            'planner': self.planner_type,
            'run_id': self.run_id,
            'timestamp': datetime.now().isoformat(),
            'waypoints': [],
            'total_time': 0.0,
            'total_distance': 0.0,
            'total_path_length': 0.0,
            'min_obstacle_distance': float('inf'),
            'success_count': 0,
            'failure_count': 0,
        }
        
        # Tracking variables
        self.current_distance = 0.0
        self.last_odom_pose = None
        self.current_min_obstacle = float('inf')
        self.current_path_length = 0.0
        
        # Subscribe to relevant topics for metrics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        
        self.get_logger().info('Waypoint Navigator initialized')
        self.get_logger().info(f'  Planner: {self.planner_type}')
        self.get_logger().info(f'  Output: {self.output_dir}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
    
    def create_waypoint(self, x, y, yaw):
        """Create a PoseStamped waypoint."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def odom_callback(self, msg):
        """Track distance traveled from odometry."""
        current_pose = msg.pose.pose
        
        if self.last_odom_pose is not None:
            dx = current_pose.position.x - self.last_odom_pose.position.x
            dy = current_pose.position.y - self.last_odom_pose.position.y
            self.current_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_odom_pose = current_pose
    
    def scan_callback(self, msg):
        """Track minimum obstacle distance."""
        valid_ranges = [r for r in msg.ranges 
                       if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.current_min_obstacle:
                self.current_min_obstacle = min_dist
    
    def plan_callback(self, msg):
        """Track path length from planner."""
        if not msg.poses:
            return
        
        path_length = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            path_length += math.sqrt(dx*dx + dy*dy)
        
        self.current_path_length = path_length
    
    def reset_waypoint_metrics(self):
        """Reset metrics for new waypoint."""
        self.current_distance = 0.0
        self.current_min_obstacle = float('inf')
        self.current_path_length = 0.0
    
    def navigate_waypoints(self):
        """Navigate through all waypoints sequentially with metrics."""
        self.get_logger().info('Waiting for Nav2 to become available...')
        
        # Wait for Nav2 to become available
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')
        
        # Set initial pose (same as first waypoint)
        initial_pose = self.waypoints[0]
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f'Set initial pose at {self.waypoint_names[0]}')
        
        # Wait for AMCL to localize the robot
        # AMCL needs time to converge particles and establish a good pose estimate
        self.get_logger().info('Waiting for AMCL to localize robot...')
        localization_timeout = 15.0  # 15 seconds max
        start_time = time.time()
        localized = False
        
        # Try to verify localization by checking if TF transform exists
        import tf2_ros
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self)
        
        while time.time() - start_time < localization_timeout:
            try:
                # Check if map->base_link transform exists (indicates AMCL is working)
                # Use zero time to get latest available transform
                transform = buffer.lookup_transform(
                    'map', 'base_link', 
                    rclpy.time.Time()
                )
                # If we get here, transform exists - check if it's reasonable
                if abs(transform.transform.translation.x) < 100 and abs(transform.transform.translation.y) < 100:
                    localized = True
                    self.get_logger().info('AMCL localized successfully!')
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Transform not available yet, keep waiting
                pass
            time.sleep(0.5)
        
        if not localized:
            self.get_logger().warn('AMCL localization timeout - proceeding anyway (may cause navigation issues)')
        
        # Additional buffer to ensure AMCL particles have converged
        time.sleep(2.0)
        
        # Start mission timer
        mission_start = time.time()
        
        self.get_logger().info(f'Starting navigation: {" -> ".join(self.waypoint_names)}')
        self.get_logger().info('=' * 60)
        
        # Navigate to each waypoint
        for i, (waypoint, name) in enumerate(zip(self.waypoints, self.waypoint_names)):
            self.get_logger().info(f'\n[Waypoint {i+1}/{len(self.waypoints)}] Navigating to {name}...')
            
            # Reset waypoint metrics
            self.reset_waypoint_metrics()
            waypoint_start = time.time()
            
            # Update waypoint timestamp
            waypoint.header.stamp = self.navigator.get_clock().now().to_msg()
            
            # Send goal
            self.navigator.goToPose(waypoint)
            
            # Wait for completion with progress updates
            iteration = 0
            max_iterations = 600  # 60 seconds max per waypoint (600 * 0.1s)
            last_distance = None
            stuck_count = 0
            
            while not self.navigator.isTaskComplete() and iteration < max_iterations:
                feedback = self.navigator.getFeedback()
                if feedback and iteration % 20 == 0:  # Log every 2 seconds
                    if hasattr(feedback, 'distance_remaining'):
                        distance = feedback.distance_remaining
                        self.get_logger().info(
                            f'  Distance remaining: {distance:.2f}m')
                        
                        # Check if robot is stuck (distance not changing)
                        if last_distance is not None:
                            if abs(distance - last_distance) < 0.05:  # Less than 5cm change
                                stuck_count += 1
                                if stuck_count > 10:  # Stuck for 2 seconds
                                    self.get_logger().warn(
                                        f'  ⚠ Robot appears stuck (distance unchanged for {stuck_count*0.2:.1f}s)')
                            else:
                                stuck_count = 0
                        last_distance = distance
                    else:
                        self.get_logger().info(f'  Navigating... ({iteration/10:.0f}s)')
                iteration += 1
                time.sleep(0.1)
            
            if iteration >= max_iterations:
                self.get_logger().warn(f'  ⚠ Navigation timeout after {max_iterations/10:.0f}s')
            
            # Get result
            waypoint_time = time.time() - waypoint_start
            result = self.navigator.getResult()
            
            # Record waypoint metrics
            waypoint_metrics = {
                'name': name,
                'goal_x': waypoint.pose.position.x,
                'goal_y': waypoint.pose.position.y,
                'time_sec': waypoint_time,
                'distance_traveled_m': self.current_distance,
                'path_length_m': self.current_path_length,
                'min_obstacle_m': self.current_min_obstacle if self.current_min_obstacle != float('inf') else None,
                'success': result == TaskResult.SUCCEEDED,
            }
            self.metrics['waypoints'].append(waypoint_metrics)
            
            # Update totals
            self.metrics['total_distance'] += self.current_distance
            self.metrics['total_path_length'] += self.current_path_length
            if self.current_min_obstacle < self.metrics['min_obstacle_distance']:
                self.metrics['min_obstacle_distance'] = self.current_min_obstacle
            
            # Log result
            if result == TaskResult.SUCCEEDED:
                self.metrics['success_count'] += 1
                self.get_logger().info(
                    f'  ✓ {name} reached in {waypoint_time:.2f}s '
                    f'(distance: {self.current_distance:.2f}m, path: {self.current_path_length:.2f}m)')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'  ✗ Navigation to {name} was canceled')
                self.metrics['failure_count'] += 1
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'  ✗ Navigation to {name} FAILED!')
                self.metrics['failure_count'] += 1
            else:
                self.get_logger().warn(f'  ? Unknown result for {name}: {result}')
                self.metrics['failure_count'] += 1
        
        # Record total mission time
        self.metrics['total_time'] = time.time() - mission_start
        
        # Print summary
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('NAVIGATION COMPLETE')
        self.get_logger().info('=' * 60)
        self.print_summary()
        
        # Save metrics
        self.save_metrics()
        
        return self.metrics['success_count'] == len(self.waypoints)
    
    def print_summary(self):
        """Print navigation summary."""
        success_rate = self.metrics['success_count'] / len(self.waypoints) * 100
        
        self.get_logger().info(f"\nPlanner: {self.metrics['planner']}")
        self.get_logger().info(f"Run ID: {self.metrics['run_id']}")
        self.get_logger().info(f"\nResults:")
        self.get_logger().info(f"  Waypoints: {self.metrics['success_count']}/{len(self.waypoints)} ({success_rate:.0f}% success)")
        self.get_logger().info(f"  Total Time: {self.metrics['total_time']:.2f}s")
        self.get_logger().info(f"  Total Distance: {self.metrics['total_distance']:.2f}m")
        self.get_logger().info(f"  Total Path Length: {self.metrics['total_path_length']:.2f}m")
        
        if self.metrics['min_obstacle_distance'] != float('inf'):
            self.get_logger().info(f"  Min Obstacle Distance: {self.metrics['min_obstacle_distance']:.2f}m")
        
        # Efficiency ratio
        if self.metrics['total_path_length'] > 0:
            efficiency = self.metrics['total_distance'] / self.metrics['total_path_length'] * 100
            self.get_logger().info(f"  Path Following Efficiency: {efficiency:.1f}%")
    
    def save_metrics(self):
        """Save metrics to CSV and JSON files."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # File paths
        csv_file = os.path.join(
            self.output_dir,
            f'navigation_{self.planner_type}_{self.run_id}_{timestamp}.csv')
        json_file = os.path.join(
            self.output_dir,
            f'navigation_{self.planner_type}_{self.run_id}_{timestamp}.json')
        
        # Save CSV
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Summary section
            writer.writerow(['=== NAVIGATION METRICS ==='])
            writer.writerow(['planner', self.metrics['planner']])
            writer.writerow(['run_id', self.metrics['run_id']])
            writer.writerow(['timestamp', self.metrics['timestamp']])
            writer.writerow(['total_time_sec', f"{self.metrics['total_time']:.2f}"])
            writer.writerow(['total_distance_m', f"{self.metrics['total_distance']:.2f}"])
            writer.writerow(['total_path_length_m', f"{self.metrics['total_path_length']:.2f}"])
            writer.writerow(['min_obstacle_m', 
                           f"{self.metrics['min_obstacle_distance']:.2f}" 
                           if self.metrics['min_obstacle_distance'] != float('inf') else 'N/A'])
            writer.writerow(['success_count', self.metrics['success_count']])
            writer.writerow(['failure_count', self.metrics['failure_count']])
            writer.writerow(['success_rate', f"{self.metrics['success_count']/len(self.waypoints)*100:.1f}%"])
            writer.writerow([])
            
            # Per-waypoint section
            writer.writerow(['=== WAYPOINT DETAILS ==='])
            writer.writerow(['waypoint', 'goal_x', 'goal_y', 'time_sec', 
                           'distance_m', 'path_length_m', 'min_obstacle_m', 'success'])
            
            for wp in self.metrics['waypoints']:
                writer.writerow([
                    wp['name'],
                    f"{wp['goal_x']:.2f}",
                    f"{wp['goal_y']:.2f}",
                    f"{wp['time_sec']:.2f}",
                    f"{wp['distance_traveled_m']:.2f}",
                    f"{wp['path_length_m']:.2f}",
                    f"{wp['min_obstacle_m']:.2f}" if wp['min_obstacle_m'] else 'N/A',
                    wp['success'],
                ])
        
        # Save JSON
        json_data = {
            'planner': self.metrics['planner'],
            'run_id': self.metrics['run_id'],
            'timestamp': self.metrics['timestamp'],
            'summary': {
                'total_time_sec': round(self.metrics['total_time'], 3),
                'total_distance_m': round(self.metrics['total_distance'], 3),
                'total_path_length_m': round(self.metrics['total_path_length'], 3),
                'min_obstacle_m': round(self.metrics['min_obstacle_distance'], 3) if self.metrics['min_obstacle_distance'] != float('inf') else None,
                'success_count': self.metrics['success_count'],
                'failure_count': self.metrics['failure_count'],
                'waypoints_total': len(self.waypoints),
                'success_rate_percent': round(self.metrics['success_count'] / len(self.waypoints) * 100, 1),
                'path_efficiency_percent': round(self.metrics['total_distance'] / self.metrics['total_path_length'] * 100, 1) if self.metrics['total_path_length'] > 0 else None
            },
            'waypoints': []
        }
        
        for wp in self.metrics['waypoints']:
            json_data['waypoints'].append({
                'name': wp['name'],
                'goal': {'x': wp['goal_x'], 'y': wp['goal_y']},
                'time_sec': round(wp['time_sec'], 3),
                'distance_traveled_m': round(wp['distance_traveled_m'], 3),
                'path_length_m': round(wp['path_length_m'], 3),
                'min_obstacle_m': round(wp['min_obstacle_m'], 3) if wp['min_obstacle_m'] else None,
                'success': wp['success']
            })
        
        with open(json_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        self.get_logger().info(f'\nMetrics saved to:')
        self.get_logger().info(f'  CSV:  {csv_file}')
        self.get_logger().info(f'  JSON: {json_file}')
        return csv_file, json_file


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    navigator_node = WaypointNavigator()
    
    try:
        success = navigator_node.navigate_waypoints()
        if success:
            navigator_node.get_logger().info('\n🎉 Delivery route completed successfully!')
        else:
            navigator_node.get_logger().error('\n❌ Delivery route had failures!')
    except KeyboardInterrupt:
        navigator_node.get_logger().info('Waypoint navigation interrupted by user')
    except Exception as e:
        navigator_node.get_logger().error(f'Error during navigation: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            navigator_node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Already shutdown by BasicNavigator


if __name__ == '__main__':
    main()
