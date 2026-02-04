#!/usr/bin/env python3
"""
Metrics Logger Node - Comprehensive Navigation Performance Tracking

Collects and logs navigation metrics for comparing planners:
- Time to goal (seconds)
- Path length (meters)
- Planning time (milliseconds)
- Minimum obstacle distance (safety metric)
- Number of recovery behaviors triggered
- Success/failure rate

Exports metrics to CSV for analysis.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import math
import os
import csv
from datetime import datetime


class MetricsLogger(Node):
    """
    Comprehensive navigation metrics logger.
    
    Subscribes to:
    - /plan: Global path from planner
    - /odom: Robot odometry for tracking distance
    - /scan: LiDAR for obstacle distance
    - /goal_pose: Navigation goals
    - /navigate_to_pose/_action/status: Goal status
    
    Logs:
    - Per-waypoint metrics
    - Total mission metrics
    - Exports to CSV
    """
    
    def __init__(self):
        super().__init__('metrics_logger')
        
        # Declare parameters
        self.declare_parameter('planner_type', 'unknown')
        self.declare_parameter('output_dir', '/tmp/nav_metrics')
        self.declare_parameter('run_id', 'run_001')
        
        self.planner_type = self.get_parameter('planner_type').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.run_id = self.get_parameter('run_id').get_parameter_value().string_value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize metrics storage
        self.reset_metrics()
        
        # Navigation state
        self.is_navigating = False
        self.current_goal = None
        self.goal_start_time = None
        self.last_odom_pose = None
        self.mission_start_time = None
        self.waypoint_count = 0
        
        # Subscribers
        self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', 
            self.status_callback, 10)
        
        # Publishers for metrics (for external monitoring)
        self.metrics_pub = self.create_publisher(String, '/navigation_metrics', 10)
        
        # Timer for periodic status logging
        self.create_timer(5.0, self.periodic_status)
        
        self.get_logger().info(f'Metrics Logger initialized')
        self.get_logger().info(f'  Planner: {self.planner_type}')
        self.get_logger().info(f'  Output: {self.output_dir}')
        self.get_logger().info(f'  Run ID: {self.run_id}')
    
    def reset_metrics(self):
        """Reset all metric counters."""
        self.metrics = {
            'planner': self.planner_type if hasattr(self, 'planner_type') else 'unknown',
            'run_id': self.run_id if hasattr(self, 'run_id') else 'unknown',
            'timestamp': datetime.now().isoformat(),
            'total_time': 0.0,
            'total_distance': 0.0,
            'total_path_length': 0.0,
            'min_obstacle_distance': float('inf'),
            'recovery_count': 0,
            'waypoints_completed': 0,
            'waypoints_failed': 0,
            'planning_times': [],
            'waypoint_times': [],
            'waypoint_distances': [],
        }
        self.current_waypoint_metrics = {}
    
    def start_waypoint(self, goal_pose):
        """Start tracking metrics for a new waypoint."""
        self.waypoint_count += 1
        self.is_navigating = True
        self.current_goal = goal_pose
        self.goal_start_time = self.get_clock().now()
        
        if self.mission_start_time is None:
            self.mission_start_time = self.goal_start_time
        
        self.current_waypoint_metrics = {
            'waypoint_id': self.waypoint_count,
            'start_time': self.goal_start_time,
            'goal_x': goal_pose.pose.position.x,
            'goal_y': goal_pose.pose.position.y,
            'distance_traveled': 0.0,
            'path_length': 0.0,
            'min_obstacle_dist': float('inf'),
            'planning_time_ms': 0.0,
        }
        
        self.get_logger().info(f'Started waypoint {self.waypoint_count}: '
                              f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
    
    def complete_waypoint(self, success=True):
        """Complete current waypoint and record metrics."""
        if not self.is_navigating:
            return
        
        end_time = self.get_clock().now()
        elapsed = (end_time - self.goal_start_time).nanoseconds / 1e9
        
        self.current_waypoint_metrics['elapsed_time'] = elapsed
        self.current_waypoint_metrics['success'] = success
        
        # Update totals
        self.metrics['waypoint_times'].append(elapsed)
        self.metrics['waypoint_distances'].append(
            self.current_waypoint_metrics['distance_traveled'])
        
        if success:
            self.metrics['waypoints_completed'] += 1
            self.get_logger().info(
                f'Waypoint {self.waypoint_count} completed in {elapsed:.2f}s, '
                f'distance: {self.current_waypoint_metrics["distance_traveled"]:.2f}m')
        else:
            self.metrics['waypoints_failed'] += 1
            self.get_logger().warn(f'Waypoint {self.waypoint_count} FAILED')
        
        # Update minimum obstacle distance
        if self.current_waypoint_metrics['min_obstacle_dist'] < self.metrics['min_obstacle_distance']:
            self.metrics['min_obstacle_distance'] = self.current_waypoint_metrics['min_obstacle_dist']
        
        self.is_navigating = False
        self.current_goal = None
    
    def plan_callback(self, msg):
        """Handle received path plan."""
        if not msg.poses:
            return
        
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            path_length += math.sqrt(dx*dx + dy*dy)
        
        self.current_waypoint_metrics['path_length'] = path_length
        self.metrics['total_path_length'] += path_length
        
        self.get_logger().debug(f'Received path with {len(msg.poses)} poses, '
                               f'length: {path_length:.2f}m')
    
    def odom_callback(self, msg):
        """Track robot movement from odometry."""
        current_pose = msg.pose.pose
        
        if self.last_odom_pose is not None and self.is_navigating:
            dx = current_pose.position.x - self.last_odom_pose.position.x
            dy = current_pose.position.y - self.last_odom_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            self.current_waypoint_metrics['distance_traveled'] += distance
            self.metrics['total_distance'] += distance
        
        self.last_odom_pose = current_pose
    
    def scan_callback(self, msg):
        """Track minimum obstacle distance from LiDAR."""
        if not self.is_navigating:
            return
        
        # Filter valid ranges
        valid_ranges = [r for r in msg.ranges 
                       if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.current_waypoint_metrics['min_obstacle_dist']:
                self.current_waypoint_metrics['min_obstacle_dist'] = min_dist
    
    def goal_callback(self, msg):
        """Handle new navigation goal."""
        self.start_waypoint(msg)
    
    def status_callback(self, msg):
        """Handle navigation status updates."""
        if not msg.status_list:
            return
        
        # Get latest status
        latest = msg.status_list[-1]
        
        # Status codes: 1=ACTIVE, 2=PREEMPTED, 3=SUCCEEDED, 4=ABORTED, 5=REJECTED
        if latest.status == 3:  # SUCCEEDED
            self.complete_waypoint(success=True)
        elif latest.status in [4, 5]:  # ABORTED or REJECTED
            self.complete_waypoint(success=False)
    
    def periodic_status(self):
        """Periodically log status."""
        if self.is_navigating:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            dist = self.current_waypoint_metrics.get('distance_traveled', 0)
            self.get_logger().info(f'Navigation in progress: {elapsed:.1f}s, {dist:.2f}m traveled')
    
    def save_metrics(self):
        """Save collected metrics to CSV."""
        # Calculate final totals
        if self.mission_start_time:
            total_time = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
            self.metrics['total_time'] = total_time
        
        # Create CSV filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_file = os.path.join(
            self.output_dir, 
            f'metrics_{self.planner_type}_{self.run_id}_{timestamp}.csv')
        
        # Write summary CSV
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            writer.writerow(['planner', self.metrics['planner']])
            writer.writerow(['run_id', self.metrics['run_id']])
            writer.writerow(['timestamp', self.metrics['timestamp']])
            writer.writerow(['total_time_sec', f"{self.metrics['total_time']:.2f}"])
            writer.writerow(['total_distance_m', f"{self.metrics['total_distance']:.2f}"])
            writer.writerow(['total_path_length_m', f"{self.metrics['total_path_length']:.2f}"])
            writer.writerow(['min_obstacle_distance_m', 
                           f"{self.metrics['min_obstacle_distance']:.2f}" 
                           if self.metrics['min_obstacle_distance'] != float('inf') else 'N/A'])
            writer.writerow(['waypoints_completed', self.metrics['waypoints_completed']])
            writer.writerow(['waypoints_failed', self.metrics['waypoints_failed']])
            writer.writerow(['success_rate', 
                           f"{self.metrics['waypoints_completed'] / max(1, self.metrics['waypoints_completed'] + self.metrics['waypoints_failed']) * 100:.1f}%"])
            
            if self.metrics['waypoint_times']:
                writer.writerow(['avg_waypoint_time_sec', 
                               f"{sum(self.metrics['waypoint_times']) / len(self.metrics['waypoint_times']):.2f}"])
        
        self.get_logger().info(f'Metrics saved to: {csv_file}')
        return csv_file
    
    def get_summary(self):
        """Get a summary string of current metrics."""
        success_rate = (self.metrics['waypoints_completed'] / 
                       max(1, self.metrics['waypoints_completed'] + self.metrics['waypoints_failed']) * 100)
        
        return (f"Planner: {self.metrics['planner']}\n"
                f"Waypoints: {self.metrics['waypoints_completed']}/{self.metrics['waypoints_completed'] + self.metrics['waypoints_failed']}\n"
                f"Success Rate: {success_rate:.1f}%\n"
                f"Total Distance: {self.metrics['total_distance']:.2f}m\n"
                f"Total Path Length: {self.metrics['total_path_length']:.2f}m\n"
                f"Min Obstacle Dist: {self.metrics['min_obstacle_distance']:.2f}m")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = MetricsLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down metrics logger...')
        # Save metrics on shutdown
        csv_file = node.save_metrics()
        node.get_logger().info(f'\n{node.get_summary()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
