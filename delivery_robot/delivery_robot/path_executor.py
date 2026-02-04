#!/usr/bin/env python3
"""
Path Executor - Executes planned paths by sending cmd_vel commands.
This demonstrates the planner algorithm by actually moving the robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import time


class PathExecutor(Node):
    """Computes path using planner and executes it by sending cmd_vel."""
    
    def __init__(self):
        super().__init__('path_executor')
        
        # Parameters
        self.declare_parameter('planner', 'smac')
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 7.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        self.planner = self.get_parameter('planner').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Action client for path planning
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        
        # State
        self.current_pose = None
        self.current_yaw = 0.0
        self.path = None
        self.path_index = 0
        self.executing = False
        
        self.get_logger().info(f'Path Executor initialized')
        self.get_logger().info(f'  Planner: {self.planner}')
        self.get_logger().info(f'  Goal: ({self.goal_x}, {self.goal_y})')
        
        # Start after a short delay
        self.create_timer(3.0, self.start_navigation)
    
    def odom_callback(self, msg):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def start_navigation(self):
        """Request path from planner and start execution."""
        if self.path is not None:
            return  # Already started
        
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry...')
            return
        
        self.get_logger().info('Requesting path from planner...')
        
        # Check if action server is available
        if not self.path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Path planning action server not available!')
            self.get_logger().info('Falling back to direct path...')
            self.create_direct_path()
            return
        
        # Create goal
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = self.current_pose.position.x
        goal_msg.start.pose.position.y = self.current_pose.position.y
        goal_msg.start.pose.orientation.w = 1.0
        
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = self.goal_x
        goal_msg.goal.pose.position.y = self.goal_y
        goal_msg.goal.pose.orientation.w = 1.0
        
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = True
        
        # Send goal
        self.get_logger().info(f'Computing path with {self.planner} planner...')
        future = self.path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Path planning goal rejected!')
            self.get_logger().info('Falling back to direct path...')
            self.create_direct_path()
            return
        
        self.get_logger().info('Path planning goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)
    
    def path_result_callback(self, future):
        """Handle path result."""
        result = future.result().result
        if result.path.poses:
            self.get_logger().info(f'Path computed with {len(result.path.poses)} waypoints!')
            self.get_logger().info(f'Planning time: {result.planning_time.nanosec / 1e6:.1f} ms')
            self.path = result.path.poses
            self.path_pub.publish(result.path)
            self.start_execution()
        else:
            self.get_logger().error('Empty path returned!')
            self.create_direct_path()
    
    def create_direct_path(self):
        """Create a simple direct path to goal."""
        self.get_logger().info('Creating direct path to goal...')
        
        if self.current_pose is None:
            self.get_logger().error('No odometry available!')
            return
        
        # Create intermediate waypoints
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        
        num_points = 20
        self.path = []
        
        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = start_x + t * (self.goal_x - start_x)
            pose.pose.position.y = start_y + t * (self.goal_y - start_y)
            pose.pose.orientation.w = 1.0
            self.path.append(pose)
        
        self.get_logger().info(f'Direct path created with {len(self.path)} waypoints')
        self.start_execution()
    
    def start_execution(self):
        """Start executing the path."""
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║  EXECUTING PATH - Watch the robot move in Gazebo!         ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        
        self.path_index = 0
        self.executing = True
        self.create_timer(0.1, self.execute_step)  # 10 Hz control loop
    
    def execute_step(self):
        """Execute one step of path following."""
        if not self.executing or self.path is None:
            return
        
        if self.current_pose is None:
            return
        
        if self.path_index >= len(self.path):
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            self.executing = False
            return
        
        # Get current target waypoint
        target = self.path[self.path_index]
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        # Current position
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        
        # Distance and angle to target
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        
        # Angle difference
        angle_diff = target_angle - self.current_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Control
        cmd = Twist()
        
        if distance < 0.15:  # Close enough to waypoint
            self.path_index += 1
            if self.path_index % 5 == 0:
                self.get_logger().info(f'Waypoint {self.path_index}/{len(self.path)} reached')
        elif abs(angle_diff) > 0.3:  # Need to turn first
            cmd.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
            cmd.linear.x = 0.1  # Small forward motion while turning
        else:  # Move forward
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = angle_diff * 1.5  # Proportional angular control
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║  NAVIGATION COMPLETE!                                     ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutting down
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

