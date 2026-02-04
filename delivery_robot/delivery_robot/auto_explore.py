#!/usr/bin/env python3
"""
Auto Exploration Node for Map Creation
Moves the robot in a pattern to create a map using SLAM.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class AutoExplorer(Node):
    """Automatically explores the environment for mapping."""
    
    def __init__(self):
        super().__init__('auto_explorer')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Auto Explorer started - Creating map...')
        
        # Movement patterns for exploration
        self.explore()
    
    def send_velocity(self, linear_x, angular_z, duration):
        """Send velocity command for specified duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        time.sleep(0.5)
    
    def explore(self):
        """Execute exploration pattern."""
        self.get_logger().info('Starting exploration pattern...')
        
        movements = [
            # (linear_x, angular_z, duration, description)
            (0.2, 0.0, 3.0, "Forward"),
            (0.0, 0.5, 2.0, "Turn left"),
            (0.2, 0.0, 4.0, "Forward"),
            (0.0, -0.5, 2.0, "Turn right"),
            (0.2, 0.0, 5.0, "Forward to storage"),
            (0.0, 0.5, 3.0, "Turn around"),
            (0.2, 0.0, 4.0, "Forward"),
            (0.0, -0.5, 2.0, "Turn right"),
            (0.2, 0.0, 6.0, "Forward to office"),
            (0.0, 0.5, 2.0, "Turn"),
            (0.2, 0.0, 3.0, "Forward"),
            (0.0, 0.5, 4.0, "Full turn"),
            (0.2, 0.0, 5.0, "Return"),
        ]
        
        for i, (lin, ang, dur, desc) in enumerate(movements):
            self.get_logger().info(f'Step {i+1}/{len(movements)}: {desc}')
            self.send_velocity(lin, ang, dur)
        
        self.get_logger().info('Exploration complete! Map should be ready.')
        self.get_logger().info('Save map with: ros2 run nav2_map_server map_saver_cli -f <path>')


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

