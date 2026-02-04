#!/usr/bin/env python3
"""
Fix timestamps for sensor data from Gazebo.
This bridges Gazebo's simulation time to ROS2's wall clock time for Nav2.
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SensorTimestampFix(Node):
    """Republishes sensor data with WALL CLOCK timestamps for Nav2 compatibility."""
    
    def __init__(self):
        super().__init__('sensor_timestamp_fix')
        
        # Create a SYSTEM clock (wall clock) - always returns real time
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        
        # Subscribe to raw topics from Gazebo bridge (sim time stamps)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_callback, 10)
        
        # Publish to topics Nav2 expects (wall clock stamps)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.get_logger().info('Sensor timestamp fix node started')
        self.get_logger().info('  /scan_raw -> /scan (wall clock)')
        self.get_logger().info('  /odom_raw -> /odom (wall clock)')
    
    def scan_callback(self, msg):
        """Republish scan with WALL CLOCK timestamp."""
        # Use wall clock - NOT self.get_clock() which may return sim time
        msg.header.stamp = self.wall_clock.now().to_msg()
        # Fix frame_id (Gazebo uses long format like delivery_robot/lidar_link/lidar)
        if 'delivery_robot' in msg.header.frame_id:
            msg.header.frame_id = 'lidar_link'
        self.scan_pub.publish(msg)
    
    def odom_callback(self, msg):
        """Republish odom with WALL CLOCK timestamp."""
        msg.header.stamp = self.wall_clock.now().to_msg()
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorTimestampFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

