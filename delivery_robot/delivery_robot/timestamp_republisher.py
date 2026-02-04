#!/usr/bin/env python3
"""
Timestamp Republisher Node

This node subscribes to sensor topics from Gazebo (which have simulation time timestamps)
and republishes them with wall clock timestamps. This solves the TF_OLD_DATA issue that
occurs when using Gazebo Harmonic with Nav2.

The problem: Gazebo publishes sensor data with simulation timestamps, but when 
use_sim_time is False, Nav2 uses wall clock time for TF lookups. This causes 
a timestamp mismatch where Nav2 can't find transforms for the sensor data.

The solution: Republish sensor data with current wall clock timestamps so TF
lookups succeed.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TimestampRepublisher(Node):
    """Republishes sensor data with wall clock timestamps."""
    
    def __init__(self):
        super().__init__('timestamp_republisher')
        
        # Publishers with wall clock time
        self.scan_pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_fixed', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribers to raw topics
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Timestamp Republisher started')
        self.get_logger().info('  /scan -> /scan_fixed (with wall clock time)')
        self.get_logger().info('  /odom -> /odom_fixed + TF (with wall clock time)')
    
    def publish_static_transforms(self):
        """Publish static transforms for robot links."""
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # base_link -> lidar_link
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = now
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.1
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.12
        lidar_tf.transform.rotation.w = 1.0
        transforms.append(lidar_tf)
        
        # lidar_link -> delivery_robot/lidar_link/lidar
        lidar_gz_tf = TransformStamped()
        lidar_gz_tf.header.stamp = now
        lidar_gz_tf.header.frame_id = 'lidar_link'
        lidar_gz_tf.child_frame_id = 'delivery_robot/lidar_link/lidar'
        lidar_gz_tf.transform.rotation.w = 1.0
        transforms.append(lidar_gz_tf)
        
        # base_link -> base_footprint
        footprint_tf = TransformStamped()
        footprint_tf.header.stamp = now
        footprint_tf.header.frame_id = 'base_link'
        footprint_tf.child_frame_id = 'base_footprint'
        footprint_tf.transform.translation.z = -0.05
        footprint_tf.transform.rotation.w = 1.0
        transforms.append(footprint_tf)
        
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms')
    
    def scan_callback(self, msg):
        """Republish scan with wall clock time."""
        # Create new message with same data but wall clock time
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'lidar_link'  # Use our TF frame
        
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities
        
        self.scan_pub.publish(new_msg)
    
    def odom_callback(self, msg):
        """Republish odom with wall clock time and publish TF."""
        now = self.get_clock().now().to_msg()
        
        # Republish odometry with wall clock time
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.stamp = now
        new_msg.header.frame_id = 'odom'
        new_msg.child_frame_id = 'base_link'
        new_msg.pose = msg.pose
        new_msg.twist = msg.twist
        self.odom_pub.publish(new_msg)
        
        # Publish map -> odom (identity, assuming perfect localization)
        map_odom = TransformStamped()
        map_odom.header.stamp = now
        map_odom.header.frame_id = 'map'
        map_odom.child_frame_id = 'odom'
        map_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(map_odom)
        
        # Publish odom -> base_link
        odom_base = TransformStamped()
        odom_base.header.stamp = now
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'
        odom_base.transform.translation.x = msg.pose.pose.position.x
        odom_base.transform.translation.y = msg.pose.pose.position.y
        odom_base.transform.translation.z = msg.pose.pose.position.z
        odom_base.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(odom_base)


def main(args=None):
    rclpy.init(args=args)
    node = TimestampRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



