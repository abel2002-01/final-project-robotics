#!/usr/bin/env python3
"""
Odometry and Robot TF publisher.
Subscribes to /odom and publishes TF transforms for the robot.
Uses simulation time when use_sim_time is enabled.
Also publishes static transforms for robot links including lidar.

Based on ROS2 sim time best practices:
https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-ROS-Clocks-and-sim-time
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomToTF(Node):
    """Converts odometry messages to TF transforms and publishes static robot TFs."""
    
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Check if we're using simulation time (param already declared by ROS2 runtime)
        # Don't redeclare it - just get it
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except:
            self.use_sim_time = False
        
        # Create a SYSTEM clock (wall clock) that always returns real time
        # This is needed because TF2 has issues with sim time synchronization
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        
        # Dynamic TF broadcaster for odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Static TF broadcaster for robot links
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to odometry from Gazebo bridge
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Odometry to TF node started')
        self.get_logger().info(f'  use_sim_time: {self.use_sim_time}')
    
    def publish_static_transforms(self):
        """Publish static transforms for robot links."""
        from builtin_interfaces.msg import Time
        
        transforms = []
        # Use zero time for static transforms (means "always valid")
        zero_time = Time()
        
        # base_link -> lidar_link (from robot SDF: pose 0.1 0 0.12)
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = zero_time
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.1
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.12
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0
        transforms.append(lidar_tf)
        
        # Also publish for the Gazebo-style frame names
        # delivery_robot/lidar_link/lidar -> lidar_link (identity transform)
        lidar_gz_tf = TransformStamped()
        lidar_gz_tf.header.stamp = zero_time
        lidar_gz_tf.header.frame_id = 'lidar_link'
        lidar_gz_tf.child_frame_id = 'delivery_robot/lidar_link/lidar'
        lidar_gz_tf.transform.translation.x = 0.0
        lidar_gz_tf.transform.translation.y = 0.0
        lidar_gz_tf.transform.translation.z = 0.0
        lidar_gz_tf.transform.rotation.x = 0.0
        lidar_gz_tf.transform.rotation.y = 0.0
        lidar_gz_tf.transform.rotation.z = 0.0
        lidar_gz_tf.transform.rotation.w = 1.0
        transforms.append(lidar_gz_tf)
        
        # base_link -> base_footprint (for AMCL compatibility)
        footprint_tf = TransformStamped()
        footprint_tf.header.stamp = zero_time
        footprint_tf.header.frame_id = 'base_link'
        footprint_tf.child_frame_id = 'base_footprint'
        footprint_tf.transform.translation.x = 0.0
        footprint_tf.transform.translation.y = 0.0
        footprint_tf.transform.translation.z = -0.05
        footprint_tf.transform.rotation.x = 0.0
        footprint_tf.transform.rotation.y = 0.0
        footprint_tf.transform.rotation.z = 0.0
        footprint_tf.transform.rotation.w = 1.0
        transforms.append(footprint_tf)
        
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms for robot links')
    
    def odom_callback(self, msg):
        """Convert odometry to TF transform with wall clock time.
        
        Uses wall clock time to avoid synchronization issues with Nav2.
        """
        t = TransformStamped()
        
        # Use node clock (supports simulation time)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        # Disable map->odom publication as AMCL provides this
        # map_odom = TransformStamped()
        # map_odom.header.stamp = t.header.stamp
        # map_odom.header.frame_id = 'map'
        # map_odom.child_frame_id = 'odom'
        # map_odom.transform.rotation.w = 1.0
        # self.tf_broadcaster.sendTransform(map_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
