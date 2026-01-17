#!/usr/bin/env python3
"""
Waypoint Navigator Node
Navigates robot through a sequence of waypoints using Nav2 SimpleCommander.
Waypoints: Reception -> Storage -> Office
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math


class WaypointNavigator(Node):
    """Navigates robot through delivery waypoints."""
    
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Create Nav2 navigator
        self.navigator = BasicNavigator()
        
        # Waypoint poses (from office_world.sdf)
        # Reception: x=2, y=1, yaw=0
        # Storage: x=4, y=7.5, yaw=0
        # Office: x=12.5, y=3, yaw=0
        self.waypoint_names = ['Reception', 'Storage', 'Office']
        self.waypoints = [
            self.create_waypoint(2.0, 1.0, 0.0),
            self.create_waypoint(4.0, 7.5, 0.0),
            self.create_waypoint(12.5, 3.0, 0.0),
        ]
        
        self.get_logger().info('Waypoint Navigator initialized')
        self.get_logger().info(f'Defined {len(self.waypoints)} waypoints')
    
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
    
    def navigate_waypoints(self):
        """Navigate through all waypoints sequentially."""
        self.get_logger().info('Waiting for Nav2 to become available...')
        
        # Wait for Nav2 to become available
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')
        
        # Set initial pose (same as first waypoint for now)
        initial_pose = self.waypoints[0]
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f'Set initial pose at {self.waypoint_names[0]}')
        
        # Give Nav2 a moment to set the initial pose
        time.sleep(2.0)
        
        # Navigate through waypoints
        self.get_logger().info(f'Starting navigation: {" -> ".join(self.waypoint_names)}')
        
        # Set waypoints
        self.navigator.followWaypoints(self.waypoints)
        
        i = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:  # Log every 10th iteration
                self.get_logger().info(
                    f'Executing waypoint {feedback.current_waypoint + 1}/{len(self.waypoints)}'
                )
            i += 1
            time.sleep(0.1)
        
        # Check result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully completed all waypoints!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Waypoint navigation was canceled')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('Waypoint navigation failed!')
            return False
        else:
            self.get_logger().warn(f'Unknown result: {result}')
            return False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    navigator_node = WaypointNavigator()
    
    try:
        success = navigator_node.navigate_waypoints()
        if success:
            navigator_node.get_logger().info('Delivery route completed successfully!')
        else:
            navigator_node.get_logger().error('Delivery route failed!')
    except KeyboardInterrupt:
        navigator_node.get_logger().info('Waypoint navigation interrupted by user')
    except Exception as e:
        navigator_node.get_logger().error(f'Error during navigation: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
