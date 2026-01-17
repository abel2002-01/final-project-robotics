"""
Metrics Logger Node
Logs navigation performance metrics for algorithm comparison.
This will be fully implemented in Phase 5.
"""

import rclpy
from rclpy.node import Node


class MetricsLogger(Node):
    """
    A node that logs navigation metrics for comparing DWB vs MPPI controllers.
    
    Metrics to log:
    - Success rate (reached goal or not)
    - Time to goal (seconds)
    - Path length (meters)
    - Minimum obstacle distance (safety metric)
    - Number of recovery behaviors triggered
    """
    
    def __init__(self):
        super().__init__('metrics_logger')
        self.get_logger().info('Metrics Logger initialized')
        self.get_logger().info('This node will be fully implemented in Phase 5')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

