#!/usr/bin/env python3
"""Simple map saver script."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import yaml
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_simple')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.map_received = False
        self.get_logger().info('Waiting for map...')
    
    def map_callback(self, msg):
        if self.map_received:
            return
            
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')
        
        # Convert to image
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        
        # Convert: -1 (unknown) -> 205, 0 (free) -> 254, 100 (occupied) -> 0
        img_data = np.zeros_like(data, dtype=np.uint8)
        img_data[data == -1] = 205  # Unknown
        img_data[data == 0] = 254   # Free
        img_data[data == 100] = 0   # Occupied
        img_data[(data > 0) & (data < 100)] = 128  # Intermediate
        
        # Flip vertically (ROS maps are bottom-up)
        img_data = np.flipud(img_data)
        
        # Save image
        map_dir = '/home/abel/robo/Final-Project/increment_01_basic_navigation/delivery_robot/maps'
        os.makedirs(map_dir, exist_ok=True)
        
        img = Image.fromarray(img_data, mode='L')
        img.save(f'{map_dir}/office_map.pgm')
        
        # Save YAML
        yaml_data = {
            'image': 'office_map.pgm',
            'mode': 'trinary',
            'resolution': msg.info.resolution,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        
        with open(f'{map_dir}/office_map.yaml', 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        self.get_logger().info(f'Map saved to {map_dir}')
        self.map_received = True
        raise SystemExit(0)

def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

