"""
Launch file to start Gazebo with the office world.
Supports both full office world (with Fuel models) and simple world (no external dependencies).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Use simple world by default (no Fuel dependencies)
    world_file = os.path.join(pkg_delivery_robot, 'worlds', 'office_world_simple.sdf')
    
    # Fallback to original if simple doesn't exist
    if not os.path.exists(world_file):
        world_file = os.path.join(pkg_delivery_robot, 'worlds', 'office_world.sdf')
    
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo simulator launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
    ])
