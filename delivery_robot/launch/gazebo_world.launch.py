"""
Launch file to start Gazebo with the professional office world.
This launches the aesthetically enhanced office environment for the delivery robot project.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # World file path - points to the professionally designed office world
    world_file = os.path.join(pkg_delivery_robot, 'worlds', 'office_world.sdf')
    
    # Verify world file exists
    if not os.path.exists(world_file):
        raise FileNotFoundError(
            f"World file not found: {world_file}\n"
            f"Please ensure the world file exists in the worlds directory."
        )
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo simulator launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
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

