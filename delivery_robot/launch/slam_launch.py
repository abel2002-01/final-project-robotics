"""
Launch file for SLAM Toolbox to create map of office world.
Use this for offline mapping before navigation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    slam_params_file = LaunchConfiguration('slam_params_file', default=os.path.join(
        pkg_delivery_robot, 'config', 'slam_params.yaml'))
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to the SLAM parameters file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        slam_node,
    ])

