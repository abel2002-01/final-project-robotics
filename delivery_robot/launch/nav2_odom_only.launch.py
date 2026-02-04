#!/usr/bin/env python3
"""
Nav2 Launch File WITHOUT AMCL - Odometry-Only Localization

This launch file starts Nav2 without AMCL, using only odometry for localization.
The map->odom transform is published by the timestamp_republisher node.

This avoids TF timestamp synchronization issues with Gazebo Harmonic.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('delivery_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Parameters
    namespace = LaunchConfiguration('namespace', default='')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = LaunchConfiguration('map')
    
    # Lifecycle nodes (no AMCL)
    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
    ]
    
    # Configure parameters
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/tmp/nav2_no_amcl.yaml',
            description='Full path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, 'maps', 'office_map.yaml'),
            description='Full path to map yaml file'
        ),
        
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),
        
        # Planner (SmacPlanner2D)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        
        # Controller (DWB)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel')],
        ),
        
        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        
        # Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
        ),
    ])



