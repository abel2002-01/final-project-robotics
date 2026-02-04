#!/usr/bin/env python3
"""
Gazebo Navigation Demo Launch File
Properly synchronizes simulation time across all nodes
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arguments
    planner_arg = DeclareLaunchArgument(
        'planner', default_value='smac',
        description='Planner to use: smac or navfn'
    )
    
    planner = LaunchConfiguration('planner')
    
    # Gazebo world path
    world_path = os.path.join(pkg_share, 'worlds', 'office_world_simple.sdf')
    robot_path = os.path.join(pkg_share, 'models', 'delivery_robot.sdf')
    
    # 1. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )
    
    # 2. Spawn robot (after 8 seconds)
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/office_world_professional/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{robot_path}" name: "delivery_robot" pose: {{position: {{x: 2.0 y: 1.0 z: 0.05}}}}'
                ],
                output='screen'
            )
        ]
    )
    
    # 3. Clock bridge (after 10 seconds) - CRITICAL for time sync
    clock_bridge = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen'
            )
        ]
    )
    
    # 4. Cmd_vel bridge (ROS2 -> Gazebo) - CRITICAL for robot movement
    cmd_vel_bridge = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='cmd_vel_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                output='screen'
            )
        ]
    )
    
    # 5. Odom bridge (Gazebo -> ROS2)
    odom_bridge = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='odom_bridge',
                arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                output='screen'
            )
        ]
    )
    
    # 6. Scan bridge (Gazebo -> ROS2)
    scan_bridge = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='scan_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                output='screen'
            )
        ]
    )
    
    # 7. Static TF: map -> odom (identity)
    map_odom_tf = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_odom_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 8. Dynamic TF from odometry using our custom node
    odom_tf = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='odom_to_tf',
                name='odom_to_tf',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # 9. Nav2 bringup (after 18 seconds)
    nav2_launch = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_share, 'config', 'nav2_params_smac.yaml'),
                    'autostart': 'true'
                }.items()
            )
        ]
    )
    
    # 10. Send navigation goal (after 40 seconds to let Nav2 fully initialize)
    send_goal = TimerAction(
        period=45.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    '{pose: {header: {frame_id: "map"}, pose: {position: {x: 4.0, y: 7.5, z: 0.0}, orientation: {w: 1.0}}}}',
                    '--feedback'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        planner_arg,
        gazebo,
        spawn_robot,
        clock_bridge,
        cmd_vel_bridge,
        odom_bridge,
        scan_bridge,
        map_odom_tf,
        odom_tf,
        nav2_launch,
        send_goal,
    ])

