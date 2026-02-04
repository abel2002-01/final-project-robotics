#!/usr/bin/env python3
"""
Simulation Bringup Launch File

This launch file properly starts Gazebo simulation with Nav2 navigation.
Key insight from ROS2 documentation: Clock bridge must start FIRST and
all nodes must use use_sim_time:=true for proper timestamp synchronization.

Based on: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-ROS-Clocks-and-sim-time
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('delivery_robot')
    
    # Declare arguments
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='smac',
        description='Planner to use: navfn or smac'
    )
    
    # Set use_sim_time globally via environment (affects all ROS nodes)
    use_sim_time_env = SetEnvironmentVariable(
        name='ROS_TIME_OVERRIDE',
        value='1'
    )
    
    # World file path
    world_file = os.path.join(pkg_dir, 'worlds', 'office_world_simple.sdf')
    
    # Robot model file
    robot_file = os.path.join(pkg_dir, 'models', 'delivery_robot.sdf')
    
    # ========== STEP 1: Start Gazebo ==========
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--force-version', '8'],
        output='screen',
        shell=False
    )
    
    # ========== STEP 2: Clock Bridge (MUST START FIRST!) ==========
    # This is critical - clock must be available before any other nodes
    clock_bridge = TimerAction(
        period=5.0,  # Wait for Gazebo to start
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )
        ]
    )
    
    # ========== STEP 3: Spawn Robot ==========
    spawn_robot = TimerAction(
        period=8.0,  # Wait for Gazebo to be ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/office_world_professional/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', f'sdf_filename: "{robot_file}", name: "delivery_robot", pose: {{position: {{x: 2.0, y: 1.0, z: 0.1}}}}'
                ],
                output='screen'
            )
        ]
    )
    
    # ========== STEP 4: Sensor Bridges (after clock is ready) ==========
    sensor_bridges = TimerAction(
        period=12.0,  # Wait for clock to be publishing
        actions=[
            # Scan bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='scan_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            # Odom bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='odom_bridge',
                arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            # Cmd_vel bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='cmd_vel_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ]
    )
    
    # ========== STEP 5: TF Publisher (with sim time) ==========
    odom_to_tf = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='odom_to_tf',
                name='odom_to_tf',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )
    
    # ========== STEP 6: Nav2 Bringup (with sim time) ==========
    nav2_params = PathJoinSubstitution([
        pkg_dir, 'config', 
        ['nav2_params_', LaunchConfiguration('planner'), '.yaml']
    ])
    
    nav2_bringup = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'planner': LaunchConfiguration('planner'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        planner_arg,
        use_sim_time_env,
        gazebo,
        clock_bridge,
        spawn_robot,
        sensor_bridges,
        odom_to_tf,
        nav2_bringup,
    ])



