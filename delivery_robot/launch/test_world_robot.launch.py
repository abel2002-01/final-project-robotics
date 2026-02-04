"""
Test launch file - Launches world + robot + bridge for verification.
Use this to test if the basic system works before running full navigation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # World file
    world_file = os.path.join(pkg_delivery_robot, 'worlds', 'office_world.sdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # 2. Spawn robot after Gazebo starts (5 second delay)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/office_world_professional/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 
                    'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Turtlebot3 Waffle", '
                    'name: "turtlebot3_waffle", '
                    'pose: {position: {x: 2.0, y: 1.0, z: 0.01}}'
                ],
                output='screen'
            )
        ]
    )
    
    # 3. Bridge topics (8 second delay to ensure robot is spawned)
    bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
            )
        ]
    )
    
    # 4. Static transform publisher for base_link to base_scan (LiDAR)
    # TurtleBot3 Waffle LiDAR is mounted 0.122m above base_link
    static_tf_base_to_scan = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_base_scan',
                output='screen',
                arguments=['0', '0', '0.122', '0', '0', '0', 'base_link', 'base_scan'],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # 5. Odometry to TF publisher (converts /odom to TF odom->base_link)
    odom_to_tf_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='odom_to_tf',
                name='odom_to_tf',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gazebo,
        spawn_robot,
        bridge,
        static_tf_base_to_scan,
        odom_to_tf_node,
    ])

