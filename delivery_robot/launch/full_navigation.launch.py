"""
Full navigation launch file - combines everything.
Launches: Gazebo world + Robot spawn + Bridges + Nav2 + Waypoint navigator

FIXED: Now includes ros_gz_bridge and odom_to_tf nodes that were missing!

Usage: ros2 launch delivery_robot full_navigation.launch.py planner:=navfn
       ros2 launch delivery_robot full_navigation.launch.py planner:=smac
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    
    # Launch arguments
    planner = LaunchConfiguration('planner', default='navfn')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # ========== STEP 1: Gazebo World (t=0s) ==========
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_delivery_robot, 'launch', 'gazebo_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # ========== STEP 2: Clock Bridge (t=3s) ==========
    # CRITICAL: Clock bridge must start before other ROS nodes for sim time to work
    clock_bridge = TimerAction(
        period=3.0,
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
    
    # ========== STEP 3: Robot Spawn (t=5s) ==========
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_delivery_robot, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={
                    'x': '2.0',
                    'y': '1.0',
                    'z': '0.01',
                    'yaw': '0.0',
                }.items()
            )
        ]
    )
    
    # ========== STEP 4: Sensor Bridges (t=10s) ==========
    # Bridge Gazebo topics to ROS2 _raw topics (sim time stamps)
    sensor_bridges = TimerAction(
        period=10.0,
        actions=[
            # LiDAR scan bridge -> /scan_raw (will be republished with wall clock)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='scan_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                remappings=[('/scan', '/scan_raw')],
                output='screen',
            ),
            # Odometry bridge -> /odom_raw (will be republished with wall clock)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='odom_bridge',
                arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                remappings=[('/odom', '/odom_raw')],
                output='screen',
            ),
            # Velocity command bridge (ROS -> Gazebo)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='cmd_vel_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                output='screen',
            ),
        ]
    )
    
    # ========== STEP 5: Timestamp Fixer (t=11s) ==========
    # Republish sensor data with WALL CLOCK timestamps for Nav2
    timestamp_fixer = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='scan_timestamp_fix',
                name='sensor_timestamp_fix',
                output='screen',
            )
        ]
    )
    
    # ========== STEP 6: TF Publisher (t=12s) ==========
    # Converts odometry to TF and publishes robot transforms (uses wall clock)
    odom_to_tf = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='odom_to_tf',
                name='odom_to_tf',
                output='screen',
            )
        ]
    )
    
    # ========== STEP 6: Nav2 Bringup (t=15s) ==========
    nav2_bringup = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_delivery_robot, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'planner': planner,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )
    
    # ========== STEP 7: Waypoint Navigator (t=25s) ==========
    # Give Nav2 extra time to fully initialize
    waypoint_navigator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )
    
    # ========== RViz Visualization ==========
    rviz_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_default_view.rviz')
    rviz_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(use_rviz),
                parameters=[{'use_sim_time': True}],
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'planner',
            default_value='navfn',
            description='Global planner to use: navfn or smac'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true'
        ),
        
        # Launch sequence (properly ordered with bridges, timestamp fix, and TF)
        gazebo_world,       # t=0s:  Start Gazebo
        clock_bridge,       # t=3s:  Bridge clock for sim time
        spawn_robot,        # t=5s:  Spawn robot
        sensor_bridges,     # t=10s: Bridge sensors to *_raw topics
        timestamp_fixer,    # t=11s: Republish with wall clock timestamps
        odom_to_tf,         # t=12s: Publish TF transforms (wall clock)
        nav2_bringup,       # t=15s: Start Nav2 stack
        rviz_node,          # t=15s: Start RViz
        waypoint_navigator, # t=25s: Start waypoint navigation
    ])
