"""
Full navigation launch file - combines everything.
Launches: Gazebo world + Robot spawn + Nav2 + Waypoint navigator
Usage: ros2 launch delivery_robot full_navigation.launch.py planner:=navfn
       ros2 launch delivery_robot full_navigation.launch.py planner:=smac
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Launch arguments
    planner = LaunchConfiguration('planner', default='navfn')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Gazebo world launch
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_delivery_robot, 'launch', 'gazebo_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Robot spawn (delay a bit to let Gazebo start)
    spawn_robot = TimerAction(
        period=3.0,
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
    
    # Nav2 bringup (delay to let robot spawn)
    nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_delivery_robot, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'planner': planner,
                    'use_sim_time': use_sim_time,
                    'initial_pose_x': '2.0',
                    'initial_pose_y': '1.0',
                    'initial_pose_yaw': '0.0',
                }.items()
            )
        ]
    )
    
    # Waypoint navigator (delay to let Nav2 initialize)
    waypoint_navigator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='delivery_robot',
                executable='waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # RViz (optional)
    rviz_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
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
        gazebo_world,
        spawn_robot,
        nav2_bringup,
        waypoint_navigator,
        rviz_node,
    ])

