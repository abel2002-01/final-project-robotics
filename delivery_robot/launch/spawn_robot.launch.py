"""
Launch file to spawn TurtleBot3 Waffle in Gazebo.
Robot spawns at Reception waypoint (x=2, y=1).
Uses gz service command to spawn model from Fuel.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Reception waypoint pose (from office_world.sdf)
    initial_x = LaunchConfiguration('x', default='2.0')
    initial_y = LaunchConfiguration('y', default='1.0')
    initial_z = LaunchConfiguration('z', default='0.01')
    initial_yaw = LaunchConfiguration('yaw', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default='turtlebot3_waffle')
    world_name = LaunchConfiguration('world_name', default='office_world_professional')
    
    # Spawn robot using gz service
    # Wait a bit for Gazebo to be ready
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', ['/world/', world_name, '/create'],
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', [
                        'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Turtlebot3 Waffle", ',
                        'name: "', robot_name, '", ',
                        'pose: {position: {x: ', initial_x, ', y: ', initial_y, ', z: ', initial_z, '}, ',
                        'orientation: {yaw: ', initial_yaw, '}}'
                    ]
                ],
                output='screen',
                shell=True
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'x',
            default_value='2.0',
            description='Initial x position (Reception waypoint)'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='1.0',
            description='Initial y position (Reception waypoint)'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.01',
            description='Initial z position'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw orientation (radians)'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot3_waffle',
            description='Name of the robot model'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='office_world_professional',
            description='Name of the Gazebo world'
        ),
        spawn_robot,
    ])

