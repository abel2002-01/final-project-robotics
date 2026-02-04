"""
Launch file to spawn the delivery robot in Gazebo.
Robot spawns at Reception waypoint (x=2, y=1).
Uses local SDF model file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    
    # Robot model path
    robot_sdf = os.path.join(pkg_delivery_robot, 'models', 'delivery_robot.sdf')
    
    # Spawn robot using gz service with sdf_filename pointing to local file
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'gz service -s /world/office_world_professional/create '
                    f'--reqtype gz.msgs.EntityFactory '
                    f'--reptype gz.msgs.Boolean '
                    f'--timeout 5000 '
                    f'--req \'sdf_filename: "{robot_sdf}" '
                    f'name: "delivery_robot" '
                    f'pose: {{position: {{x: 2.0 y: 1.0 z: 0.0}} orientation: {{w: 1.0 x: 0.0 y: 0.0 z: 0.0}}}}\''
                ],
                output='screen'
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
        spawn_robot,
    ])
