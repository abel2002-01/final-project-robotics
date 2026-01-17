"""
Launch file for Nav2 stack with planner selection.
Accepts planner argument: planner:=navfn or planner:=smac
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    planner = LaunchConfiguration('planner', default='navfn')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.join(
        pkg_delivery_robot, 'maps', 'office_map.yaml'))
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='2.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='1.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')
    
    # Select config file based on planner argument
    navfn_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_params_navfn.yaml')
    smac_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_params_smac.yaml')
    
    # Conditionally select config file
    nav2_params_file = PythonExpression([
        "config_navfn if planner == 'navfn' else config_smac"
    ])
    
    # We'll use a simpler approach: use IfCondition to set the params file
    # Actually, let's use a more direct approach with substitution
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ],
    )
    
    # AMCL localization
    amcl_config_file = os.path.join(pkg_nav2_bringup, 'params', 'amcl_config.yaml')
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config_file,
            {
                'use_sim_time': use_sim_time,
                'initial_pose.x': initial_pose_x,
                'initial_pose.y': initial_pose_y,
                'initial_pose.yaw': initial_pose_yaw,
            }
        ],
    )
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
        ],
    )
    
    # Nav2 with NavFn config
    nav2_navfn_group = GroupAction(
        condition=IfCondition(PythonExpression(["planner == 'navfn'"])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[navfn_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[navfn_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[navfn_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[navfn_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[navfn_config, {'use_sim_time': use_sim_time}],
            ),
        ]
    )
    
    # Nav2 with SmacPlanner2D config
    nav2_smac_group = GroupAction(
        condition=IfCondition(PythonExpression(["planner == 'smac'"])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[smac_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[smac_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[smac_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[smac_config, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[smac_config, {'use_sim_time': use_sim_time}],
            ),
        ]
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
            'map',
            default_value=map_file,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='2.0',
            description='Initial pose x (Reception waypoint)'
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='1.0',
            description='Initial pose y (Reception waypoint)'
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw (radians)'
        ),
        map_server_node,
        amcl_node,
        nav2_navfn_group,
        nav2_smac_group,
        lifecycle_manager,
    ])

