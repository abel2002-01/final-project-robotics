"""
Launch file for Nav2 stack with planner selection.
Accepts planner argument: planner:=navfn or planner:=smac
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_delivery_robot = get_package_share_directory('delivery_robot')
    
    # Launch configurations
    planner_arg = LaunchConfiguration('planner')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    
    # Config files
    navfn_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_params_navfn.yaml')
    smac_config = os.path.join(pkg_delivery_robot, 'config', 'nav2_params_smac.yaml')
    default_map = os.path.join(pkg_delivery_robot, 'maps', 'office_map.yaml')
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }],
    )
    
    # AMCL node with inline parameters
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'set_initial_pose': True,
            'initial_pose.x': 2.0,
            'initial_pose.y': 1.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'max_particles': 500,
            'min_particles': 100,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'scan_topic': 'scan',
            'tf_broadcast': True,
        }],
    )
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 60.0,
            'attempt_duration': 60.0,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }],
    )
    
    # NavFn planner group - use proper string comparison
    is_navfn = PythonExpression([
        "'", planner_arg, "' == 'navfn'"
    ])
    
    nav2_navfn_group = GroupAction(
        condition=IfCondition(is_navfn),
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
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
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
    
    # SmacPlanner2D group
    is_smac = PythonExpression([
        "'", planner_arg, "' == 'smac'"
    ])
    
    nav2_smac_group = GroupAction(
        condition=IfCondition(is_smac),
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
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
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
            default_value=default_map,
            description='Full path to map yaml file'
        ),
        map_server_node,
        amcl_node,
        nav2_navfn_group,
        nav2_smac_group,
        lifecycle_manager,
    ])
