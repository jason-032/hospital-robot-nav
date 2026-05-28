#!/usr/bin/env python3
"""
Generic multi-floor launch file for the hospital robot navigation simulation.

Usage:
  ros2 launch spatial_maps spatial_maps.launch.py              # defaults to 1F
  ros2 launch spatial_maps spatial_maps.launch.py floor:=2F
  ros2 launch spatial_maps spatial_maps.launch.py floor:=B1F

All floors share the same map coordinate system and robot spawn point (21, 38).
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    spatial_maps_share = get_package_share_directory('spatial_maps')
    robot_desc_share   = get_package_share_directory('my_robot_description')
    ros_gz_sim_share   = get_package_share_directory('ros_gz_sim')

    declare_floor = DeclareLaunchArgument(
        'floor', default_value='1F',
        description='Floor to simulate (B1F, 1F, 2F … 14F)')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    floor        = LaunchConfiguration('floor')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file   = os.path.join(robot_desc_share, 'urdf', 'my_robot.urdf.xacro')
    nav2_params = os.path.join(spatial_maps_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(spatial_maps_share, 'config', 'spatial_maps_1f.rviz')
    semantic_json = '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json'

    # Floor-specific paths are resolved at launch time via Python substitution.
    # LaunchConfiguration values aren't plain strings here, so we use a helper
    # OpaqueFunction to resolve them after argument parsing.
    from launch.actions import OpaqueFunction

    def launch_setup(context, *args, **kwargs):
        floor_val = context.launch_configurations['floor']
        world_val = floor_val.lower()

        world_file    = os.path.join(spatial_maps_share, 'worlds', f'{world_val}.world')
        map_yaml      = os.path.join(spatial_maps_share, 'maps',   f'{floor_val}.yaml')
        bridge_config = os.path.join(spatial_maps_share, 'config', f'gz_bridge_{world_val}.yaml')

        # context.launch_configurations always returns strings; convert to bool
        # so nodes that declare use_sim_time as bool don't throw InvalidParameterType.
        use_sim = context.launch_configurations.get('use_sim_time', 'true').lower() == 'true'

        cleanup = ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -9 -O 10 -f "gz sim" 2>/dev/null; '
                 'pkill -9 -O 10 -f "lib/spatial_maps/" 2>/dev/null; '
                 'true'],
            output='screen',
            name='pre_launch_cleanup',
        )

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': world_file + ' -r'}.items()
        )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_file])},
                {'use_sim_time': use_sim}
            ]
        )

        spawn_robot = TimerAction(
            period=5.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-name',  'my_robot',
                    '-topic', 'robot_description',
                    '-x', '21.0', '-y', '38.0', '-z', '0.1', '-Y', '0.0'
                ]
            )]
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}]
        )

        amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}]
        )

        odom_tf_republisher = Node(
            package='spatial_maps',
            executable='odom_tf_republisher.py',
            name='odom_tf_republisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim}]
        )

        joint_state_relay = Node(
            package='spatial_maps',
            executable='joint_state_relay.py',
            name='joint_state_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim}]
        )

        map_publisher = Node(
            package='spatial_maps',
            executable='map_publisher_node.py',
            name='map_publisher',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml},
                {'frame_id': 'map'},
                {'use_sim_time': use_sim}
            ]
        )

        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}]
        )

        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}]
        )

        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}],
            remappings=[('cmd_vel', '/cmd_vel')]
        )

        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}]
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim}]
        )

        nav2_lifecycle_manager = TimerAction(
            period=12.0,
            actions=[Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_nav',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim},
                    {'autostart': True},
                    {'node_names': [
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'smoother_server',
                        'bt_navigator',
                    ]}
                ]
            )]
        )

        poi_publisher = Node(
            package='spatial_maps',
            executable='poi_publisher_node.py',
            name='poi_publisher',
            output='screen',
            parameters=[
                {'semantic_json': semantic_json},
                {'floor': floor_val},
                {'marker_frame': 'map'},
                {'publish_rate_hz': 1.0}
            ]
        )

        poi_click_node = Node(
            package='spatial_maps',
            executable='poi_click_node.py',
            name='poi_click_node',
            output='screen',
            parameters=[
                {'semantic_json': semantic_json},
                {'floor': floor_val},
            ]
        )

        poi_nav_node = Node(
            package='spatial_maps',
            executable='poi_nav_node.py',
            name='poi_nav_node',
            output='screen',
            parameters=[
                {'semantic_json': semantic_json},
                {'floor': floor_val},
                {'goal_z': 0.0},
                {'map_yaml': map_yaml},
                {'robot_start_x': 21.0},
                {'robot_start_y': 38.0},
            ]
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim}]
        )

        return [
            cleanup,
            gz_sim,
            robot_state_publisher,
            ros_gz_bridge,
            amcl,
            odom_tf_republisher,
            joint_state_relay,
            map_publisher,
            bt_navigator,
            planner_server,
            controller_server,
            behavior_server,
            smoother_server,
            nav2_lifecycle_manager,
            poi_publisher,
            poi_click_node,
            poi_nav_node,
            rviz,
            spawn_robot,
        ]

    return LaunchDescription([
        declare_floor,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
