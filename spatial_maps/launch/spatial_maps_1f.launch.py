#!/usr/bin/env python3
"""
Launch Gz Sim with the 1F hospital world, spawn my_robot, and bring up nav2.

Usage:
  ros2 launch spatial_maps spatial_maps_1f.launch.py

Send a robot to a POI:
  ros2 topic pub --once /goal_poi std_msgs/String "data: 'S1312'"
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

    world_file    = os.path.join(spatial_maps_share, 'worlds', '1f.world')
    urdf_file     = os.path.join(robot_desc_share,   'urdf',   'my_robot.urdf.xacro')
    bridge_config = os.path.join(spatial_maps_share, 'config', 'gz_bridge_1f.yaml')
    rviz_config   = os.path.join(spatial_maps_share, 'config', 'spatial_maps_1f.rviz')
    map_yaml      = os.path.join(spatial_maps_share, 'maps',   '1F.yaml')
    nav2_params   = os.path.join(spatial_maps_share, 'config', 'nav2_params.yaml')
    semantic_json = '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Kill any leftover processes from previous launches before starting fresh.
    # Without this, orphaned nodes accumulate across Ctrl+C relaunches and
    # produce ghost robots, duplicate TF publishers, and TF_OLD_DATA floods.
    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             # -O 10: only kill processes older than 10 s — spares the newly
             # spawned Gz while still evicting zombie Gz servers from prior
             # launches that weren't stopped with Ctrl+C.
             'pkill -9 -O 10 -f "gz sim" 2>/dev/null; '
             # -O 10 on both commands: only kills processes older than 10 s,
             # so freshly spawned nodes from this launch are always safe.
             'pkill -9 -O 10 -f "lib/spatial_maps/" 2>/dev/null; '
             'true'],
        output='screen',
        name='pre_launch_cleanup',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    # ── Gz Sim ────────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file + ' -r'}.items()
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_file])},
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Spawn robot (delayed 5 s to let Gz Sim finish loading) ────────────────
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

    # ── ROS <-> Gz bridge ─────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    # ── Static TF: map → odom ─────────────────────────────────────────────────
    # Robot spawns at BIM coords (21, 38, 0.1). This anchors map to odom.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=['21.0', '38.0', '0.1', '0', '0', '0', 'map', 'odom']
    )

    # ── Odom TF republisher ───────────────────────────────────────────────────
    # The bridge converts Gz odometry to /odom but does NOT publish TF.
    # This node is the sole publisher of odom → base_footprint.
    # Monotonic guard + odom_publish_frequency=50 keeps it at one TF per
    # sim clock tick — no race, no TF_OLD_DATA flood.
    odom_tf_republisher = Node(
        package='spatial_maps',
        executable='odom_tf_republisher.py',
        name='odom_tf_republisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Joint state relay ─────────────────────────────────────────────────────
    # Bridge publishes to /joint_states_gz with Gz timestamps.
    # Relay re-stamps with now() so RSP publishes fresh joint TF,
    # fixing "No transform" for wheel/arm links in RViz.
    joint_state_relay = Node(
        package='spatial_maps',
        executable='joint_state_relay.py',
        name='joint_state_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Map publisher (no lifecycle dependency — always publishing) ───────────
    map_publisher = Node(
        package='spatial_maps',
        executable='map_publisher_node.py',
        name='map_publisher',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'frame_id': 'map'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # ── Nav2 stack ────────────────────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    # Delayed 12 s — robot spawns at 5 s, odom TF needs a moment to flow
    nav2_lifecycle_manager = TimerAction(
        period=12.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'smoother_server',
                    'bt_navigator',
                ]}
            ]
        )]
    )

    # ── POI publisher (markers in RViz) ───────────────────────────────────────
    poi_publisher = Node(
        package='spatial_maps',
        executable='poi_publisher_node.py',
        name='poi_publisher',
        output='screen',
        parameters=[
            {'semantic_json': semantic_json},
            {'floor': '1F'},
            {'marker_frame': 'map'},
            {'publish_rate_hz': 1.0}
        ]
    )

    # ── POI click node (Publish Point tool → nearest room → /goal_poi) ──────
    poi_click_node = Node(
        package='spatial_maps',
        executable='poi_click_node.py',
        name='poi_click_node',
        output='screen',
        parameters=[
            {'semantic_json': semantic_json},
            {'floor': '1F'},
        ]
    )

    # ── POI navigation node ───────────────────────────────────────────────────
    poi_nav_node = Node(
        package='spatial_maps',
        executable='poi_nav_node.py',
        name='poi_nav_node',
        output='screen',
        parameters=[
            {'semantic_json': semantic_json},
            {'floor': '1F'},
            {'goal_z': 0.0},
            {'map_yaml': map_yaml},
            {'robot_start_x': 21.0},
            {'robot_start_y': 38.0},
        ]
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        cleanup,
        declare_use_sim_time,
        gz_sim,
        robot_state_publisher,
        ros_gz_bridge,
        map_to_odom_tf,
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
        spawn_robot,      # delayed 5 s
    ])
