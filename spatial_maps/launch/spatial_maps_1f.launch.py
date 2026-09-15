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
                             IncludeLaunchDescription, SetEnvironmentVariable,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
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
    # 1F overlay: footprint polygon, Smac State Lattice planner, smooth inflation,
    # doorway speed filter. Later files in a parameters list override earlier ones.
    nav2_params_1f  = os.path.join(spatial_maps_share, 'config', 'nav2_params_1f.yaml')
    map_yaml_global = os.path.join(spatial_maps_share, 'maps',   '1F_05.yaml')
    speed_mask_yaml = os.path.join(spatial_maps_share, 'maps',   '1F_speed_mask.yaml')
    semantic_json = '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json'

    # Expose spatial_maps/models/ to Gz Sim so model://pgm_walls_1f resolves.
    # The models/ directory is not installed to the ROS2 share path, so we
    # derive its location from this launch file's real path (symlink-safe).
    _pkg_src = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
    _models_dir = os.path.join(_pkg_src, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        _models_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # headless:=true runs Gz Sim server-only (no 3D GUI window), which frees the
    # renderer's CPU budget and raises the real-time factor — the sweep then
    # consumes its sim-time budget in far less wall-clock time. Watch in RViz.
    headless = LaunchConfiguration('headless', default='false')

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

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gz Sim server-only (no GUI) for a higher real-time factor')

    # ── Gz Sim ────────────────────────────────────────────────────────────────
    # '-r' runs immediately; '-s' (server-only) is appended when headless:=true.
    gz_args = PythonExpression(
        ["'", world_file, " -r' + (' -s' if '", headless, "' == 'true' else '')"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
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

    # ── Spawn robot (delayed 30 s — 1F loads 1108 PGM wall collision boxes)  ──
    spawn_robot = TimerAction(
        period=30.0,
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

    # ── map → odom ────────────────────────────────────────────────────────────
    # AMCL publishes this transform on 1F (tf_broadcast: true in the 1F overlay).
    # The static publisher that used to anchor it at the spawn point is gone: two
    # publishers of the same transform fight, and odometry drift went uncorrected,
    # which drifted the robot 0.6 m off course over 35 m on 15 September 2026.

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

    # ── AMCL ─────────────────────────────────────────────────────────────────
    # PGM-derived collision walls now align with the map, so LiDAR can match.
    # static_transform_publisher above bootstraps map->odom at spawn; AMCL
    # takes over once it has enough particles (dynamic TF overrides static).
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}]
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
            # The map is latched (transient local), so one publish is enough. At the
            # 1 Hz default AMCL re-received the map every second and reinitialised,
            # which left map->odom stale and aborted every goal (15 September 2026).
            {'publish_rate_hz': 0.001},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 0.05 m copy of the map for the global costmap, which must match the 5 cm
    # lattice primitives. AMCL and the local costmap keep the 0.02 m /map.
    map_publisher_global = Node(
        package='spatial_maps',
        executable='map_publisher_node.py',
        name='map_publisher_global',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_global},
            {'frame_id': 'map'},
            {'publish_rate_hz': 0.001},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/map', '/map_global')]
    )

    # ── Doorway speed filter (mask server + filter info server) ──────────────
    speed_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_filter_mask_server',
        output='screen',
        parameters=[nav2_params_1f, {'use_sim_time': use_sim_time},
                    {'yaml_filename': speed_mask_yaml}]
    )

    speed_costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_costmap_filter_info_server',
        output='screen',
        parameters=[nav2_params_1f, {'use_sim_time': use_sim_time}]
    )

    # ── Nav2 stack ────────────────────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params, nav2_params_1f, {'use_sim_time': use_sim_time}]
    )

    # Delayed 42 s — robot spawns at 30 s, odom TF needs ~12 s to flow
    nav2_lifecycle_manager = TimerAction(
        period=42.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                # Matches spatial_maps.launch.py. With the default 4 s, AMCL
                # missed heartbeats and crash-looped once the 1F world ran near
                # real time (mesh collision, RT ~1.0), shutting down Nav2.
                {'bond_timeout': 30.0},
                {'node_names': [
                    'speed_filter_mask_server',
                    'speed_costmap_filter_info_server',
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
        set_gz_resource_path,   # must precede gz_sim so Gz finds model://pgm_walls_1f
        cleanup,
        declare_use_sim_time,
        declare_headless,
        gz_sim,
        robot_state_publisher,
        ros_gz_bridge,
        amcl,
        odom_tf_republisher,
        joint_state_relay,
        map_publisher,
        map_publisher_global,
        speed_filter_mask_server,
        speed_costmap_filter_info_server,
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
