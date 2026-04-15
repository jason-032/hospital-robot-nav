#!/usr/bin/env python3
"""Single floor map server launch for 6F"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('spatial_maps')
    
    map_file = LaunchConfiguration('map_file')
    frame_id = LaunchConfiguration('frame_id', default='map_6f')
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_share, 'maps', '6F.yaml'),
        description='Path to map YAML file'
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_6f',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'frame_id': frame_id}
        ]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_6f',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server_6f']}
        ]
    )
    
    return LaunchDescription([
        declare_map_file,
        map_server_node,
        lifecycle_manager
    ])
