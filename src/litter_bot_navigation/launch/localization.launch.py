#!/usr/bin/env python3
"""
Localization launch file using AMCL with pre-generated map.
Use this instead of SLAM when you have a static map of the environment.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('litter_bot_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Map file path
    map_yaml = os.path.join(pkg_nav, 'maps', 'med_cim_map.yaml')
    
    # Nav2 params
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    
    # Rewrite params with use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Map server - serves the static map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml},
                {'topic_name': 'map'},
                {'frame_id': 'map'}
            ]
        ),
        
        # AMCL - Adaptive Monte Carlo Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params]
        ),
        
        # Lifecycle manager for map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ])


