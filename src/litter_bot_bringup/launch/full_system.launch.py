#!/usr/bin/env python3
"""
Full System Bringup Launch File

Launches all components:
1. Gazebo simulation with TurtleBot3 Waffle + Med Çim world
2. Localization (AMCL with static map)
3. Nav2 Navigation Stack
4. Coverage Planner
5. Coordinator (state machine)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories - compute ALL paths upfront
    pkg_gazebo = get_package_share_directory('litter_bot_gazebo')
    pkg_nav = get_package_share_directory('litter_bot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Pre-compute all paths
    simulation_launch = os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')
    localization_launch = os.path.join(pkg_nav, 'launch', 'localization.launch.py')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    navigation_launch = os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')

    use_sim_time = 'true'

    return LaunchDescription([
        # Set TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        # 1. Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': '0.0',
                'y_pose': '0.0',
            }.items()
        ),

        # 2. Localization (AMCL with static map) - wait for Gazebo
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
            ]
        ),

        # 3. Nav2 Navigation Stack - wait for localization
        TimerAction(
            period=18.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params,
                    }.items()
                ),
            ]
        ),

        # 4. Coverage Planner - wait for Nav2
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='litter_bot_navigation',
                    executable='coverage_planner_node',
                    name='coverage_planner_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'area_width': 6.0,
                        'area_height': 4.0,
                        'area_origin_x': 0.5,
                        'area_origin_y': -2.0,
                        'lane_width': 1.0,
                        'waypoint_spacing': 2.0,
                    }]
                ),
            ]
        ),

        # 5. Perception - Color-based Detection (for Gazebo simulation)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='litter_bot_perception',
                    executable='depth_detection_node',
                    name='depth_detection_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'min_contour_area': 50,
                        'max_contour_area': 15000,
                        'detection_cooldown': 3.0,
                        'use_depth': True,
                    }]
                ),
            ]
        ),

        # 6. Litter Manager - simulates collection by deleting litter from Gazebo
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='litter_bot',
                    executable='litter_manager_node',
                    name='litter_manager_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'collection_distance': 0.3,  # meters
                    }]
                ),
            ]
        ),

        # 7. Coordinator - wait for all subsystems
        TimerAction(
            period=35.0,
            actions=[
                Node(
                    package='litter_bot',
                    executable='coordinator_node',
                    name='coordinator_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                    }]
                ),
            ]
        ),
    ])
