#!/usr/bin/env python3
"""
Simulation Only Launch File

Launches just the Gazebo simulation for testing and development.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('litter_bot_gazebo')
    pkg_description = get_package_share_directory('litter_bot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

