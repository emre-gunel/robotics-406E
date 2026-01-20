#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_manipulation = get_package_share_directory('litter_bot_manipulation')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Gripper Controller
        Node(
            package='litter_bot_manipulation',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'open_position': 0.019,
                'closed_position': -0.01,
                'grasp_position': 0.005,
                'move_time': 1.0,
            }]
        ),

        # Manipulation Node
        Node(
            package='litter_bot_manipulation',
            executable='manipulation_node',
            name='manipulation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'arm_speed': 0.5,
                'grasp_height_offset': 0.05,
                'approach_distance': 0.1,
            }]
        ),
    ])

