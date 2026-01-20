#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt',
            description='Path to YOLOv8 model'
        ),

        # Detection Node
        Node(
            package='litter_bot_perception',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'confidence_threshold': 0.5,
                'image_topic': '/camera/image_raw',
                'publish_annotated': True,
                'detect_humans': True,
                'detect_litter': True,
            }]
        ),

        # Litter Localizer Node
        Node(
            package='litter_bot_perception',
            executable='litter_localizer_node',
            name='litter_localizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'target_frame': 'base_footprint',
                'camera_frame': 'camera_rgb_optical_frame',
            }]
        ),
    ])

