#!/usr/bin/env python3
"""
Litter Manager Node

Monitors robot position and litter positions. When the robot gets close 
enough to a litter (distance <= 0.3m), it deletes the litter from Gazebo.

This simulates the robot "collecting" the litter.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteEntity
import math
from typing import Dict, Optional


class LitterManagerNode(Node):
    """
    Manages litter collection in Gazebo simulation.
    When robot gets within threshold distance, litter is deleted.
    """

    def __init__(self):
        super().__init__('litter_manager_node')

        # Parameters
        self.declare_parameter('collection_distance', 0.3)  # meters
        
        self.collection_distance = self.get_parameter('collection_distance').value
        
        # Robot current position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Litter positions (from Gazebo world file)
        # Format: {name: {'x': x, 'y': y, 'collected': False}}
        # 30 litters in 5x6 grid pattern
        self.litters: Dict[str, Dict] = {
            # Row 1 (y = -1.6)
            'litter_01': {'x': 1.0, 'y': -1.6, 'collected': False},
            'litter_02': {'x': 2.0, 'y': -1.6, 'collected': False},
            'litter_03': {'x': 3.0, 'y': -1.6, 'collected': False},
            'litter_04': {'x': 4.0, 'y': -1.6, 'collected': False},
            'litter_05': {'x': 5.0, 'y': -1.6, 'collected': False},
            'litter_06': {'x': 6.0, 'y': -1.6, 'collected': False},
            # Row 2 (y = -0.8)
            'litter_07': {'x': 1.0, 'y': -0.8, 'collected': False},
            'litter_08': {'x': 2.0, 'y': -0.8, 'collected': False},
            'litter_09': {'x': 3.0, 'y': -0.8, 'collected': False},
            'litter_10': {'x': 4.0, 'y': -0.8, 'collected': False},
            'litter_11': {'x': 5.0, 'y': -0.8, 'collected': False},
            'litter_12': {'x': 6.0, 'y': -0.8, 'collected': False},
            # Row 3 (y = 0.0)
            'litter_13': {'x': 1.0, 'y': 0.0, 'collected': False},
            'litter_14': {'x': 2.0, 'y': 0.0, 'collected': False},
            'litter_15': {'x': 3.0, 'y': 0.0, 'collected': False},
            'litter_16': {'x': 4.0, 'y': 0.0, 'collected': False},
            'litter_17': {'x': 5.0, 'y': 0.0, 'collected': False},
            'litter_18': {'x': 6.0, 'y': 0.0, 'collected': False},
            # Row 4 (y = 0.8)
            'litter_19': {'x': 1.0, 'y': 0.8, 'collected': False},
            'litter_20': {'x': 2.0, 'y': 0.8, 'collected': False},
            'litter_21': {'x': 3.0, 'y': 0.8, 'collected': False},
            'litter_22': {'x': 4.0, 'y': 0.8, 'collected': False},
            'litter_23': {'x': 5.0, 'y': 0.8, 'collected': False},
            'litter_24': {'x': 6.0, 'y': 0.8, 'collected': False},
            # Row 5 (y = 1.6)
            'litter_25': {'x': 1.0, 'y': 1.6, 'collected': False},
            'litter_26': {'x': 2.0, 'y': 1.6, 'collected': False},
            'litter_27': {'x': 3.0, 'y': 1.6, 'collected': False},
            'litter_28': {'x': 4.0, 'y': 1.6, 'collected': False},
            'litter_29': {'x': 5.0, 'y': 1.6, 'collected': False},
            'litter_30': {'x': 6.0, 'y': 1.6, 'collected': False},
        }
        
        # Statistics
        self.total_collected = 0
        
        # Callback group
        self.cb_group = ReentrantCallbackGroup()
        
        # Service client for deleting entities
        self.delete_entity_client = self.create_client(
            DeleteEntity, '/delete_entity',
            callback_group=self.cb_group
        )
        
        # Subscribe to robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Timer to check distances periodically
        self.check_timer = self.create_timer(0.5, self.check_litter_distances)
        
        self.get_logger().info('Litter Manager Node initialized')
        self.get_logger().info(f'Collection distance: {self.collection_distance}m')
        self.get_logger().info(f'Monitoring {len(self.litters)} litters')
        
        # Wait for delete service
        self.get_logger().info('Waiting for /delete_entity service...')
        
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Debug: Log position occasionally
        if not hasattr(self, '_odom_counter'):
            self._odom_counter = 0
        self._odom_counter += 1
        if self._odom_counter % 50 == 0:  # Every 50 messages (~5 seconds at 10Hz)
            self.get_logger().debug(
                f'Robot at ({self.robot_x:.2f}, {self.robot_y:.2f})'
            )
        
    def calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate 2D Euclidean distance."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def check_litter_distances(self):
        """Check distance to all uncollected litters."""
        closest_distance = float('inf')
        closest_litter = None
        
        for litter_name, litter_info in self.litters.items():
            # Skip already collected litters
            if litter_info['collected']:
                continue
                
            # Calculate distance
            distance = self.calculate_distance(
                self.robot_x, self.robot_y,
                litter_info['x'], litter_info['y']
            )
            
            # Track closest
            if distance < closest_distance:
                closest_distance = distance
                closest_litter = litter_name
            
            # Check if within collection range
            if distance <= self.collection_distance:
                self.get_logger().info(
                    f'🎯 Robot close to {litter_name} (distance: {distance:.2f}m) - Collecting!'
                )
                self.collect_litter(litter_name)
        
        # Debug: Log closest litter occasionally
        if closest_litter and not hasattr(self, '_check_counter'):
            self._check_counter = 0
        if closest_litter:
            self._check_counter = getattr(self, '_check_counter', 0) + 1
            if self._check_counter % 10 == 0:  # Every 10 checks (~5 seconds)
                self.get_logger().info(
                    f'📍 Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                    f'Closest: {closest_litter} at {closest_distance:.2f}m | '
                    f'Collected: {self.total_collected}/{len(self.litters)}'
                )
    
    def collect_litter(self, litter_name: str):
        """
        Collect litter by deleting it from Gazebo.
        
        Args:
            litter_name: Name of the litter entity in Gazebo
        """
        # Mark as collected immediately to prevent double-collection
        self.litters[litter_name]['collected'] = True
        
        # Delete from Gazebo
        if not self.delete_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Delete entity service not available!')
            return
        
        # Create request
        request = DeleteEntity.Request()
        request.name = litter_name
        
        # Call service asynchronously
        future = self.delete_entity_client.call_async(request)
        future.add_done_callback(
            lambda f: self.delete_callback(f, litter_name)
        )
    
    def delete_callback(self, future, litter_name: str):
        """Handle delete entity service response."""
        try:
            response = future.result()
            if response.success:
                self.total_collected += 1
                self.get_logger().info(
                    f'✅ Successfully collected {litter_name}! '
                    f'Total collected: {self.total_collected}/{len(self.litters)}'
                )
            else:
                self.get_logger().error(
                    f'❌ Failed to delete {litter_name}: {response.status_message}'
                )
                # Mark as not collected so we can retry
                self.litters[litter_name]['collected'] = False
        except Exception as e:
            self.get_logger().error(f'Error in delete callback: {e}')
            self.litters[litter_name]['collected'] = False


def main(args=None):
    rclpy.init(args=args)
    
    node = LitterManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

