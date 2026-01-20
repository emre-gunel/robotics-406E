#!/usr/bin/env python3
"""
Gripper Controller Node

Controls the parallel gripper for litter pickup.
Uses joint position commands to open/close the gripper.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from enum import Enum
import time


class GripperState(Enum):
    OPEN = 0
    CLOSED = 1
    MOVING = 2
    UNKNOWN = 3


class GripperControllerNode(Node):
    """
    Simple gripper controller for parallel gripper.
    """

    def __init__(self):
        super().__init__('gripper_controller')

        # Declare parameters
        self.declare_parameter('open_position', 0.019)   # Max open
        self.declare_parameter('closed_position', -0.01)  # Closed
        self.declare_parameter('grasp_position', 0.005)   # Grasping small objects
        self.declare_parameter('move_time', 1.0)         # Time to move gripper

        # Get parameters
        self.open_pos = self.get_parameter('open_position').value
        self.closed_pos = self.get_parameter('closed_position').value
        self.grasp_pos = self.get_parameter('grasp_position').value
        self.move_time = self.get_parameter('move_time').value

        # State
        self.current_position = 0.0
        self.target_position = 0.0
        self.state = GripperState.UNKNOWN

        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publishers - Direct joint command
        self.gripper_cmd_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        
        # Alternative: Direct position command for Gazebo
        self.gripper_pos_pub = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10
        )

        # Services
        self.open_srv = self.create_service(
            Trigger, 'gripper/open', self.open_callback,
            callback_group=self.callback_group
        )
        self.close_srv = self.create_service(
            Trigger, 'gripper/close', self.close_callback,
            callback_group=self.callback_group
        )
        self.grasp_srv = self.create_service(
            Trigger, 'gripper/grasp', self.grasp_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Gripper Controller initialized')
        self.get_logger().info(f'Positions - Open: {self.open_pos}, Closed: {self.closed_pos}, Grasp: {self.grasp_pos}')

    def joint_state_callback(self, msg: JointState):
        """Update current gripper position from joint states."""
        try:
            if 'gripper_left_joint' in msg.name:
                idx = msg.name.index('gripper_left_joint')
                self.current_position = msg.position[idx]
                
                # Determine state
                if abs(self.current_position - self.open_pos) < 0.002:
                    self.state = GripperState.OPEN
                elif abs(self.current_position - self.closed_pos) < 0.002:
                    self.state = GripperState.CLOSED
                else:
                    self.state = GripperState.MOVING
        except (ValueError, IndexError):
            pass

    def send_gripper_command(self, position: float) -> bool:
        """
        Send position command to gripper joints.
        Both gripper fingers move symmetrically.
        """
        self.target_position = position
        
        # Method 1: JointTrajectory command
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]  # Symmetric movement
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = int(self.move_time)
        point.time_from_start.nanosec = int((self.move_time % 1) * 1e9)
        
        trajectory.points.append(point)
        self.gripper_cmd_pub.publish(trajectory)

        # Method 2: Direct position command (backup for Gazebo)
        cmd = Float64MultiArray()
        cmd.data = [position, position]
        self.gripper_pos_pub.publish(cmd)

        self.get_logger().info(f'Gripper command sent: position={position:.3f}')
        return True

    def open_callback(self, request, response):
        """Service callback to open gripper."""
        self.get_logger().info('Opening gripper...')
        success = self.send_gripper_command(self.open_pos)
        response.success = success
        response.message = 'Gripper opening' if success else 'Failed to open gripper'
        return response

    def close_callback(self, request, response):
        """Service callback to close gripper."""
        self.get_logger().info('Closing gripper...')
        success = self.send_gripper_command(self.closed_pos)
        response.success = success
        response.message = 'Gripper closing' if success else 'Failed to close gripper'
        return response

    def grasp_callback(self, request, response):
        """Service callback to grasp (partial close for objects)."""
        self.get_logger().info('Grasping...')
        success = self.send_gripper_command(self.grasp_pos)
        response.success = success
        response.message = 'Gripper grasping' if success else 'Failed to grasp'
        return response

    def is_open(self) -> bool:
        """Check if gripper is open."""
        return self.state == GripperState.OPEN

    def is_closed(self) -> bool:
        """Check if gripper is closed."""
        return self.state == GripperState.CLOSED


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

