#!/usr/bin/env python3
"""
Manipulation Node

Handles the complete pick-and-place pipeline for litter collection:
1. Receive target pose from perception
2. Plan arm trajectory to pre-grasp position
3. Approach and grasp
4. Lift and deposit in collection bin
5. Return to home position
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from enum import Enum
import math
import time
from typing import List, Optional
from dataclasses import dataclass


class ManipState(Enum):
    IDLE = 0
    MOVING_TO_PREGRASP = 1
    APPROACHING = 2
    GRASPING = 3
    LIFTING = 4
    MOVING_TO_BIN = 5
    RELEASING = 6
    RETURNING_HOME = 7
    ERROR = 8


@dataclass
class ArmPose:
    """Represents arm joint positions."""
    joint1: float = 0.0
    joint2: float = 0.0
    joint3: float = 0.0
    joint4: float = 0.0

    def to_list(self) -> List[float]:
        return [self.joint1, self.joint2, self.joint3, self.joint4]


class ManipulationNode(Node):
    """
    Controls the OpenMANIPULATOR-X arm for litter pickup.
    """

    # Predefined poses
    HOME_POSE = ArmPose(0.0, -1.0, 0.3, 0.7)
    PREGRASP_POSE = ArmPose(0.0, 0.3, 0.8, 0.5)
    DEPOSIT_POSE = ArmPose(3.14, -0.5, 0.5, 0.5)  # Rotated to back (collection bin)

    def __init__(self):
        super().__init__('manipulation_node')

        # Declare parameters
        self.declare_parameter('arm_speed', 0.5)  # Speed factor
        self.declare_parameter('grasp_height_offset', 0.05)  # Height above object for pre-grasp
        self.declare_parameter('approach_distance', 0.1)  # Approach distance

        # Get parameters
        self.arm_speed = self.get_parameter('arm_speed').value
        self.grasp_height_offset = self.get_parameter('grasp_height_offset').value
        self.approach_distance = self.get_parameter('approach_distance').value

        # State
        self.state = ManipState.IDLE
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.target_litter_pose: Optional[PoseStamped] = None
        self.pickup_in_progress = False
        self.successful_pickups = 0
        self.failed_pickups = 0

        # Callback groups
        self.cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.litter_pose_sub = self.create_subscription(
            PoseStamped, '/litter_pose_3d', self.litter_pose_callback, 10,
            callback_group=self.cb_group
        )

        # Publishers
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.pickup_complete_pub = self.create_publisher(
            Bool, '/pickup_complete', 10
        )
        self.manipulation_status_pub = self.create_publisher(
            Bool, '/manipulation_active', 10
        )

        # Service clients for gripper
        self.gripper_open_client = self.create_client(
            Trigger, 'gripper/open', callback_group=self.cb_group
        )
        self.gripper_close_client = self.create_client(
            Trigger, 'gripper/close', callback_group=self.cb_group
        )
        self.gripper_grasp_client = self.create_client(
            Trigger, 'gripper/grasp', callback_group=self.cb_group
        )

        # Services
        self.pick_srv = self.create_service(
            Trigger, 'pick_litter', self.pick_litter_callback,
            callback_group=self.cb_group
        )
        self.home_srv = self.create_service(
            Trigger, 'arm/home', self.go_home_callback,
            callback_group=self.cb_group
        )

        # Status timer
        self.status_timer = self.create_timer(0.5, self.publish_status)

        # Move to home on startup (one-shot)
        self.initial_home_done = False
        self.initial_home_timer = self.create_timer(2.0, self.initial_home, callback_group=self.cb_group)

        self.get_logger().info('Manipulation Node initialized')

    def initial_home(self):
        """Move arm to home position on startup (one-shot)."""
        if self.initial_home_done:
            return
        self.initial_home_done = True
        
        self.get_logger().info('Moving arm to home position...')
        self.move_to_pose(self.HOME_POSE, duration=3.0)
        self.call_gripper_open()
        
        # Cancel the timer
        self.initial_home_timer.cancel()

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions."""
        joint_map = {'joint1': 0, 'joint2': 1, 'joint3': 2, 'joint4': 3}
        for name, idx in joint_map.items():
            if name in msg.name:
                joint_idx = msg.name.index(name)
                self.current_joint_positions[idx] = msg.position[joint_idx]

    def litter_pose_callback(self, msg: PoseStamped):
        """Store latest litter pose for pickup."""
        if not self.pickup_in_progress:
            self.target_litter_pose = msg
            self.get_logger().debug(
                f'New litter pose: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
            )

    def publish_status(self):
        """Publish manipulation status."""
        msg = Bool()
        msg.data = self.pickup_in_progress
        self.manipulation_status_pub.publish(msg)

    def pick_litter_callback(self, request, response):
        """Service callback to initiate pickup sequence."""
        if self.pickup_in_progress:
            response.success = False
            response.message = 'Pickup already in progress'
            return response

        if self.target_litter_pose is None:
            response.success = False
            response.message = 'No litter pose available'
            return response

        self.get_logger().info('Starting pickup sequence...')
        self.pickup_in_progress = True
        
        success = self.execute_pickup_sequence()
        
        self.pickup_in_progress = False
        
        if success:
            self.successful_pickups += 1
            response.success = True
            response.message = f'Pickup successful! Total: {self.successful_pickups}'
        else:
            self.failed_pickups += 1
            response.success = False
            response.message = f'Pickup failed. Failures: {self.failed_pickups}'

        # Signal pickup completion
        complete_msg = Bool()
        complete_msg.data = success
        self.pickup_complete_pub.publish(complete_msg)

        return response

    def go_home_callback(self, request, response):
        """Service callback to move arm to home position."""
        self.get_logger().info('Moving to home position...')
        success = self.move_to_pose(self.HOME_POSE, duration=2.0)
        response.success = success
        response.message = 'Moved to home' if success else 'Failed to move home'
        return response

    def execute_pickup_sequence(self) -> bool:
        """
        Execute the full pickup sequence:
        1. Move to pre-grasp position
        2. Open gripper
        3. Approach target
        4. Close gripper (grasp)
        5. Lift object
        6. Move to deposit bin
        7. Open gripper (release)
        8. Return to home
        """
        try:
            # Step 1: Pre-grasp position
            self.state = ManipState.MOVING_TO_PREGRASP
            self.get_logger().info('Step 1: Moving to pre-grasp position')
            
            pregrasp = self.compute_pregrasp_pose()
            if not self.move_to_pose(pregrasp, duration=2.0):
                raise Exception('Failed to reach pre-grasp position')
            time.sleep(0.5)

            # Step 2: Open gripper
            self.get_logger().info('Step 2: Opening gripper')
            if not self.call_gripper_open():
                raise Exception('Failed to open gripper')
            time.sleep(0.5)

            # Step 3: Approach (lower to grasp position)
            self.state = ManipState.APPROACHING
            self.get_logger().info('Step 3: Approaching target')
            
            grasp_pose = self.compute_grasp_pose()
            if not self.move_to_pose(grasp_pose, duration=1.5):
                raise Exception('Failed to approach target')
            time.sleep(0.3)

            # Step 4: Grasp
            self.state = ManipState.GRASPING
            self.get_logger().info('Step 4: Grasping object')
            if not self.call_gripper_grasp():
                raise Exception('Failed to grasp')
            time.sleep(0.8)

            # Step 5: Lift
            self.state = ManipState.LIFTING
            self.get_logger().info('Step 5: Lifting object')
            if not self.move_to_pose(pregrasp, duration=1.5):
                raise Exception('Failed to lift object')
            time.sleep(0.3)

            # Step 6: Move to deposit position
            self.state = ManipState.MOVING_TO_BIN
            self.get_logger().info('Step 6: Moving to deposit bin')
            if not self.move_to_pose(self.DEPOSIT_POSE, duration=2.0):
                raise Exception('Failed to reach deposit position')
            time.sleep(0.5)

            # Step 7: Release
            self.state = ManipState.RELEASING
            self.get_logger().info('Step 7: Releasing object')
            if not self.call_gripper_open():
                raise Exception('Failed to release')
            time.sleep(0.5)

            # Step 8: Return home
            self.state = ManipState.RETURNING_HOME
            self.get_logger().info('Step 8: Returning to home position')
            if not self.move_to_pose(self.HOME_POSE, duration=2.0):
                raise Exception('Failed to return home')

            self.state = ManipState.IDLE
            self.get_logger().info('Pickup sequence completed successfully!')
            return True

        except Exception as e:
            self.get_logger().error(f'Pickup failed: {e}')
            self.state = ManipState.ERROR
            
            # Try to recover by going home
            self.call_gripper_open()
            self.move_to_pose(self.HOME_POSE, duration=3.0)
            self.state = ManipState.IDLE
            return False

    def compute_pregrasp_pose(self) -> ArmPose:
        """
        Compute pre-grasp joint positions based on target litter pose.
        
        In a full implementation, this would use inverse kinematics.
        For simplicity, we use a fixed pre-grasp pose with slight adjustments.
        """
        if self.target_litter_pose is None:
            return self.PREGRASP_POSE

        # Get target position
        x = self.target_litter_pose.pose.position.x
        y = self.target_litter_pose.pose.position.y
        
        # Simple angle calculation for joint1 (base rotation)
        base_angle = math.atan2(y, x)
        
        # Clamp to joint limits
        base_angle = max(-2.8, min(2.8, base_angle))
        
        return ArmPose(
            joint1=base_angle,
            joint2=0.2,
            joint3=0.6,
            joint4=0.8
        )

    def compute_grasp_pose(self) -> ArmPose:
        """
        Compute grasp joint positions (lower than pre-grasp).
        """
        pregrasp = self.compute_pregrasp_pose()
        
        # Lower the arm for grasping
        return ArmPose(
            joint1=pregrasp.joint1,
            joint2=0.5,   # More extended
            joint3=0.9,
            joint4=0.4
        )

    def move_to_pose(self, pose: ArmPose, duration: float = 2.0) -> bool:
        """
        Send trajectory command to move arm to specified pose.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = pose.to_list()
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        trajectory.points.append(point)

        self.arm_trajectory_pub.publish(trajectory)
        
        # Wait for execution
        time.sleep(duration + 0.5)
        return True

    def call_gripper_open(self) -> bool:
        """Call gripper open service."""
        if not self.gripper_open_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gripper open service not available')
            return False
        
        future = self.gripper_open_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success
        return False

    def call_gripper_grasp(self) -> bool:
        """Call gripper grasp service."""
        if not self.gripper_grasp_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gripper grasp service not available')
            return False
        
        future = self.gripper_grasp_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success
        return False


def main(args=None):
    rclpy.init(args=args)
    
    node = ManipulationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

