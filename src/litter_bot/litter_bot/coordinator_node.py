#!/usr/bin/env python3
"""
Litter Bot Coordinator Node

Main state machine that coordinates all subsystems:
- Navigation (coverage path following)
- Perception (litter detection)
- Manipulation (pickup and deposit)

State Machine:
IDLE -> PATROL -> LITTER_DETECTED -> APPROACHING -> PICKING -> RETURNING -> PATROL
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient
from enum import Enum
import math
from typing import Optional


class RobotState(Enum):
    IDLE = "IDLE"
    PATROL = "PATROL"
    LITTER_DETECTED = "LITTER_DETECTED"
    APPROACHING = "APPROACHING"
    PICKING = "PICKING"
    RETURNING = "RETURNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


class NavigationStatus(Enum):
    IDLE = 0
    ACTIVE = 1
    SUCCEEDED = 2
    ABORTED = 3
    CANCELED = 4


class CoordinatorNode(Node):
    """
    Central coordinator that manages the robot's behavior using a state machine.
    """

    def __init__(self):
        super().__init__('coordinator_node')

        # Parameters
        self.declare_parameter('approach_distance', 0.5)
        self.declare_parameter('pickup_duration', 2.0)  # Simulated pickup time
        self.approach_distance = self.get_parameter('approach_distance').value
        self.pickup_duration = self.get_parameter('pickup_duration').value

        # State
        self.state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        
        # Litter handling
        self.litter_pose: Optional[PoseStamped] = None
        self.litter_handled = False  # Flag to prevent re-processing same litter
        
        # Position tracking
        self.current_pose: Optional[PoseStamped] = None
        self.saved_patrol_pose: Optional[PoseStamped] = None
        
        # Navigation status
        self.nav_status = NavigationStatus.IDLE
        
        # State entry flags (to run entry logic only once)
        self._state_entered = False
        
        # Pickup timing
        self._pickup_start_time = None
        
        # Debounce - ignore litter signals for N seconds after handling
        self._last_litter_handled_time = None
        self._litter_debounce_sec = 15.0
        
        # Statistics
        self.total_litter_collected = 0
        self.start_time = None

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Service clients
        self.start_coverage_client = self.create_client(
            Trigger, 'start_coverage', callback_group=self.cb_group
        )
        self.pause_coverage_client = self.create_client(
            Trigger, 'pause_coverage', callback_group=self.cb_group
        )
        self.resume_coverage_client = self.create_client(
            Trigger, 'resume_coverage', callback_group=self.cb_group
        )

        # Subscribers
        self.litter_detected_sub = self.create_subscription(
            Bool, '/litter_detected', self.litter_detected_callback, 10
        )
        self.litter_pose_sub = self.create_subscription(
            PoseStamped, '/litter_pose', self.litter_pose_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.start_mission_srv = self.create_service(
            Trigger, 'start_mission', self.start_mission_callback,
            callback_group=self.cb_group
        )
        self.stop_mission_srv = self.create_service(
            Trigger, 'stop_mission', self.stop_mission_callback,
            callback_group=self.cb_group
        )

        # State machine timer (runs every 100ms)
        self.state_timer = self.create_timer(
            0.1, self.state_machine_tick, callback_group=self.cb_group
        )

        # Status publish timer
        self.status_timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info('Coordinator Node initialized')
        self.get_logger().info('Waiting for mission start command...')

    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # ==================== CALLBACKS ====================

    def litter_detected_callback(self, msg: Bool):
        """Handle litter detection signal."""
        if not msg.data:
            return
            
        # Only accept during PATROL
        if self.state != RobotState.PATROL:
            return
        
        # Debounce - check if we recently handled litter
        if self._last_litter_handled_time is not None:
            elapsed = (self.get_clock().now() - self._last_litter_handled_time).nanoseconds / 1e9
            if elapsed < self._litter_debounce_sec:
                self.get_logger().debug(f'Ignoring litter signal (debounce: {elapsed:.1f}s < {self._litter_debounce_sec}s)')
                return
        
        # Check if we have a valid litter pose
        if self.litter_pose is None:
            self.get_logger().warn('Litter detected but no pose available')
            return
        
        self.get_logger().info('🗑️ Litter detected! Interrupting patrol...')
        self.change_state(RobotState.LITTER_DETECTED)

    def litter_pose_callback(self, msg: PoseStamped):
        """Store litter pose."""
        self.litter_pose = msg

    def odom_callback(self, msg: Odometry):
        """Track robot's current position."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.current_pose = pose

    # ==================== SERVICE CALLBACKS ====================

    def start_mission_callback(self, request, response):
        """Start the cleaning mission."""
        if self.state not in [RobotState.IDLE, RobotState.COMPLETED]:
            if self.state == RobotState.PATROL:
                response.success = True
                response.message = 'Mission already running'
                return response
            response.success = False
            response.message = f'Cannot start: robot is in {self.state.value} state'
            return response

        self.get_logger().info('🚀 Starting cleaning mission!')
        self.start_time = self.get_clock().now()
        self.total_litter_collected = 0
        self._last_litter_handled_time = None
        
        # Start coverage planner
        if self.call_service_sync(self.start_coverage_client):
            self.change_state(RobotState.PATROL)
            response.success = True
            response.message = 'Mission started'
        else:
            response.success = False
            response.message = 'Failed to start coverage planner'

        return response

    def stop_mission_callback(self, request, response):
        """Stop the cleaning mission."""
        self.get_logger().info('Stopping mission...')
        self.stop_robot()
        self.change_state(RobotState.IDLE)
        
        response.success = True
        response.message = f'Mission stopped. Collected {self.total_litter_collected} items.'
        return response

    # ==================== STATE MACHINE ====================

    def change_state(self, new_state: RobotState):
        """Change state with logging."""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self._state_entered = False  # Reset entry flag
            self.get_logger().info(f'State: {self.previous_state.value} -> {new_state.value}')

    def state_machine_tick(self):
        """Main state machine logic - called every 100ms."""
        
        # ===== IDLE =====
        if self.state == RobotState.IDLE:
            pass  # Wait for start command

        # ===== PATROL =====
        elif self.state == RobotState.PATROL:
            # Coverage planner handles navigation
            # Litter detection triggers state change via callback
            pass

        # ===== LITTER_DETECTED =====
        elif self.state == RobotState.LITTER_DETECTED:
            if not self._state_entered:
                self._state_entered = True
                
                # Save current position for return
                if self.current_pose is not None:
                    self.saved_patrol_pose = PoseStamped()
                    self.saved_patrol_pose.header = self.current_pose.header
                    self.saved_patrol_pose.pose = self.current_pose.pose
                    self.get_logger().info(
                        f'📍 Saved patrol position: ({self.saved_patrol_pose.pose.position.x:.2f}, '
                        f'{self.saved_patrol_pose.pose.position.y:.2f})'
                    )
                
                # Pause coverage
                self.call_service_sync(self.pause_coverage_client)
                
                # Start navigation to litter
                self.navigate_to_litter()
                self.change_state(RobotState.APPROACHING)

        # ===== APPROACHING =====
        elif self.state == RobotState.APPROACHING:
            if not self._state_entered:
                self._state_entered = True
                self.get_logger().info('🚶 Approaching litter...')
            
            # Wait for navigation to complete
            if self.nav_status == NavigationStatus.SUCCEEDED:
                self.get_logger().info('✓ Reached litter position')
                self.nav_status = NavigationStatus.IDLE
                self.change_state(RobotState.PICKING)
            elif self.nav_status == NavigationStatus.ABORTED:
                self.get_logger().warn('⚠️ Navigation aborted, proceeding to pickup anyway')
                self.nav_status = NavigationStatus.IDLE
                self.change_state(RobotState.PICKING)
            elif self.nav_status == NavigationStatus.CANCELED:
                self.get_logger().warn('Navigation canceled, returning to patrol')
                self.nav_status = NavigationStatus.IDLE
                self.change_state(RobotState.PATROL)
                self.call_service_sync(self.resume_coverage_client)

        # ===== PICKING =====
        elif self.state == RobotState.PICKING:
            if not self._state_entered:
                self._state_entered = True
                self._pickup_start_time = self.get_clock().now()
                self.get_logger().info('🤖 Picking up litter (simulated)...')
            
            # Simulate pickup duration
            elapsed = (self.get_clock().now() - self._pickup_start_time).nanoseconds / 1e9
            if elapsed >= self.pickup_duration:
                # Pickup complete!
                self.total_litter_collected += 1
                self._last_litter_handled_time = self.get_clock().now()
                self.litter_pose = None  # Clear litter target
                
                self.get_logger().info(f'✅ Pickup complete! Total: {self.total_litter_collected}')
                self.change_state(RobotState.RETURNING)

        # ===== RETURNING =====
        elif self.state == RobotState.RETURNING:
            if not self._state_entered:
                self._state_entered = True
                
                if self.saved_patrol_pose is not None:
                    self.get_logger().info(
                        f'🔙 Returning to patrol position: ({self.saved_patrol_pose.pose.position.x:.2f}, '
                        f'{self.saved_patrol_pose.pose.position.y:.2f})'
                    )
                    self.navigate_to_pose(self.saved_patrol_pose)
                else:
                    self.get_logger().warn('No saved position, resuming patrol directly')
                    self.change_state(RobotState.PATROL)
                    self.call_service_sync(self.resume_coverage_client)
            
            # Wait for navigation to complete
            if self.nav_status == NavigationStatus.SUCCEEDED:
                self.get_logger().info('✓ Returned to patrol position')
                self.nav_status = NavigationStatus.IDLE
                self.saved_patrol_pose = None
                self.change_state(RobotState.PATROL)
                self.call_service_sync(self.resume_coverage_client)
            elif self.nav_status in [NavigationStatus.ABORTED, NavigationStatus.CANCELED]:
                self.get_logger().warn('Return navigation failed, resuming patrol anyway')
                self.nav_status = NavigationStatus.IDLE
                self.saved_patrol_pose = None
                self.change_state(RobotState.PATROL)
                self.call_service_sync(self.resume_coverage_client)

        # ===== COMPLETED =====
        elif self.state == RobotState.COMPLETED:
            if not self._state_entered:
                self._state_entered = True
                self.print_mission_summary()

        # ===== ERROR =====
        elif self.state == RobotState.ERROR:
            if not self._state_entered:
                self._state_entered = True
                self.get_logger().error('Error state - stopping robot')
                self.stop_robot()

    # ==================== NAVIGATION ====================

    def navigate_to_litter(self):
        """Navigate to litter position with approach offset."""
        if self.litter_pose is None:
            self.get_logger().warn('No litter pose to navigate to')
            return

        # Create goal slightly before the litter
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Offset approach (approach from current direction)
        litter_x = self.litter_pose.pose.position.x
        litter_y = self.litter_pose.pose.position.y
        
        if self.current_pose is not None:
            # Calculate direction from robot to litter
            dx = litter_x - self.current_pose.pose.position.x
            dy = litter_y - self.current_pose.pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 0.01:
                # Approach from current direction
                goal_pose.pose.position.x = litter_x - (dx / dist) * self.approach_distance
                goal_pose.pose.position.y = litter_y - (dy / dist) * self.approach_distance
                
                # Face toward litter
                yaw = math.atan2(dy, dx)
                goal_pose.pose.orientation.z = math.sin(yaw / 2)
                goal_pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                goal_pose.pose.position.x = litter_x
                goal_pose.pose.position.y = litter_y
                goal_pose.pose.orientation.w = 1.0
        else:
            goal_pose.pose.position.x = litter_x - self.approach_distance
            goal_pose.pose.position.y = litter_y
            goal_pose.pose.orientation.w = 1.0
        
        goal_pose.pose.position.z = 0.0
        
        self.get_logger().info(
            f'🎯 Navigating to litter: ({goal_pose.pose.position.x:.2f}, '
            f'{goal_pose.pose.position.y:.2f})'
        )
        
        self.navigate_to_pose(goal_pose)

    def navigate_to_pose(self, goal_pose: PoseStamped):
        """Send navigation goal."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.nav_status = NavigationStatus.ABORTED
            return
        
        self.nav_status = NavigationStatus.ACTIVE
        
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Navigation goal rejected')
            self.nav_status = NavigationStatus.ABORTED
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """Handle navigation completion."""
        result = future.result()
        status = result.status
        
        # Status codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:
            self.nav_status = NavigationStatus.SUCCEEDED
        elif status == 5:
            self.nav_status = NavigationStatus.CANCELED
        else:
            self.nav_status = NavigationStatus.ABORTED

    # ==================== HELPERS ====================

    def stop_robot(self):
        """Stop robot movement."""
        self.cmd_vel_pub.publish(Twist())

    def call_service_sync(self, client, timeout=5.0) -> bool:
        """Call a Trigger service synchronously (non-blocking wait)."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {client.srv_name} not available')
            return False
        
        future = client.call_async(Trigger.Request())
        
        # Non-blocking wait
        start = self.get_clock().now()
        while not future.done():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn(f'Service {client.srv_name} timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = future.result()
        if result is not None:
            self.get_logger().debug(f'Service result: {result.message}')
            return result.success or 'already' in result.message.lower()
        return False

    def print_mission_summary(self):
        """Print mission statistics."""
        if self.start_time is None:
            return
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🏁 MISSION COMPLETE')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Total litter collected: {self.total_litter_collected}')
        self.get_logger().info(f'Mission duration: {elapsed:.1f} seconds')
        if elapsed > 0:
            self.get_logger().info(f'Rate: {self.total_litter_collected / elapsed * 60:.1f} items/min')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    
    node = CoordinatorNode()
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
