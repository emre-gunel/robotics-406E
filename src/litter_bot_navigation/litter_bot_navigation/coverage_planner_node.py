#!/usr/bin/env python3
"""
Coverage Path Planner Node

Generates and executes a coverage path (waypoint-based S-pattern)
to ensure the robot traverses the entire operation area.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math
from enum import Enum
from typing import List, Tuple


class CoverageState(Enum):
    IDLE = 0
    COVERING = 1
    PAUSED = 2
    COMPLETED = 3


class CoveragePlannerNode(Node):
    """
    Coverage path planner that generates S-pattern waypoints
    and navigates through them using Nav2.
    """

    def __init__(self):
        super().__init__('coverage_planner_node')

        # Declare parameters
        self.declare_parameter('area_width', 18.0)      # meters (20m - margins)
        self.declare_parameter('area_height', 8.0)      # meters (10m - margins)
        self.declare_parameter('area_origin_x', -9.0)   # start x
        self.declare_parameter('area_origin_y', -4.0)   # start y
        self.declare_parameter('lane_width', 1.0)       # distance between lanes
        self.declare_parameter('waypoint_spacing', 2.0) # distance between waypoints in lane

        # Get parameters
        self.area_width = self.get_parameter('area_width').value
        self.area_height = self.get_parameter('area_height').value
        self.origin_x = self.get_parameter('area_origin_x').value
        self.origin_y = self.get_parameter('area_origin_y').value
        self.lane_width = self.get_parameter('lane_width').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value

        # State
        self.state = CoverageState.IDLE
        self.current_waypoint_idx = 0
        self.waypoints: List[PoseStamped] = []
        self.litter_detected = False
        
        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Action clients
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self._waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self.callback_group
        )

        # Publishers
        self.coverage_status_pub = self.create_publisher(Bool, '/coverage_active', 10)
        
        # Services
        self.start_srv = self.create_service(
            Trigger, 'start_coverage', self.start_coverage_callback,
            callback_group=self.callback_group
        )
        self.pause_srv = self.create_service(
            Trigger, 'pause_coverage', self.pause_coverage_callback,
            callback_group=self.callback_group
        )
        self.resume_srv = self.create_service(
            Trigger, 'resume_coverage', self.resume_coverage_callback,
            callback_group=self.callback_group
        )
        
        # Subscribers - pause when litter is detected
        self.litter_sub = self.create_subscription(
            Bool, '/litter_detected', self.litter_callback, 10,
            callback_group=self.callback_group
        )

        # Timer to publish status
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Coverage Planner Node initialized')
        self.get_logger().info(f'Area: {self.area_width}x{self.area_height}m, '
                               f'Origin: ({self.origin_x}, {self.origin_y})')

    def generate_coverage_waypoints(self) -> List[PoseStamped]:
        """
        Generate S-pattern (boustrophedon) waypoints for coverage.
        """
        waypoints = []
        num_lanes = int(self.area_height / self.lane_width)
        
        for lane_idx in range(num_lanes):
            y = self.origin_y + lane_idx * self.lane_width + self.lane_width / 2
            
            # Determine direction (alternating for S-pattern)
            if lane_idx % 2 == 0:
                # Left to right
                x_start = self.origin_x
                x_end = self.origin_x + self.area_width
                x_step = self.waypoint_spacing
            else:
                # Right to left
                x_start = self.origin_x + self.area_width
                x_end = self.origin_x
                x_step = -self.waypoint_spacing
            
            # Generate waypoints along the lane
            x = x_start
            while (x_step > 0 and x <= x_end) or (x_step < 0 and x >= x_end):
                pose = self.create_pose(x, y, 0.0 if x_step > 0 else math.pi)
                waypoints.append(pose)
                x += x_step
            
            # Ensure we have the end point
            if waypoints and abs(waypoints[-1].pose.position.x - x_end + x_step) > 0.1:
                pose = self.create_pose(x_end - x_step, y, 0.0 if x_step > 0 else math.pi)
                waypoints.append(pose)

        self.get_logger().info(f'Generated {len(waypoints)} waypoints for coverage')
        return waypoints

    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def start_coverage_callback(self, request, response):
        """Service callback to start coverage."""
        if self.state == CoverageState.COVERING:
            response.success = False
            response.message = 'Coverage already in progress'
            return response
        
        self.get_logger().info('Starting coverage path...')
        self.waypoints = self.generate_coverage_waypoints()
        self.current_waypoint_idx = 0
        self.state = CoverageState.COVERING
        
        # Start navigation
        self.navigate_to_next_waypoint()
        
        response.success = True
        response.message = f'Started coverage with {len(self.waypoints)} waypoints'
        return response

    def pause_coverage_callback(self, request, response):
        """Service callback to pause coverage."""
        if self.state != CoverageState.COVERING:
            response.success = False
            response.message = 'Not currently covering'
            return response
        
        self.state = CoverageState.PAUSED
        # Note: In a full implementation, we would cancel the current navigation goal
        
        response.success = True
        response.message = 'Coverage paused'
        return response

    def resume_coverage_callback(self, request, response):
        """Service callback to resume coverage."""
        if self.state != CoverageState.PAUSED:
            response.success = False
            response.message = 'Not currently paused'
            return response
        
        self.state = CoverageState.COVERING
        self.navigate_to_next_waypoint()
        
        response.success = True
        response.message = 'Coverage resumed'
        return response

    def litter_callback(self, msg: Bool):
        """Callback when litter is detected - pause for pickup."""
        if msg.data and self.state == CoverageState.COVERING:
            self.litter_detected = True
            self.get_logger().info('Litter detected! Pausing for pickup...')
            # The main coordinator will handle the pickup and resume

    def navigate_to_next_waypoint(self):
        """Send the robot to the next waypoint."""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.state = CoverageState.COMPLETED
            self.get_logger().info('Coverage completed!')
            return
        
        if self.state != CoverageState.COVERING:
            return
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        waypoint.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
        )
        
        # Wait for action server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return
        
        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from Nav2."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            # Try next waypoint anyway
            self.current_waypoint_idx += 1
            self.navigate_to_next_waypoint()
            return
        
        self.get_logger().debug('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        # Check navigation result
        # Status codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✓ Reached waypoint {self.current_waypoint_idx + 1}')
        elif status == 6:  # ABORTED (obstacle, planning failed, etc.)
            self.get_logger().warning(f'✗ Waypoint {self.current_waypoint_idx + 1} aborted (obstacle?), skipping...')
        elif status == 5:  # CANCELED
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1} canceled')
        else:
            self.get_logger().warning(f'Waypoint {self.current_waypoint_idx + 1} finished with status {status}')
        
        # Move to next waypoint regardless of result
        self.current_waypoint_idx += 1
        
        # Small delay before next waypoint
        self._next_wp_timer = self.create_timer(0.5, self.navigate_to_next_waypoint_once)

    def navigate_to_next_waypoint_once(self):
        """One-shot timer callback to navigate to next waypoint."""
        self.navigate_to_next_waypoint()
        # Destroy timer to prevent memory leak
        if hasattr(self, '_next_wp_timer'):
            self.destroy_timer(self._next_wp_timer)

    def publish_status(self):
        """Publish coverage status."""
        msg = Bool()
        msg.data = (self.state == CoverageState.COVERING)
        self.coverage_status_pub.publish(msg)

    def get_coverage_progress(self) -> float:
        """Return coverage progress as percentage."""
        if not self.waypoints:
            return 0.0
        return (self.current_waypoint_idx / len(self.waypoints)) * 100.0


def main(args=None):
    rclpy.init(args=args)
    
    node = CoveragePlannerNode()
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

