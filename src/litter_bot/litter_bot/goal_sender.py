#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__("goal_sender")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        # Wait for Nav2 action server
        self.get_logger().info("Waiting for Nav2 action server (navigate_to_pose)...")
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        # Simple yaw->quat (only z,w for planar)
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected ❌")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted ✅ Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received ✅ {result}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GoalSender()

    # örnek goal (sonra param/arg yaparız)
    node.send_goal(1.0, 0.0, 0.0)

    rclpy.spin(node)


if __name__ == "__main__":
    main()

