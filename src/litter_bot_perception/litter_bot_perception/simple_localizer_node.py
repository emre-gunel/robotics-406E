#!/usr/bin/env python3
"""
Simple Litter Localizer Node

Converts 2D detections to 3D poses using ground plane assumption.
No depth camera required - estimates distance from bounding box size.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool

import numpy as np
import math

import tf2_ros
from tf2_ros import TransformException


class SimpleLitterLocalizer(Node):
    """
    Converts 2D litter detections to 3D map poses using ground plane assumption.
    Estimates distance from bounding box size (larger box = closer object).
    """

    def __init__(self):
        super().__init__('simple_localizer_node')

        # Parameters
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_rgb_frame')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('expected_litter_size', 0.05)  # meters (5cm)
        self.declare_parameter('camera_height', 0.15)  # camera height from ground
        self.declare_parameter('max_detection_range', 3.0)  # max valid range

        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.expected_litter_size = self.get_parameter('expected_litter_size').value
        self.camera_height = self.get_parameter('camera_height').value
        self.max_range = self.get_parameter('max_detection_range').value

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.image_width = 640
        self.image_height = 480

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/litter_detections', self.detection_callback, 10
        )

        # Publishers
        self.litter_pose_pub = self.create_publisher(PoseStamped, '/litter_pose', 10)
        self.litter_detected_pub = self.create_publisher(Bool, '/litter_detected', 10)

        self.get_logger().info('Simple Litter Localizer initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics."""
        if self.fx is None:
            K = np.array(msg.k).reshape(3, 3)
            self.fx = K[0, 0]
            self.fy = K[1, 1]
            self.cx = K[0, 2]
            self.cy = K[1, 2]
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(f'Camera: {self.image_width}x{self.image_height}, '
                                   f'fx={self.fx:.1f}, fy={self.fy:.1f}')

    def detection_callback(self, msg: Detection2DArray):
        """Convert 2D detections to 3D poses."""
        if self.fx is None:
            return

        for detection in msg.detections:
            pose = self.estimate_3d_pose(detection)
            if pose is not None:
                self.litter_pose_pub.publish(pose)
                self.litter_detected_pub.publish(Bool(data=True))
                return  # Only process first detection per frame

    def estimate_3d_pose(self, detection) -> PoseStamped:
        """
        Estimate 3D position from 2D detection using:
        1. Bounding box size to estimate distance
        2. Pixel position to estimate angle
        3. Ground plane assumption (litter is on floor)
        """
        # Get bounding box
        cx = detection.bbox.center.position.x
        cy = detection.bbox.center.position.y
        bbox_width = detection.bbox.size_x
        bbox_height = detection.bbox.size_y

        if bbox_width < 5 or bbox_height < 5:
            return None  # Too small

        # Estimate distance from bbox size
        # Assumes: pixel_size = (focal_length * real_size) / distance
        # Therefore: distance = (focal_length * real_size) / pixel_size
        avg_bbox = (bbox_width + bbox_height) / 2
        estimated_distance = (self.fy * self.expected_litter_size) / avg_bbox

        # Clamp distance
        if estimated_distance > self.max_range or estimated_distance < 0.2:
            return None

        # Calculate angle from image center
        angle_x = math.atan2(cx - self.cx, self.fx)

        # Calculate 3D position in camera frame
        x_cam = estimated_distance * math.sin(angle_x)
        y_cam = 0.0  # Ground plane
        z_cam = estimated_distance * math.cos(angle_x)

        # Get transform from camera to map
        try:
            # First get camera position in map frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                'base_footprint',  # Robot base
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Robot position
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Robot yaw from quaternion
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Transform litter position to map frame
            # Camera is forward-facing, so z_cam is forward, x_cam is right
            litter_x = robot_x + z_cam * math.cos(yaw) - x_cam * math.sin(yaw)
            litter_y = robot_y + z_cam * math.sin(yaw) + x_cam * math.cos(yaw)

            # Create pose
            pose = PoseStamped()
            pose.header.frame_id = self.target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = litter_x
            pose.pose.position.y = litter_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            self.get_logger().info(
                f'🎯 Litter detected at ({litter_x:.2f}, {litter_y:.2f}) '
                f'[dist={estimated_distance:.2f}m, angle={math.degrees(angle_x):.1f}°]'
            )

            return pose

        except TransformException as e:
            self.get_logger().warn(f'TF error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLitterLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

