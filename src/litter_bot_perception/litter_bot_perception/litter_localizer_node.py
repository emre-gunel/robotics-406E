#!/usr/bin/env python3
"""
Litter Localizer Node

Converts 2D detections to 3D poses using depth camera data
and transforms them to the robot's base frame using tf2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from std_msgs.msg import Header

from cv_bridge import CvBridge
import numpy as np
import math

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class LitterLocalizerNode(Node):
    """
    Converts 2D litter detections to 3D poses in the robot's base frame.
    """

    def __init__(self):
        super().__init__('litter_localizer_node')

        # Declare parameters
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 5.0)

        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Latest depth image
        self.latest_depth = None
        self.depth_encoding = None

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for camera topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/litter_detections', self.detection_callback, 10
        )

        # Publishers - use /litter_pose for coordinator compatibility
        self.litter_pose_pub = self.create_publisher(
            PoseStamped, '/litter_pose', 10
        )
        self.litter_poses_pub = self.create_publisher(
            PoseArray, '/litter_poses', 10
        )

        self.get_logger().info('Litter Localizer Node initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics from camera info."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(f'Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                                   f'cx={self.cx:.2f}, cy={self.cy:.2f}')

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            self.depth_encoding = msg.encoding
            if msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == '16UC1':
                depth_16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.latest_depth = depth_16.astype(np.float32) / 1000.0  # mm to meters
            else:
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.latest_depth = self.latest_depth.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {e}')

    def detection_callback(self, msg: Detection2DArray):
        """Process detections and compute 3D poses."""
        if self.latest_depth is None:
            self.get_logger().warn('No depth image available yet')
            return

        if self.camera_matrix is None:
            self.get_logger().warn('No camera info available yet')
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for detection in msg.detections:
            pose_3d = self.detection_to_3d_pose(detection, msg.header)
            if pose_3d is not None:
                # Publish individual pose
                self.litter_pose_pub.publish(pose_3d)
                pose_array.poses.append(pose_3d.pose)

        # Publish all poses as array
        if pose_array.poses:
            self.litter_poses_pub.publish(pose_array)

    def detection_to_3d_pose(self, detection, header: Header) -> PoseStamped:
        """
        Convert a 2D detection to a 3D pose.
        
        Args:
            detection: Detection2D message
            header: Header with timestamp
            
        Returns:
            PoseStamped in target frame, or None if conversion fails
        """
        # Get pixel coordinates of detection center
        u = int(detection.bbox.center.position.x)
        v = int(detection.bbox.center.position.y)

        # Validate pixel coordinates
        h, w = self.latest_depth.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f'Detection center ({u}, {v}) outside image bounds')
            return None

        # Get depth at detection center (use small region average for robustness)
        depth = self.get_depth_at_point(u, v)
        if depth is None or not (self.min_depth < depth < self.max_depth):
            self.get_logger().debug(f'Invalid depth at ({u}, {v}): {depth}')
            return None

        # Convert pixel + depth to 3D point in camera frame
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth

        # Create pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = self.camera_frame
        pose_camera.header.stamp = header.stamp
        pose_camera.pose.position.x = float(x_cam)
        pose_camera.pose.position.y = float(y_cam)
        pose_camera.pose.position.z = float(z_cam)
        # Orientation pointing down (for grasping)
        pose_camera.pose.orientation.w = 1.0

        # Transform to target frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_camera, transform
            )
            
            # Set proper grasping orientation (gripper pointing down)
            pose_base.pose.orientation.x = 0.0
            pose_base.pose.orientation.y = 0.707  # 90 degree pitch
            pose_base.pose.orientation.z = 0.0
            pose_base.pose.orientation.w = 0.707

            self.get_logger().info(
                f'Litter at: ({pose_base.pose.position.x:.2f}, '
                f'{pose_base.pose.position.y:.2f}, {pose_base.pose.position.z:.2f}) '
                f'in {self.target_frame}'
            )
            
            return pose_base

        except TransformException as e:
            self.get_logger().warn(f'Failed to transform pose: {e}')
            return None

    def get_depth_at_point(self, u: int, v: int, window_size: int = 5) -> float:
        """
        Get robust depth estimate at pixel (u, v) using median of window.
        """
        h, w = self.latest_depth.shape[:2]
        half = window_size // 2
        
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
        
        window = self.latest_depth[v_min:v_max, u_min:u_max]
        
        # Filter valid depths
        valid_depths = window[(window > self.min_depth) & (window < self.max_depth)]
        
        if len(valid_depths) == 0:
            return None
        
        return float(np.median(valid_depths))


def main(args=None):
    rclpy.init(args=args)
    node = LitterLocalizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

