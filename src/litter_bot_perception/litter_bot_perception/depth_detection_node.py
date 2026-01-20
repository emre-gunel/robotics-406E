#!/usr/bin/env python3
"""
Depth-Aware Color Detection for Gazebo Simulation

Uses RGB for color detection + Depth for accurate 3D positioning.
Much more accurate than monocular estimation!
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
import numpy as np
import math

import tf2_ros
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DepthDetectionNode(Node):
    """Depth-aware RED litter detection for simulation."""

    def __init__(self):
        super().__init__('depth_detection_node')

        # Parameters
        self.declare_parameter('min_contour_area', 50)
        self.declare_parameter('max_contour_area', 20000)
        self.declare_parameter('detection_cooldown', 3.0)
        self.declare_parameter('use_depth', True)
        
        self.min_area = self.get_parameter('min_contour_area').value
        self.max_area = self.get_parameter('max_contour_area').value
        self.cooldown = self.get_parameter('detection_cooldown').value
        self.use_depth = self.get_parameter('use_depth').value

        self.bridge = CvBridge()
        self.last_detection_time = None
        
        # Camera info
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False
        
        # Latest depth image
        self.depth_image = None
        self.depth_received = False
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Try multiple topic patterns (RealSense style)
        # Subscribers - check which topics are available
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, sensor_qos
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )
        
        # Fallback to old topic names
        self.image_sub_fallback = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos
        )
        self.camera_info_sub_fallback = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.litter_detected_pub = self.create_publisher(Bool, '/litter_detected', 10)
        self.litter_pose_pub = self.create_publisher(PoseStamped, '/litter_pose', 10)
        self.annotated_pub = self.create_publisher(Image, '/detection/annotated_image', 10)

        self.get_logger().info('🔴 Depth Detection Node initialized')
        self.get_logger().info(f'Use depth: {self.use_depth}')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            K = np.array(msg.k).reshape(3, 3)
            self.fx = K[0, 0]
            self.fy = K[1, 1]
            self.cx = K[0, 2]
            self.cy = K[1, 2]
            self.camera_info_received = True
            self.get_logger().info(f'📷 Camera: {msg.width}x{msg.height}, fx={self.fx:.1f}')

    def depth_callback(self, msg):
        """Store latest depth image."""
        try:
            # Depth images can be 16UC1 (mm) or 32FC1 (meters)
            if msg.encoding == '32FC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            elif msg.encoding == '16UC1':
                depth_mm = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.depth_image = depth_mm.astype(np.float32) / 1000.0  # Convert to meters
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
            
            if not self.depth_received:
                self.get_logger().info('📏 Depth data received!')
                self.depth_received = True
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return

        h, w = cv_image.shape[:2]
        
        # Look at bottom 70% of image (ground level)
        roi_start = int(h * 0.3)
        roi = cv_image[roi_start:, :]

        # Detect RED only
        red_contours = self.detect_red(roi)
        
        annotated = cv_image.copy()
        
        # Draw ROI line
        cv2.line(annotated, (0, roi_start), (w, roi_start), (0, 255, 255), 2)
        status_text = 'DEPTH' if self.depth_received else 'NO DEPTH'
        cv2.putText(annotated, f'Mode: {status_text}', (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        best_detection = None
        max_area = 0
        
        # Find largest red object
        for cnt in red_contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                if area > max_area:
                    max_area = area
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00']) + roi_start
                        best_detection = (cx, cy, area, cnt)

        # Draw all red contours
        for cnt in red_contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                cnt_offset = cnt.copy()
                cnt_offset[:, :, 1] += roi_start
                cv2.drawContours(annotated, [cnt_offset], -1, (0, 0, 255), 2)

        # Process best detection
        if best_detection:
            cx, cy, area, cnt = best_detection
            cv2.circle(annotated, (cx, cy), 10, (0, 255, 0), -1)
            
            # Get depth at this pixel if available
            depth_val = None
            if self.depth_received and self.depth_image is not None:
                depth_val = self.get_depth_at_pixel(cx, cy)
            
            if depth_val is not None:
                cv2.putText(annotated, f'LITTER {depth_val:.2f}m', (cx-50, cy-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(annotated, f'LITTER (est)', (cx-50, cy-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish annotated image
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except:
            pass

        # Process detection
        if best_detection:
            cx, cy, area, _ = best_detection
            
            # Cooldown check
            now = self.get_clock().now()
            if self.last_detection_time is not None:
                elapsed = (now - self.last_detection_time).nanoseconds / 1e9
                if elapsed < self.cooldown:
                    return
            
            self.get_logger().info(f'🔴 LITTER at pixel ({cx}, {cy}), area={int(area)}')
            
            # Estimate 3D position (with depth if available)
            pose = self.estimate_pose(cx, cy, area)
            if pose is not None:
                self.litter_pose_pub.publish(pose)
                self.litter_detected_pub.publish(Bool(data=True))
                self.last_detection_time = now

    def detect_red(self, image):
        """Detect RED objects using HSV color space."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red wraps around in HSV (0 and 180)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([160, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        
        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def get_depth_at_pixel(self, px, py, window_size=5):
        """Get median depth value around a pixel."""
        if self.depth_image is None:
            return None
            
        h, w = self.depth_image.shape[:2]
        
        # Clamp coordinates
        px = max(window_size, min(w - window_size - 1, px))
        py = max(window_size, min(h - window_size - 1, py))
        
        # Get window around pixel
        window = self.depth_image[py-window_size:py+window_size+1, 
                                   px-window_size:px+window_size+1]
        
        # Filter valid depths (not NaN, not 0, reasonable range)
        valid = window[(~np.isnan(window)) & (window > 0.1) & (window < 8.0)]
        
        if len(valid) == 0:
            return None
            
        return float(np.median(valid))

    def estimate_pose(self, px, py, area):
        """Estimate 3D pose using depth camera or fallback to area estimation."""
        if self.fx is None:
            return None
            
        try:
            # Get robot position
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # Try to get depth measurement
            depth = None
            if self.use_depth and self.depth_received:
                depth = self.get_depth_at_pixel(px, py)
            
            if depth is not None:
                # ACCURATE: Use actual depth measurement
                estimated_distance = depth
                self.get_logger().info(f'📏 Using DEPTH: {depth:.2f}m')
            else:
                # FALLBACK: Estimate from area
                ref_distance = 1.5
                ref_area = 1000.0
                estimated_distance = ref_distance * math.sqrt(ref_area / max(10, area))
                estimated_distance = max(0.3, min(5.0, estimated_distance))
                self.get_logger().info(f'📐 Using AREA estimate: {estimated_distance:.2f}m')
            
            # Angle from image center
            angle_x = math.atan2(px - self.cx, self.fx)
            
            # Calculate world position
            litter_x = robot_x + estimated_distance * math.cos(yaw - angle_x)
            litter_y = robot_y + estimated_distance * math.sin(yaw - angle_x)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = litter_x
            pose.pose.position.y = litter_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.get_logger().info(
                f'📍 Litter at ({litter_x:.2f}, {litter_y:.2f}), dist={estimated_distance:.2f}m'
            )
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = DepthDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


