#!/usr/bin/env python3
"""
Simple RED Color Detection for Gazebo Simulation

Only detects RED objects = litter (bottle caps, etc.)
All litter in Gazebo is RED for easy detection.
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


class ColorDetectionNode(Node):
    """Simple RED-only litter detection for simulation."""

    def __init__(self):
        super().__init__('color_detection_node')

        # Parameters
        self.declare_parameter('min_contour_area', 30)
        self.declare_parameter('max_contour_area', 15000)
        self.declare_parameter('detection_cooldown', 5.0)
        
        self.min_area = self.get_parameter('min_contour_area').value
        self.max_area = self.get_parameter('max_contour_area').value
        self.cooldown = self.get_parameter('detection_cooldown').value

        self.bridge = CvBridge()
        self.last_detection_time = None
        
        # Camera info
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.image_height = 480
        self.image_width = 640
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.litter_detected_pub = self.create_publisher(Bool, '/litter_detected', 10)
        self.litter_pose_pub = self.create_publisher(PoseStamped, '/litter_pose', 10)
        self.annotated_pub = self.create_publisher(Image, '/detection/annotated_image', 10)

        self.get_logger().info('🔴 RED Detection Node initialized')
        self.get_logger().info('Looking for RED objects only (all litter is RED)')

    def camera_info_callback(self, msg):
        if self.fx is None:
            K = np.array(msg.k).reshape(3, 3)
            self.fx = K[0, 0]
            self.fy = K[1, 1]
            self.cx = K[0, 2]
            self.cy = K[1, 2]
            self.image_height = msg.height
            self.image_width = msg.width
            self.get_logger().info(f'Camera: {msg.width}x{msg.height}')

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
        cv2.putText(annotated, 'Detection Zone', (10, roi_start + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
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

        # Mark best detection
        if best_detection:
            cx, cy, area, cnt = best_detection
            cv2.circle(annotated, (cx, cy), 10, (0, 255, 0), -1)
            cv2.putText(annotated, f'LITTER {int(area)}', (cx-40, cy-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Status
        status = f'RED objects: {len([c for c in red_contours if self.min_area < cv2.contourArea(c) < self.max_area])}'
        cv2.putText(annotated, status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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
            
            self.get_logger().info(f'🔴 LITTER detected at ({cx}, {cy}), area={int(area)}')
            
            # Estimate 3D position
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

    def estimate_pose(self, px, py, area):
        """Estimate 3D pose from pixel position and area."""
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
            
            # Better distance estimation based on area
            # Empirical formula: bigger area = closer
            # At 1m, typical litter shows ~2000-3000 pixel area
            # At 3m, typical litter shows ~200-400 pixel area
            # Use inverse square root relationship
            
            # Reference: at 1.5m distance, area ~ 1000 pixels
            ref_distance = 1.5
            ref_area = 1000.0
            
            # Distance scales inversely with sqrt(area)
            estimated_distance = ref_distance * math.sqrt(ref_area / max(10, area))
            
            # Clamp to reasonable range
            estimated_distance = max(0.3, min(5.0, estimated_distance))
            
            # Angle from image center (normalized for any resolution)
            # Positive px offset = object to the right in camera frame
            # In robot frame: right = -Y, so we subtract the angle
            angle_x = math.atan2(px - self.cx, self.fx)
            
            # Calculate world position
            # Robot faces forward (yaw direction), camera aligned with robot
            # Object to the right in image = negative angle offset from yaw
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
                f'📍 Litter at ({litter_x:.2f}, {litter_y:.2f}), dist={estimated_distance:.2f}m, area={int(area)}'
            )
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
