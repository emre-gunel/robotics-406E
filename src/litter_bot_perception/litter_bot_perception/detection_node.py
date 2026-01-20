#!/usr/bin/env python3
"""
YOLOv8 Detection Node

Detects litter objects (cigarette butts, paper, bottle caps) and humans
using YOLOv8 model on camera images.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip install ultralytics")


class DetectionNode(Node):
    """
    YOLOv8-based detection node for litter and human detection.
    """

    # Class mappings for litter detection
    # Using COCO classes for humans, custom classes for litter
    LITTER_CLASSES = {
        'cigarette_butt': 0,
        'paper': 1,
        'bottle_cap': 2,
    }
    
    COCO_PERSON_CLASS = 0  # Person class in COCO dataset

    def __init__(self):
        super().__init__('detection_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')  # Use pretrained model
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('detect_humans', True)
        self.declare_parameter('detect_litter', True)

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.detect_humans = self.get_parameter('detect_humans').value
        self.detect_litter = self.get_parameter('detect_litter').value

        # CV Bridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                device = 'cuda' if self.use_gpu else 'cpu'
                self.get_logger().info(f'Loaded YOLOv8 model: {self.model_path} on {device}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().warn('YOLO not available, running in dummy mode')

        # QoS for camera topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/litter_detections', 10
        )
        self.human_detection_pub = self.create_publisher(
            Detection2DArray, '/human_detections', 10
        )
        self.litter_detected_pub = self.create_publisher(
            Bool, '/litter_detected', 10
        )
        self.annotated_image_pub = self.create_publisher(
            Image, '/detection/annotated_image', 10
        )

        # State
        self.last_detection_time = self.get_clock().now()
        self.detection_count = 0

        self.get_logger().info('Detection Node initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run detection
        litter_detections, human_detections, annotated_image = self.detect(cv_image, msg.header)

        # Publish litter detections
        if litter_detections.detections:
            self.detection_pub.publish(litter_detections)
            self.litter_detected_pub.publish(Bool(data=True))
            self.detection_count += len(litter_detections.detections)
        else:
            self.litter_detected_pub.publish(Bool(data=False))

        # Publish human detections
        if human_detections.detections:
            self.human_detection_pub.publish(human_detections)

        # Publish annotated image
        if self.publish_annotated and annotated_image is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')

    def detect(self, image: np.ndarray, header: Header):
        """
        Run YOLOv8 detection on image.
        
        Returns:
            litter_detections: Detection2DArray for litter
            human_detections: Detection2DArray for humans
            annotated_image: Image with bounding boxes drawn
        """
        litter_detections = Detection2DArray()
        litter_detections.header = header

        human_detections = Detection2DArray()
        human_detections.header = header

        annotated_image = image.copy()

        if self.model is None:
            # Dummy mode: simulate random detections for testing
            return self._dummy_detect(image, header, annotated_image)

        try:
            # Run inference
            results = self.model(image, conf=self.confidence_threshold, verbose=False)

            for result in results:
                boxes = result.boxes
                
                # Debug: log all detections
                if len(boxes) > 0:
                    classes_detected = [self.model.names[int(b.cls[0])] for b in boxes]
                    self.get_logger().info(f'🔍 Detected: {classes_detected}')
                
                for i, box in enumerate(boxes):
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    xyxy = box.xyxy[0].cpu().numpy()
                    
                    x1, y1, x2, y2 = xyxy
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    w = x2 - x1
                    h = y2 - y1

                    # Create detection message
                    detection = Detection2D()
                    detection.header = header
                    detection.bbox.center.position.x = float(cx)
                    detection.bbox.center.position.y = float(cy)
                    detection.bbox.size_x = float(w)
                    detection.bbox.size_y = float(h)

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)

                    # Classify as human or potential litter
                    class_name = self.model.names[cls_id]
                    
                    if cls_id == self.COCO_PERSON_CLASS and self.detect_humans:
                        # Human detection
                        human_detections.detections.append(detection)
                        color = (0, 0, 255)  # Red for humans
                        label = f'Person: {conf:.2f}'
                        
                    elif self.detect_litter and self._is_potential_litter(class_name):
                        # Litter detection
                        litter_detections.detections.append(detection)
                        color = (0, 255, 0)  # Green for litter
                        label = f'{class_name}: {conf:.2f}'
                    else:
                        continue

                    # Draw on annotated image
                    cv2.rectangle(annotated_image, 
                                  (int(x1), int(y1)), 
                                  (int(x2), int(y2)), 
                                  color, 2)
                    cv2.putText(annotated_image, label,
                                (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')

        return litter_detections, human_detections, annotated_image

    def _is_potential_litter(self, class_name: str) -> bool:
        """
        Check if detected class could be litter.
        
        For simulation, we use COCO classes that might represent litter:
        - 'cup', 'bottle', 'wine glass' -> bottle cap proxy
        - 'book', 'paper' -> paper proxy
        - Various small objects
        
        In production, train custom model on litter dataset.
        """
        litter_proxies = [
            'cup', 'bottle', 'wine glass',  # Bottle cap proxies
            'book',  # Paper proxy
            'cell phone', 'remote',  # Small object proxies
            'scissors', 'toothbrush',  # Cigarette butt proxy (similar size)
            'handbag', 'backpack', 'suitcase',  # Bags
            'sports ball', 'frisbee',  # Round objects
            'potted plant', 'vase',  # Plant-like
            'chair', 'bench',  # Furniture (for testing)
        ]
        return class_name.lower() in [p.lower() for p in litter_proxies]

    def _dummy_detect(self, image: np.ndarray, header: Header, annotated_image: np.ndarray):
        """Dummy detection for testing without YOLO model."""
        import random
        
        litter_detections = Detection2DArray()
        litter_detections.header = header
        
        human_detections = Detection2DArray()
        human_detections.header = header

        # Occasionally generate fake detections for testing
        if random.random() < 0.05:  # 5% chance
            h, w = image.shape[:2]
            
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = float(random.randint(100, w - 100))
            detection.bbox.center.position.y = float(random.randint(100, h - 100))
            detection.bbox.size_x = float(random.randint(20, 50))
            detection.bbox.size_y = float(random.randint(20, 50))

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'litter'
            hypothesis.hypothesis.score = random.uniform(0.6, 0.95)
            detection.results.append(hypothesis)

            litter_detections.detections.append(detection)

            # Draw on annotated image
            x = int(detection.bbox.center.position.x)
            y = int(detection.bbox.center.position.y)
            w = int(detection.bbox.size_x / 2)
            h = int(detection.bbox.size_y / 2)
            cv2.rectangle(annotated_image, (x - w, y - h), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(annotated_image, 'Litter (dummy)', (x - w, y - h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return litter_detections, human_detections, annotated_image


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

