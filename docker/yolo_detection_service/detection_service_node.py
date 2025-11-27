#!/usr/bin/env python3
"""
YOLOv10 + DeepSORT Detection Service Node
Microservices Architecture - Standalone ML Detection Container

Responsibilities:
- Subscribe to camera image topics
- Run YOLOv10 object detection
- Track objects with DeepSORT
- Publish structured detection results
"""

import sys
import time
import os
from pathlib import Path
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from airsim_interfaces.msg import ObjectDetection, ObjectDetectionArray

# YOLOv10 + DeepSORT Integration
DETECTION_PATH = Path('/detection_ws/yolov10_tracking')
sys.path.insert(0, str(DETECTION_PATH))

try:
    from ultralytics import YOLOv10
    import torch
    from deep_sort_pytorch.utils.parser import get_config
    from deep_sort_pytorch.deep_sort import DeepSort
    YOLO_AVAILABLE = True
    print("[INFO] YOLOv10 + DeepSORT imported successfully")
except ImportError as e:
    YOLO_AVAILABLE = False
    print(f"[ERROR] YOLOv10 import failed: {e}")
    sys.exit(1)

# COCO class names (80 classes)
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]


class DetectionServiceNode(Node):
    """Standalone YOLOv10 + DeepSORT detection service"""

    def __init__(self):
        super().__init__('yolov10_detection_service')

        # Parameters
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)

        # Camera topics from environment variable or default
        default_camera_topics = os.environ.get('CAMERA_TOPICS', '/Drone1/Camera_0_Scene/image')
        camera_topics_list = [t.strip() for t in default_camera_topics.split(',')]
        self.declare_parameter('camera_topics', camera_topics_list)
        self.declare_parameter('model_name', 'jameslahm/yolov10n')

        # Class filtering parameters (runtime configurable)
        # Default: Track people and vehicles only (person, car, truck, bus, motorcycle)
        default_allowed_classes = os.environ.get('ALLOWED_CLASSES', 'person,car,truck,bus,motorcycle')
        default_allowed_list = [c.strip() for c in default_allowed_classes.split(',')]
        self.declare_parameter('allowed_classes', default_allowed_list)

        enable_filter_default = os.environ.get('ENABLE_CLASS_FILTER', 'true').lower() == 'true'
        self.declare_parameter('enable_class_filter', enable_filter_default)

        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.camera_topics = self.get_parameter('camera_topics').value
        self.model_name = self.get_parameter('model_name').value
        self.allowed_classes = self.get_parameter('allowed_classes').value
        self.enable_class_filter = self.get_parameter('enable_class_filter').value

        # Cache allowed class IDs for performance
        self.allowed_class_ids = self._get_allowed_class_ids()

        self.get_logger().info(f"Detection Service Configuration:")
        self.get_logger().info(f"  Model: {self.model_name}")
        self.get_logger().info(f"  Confidence threshold: {self.conf_threshold}")
        self.get_logger().info(f"  IOU threshold: {self.iou_threshold}")
        self.get_logger().info(f"  Camera topics: {self.camera_topics}")
        if self.enable_class_filter:
            self.get_logger().info(f"  Class filter ENABLED: {self.allowed_classes}")
            self.get_logger().info(f"    → Tracking class IDs: {sorted(self.allowed_class_ids)}")
        else:
            self.get_logger().info(f"  Class filter DISABLED: All 80 COCO classes will be detected")

        # Initialize YOLOv10 + DeepSORT
        self.bridge = CvBridge()
        self.model = None
        self.deepsort_trackers = {}  # One tracker per camera
        self.initialize_detection_systems()

        # Create subscribers and publishers
        self.image_subs = []
        self.detection_pubs = {}

        for topic in self.camera_topics:
            # Extract camera ID from topic (e.g., "/Drone1/Camera_0_Scene/image" -> "Camera_0_Scene")
            camera_id = self._extract_camera_id(topic)

            # Subscribe to image topic
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cid=camera_id: self.image_callback(msg, cid),
                10
            )
            self.image_subs.append(sub)

            # Create detection publisher for this camera
            pub_topic = f'/detections/{camera_id}'
            pub = self.create_publisher(ObjectDetectionArray, pub_topic, 10)
            self.detection_pubs[camera_id] = pub

            self.get_logger().info(f"  Subscribed: {topic} -> Publishing: {pub_topic}")

        self.get_logger().info("YOLOv10 Detection Service ready!")

    def _extract_camera_id(self, topic):
        """Extract camera ID from topic name
        Examples:
          /Drone1/Camera_0_Scene/image -> Camera_0_Scene
          /Drone1/front_center/Scene -> front_center
        """
        parts = topic.strip('/').split('/')
        # Look for Camera_N_Scene pattern or similar
        for i, part in enumerate(parts):
            if 'Camera' in part or 'camera' in part:
                # Check if next part is 'Scene' or similar
                if i + 1 < len(parts) and parts[i+1] in ['Scene', 'scene']:
                    return f"{part}_{parts[i+1]}"
                return part
            # Legacy support for front_center, etc.
            if part in ['front_center', 'front_left', 'front_right', 'back']:
                return part
        # Fallback: use second-to-last component before /image or /Scene
        if len(parts) >= 2:
            return parts[-2] if parts[-1] in ['image', 'Scene'] else parts[1]
        return "camera_0"

    def _get_allowed_class_ids(self):
        """Convert allowed class names to COCO class IDs

        Returns:
            set: Set of allowed class IDs (0-79), or None if filtering disabled
        """
        if not self.enable_class_filter:
            return None

        allowed_ids = set()
        for class_name in self.allowed_classes:
            try:
                # Case-insensitive lookup
                class_id = COCO_CLASSES.index(class_name.lower())
                allowed_ids.add(class_id)
                self.get_logger().debug(f"Allowed class: '{class_name}' (ID: {class_id})")
            except ValueError:
                self.get_logger().warning(
                    f"Unknown class name: '{class_name}' - Valid classes: {', '.join(COCO_CLASSES[:10])}..."
                )

        if not allowed_ids and self.enable_class_filter:
            self.get_logger().error("No valid class names provided! Disabling filter.")
            return None

        return allowed_ids

    def initialize_detection_systems(self):
        """Initialize YOLOv10 model and DeepSORT trackers"""
        if not YOLO_AVAILABLE:
            self.get_logger().error("YOLOv10 not available!")
            return

        self.get_logger().info(f"Loading YOLOv10 model: {self.model_name}")

        try:
            # Load YOLOv10 model from Hugging Face
            self.model = YOLOv10.from_pretrained(self.model_name)

            # Detect GPU availability (PyTorch nightly should support RTX 5080 sm_120)
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            if device == 'cuda':
                gpu_name = torch.cuda.get_device_name(0)
                self.get_logger().info(f"✓ YOLOv10 model loaded successfully (GPU: {gpu_name})")
            else:
                self.get_logger().warn("GPU not available, using CPU mode")
                self.get_logger().info("✓ YOLOv10 model loaded successfully (CPU mode)")

            # Warmup model
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = self.model.predict(dummy_img, verbose=False)
            self.get_logger().info("✓ Model warmup complete")

            # Initialize DeepSORT config
            cfg = get_config()
            cfg.merge_from_file(str(DETECTION_PATH / 'yolov10' / 'deep_sort_pytorch' / 'configs' / 'deep_sort.yaml'))

            # Create tracker for each camera
            for topic in self.camera_topics:
                camera_id = self._extract_camera_id(topic)
                # GPU-accelerated tracking with PyTorch 2.8.0 + CUDA 12.8
                self.deepsort_trackers[camera_id] = DeepSort(
                    cfg.DEEPSORT.REID_CKPT,
                    max_dist=cfg.DEEPSORT.MAX_DIST,
                    min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                    nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP,
                    max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                    max_age=cfg.DEEPSORT.MAX_AGE,
                    n_init=cfg.DEEPSORT.N_INIT,
                    nn_budget=cfg.DEEPSORT.NN_BUDGET,
                    use_cuda=torch.cuda.is_available()
                )
                self.get_logger().info(f"✓ DeepSORT tracker initialized for {camera_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize detection systems: {e}")
            import traceback
            traceback.print_exc()

    def image_callback(self, msg, camera_id):
        """Process incoming camera image"""
        try:
            import time
            callback_start = time.time()

            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # DIAGNOSTIC: Log image receipt
            self.get_logger().info(
                f"[RX] {camera_id}: Received {cv_image.shape[1]}x{cv_image.shape[0]} image",
                throttle_duration_sec=5.0
            )

            # Run detection + tracking
            detections = self.detect_and_track(cv_image, camera_id)

            # Publish detections
            detection_array = ObjectDetectionArray()
            detection_array.header = msg.header
            detection_array.camera_id = camera_id
            detection_array.detections = detections

            self.detection_pubs[camera_id].publish(detection_array)

            # DIAGNOSTIC: Log publish statistics
            callback_time = (time.time() - callback_start) * 1000  # ms
            self.get_logger().info(
                f"[PUBLISH] {camera_id}: Published {len(detections)} detections (total pipeline: {callback_time:.1f}ms)",
                throttle_duration_sec=5.0
            )

        except Exception as e:
            self.get_logger().error(f"Error processing image from {camera_id}: {e}")

    def detect_and_track(self, image, camera_id):
        """Run YOLOv10 detection + DeepSORT tracking"""
        if self.model is None:
            return []

        detections = []

        try:
            import time
            inference_start = time.time()

            # YOLOv10 prediction (NMS-free detection)
            # GPU-accelerated with PyTorch 2.8.0 + CUDA 12.8 (sm_120 support)
            results = self.model.predict(
                image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False
            )

            inference_time = (time.time() - inference_start) * 1000  # ms

            # Extract detections
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes

                # DIAGNOSTIC: Log raw detection count
                raw_detection_count = len(boxes)
                self.get_logger().info(
                    f"[DETECT] {camera_id}: YOLOv10 found {raw_detection_count} objects (inference: {inference_time:.1f}ms)",
                    throttle_duration_sec=5.0
                )

                if len(boxes) > 0:
                    xywh_bboxs = []
                    confs = []
                    oids = []
                    conf_map = {}  # Map index to confidence for later retrieval
                    filtered_count = 0  # Track how many were filtered out

                    for idx, box in enumerate(boxes):
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0].cpu())
                        cls = int(box.cls[0].cpu())

                        # Apply class filtering if enabled
                        if self.allowed_class_ids is not None and cls not in self.allowed_class_ids:
                            filtered_count += 1
                            continue  # Skip this detection (not in allowed classes)

                        # Convert xyxy to xywh for DeepSORT
                        x1, y1, x2, y2 = xyxy
                        x_c = (x1 + x2) / 2
                        y_c = (y1 + y2) / 2
                        bbox_w = x2 - x1
                        bbox_h = y2 - y1

                        # Store confidence mapped to index in filtered list
                        conf_map[len(xywh_bboxs)] = conf

                        xywh_bboxs.append([x_c, y_c, bbox_w, bbox_h])
                        confs.append([conf])
                        oids.append(cls)

                    # DIAGNOSTIC: Log filtering results
                    if filtered_count > 0:
                        self.get_logger().info(
                            f"[FILTER] {camera_id}: Filtered out {filtered_count}/{raw_detection_count} detections (kept {len(xywh_bboxs)} allowed classes)",
                            throttle_duration_sec=5.0
                        )

                    # Update DeepSORT tracker
                    tracker = self.deepsort_trackers.get(camera_id)
                    if tracker and len(xywh_bboxs) > 0:
                        xywhs = torch.Tensor(xywh_bboxs)
                        confss = torch.Tensor(confs)
                        outputs = tracker.update(xywhs, confss, oids, image)

                        # Create detection messages
                        if len(outputs) > 0:
                            for idx, output in enumerate(outputs):
                                x1, y1, x2, y2 = output[:4]
                                track_id = int(output[4]) if len(output) > 4 else -1
                                class_id = int(output[5]) if len(output) > 5 else 0

                                det = ObjectDetection()
                                det.x = float(x1)
                                det.y = float(y1)
                                det.width = float(x2 - x1)
                                det.height = float(y2 - y1)
                                det.class_id = class_id
                                det.class_name = COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else "unknown"
                                # Use confidence from map, or default to threshold value
                                det.confidence = conf_map.get(idx, self.conf_threshold)
                                det.track_id = track_id

                                detections.append(det)

        except Exception as e:
            self.get_logger().error(f"Detection error for {camera_id}: {e}")

        return detections


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DetectionServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] Detection service crashed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
