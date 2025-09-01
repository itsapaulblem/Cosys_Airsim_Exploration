#!/usr/bin/env python3
"""
Motion Detection Node for AirSim ROS2 Integration with YOLOv7 + DeepSORT Tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from airsim_interfaces.msg import TargetDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.serialization
import time
from collections import deque, defaultdict
from pathlib import Path
import sys
import os

# YOLOv7 + DeepSORT Integration Setup
DETECTION_PATH = Path(__file__).parent / 'YOLOv7-DeepSORT-Object-Tracking'
sys.path.insert(0, str(DETECTION_PATH))

# Debug: Check if directories exist
print(f"[DEBUG] Script location: {Path(__file__).parent}")
print(f"[DEBUG] Detection path: {DETECTION_PATH}")
print(f"[DEBUG] Detection path exists: {DETECTION_PATH.exists()}")

try:
    from models.experimental import attempt_load
    from utils.datasets import LoadStreams, LoadImages
    from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
    from utils.torch_utils import select_device, time_synchronized
    from deep_sort_pytorch.utils.parser import get_config
    from deep_sort_pytorch.deep_sort import DeepSort
    YOLO_DEEPSORT_AVAILABLE = True
    print("[DEBUG] ✓ YOLOv7+DeepSORT imports successful")
except ImportError as e:
    YOLO_DEEPSORT_AVAILABLE = False
    print(f"[DEBUG] ✗ YOLOv7+DeepSORT imports failed: {e}")

class MotionDetectionNode(Node):
    def __init__(self):
        super().__init__('motion_detection_node')

        # ROS 2 Parameters Declaration and Retrieval
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('camera_topic', '/drone1/airsim_drone1/camera0/image')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('motion_threshold', 15.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('trail_length', 30)
        
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)
        self.enable_vis = self.get_parameter('enable_visualization').value
        self.trail_length = int(self.get_parameter('trail_length').value)

        self.bridge = CvBridge()

        # Debug counters
        self.frame_count = 0
        self.last_detection_count = 0
        self.last_moving_count = 0

        # Detection mode flags - ADD THESE MISSING VARIABLES
        self.use_yolo_deepsort = False
        self.use_yolo_only = False

        # COCO class names (from deep_sort_tracking_id.py)
        self.names = self.load_coco_names()
        
        # Color palette for different classes
        self.palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)
        
        # Motion tracking data structure (similar to deep_sort_tracking_id.py)
        self.data_deque = {}

        # Initialize detection systems
        self.initialize_detection_systems()
        
        # OpenCV tracking variables for fallback
        self.background_subtractor = None
        self.kernel = None
        self.previous_detections = {}
        self.next_track_id = 1
        
        # ROS2 communication interfaces
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(
            TargetDetection, 'target_detection', 10)
        
        if self.enable_vis:
            self.vis_pub = self.create_publisher(
                Image, 'detection_visualization', 10)
        
        # Create debug timer
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        
        self.get_logger().info(f'Motion Detection node initialized for {self.vehicle_name}')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'Motion threshold: {self.motion_threshold} pixels')
        self.get_logger().info(f'YOLOv7+DeepSORT: {"✓" if self.use_yolo_deepsort else "✗"}')

    def load_coco_names(self):
        """Load COCO class names"""
        return ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 
                'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 
                'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 
                'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 
                'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 
                'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 
                'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 
                'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 
                'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 
                'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

    def debug_status(self):
        """Debug timer callback to show system status"""
        self.get_logger().info(f'[DEBUG] Frames processed: {self.frame_count}, '
                              f'Last detections: {self.last_detection_count}, '
                              f'Last moving: {self.last_moving_count}, '
                              f'Active tracks: {len(self.data_deque)}')

    def initialize_detection_systems(self):  # FIX INDENTATION HERE
        """Initialize YOLOv7 + DeepSORT system with better checkpoint handling"""
        self.get_logger().info("[DEBUG] Initializing detection systems...")
        
        if YOLO_DEEPSORT_AVAILABLE:
            try:
                # Initialize device
                self.device = select_device('')
                self.half = self.device.type != 'cpu'
                self.get_logger().info(f"[DEBUG] Selected device: {self.device}, half precision: {self.half}")
                
                # Load YOLOv7 model
                weights_path = DETECTION_PATH / 'yolov7.pt'
                self.get_logger().info(f"[DEBUG] Looking for weights at: {weights_path}")
                
                if weights_path.exists():
                    self.get_logger().info("[DEBUG] YOLOv7 weights found, loading model...")
                    
                    # Handle PyTorch 2.6+ weights_only issue
                    try:
                        self.get_logger().info("[DEBUG] Attempting safe loading with whitelisted globals...")
                        yolo_safe_globals = [
                            'models.yolo.Model', 'models.common.Conv', 'models.common.Bottleneck',
                            'models.common.SPP', 'models.common.DWConv', 'models.common.Focus',
                            'models.common.C3', 'models.common.C3TR', 'models.common.SPPF',
                            'models.common.Concat', 'models.common.Detect', 'models.experimental.MixConv2d',
                            'models.experimental.CrossConv', 'models.experimental.Sum',
                            'torch.nn.modules.conv.Conv2d', 'torch.nn.modules.batchnorm.BatchNorm2d',
                            'torch.nn.modules.activation.SiLU', 'torch.nn.modules.pooling.MaxPool2d',
                            'torch.nn.modules.upsampling.Upsample'
                        ]
                        
                        with torch.serialization.safe_globals(yolo_safe_globals):
                            self.model = attempt_load(str(weights_path), map_location=self.device)
                        self.get_logger().info("[DEBUG] ✓ Safe loading successful")
                        
                    except Exception as safe_error:
                        self.get_logger().warn(f"[DEBUG] Safe loading failed: {safe_error}")
                        self.get_logger().warn("[DEBUG] Attempting unsafe loading (trusted source assumed)...")
                        
                        import types
                        original_load = torch.load
                        
                        def unsafe_load(*args, **kwargs):
                            kwargs['weights_only'] = False
                            return original_load(*args, **kwargs)
                        
                        torch.load = unsafe_load
                        self.model = attempt_load(str(weights_path), map_location=self.device)
                        torch.load = original_load
                        self.get_logger().info("[DEBUG] ✓ Unsafe loading successful")
                    
                    self.stride = int(self.model.stride.max())
                    self.img_size = 640
                    
                    if self.half:
                        self.model.half()
                    
                    self.model.eval()
                    self.get_logger().info("[DEBUG] YOLOv7 model loaded successfully")
                    
                    # Try to initialize DeepSORT with checkpoint validation
                    self.get_logger().info("[DEBUG] Initializing DeepSORT...")
                    checkpoint_path = DETECTION_PATH / "deep_sort_pytorch" / "deep_sort" / "deep" / "checkpoint" / "ckpt.t7"
                    config_path = DETECTION_PATH / "deep_sort_pytorch" / "configs" / "deep_sort.yaml"
                    
                    if checkpoint_path.exists() and config_path.exists():
                        cfg_deep = get_config()
                        cfg_deep.merge_from_file(str(config_path))
                        
                        self.deepsort = DeepSort(
                            str(checkpoint_path),
                            max_dist=cfg_deep.DEEPSORT.MAX_DIST,
                            min_confidence=cfg_deep.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=cfg_deep.DEEPSORT.NMS_MAX_OVERLAP,
                            max_iou_distance=cfg_deep.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg_deep.DEEPSORT.MAX_AGE,
                            n_init=cfg_deep.DEEPSORT.N_INIT,
                            nn_budget=cfg_deep.DEEPSORT.NN_BUDGET,
                            use_cuda=torch.cuda.is_available()
                        )
                        
                        self.use_yolo_deepsort = True
                        self.use_yolo_only = False
                        self.get_logger().info('[DEBUG] ✓ YOLOv7 + DeepSORT initialized successfully')
                        self.warmup_model()
                        
                    else:
                        self.get_logger().warn(f"[DEBUG] DeepSORT checkpoint not found at {checkpoint_path}")
                        self.get_logger().warn("[DEBUG] Using YOLOv7 only (without tracking)")
                        self.use_yolo_deepsort = False
                        self.use_yolo_only = True
                        self.warmup_model()
                        
                else:
                    self.get_logger().error(f'[DEBUG] ✗ YOLOv7 weights not found at {weights_path}')
                    self.use_yolo_deepsort = False
                    self.use_yolo_only = False
                    
            except Exception as e:
                self.get_logger().error(f'[DEBUG] ✗ Failed to initialize YOLOv7+DeepSORT: {e}')
                import traceback
                self.get_logger().error(f'[DEBUG] Full traceback: {traceback.format_exc()}')
                self.use_yolo_deepsort = False
                self.use_yolo_only = False
        else:
            self.get_logger().warn("[DEBUG] YOLOv7+DeepSORT imports not available")
            self.use_yolo_deepsort = False
            self.use_yolo_only = False
            
        # Fallback to OpenCV if YOLO+DeepSORT not available
        if not self.use_yolo_deepsort and not self.use_yolo_only:
            self.get_logger().warn("[DEBUG] Setting up OpenCV motion detection fallback")
            self.setup_opencv_detection()

    def warmup_model(self):
        """Warm up the YOLOv7 model"""
        try:
            dummy_img = torch.zeros(1, 3, self.img_size, self.img_size).to(self.device).type_as(next(self.model.parameters()))
            for _ in range(3):
                _ = self.model(dummy_img)
            self.get_logger().info("[DEBUG] Model warmed up successfully")
        except Exception as e:
            self.get_logger().warn(f"[DEBUG] Model warmup failed: {e}")

    def setup_opencv_detection(self):
        """Setup OpenCV motion detection fallback"""
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(
            detectShadows=True, varThreshold=50)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.get_logger().info("[DEBUG] ✓ OpenCV motion detection initialized")

    # ADD MISSING YOLO-ONLY DETECTION METHOD
    def detect_yolo_only(self, image):
        """YOLOv7 detection without DeepSORT tracking"""
        im0 = image.copy()
        
        # Preprocess image
        img = self.letterbox(image, self.img_size, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_threshold, self.iou_threshold, classes=None, agnostic=False)
        
        detections = []
        
        # Process detections
        for i, det in enumerate(pred):
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    x1, y1, x2, y2 = [int(coord) for coord in xyxy]
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    detections.append({
                        'bbox': [x1, y1, x2 - x1, y2 - y1],
                        'center': [center_x, center_y],
                        'confidence': float(conf),
                        'class': int(cls),
                        'xyxy': [x1, y1, x2, y2]
                    })
        
        return detections, im0

    # ADD MISSING BASIC TRACKING FOR YOLO DETECTIONS
    def track_yolo_detections_basic(self, detections):
        """Basic tracking for YOLO detections without DeepSORT"""
        current_time = time.time()
        moving_targets = []
        
        for detection in detections:
            center = detection['center']
            
            # Find closest previous detection
            best_track_id = None
            min_distance = float('inf')
            
            for track_id, prev_data in self.previous_detections.items():
                prev_center = prev_data['center']
                distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                
                if distance < min_distance and distance < 100:
                    min_distance = distance
                    best_track_id = track_id
            
            if best_track_id is None:
                track_id = self.next_track_id
                self.next_track_id += 1
            else:
                track_id = best_track_id
            
            # Update tracking data
            if track_id not in self.data_deque:
                self.data_deque[track_id] = deque(maxlen=self.trail_length)
                
            self.data_deque[track_id].appendleft(center)
            self.previous_detections[track_id] = {
                'center': center,
                'timestamp': current_time,
                'class': detection['class'],
                'confidence': detection['confidence']
            }
            
            # Check if moving
            if self.is_track_moving(track_id):
                world_pos = self.pixel_to_world(center)
                velocity = self.calculate_track_velocity(track_id)
                
                moving_targets.append({
                    'track_id': track_id,
                    'world_x': world_pos[0],
                    'world_y': world_pos[1],
                    'world_z': 0.0,
                    'confidence': detection['confidence'],
                    'velocity': velocity,
                    'class': detection['class'],
                    'bbox': detection['bbox']
                })
        
        # Clean up old tracks
        self.cleanup_old_tracks(current_time)
        return moving_targets

    # ADD MISSING YOLO DETECTION DRAWING METHOD
    def draw_yolo_detections(self, image, detections, moving_targets):
        """Draw YOLO detections with basic tracking"""
        vis_image = image.copy()
        moving_track_ids = {target['track_id'] for target in moving_targets}
        
        for detection in detections:
            x1, y1, x2, y2 = detection['xyxy']
            center = detection['center']
            class_id = detection['class']
            confidence = detection['confidence']
            
            # Find track ID
            track_id = None
            for tid, prev_data in self.previous_detections.items():
                prev_center = prev_data['center']
                distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                if distance < 50:
                    track_id = tid
                    break
            
            is_moving = track_id in moving_track_ids if track_id else False
            color = self.compute_color_for_labels(class_id)
            
            # Draw bounding box
            thickness = 3 if is_moving else 2
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, thickness)
            
            # Draw trail if available
            if track_id and track_id in self.data_deque:
                positions = list(self.data_deque[track_id])
                for i in range(1, len(positions)):
                    if positions[i - 1] is None or positions[i] is None:
                        continue
                    thickness_trail = int(np.sqrt(self.trail_length / float(i + i)) * 1.5)
                    cv2.line(vis_image, 
                           (int(positions[i - 1][0]), int(positions[i - 1][1])), 
                           (int(positions[i][0]), int(positions[i][1])), 
                           color, thickness_trail)
            
            # Draw label
            class_name = self.names[class_id] if class_id < len(self.names) else f"class_{class_id}"
            label = f"{class_name} {confidence:.2f}"
            if track_id:
                label += f" ID:{track_id}"
            if is_moving:
                velocity = next((t['velocity'] for t in moving_targets if t['track_id'] == track_id), 0)
                label += f" V:{velocity:.1f}"
            
            cv2.putText(vis_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add status text
        mode = "YOLO+DeepSORT" if self.use_yolo_deepsort else ("YOLO-only" if self.use_yolo_only else "OpenCV")
        status = f"{mode} | Frame: {self.frame_count} | Detections: {len(detections)} | Moving: {len(moving_targets)}"
        cv2.putText(vis_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return vis_image

    # Helper functions from deep_sort_tracking_id.py
    def xyxy_to_xywh(self, *xyxy):
        """Convert bounding box format"""
        bbox_left = min([xyxy[0].item(), xyxy[2].item()])
        bbox_top = min([xyxy[1].item(), xyxy[3].item()])
        bbox_w = abs(xyxy[0].item() - xyxy[2].item())
        bbox_h = abs(xyxy[1].item() - xyxy[3].item())
        x_c = (bbox_left + bbox_w / 2)
        y_c = (bbox_top + bbox_h / 2)
        w = bbox_w
        h = bbox_h
        return x_c, y_c, w, h

    def compute_color_for_labels(self, label):
        """Compute color for class labels (from deep_sort_tracking_id.py)"""
        if label == 0:  # person
            color = (85, 45, 255)
        elif label == 2:  # Car
            color = (222, 82, 175)
        elif label == 3:  # Motorbike
            color = (0, 204, 255)
        elif label == 5:  # Bus
            color = (0, 149, 255)
        else:
            color = [int((p * (label ** 2 - label + 1)) % 255) for p in self.palette]
        return tuple(color)

    def draw_border(self, img, pt1, pt2, color, thickness, r, d):
        """Draw fancy border (from deep_sort_tracking_id.py)"""
        x1, y1 = pt1
        x2, y2 = pt2
        # Top left
        cv2.line(img, (x1 + r, y1), (x1 + r + d, y1), color, thickness)
        cv2.line(img, (x1, y1 + r), (x1, y1 + r + d), color, thickness)
        cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness)
        # Top right
        cv2.line(img, (x2 - r, y1), (x2 - r - d, y1), color, thickness)
        cv2.line(img, (x2, y1 + r), (x2, y1 + r + d), color, thickness)
        cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness)
        # Bottom left
        cv2.line(img, (x1 + r, y2), (x1 + r + d, y2), color, thickness)
        cv2.line(img, (x1, y2 - r), (x1, y2 - r - d), color, thickness)
        cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness)
        # Bottom right
        cv2.line(img, (x2 - r, y2), (x2 - r - d, y2), color, thickness)
        cv2.line(img, (x2, y2 - r), (x2, y2 - r - d), color, thickness)
        cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness)

        cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1, cv2.LINE_AA)
        cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r - d), color, -1, cv2.LINE_AA)
        
        cv2.circle(img, (x1 + r, y1 + r), 2, color, 12)
        cv2.circle(img, (x2 - r, y1 + r), 2, color, 12)
        cv2.circle(img, (x1 + r, y2 - r), 2, color, 12)
        cv2.circle(img, (x2 - r, y2 - r), 2, color, 12)
        
        return img

    def UI_box(self, x, img, color=None, label=None, line_thickness=None):
        """Draw UI box (from deep_sort_tracking_id.py)"""
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        color = color or [np.random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        if label:
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            img = self.draw_border(img, (c1[0], c1[1] - t_size[1] - 3), (c1[0] + t_size[0], c1[1] + 3), color, 1, 8, 2)
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    def draw_boxes(self, img, bbox, object_id, identities=None, offset=(0, 0)):
        """Draw tracking boxes with trails (adapted from deep_sort_tracking_id.py)"""
        height, width, _ = img.shape
        
        # Remove tracked point from buffer if object is lost
        for key in list(self.data_deque):
            if key not in identities:
                self.data_deque.pop(key)

        moving_objects = []

        for i, box in enumerate(bbox):
            x1, y1, x2, y2 = [int(i) for i in box]
            x1 += offset[0]
            x2 += offset[0]
            y1 += offset[1]
            y2 += offset[1]

            # Code to find center of bottom edge
            center = (int((x2 + x1) / 2), int((y2 + y2) / 2))

            # Get ID of object
            id = int(identities[i]) if identities is not None else 0

            # Create new buffer for new object
            if id not in self.data_deque:  
                self.data_deque[id] = deque(maxlen=self.trail_length)

            color = self.compute_color_for_labels(object_id[i])
            obj_name = self.names[object_id[i]]
            label = '{}{:d}'.format("", id) + ":" + '%s' % (obj_name)

            # Add center to buffer
            self.data_deque[id].appendleft(center)
            
            # Check if object is moving
            is_moving = self.is_track_moving(id)
            if is_moving:
                velocity = self.calculate_track_velocity(id)
                world_pos = self.pixel_to_world(center)
                
                moving_objects.append({
                    'track_id': id,
                    'world_x': world_pos[0],
                    'world_y': world_pos[1],
                    'world_z': 0.0,
                    'confidence': 0.8,  # You could pass actual confidence from detection
                    'velocity': velocity,
                    'class': object_id[i],
                    'bbox': [x1, y1, x2-x1, y2-y1],
                    'center': center
                })

            self.UI_box(box, img, label=label, color=color, line_thickness=2)
            
            # Draw trail
            for j in range(1, len(self.data_deque[id])):
                if self.data_deque[id][j - 1] is None or self.data_deque[id][j] is None:
                    continue
                thickness = int(np.sqrt(self.trail_length / float(j + j)) * 1.5)
                cv2.line(img, self.data_deque[id][j - 1], self.data_deque[id][j], color, thickness)
                
        return img, moving_objects

    def is_track_moving(self, track_id):
        """Check if tracked object is moving"""
        positions = list(self.data_deque[track_id])
        if len(positions) < 5:
            return False
            
        recent_positions = positions[-10:]  # Last 10 positions
        if len(recent_positions) < 3:
            return False
            
        start_pos = recent_positions[0]
        end_pos = recent_positions[-1]
        displacement = np.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        
        return displacement > self.motion_threshold

    def calculate_track_velocity(self, track_id):
        """Calculate velocity of tracked object"""
        positions = list(self.data_deque[track_id])
        
        if len(positions) < 2:
            return 0.0
            
        recent_positions = positions[-5:]
        if len(recent_positions) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i][0] - recent_positions[i-1][0]
            dy = recent_positions[i][1] - recent_positions[i-1][1]
            distance = np.sqrt(dx**2 + dy**2)
            total_distance += distance
            
        # Estimate velocity (assuming 30 FPS)
        velocity_pixels_per_frame = total_distance / len(recent_positions)
        velocity_pixels_per_second = velocity_pixels_per_frame * 30  # Assuming 30 FPS
        return velocity_pixels_per_second * 0.01  # Convert to world units

    def detect_and_track_yolo_deepsort(self, image):
        """YOLOv7 + DeepSORT detection and tracking (adapted from deep_sort_tracking_id.py)"""
        im0 = image.copy()
        
        # Preprocess image
        img = self.letterbox(image, self.img_size, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_threshold, self.iou_threshold, classes=None, agnostic=False)
        t3 = time_synchronized()

        moving_targets = []
        
        # Process detections
        for i, det in enumerate(pred):
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                xywh_bboxs = []
                confs = []
                oids = []
                
                # Prepare data for DeepSORT
                for *xyxy, conf, cls in reversed(det):
                    x_c, y_c, bbox_w, bbox_h = self.xyxy_to_xywh(*xyxy)
                    xywh_obj = [x_c, y_c, bbox_w, bbox_h]
                    xywh_bboxs.append(xywh_obj)
                    confs.append([conf.item()])
                    oids.append(int(cls))

                xywhs = torch.Tensor(xywh_bboxs)
                confss = torch.Tensor(confs)
                
                # Update DeepSORT
                outputs = self.deepsort.update(xywhs, confss, oids, im0)
                
                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -2]
                    object_id = outputs[:, -1]

                    # Draw boxes and get moving targets
                    im0, moving_targets = self.draw_boxes(im0, bbox_xyxy, object_id, identities)

        self.get_logger().debug(f'YOLOv7 inference: {(1E3 * (t2 - t1)):.1f}ms, NMS: {(1E3 * (t3 - t2)):.1f}ms')
        return moving_targets, im0

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        """Resize and pad image while meeting stride-multiple constraints"""
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)

    def image_callback(self, msg):
        """Main image processing callback"""
        try:
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'[DEBUG] Processing frame {self.frame_count}')
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if self.use_yolo_deepsort:
                moving_targets, vis_image = self.detect_and_track_yolo_deepsort(cv_image)
                self.last_detection_count = len(self.data_deque)
            elif self.use_yolo_only:
                detections, vis_image = self.detect_yolo_only(cv_image)
                moving_targets = self.track_yolo_detections_basic(detections)
                vis_image = self.draw_yolo_detections(vis_image, detections, moving_targets)
                self.last_detection_count = len(detections)
            else:
                detections = self.detect_motion_opencv(cv_image)
                moving_targets = self.track_motion_basic(detections)
                vis_image = self.draw_opencv_detections(cv_image, detections, moving_targets)
                self.last_detection_count = len(detections)
            
            self.last_moving_count = len(moving_targets)
            
            if self.last_moving_count > 0:
                self.get_logger().info(f'✓ Frame {self.frame_count}: {self.last_moving_count} moving targets detected')
            
            # Publish results
            self.publish_moving_targets(moving_targets, msg.header)
            
            # Publish visualization
            if self.enable_vis and vis_image is not None:
                self.publish_visualization(vis_image, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'[DEBUG] Image processing error: {e}')
            import traceback
            self.get_logger().error(f'[DEBUG] Full traceback: {traceback.format_exc()}')

    def detect_motion_opencv(self, image):
        """OpenCV motion detection fallback with debug info"""
        self.get_logger().debug("[DEBUG] Running OpenCV motion detection...")
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        fg_mask = self.background_subtractor.apply(gray)
        
        # Morphological operations
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w / 2
                center_y = y + h / 2
                
                detections.append({
                    'bbox': [x, y, w, h],
                    'center': [center_x, center_y],
                    'confidence': 0.8,
                    'class': 0,  # Unknown object class
                    'area': area
                })
        
        return detections

    def track_motion_basic(self, detections):
        """Basic motion tracking fallback"""
        current_time = time.time()
        moving_targets = []
        
        for detection in detections:
            center = detection['center']
            
            # Find closest previous detection
            best_track_id = None
            min_distance = float('inf')
            
            for track_id, prev_data in self.previous_detections.items():
                prev_center = prev_data['center']
                distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                
                if distance < min_distance and distance < 100:
                    min_distance = distance
                    best_track_id = track_id
            
            if best_track_id is None:
                track_id = self.next_track_id
                self.next_track_id += 1
            else:
                track_id = best_track_id
            
            # Update tracking data
            if track_id not in self.data_deque:
                self.data_deque[track_id] = deque(maxlen=self.trail_length)
                
            self.data_deque[track_id].appendleft(center)
            self.previous_detections[track_id] = {
                'center': center,
                'timestamp': current_time
            }
            
            # Check if moving
            if self.is_track_moving(track_id):
                world_pos = self.pixel_to_world(center)
                velocity = self.calculate_track_velocity(track_id)
                
                moving_targets.append({
                    'track_id': track_id,
                    'world_x': world_pos[0],
                    'world_y': world_pos[1],
                    'world_z': 0.0,
                    'confidence': detection['confidence'],
                    'velocity': velocity,
                    'class': detection['class'],
                    'bbox': detection['bbox']
                })
        
        # Clean up old tracks
        self.cleanup_old_tracks(current_time)
        return moving_targets

    def draw_opencv_detections(self, image, detections, moving_targets):
        """Draw OpenCV detections with trails"""
        vis_image = image.copy()
        moving_track_ids = {target['track_id'] for target in moving_targets}
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            center = detection['center']
            
            # Find track ID
            track_id = None
            for tid, prev_data in self.previous_detections.items():
                prev_center = prev_data['center']
                distance = np.sqrt((center[0] - prev_center[0])**2 + (center[1] - prev_center[1])**2)
                if distance < 50:
                    track_id = tid
                    break
            
            is_moving = track_id in moving_track_ids if track_id else False
            color = (0, 255, 0) if is_moving else (255, 0, 0)
            thickness = 3 if is_moving else 2
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), color, thickness)
            
            # Draw trail if available
            if track_id and track_id in self.data_deque:
                positions = list(self.data_deque[track_id])
                for i in range(1, len(positions)):
                    if positions[i - 1] is None or positions[i] is None:
                        continue
                    thickness_trail = int(np.sqrt(self.trail_length / float(i + i)) * 1.5)
                    cv2.line(vis_image, 
                           (int(positions[i - 1][0]), int(positions[i - 1][1])), 
                           (int(positions[i][0]), int(positions[i][1])), 
                           color, thickness_trail)
            
            # Draw label
            label = f"ID:{track_id}" if track_id else "New"
            if is_moving:
                velocity = next((t['velocity'] for t in moving_targets if t['track_id'] == track_id), 0)
                label += f" V:{velocity:.1f}"
            
            cv2.putText(vis_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add status text
        status = f"{'YOLO+DeepSORT' if self.use_yolo_deepsort else 'OpenCV'} | Frame: {self.frame_count} | Moving: {len(moving_targets)}"
        cv2.putText(vis_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return vis_image

    def cleanup_old_tracks(self, current_time, max_age=3.0):
        """Remove old tracks that haven't been updated"""
        tracks_to_remove = []
        
        for track_id in list(self.previous_detections.keys()):
            last_seen = self.previous_detections[track_id]['timestamp']
            if current_time - last_seen > max_age:
                tracks_to_remove.append(track_id)
        
        for track_id in tracks_to_remove:
            if track_id in self.data_deque:
                del self.data_deque[track_id]
            if track_id in self.previous_detections:
                del self.previous_detections[track_id]

    def pixel_to_world(self, pixel_center):
        """Convert pixel coordinates to world coordinates"""
        center_x, center_y = pixel_center
        world_x = (center_x - 320) * 0.01
        world_y = (center_y - 240) * 0.01
        return [world_x, world_y]

    def publish_moving_targets(self, moving_targets, header):
        """Publish detected moving targets"""
        for target in moving_targets:
            detection_msg = TargetDetection()
            detection_msg.header = header
            detection_msg.vehicle_name = self.vehicle_name
            detection_msg.target_x = target['world_x']
            detection_msg.target_y = target['world_y']
            detection_msg.target_z = target['world_z']
            detection_msg.confidence = target['confidence']
            
            self.detection_pub.publish(detection_msg)
            
            class_name = self.names[target['class']] if target['class'] < len(self.names) else f"class_{target['class']}"
            self.get_logger().info(f'✓ Moving {class_name} detected: ID={target["track_id"]}, '
                                  f'pos=({target["world_x"]:.2f}, {target["world_y"]:.2f}), '
                                  f'velocity={target["velocity"]:.2f}')

    def publish_visualization(self, vis_image, header):
        """Publish visualization image"""
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
            vis_msg.header = header
            self.vis_pub.publish(vis_msg)
                
        except Exception as e:
            self.get_logger().error(f'[DEBUG] Visualization error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotionDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()