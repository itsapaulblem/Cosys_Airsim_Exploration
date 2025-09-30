#!/usr/bin/env python3
"""
Multi-Drone Motion Detection Node with Individual Drone Behaviors
Expands motion_detection_node.py to support multiple drones with identical individual behaviors plus coordination.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from geometry_msgs.msg import Vector3Stamped, TwistStamped, PoseStamped
from std_msgs.msg import Float64
from airsim_interfaces.srv import Takeoff, Land
from airsim_interfaces.msg import VelCmd, GPSYaw
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from threading import Timer, RLock
import time
import sys
import os
import traceback
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

# Add detection path to system path - FIXED PATH
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DETECTION_PATH = os.path.join(SCRIPT_DIR, 'YOLOv7-DeepSORT-Object-Tracking')
sys.path.insert(0, DETECTION_PATH)

# Try to import YOLO and DeepSORT (same as individual node)
YOLO_DEEPSORT_AVAILABLE = False
try:
    import torch
    import yaml
    from pathlib import Path
    from utils.general import non_max_suppression, scale_coords, xyxy2xywh
    from utils.torch_utils import select_device
    from models.experimental import attempt_load
    YOLO_DEEPSORT_AVAILABLE = True
    print("YOLOv7+DeepSORT imports successful")
except ImportError as e:
    print(f"YOLOv7+DeepSORT not available: {e}")

# Try to import AirSim
try:
    import airsim
    AIRSIM_AVAILABLE = True
except ImportError:
    print("AirSim not available, running without AirSim integration")
    AIRSIM_AVAILABLE = False

# Additional imports needed
import math
import urllib.request
from collections import deque

# Multi-drone coordination dataclasses
@dataclass
class DroneTarget:
    """Target detected by a specific drone"""
    drone_id: str
    target_id: int
    x: float
    y: float
    z: float
    confidence: float
    last_seen: float
    velocity: float
    camera_id: int
    bbox: List[float]
    world_position: Tuple[float, float, float]

@dataclass
class GlobalTarget:
    """Global target with data from multiple drones"""
    global_id: str
    x: float
    y: float
    z: float
    confidence: float
    last_seen: float
    contributing_drones: List[str]
    primary_drone: str
    track_history: deque
    world_position: Tuple[float, float, float]

class MultiDroneTargetDetection(Node):
    def __init__(self):
        super().__init__('multi_drone_target_detection')
        
        # Parameters - same as individual node
        self.declare_node_parameters()
        self.load_parameters()
        
        # Individual drone behaviors (same as motion_detection_node.py)
        self.bridge = CvBridge()
        self.num_cameras = 4
        self.primary_camera = 0
        
        # Multi-drone coordination data structures
        self.drone_targets: Dict[str, List[DroneTarget]] = {}
        self.global_targets: Dict[str, GlobalTarget] = {}
        self.drone_states: Dict[str, str] = {}
        self.drone_locks: Dict[str, threading.Lock] = {}
        
        # Per-drone tracking data (same structure as individual node)
        self.drone_camera_data: Dict[str, Dict] = {}
        self.drone_following_states: Dict[str, Dict] = {}
        self.drone_control_data: Dict[str, Dict] = {}
        
        # Shared detection system
        self.initialize_detection_systems()
        self.setup_drone_interfaces()
        
        # Multi-drone coordination timers
        self.target_merger_timer = self.create_timer(0.1, self.merge_and_coordinate_targets)
        self.status_timer = self.create_timer(2.0, self.publish_coordination_status)
        
        # Global coordination publishers
        self.global_target_pub = self.create_publisher(
            PoseStamped, '/global_target_detection', 10)
        
        self.get_logger().info(f"Multi-drone detection system initialized for {len(self.drone_names)} drones")

    def declare_node_parameters(self):
        """Declare ROS parameters - same as individual node + multi-drone params"""
        # Individual drone parameters (from motion_detection_node.py)
        self.declare_parameter('drone_names', ['Drone1', 'Drone2'])
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('motion_threshold', 15.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('trail_length', 30)
        self.declare_parameter('enable_following', True)
        self.declare_parameter('follow_distance', 0.5)
        self.declare_parameter('takeoff_height', 3.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('image_quality', 95)
        self.declare_parameter('enable_image_resize', True)
        
        # Multi-drone coordination parameters
        self.declare_parameter('target_merge_distance', 5.0)
        self.declare_parameter('enable_coordination', True)
        self.declare_parameter('formation_type', 'triangle')
        self.declare_parameter('formation_distance', 10.0)
        self.declare_parameter('num_cameras_per_drone', 4)

    def load_parameters(self):
        """Load parameters from ROS - same as individual node + multi-drone params"""
        # Individual drone parameters
        self.drone_names = self.get_parameter('drone_names').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)
        self.enable_vis = self.get_parameter('enable_visualization').value
        self.trail_length = int(self.get_parameter('trail_length').value)
        self.enable_following = self.get_parameter('enable_following').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.image_quality = int(self.get_parameter('image_quality').value)
        self.enable_image_resize = self.get_parameter('enable_image_resize').value
        
        # Multi-drone coordination parameters
        self.target_merge_distance = self.get_parameter('target_merge_distance').value
        self.enable_coordination = self.get_parameter('enable_coordination').value
        self.formation_type = self.get_parameter('formation_type').value
        self.formation_distance = self.get_parameter('formation_distance').value
        self.num_cameras = self.get_parameter('num_cameras_per_drone').value
        
        # Camera orientations (same as individual node)
        self.camera_fov_horizontal = 90.0
        self.camera_fov_vertical = 60.0
        self.camera_orientations = {
            0: {'yaw': 0.0, 'name': 'front'},
            1: {'yaw': 90.0, 'name': 'right'},
            2: {'yaw': 180.0, 'name': 'back'},
            3: {'yaw': -90.0, 'name': 'left'}
        }

    def initialize_detection_systems(self):
        """Initialize YOLO detection system"""
        self.use_yolo_deepsort = False
        self.names = self.load_coco_names()
        
        if YOLO_DEEPSORT_AVAILABLE:
            try:
                self.device = select_device('')
                self.half = self.device.type != 'cpu'
                
                weights_path = Path(DETECTION_PATH) / 'yolov7.pt'
                
                # Download weights if missing
                if not weights_path.exists():
                    self.get_logger().info("Downloading YOLOv7 weights...")
                    import urllib.request
                    try:
                        weights_url = "https://github.com/WongKinYiu/yolov7/releases/download/carassets/yolov7.pt"
                        urllib.request.urlretrieve(weights_url, str(weights_path))
                        self.get_logger().info("YOLOv7 weights downloaded successfully")
                    except Exception as e:
                        self.get_logger().error(f"Failed to download YOLOv7 weights: {e}")
                        self.use_yolo_deepsort = False
                        return
                
                if weights_path.exists():
                    # Load model using the exact same method as motion_detection_node.py
                    try:
                        self.get_logger().info("Loading YOLO model...")
                        # Use torch.load with weights_only=False for PyTorch 2.6 compatibility
                        checkpoint = torch.load(str(weights_path), map_location=self.device, weights_only=False)
                        
                        if isinstance(checkpoint, dict) and 'model' in checkpoint:
                            self.model = checkpoint['model'].float()
                        else:
                            # Try attempt_load as fallback
                            try:
                                self.model = attempt_load(str(weights_path), map_location=self.device)
                            except Exception:
                                self.get_logger().error("Failed to load model with both methods")
                                self.use_yolo_deepsort = False
                                return
                    
                    except Exception as e:
                        self.get_logger().error(f"Model loading failed: {e}")
                        self.use_yolo_deepsort = False
                        return
                    
                    # Set model properties
                    try:
                        self.stride = int(self.model.stride.max()) if hasattr(self.model, 'stride') else 32
                        self.img_size = 640
                        
                        if self.half:
                            self.model.half()
                        
                        self.model.eval()
                        
                        # Test the model with a dummy input
                        test_img = torch.zeros((1, 3, self.img_size, self.img_size)).to(self.device)
                        if self.half:
                            test_img = test_img.half()
                        
                        with torch.no_grad():
                            _ = self.model(test_img)
                        
                        self.get_logger().info("YOLO model test successful")
                        
                    except Exception as e:
                        self.get_logger().error(f"Model initialization test failed: {e}")
                        self.use_yolo_deepsort = False
                        return
                    
                    # Initialize simple tracking (DeepSORT often has compatibility issues)
                    self.trackers = {}
                    self.track_id_counter = 0
                    for drone_name in self.drone_names:
                        self.trackers[drone_name] = {}
                    
                    self.use_yolo_deepsort = True
                    self.get_logger().info("YOLOv7 initialized successfully")
                else:
                    self.get_logger().warn(f"YOLO weights not found at {weights_path}")
                    self.use_yolo_deepsort = False
                    
            except Exception as e:
                self.get_logger().error(f"Failed to initialize YOLO: {e}")
                traceback.print_exc()
                self.use_yolo_deepsort = False
        else:
            self.get_logger().warn("YOLOv7+DeepSORT not available, running without AI detection")

    def setup_drone_interfaces(self):
        """Setup ROS interfaces for all drones"""
        self.image_subs = {}
        self.cmd_vel_pubs = {}
        self.takeoff_clients = {}
        self.land_clients = {}
        self.drone_camera_data = {}
        
        for drone_name in self.drone_names:
            self.drone_targets[drone_name] = []
            self.drone_states[drone_name] = 'IDLE'
            self.drone_locks[drone_name] = threading.Lock()
            self.drone_camera_data[drone_name] = {}
            
            self.image_subs[drone_name] = {}
            
            # Initialize visualization publishers
            if not hasattr(self, 'vis_pubs'):
                self.vis_pubs = {}
            if self.enable_vis:
                self.vis_pubs[drone_name] = {}
            
            for cam_id in range(self.num_cameras):
                # FIXED: Use proper drone names (Drone1, Drone2 not drone1, drone2)
                topic_name = f'/{drone_name}/camera{cam_id}/image'
                self.image_subs[drone_name][cam_id] = self.create_subscription(
                    Image, topic_name,
                    lambda msg, d=drone_name, c=cam_id: self.image_callback(msg, d, c),
                    10
                )
                
                # Add visualization publisher for each camera (viewable in rqt_image_view)
                if self.enable_vis:
                    vis_topic = f'/{drone_name}/camera{cam_id}/detection_image'
                    self.vis_pubs[drone_name][cam_id] = self.create_publisher(Image, vis_topic, 10)
                
                self.drone_camera_data[drone_name][cam_id] = {
                    'data_deque': {},
                    'frame_count': 0,
                    'last_image': None,
                    'background_subtractor': cv2.createBackgroundSubtractorMOG2(),
                    'previous_frame': None
                }
            
            if self.enable_following:
                self.cmd_vel_pubs[drone_name] = self.create_publisher(
                    VelCmd, f'/{drone_name}/vel_cmd_body_frame', 10)
                
                self.takeoff_clients[drone_name] = self.create_client(
                    Takeoff, f'/{drone_name}/takeoff')
                
                self.land_clients[drone_name] = self.create_client(
                    Land, f'/{drone_name}/land')
            
            self.get_logger().info(f"Setup interfaces for drone: {drone_name}")

    def load_coco_names(self):
        """Load COCO dataset class names"""
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

    def image_callback(self, msg, drone_name, camera_id):
        """Process image from specific drone camera with crosshairs (same as individual node)"""
        try:
            with self.drone_locks[drone_name]:
                if msg.encoding == 'rgb8':
                    cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                if cv_image.shape[1] != self.image_width or cv_image.shape[0] != self.image_height:
                    cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
                
                self.drone_camera_data[drone_name][camera_id]['last_image'] = cv_image
                self.drone_camera_data[drone_name][camera_id]['frame_count'] += 1
                
                # Create visualization image with crosshairs (same as individual node)
                vis_image = cv_image.copy()
                self.draw_crosshairs(vis_image)
                
                # Process detection
                targets = []
                if self.use_yolo_deepsort:
                    targets = self.detect_targets_yolo(cv_image, drone_name, camera_id, vis_image)
                else:
                    targets = self.detect_motion_opencv(cv_image, drone_name, camera_id, vis_image)
                
                if targets:
                    self.update_drone_targets(drone_name, camera_id, targets, msg.header)
                
                # Publish visualization with crosshairs (viewable in rqt_image_view)
                if self.enable_vis and hasattr(self, 'vis_pubs') and drone_name in self.vis_pubs and camera_id in self.vis_pubs[drone_name]:
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                    vis_msg.header = msg.header
                    self.vis_pubs[drone_name][camera_id].publish(vis_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing {drone_name}/camera{camera_id}: {e}")

    def draw_crosshairs(self, image):
        """Draw crosshairs in center of image - same as individual node"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Green crosshairs
        color = (0, 255, 0)  # BGR green
        thickness = 2
        length = 30
        
        # Horizontal line
        cv2.line(image, (center_x - length, center_y), (center_x + length, center_y), color, thickness)
        # Vertical line  
        cv2.line(image, (center_x, center_y - length), (center_x, center_y + length), color, thickness)
        
        # Center dot
        cv2.circle(image, (center_x, center_y), 3, color, -1)
    
    def detect_motion_opencv(self, image, drone_name, camera_id, vis_image):
        """OpenCV motion detection fallback with visualization"""
        try:
            camera_data = self.drone_camera_data[drone_name][camera_id]
            
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            targets = []
            if camera_data['previous_frame'] is not None:
                # Calculate frame difference
                diff = cv2.absdiff(camera_data['previous_frame'], gray)
                _, thresh = cv2.threshold(diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
                
                # Find contours
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for i, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area > 500:  # Minimum area threshold
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Draw bounding box on visualization
                        cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(vis_image, "Motion", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        world_pos = self.pixel_to_world_position([center_x, center_y], drone_name, camera_id)
                        
                        target = DroneTarget(
                            drone_id=drone_name,
                            target_id=i,
                            x=float(center_x),
                            y=float(center_y),
                            z=0.0,
                            confidence=0.5,
                            last_seen=time.time(),
                            velocity=0.0,
                            camera_id=camera_id,
                            bbox=[float(x), float(y), float(w), float(h)],
                            world_position=world_pos
                        )
                        targets.append(target)
            
            camera_data['previous_frame'] = gray
            return targets
            
        except Exception as e:
            self.get_logger().error(f"Motion detection error: {str(e)}")
            return []
    
    def detect_targets_yolo(self, image, drone_name, camera_id, vis_image):
        """YOLO detection for specific drone camera with visualization"""
        try:
            im0 = image.copy()
            img = self.letterbox(image, self.img_size, stride=self.stride)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            
            with torch.no_grad():
                pred = self.model(img)[0]
            
            pred = non_max_suppression(pred, self.conf_threshold, 0.45, classes=[0], agnostic=False)
            
            targets = []
            for i, det in enumerate(pred):
                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                    
                    for *xyxy, conf, cls in reversed(det):
                        if int(cls) == 0:  # Person class only
                            x1, y1, x2, y2 = xyxy
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            # Draw bounding box and label on visualization
                            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                            label = f'person {conf:.2f}'
                            cv2.putText(vis_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                            
                            world_pos = self.pixel_to_world_position([center_x, center_y], drone_name, camera_id)
                            
                            # Simple tracking - assign unique ID
                            self.track_id_counter += 1
                            
                            target = DroneTarget(
                                drone_id=drone_name,
                                target_id=self.track_id_counter,
                                x=float(center_x),
                                y=float(center_y),
                                z=0.0,
                                confidence=float(conf),
                                last_seen=time.time(),
                                velocity=0.0,
                                camera_id=camera_id,
                                bbox=[float(x1), float(y1), float(x2-x1), float(y2-y1)],
                                world_position=world_pos
                            )
                            targets.append(target)
            
            return targets
            
        except Exception as e:
            self.get_logger().error(f"YOLO detection error for {drone_name}: {e}")
            return []

    def update_drone_targets(self, drone_name, camera_id, targets, header):
        """Update targets for specific drone with individual behaviors (same as motion_detection_node)"""
        current_time = time.time()
        self.drone_targets[drone_name] = targets
        
        if targets:
            # Check for person detection to trigger takeoff (same as individual node)
            person_detected = any(target.drone_id == drone_name and target.confidence > self.conf_threshold for target in targets)
            
            if person_detected and self.drone_states[drone_name] == 'IDLE' and self.enable_following:
                self.initiate_takeoff(drone_name)
                self.drone_states[drone_name] = 'TARGET_DETECTED'
                self.get_logger().info(f"{drone_name}: Person detected! Initiating takeoff and tracking mode")
            elif targets:
                if self.drone_states[drone_name] == 'IDLE':
                    self.drone_states[drone_name] = 'TARGET_DETECTED'
                    self.get_logger().info(f"{drone_name}: Target detected, switching to tracking mode")
    
    def initiate_takeoff(self, drone_name):
        """Initiate takeoff for specific drone (same as individual node)"""
        if drone_name in self.takeoff_clients and self.enable_following:
            if not self.takeoff_clients[drone_name].wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"{drone_name}: Takeoff service not available")
                return
            
            request = Takeoff.Request()
            request.waitonlastask = True
            
            future = self.takeoff_clients[drone_name].call_async(request)
            future.add_done_callback(lambda fut, d=drone_name: self.takeoff_response_callback(fut, d))
            
            self.get_logger().info(f"{drone_name}: Takeoff initiated")
    
    def takeoff_response_callback(self, future, drone_name):
        """Handle takeoff service response for specific drone"""
        try:
            response = future.result()
            if response.ack:
                self.drone_states[drone_name] = 'AIRBORNE'
                self.get_logger().info(f"{drone_name}: Takeoff successful, now airborne")
            else:
                self.get_logger().warn(f"{drone_name}: Takeoff failed")
                self.drone_states[drone_name] = 'IDLE'
        except Exception as e:
            self.get_logger().error(f"{drone_name}: Takeoff service call failed: {str(e)}")
            self.drone_states[drone_name] = 'IDLE'

    def merge_and_coordinate_targets(self):
        """Merge targets from all drones and coordinate response"""
        current_time = time.time()
        
        all_drone_targets = []
        for drone_name, targets in self.drone_targets.items():
            for target in targets:
                if current_time - target.last_seen < 2.0:
                    all_drone_targets.append(target)
        
        if not all_drone_targets:
            return
        
        merged_targets = self.merge_nearby_targets(all_drone_targets)
        self.update_global_targets(merged_targets)
        
        if self.enable_coordination:
            self.coordinate_drone_actions()

    def merge_nearby_targets(self, drone_targets):
        """Merge targets that are likely the same object"""
        merged = []
        used_targets = set()
        
        for i, target1 in enumerate(drone_targets):
            if i in used_targets:
                continue
            
            cluster = [target1]
            used_targets.add(i)
            
            for j, target2 in enumerate(drone_targets[i+1:], i+1):
                if j in used_targets:
                    continue
                
                dist = math.sqrt(
                    (target1.world_position[0] - target2.world_position[0])**2 +
                    (target1.world_position[1] - target2.world_position[1])**2 +
                    (target1.world_position[2] - target2.world_position[2])**2
                )
                
                if dist < self.target_merge_distance:
                    cluster.append(target2)
                    used_targets.add(j)
            
            if cluster:
                merged_target = self.create_merged_target(cluster)
                merged.append(merged_target)
        
        return merged

    def create_merged_target(self, target_cluster):
        """Create a merged target from multiple drone detections"""
        total_weight = sum(t.confidence for t in target_cluster)
        
        avg_x = sum(t.world_position[0] * t.confidence for t in target_cluster) / total_weight
        avg_y = sum(t.world_position[1] * t.confidence for t in target_cluster) / total_weight
        avg_z = sum(t.world_position[2] * t.confidence for t in target_cluster) / total_weight
        
        max_confidence = max(t.confidence for t in target_cluster)
        contributing_drones = list(set(t.drone_id for t in target_cluster))
        primary_drone = max(target_cluster, key=lambda t: t.confidence).drone_id
        
        global_id = f"global_{int(time.time() * 1000)}"
        
        return GlobalTarget(
            global_id=global_id,
            x=avg_x, y=avg_y, z=avg_z,
            confidence=max_confidence,
            last_seen=time.time(),
            contributing_drones=contributing_drones,
            primary_drone=primary_drone,
            track_history=deque(maxlen=50),
            world_position=(avg_x, avg_y, avg_z)
        )

    def update_global_targets(self, merged_targets):
        """Update global target tracking"""
        current_time = time.time()
        
        expired_targets = [global_id for global_id, target in self.global_targets.items() 
                          if current_time - target.last_seen > 5.0]
        
        for global_id in expired_targets:
            del self.global_targets[global_id]
        
        for merged_target in merged_targets:
            matched_id = self.match_global_target(merged_target)
            
            if matched_id:
                existing = self.global_targets[matched_id]
                existing.x, existing.y, existing.z = merged_target.x, merged_target.y, merged_target.z
                existing.confidence = max(existing.confidence, merged_target.confidence)
                existing.last_seen = current_time
                existing.track_history.append((merged_target.x, merged_target.y, current_time))
            else:
                merged_target.track_history.append((merged_target.x, merged_target.y, current_time))
                self.global_targets[merged_target.global_id] = merged_target
        
        if self.global_targets:
            best_target = max(self.global_targets.values(), key=lambda t: t.confidence)
            self.publish_global_target(best_target)

    def match_global_target(self, merged_target):
        """Match merged target with existing global target"""
        for global_id, existing in self.global_targets.items():
            distance = math.sqrt(
                (existing.x - merged_target.x)**2 +
                (existing.y - merged_target.y)**2 +
                (existing.z - merged_target.z)**2
            )
            if distance < self.target_merge_distance:
                return global_id
        return None

    def coordinate_drone_actions(self):
        """Coordinate actions between drones"""
        if not self.global_targets:
            return
        
        best_target = max(self.global_targets.values(), key=lambda t: t.confidence)
        roles = self.assign_drone_roles(best_target)
        
        if self.enable_following:
            self.execute_formation_following(best_target, roles)

    def assign_drone_roles(self, target):
        """Assign roles to drones for coordinated tracking"""
        roles = {}
        if target.primary_drone in self.drone_names:
            roles[target.primary_drone] = 'PRIMARY_TRACKER'
        
        support_drones = [d for d in self.drone_names if d != target.primary_drone]
        angles = [120, 240] if self.formation_type == 'triangle' else [90, 270]
        
        for i, drone in enumerate(support_drones[:len(angles)]):
            roles[drone] = f'SUPPORT_{angles[i]}'
        
        return roles

    def execute_formation_following(self, target, roles):
        """Execute coordinated formation following"""
        for drone_name, role in roles.items():
            if drone_name in self.cmd_vel_pubs:
                try:
                    vel_cmd = self.calculate_formation_velocity(drone_name, target, role)
                    self.cmd_vel_pubs[drone_name].publish(vel_cmd)
                except Exception as e:
                    self.get_logger().error(f"Formation control error for {drone_name}: {e}")

    def calculate_formation_velocity(self, drone_name, target, role):
        """Calculate velocity command for drone in formation"""
        vel_cmd = VelCmd()
        
        if role == 'PRIMARY_TRACKER':
            vel_cmd.twist.linear.x = 1.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 0.5
        elif role.startswith('SUPPORT_'):
            angle = int(role.split('_')[1])
            offset_x = self.formation_distance * math.cos(math.radians(angle))
            offset_y = self.formation_distance * math.sin(math.radians(angle))
            
            vel_cmd.twist.linear.x = 0.8
            vel_cmd.twist.linear.y = offset_y * 0.1
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 0.2
        
        return vel_cmd

    def publish_global_target(self, target):
        """Publish the best global target"""
        detection_msg = PoseStamped()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "world"
        detection_msg.pose.position.x = target.x
        detection_msg.pose.position.y = target.y
        detection_msg.pose.position.z = target.z
        detection_msg.pose.orientation.w = 1.0  # Default orientation
        
        self.global_target_pub.publish(detection_msg)

    def individual_drone_control_loop(self, drone_name):
        """Individual drone control loop (same as motion_detection_node)"""
        if not self.enable_following:
            return
        
        # Simple hover behavior - in a full implementation this would contain
        # the complete PID control logic from motion_detection_node.py
        if drone_name in self.cmd_vel_pubs and self.drone_states[drone_name] == 'AIRBORNE':
            # Check if we have recent targets for this drone
            current_time = time.time()
            recent_targets = [t for t in self.drone_targets.get(drone_name, []) 
                            if current_time - t.last_seen < 2.0]
            
            if recent_targets:
                # Simple following behavior - would be enhanced with full PID
                vel_cmd = VelCmd()
                vel_cmd.twist.linear.x = 0.5  # Slow forward
                vel_cmd.twist.linear.y = 0.0
                vel_cmd.twist.linear.z = 0.0
                vel_cmd.twist.angular.z = 0.2  # Slow rotation
                self.cmd_vel_pubs[drone_name].publish(vel_cmd)
            else:
                # Hover in place
                vel_cmd = VelCmd()
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.linear.y = 0.0
                vel_cmd.twist.linear.z = 0.0
                vel_cmd.twist.angular.z = 0.1  # Slow search rotation
                self.cmd_vel_pubs[drone_name].publish(vel_cmd)

    def publish_coordination_status(self):
        """Publish coordination status"""
        active_drones = len([d for d in self.drone_states.values() if d != 'IDLE'])
        total_targets = len(self.global_targets)
        
        self.get_logger().info(f"Multi-drone status: {active_drones}/{len(self.drone_names)} drones active, "
                             f"{total_targets} global targets tracked")

    def pixel_to_world_position(self, pixel_center, drone_name, camera_id):
        """Convert pixel coordinates to world position"""
        x_rel = (pixel_center[0] - self.image_width/2) * 0.01
        y_rel = (pixel_center[1] - self.image_height/2) * 0.01
        z_rel = 0.0
        return (x_rel, y_rel, z_rel)

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        """Resize and pad image while meeting stride-multiple constraints"""
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)

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

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MultiDroneTargetDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down multi-drone system")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()