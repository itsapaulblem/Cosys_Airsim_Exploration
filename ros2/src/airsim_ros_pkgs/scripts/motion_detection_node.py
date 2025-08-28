#!/usr/bin/env python3
#TODO 
"""
Motion Detection Node for AirSim ROS2 Integration

PURPOSE: 
-------
This node implements advanced computer vision capabilities for autonomous drone systems,
providing real-time detection and tracking of moving objects in the AirSim simulation
environment. It serves as the AI brain for search-and rescue, surveillance and
autonomous target tracking missions.

MAIN FUNCTIONALITY:
------------------
- Real-time Object Detection: Uses YOLOv7 neural network for object recognition
- Motion Analysis: OpenCV-based motion detection as reliable fallback system
- Multi-Object Tracking: Assign unique IDs to detected objects for persistent tracking
- Target Classification: Filters stationary objects, publishing only moving targets
- 360 degree Vision Coverage: Processes 4 camera array for comprehensive situational awaremess
- ROS2 Integration: Publishes detection data via standard ROS 2 topics and services

AI DETECTION WORKFLOW:
---------------------
1. Camera Image Reception --> 2. YOLO Object Detection --> 3. Motion Analysis --> 4. Object Tracking --> 5. Target Classification --> 6. ROS 2 Message Publishing --> 7. Multirotor Control Integration

ROS 2 INTERFACES:
----------------
Subscribers: /droneX/camera0/image (sensor_msgs/Image)
Publishers: /target_detection (airsim_interfaces/TargetDetection)
Parameters: confidence_threshold, motion_threshold, vehicle_name

USAGE IN MISSIONS:
-----------------
- Search Missions: Drone hovers and scans for moving targets
- Track Missions: Drone follows detected target autonomously
- Surveillance Missions: Continuous monitoring with target alerts
- Fleet Coordination: Multiple detection nodes for comprehensive coverage
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from airsim_interfaces.msg import TargetDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import time
from collections import deque
from pathlib import Path
import sys

# YOLOv7 Integration Setup
YOLO_PATH = Path(__file__).parent / 'ai_detection' / 'yolov7'
sys.path.insert(0, str(YOLO_PATH))

try:
    from models.experimental import attempt_load
    from utils.general import non_max_suppression, scale_coords
    from utils.torch_utils import select_device
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"YOLOv7 import error: {e}")
    YOLO_AVAILABLE = False

class MotionDetectionNode(Node):
    def __init__(self):
        super().__init__('motion_detection_node')

        # ROS 2 Parameters Declaration and Retrieval
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('camera_topic', '/drone1/airsim_drone1/camera0/image')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('motion_threshold', 15.0)
        
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)

        self.bridge = CvBridge()

        # AI Detection System Initialization
        if YOLO_AVAILABLE:
            self.device = select_device('')
            self.model = self.load_yolo_model()
            self.use_yolo = self.model is not None
        else:
            self.use_yolo = False
        
        if not self.use_yolo:
            self.get_logger().warn("YOLO not available, using OpenCV motion detection")
            self.setup_opencv_detection()
        
        # Motion tracking variables
        self.previous_detections = {}
        self.position_history = {}
        self.track_id_counter = 0
        self.max_history_length = 10
        
        # ROS2 communication interfaces
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(
            TargetDetection, 'target_detection', 10)
        
        self.get_logger().info(f'Motion Detection node initialized for {self.vehicle_name}')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Motion threshold: {self.motion_threshold} pixels')
        
    def setup_opencv_detection(self):
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(
            detectShadows=True, varThreshold=50)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
    def load_yolo_model(self):
        try:
            weights_path = YOLO_PATH / 'yolov7.pt'
            if not weights_path.exists():
                self.get_logger().error(f'YOLOv7 weights not found at {weights_path}')
                return None
            model = attempt_load(str(weights_path), map_location=self.device)
            model.eval()
            self.get_logger().info('YOLOv7 model loaded successfully')
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return None
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            if self.use_yolo:
                detections = self.detect_objects_yolo(cv_image)
            else:
                detections = self.detect_motion_opencv(cv_image)
            moving_targets = self.track_motion(detections)
            self.publish_moving_targets(moving_targets, msg.header)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def detect_motion_opencv(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        fg_mask = self.background_subtractor.apply(gray)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w / 2
                center_y = y + h / 2
                detections.append({
                    'bbox': [x, y, w, h],
                    'center': [center_x, center_y],
                    'confidence': 0.8,
                    'class': 0
                })
        return detections
    
    def detect_objects_yolo(self, image):
        if self.model is None:
            return []
        try:
            img = cv2.resize(image, (640, 640))
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device).float() / 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            with torch.no_grad():
                pred = self.model(img)[0]
            pred = non_max_suppression(pred, self.conf_threshold, 0.45)
            detections = []
            for det in pred:
                if det is not None and len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], image.shape).round()
                    for *box, conf, cls in det:
                        x1, y1, x2, y2 = map(int, box)
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        detections.append({
                            'bbox': [x1, y1, x2 - x1, y2 - y1],
                            'center': [center_x, center_y],
                            'confidence': float(conf),
                            'class': int(cls)
                        })
            return detections
        except Exception as e:
            self.get_logger().error(f'YOLO detection error: {e}')
            return []
    
    def track_motion(self, current_detections):
        current_time = time.time()
        moving_targets = []
        matched_detections = {}
        unmatched_detections = current_detections.copy()
        
        # Clean up old detections
        for track_id, prev_data in list(self.previous_detections.items()):
            if current_time - prev_data['timestamp'] > 2.0:
                if track_id in self.position_history:
                    del self.position_history[track_id]
                del self.previous_detections[track_id]
                continue
                
            # Find best match for existing track
            best_match = None
            best_distance = float('inf')
            for i, detection in enumerate(unmatched_detections):
                prev_center = prev_data['center']
                curr_center = detection['center']
                distance = np.sqrt((prev_center[0] - curr_center[0])**2 + 
                                 (prev_center[1] - curr_center[1])**2)
                if distance < 100 and distance < best_distance:
                    best_distance = distance
                    best_match = i
            
            if best_match is not None:
                matched_detections[track_id] = unmatched_detections.pop(best_match)
        
        # Update existing tracks
        for track_id, detection in matched_detections.items():
            if track_id not in self.position_history:
                self.position_history[track_id] = deque(maxlen=self.max_history_length)
            
            self.position_history[track_id].append(detection['center'])
            self.previous_detections[track_id] = {
                'bbox': detection['bbox'],
                'center': detection['center'],
                'confidence': detection['confidence'],
                'class': detection['class'],
                'timestamp': current_time
            }
            
            # Check if moving
            if self.is_moving(track_id):
                world_pos = self.pixel_to_world(detection['center'])
                moving_targets.append({
                    'track_id': track_id,
                    'world_x': world_pos[0],
                    'world_y': world_pos[1],
                    'world_z': 0.0,
                    'confidence': detection['confidence'],
                    'velocity': self.calculate_velocity(track_id)
                })
        
        # Create new tracks for unmatched detections
        for detection in unmatched_detections:
            new_track_id = self.track_id_counter
            self.track_id_counter += 1
            
            self.previous_detections[new_track_id] = {
                'bbox': detection['bbox'],
                'center': detection['center'],
                'confidence': detection['confidence'],
                'class': detection['class'],
                'timestamp': current_time
            }
            self.position_history[new_track_id] = deque(maxlen=self.max_history_length)
            self.position_history[new_track_id].append(detection['center'])
            
        return moving_targets
    
    def is_moving(self, track_id):
        if track_id not in self.position_history:
            return False
        
        positions = list(self.position_history[track_id])
        if len(positions) < 3:
            return False
            
        start_pos = positions[0]
        end_pos = positions[-1]
        total_displacement = np.sqrt((end_pos[0] - start_pos[0])**2 + 
                                   (end_pos[1] - start_pos[1])**2)
        
        return total_displacement > self.motion_threshold
    
    def calculate_velocity(self, track_id):
        if track_id not in self.position_history:
            return 0.0
            
        positions = list(self.position_history[track_id])
        if len(positions) < 2:
            return 0.0
            
        recent_positions = positions[-min(5, len(positions)):]
        if len(recent_positions) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i][0] - recent_positions[i-1][0]
            dy = recent_positions[i][1] - recent_positions[i-1][1]
            distance = np.sqrt(dx**2 + dy**2)
            total_distance += distance
            
        velocity_pixels_per_frame = total_distance / (len(recent_positions) - 1)
        velocity_world = velocity_pixels_per_frame * 0.02
        return velocity_world
    
    def pixel_to_world(self, pixel_center):
        center_x, center_y = pixel_center
        world_x = (center_x - 320) * 0.01
        world_y = (center_y - 240) * 0.01
        return [world_x, world_y]
    
    def publish_moving_targets(self, moving_targets, header):
        for target in moving_targets:
            detection_msg = TargetDetection()
            detection_msg.header = header
            detection_msg.vehicle_name = self.vehicle_name
            detection_msg.target_x = target['world_x']
            detection_msg.target_y = target['world_y']
            detection_msg.target_z = target['world_z']
            detection_msg.confidence = target['confidence']
            
            self.detection_pub.publish(detection_msg)
            
            self.get_logger().info(f'Moving target detected: '
                                  f'ID={target["track_id"]}, '
                                  f'pos=({target["world_x"]:.2f}, {target["world_y"]:.2f}), '
                                  f'confidence={target["confidence"]:.2f}')

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