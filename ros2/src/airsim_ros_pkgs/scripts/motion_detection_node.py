#!/usr/bin/env python3
"""
Multi-Camera Motion Detection Node for AirSim ROS2 Integration
Lightweight Client - Consumes detections from YOLOv10 microservice
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from airsim_interfaces.msg import TargetDetection, VelCmd, ObjectDetection, ObjectDetectionArray
from airsim_interfaces.srv import Takeoff, Land
from cv_bridge import CvBridge
import cv2
import numpy as np
import warnings

# Silence FutureWarnings about deprecated numpy aliases
warnings.filterwarnings("ignore", category=FutureWarning)

# Compatibility shim for deprecated numpy aliases
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'bool'):
    np.bool = bool
if not hasattr(np, 'object'):
    np.object = object
if not hasattr(np, 'long'):
    np.long = int

import time
import math
import traceback
from collections import deque
from pathlib import Path
import sys
import threading

class MultiCameraMotionDetectionNode(Node):
    def __init__(self):
        super().__init__('multi_camera_motion_detection_node')

        # ROS 2 Parameters Declaration and Retrieval
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('confidence_threshold', 0.08)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('motion_threshold', 15.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('trail_length', 30)
        self.declare_parameter('enable_following', True)
        self.declare_parameter('follow_distance', 0.1)
        self.declare_parameter('takeoff_height', 5.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('image_quality', 95)
        self.declare_parameter('enable_image_resize', True)

        self.vehicle_name = self.get_parameter('vehicle_name').value
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

        self.num_cameras = 1  # PERFORMANCE: Use only front camera for 4x FPS improvement
        self.primary_camera = 0
        # Updated to match actual AirSim camera topics: /Drone1/Camera_N_Scene/image
        self.camera_topics = [
            f'/{self.vehicle_name}/Camera_0_Scene/image',
            # PERFORMANCE: Disabled 3 extra cameras for 4x FPS boost
            # f'/{self.vehicle_name}/Camera_1_Scene/image',
            # f'/{self.vehicle_name}/Camera_2_Scene/image',
            # f'/{self.vehicle_name}/Camera_3_Scene/image'
        ]

        self.bridge = CvBridge()

        self.camera_frame_counts = {}
        self.camera_fps_timestamps = {}  # Track timestamps for FPS calculation
        self.camera_fps_window = 5.0  # Calculate FPS over 5 second window
        self.camera_last_images = {}
        self.camera_detection_counts = {}
        self.camera_moving_counts = {}
        self.camera_data_deques = {}
        self.all_camera_targets = {}
        self.camera_locks = {}
        self.target_person_data_lock = threading.Lock()
        self.target_data_timestamp = 0.0
        self.use_yolo_deepsort = False
        self.names = self.load_coco_names()
        self.palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)
        self.target_person_id = None
        self.target_camera_id = None
        self.last_person_detection_time = 0.0
        self.following_active = False
        self.camera_fov_horizontal = 90.0
        self.camera_fov_vertical = 60.0
        self.camera_orientations = {
            0: {'yaw': 0.0, 'name': 'front'},
            1: {'yaw': 90.0, 'name': 'right'},
            2: {'yaw': 180.0, 'name': 'back'},
            3: {'yaw': -90.0, 'name': 'left'}
        }
        self.target_person_position = None
        self.target_locked = False
        self.target_lock_confidence = 0.0
        self.target_lock_threshold = 0.4

        # Target position smoothing (simplified)
        self.target_position_buffer = deque(maxlen=5)

        # Drone state management (simplified)
        self.drone_state = 'IDLE'
        self.takeoff_initiated = False
        self.takeoff_complete = False
        self.current_altitude = 0.0
        self.takeoff_start_time = None
        self.last_movement_direction = 0.0
        self.target_person_data = None
        self.initialize_detection_systems()
        self.background_subtractor = None
        self.kernel = None
        self.previous_detections = {}
        self.next_track_id = 1
        self.setup_multi_camera_interfaces()
        if self.enable_following:
            self.cmd_vel_pub = self.create_publisher(
                VelCmd, f'{self.vehicle_name}/vel_cmd_body_frame', 10)
            # Use global services (vehicle nodes create these at root namespace)
            self.takeoff_client = self.create_client(Takeoff,  f'{self.vehicle_name}/takeoff')
            self.land_client = self.create_client(Land,  f'{self.vehicle_name}/land')

        self.debug_timer = self.create_timer(5.0, self.debug_status)
        if self.enable_following:
            self.control_timer = self.create_timer(0.067, self.control_loop)
        self.get_logger().info(f' Multi-Camera Motion Detection node initialized for {self.vehicle_name}')
        self.get_logger().info(f' Using {self.num_cameras} cameras: {", ".join(self.camera_topics)}')
        self.get_logger().info(f' Primary camera: {self.primary_camera} ({self.camera_orientations[self.primary_camera]["name"]})')
        self.get_logger().info(f' Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f' Motion threshold: {self.motion_threshold} pixels')
        self.get_logger().info(f" Person following: {'enabled' if self.enable_following else 'disabled'}")
        self.get_logger().info(f' YOLOv10+DeepSORT: {"enabled" if self.use_yolo_deepsort else "disabled"}')

    def setup_multi_camera_interfaces(self):
        """Setup ROS subscriptions and publishers for all 4 cameras"""
        self.image_subs = {}
        self.vis_pubs = {}

        # Initialize data structures and create interfaces for each camera
        for cam_id in range(self.num_cameras):
            # Initialize per-camera tracking data structures
            self.camera_frame_counts[cam_id] = 0
            self.camera_fps_timestamps[cam_id] = []  # FPS tracking
            self.camera_last_images[cam_id] = None
            self.camera_detection_counts[cam_id] = 0
            self.camera_moving_counts[cam_id] = 0
            self.camera_data_deques[cam_id] = {}
            self.camera_locks[cam_id] = threading.Lock()
            self.all_camera_targets[cam_id] = []

            # Create image subscription for each camera
            self.image_subs[cam_id] = self.create_subscription(
                Image, self.camera_topics[cam_id], 
                lambda msg, cam_id=cam_id: self.image_callback(msg, cam_id), 10)

            # Create visualization publisher for each camera if visualisation is enabled
            if self.enable_vis:
                vis_topic = f'detection_visualization_cam{cam_id}'
                self.vis_pubs[cam_id] = self.create_publisher(Image, vis_topic, 10)

            cam_name = self.camera_orientations[cam_id]['name']
            self.get_logger().info(f'Camera {cam_id} ({cam_name}): {self.camera_topics[cam_id]} -> {vis_topic if self.enable_vis else "no viz"}')

        # Global detection publisher for publishing target detections
        self.detection_pub = self.create_publisher(TargetDetection, 'target_detection', 10)

    def load_coco_names(self):
        """Load COCO dataset class names for object detection"""
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
        """Debug timer callback showing status of all cameras every 5 seconds"""
        camera_status = []
        for cam_id in range(self.num_cameras):
            # Calculate FPS from timestamp window
            timestamps = self.camera_fps_timestamps.get(cam_id, [])
            if len(timestamps) > 1:
                time_span = timestamps[-1] - timestamps[0] if len(timestamps) > 0 else 1.0
                fps = len(timestamps) / time_span if time_span > 0 else 0.0
            else:
                fps = 0.0

            detections = self.camera_detection_counts.get(cam_id, 0)
            moving = self.camera_moving_counts.get(cam_id, 0)
            cam_name = self.camera_orientations[cam_id]['name']
            camera_status.append(f"{cam_name}:[FPS:{fps:.1f} D:{detections} M:{moving}]")

        following_status = f", Following: {self.following_active}, Target ID: {self.target_person_id}, Target Cam: {self.target_camera_id}" if self.enable_following else ""

        self.get_logger().info(f'[DEBUG] State: {self.drone_state}, {" ".join(camera_status)}{following_status}')

    def merge_camera_detections(self):
        """Merge detections from all cameras into global coordinate system"""
        merged_targets = []

        for cam_id, targets in self.all_camera_targets.items():
            if not targets:
                continue

            cam_orientation = self.camera_orientations[cam_id]

            for target in targets:
                transformed_target = target.copy()
                transformed_target['camera_id'] = cam_id
                transformed_target['camera_name'] = cam_orientation['name']

                cam_yaw = math.radians(cam_orientation['yaw'])
                world_x = target['world_x'] * math.cos(cam_yaw) - target['world_y'] * math.sin(cam_yaw)
                world_y = target['world_x'] * math.sin(cam_yaw) + target['world_y'] * math.cos(cam_yaw)

                transformed_target['world_x'] = world_x
                transformed_target['world_y'] = world_y
                transformed_target['global_track_id'] = f"cam{cam_id}_id{target['track_id']}"

                merged_targets.append(transformed_target)

        return merged_targets

    def select_target_person_multi_camera(self, merged_targets):
        """Select the best person target from multiple camera views"""
        # TEMPORARY: Disabled class filter to test if YOLO is detecting anything
        # persons = [target for target in merged_targets if target.get('class', -1) == 0]
        persons = merged_targets  # Accept all detected classes for testing

        if not persons:
            return None, None
        if self.target_locked and self.target_camera_id is not None:
            for person in persons:
                if (person.get('camera_id') == self.target_camera_id and 
                    person.get('track_id') == self.target_person_id):
                    self.target_lock_confidence = self.calculate_target_confidence(person, self.target_camera_id)
                    return person, self.target_camera_id
            if self.target_person_position is not None:
                best_match = None
                best_camera = None
                best_confidence = 0.0

                for person in persons:
                    distance = np.sqrt((person['world_x'] - self.target_person_position[0])**2 + 
                                     (person['world_y'] - self.target_person_position[1])**2)

                    if distance < 7.0:
                        confidence = self.calculate_target_confidence(person, person.get('camera_id'))
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_match = person
                            best_camera = person.get('camera_id')
                if best_match and best_confidence > 0.5:
                    self.get_logger().info(f'Target re-acquired in {self.camera_orientations[best_camera]["name"]} camera')
                    self.target_person_id = best_match.get('track_id')
                    self.target_camera_id = best_camera
                    self.target_lock_confidence = best_confidence
                    return best_match, best_camera
        best_person = None
        best_camera = None
        best_confidence = 0.0

        for person in persons:
            camera_id = person.get('camera_id')
            confidence = self.calculate_target_confidence(person, camera_id)

            if confidence > best_confidence:
                best_confidence = confidence
                best_person = person
                best_camera = camera_id
        if best_person and best_confidence > self.target_lock_threshold:
            if not self.target_locked:
                self.get_logger().info(f'Target LOCKED: ID {best_person.get("track_id")} in {self.camera_orientations[best_camera]["name"]} camera (confidence: {best_confidence:.2f})')
                self.target_locked = True

            self.target_lock_confidence = best_confidence
            return best_person, best_camera
        if best_person:
            self.target_locked = False
            self.target_lock_confidence = best_confidence
            return best_person, best_camera

        return None, None                        

    def smooth_target_position(self, new_center):
        """Reduced smoothing to be more responsive"""
        if not new_center:
            return None

        self.target_position_buffer.append(new_center)

        if len(self.target_position_buffer) < 2:
            return new_center
        
        # Use simple averaging instead of weighted smoothing
        positions = list(self.target_position_buffer)
        avg_x = sum(pos[0] for pos in positions[-3:]) / min(len(positions), 3)
        avg_y = sum(pos[1] for pos in positions[-3:]) / min(len(positions), 3)

        return [avg_x, avg_y]

    def update_person_following_multi_camera(self, merged_targets):
        """Update person following logic with multi-camera support"""
        current_time = time.time()
        target_person, target_camera = self.select_target_person_multi_camera(merged_targets)

        # General collision avoidance logic
        obstacle_classes = [0, 2, 3, 5, 7, 9, 58]  # person, car, motorcycle, bus, truck, traffic light, potted plant
        obstacle_targets = [t for t in merged_targets if t.get('class', -1) in obstacle_classes]
        repulse_x, repulse_y = 0.0, 0.0
        for obs in obstacle_targets:
            obs_distance = np.sqrt(obs['world_x']**2 + obs['world_y']**2)
            if obs_distance < 3.0:
                direction = np.arctan2(obs['world_y'], obs['world_x'])
                strength = 1.0 / (obs_distance + 0.1)
                repulse_x -= strength * np.cos(direction)
                repulse_y -= strength * np.sin(direction)
        if abs(repulse_x) > 0.01 or abs(repulse_y) > 0.01:
            self.collision_avoidance_cmd = {'x': repulse_x, 'y': repulse_y}
        else:
            self.collision_avoidance_cmd = None

        if target_person:
            # Target found - switch to following
            if not self.takeoff_initiated and self.drone_state == 'IDLE':
                self.initiate_takeoff()

            self.target_person_id = target_person.get('track_id')
            self.target_camera_id = target_camera
            self.last_person_detection_time = current_time
            self.target_person_position = [target_person['world_x'], target_person['world_y']]

            if self.takeoff_complete:
                # Reset hover attributes when target is found
                if hasattr(self, 'hover_search_start_time'):
                    delattr(self, 'hover_search_start_time')
                if hasattr(self, 'hover_stationary_logged'):
                    delattr(self, 'hover_stationary_logged')
                
                self.following_active = True
                self.drone_state = 'FOLLOWING'
                self.target_person_data = target_person

                cam_name = self.camera_orientations[target_camera]['name']
                if self.camera_frame_counts.get(target_camera, 0) % 60 == 0:
                    self.get_logger().info(f'FOLLOWING: Person ID {self.target_person_id} '
                                         f'from {cam_name} camera')
        else:
            # No target found
            time_since_last = current_time - self.last_person_detection_time
            
            if self.following_active and time_since_last > 1.0:  # Lost target
                self.get_logger().warn(f'Target lost - switching to HOVERING')
                self.following_active = False
                self.target_locked = False
                self.drone_state = 'HOVERING'
                
                # Reset hover timer to start fresh 20-second cycle
                if hasattr(self, 'hover_search_start_time'):
                    delattr(self, 'hover_search_start_time')
                if hasattr(self, 'hover_stationary_logged'):
                    delattr(self, 'hover_stationary_logged')
                    
            elif self.takeoff_complete and not self.following_active and self.drone_state != 'HOVERING':
                self.drone_state = 'HOVERING'
                # Reset hover timer for new hover cycle
                if hasattr(self, 'hover_search_start_time'):
                    delattr(self, 'hover_search_start_time')
                if hasattr(self, 'hover_stationary_logged'):
                    delattr(self, 'hover_stationary_logged')

    def initiate_takeoff(self):
        """Initiate drone takeoff sequence using ROS service"""
        if not self.takeoff_initiated:
            self.get_logger().info(f'INITIATING TAKEOFF to {self.takeoff_height}m via ROS service')
            self.drone_state = 'TAKING_OFF'
            self.takeoff_initiated = True
            self.takeoff_start_time = time.time()

            if self.takeoff_client.wait_for_service(timeout_sec=5.0):
                request = Takeoff.Request()
                future = self.takeoff_client.call_async(request)
                future.add_done_callback(self.takeoff_response_callback)
                self.get_logger().info('Takeoff service call initiated')
            else:
                self.get_logger().error('Takeoff service not available')
                self.takeoff_initiated = False
                self.drone_state = 'IDLE'

    def takeoff_response_callback(self, future):
        """Handle takeoff service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Takeoff service successful: {response.message}')
                self.takeoff_complete = True
                self.drone_state = 'HOVERING'
                self.current_altitude = self.takeoff_height
            else:
                self.get_logger().error(f'Takeoff service failed: {response.message}')
                self.takeoff_initiated = False
                self.drone_state = 'IDLE'
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
            self.takeoff_initiated = False
            self.drone_state = 'IDLE'

    def hover_drone(self):
        """Stable hover - all velocities zero"""
        if self.enable_following:
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = 0.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            vel_cmd.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(vel_cmd)

    def slow_forward_search(self):
        """Slow forward movement while rotating to search"""
        if self.enable_following:
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = 0.1
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            vel_cmd.twist.angular.z = 0.3  # Reduced rotation speed
            self.cmd_vel_pub.publish(vel_cmd)
            self.get_logger().info('SLOW FORWARD SEARCH: Moving forward slowly while rotating to find target')

    def pixel_to_world_direction(self, pixel_center, image_center):
        """Convert pixel coordinates to world direction angles"""
        dx = pixel_center[0] - image_center[0]
        dy = pixel_center[1] - image_center[1]

        dx_normalized = dx / (self.image_width / 2.0)
        dy_normalized = dy / (self.image_height / 2.0)

        yaw_angle = dx_normalized * math.radians(self.camera_fov_horizontal / 2.0)
        pitch_angle = -dy_normalized * math.radians(self.camera_fov_vertical / 2.0)

        return yaw_angle, pitch_angle

    def estimate_person_distance(self, bbox):
        bbox_height = bbox[3]

        if bbox_height > 0:
            raw_distance = (1.7 * self.image_height) / (bbox_height * math.tan(math.radians(self.camera_fov_vertical/2)) * 2)
            raw_distance = max(1.5, min(raw_distance, 20.0))

            if not hasattr(self, 'distance_history'):
                self.distance_history = deque(maxlen=5)

            self.distance_history.append(raw_distance)
            if len(self.distance_history) >= 3:
                smoothed_distance = np.median(list(self.distance_history))
            else:
                smoothed_distance = raw_distance

            return smoothed_distance
        return self.follow_distance


    def update_target_data_async(self, moving_targets, camera_id, header):
        """Thread safe target data update"""
        current_time = time.time()

        # TEMPORARY: Disabled class filter to test if YOLO is detecting anything
        # persons = [target for target in moving_targets if target.get('class', -1) == 0]
        persons = moving_targets  # Accept all detected classes for testing
        if not persons:
            return
        best_person = None
        best_confidence = 0.0

        for person in persons:
            confidence = self.calculate_target_confidence(person, camera_id)
            if camera_id == self.primary_camera:
                confidence *= 1.2

            if confidence > best_confidence:
                best_confidence = confidence
                best_person = person

        with self.target_person_data_lock:
            data_age = current_time - self.target_data_timestamp

            if (best_person and 
                (self.target_person_data is None or 
                 best_confidence > getattr(self, 'target_lock_confidence', 0.0) or
                 data_age > 0.5)):


                self.target_person_data = best_person.copy()
                self.target_person_data['camera_id'] = camera_id
                self.target_data_timestamp = current_time
                self.target_lock_confidence = best_confidence
                if not self.takeoff_initiated and self.drone_state == 'IDLE':
                    self.initiate_takeoff()

                if self.takeoff_complete:
                    self.following_active = True
                    self.drone_state = 'FOLLOWING'
                    self.last_person_detection_time = current_time

    def publish_velocity_command(self, vel_cmd):
        # Only publish if takeoff is complete and drone is in a valid state
        if self.takeoff_complete and self.drone_state in ['FOLLOWING', 'SEARCHING', 'HOVERING']:
            self.cmd_vel_pub.publish(vel_cmd)

    def control_loop(self):
        """Simplified 4-state control loop: IDLE, TAKING_OFF, HOVERING, FOLLOWING"""
        dt = 0.067  # ~15Hz

        # State 1: TAKING_OFF - wait for completion
        if self.drone_state == 'TAKING_OFF' and not self.takeoff_complete:
            return

        # State 2: HOVERING - move in last direction then stay stationary
        if self.drone_state == 'HOVERING':
            self.hover_search_behavior()
            return

        # State 3: FOLLOWING - quick acceleration towards target
        if not self.following_active or not hasattr(self, 'target_person_data'):
            if self.takeoff_complete:
                # If no target and airborne, switch to hovering
                if self.drone_state != 'HOVERING':
                    self.drone_state = 'HOVERING'
                    # Reset hover timer when switching states
                    if hasattr(self, 'hover_search_start_time'):
                        delattr(self, 'hover_search_start_time')
                    if hasattr(self, 'hover_stationary_logged'):
                        delattr(self, 'hover_stationary_logged')
                self.hover_search_behavior()
            return

        try:
            person = self.target_person_data
            target_camera = person.get('camera_id', 0)

            bbox = person.get('bbox', [0, 0, 100, 100])
            center = person.get('center', [bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2])
            
            smoothed_center = self.smooth_target_position(center)
            if smoothed_center:
                center = smoothed_center

            image_center = [self.image_width / 2, self.image_height / 2]
            yaw_angle, pitch_angle = self.pixel_to_world_direction(center, image_center)
            estimated_distance = self.estimate_person_distance(bbox)

            # Calculate camera offset for multi-camera support
            cam_yaw_offset = math.radians(self.camera_orientations[target_camera]['yaw'])
            world_yaw_angle = yaw_angle + cam_yaw_offset
            
            # YAW Control - turn to face target
            yaw_deadband = math.radians(10)  # Smaller deadband for more responsive turning
            if abs(world_yaw_angle) > yaw_deadband:
                yaw_cmd = np.sign(world_yaw_angle) * 1.2  # Faster turning
            else:
                yaw_cmd = 0.0

            # Multi-directional movement based on which camera sees target
            distance_error = estimated_distance - self.follow_distance
            
            # Calculate base movement speed
            if distance_error > 2.0:
                move_speed = 2.5
            elif distance_error > 1.0:
                move_speed = 2.0
            elif distance_error > 0.5:
                move_speed = 1.5
            elif distance_error > 0.2:
                move_speed = 0.8
            elif distance_error < -0.5:
                move_speed = -1.0
            else:
                move_speed = 0.2

            if target_camera == 0:  # Front camera
                forward_cmd = move_speed
                side_cmd = 0.0
            elif target_camera == 1:  # Right camera  
                forward_cmd = 0.0
                side_cmd = -move_speed  # Move right (negative Y)
            elif target_camera == 2:  # Back camera
                forward_cmd = -move_speed  # Move backward
                side_cmd = 0.0
            elif target_camera == 3:  # Left camera
                forward_cmd = 0.0  
                side_cmd = move_speed   # Move left (positive Y)
            else:
                forward_cmd = move_speed  # Default to forward
                side_cmd = 0.0

            # Create and publish velocity command
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = forward_cmd
            vel_cmd.twist.linear.y = side_cmd 
            vel_cmd.twist.linear.z = 0.0  
            vel_cmd.twist.angular.z = yaw_cmd
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0

            # Store last movement direction for hover behavior
            self.last_movement_direction = max(abs(forward_cmd), abs(side_cmd))

            self.publish_velocity_command(vel_cmd)

            # Enhanced logging
            cam_name = self.camera_orientations[target_camera]['name']
            if sum(self.camera_frame_counts.values()) % 60 == 0:
                self.get_logger().info(f'FOLLOWING from {cam_name}: '
                                     f'dist={estimated_distance:.1f}m, '
                                     f'cmd=[x:{forward_cmd:.2f}, y:{side_cmd:.2f}, yaw:{yaw_cmd:.2f}]')

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            # On error, switch to hovering
            self.drone_state = 'HOVERING'
            if hasattr(self, 'hover_search_start_time'):
                delattr(self, 'hover_search_start_time')

    def hover_search_behavior(self):
        """Simplified hover behavior: move in last seen direction for 20 seconds, then stay stationary"""
        
        # Initialize hover search timer if not exists
        if not hasattr(self, 'hover_search_start_time'):
            self.hover_search_start_time = time.time()
            
            # Store last movement direction when entering hover
            if hasattr(self, 'last_movement_direction'):
                # Much slower - cap at 0.1 m/s for very gentle movement
                self.hover_forward_speed = min(0.1, max(0.05, abs(self.last_movement_direction) * 0.1))
            else:
                self.hover_forward_speed = 0.08
                
            self.get_logger().info(f'HOVER: Starting movement in last direction at speed {self.hover_forward_speed:.2f}')

        current_time = time.time()
        search_duration = current_time - self.hover_search_start_time

        vel_cmd = VelCmd()
        
        if search_duration < 20.0:
            # First 20 seconds: Move in last seen target direction
            vel_cmd.twist.linear.x = self.hover_forward_speed
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 0.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            
            # Log countdown for last 3 seconds only
            if search_duration > 17.0:
                remaining = 20.0 - search_duration
                if int(remaining) <= 3:  # Only log final 3 seconds
                    self.get_logger().info(f'HOVER: Switching to stationary in {remaining:.1f} seconds')
            
        else:
            # After 20 seconds: Stay stationary (idle)
            vel_cmd.twist.linear.x = 0.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 0.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            
            # Log once when switching to stationary
            if not hasattr(self, 'hover_stationary_logged'):
                self.hover_stationary_logged = True
                self.get_logger().info('HOVER: Now stationary - waiting for target')

        if self.enable_following:
            self.cmd_vel_pub.publish(vel_cmd)

    def search_for_lost_target(self):
        """Deprecated - redirects to hover_search_behavior"""
        # All search functionality consolidated into hover_search_behavior
        self.hover_search_behavior()

    def initialize_detection_systems(self):
        """Initialize detection client (subscribes to microservice)"""
        self.get_logger().info("[INFO] Microservices Architecture - Detection client mode")
        self.get_logger().info("[INFO] Consuming detections from YOLOv10 detection service")

        # Microservice mode - subscribe to detection service
        self.use_yolo_deepsort = True  # Service provides tracked detections
        self.use_yolo_only = False
        self.detection_subscribers = {}

        # Subscribe to detection topics from microservice (matching YOLOv10 service output)
        # YOLOv10 publishes to /detections/Camera_N_Scene based on camera topic extraction
        camera_ids = ['Camera_0_Scene', 'Camera_1_Scene', 'Camera_2_Scene', 'Camera_3_Scene']
        for i, camera_id in enumerate(camera_ids):
            detection_topic = f'/detections/{camera_id}'
            sub = self.create_subscription(
                ObjectDetectionArray,
                detection_topic,
                lambda msg, cid=i: self.detection_callback(msg, cid),  # Use numeric ID internally (0-3)
                10
            )
            self.detection_subscribers[camera_id] = sub
            self.get_logger().info(f"  Subscribed to: {detection_topic} (camera {i})")

        # Initialize detection storage
        self.latest_detections = {}  # camera_id -> ObjectDetectionArray
        self.detection_lock = threading.Lock()

        self.get_logger().info("[SUCCESS] Detection client initialized")

    def warmup_model(self):
        """No longer needed - model runs in separate microservice"""
        pass

    def setup_opencv_detection(self):
        """Setup OpenCV motion detection fallback"""
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True, varThreshold=50)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.get_logger().info("[DEBUG] OpenCV motion detection initialized")

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
        """Compute color for class labels"""
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
        """Draw fancy border"""
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
        """Draw simple rectangle box"""
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        color = color or [np.random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        # Draw rectangle
        cv2.rectangle(img, c1, c2, color, tl)
        # Draw label
        if label:
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            cv2.rectangle(img, (c1[0], c1[1] - t_size[1] - 3), (c1[0] + t_size[0], c1[1]), color, -1)
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, (255, 255, 255), thickness=tf, lineType=cv2.LINE_AA)

    def draw_boxes(self, img, bbox, object_id, identities=None, offset=(0, 0), camera_id=0):
        """Draw tracking boxes with trails - camera-specific version"""
        height, width, _ = img.shape

        # Use camera-specific data deque
        data_deque = self.camera_data_deques[camera_id]

        # Remove tracked point from buffer if object is lost
        for key in list(data_deque):
            if identities is None or key not in identities:
                data_deque.pop(key)

        moving_objects = []

        for i, box in enumerate(bbox):
            x1, y1, x2, y2 = [int(i) for i in box]
            x1 += offset[0]
            x2 += offset[0]
            y1 += offset[1]
            y2 += offset[1]

            center = (int((x2 + x1) / 2), int((y2 + y1) / 2))
            id = int(identities[i]) if identities is not None else 0

            if id not in data_deque:  
                data_deque[id] = deque(maxlen=self.trail_length)

            color = self.compute_color_for_labels(object_id[i])
            obj_name = self.names[object_id[i]]
            label = '{}{:d}'.format("", id) + ":" + '%s' % (obj_name)

            data_deque[id].appendleft(center)

            # For person class, always consider as potential target
            is_moving = self.is_track_moving_camera(id, camera_id)
            is_person = object_id[i] == 0

            if is_moving or is_person:
                velocity = self.calculate_track_velocity_camera(id, camera_id) if is_moving else 0.0
                world_pos = self.pixel_to_world(center)

                moving_objects.append({
                    'track_id': id,
                    'world_x': world_pos[0],
                    'world_y': world_pos[1],
                    'world_z': 0.0,
                    'confidence': 0.8,
                    'velocity': velocity,
                    'class': object_id[i],
                    'bbox': [x1, y1, x2-x1, y2-y1],
                    'center': center
                })

            self.UI_box(box, img, label=label, color=color, line_thickness=2)

            # Draw trail
            for j in range(1, len(data_deque[id])):
                if data_deque[id][j - 1] is None or data_deque[id][j] is None:
                    continue
                thickness = int(np.sqrt(self.trail_length / float(j + j)) * 1.5)
                cv2.line(img, data_deque[id][j - 1], data_deque[id][j], color, thickness)

        return img, moving_objects

    def is_track_moving_camera(self, track_id, camera_id):
        """Check if tracked object is moving for specific camera"""
        data_deque = self.camera_data_deques[camera_id]

        if track_id not in data_deque:
            return True

        positions = list(data_deque[track_id])
        if len(positions) < 3:
            return True

        recent_positions = positions[-5:]
        if len(recent_positions) < 2:
            return True

        start_pos = recent_positions[0]
        end_pos = recent_positions[-1]
        displacement = np.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)

        return displacement > (self.motion_threshold * 0.5)

    def calculate_track_velocity_camera(self, track_id, camera_id):
        """Calculate velocity for camera-specific track"""
        data_deque = self.camera_data_deques[camera_id]

        if track_id not in data_deque:
            return 0.0

        positions = list(data_deque[track_id])

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

        velocity_pixels_per_frame = total_distance / len(recent_positions)
        velocity_pixels_per_second = velocity_pixels_per_frame * 20
        return velocity_pixels_per_second * 0.01

    def detection_callback(self, msg, camera_id):
        """Receive detections from YOLOv10 microservice"""
        with self.detection_lock:
            self.latest_detections[camera_id] = msg

    def detect_and_track_yolo_deepsort(self, image, camera_id):
        """Process detections from microservice (no longer runs YOLOv10 directly)"""
        im0 = image.copy()
        moving_targets = []

        # Get latest detections from microservice
        with self.detection_lock:
            detections_msg = self.latest_detections.get(camera_id)

        # DIAGNOSTIC: Log raw detections before any filtering
        if detections_msg and len(detections_msg.detections) > 0:
            detected_classes = [f"{det.class_name}(id:{det.class_id})" for det in detections_msg.detections[:5]]  # First 5
            self.get_logger().info(f"[RAW YOLO] Camera {camera_id}: {len(detections_msg.detections)} detections: {detected_classes}")

        if detections_msg and len(detections_msg.detections) > 0:
            # Convert ObjectDetectionArray to moving_targets format
            for det in detections_msg.detections:
                # Build bounding box array for draw_boxes compatibility
                x1 = det.x
                y1 = det.y
                x2 = det.x + det.width
                y2 = det.y + det.height

                # Create target entry
                target = {
                    'bbox': [x1, y1, x2, y2],
                    'track_id': det.track_id,
                    'class_id': det.class_id,
                    'class_name': det.class_name,
                    'confidence': det.confidence
                }
                moving_targets.append(target)

            # Draw boxes on image (reuse existing visualization logic)
            if len(moving_targets) > 0:
                bbox_xyxy = np.array([[t['bbox'][0], t['bbox'][1], t['bbox'][2], t['bbox'][3]] for t in moving_targets])
                identities = np.array([t['track_id'] for t in moving_targets])
                object_ids = np.array([t['class_id'] for t in moving_targets])

                im0, moving_targets = self.draw_boxes(im0, bbox_xyxy, object_ids, identities, camera_id=camera_id)

        return moving_targets, im0

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        """Resize and pad image while meeting stride-multiple constraints"""
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)

        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)

    def pixel_to_world(self, pixel_center):
        """Convert pixel coordinates to world coordinates"""
        center_x, center_y = pixel_center
        world_x = (center_x - 320) * 0.01
        world_y = (center_y - 240) * 0.01
        return [world_x, world_y]

    def image_callback(self, msg, camera_id):
        """Enhanced image processing with improved resolution handling"""
        try:
            current_time = time.time()
            with self.camera_locks[camera_id]:
                self.camera_frame_counts[camera_id] += 1
                # Track timestamp for FPS calculation
                self.camera_fps_timestamps[camera_id].append(current_time)
                # Keep only timestamps within the window
                cutoff_time = current_time - self.camera_fps_window
                self.camera_fps_timestamps[camera_id] = [
                    t for t in self.camera_fps_timestamps[camera_id] if t > cutoff_time
                ]

            # Log incoming image details for debugging (first frame only)
            if self.camera_frame_counts[camera_id] == 1:
                self.get_logger().info(f'Camera {camera_id} input: {msg.width}x{msg.height}, '
                                     f'encoding: {msg.encoding}, step: {msg.step}, '
                                     f'data_size: {len(msg.data)}')

            # Convert ROS image to OpenCV format with proper error handling
            try:
                if msg.encoding == 'rgb8':
                    # Verify buffer size before conversion
                    expected_size = msg.height * msg.width * 3  # 3 channels for RGB
                    if len(msg.data) < expected_size:
                        self.get_logger().error(f'Camera {camera_id}: Buffer too small. '
                                              f'Expected: {expected_size}, Got: {len(msg.data)}')
                        return

                    cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                else:
                    # For bgr8 encoding
                    expected_size = msg.height * msg.width * 3  # 3 channels for BGR
                    if len(msg.data) < expected_size:
                        self.get_logger().error(f'Camera {camera_id}: Buffer too small. '
                                              f'Expected: {expected_size}, Got: {len(msg.data)}')
                        return

                    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            except Exception as e:
                self.get_logger().error(f'Camera {camera_id}: Image conversion failed: {e}')
                return

            # Log actual converted image size (first frame only)
            if self.camera_frame_counts[camera_id] == 1:
                self.get_logger().info(f'Camera {camera_id} converted: {cv_image.shape[1]}x{cv_image.shape[0]} '
                                     f'channels: {cv_image.shape[2]}')

            # Resize to target resolution with high-quality interpolation
            target_size = (self.image_width, self.image_height)
            if cv_image.shape[1] != self.image_width or cv_image.shape[0] != self.image_height:
                if self.enable_image_resize:
                    # Use high-quality interpolation for better results
                    cv_image = cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LANCZOS4)

                    # Apply sharpening filter for better visual quality
                    if self.image_quality > 80:
                        kernel = np.array([[-1,-1,-1],
                                         [-1, 9,-1],
                                         [-1,-1,-1]])
                        cv_image = cv2.filter2D(cv_image, -1, kernel * 0.5)

                        cv_image = np.clip(cv_image, 0, 255).astype(np.uint8)

                    if self.camera_frame_counts[camera_id] == 1:
                        self.get_logger().info(f'Camera {camera_id} resized to: {cv_image.shape[1]}x{cv_image.shape[0]}')
                else:
                    # If resizing is disabled, use original image but log warning
                    self.get_logger().warn_once(f'Camera {camera_id}: Received {cv_image.shape[1]}x{cv_image.shape[0]} '
                                              f'but expected {self.image_width}x{self.image_height}')

            self.camera_last_images[camera_id] = cv_image

            # Process with detection system
            if self.use_yolo_deepsort:
                try:
                    moving_targets, vis_image = self.detect_and_track_yolo_deepsort(cv_image, camera_id)
                    self.camera_detection_counts[camera_id] = len(self.camera_data_deques[camera_id])
                except Exception as e:
                    self.get_logger().error(f'Camera {camera_id}: YOLO detection failed: {e}')
                    moving_targets, vis_image = [], cv_image
                    self.camera_detection_counts[camera_id] = 0
            else:
                moving_targets, vis_image = [], cv_image
                self.camera_detection_counts[camera_id] = 0

            self.camera_moving_counts[camera_id] = len(moving_targets)
            self.all_camera_targets[camera_id] = moving_targets

            # Update person following (run for all cameras)
            if self.enable_following:
                merged_targets = self.merge_camera_detections()
                self.update_person_following_multi_camera(merged_targets)
                for target in merged_targets:
                    self.publish_target_detection(target, msg.header)

            # Publish high-quality visualization
            if self.enable_vis and vis_image is not None and camera_id in self.vis_pubs:
                self.publish_camera_visualization(vis_image, camera_id, msg.header)

        except Exception as e:
            self.get_logger().error(f'Camera {camera_id} processing error: {e}')
            import traceback
            self.get_logger().error(f'Camera {camera_id} traceback: {traceback.format_exc()}')

    def publish_target_detection(self, target, header):
        """Publish individual target detection"""
        detection_msg = TargetDetection()
        detection_msg.header = header
        detection_msg.vehicle_name = self.vehicle_name
        detection_msg.target_x = target['world_x']
        detection_msg.target_y = target['world_y']
        detection_msg.target_z = target['world_z']
        detection_msg.confidence = target['confidence']

        self.detection_pub.publish(detection_msg)

    def publish_camera_visualization(self, vis_image, camera_id, header):
        """Publish high-quality camera-specific visualization"""
        try:
            # Add camera info to visualization
            cam_name = self.camera_orientations[camera_id]['name']

            # Ensure image is at target resolution
            if vis_image.shape[1] != self.image_width or vis_image.shape[0] != self.image_height:
                vis_image = cv2.resize(vis_image, (self.image_width, self.image_height), 
                                     interpolation=cv2.INTER_LANCZOS4)

            # Scale text and line thickness based on resolution
            font_scale = max(0.8, self.image_width / 1280.0)
            thickness = max(2, int(self.image_width / 640))

            # Camera status overlay with better visibility
            state_color = {
                'IDLE': (128, 128, 128),
                'TAKING_OFF': (0, 255, 255),
                'HOVERING': (255, 255, 0),
                'FOLLOWING': (0, 255, 0)
            }.get(self.drone_state, (255, 255, 255))

            cv2.putText(vis_image, f"CAMERA: {cam_name.upper()}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
            cv2.putText(vis_image, f"STATE: {self.drone_state}", (10, 75),
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale, state_color, thickness)

            # FPS overlay (top-right corner)
            timestamps = self.camera_fps_timestamps.get(camera_id, [])
            if len(timestamps) > 1:
                time_span = timestamps[-1] - timestamps[0] if len(timestamps) > 0 else 1.0
                fps = len(timestamps) / time_span if time_span > 0 else 0.0
            else:
                fps = 0.0

            fps_text = f"FPS: {fps:.1f}"
            # Calculate text size for right alignment
            (text_width, text_height), baseline = cv2.getTextSize(
                fps_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )
            fps_x = self.image_width - text_width - 10
            cv2.putText(vis_image, fps_text, (fps_x, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 255), thickness)

            # Target following status
            if self.following_active and self.target_camera_id == camera_id:
                # cv2.rectangle(vis_image, (5, 90), (450, 120), text_bg_color, -1)
                cv2.putText(vis_image, f"FOLLOWING TARGET ID: {self.target_person_id}", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)

                # Add target location information
                if hasattr(self, 'target_person_data') and self.target_person_data:
                    try:
                        # Get target center in image coordinates (pixels)
                        target_center = self.target_person_data.get('center', [0, 0])
                        target_bbox = self.target_person_data.get('bbox', [0, 0, 0, 0])

                        # Get estimated distance
                        estimated_distance = getattr(self, 'last_estimated_distance', 0.0)

                        # World position if available
                        world_pos = getattr(self, 'target_person_position', [0.0, 0.0])

                        # Display target pixel location
                        cv2.putText(vis_image, f"TARGET POS: ({int(target_center[0])}, {int(target_center[1])})", 
                                   (10, 140), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 255, 255), thickness)

                        # Display estimated distance
                        cv2.putText(vis_image, f"DISTANCE: {estimated_distance:.2f}m", 
                                   (10, 165), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 255, 255), thickness)

                        # Display world coordinates
                        cv2.putText(vis_image, f"WORLD: ({world_pos[0]:.2f}, {world_pos[1]:.2f})", 
                                   (10, 190), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 255, 255), thickness)

                        # Display bounding box size
                        bbox_width = int(target_bbox[2]) if len(target_bbox) > 2 else 0
                        bbox_height = int(target_bbox[3]) if len(target_bbox) > 3 else 0
                        cv2.putText(vis_image, f"BBOX: {bbox_width}x{bbox_height}", 
                                   (10, 215), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 255, 255), thickness)

                        # Draw crosshair at target position
                        if target_center[0] > 0 and target_center[1] > 0:
                            target_x, target_y = int(target_center[0]), int(target_center[1])
                            # Draw target crosshair
                            cv2.line(vis_image, (target_x - 20, target_y), (target_x + 20, target_y), (0, 0, 255), 3)
                            cv2.line(vis_image, (target_x, target_y - 20), (target_x, target_y + 20), (0, 0, 255), 3)
                            cv2.circle(vis_image, (target_x, target_y), 8, (0, 0, 255), 2)

                            # Label the target
                            cv2.putText(vis_image, "TARGET", (target_x + 25, target_y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.6, (0, 0, 255), 2)

                    except Exception as e:
                        cv2.putText(vis_image, f"TARGET DATA ERROR", (10, 140), 
                                   cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (0, 0, 255), thickness)
            elif self.enable_following:
                status = "PRIMARY CAMERA" if camera_id == self.primary_camera else "SECONDARY VIEW"
                # cv2.rectangle(vis_image, (5, 160), (300, 190), text_bg_color, -1)
                cv2.putText(vis_image, status, (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 
                           font_scale, (255, 255, 0), thickness)

            # Frame count and resolution info
            frame_count = self.camera_frame_counts.get(camera_id, 0)
            info_text = f"FRAME: {frame_count} | RES: {self.image_width}x{self.image_height}"
            # cv2.rectangle(vis_image, (5, 200), (500, 230), text_bg_color, -1)
            cv2.putText(vis_image, info_text, (10, 280), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, (255, 255, 255), thickness)

            # Draw enhanced crosshair at image center for reference
            center_x, center_y = self.image_width // 2, self.image_height // 2
            crosshair_size = max(30, self.image_width // 40)
            cv2.line(vis_image, (center_x - crosshair_size, center_y), 
                    (center_x + crosshair_size, center_y), (0, 255, 0), 3)
            cv2.line(vis_image, (center_x, center_y - crosshair_size), 
                    (center_x, center_y + crosshair_size), (0, 255, 0), 3)
            cv2.circle(vis_image, (center_x, center_y), 5, (0, 255, 0), -1)

            # Convert back to ROS message with proper encoding
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
            vis_msg.header = header
            self.vis_pubs[camera_id].publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f'Camera {camera_id} visualization error: {e}')

    # Removed unused target prediction method

    def calculate_target_confidence(self, target, camera_id):
        if not target:
            return 0.0

        confidence = target.get('confidence', 0.5)
        bbox = target.get('bbox', [0, 0, 1, 1])


        # Size factor - larger targets are more reliable
        bbox_area = bbox[2] * bbox[3]
        size_factor = min(bbox_area / (self.image_width * self.image_height * 0.1), 1.0)

        center = target.get('center', [self.image_width / 2, self.image_height / 2])
        center_distance = np.sqrt((center[0] - self.image_width / 2) ** 2 + (center[1] - self.image_height / 2) ** 2)
        max_distance = np.sqrt((self.image_width / 2) ** 2 + (self.image_height / 2) ** 2)
        center_factor = 1.0 - (center_distance / max_distance)

        consistency_factor = 0.8 if target.get('track_id') in self.camera_data_deques.get(camera_id, {}) else 0.5

        camera_factor = 1.2 if camera_id == self.primary_camera else 1.0

        total_confidence = confidence * size_factor * center_factor * consistency_factor * camera_factor
        return min(total_confidence, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraMotionDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Removed auto-landing and zero velocity commands to prevent unwanted landing on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
