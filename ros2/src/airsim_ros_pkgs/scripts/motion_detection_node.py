#!/usr/bin/env python3
"""
Multi-Camera Motion Detection Node for AirSim ROS2 Integration with YOLOv7 + DeepSORT Tracking and Person Following
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from airsim_interfaces.msg import TargetDetection, VelCmd
from airsim_interfaces.srv import Takeoff, Land
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.serialization
import time
import math
import traceback
from collections import deque
from pathlib import Path
import sys
import threading

# YOLOv7 + DeepSORT Integration Setup
DETECTION_PATH = Path(__file__).parent / 'YOLOv7-DeepSORT-Object-Tracking'
sys.path.insert(0, str(DETECTION_PATH))

try:
    from models.experimental import attempt_load
    from utils.datasets import LoadStreams, LoadImages
    from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
    from utils.torch_utils import select_device, time_synchronized
    from deep_sort_pytorch.utils.parser import get_config
    from deep_sort_pytorch.deep_sort import DeepSort
    YOLO_DEEPSORT_AVAILABLE = True
    print("YOLOv7+DeepSORT imports successful")
except ImportError as e:
    YOLO_DEEPSORT_AVAILABLE = False
    print(f"YOLOv7+DeepSORT imports failed: {e}")

class MultiCameraMotionDetectionNode(Node):
    def __init__(self):
        super().__init__('multi_camera_motion_detection_node')

        # ROS 2 Parameters Declaration and Retrieval
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('confidence_threshold', 0.2)
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

        self.num_cameras = 4
        self.primary_camera = 0
        self.camera_topics = [
            f'/{self.vehicle_name.lower()}/camera0/image',
            f'/{self.vehicle_name.lower()}/camera1/image', 
            f'/{self.vehicle_name.lower()}/camera2/image',
            f'/{self.vehicle_name.lower()}/camera3/image'
        ]

        self.bridge = CvBridge()

        self.camera_frame_counts = {}
        self.camera_last_images = {}
        self.camera_detection_counts = {}
        self.camera_moving_counts = {}
        self.camera_data_deques = {}
        self.all_camera_targets = {}
        self.camera_locks = {}
        self.target_person_data_lock = threading.Lock()
        self.target_data_timestamp = 0.0
        self.use_yolo_deepsort = False
        self.use_yolo_only = False
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
        self.target_lock_threshold = 0.7
        self.target_history = deque(maxlen=10)
        self.lost_target_search_pattern = 0

        self.velocity_smoother = {
            'x': deque(maxlen=8),
            'y': deque(maxlen=8),
            'z': deque(maxlen=8),
            'yaw': deque(maxlen=8)
        }

        self.target_position_buffer = deque(maxlen=5)
        self.target_smoothing_factor = 0.8
        self.max_acceleration = {
            'x': 1.5,
            'y': 1.5,
            'z': 1.0,
            'yaw': 2.0
        }
        self.previous_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

        self.last_target_center = None
        self.target_prediction = None

        self.drone_state = 'IDLE'
        self.takeoff_initiated = False
        self.takeoff_complete = False
        self.current_altitude = 0.0
        self.takeoff_start_time = None 
        self.pid_x = {'kp': 0.6, 'ki': 0.008, 'kd': 0.03, 'prev_error': 0.0, 'integral': 0.0}
        self.pid_y = {'kp': 0.7, 'ki': 0.005, 'kd': 0.02, 'prev_error': 0.0, 'integral': 0.0}
        self.pid_z = {'kp': 0.35, 'ki': 0.003, 'kd': 0.01, 'prev_error': 0.0, 'integral': 0.0}
        self.pid_yaw = {'kp': 0.8, 'ki': 0.008, 'kd': 0.04, 'prev_error': 0.0, 'integral': 0.0}
        self.deadband_x = 0.3
        self.deadband_y = 0.3
        self.deadband_z = 0.3
        self.deadband_yaw = 0.15
    self.last_movement_direction = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    self.search_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.momentum_decay_factor = 0.95
        self.min_momentum_threshold = 0.05
        self.search_momentum_active = False
        self.momentum_search_duration = 10.0
        self.momentum_search_start_time = 0.0
        self.target_person_data = None
        self.control_loop_frequency = 15.0
        self.image_processing_frequency = {
            0: 15.0,
            1: 10.0,
            2: 10.0,
            3: 10.0
        }
        self.camera_frame_skip = {i: 0 for i in range(self.num_cameras)}
        self.initialize_detection_systems()
        self.background_subtractor = None
        self.kernel = None
        self.previous_detections = {}
        self.next_track_id = 1
        self.setup_multi_camera_interfaces()
        if self.enable_following:
            self.cmd_vel_pub = self.create_publisher(
                VelCmd, f'{self.vehicle_name.lower()}/vel_cmd_body_frame', 10)
            self.takeoff_client = self.create_client(Takeoff, f'{self.vehicle_name.lower()}/takeoff')
            self.land_client = self.create_client(Land, f'{self.vehicle_name.lower()}/land')

        self.debug_timer = self.create_timer(5.0, self.debug_status)
        if self.enable_following:
            self.control_timer = self.create_timer(0.067, self.control_loop)
        self.get_logger().info(f' Multi-Camera Motion Detection node initialized for {self.vehicle_name}')
        self.get_logger().info(f' Using {self.num_cameras} cameras: {", ".join(self.camera_topics)}')
        self.get_logger().info(f' Primary camera: {self.primary_camera} ({self.camera_orientations[self.primary_camera]["name"]})')
        self.get_logger().info(f' Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f' Motion threshold: {self.motion_threshold} pixels')
        self.get_logger().info(f" Person following: {'enabled' if self.enable_following else 'disabled'}")
        self.get_logger().info(f' YOLOv7+DeepSORT: {"enabled" if self.use_yolo_deepsort else "disabled"}')

    def setup_multi_camera_interfaces(self):
        """Setup ROS subscriptions and publishers for all 4 cameras"""
        self.image_subs = {}
        self.vis_pubs = {}

        # Initialize data structures and create interfaces for each camera
        for cam_id in range(self.num_cameras):
            # Initialize per-camera tracking data structures
            self.camera_frame_counts[cam_id] = 0
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
            frames = self.camera_frame_counts.get(cam_id, 0)
            detections = self.camera_detection_counts.get(cam_id, 0)
            moving = self.camera_moving_counts.get(cam_id, 0)
            cam_name = self.camera_orientations[cam_id]['name']
            camera_status.append(f"{cam_name}:[F:{frames} D:{detections} M:{moving}]")

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
        persons = [target for target in merged_targets if target.get('class', -1) == 0]

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

    def smooth_velocity_command(self, cmd_type, new_value):
        """Enhanced velocity smoothing with exponential averaging and acceleration limiting"""
        self.velocity_smoother[cmd_type].append(new_value)
        if len(self.velocity_smoother[cmd_type]) > 1:
            buffer_values = list(self.velocity_smoother[cmd_type])
            smoothing_factor = 0.9
            weights = np.array([smoothing_factor ** (len(buffer_values) - 1 - i) 
                               for i in range(len(buffer_values))])
            weights = weights / weights.sum()
            smoothed = np.average(buffer_values, weights=weights)
        else:
            smoothed = new_value

        dt = 0.067
        conservative_max_accel = self.max_acceleration[cmd_type] * 0.6
        max_delta = conservative_max_accel * dt
        prev_vel = self.previous_velocity[cmd_type]
        if abs(smoothed - prev_vel) > max_delta:
            if smoothed > prev_vel:
                smoothed = prev_vel + max_delta
            else:
                smoothed = prev_vel - max_delta

        self.previous_velocity[cmd_type] = smoothed
        return smoothed


    def apply_deadband(self, value, deadband):
        """Apply deadband to reduce jitter around zero"""
        if abs(value) < deadband:
            return 0.0
        return value

    def smooth_target_position(self, new_center):
        """Smooth target position to reduce tracking jitter"""
        if not new_center:
            return None

        self.target_position_buffer.append(new_center)

        if len(self.target_position_buffer) < 2:
            return new_center
        positions = list(self.target_position_buffer)
        weights = [self.target_smoothing_factor ** (len(positions) - 1 - i) 
                  for i in range(len(positions))]
        total_weight = sum(weights)

        smoothed_x = sum(pos[0] * w for pos, w in zip(positions, weights)) / total_weight
        smoothed_y = sum(pos[1] * w for pos, w in zip(positions, weights)) / total_weight

        return [smoothed_x, smoothed_y]

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
            if not self.takeoff_initiated and self.drone_state == 'IDLE':
                self.initiate_takeoff()

            old_id = self.target_person_id
            self.target_person_id = target_person.get('track_id')
            self.target_camera_id = target_camera
            self.last_person_detection_time = current_time

            # Store position for tracking continuity between camera switches
            self.target_person_position = [target_person['world_x'], target_person['world_y']]
            current_center = target_person.get('center')

            if current_center:
                self.target_prediction = self.predict_target_position(current_center)
                self.last_target_center = current_center

            # Add to history for trend analysis and position prediction
            self.target_history.append({
                'time': current_time,
                'position': self.target_person_position,
                'center': current_center,
                'confidence': target_person.get('confidence', 0.5),
                'camera_id': target_camera
            })

            if self.takeoff_complete:
                self.following_active = True
                self.drone_state = 'FOLLOWING'
                self.target_person_data = target_person

                # Reset search patterns when target is reacquired
                self.lost_target_search_pattern = 0
                self.search_momentum_active = False

                if old_id != self.target_person_id and self.camera_frame_counts.get(target_camera, 0) % 30 == 0:
                    cam_name = self.camera_orientations[target_camera]['name']
                    self.get_logger().info(f'Following person ID: {self.target_person_id} '
                                         f'from {cam_name} camera, confidence: {target_person.get("confidence", 0.5):.2f}')
        else:
            self.collision_avoidance_cmd = None
            # No person detected - implement HOVER and SEARCH behavior
            time_since_last = current_time - self.last_person_detection_time
            if self.following_active and time_since_last > 1.5:  # Reduced timeout for faster response
                self.get_logger().warn(f'Lost person ID: {self.target_person_id} from all cameras - hovering and searching')
                self.following_active = False
                self.target_locked = False
                self.target_lock_confidence = 0.0
                self.drone_state = 'SEARCHING'
                # This will trigger hover search behavior in control loop
            elif self.following_active and time_since_last > 0.5:  # Quick response to temporary loss
                self.drone_state = 'SEARCHING'
                # This will trigger hover search behavior
            elif self.takeoff_complete and not self.following_active:
                self.drone_state = 'HOVERING'
                self.hover_drone()

    def initiate_search_pattern(self):
        """Initiate systematic search when target is completely lost"""
        self.lost_target_search_pattern = (self.lost_target_search_pattern + 1) % 8
        vel_cmd = VelCmd()
        if self.lost_target_search_pattern % 2 == 0:
            vel_cmd.twist.linear.x = 4.0
        else:
            vel_cmd.twist.linear.x = 2.0

        vel_cmd.twist.linear.y = 0.0
        vel_cmd.twist.linear.z = 0.0
        vel_cmd.twist.angular.z = 3.0 if self.lost_target_search_pattern < 4 else -3.0
        vel_cmd.twist.angular.x = 0.0
        vel_cmd.twist.angular.y = 0.0

        if self.enable_following:
            self.cmd_vel_pub.publish(vel_cmd)

    def search_for_lost_target(self):
        """Search behavior when target is temporarily lost"""
        if (abs(self.last_movement_direction['x']) > 0.1 or 
            abs(self.last_movement_direction['y']) > 0.1):
            reduced_forward = self.last_movement_direction['x'] * 0.3
            reduced_side = self.last_movement_direction['y'] * 0.3

            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = reduced_forward
            vel_cmd.twist.linear.y = reduced_side
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 4.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
        else:
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = 0.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 4.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0

        if self.enable_following:
            self.cmd_vel_pub.publish(vel_cmd)

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
                time.sleep(1.0)  # Add a short delay before sending velocity commands
            else:
                self.get_logger().error(f'Takeoff service failed: {response.message}')
                self.takeoff_initiated = False
                self.drone_state = 'IDLE'
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
            self.takeoff_initiated = False
            self.drone_state = 'IDLE'

    def hover_drone(self):
        """Make drone slowly inch forward while rotating when losing target"""
        if self.enable_following:
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = 0.1
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            vel_cmd.twist.angular.z = 0.8
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
            vel_cmd.twist.angular.z = 0.6
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

    def pid_control(self, pid_params, error, dt):
        """PID controller with integral windup protection"""
        if dt <= 0:
            return 0.0

        if abs(error) < 0.05:
            error = 0.0

        max_integral = 5.0
        if abs(error) < 0.5:
            pid_params['integral'] = max(-max_integral, min(pid_params['integral'] + error * dt, max_integral))

        derivative = (error - pid_params['prev_error']) / dt if pid_params['prev_error'] is not None else 0.0
        if abs(derivative) > 10.0:
            derivative = 10.0 if derivative > 0 else -10.0

        output = (pid_params['kp'] * error + 
                 pid_params['ki'] * pid_params['integral'] + 
                 pid_params['kd'] * derivative)

        pid_params['prev_error'] = error
        return output

    def update_target_data_async(self, moving_targets, camera_id, header):
        """Thread safe target data update"""
        current_time = time.time()

        persons = [target for target in moving_targets if target.get('class', -1) == 0]
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
        dt = 0.067  # ~15Hz, so always >2Hz

        if self.drone_state == 'TAKING_OFF' and not self.takeoff_complete:
            return

        if self.drone_state == 'SEARCHING':
            self.momentum_search_behavior()
            return

        if not self.following_active or not hasattr(self, 'target_person_data'):
            if self.takeoff_complete and self.drone_state != 'SEARCHING':
                self.hover_drone()
            return
        try:
            person = self.target_person_data
            target_camera = person.get('camera_id', 0)
            bbox = person.get('bbox', [0, 0, 100, 100])
            center = person.get('center', [bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2])
            smoothed_center = self.smooth_target_position(center)
            if smoothed_center:
                center = smoothed_center
            if self.target_prediction:
                center = self.target_prediction
            image_center = [self.image_width / 2, self.image_height / 2]
            yaw_angle, pitch_angle = self.pixel_to_world_direction(center, image_center)
            estimated_distance = self.estimate_person_distance(bbox)
            if hasattr(self, 'last_estimated_distance'):
                max_distance_change = 2.0
                distance_change = abs(estimated_distance - self.last_estimated_distance)

                if distance_change > max_distance_change:
                    if estimated_distance > self.last_estimated_distance:
                        estimated_distance = self.last_estimated_distance + max_distance_change
                    else:
                        estimated_distance = self.last_estimated_distance - max_distance_change

            self.last_estimated_distance = estimated_distance
            cam_yaw_offset = math.radians(self.camera_orientations[target_camera]['yaw'])
            world_yaw_angle = yaw_angle + cam_yaw_offset

            yaw_error = world_yaw_angle * 0.5
            distance_error = estimated_distance - self.follow_distance
            if distance_error > 0:
                distance_error = distance_error * 0.4
            else:
                distance_error = abs(distance_error) * 0.1

            height_error = pitch_angle * 0.15
            side_error = yaw_angle * 0.12

            yaw_cmd = self.pid_control(self.pid_yaw, yaw_error, dt)
            forward_cmd = self.pid_control(self.pid_x, distance_error, dt)
            height_cmd = self.pid_control(self.pid_z, height_error, dt)
            side_cmd = self.pid_control(self.pid_y, side_error, dt)

            # --- General collision avoidance ---
            if hasattr(self, 'collision_avoidance_cmd') and self.collision_avoidance_cmd:
                forward_cmd += self.collision_avoidance_cmd['x']
                side_cmd += self.collision_avoidance_cmd['y']

            if forward_cmd <= 0:
                forward_cmd = 0.4
            yaw_cmd = self.apply_deadband(yaw_cmd, self.deadband_yaw)
            forward_cmd = self.apply_deadband(forward_cmd, self.deadband_x)
            height_cmd = self.apply_deadband(height_cmd, self.deadband_z)
            side_cmd = self.apply_deadband(side_cmd, self.deadband_y)

            yaw_cmd = max(-1.0, min(yaw_cmd, 1.0))
            forward_cmd = max(0.1, min(forward_cmd, 2.0))
            height_cmd = max(-0.8, min(height_cmd, 0.8))
            side_cmd = max(-2.0, min(side_cmd, 2.0))
            if target_camera != self.primary_camera:
                cos_offset = math.cos(cam_yaw_offset)
                sin_offset = math.sin(cam_yaw_offset)

                transformed_forward = forward_cmd * cos_offset - side_cmd * sin_offset
                transformed_side = forward_cmd * sin_offset + side_cmd * cos_offset

                forward_cmd = transformed_forward
                side_cmd = transformed_side
            self.last_movement_direction['x'] = forward_cmd
            self.last_movement_direction['y'] = side_cmd
            self.last_movement_direction['z'] = height_cmd
            self.last_movement_direction['yaw'] = yaw_cmd
            self.search_momentum_active = False
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = forward_cmd
            vel_cmd.twist.linear.y = side_cmd
            vel_cmd.twist.linear.z = height_cmd
            vel_cmd.twist.angular.z = yaw_cmd
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0

            self.publish_velocity_command(vel_cmd)

            if sum(self.camera_frame_counts.values()) % 40 == 0:
                self.get_logger().info(f'CONTROL CMD: forward={forward_cmd:.2f}, side={side_cmd:.2f}, '
                                     f'height={height_cmd:.2f}, yaw={yaw_cmd:.2f}')

            if sum(self.camera_frame_counts.values()) % 60 == 0:
                cam_name = self.camera_orientations[target_camera]['name']
                lock_status = "LOCKED" if self.target_locked else "TRACKING"
                self.get_logger().info(f'{lock_status} from {cam_name}: '
                                     f'dist={estimated_distance:.1f}m->target:{self.follow_distance}m, '
                                     f'conf={self.target_lock_confidence:.2f}, '
                                     f'err=[y:{math.degrees(world_yaw_angle):.1f}°, d:{distance_error:.1f}m], '
                                     f'cmd=[f:{forward_cmd:.2f}, s:{side_cmd:.2f}, u:{height_cmd:.2f}, y:{yaw_cmd:.2f}]')

        except Exception as e:
            self.get_logger().error(f'Enhanced control loop error: {e}')
            self.hover_drone()

    def momentum_search_behavior(self):
        """Hover and rotate search instead of momentum-based movement"""
        current_time = time.time()

        # Initialize search if not already active
        if not self.search_momentum_active:
            self.search_momentum_active = True
            self.momentum_search_start_time = current_time
            self.search_velocity = self.last_movement_direction.copy()  # Start with last direction
            self.get_logger().info('TARGET LOST: Continuing in last known direction with momentum')

        # Decay velocity
        decay = self.momentum_decay_factor  # e.g., 0.95
        for key in ['x', 'y', 'z', 'yaw']:
            self.search_velocity[key] *= decay

        # Limit velocity change to avoid PX4 flight task errors
        max_search_vel = 0.8  # m/s, conservative for PX4
        for key in ['x', 'y', 'z']:
            self.search_velocity[key] = max(-max_search_vel, min(self.search_velocity[key], max_search_vel))
        self.search_velocity['yaw'] = max(-0.8, min(self.search_velocity['yaw'], 0.8))

        # If velocity is very small, stop and rotate to search
        if abs(self.search_velocity['x']) < self.min_momentum_threshold and \
           abs(self.search_velocity['y']) < self.min_momentum_threshold:
            self.search_velocity['x'] = 0.0
            self.search_velocity['y'] = 0.0
            vel_cmd = VelCmd()
            vel_cmd.twist.linear.x = 0.0
            vel_cmd.twist.linear.y = 0.0
            vel_cmd.twist.linear.z = 0.0
            vel_cmd.twist.angular.z = 0.8  # Rotate to search
            vel_cmd.twist.angular.x = 0.0
            vel_cmd.twist.angular.y = 0.0
            if self.enable_following:
                self.cmd_vel_pub.publish(vel_cmd)
            self.get_logger().info('Momentum search stopped, rotating to search')
            return

        # Continue moving in decayed direction
        vel_cmd = VelCmd()
        vel_cmd.twist.linear.x = self.search_velocity['x']
        vel_cmd.twist.linear.y = self.search_velocity['y']
        vel_cmd.twist.linear.z = self.search_velocity['z']
        vel_cmd.twist.angular.z = self.search_velocity['yaw']
        vel_cmd.twist.angular.x = 0.0
        vel_cmd.twist.angular.y = 0.0

        if self.enable_following:
            self.cmd_vel_pub.publish(vel_cmd)

    def initialize_detection_systems(self):
        """Initialize YOLOv7 + DeepSORT system"""
        self.get_logger().info("[DEBUG] Initializing detection systems...")

        if YOLO_DEEPSORT_AVAILABLE:
            try:
                self.device = select_device('')
                self.half = self.device.type != 'cpu'
                self.get_logger().info(f"[DEBUG] Selected device: {self.device}")

                weights_path = DETECTION_PATH / 'yolov7.pt'
                self.get_logger().info(f"[DEBUG] Looking for weights at: {weights_path}")

                if weights_path.exists():
                    self.get_logger().info("[DEBUG] YOLOv7 weights found, loading model...")

                    try:
                        self.get_logger().info("[DEBUG] Attempting safe loading...")
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
                        self.get_logger().info("[DEBUG] Safe loading successful")

                    except Exception as safe_error:
                        self.get_logger().warn(f"[DEBUG] Safe loading failed: {safe_error}")
                        self.get_logger().warn("[DEBUG] Attempting unsafe loading...")

                        original_load = torch.load

                        def unsafe_load(*args, **kwargs):
                            kwargs['weights_only'] = False
                            return original_load(*args, **kwargs)

                        torch.load = unsafe_load
                        self.model = attempt_load(str(weights_path), map_location=self.device)
                        torch.load = original_load
                        self.get_logger().info("[DEBUG] Unsafe loading successful")

                    self.stride = int(self.model.stride.max())
                    self.img_size = 640

                    if self.half:
                        self.model.half()

                    self.model.eval()
                    self.get_logger().info("[DEBUG] YOLOv7 model loaded successfully")

                    # Initialize DeepSORT
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
                        self.get_logger().info('[DEBUG] YOLOv7 + DeepSORT initialized successfully')
                        self.warmup_model()

                    else:
                        self.get_logger().warn("[DEBUG] Using YOLOv7 only")
                        self.use_yolo_deepsort = False
                        self.use_yolo_only = True
                        self.warmup_model()

                else:
                    self.use_yolo_deepsort = False
                    self.use_yolo_only = False

            except Exception as e:
                self.get_logger().error(f'[DEBUG] Failed to initialize YOLOv7+DeepSORT: {e}')
                self.use_yolo_deepsort = False
                self.use_yolo_only = False
        else:
            self.use_yolo_deepsort = False
            self.use_yolo_only = False

        # Fallback to OpenCV
        if not self.use_yolo_deepsort and not self.use_yolo_only:
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

    def detect_and_track_yolo_deepsort(self, image, camera_id):
        """YOLOv7 + DeepSORT detection and tracking for specific camera"""
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

        pred = non_max_suppression(pred, self.conf_threshold, self.iou_threshold, classes=None, agnostic=False)

        moving_targets = []

        for i, det in enumerate(pred):
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                xywh_bboxs = []
                confs = []
                oids = []

                for *xyxy, conf, cls in reversed(det):
                    x_c, y_c, bbox_w, bbox_h = self.xyxy_to_xywh(*xyxy)
                    xywh_obj = [x_c, y_c, bbox_w, bbox_h]
                    xywh_bboxs.append(xywh_obj)
                    confs.append([conf.item()])
                    oids.append(int(cls))

                xywhs = torch.Tensor(xywh_bboxs)
                confss = torch.Tensor(confs)

                outputs = self.deepsort.update(xywhs, confss, oids, im0)

                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -2]
                    object_id = outputs[:, -1]

                    im0, moving_targets = self.draw_boxes(im0, bbox_xyxy, object_id, identities, camera_id=camera_id)

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
            with self.camera_locks[camera_id]:
                self.camera_frame_counts[camera_id] += 1

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
            cv2.putText(vis_image, f"CAMERA: {cam_name.upper()}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
            cv2.putText(vis_image, f"STATE: {self.drone_state}", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, state_color, thickness)

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

    def predict_target_position(self, current_center):
        if self.last_target_center is None or current_center is None:
            return current_center

        dx = current_center[0] - self.last_target_center[0]
        dy = current_center[1] - self.last_target_center[1]

        prediction_frames = 2
        predicted_x = current_center[0] + dx * prediction_frames
        predicted_y = current_center[1] + dy * prediction_frames

        # Clamp to image bounds
        predicted_x = max(0, min(predicted_x, self.image_width))
        predicted_y = max(0, min(predicted_y, self.image_height))

        return [predicted_x, predicted_y]

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
        if hasattr(node, 'enable_following') and node.enable_following:
            node.get_logger().info('Shutting down - Landing drone via RPC service')

            if hasattr(node, 'land_client') and node.land_client.wait_for_service(timeout_sec=5.0):
                request = Land.Request()
                future = node.land_client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

                try:
                    response = future.result()
                    if response.success:
                        node.get_logger().info(f'Landing successful: {response.message}')
                    else:
                        node.get_logger().error(f'Landing failed: {response.message}')
                except Exception as e:
                    node.get_logger().error(f'Landing service call failed: {e}')

            if hasattr(node, 'cmd_vel_pub'):
                vel_cmd = VelCmd()
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.linear.y = 0.0
                vel_cmd.twist.linear.z = 0.0
                vel_cmd.twist.angular.z = 0.0
                node.cmd_vel_pub.publish(vel_cmd)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
