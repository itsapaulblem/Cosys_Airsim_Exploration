#!/usr/bin/env python3
"""
AirSim ROS2 Python Subscriber Example

This script demonstrates how to subscribe to various AirSim ROS2 topics and 
interact with the AirSim node using Python and rclpy.

Requirements:
- ROS2 Foxy/Galactic/Humble
- AirSim ROS2 workspace built and sourced
- AirSim simulation running
- airsim_node running

Usage:
    python3 airsim_ros2_subscriber.py [options]

Author: Generated for Cosys-AirSim ROS2 Integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import argparse
import sys
import json
import time
import math
from datetime import datetime
import threading

# Standard ROS2 messages
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, MagneticField, PointCloud2, CompressedImage
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion

# AirSim custom messages
from airsim_interfaces.msg import (
    Environment,
    InstanceSegmentationList,
    ObjectTransformsList,
    VelCmd,
    CarState,
    ComputerVisionState,
    GPSYaw,
    Altimeter
)

# AirSim services
from airsim_interfaces.srv import (
    Takeoff,
    Land,
    SetAltitude,
    SetLocalPosition,
    Reset,
    RefreshInstanceSegmentation,
    RefreshObjectTransforms
)

class AirSimROS2Subscriber(Node):
    """
    Comprehensive ROS2 subscriber for AirSim topics with examples of:
    - Vehicle state monitoring (odometry, GPS, IMU)
    - Sensor data processing (cameras, LiDAR, etc.)
    - Instance segmentation and object tracking
    - Command publishing (velocity commands)
    - Service calling (takeoff, land, etc.)
    """
    
    def __init__(self, vehicle_name="", enable_logging=True, log_file=None):
        super().__init__('airsim_ros2_subscriber')
        
        self.vehicle_name = vehicle_name
        self.enable_logging = enable_logging
        self.log_file = log_file
        
        # Vehicle state storage
        self.current_odom = None
        self.current_gps = None
        self.current_imu = None
        self.current_env = None
        self.last_update_time = None
        
        # Thread-safe data access
        self.data_lock = threading.Lock()
        
        # Statistics
        self.message_counts = {
            'odometry': 0,
            'gps': 0,
            'imu': 0,
            'environment': 0,
            'instance_segmentation': 0,
            'object_transforms': 0
        }
        
        # Setup callback groups for parallel processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Configure QoS profiles
        self.setup_qos_profiles()
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup service clients
        self.setup_service_clients()
        
        # Setup logging
        if self.enable_logging and self.log_file:
            self.setup_logging()
        
        # Create timer for periodic status updates
        self.create_timer(5.0, self.print_status, callback_group=self.callback_group)
        
        self.get_logger().info(f'AirSim ROS2 Subscriber initialized for vehicle: {vehicle_name or "all"}')
        
    def setup_qos_profiles(self):
        """Configure QoS profiles for different types of data"""
        # Best effort for real-time sensor data
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Reliable for important state data
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # High-frequency for control commands
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
    def setup_subscribers(self):
        """Setup all ROS2 subscribers"""
        # Vehicle state subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_topic_name('odom_local'),
            self.odometry_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.get_topic_name('global_gps'),
            self.gps_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        self.env_sub = self.create_subscription(
            Environment,
            self.get_topic_name('environment'),
            self.environment_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        # Sensor subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_topic_name('imu'),
            self.imu_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        self.mag_sub = self.create_subscription(
            MagneticField,
            self.get_topic_name('magnetometer'),
            self.magnetometer_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        # AirSim-specific subscribers
        self.instance_seg_sub = self.create_subscription(
            InstanceSegmentationList,
            self.get_topic_name('instance_segmentation'),
            self.instance_segmentation_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        self.obj_transforms_sub = self.create_subscription(
            ObjectTransformsList,
            self.get_topic_name('object_transforms'),
            self.object_transforms_callback,
            self.sensor_qos,
            callback_group=self.callback_group
        )
        
        # Vehicle-specific subscribers
        if self.vehicle_name:
            self.car_state_sub = self.create_subscription(
                CarState,
                self.get_topic_name('car_state'),
                self.car_state_callback,
                self.sensor_qos,
                callback_group=self.callback_group
            )
            
            self.cv_state_sub = self.create_subscription(
                ComputerVisionState,
                self.get_topic_name('computer_vision_state'),
                self.computer_vision_state_callback,
                self.sensor_qos,
                callback_group=self.callback_group
            )
        
    def setup_publishers(self):
        """Setup command publishers"""
        self.vel_cmd_world_pub = self.create_publisher(
            VelCmd,
            self.get_topic_name('vel_cmd_world_frame'),
            self.control_qos
        )
        
        self.vel_cmd_body_pub = self.create_publisher(
            VelCmd,
            self.get_topic_name('vel_cmd_body_frame'),
            self.control_qos
        )
        
    def setup_service_clients(self):
        """Setup service clients"""
        self.takeoff_client = self.create_client(
            Takeoff,
            self.get_topic_name('takeoff')
        )
        
        self.land_client = self.create_client(
            Land,
            self.get_topic_name('land')
        )
        
        self.set_altitude_client = self.create_client(
            SetAltitude,
            self.get_topic_name('set_altitude')
        )
        
        self.reset_client = self.create_client(
            Reset,
            self.get_topic_name('reset')
        )
        
        self.refresh_instance_seg_client = self.create_client(
            RefreshInstanceSegmentation,
            self.get_topic_name('refresh_instance_segmentation')
        )
        
    def setup_logging(self):
        """Setup file logging"""
        self.log_file_handle = open(self.log_file, 'w')
        self.log_file_handle.write(f"# AirSim ROS2 Subscriber Log - {datetime.now()}\n")
        self.log_file_handle.write("# timestamp,message_type,data\n")
        
    def get_topic_name(self, base_topic):
        """Get topic name with vehicle namespace if specified"""
        if self.vehicle_name:
            return f'/{self.vehicle_name}/{base_topic}'
        return f'/{base_topic}'
        
    def log_message(self, msg_type, data):
        """Log message to file if enabled"""
        if self.enable_logging and hasattr(self, 'log_file_handle'):
            timestamp = time.time()
            log_entry = f"{timestamp},{msg_type},{json.dumps(data)}\n"
            self.log_file_handle.write(log_entry)
            self.log_file_handle.flush()
            
    # Callback functions
    def odometry_callback(self, msg):
        """Process odometry messages"""
        with self.data_lock:
            self.current_odom = msg
            self.message_counts['odometry'] += 1
            self.last_update_time = self.get_clock().now()
            
        # Extract position and orientation
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        lin_vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular
        
        # Log detailed information
        odom_data = {
            'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
            'orientation': {'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w},
            'linear_velocity': {'x': lin_vel.x, 'y': lin_vel.y, 'z': lin_vel.z},
            'angular_velocity': {'x': ang_vel.x, 'y': ang_vel.y, 'z': ang_vel.z}
        }
        
        self.log_message('odometry', odom_data)
        
        # Print every 10th message to avoid spam
        if self.message_counts['odometry'] % 10 == 0:
            self.get_logger().info(
                f'Odometry #{self.message_counts["odometry"]}: '
                f'Pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'Vel=({lin_vel.x:.2f}, {lin_vel.y:.2f}, {lin_vel.z:.2f})'
            )
            
    def gps_callback(self, msg):
        """Process GPS messages"""
        with self.data_lock:
            self.current_gps = msg
            self.message_counts['gps'] += 1
            
        gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status,
            'service': msg.status.service
        }
        
        self.log_message('gps', gps_data)
        
        if self.message_counts['gps'] % 5 == 0:
            self.get_logger().info(
                f'GPS #{self.message_counts["gps"]}: '
                f'Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.2f}m'
            )
            
    def imu_callback(self, msg):
        """Process IMU messages"""
        with self.data_lock:
            self.current_imu = msg
            self.message_counts['imu'] += 1
            
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        
        imu_data = {
            'linear_acceleration': {'x': accel.x, 'y': accel.y, 'z': accel.z},
            'angular_velocity': {'x': gyro.x, 'y': gyro.y, 'z': gyro.z},
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            }
        }
        
        self.log_message('imu', imu_data)
        
        if self.message_counts['imu'] % 20 == 0:
            self.get_logger().info(
                f'IMU #{self.message_counts["imu"]}: '
                f'Accel=({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}), '
                f'Gyro=({gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f})'
            )
            
    def magnetometer_callback(self, msg):
        """Process magnetometer messages"""
        mag_field = msg.magnetic_field
        mag_data = {
            'magnetic_field': {'x': mag_field.x, 'y': mag_field.y, 'z': mag_field.z}
        }
        
        self.log_message('magnetometer', mag_data)
        
        self.get_logger().debug(
            f'Magnetometer: Field=({mag_field.x:.2f}, {mag_field.y:.2f}, {mag_field.z:.2f})'
        )
        
    def environment_callback(self, msg):
        """Process environment messages"""
        with self.data_lock:
            self.current_env = msg
            self.message_counts['environment'] += 1
            
        env_data = {
            'header': {
                'frame_id': msg.header.frame_id,
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
        }
        
        self.log_message('environment', env_data)
        
        self.get_logger().debug(f'Environment: Frame={msg.header.frame_id}')
        
    def instance_segmentation_callback(self, msg):
        """Process instance segmentation messages"""
        with self.data_lock:
            self.message_counts['instance_segmentation'] += 1
            
        seg_data = {
            'num_objects': len(msg.labels),
            'labels': [{'id': label.id, 'name': label.name} for label in msg.labels]
        }
        
        self.log_message('instance_segmentation', seg_data)
        
        self.get_logger().info(f'Instance Segmentation: {len(msg.labels)} objects detected')
        for label in msg.labels:
            self.get_logger().debug(f'  - Object {label.id}: {label.name}')
            
    def object_transforms_callback(self, msg):
        """Process object transforms messages"""
        with self.data_lock:
            self.message_counts['object_transforms'] += 1
            
        transforms_data = {
            'num_objects': len(msg.transforms),
            'transforms': []
        }
        
        for transform in msg.transforms:
            transform_data = {
                'name': transform.name,
                'position': {
                    'x': transform.position.x,
                    'y': transform.position.y,
                    'z': transform.position.z
                },
                'orientation': {
                    'x': transform.orientation.x,
                    'y': transform.orientation.y,
                    'z': transform.orientation.z,
                    'w': transform.orientation.w
                }
            }
            transforms_data['transforms'].append(transform_data)
            
        self.log_message('object_transforms', transforms_data)
        
        self.get_logger().info(f'Object Transforms: {len(msg.transforms)} objects')
        
    def car_state_callback(self, msg):
        """Process car state messages"""
        self.get_logger().debug(f'Car State: Speed={msg.speed:.2f}, Gear={msg.gear}')
        
    def computer_vision_state_callback(self, msg):
        """Process computer vision state messages"""
        self.get_logger().debug(f'Computer Vision State received')
        
    # Command publishing functions
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0, duration=1.0, world_frame=True):
        """
        Send velocity command to AirSim
        
        Args:
            vx, vy, vz: Velocity in m/s
            yaw_rate: Yaw rate in rad/s
            duration: Command duration in seconds
            world_frame: Use world frame (True) or body frame (False)
        """
        vel_cmd = VelCmd()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = "world" if world_frame else "body"
        vel_cmd.vehicle_name = self.vehicle_name
        vel_cmd.vx = vx
        vel_cmd.vy = vy
        vel_cmd.vz = vz
        vel_cmd.yaw_rate = yaw_rate
        
        publisher = self.vel_cmd_world_pub if world_frame else self.vel_cmd_body_pub
        publisher.publish(vel_cmd)
        
        self.get_logger().info(
            f'Sent velocity command: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, '
            f'yaw_rate={yaw_rate:.2f} ({"world" if world_frame else "body"} frame)'
        )
        
    # Service calling functions
    def call_takeoff(self, timeout_sec=20.0):
        """Call takeoff service"""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available')
            return False
            
        request = Takeoff.Request()
        request.waitonfull_timeout = timeout_sec
        
        future = self.takeoff_client.call_async(request)
        return future
        
    def call_land(self, timeout_sec=20.0):
        """Call land service"""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Land service not available')
            return False
            
        request = Land.Request()
        request.waitonfull_timeout = timeout_sec
        
        future = self.land_client.call_async(request)
        return future
        
    def call_set_altitude(self, altitude, timeout_sec=20.0):
        """Call set altitude service"""
        if not self.set_altitude_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set altitude service not available')
            return False
            
        request = SetAltitude.Request()
        request.altitude = altitude
        request.waitonfull_timeout = timeout_sec
        
        future = self.set_altitude_client.call_async(request)
        return future
        
    def call_reset(self):
        """Call reset service"""
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Reset service not available')
            return False
            
        request = Reset.Request()
        future = self.reset_client.call_async(request)
        return future
        
    def print_status(self):
        """Print periodic status information"""
        with self.data_lock:
            current_time = self.get_clock().now()
            
            self.get_logger().info(
                f'=== AirSim ROS2 Subscriber Status ===\n'
                f'Vehicle: {self.vehicle_name or "all"}\n'
                f'Message Counts: {self.message_counts}\n'
                f'Last Update: {self.last_update_time.nanoseconds * 1e-9 if self.last_update_time else "Never"}\n'
                f'Current Position: {self.get_position_string()}\n'
                f'Current GPS: {self.get_gps_string()}\n'
                f'=================================='
            )
            
    def get_position_string(self):
        """Get formatted position string"""
        if self.current_odom:
            pos = self.current_odom.pose.pose.position
            return f'({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
        return 'No data'
        
    def get_gps_string(self):
        """Get formatted GPS string"""
        if self.current_gps:
            return f'({self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f}, {self.current_gps.altitude:.2f})'
        return 'No data'
        
    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'log_file_handle'):
            self.log_file_handle.close()

def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='AirSim ROS2 Subscriber')
    parser.add_argument('--vehicle', '-v', type=str, default='', 
                       help='Vehicle name (leave empty for all vehicles)')
    parser.add_argument('--log-file', '-l', type=str, default=None,
                       help='Log file path for message logging')
    parser.add_argument('--no-logging', action='store_true',
                       help='Disable console logging')
    parser.add_argument('--demo', action='store_true',
                       help='Run demonstration commands')
    
    # Parse known args to allow ROS2 args
    parsed_args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=unknown)
    
    try:
        # Create subscriber node
        subscriber = AirSimROS2Subscriber(
            vehicle_name=parsed_args.vehicle,
            enable_logging=not parsed_args.no_logging,
            log_file=parsed_args.log_file
        )
        
        # Use multithreaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(subscriber)
        
        # Run demonstration if requested
        if parsed_args.demo:
            demo_thread = threading.Thread(target=run_demo, args=(subscriber,))
            demo_thread.daemon = True
            demo_thread.start()
            
        try:
            # Spin the executor
            executor.spin()
        except KeyboardInterrupt:
            subscriber.get_logger().info('Shutting down...')
            
    except Exception as e:
        print(f'Error: {e}')
        return 1
    finally:
        # Cleanup
        if 'subscriber' in locals():
            subscriber.cleanup()
            subscriber.destroy_node()
        rclpy.shutdown()
        
    return 0

def run_demo(subscriber):
    """Run demonstration commands"""
    subscriber.get_logger().info('Starting demo in 5 seconds...')
    time.sleep(5)
    
    # Example 1: Send velocity commands
    subscriber.get_logger().info('Demo: Moving forward for 3 seconds')
    subscriber.send_velocity_command(1.0, 0.0, 0.0, 0.0)
    time.sleep(3)
    
    subscriber.get_logger().info('Demo: Stopping')
    subscriber.send_velocity_command(0.0, 0.0, 0.0, 0.0)
    time.sleep(2)
    
    # Example 2: Try takeoff (if supported)
    subscriber.get_logger().info('Demo: Attempting takeoff')
    future = subscriber.call_takeoff()
    if future:
        rclpy.spin_until_future_complete(subscriber, future, timeout_sec=10.0)
        if future.result():
            subscriber.get_logger().info('Demo: Takeoff successful')
        else:
            subscriber.get_logger().info('Demo: Takeoff failed or not supported')
    
    time.sleep(5)
    
    # Example 3: Try landing (if supported)
    subscriber.get_logger().info('Demo: Attempting landing')
    future = subscriber.call_land()
    if future:
        rclpy.spin_until_future_complete(subscriber, future, timeout_sec=10.0)
        if future.result():
            subscriber.get_logger().info('Demo: Landing successful')
        else:
            subscriber.get_logger().info('Demo: Landing failed or not supported')
    
    subscriber.get_logger().info('Demo completed')

if __name__ == '__main__':
    sys.exit(main())