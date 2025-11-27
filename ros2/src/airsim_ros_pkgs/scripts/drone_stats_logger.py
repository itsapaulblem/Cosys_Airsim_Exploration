#!/usr/bin/env python3
"""
Drone Movement Statistics Logger

A comprehensive ROS2 node for collecting and analyzing drone movement statistics
from the mission system's event detection and status reporting systems.

Usage:
    ros2 run airsim_ros_pkgs drone_stats_logger.py
    python3 drone_stats_logger.py

Author: Claude Code Integration
Version: 1.0
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from mission_search_interfaces.msg import MissionEvent, MissionStatus, TargetDetection
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from airsim_interfaces.srv import Takeoff, Land
from std_srvs.srv import Empty
import json
import os
from datetime import datetime, timezone
import math
import threading
from typing import Dict, Set, Optional, List, Any, Tuple
import logging
from enum import Enum
import time
import re

# Import database abstraction layer
from database_interface import DatabaseInterface, create_tables_if_not_exist


class FlightState(Enum):
    """Flight state enumeration for tracking vehicle status."""
    UNKNOWN = "UNKNOWN"
    GROUNDED = "GROUNDED"
    TAKING_OFF = "TAKING_OFF"
    AIRBORNE = "AIRBORNE"
    LANDING = "LANDING"
    HOVERING = "HOVERING"


class ServiceCallEvent:
    """Represents a monitored service call event."""
    def __init__(self, vehicle_name: str, service_name: str, timestamp: float, call_id: str):
        self.vehicle_name = vehicle_name
        self.service_name = service_name
        self.timestamp = timestamp
        self.call_id = call_id
        self.completed = False
        self.success = False
        self.movement_detected = False


class DroneMovementStatsLogger(Node):
    """
    Advanced drone movement statistics logger with multi-vehicle support.
    
    Features:
    - Automatic vehicle discovery and subscription
    - SQLite database storage with comprehensive schema
    - Real-time statistics aggregation
    - Thread-safe operation
    - JSON export capabilities
    - Fleet-wide analytics
    """
    
    def __init__(self):
        super().__init__('drone_stats_logger')
        
        # Configuration parameters
        self.declare_parameter('database_type', 'auto')  # auto, sqlite, postgresql
        self.declare_parameter('database_path', 'drone_movement_stats.db')
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('stats_update_rate', 10.0)  # seconds
        self.declare_parameter('discovery_rate', 30.0)    # seconds
        self.declare_parameter('export_on_shutdown', True)
        
        # Get parameters
        self.database_type = self.get_parameter('database_type').get_parameter_value().string_value
        self.db_path = self.get_parameter('database_path').get_parameter_value().string_value
        self.stats_rate = self.get_parameter('stats_update_rate').get_parameter_value().double_value
        self.discovery_rate = self.get_parameter('discovery_rate').get_parameter_value().double_value
        self.export_on_shutdown = self.get_parameter('export_on_shutdown').get_parameter_value().bool_value
        
        # Thread-safe data structures
        self.lock = threading.RLock()
        self.active_vehicles: Set[str] = set()
        self.vehicle_stats: Dict[str, Dict[str, Any]] = {}
        self.vehicle_subscriptions: Dict[str, List] = {}

        # Enhanced active event detection
        self.flight_states: Dict[str, FlightState] = {}
        self.pending_service_calls: Dict[str, List[ServiceCallEvent]] = {}
        self.altitude_history: Dict[str, List[Tuple[float, float]]] = {}  # [(timestamp, altitude), ...]
        self.velocity_history: Dict[str, List[Tuple[float, float]]] = {}  # [(timestamp, speed), ...]
        self.service_clients: Dict[str, Dict[str, Any]] = {}  # Per-vehicle service clients

        # Callback group for service calls
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Database setup
        self.setup_database()

        self.get_logger().info("Enhanced Active Event Detection enabled:")
        self.get_logger().info("  • Service call monitoring (takeoff/landing)")
        self.get_logger().info("  • Flight state machine tracking")
        self.get_logger().info("  • Movement pattern analysis")
        self.get_logger().info("  • Event correlation and validation")
        
        # Timers for periodic tasks
        self.stats_timer = self.create_timer(self.stats_rate, self.log_periodic_stats)
        self.discovery_timer = self.create_timer(self.discovery_rate, self.discover_vehicles)
        
        # Initial vehicle discovery
        self.discover_vehicles()
        
        self.get_logger().info(f"Drone Statistics Logger initialized")
        self.get_logger().info(f"Database type: {self.db.db_type.value}")
        self.get_logger().info(f"Stats update rate: {self.stats_rate}s")
        
    def setup_database(self):
        """Initialize database with unified interface supporting SQLite and PostgreSQL."""
        try:
            # Initialize database interface
            db_type = None if self.database_type == 'auto' else self.database_type
            
            # Set SQLite path if using SQLite
            if db_type == 'sqlite' or self.database_type == 'auto':
                os.environ['SQLITE_DATABASE_PATH'] = self.db_path
            
            self.db = DatabaseInterface(db_type=db_type)
            
            # Create tables if they don't exist (for SQLite)
            create_tables_if_not_exist(self.db)
            
            self.get_logger().info(f"Database initialized successfully: {self.db.db_type.value}")
            
            if self.db.db_type.value == 'postgresql':
                self.get_logger().info(f"PostgreSQL schema: {self.db.config.pg_schema}")
            else:
                self.get_logger().info(f"SQLite database: {self.db.config.sqlite_path}")
            
        except Exception as e:
            self.get_logger().error(f"Database setup failed: {e}")
            raise
    
    def discover_vehicles(self):
        """Automatically discover active vehicles and subscribe to their topics."""
        try:
            # Get list of all active nodes
            node_names_and_namespaces = self.get_node_names_and_namespaces()

            newly_discovered = 0
            all_nodes_found = []
            candidate_vehicles = []
            filtered_nodes = []

            for node_name, namespace in node_names_and_namespaces:
                full_node_name = f"{namespace}/{node_name}" if namespace else node_name
                all_nodes_found.append(full_node_name)

                # Apply precise vehicle node detection
                if self._is_vehicle_node(node_name, full_node_name):
                    candidate_vehicles.append(node_name)
                    vehicle_name = node_name  # Use just the node name as vehicle name

                    if vehicle_name not in self.active_vehicles:
                        self.subscribe_to_vehicle(vehicle_name)
                        newly_discovered += 1
                else:
                    filtered_nodes.append(full_node_name)

            # Enhanced logging for debugging
            # self.get_logger().info(f"Vehicle Discovery Report:")
            # self.get_logger().info(f"  Total nodes found: {len(all_nodes_found)}")
            # self.get_logger().info(f"  Vehicle candidates: {candidate_vehicles}")
            # self.get_logger().info(f"  Filtered out: {len(filtered_nodes)} non-vehicle nodes")

            if newly_discovered > 0:
                self.get_logger().info(f"Successfully discovered {newly_discovered} new vehicles: {candidate_vehicles[-newly_discovered:]}")

            if len(candidate_vehicles) == 0:
                self.get_logger().warn("No vehicle nodes detected. Expected nodes like 'PX4_Drone1', 'PX4_Drone2', etc.")

        except Exception as e:
            self.get_logger().error(f"Vehicle discovery failed: {e}")

    def _is_vehicle_node(self, node_name: str, full_node_name: str) -> bool:
        """Precisely determine if a node is a vehicle node."""
        try:
            # Exclude the stats logger itself
            if 'drone_stats_logger' in node_name.lower():
                return False

            # Exclude known non-vehicle nodes
            non_vehicle_patterns = [
                r'.*localization.*',
                r'.*coordination.*',
                r'.*stats.*',
                r'.*logger.*',
                r'.*monitor.*',
                r'.*_node$',  # Nodes ending with '_node' are typically utility nodes
                r'.*helper.*',
                r'.*manager.*'
            ]

            for pattern in non_vehicle_patterns:
                if re.match(pattern, node_name, re.IGNORECASE):
                    self.get_logger().debug(f"Filtered out non-vehicle node: {node_name} (matched pattern: {pattern})")
                    return False

            # Precise vehicle node patterns
            vehicle_patterns = [
                r'^PX4_Drone\d+$',      # PX4_Drone1, PX4_Drone2, etc.
                r'^Drone\d+$',          # Drone1, Drone2, etc.
                r'^Droan\d+$',          # Droan1, Droan2, etc.
                r'^UAV\d+$',            # UAV1, UAV2, etc.
                r'^Vehicle\d+$',        # Vehicle1, Vehicle2, etc.
            ]

            for pattern in vehicle_patterns:
                if re.match(pattern, node_name):
                    self.get_logger().info(f"Detected vehicle node: {node_name} (matched pattern: {pattern})")
                    return True

            # Log unmatched nodes for debugging
            if 'drone' in node_name.lower():
                self.get_logger().debug(f"Node contains 'drone' but doesn't match vehicle patterns: {node_name}")

            return False

        except Exception as e:
            self.get_logger().error(f"Error in vehicle node detection for {node_name}: {e}")
            return False
    
    def subscribe_to_vehicle(self, vehicle_name: str):
        """Subscribe to all relevant topics for a specific vehicle."""
        with self.lock:
            if vehicle_name in self.active_vehicles:
                return  # Already subscribed
                
            self.get_logger().info(f"Subscribing to topics for vehicle: {vehicle_name}")
            
            # Initialize vehicle statistics
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.vehicle_stats[vehicle_name] = {
                'total_distance': 0.0,
                'flight_time': 0.0,
                'takeoffs': 0,
                'landings': 0,
                'missions_completed': 0,
                'targets_detected': 0,
                'last_position': None,
                'last_timestamp': None,
                'first_seen': current_time,
                'last_seen': current_time
            }

            # Initialize enhanced tracking for this vehicle
            self.flight_states[vehicle_name] = FlightState.UNKNOWN
            self.pending_service_calls[vehicle_name] = []
            self.altitude_history[vehicle_name] = []
            self.velocity_history[vehicle_name] = []
            
            
            # Store subscription objects to prevent garbage collection
            self.vehicle_subscriptions[vehicle_name] = []
            
            try:
                # Mission events subscription
                events_topic = f'/{vehicle_name}/mission/events'
                events_sub = self.create_subscription(
                    MissionEvent,
                    events_topic,
                    lambda msg, vname=vehicle_name: self.mission_event_callback(msg, vname),
                    10
                )
                self.vehicle_subscriptions[vehicle_name].append(events_sub)
                
                # Mission status subscription
                status_topic = f'/{vehicle_name}/mission/status'
                status_sub = self.create_subscription(
                    MissionStatus,
                    status_topic,
                    lambda msg, vname=vehicle_name: self.mission_status_callback(msg, vname),
                    10
                )
                self.vehicle_subscriptions[vehicle_name].append(status_sub)
                
                # Target detection subscription
                detection_topic = f'/{vehicle_name}/detections/target'
                detection_sub = self.create_subscription(
                    TargetDetection,
                    detection_topic,
                    lambda msg, vname=vehicle_name: self.target_detection_callback(msg, vname),
                    10
                )
                self.vehicle_subscriptions[vehicle_name].append(detection_sub)
                
                # High-frequency odometry subscription (sampled for path analysis)
                odom_topic = f'/{vehicle_name}/odom_local_ned'
                odom_sub = self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda msg, vname=vehicle_name: self.odometry_callback(msg, vname),
                    10  # Lower queue size for high-frequency data
                )
                self.vehicle_subscriptions[vehicle_name].append(odom_sub)

                # Set up service call monitoring for this vehicle
                self.setup_service_monitoring(vehicle_name)

                # Add vehicle to active set
                self.active_vehicles.add(vehicle_name)

                # Update database with new vehicle
                self.update_vehicle_summary(vehicle_name)

                self.get_logger().info(f"Successfully subscribed to all topics for {vehicle_name}")

                # Log initial state for debugging
                self.get_logger().info(
                    f"Vehicle {vehicle_name} initialized: "
                    f"flight_state={self.flight_states[vehicle_name].value}, "
                    f"stats_initialized=True, "
                    f"service_monitoring=Ready"
                )
                
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to vehicle {vehicle_name}: {e}")

    def setup_service_monitoring(self, vehicle_name: str):
        """Set up service call monitoring for enhanced event detection."""
        try:
            # Create service clients for monitoring
            self.service_clients[vehicle_name] = {}

            # Monitor takeoff service
            takeoff_client = self.create_client(
                Takeoff,
                f'/{vehicle_name}/takeoff',
                callback_group=self.service_callback_group
            )
            self.service_clients[vehicle_name]['takeoff'] = takeoff_client

            # Monitor landing service
            land_client = self.create_client(
                Land,
                f'/{vehicle_name}/land',
                callback_group=self.service_callback_group
            )
            self.service_clients[vehicle_name]['land'] = land_client

            # Set up service call interceptors
            self.create_service(
                Takeoff,
                f'/{vehicle_name}/takeoff_monitor',
                lambda req, vname=vehicle_name: self.takeoff_service_interceptor(req, vname),
                callback_group=self.service_callback_group
            )

            self.create_service(
                Land,
                f'/{vehicle_name}/land_monitor',
                lambda req, vname=vehicle_name: self.land_service_interceptor(req, vname),
                callback_group=self.service_callback_group
            )

            self.get_logger().info(f"Service monitoring set up for {vehicle_name}")

        except Exception as e:
            self.get_logger().error(f"Failed to set up service monitoring for {vehicle_name}: {e}")

    def takeoff_service_interceptor(self, request, vehicle_name: str):
        """Intercept and monitor takeoff service calls."""
        try:
            call_id = f"takeoff_{vehicle_name}_{time.time()}"
            timestamp = self.get_clock().now().nanoseconds / 1e9

            # Record the service call
            service_event = ServiceCallEvent(vehicle_name, "takeoff", timestamp, call_id)

            with self.lock:
                self.pending_service_calls[vehicle_name].append(service_event)
                self.flight_states[vehicle_name] = FlightState.TAKING_OFF

            # Generate immediate TAKEOFF event
            self.generate_enhanced_event(vehicle_name, "TAKEOFF", "SERVICE_CALL", {
                'service_call_id': call_id,
                'service_name': 'takeoff',
                'call_timestamp': timestamp
            })

            self.get_logger().info(f"Detected takeoff service call for {vehicle_name} (ID: {call_id})")

            # Forward to actual service (if client is available)
            response = Takeoff.Response()
            if vehicle_name in self.service_clients and 'takeoff' in self.service_clients[vehicle_name]:
                client = self.service_clients[vehicle_name]['takeoff']
                if client.service_is_ready():
                    future = client.call_async(request)
                    # Note: In real implementation, we'd wait for response
                    response.success = True
                    response.message = "Takeoff initiated and monitored"
                else:
                    response.success = False
                    response.message = "Takeoff service not available"
            else:
                response.success = True
                response.message = "Takeoff monitored (service passthrough not available)"

            return response

        except Exception as e:
            self.get_logger().error(f"Error in takeoff interceptor for {vehicle_name}: {e}")
            response = Takeoff.Response()
            response.success = False
            response.message = f"Error: {e}"
            return response

    def land_service_interceptor(self, request, vehicle_name: str):
        """Intercept and monitor landing service calls."""
        try:
            call_id = f"land_{vehicle_name}_{time.time()}"
            timestamp = self.get_clock().now().nanoseconds / 1e9

            # Record the service call
            service_event = ServiceCallEvent(vehicle_name, "land", timestamp, call_id)

            with self.lock:
                self.pending_service_calls[vehicle_name].append(service_event)
                self.flight_states[vehicle_name] = FlightState.LANDING

            # Generate immediate LANDING event
            self.generate_enhanced_event(vehicle_name, "LANDING", "SERVICE_CALL", {
                'service_call_id': call_id,
                'service_name': 'land',
                'call_timestamp': timestamp
            })

            self.get_logger().info(f"Detected landing service call for {vehicle_name} (ID: {call_id})")

            # Forward to actual service (if client is available)
            response = Land.Response()
            if vehicle_name in self.service_clients and 'land' in self.service_clients[vehicle_name]:
                client = self.service_clients[vehicle_name]['land']
                if client.service_is_ready():
                    future = client.call_async(request)
                    response.success = True
                    response.message = "Landing initiated and monitored"
                else:
                    response.success = False
                    response.message = "Landing service not available"
            else:
                response.success = True
                response.message = "Landing monitored (service passthrough not available)"

            return response

        except Exception as e:
            self.get_logger().error(f"Error in land interceptor for {vehicle_name}: {e}")
            response = Land.Response()
            response.success = False
            response.message = f"Error: {e}"
            return response
    
    def mission_event_callback(self, msg: MissionEvent, vehicle_name: str):
        """Process mission events for movement statistics."""
        try:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            with self.lock:
                # Update vehicle-specific counters
                if msg.event_type == 'TAKEOFF':
                    self.vehicle_stats[vehicle_name]['takeoffs'] += 1
                elif msg.event_type == 'LANDING':
                    self.vehicle_stats[vehicle_name]['landings'] += 1
                
                # Accumulate total distance
                if msg.distance_moved > 0:
                    self.vehicle_stats[vehicle_name]['total_distance'] += msg.distance_moved
                
                self.vehicle_stats[vehicle_name]['last_seen'] = timestamp
            
            # Store event in database using proper DatabaseInterface
            self._insert_movement_event({
                'vehicle_name': vehicle_name,
                'timestamp': timestamp,
                'event_type': msg.event_type,
                'event_source': f"PASSIVE_{msg.event_source}",  # Mark as passive event
                'sequence_number': msg.sequence_number,
                'distance_moved': msg.distance_moved,
                'altitude_change': msg.altitude_change,
                'speed_change': msg.speed_change,
                'position_x': msg.current_position.x,
                'position_y': msg.current_position.y,
                'position_z': msg.current_position.z,
                'previous_x': msg.previous_position.x,
                'previous_y': msg.previous_position.y,
                'previous_z': msg.previous_position.z,
                'velocity_x': msg.current_velocity.x,
                'velocity_y': msg.current_velocity.y,
                'velocity_z': msg.current_velocity.z,
                'mission_id': msg.active_mission_id,
                'mission_phase': msg.mission_phase,
                'mission_progress': msg.mission_progress_percentage,
                'confidence_score': msg.confidence_score,
                'tags': json.dumps(msg.tags) if msg.tags else None
            })
            
            self.get_logger().debug(
                f"Logged {msg.event_type} event for {vehicle_name}: "
                f"moved {msg.distance_moved:.1f}m, altitude Δ{msg.altitude_change:.1f}m"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission event for {vehicle_name}: {e}")

    def generate_enhanced_event(self, vehicle_name: str, event_type: str, event_source: str,
                               additional_data: Dict[str, Any] = None):
        """Generate enhanced events based on active analysis."""
        try:
            timestamp = self.get_clock().now().nanoseconds / 1e9

            with self.lock:
                # Update vehicle-specific counters
                if event_type == 'TAKEOFF':
                    self.vehicle_stats[vehicle_name]['takeoffs'] += 1
                elif event_type == 'LANDING':
                    self.vehicle_stats[vehicle_name]['landings'] += 1

                self.vehicle_stats[vehicle_name]['last_seen'] = timestamp

                # Get current position and velocity if available
                current_pos = self.vehicle_stats[vehicle_name].get('last_position')

                # Prepare enhanced event data
                event_data = {
                    'vehicle_name': vehicle_name,
                    'timestamp': timestamp,
                    'event_type': event_type,
                    'event_source': f"ACTIVE_{event_source}",
                    'sequence_number': int(timestamp * 1000),  # Use timestamp as sequence
                    'distance_moved': 0.0,  # Will be updated by movement analysis
                    'altitude_change': 0.0,
                    'speed_change': 0.0,
                    'position_x': current_pos.x if current_pos else 0.0,
                    'position_y': current_pos.y if current_pos else 0.0,
                    'position_z': current_pos.z if current_pos else 0.0,
                    'previous_x': current_pos.x if current_pos else 0.0,
                    'previous_y': current_pos.y if current_pos else 0.0,
                    'previous_z': current_pos.z if current_pos else 0.0,
                    'velocity_x': 0.0,
                    'velocity_y': 0.0,
                    'velocity_z': 0.0,
                    'mission_id': '',
                    'mission_phase': f"ACTIVE_{event_type}",
                    'mission_progress': 0.0,
                    'confidence_score': 0.95,  # High confidence for service-based events
                    'tags': json.dumps(additional_data) if additional_data else None
                }

            # Store enhanced event in database
            self._insert_movement_event(event_data)

            self.get_logger().info(
                f"Generated ACTIVE {event_type} event for {vehicle_name} from {event_source}"
            )

        except Exception as e:
            self.get_logger().error(f"Error generating enhanced event for {vehicle_name}: {e}")
    
    def mission_status_callback(self, msg: MissionStatus, vehicle_name: str):
        """Process mission status updates."""
        try:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            with self.lock:
                self.vehicle_stats[vehicle_name]['last_seen'] = timestamp
                self.vehicle_stats[vehicle_name]['targets_detected'] = msg.targets_detected
            
            # Store mission statistics using proper DatabaseInterface
            self._insert_mission_status({
                'vehicle_name': vehicle_name,
                'timestamp': timestamp,
                'mission_id': msg.mission_id,
                'progress_percentage': msg.progress_percentage,
                'current_activity': msg.current_activity,
                'waypoints_completed': msg.waypoints_completed,
                'area_covered': msg.area_covered_sq_m,
                'targets_detected': msg.targets_detected,
                'mission_start_time': msg.mission_start_time.sec + msg.mission_start_time.nanosec / 1e9,
                'estimated_remaining_time': msg.estimated_remaining_time.sec + msg.estimated_remaining_time.nanosec / 1e9,
                'mission_status': msg.status
            })
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission status for {vehicle_name}: {e}")
    
    def target_detection_callback(self, msg: TargetDetection, vehicle_name: str):
        """Process target detection events."""
        try:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # Store target detection using proper DatabaseInterface
            self._insert_target_detection({
                'vehicle_name': vehicle_name,
                'timestamp': timestamp,
                'detection_id': msg.detection_id,
                'target_type': msg.target_type,
                'confidence_score': msg.confidence_score,
                'world_position_x': msg.world_position.x,
                'world_position_y': msg.world_position.y,
                'world_position_z': msg.world_position.z,
                'gps_lat': msg.gps_coordinates.x if hasattr(msg, 'gps_coordinates') else None,
                'gps_lon': msg.gps_coordinates.y if hasattr(msg, 'gps_coordinates') else None,
                'gps_alt': msg.gps_coordinates.z if hasattr(msg, 'gps_coordinates') else None,
                'detection_altitude': getattr(msg, 'detection_altitude', None),
                'detection_distance': getattr(msg, 'detection_distance', None),
                'detection_bearing': getattr(msg, 'detection_bearing', None),
                'verified': getattr(msg, 'verified', False),
                'false_positive': getattr(msg, 'false_positive', False),
                'notes': getattr(msg, 'notes', None)
            })
            
            self.get_logger().info(
                f"Logged target detection for {vehicle_name}: "
                f"{msg.target_type} with {msg.confidence_score:.1%} confidence"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing target detection for {vehicle_name}: {e}")
    
    def odometry_callback(self, msg: Odometry, vehicle_name: str):
        """Process high-frequency odometry for precise distance tracking."""
        try:
            current_pos = msg.pose.pose.position
            current_vel = msg.twist.twist.linear
            current_orient = msg.pose.pose.orientation
            current_time = self.get_clock().now().nanoseconds / 1e9

            with self.lock:
                stats = self.vehicle_stats.get(vehicle_name)
                if not stats:
                    return

                # Enhanced movement analysis for active event detection
                self._analyze_movement_patterns(vehicle_name, current_pos, current_vel, current_time)

                if stats['last_position'] is not None:
                    # Calculate distance traveled since last update
                    dx = current_pos.x - stats['last_position'].x
                    dy = current_pos.y - stats['last_position'].y
                    dz = current_pos.z - stats['last_position'].z
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                    # Only count significant movements (filter noise)
                    if distance > 0.1:  # 10cm threshold
                        stats['total_distance'] += distance

                    # Update flight time - ONLY when actually airborne
                    if stats['last_timestamp'] is not None:
                        time_delta = current_time - stats['last_timestamp']
                        if time_delta > 0 and time_delta < 60:  # Sanity check

                            # Check current flight state - only accumulate when airborne
                            current_flight_state = self.flight_states.get(vehicle_name, FlightState.UNKNOWN)
                            is_airborne = current_flight_state in [
                                FlightState.TAKING_OFF,
                                FlightState.AIRBORNE,
                                FlightState.HOVERING,
                                FlightState.LANDING
                            ]

                            if is_airborne:
                                stats['flight_time'] += time_delta
                                self.get_logger().debug(
                                    f"Flight time updated for {vehicle_name}: +{time_delta:.2f}s "
                                    f"(total: {stats['flight_time']:.2f}s, state: {current_flight_state.value})"
                                )
                            else:
                                self.get_logger().debug(
                                    f"Flight time NOT updated for {vehicle_name}: "
                                    f"state={current_flight_state.value} (not airborne)"
                                )
                
                stats['last_position'] = current_pos
                stats['last_timestamp'] = current_time
                stats['last_seen'] = current_time
            
            # Sample odometry data for database (reduced frequency)
            # Only store every ~5th sample to reduce database size
            if hasattr(self, '_odom_sample_counter'):
                self._odom_sample_counter += 1
            else:
                self._odom_sample_counter = 1
            
            if self._odom_sample_counter % 5 == 0:
                self._insert_odometry_sample({
                    'vehicle_name': vehicle_name,
                    'timestamp': current_time,
                    'position_x': current_pos.x,
                    'position_y': current_pos.y,
                    'position_z': current_pos.z,
                    'velocity_x': current_vel.x,
                    'velocity_y': current_vel.y,
                    'velocity_z': current_vel.z,
                    'orientation_w': current_orient.w,
                    'orientation_x': current_orient.x,
                    'orientation_y': current_orient.y,
                    'orientation_z': current_orient.z
                })
                
        except Exception as e:
            self.get_logger().error(f"Error processing odometry for {vehicle_name}: {e}")

    def _analyze_movement_patterns(self, vehicle_name: str, position, velocity, timestamp: float):
        """Analyze movement patterns to enhance event detection."""
        try:
            # Calculate current altitude and speed
            current_altitude = position.z
            current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            # Update history (keep last 50 samples, ~5 seconds at 10Hz)
            if vehicle_name not in self.altitude_history:
                self.altitude_history[vehicle_name] = []
            if vehicle_name not in self.velocity_history:
                self.velocity_history[vehicle_name] = []

            self.altitude_history[vehicle_name].append((timestamp, current_altitude))
            self.velocity_history[vehicle_name].append((timestamp, current_speed))

            # Keep only recent history
            cutoff_time = timestamp - 10.0  # 10 seconds
            self.altitude_history[vehicle_name] = [
                (t, alt) for t, alt in self.altitude_history[vehicle_name] if t > cutoff_time
            ]
            self.velocity_history[vehicle_name] = [
                (t, spd) for t, spd in self.velocity_history[vehicle_name] if t > cutoff_time
            ]

            # Analyze flight state transitions
            self._detect_flight_state_changes(vehicle_name, current_altitude, current_speed, timestamp)

            # Check for pending service calls that may have completed
            self._correlate_service_calls_with_movement(vehicle_name, current_altitude, current_speed, timestamp)

        except Exception as e:
            self.get_logger().error(f"Error analyzing movement patterns for {vehicle_name}: {e}")

    def _detect_flight_state_changes(self, vehicle_name: str, altitude: float, speed: float, timestamp: float):
        """Detect and log flight state changes based on movement analysis."""
        try:
            current_state = self.flight_states.get(vehicle_name, FlightState.UNKNOWN)
            new_state = current_state

            # Analyze recent altitude and speed trends
            alt_history = self.altitude_history.get(vehicle_name, [])
            spd_history = self.velocity_history.get(vehicle_name, [])

            if len(alt_history) < 5:  # Need some history for analysis
                return

            # Calculate trends
            recent_altitudes = [alt for _, alt in alt_history[-5:]]
            recent_speeds = [spd for _, spd in spd_history[-5:]]

            avg_altitude = sum(recent_altitudes) / len(recent_altitudes)
            avg_speed = sum(recent_speeds) / len(recent_speeds)

            altitude_trend = recent_altitudes[-1] - recent_altitudes[0]
            speed_trend = recent_speeds[-1] - recent_speeds[0]

            # State transition logic
            if current_state == FlightState.TAKING_OFF:
                if altitude_trend > 0.5 and avg_speed > 0.5:
                    new_state = FlightState.AIRBORNE
                    self.generate_enhanced_event(vehicle_name, "TAKEOFF_COMPLETED", "MOVEMENT_ANALYSIS", {
                        'altitude_gain': altitude_trend,
                        'average_speed': avg_speed,
                        'analysis_duration': alt_history[-1][0] - alt_history[0][0]
                    })
            elif current_state == FlightState.LANDING:
                if altitude_trend < -0.5 and avg_speed < 1.0:
                    new_state = FlightState.GROUNDED
                    self.generate_enhanced_event(vehicle_name, "LANDING_COMPLETED", "MOVEMENT_ANALYSIS", {
                        'altitude_loss': abs(altitude_trend),
                        'final_speed': avg_speed,
                        'analysis_duration': alt_history[-1][0] - alt_history[0][0]
                    })
            elif current_state in [FlightState.AIRBORNE, FlightState.UNKNOWN]:
                if avg_altitude > 1.0 and avg_speed < 0.3:
                    new_state = FlightState.HOVERING
                elif avg_altitude < 0.5 and avg_speed < 0.3:
                    new_state = FlightState.GROUNDED
                elif avg_altitude > 1.0 and avg_speed > 0.5:
                    new_state = FlightState.AIRBORNE

            # Update state if changed
            if new_state != current_state:
                self.flight_states[vehicle_name] = new_state
                self.get_logger().info(f"Flight state change for {vehicle_name}: {current_state.value} → {new_state.value}")

                # Enhanced logging for flight session tracking
                was_grounded = current_state == FlightState.GROUNDED
                is_now_grounded = new_state == FlightState.GROUNDED

                if was_grounded and not is_now_grounded:
                    # Flight session started
                    self.get_logger().info(f"✈️ Flight session STARTED for {vehicle_name} (altitude: {avg_altitude:.2f}m)")
                elif not was_grounded and is_now_grounded:
                    # Flight session ended - log total flight time
                    with self.lock:
                        total_flight_time = self.vehicle_stats[vehicle_name]['flight_time']
                        total_takeoffs = self.vehicle_stats[vehicle_name]['takeoffs']
                        total_distance = self.vehicle_stats[vehicle_name]['total_distance']

                    self.get_logger().info(
                        f"🛬 Flight session ENDED for {vehicle_name}. "
                        f"Total flight time: {total_flight_time:.2f}s, "
                        f"Total takeoffs: {total_takeoffs}, "
                        f"Total distance: {total_distance:.1f}m"
                    )

        except Exception as e:
            self.get_logger().error(f"Error detecting flight state changes for {vehicle_name}: {e}")

    def _correlate_service_calls_with_movement(self, vehicle_name: str, altitude: float, speed: float, timestamp: float):
        """Correlate pending service calls with observed movement."""
        try:
            pending_calls = self.pending_service_calls.get(vehicle_name, [])
            completed_calls = []

            for call in pending_calls:
                if call.completed:
                    continue

                time_since_call = timestamp - call.timestamp

                # Check if service call has had time to take effect (>2 seconds)
                if time_since_call > 2.0:
                    if call.service_name == "takeoff":
                        # Look for upward movement indicating successful takeoff
                        if altitude > 1.0 and speed > 0.1:
                            call.completed = True
                            call.success = True
                            call.movement_detected = True
                            completed_calls.append(call)

                            self.generate_enhanced_event(vehicle_name, "SERVICE_TAKEOFF_SUCCESS", "CORRELATION_ANALYSIS", {
                                'service_call_id': call.call_id,
                                'time_to_effect': time_since_call,
                                'altitude_achieved': altitude,
                                'speed_achieved': speed
                            })
                    elif call.service_name == "land":
                        # Look for grounded state indicating successful landing
                        if altitude < 0.5 and speed < 0.2:
                            call.completed = True
                            call.success = True
                            call.movement_detected = True
                            completed_calls.append(call)

                            self.generate_enhanced_event(vehicle_name, "SERVICE_LANDING_SUCCESS", "CORRELATION_ANALYSIS", {
                                'service_call_id': call.call_id,
                                'time_to_effect': time_since_call,
                                'final_altitude': altitude,
                                'final_speed': speed
                            })

                # Timeout incomplete calls after 30 seconds
                elif time_since_call > 30.0:
                    call.completed = True
                    call.success = False
                    completed_calls.append(call)

                    self.generate_enhanced_event(vehicle_name, f"SERVICE_{call.service_name.upper()}_TIMEOUT", "CORRELATION_ANALYSIS", {
                        'service_call_id': call.call_id,
                        'timeout_duration': time_since_call
                    })

            # Log completion status
            for call in completed_calls:
                status = "SUCCESS" if call.success else "TIMEOUT"
                self.get_logger().info(f"Service call correlation: {call.service_name} for {vehicle_name} - {status}")

        except Exception as e:
            self.get_logger().error(f"Error correlating service calls for {vehicle_name}: {e}")
    
    def update_vehicle_summary(self, vehicle_name: str):
        """Update vehicle summary statistics in database."""
        try:
            with self.lock:
                stats = self.vehicle_stats.get(vehicle_name)
                if not stats:
                    return
                
                self._upsert_vehicle_summary({
                    'vehicle_name': vehicle_name,
                    'first_seen': stats['first_seen'],
                    'last_seen': stats['last_seen'],
                    'total_distance': stats['total_distance'],
                    'total_flight_time': stats['flight_time'],
                    'total_takeoffs': stats['takeoffs'],
                    'total_landings': stats['landings'],
                    'total_missions': stats['missions_completed'],
                    'total_targets_detected': stats['targets_detected'],
                    'last_update': self.get_clock().now().nanoseconds / 1e9
                })
                
        except Exception as e:
            self.get_logger().error(f"Error updating vehicle summary for {vehicle_name}: {e}")
    
    def log_periodic_stats(self):
        """Log periodic statistics summary and update database."""
        try:
            with self.lock:
                if not self.vehicle_stats:
                    return
                
                # Update all vehicle summaries
                for vehicle_name in self.active_vehicles:
                    self.update_vehicle_summary(vehicle_name)

                # Perform periodic validation (every few cycles to avoid spam)
                if not hasattr(self, '_validation_counter'):
                    self._validation_counter = 0
                self._validation_counter += 1

                # Run validation every 6th cycle (approximately every minute if stats_rate is 10s)
                if self._validation_counter % 6 == 0:
                    self.validate_flight_time_calculation()
                
                # Log current statistics with enhanced debugging
                for vehicle_name, stats in self.vehicle_stats.items():
                    current_flight_state = self.flight_states.get(vehicle_name, FlightState.UNKNOWN)
                    pending_calls = len(self.pending_service_calls.get(vehicle_name, []))

                    # Validation: Check flight time consistency
                    flight_time_ratio = 0.0
                    if stats['takeoffs'] > 0:
                        flight_time_ratio = stats['flight_time'] / stats['takeoffs']

                    # Warning for potential issues
                    warnings = []
                    if stats['takeoffs'] > 0 and stats['flight_time'] == 0:
                        warnings.append("⚠️ Zero flight time despite takeoffs")
                    if stats['takeoffs'] > stats['landings'] + 1:
                        warnings.append("⚠️ More takeoffs than landings")
                    if current_flight_state == FlightState.UNKNOWN and stats['takeoffs'] > 0:
                        warnings.append("⚠️ Unknown flight state after takeoffs")

                    warning_text = f" | {', '.join(warnings)}" if warnings else ""

                    self.get_logger().info(
                        f"Stats [{vehicle_name}]: "
                        f"Distance: {stats['total_distance']:.1f}m, "
                        f"Flight time: {stats['flight_time']:.1f}s, "
                        f"Takeoffs: {stats['takeoffs']}, "
                        f"Landings: {stats['landings']}, "
                        f"Targets: {stats['targets_detected']}, "
                        f"State: {current_flight_state.value}, "
                        f"Pending calls: {pending_calls}, "
                        f"Avg flight/takeoff: {flight_time_ratio:.1f}s"
                        f"{warning_text}"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"Error in periodic stats logging: {e}")

    def validate_flight_time_calculation(self, vehicle_name: str = None) -> Dict[str, Any]:
        """Validate flight time calculation and provide diagnostic information."""
        try:
            validation_report = {
                'timestamp': datetime.now().isoformat(),
                'vehicles': {}
            }

            vehicles_to_check = [vehicle_name] if vehicle_name else list(self.active_vehicles)

            for vname in vehicles_to_check:
                if vname not in self.vehicle_stats:
                    continue

                stats = self.vehicle_stats[vname]
                current_state = self.flight_states.get(vname, FlightState.UNKNOWN)
                pending_calls = self.pending_service_calls.get(vname, [])

                # Calculate expected vs actual flight time
                expected_min_flight_time = max(0, (stats['takeoffs'] - stats['landings']) * 5.0)  # Minimum 5s per flight
                actual_flight_time = stats['flight_time']

                # Check for issues
                issues = []
                if stats['takeoffs'] > 0 and actual_flight_time == 0:
                    issues.append("CRITICAL: Zero flight time despite takeoffs recorded")
                if stats['takeoffs'] > 0 and actual_flight_time < expected_min_flight_time:
                    issues.append(f"WARNING: Flight time ({actual_flight_time:.1f}s) seems low for {stats['takeoffs']} takeoffs")
                if current_state == FlightState.UNKNOWN and stats['takeoffs'] > 0:
                    issues.append("WARNING: Flight state unknown despite flight activity")
                if len(pending_calls) > 3:
                    issues.append(f"WARNING: {len(pending_calls)} pending service calls may indicate issues")

                validation_report['vehicles'][vname] = {
                    'current_flight_state': current_state.value,
                    'total_flight_time_s': actual_flight_time,
                    'takeoffs': stats['takeoffs'],
                    'landings': stats['landings'],
                    'distance_traveled_m': stats['total_distance'],
                    'pending_service_calls': len(pending_calls),
                    'expected_min_flight_time_s': expected_min_flight_time,
                    'flight_time_per_takeoff_s': actual_flight_time / max(1, stats['takeoffs']),
                    'issues': issues,
                    'validation_status': 'PASS' if not issues else 'FAIL'
                }

            # Log validation report
            for vname, report in validation_report['vehicles'].items():
                status = report['validation_status']
                status_emoji = "✅" if status == 'PASS' else "❌"

                self.get_logger().info(
                    f"{status_emoji} Flight Time Validation [{vname}]: {status} - "
                    f"Flight time: {report['total_flight_time_s']:.1f}s, "
                    f"Takeoffs: {report['takeoffs']}, "
                    f"State: {report['current_flight_state']}"
                )

                for issue in report['issues']:
                    self.get_logger().warn(f"  Issue: {issue}")

            return validation_report

        except Exception as e:
            self.get_logger().error(f"Error in flight time validation: {e}")
            return {}

    def generate_summary_report(self) -> Dict[str, Any]:
        """Generate comprehensive summary report."""
        try:
            cursor = self.conn.cursor()
            
            # Get vehicle summaries
            cursor.execute('''
                SELECT vehicle_name, total_distance, total_flight_time,
                       total_takeoffs, total_landings, total_targets_detected
                FROM vehicle_summary
            ''')
            vehicles = cursor.fetchall()
            
            # Get total event counts by type
            cursor.execute('''
                SELECT event_type, COUNT(*) as count, AVG(distance_moved) as avg_distance
                FROM movement_events
                GROUP BY event_type
            ''')
            events = cursor.fetchall()
            
            # Get mission statistics
            cursor.execute('''
                SELECT COUNT(DISTINCT mission_id) as total_missions,
                       AVG(progress_percentage) as avg_progress,
                       SUM(area_covered) as total_area_covered
                FROM mission_stats
                WHERE mission_id IS NOT NULL AND mission_id != ''
            ''')
            mission_stats = cursor.fetchone()
            
            report = {
                'timestamp': datetime.now().isoformat(),
                'active_vehicles': len(self.active_vehicles),
                'vehicles': [
                    {
                        'name': v[0],
                        'total_distance_m': round(v[1], 2),
                        'total_flight_time_s': round(v[2], 1),
                        'takeoffs': v[3],
                        'landings': v[4],
                        'targets_detected': v[5]
                    } for v in vehicles
                ],
                'events_summary': [
                    {
                        'type': e[0],
                        'count': e[1],
                        'avg_distance_m': round(e[2] if e[2] else 0, 2)
                    } for e in events
                ],
                'mission_summary': {
                    'total_missions': mission_stats[0] if mission_stats[0] else 0,
                    'avg_progress_pct': round(mission_stats[1] if mission_stats[1] else 0, 1),
                    'total_area_covered_sqm': round(mission_stats[2] if mission_stats[2] else 0, 2)
                }
            }
            
            return report
            
        except Exception as e:
            self.get_logger().error(f"Error generating summary report: {e}")
            return {}
    
    def export_data(self, filepath: Optional[str] = None):
        """Export collected data to JSON file."""
        if filepath is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"drone_stats_export_{timestamp}.json"
        
        try:
            report = self.generate_summary_report()
            
            with open(filepath, 'w') as f:
                json.dump(report, f, indent=2)
            
            self.get_logger().info(f"Data exported to: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Export failed: {e}")
    
    def cleanup(self):
        """Cleanup resources and close database."""
        try:
            if self.export_on_shutdown:
                self.export_data()
            
            # Final statistics update
            with self.lock:
                for vehicle_name in self.active_vehicles:
                    self.update_vehicle_summary(vehicle_name)
            
            # Close database interface
            if hasattr(self, 'db'):
                self.db.close()
                
            self.get_logger().info("Cleanup completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
    
    # Database helper methods using proper DatabaseInterface
    def _insert_movement_event(self, event_data: dict):
        """Insert movement event into database using proper interface."""
        try:
            if self.db.db_type.value == 'postgresql':
                query = f"""
                INSERT INTO {self.db.config.pg_schema}.movement_events (
                    vehicle_name, timestamp, event_type, event_source, sequence_number,
                    distance_moved, altitude_change, speed_change,
                    position_x, position_y, position_z,
                    previous_x, previous_y, previous_z,
                    velocity_x, velocity_y, velocity_z,
                    mission_id, mission_phase, mission_progress,
                    confidence_score, tags
                ) VALUES (
                    %(vehicle_name)s, %(timestamp)s, %(event_type)s, %(event_source)s, %(sequence_number)s,
                    %(distance_moved)s, %(altitude_change)s, %(speed_change)s,
                    %(position_x)s, %(position_y)s, %(position_z)s,
                    %(previous_x)s, %(previous_y)s, %(previous_z)s,
                    %(velocity_x)s, %(velocity_y)s, %(velocity_z)s,
                    %(mission_id)s, %(mission_phase)s, %(mission_progress)s,
                    %(confidence_score)s, %(tags)s
                )
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, event_data)
            else:
                query = """
                INSERT INTO movement_events (
                    vehicle_name, timestamp, event_type, event_source, sequence_number,
                    distance_moved, altitude_change, speed_change,
                    position_x, position_y, position_z,
                    previous_x, previous_y, previous_z,
                    velocity_x, velocity_y, velocity_z,
                    mission_id, mission_phase, mission_progress,
                    confidence_score, tags
                ) VALUES (
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                )
                """
                params = tuple(event_data.get(key) for key in [
                    'vehicle_name', 'timestamp', 'event_type', 'event_source', 'sequence_number',
                    'distance_moved', 'altitude_change', 'speed_change',
                    'position_x', 'position_y', 'position_z',
                    'previous_x', 'previous_y', 'previous_z',
                    'velocity_x', 'velocity_y', 'velocity_z',
                    'mission_id', 'mission_phase', 'mission_progress',
                    'confidence_score', 'tags'
                ])
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, params)
        except Exception as e:
            self.get_logger().error(f"Failed to insert movement event: {e}")
    
    def _insert_mission_status(self, status_data: dict):
        """Insert mission status into database using proper interface."""
        try:
            if self.db.db_type.value == 'postgresql':
                query = f"""
                INSERT INTO {self.db.config.pg_schema}.mission_events (
                    vehicle_name, timestamp, mission_id, progress_percentage,
                    current_activity, waypoints_completed, area_covered,
                    targets_detected, mission_start_time, estimated_remaining_time,
                    mission_status
                ) VALUES (
                    %(vehicle_name)s, %(timestamp)s, %(mission_id)s, %(progress_percentage)s,
                    %(current_activity)s, %(waypoints_completed)s, %(area_covered)s,
                    %(targets_detected)s, %(mission_start_time)s, %(estimated_remaining_time)s,
                    %(mission_status)s
                )
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, status_data)
            else:
                # For SQLite, we'll use a simplified approach for now
                query = """
                INSERT INTO vehicle_statistics (
                    vehicle_name, targets_detected, waypoints_completed, area_covered_m2, last_updated
                ) VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(vehicle_name) DO UPDATE SET
                    targets_detected = excluded.targets_detected,
                    waypoints_completed = excluded.waypoints_completed,
                    area_covered_m2 = excluded.area_covered_m2,
                    last_updated = excluded.last_updated
                """
                params = (
                    status_data['vehicle_name'],
                    status_data['targets_detected'],
                    status_data['waypoints_completed'], 
                    status_data['area_covered'],
                    status_data['timestamp']
                )
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, params)
        except Exception as e:
            self.get_logger().error(f"Failed to insert mission status: {e}")
    
    def _insert_target_detection(self, detection_data: dict):
        """Insert target detection into database using proper interface."""
        try:
            if self.db.db_type.value == 'postgresql':
                query = f"""
                INSERT INTO {self.db.config.pg_schema}.target_detections (
                    vehicle_name, timestamp, detection_id, target_type,
                    confidence_score, world_position_x, world_position_y, world_position_z,
                    gps_lat, gps_lon, gps_alt, detection_altitude, detection_distance,
                    detection_bearing, verified, false_positive, notes
                ) VALUES (
                    %(vehicle_name)s, %(timestamp)s, %(detection_id)s, %(target_type)s,
                    %(confidence_score)s, %(world_position_x)s, %(world_position_y)s, %(world_position_z)s,
                    %(gps_lat)s, %(gps_lon)s, %(gps_alt)s, %(detection_altitude)s, %(detection_distance)s,
                    %(detection_bearing)s, %(verified)s, %(false_positive)s, %(notes)s
                )
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, detection_data)
            else:
                # For SQLite, create a simplified table entry for now
                query = """
                UPDATE vehicle_statistics 
                SET targets_detected = targets_detected + 1, last_updated = ?
                WHERE vehicle_name = ?
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, (detection_data['timestamp'], detection_data['vehicle_name']))
        except Exception as e:
            self.get_logger().error(f"Failed to insert target detection: {e}")
    
    def _insert_odometry_sample(self, odom_data: dict):
        """Insert odometry sample into database using proper interface."""
        try:
            if self.db.db_type.value == 'postgresql':
                query = f"""
                INSERT INTO {self.db.config.pg_schema}.position_history (
                    vehicle_name, timestamp, position_x, position_y, position_z,
                    velocity_x, velocity_y, velocity_z,
                    orientation_w, orientation_x, orientation_y, orientation_z
                ) VALUES (
                    %(vehicle_name)s, %(timestamp)s, %(position_x)s, %(position_y)s, %(position_z)s,
                    %(velocity_x)s, %(velocity_y)s, %(velocity_z)s,
                    %(orientation_w)s, %(orientation_x)s, %(orientation_y)s, %(orientation_z)s
                )
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, odom_data)
            # For SQLite, skip individual odometry samples to keep database light
        except Exception as e:
            self.get_logger().error(f"Failed to insert odometry sample: {e}")
    
    def _upsert_vehicle_summary(self, summary_data: dict):
        """Update or insert vehicle summary statistics."""
        try:
            if self.db.db_type.value == 'postgresql':
                query = f"""
                INSERT INTO {self.db.config.pg_schema}.vehicle_statistics (
                    vehicle_name, total_distance_m, flight_time_s, events_count,
                    targets_detected, waypoints_completed, area_covered_m2, last_updated
                ) VALUES (
                    %(vehicle_name)s, %(total_distance)s, %(total_flight_time)s, 0,
                    %(total_targets_detected)s, %(total_missions)s, 0, %(last_update)s
                )
                ON CONFLICT (vehicle_name) DO UPDATE SET
                    total_distance_m = EXCLUDED.total_distance_m,
                    flight_time_s = EXCLUDED.flight_time_s,
                    targets_detected = EXCLUDED.targets_detected,
                    waypoints_completed = EXCLUDED.waypoints_completed,
                    last_updated = EXCLUDED.last_updated
                """
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, summary_data)
            else:
                query = """
                INSERT OR REPLACE INTO vehicle_statistics (
                    vehicle_name, total_distance_m, flight_time_s,
                    targets_detected, waypoints_completed, last_updated
                ) VALUES (?, ?, ?, ?, ?, ?)
                """
                params = (
                    summary_data['vehicle_name'],
                    summary_data['total_distance'],
                    summary_data['total_flight_time'],
                    summary_data['total_targets_detected'],
                    summary_data['total_missions'],
                    summary_data['last_update']
                )
                with self.db.get_connection() as conn:
                    cursor = conn.cursor()
                    cursor.execute(query, params)
        except Exception as e:
            self.get_logger().error(f"Failed to upsert vehicle summary: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        # Create node with multi-threaded executor for better performance
        stats_logger = DroneMovementStatsLogger()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(stats_logger)
        
        stats_logger.get_logger().info("Drone Movement Statistics Logger started")
        stats_logger.get_logger().info("Press Ctrl+C to stop and generate final report")
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            stats_logger.get_logger().info("Shutdown requested by user")
        
        # Generate final report
        final_report = stats_logger.generate_summary_report()
        print("\n" + "="*60)
        print("FINAL MOVEMENT STATISTICS REPORT")
        print("="*60)
        print(f"Report generated: {final_report.get('timestamp', 'N/A')}")
        print(f"Active vehicles tracked: {final_report.get('active_vehicles', 0)}")
        
        if final_report.get('vehicles'):
            print(f"\nVehicle Statistics:")
            for vehicle in final_report['vehicles']:
                print(f"  {vehicle['name']}: "
                      f"{vehicle['total_distance_m']:.1f}m traveled, "
                      f"{vehicle['total_flight_time_s']:.1f}s flight time, "
                      f"{vehicle['takeoffs']} takeoffs, "
                      f"{vehicle['targets_detected']} targets detected")
        
        if final_report.get('events_summary'):
            print(f"\nMovement Events:")
            for event in final_report['events_summary']:
                print(f"  {event['type']}: {event['count']} events, "
                      f"avg {event['avg_distance_m']:.1f}m per event")
        
        mission_summary = final_report.get('mission_summary', {})
        print(f"\nMission Summary:")
        print(f"  Total missions: {mission_summary.get('total_missions', 0)}")
        print(f"  Average progress: {mission_summary.get('avg_progress_pct', 0):.1f}%")
        print(f"  Total area covered: {mission_summary.get('total_area_covered_sqm', 0):.1f} sq.m")
        
        print("\n" + "="*60)
        
    except Exception as e:
        print(f"Fatal error: {e}")
        return 1
    
    finally:
        if 'stats_logger' in locals():
            stats_logger.cleanup()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    exit(main())