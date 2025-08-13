#!/usr/bin/env python3

"""
Grid Search Mission Action Server

This is the main action server that handles grid search missions for AirSim drones.
It manages mission planning, execution, monitoring, and safety checks.

Author: Cosys-Lab
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
from enum import Enum
from typing import Optional, List, Dict, Any

# ROS2 message imports
from geometry_msgs.msg import Point, Twist, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState

# AirSim interface imports
from airsim_interfaces.srv import Takeoff, Land, SetLocalPosition
from airsim_interfaces.msg import VelCmd

# Mission interface imports
from airsim_mission_interfaces.action import GridSearch
from airsim_mission_interfaces.msg import (
    MissionProgress, CurrentWaypoint, SearchStats, GridSearchStatus,
    Waypoint, WaypointArray
)
from airsim_mission_interfaces.srv import GenerateGrid, UpdateMission, GetMissionStatus

# Local imports
from .grid_generator import GridGenerator
from .safety_monitor import SafetyMonitor
from .waypoint_navigator import WaypointNavigator


class MissionState(Enum):
    """Mission execution states"""
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    PAUSED = 3
    COMPLETED = 4
    ABORTED = 5
    EMERGENCY = 6
    RETURNING_HOME = 7


class GridSearchServer(Node):
    """
    Main grid search action server.
    
    Handles grid search mission requests, coordinates between components,
    and provides real-time feedback on mission progress.
    """

    def __init__(self):
        super().__init__('grid_search_server')
        
        # Initialize parameters
        self.declare_parameters()
        
        # Mission state management
        self.mission_state = MissionState.IDLE
        self.current_mission_id: Optional[str] = None
        self.current_goal_handle = None
        self.mission_data: Dict[str, Any] = {}
        self.mission_lock = threading.Lock()
        
        # Components
        self.grid_generator = GridGenerator(self)
        self.safety_monitor = SafetyMonitor(self)
        self.waypoint_navigator = WaypointNavigator(self)
        
        # Callback groups for concurrent execution
        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        
        # Action server
        self._action_server = ActionServer(
            self,
            GridSearch,
            'grid_search',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        # Service servers
        self.generate_grid_service = self.create_service(
            GenerateGrid,
            'generate_grid',
            self.generate_grid_callback,
            callback_group=self.service_callback_group
        )
        
        self.update_mission_service = self.create_service(
            UpdateMission,
            'update_mission',
            self.update_mission_callback,
            callback_group=self.service_callback_group
        )
        
        self.get_status_service = self.create_service(
            GetMissionStatus,
            'get_mission_status',
            self.get_mission_status_callback,
            callback_group=self.service_callback_group
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(
            GridSearchStatus,
            'grid_search_status',
            10
        )
        
        # Subscribers (will be set up based on vehicle name)
        self.odom_subscriber = None
        self.gps_subscriber = None
        self.battery_subscriber = None
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info("Grid Search Server initialized successfully")

    def declare_parameters(self):
        """Declare ROS2 parameters with default values"""
        self.declare_parameter('vehicle_name', 'drone_1')
        self.declare_parameter('max_concurrent_missions', 1)
        self.declare_parameter('default_takeoff_altitude', 10.0)
        self.declare_parameter('default_flight_speed', 5.0)
        self.declare_parameter('safety_check_frequency', 2.0)  # Hz
        self.declare_parameter('position_tolerance', 2.0)  # meters
        self.declare_parameter('timeout_waypoint', 30.0)  # seconds
        
    def setup_vehicle_interface(self, vehicle_name: str):
        """Set up vehicle-specific subscribers and service clients"""
        self.vehicle_name = vehicle_name
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            f'/airsim_node/{vehicle_name}/odom_local_ned',
            self.odom_callback,
            10
        )
        
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            f'/airsim_node/{vehicle_name}/global_gps',
            self.gps_callback,
            10
        )
        
        # Service clients for vehicle control
        self.takeoff_client = self.create_client(
            Takeoff,
            f'/airsim_node/{vehicle_name}/takeoff'
        )
        
        self.land_client = self.create_client(
            Land,
            f'/airsim_node/{vehicle_name}/land'
        )
        
        self.position_client = self.create_client(
            SetLocalPosition,
            f'/airsim_node/{vehicle_name}/set_local_position'
        )
        
        # Publishers for vehicle control
        self.vel_cmd_publisher = self.create_publisher(
            VelCmd,
            f'/airsim_node/{vehicle_name}/vel_cmd_body_frame',
            10
        )
        
        self.get_logger().info(f"Vehicle interface set up for {vehicle_name}")

    # Callback methods
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        with self.mission_lock:
            self.mission_data['current_position'] = msg.pose.pose.position
            self.mission_data['current_velocity'] = msg.twist.twist.linear
            self.mission_data['current_orientation'] = msg.pose.pose.orientation
            
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS updates"""
        with self.mission_lock:
            self.mission_data['current_gps'] = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude
            }
            self.mission_data['gps_accuracy'] = msg.position_covariance[0]

    # Action server callbacks
    def goal_callback(self, goal_request) -> GoalResponse:
        """Handle new goal requests"""
        self.get_logger().info(f"Received grid search goal: {goal_request.mission_id}")
        
        # Check if we can accept new goals
        if self.mission_state != MissionState.IDLE:
            self.get_logger().warn(f"Rejecting goal - current state: {self.mission_state}")
            return GoalResponse.REJECT
            
        # Validate goal parameters
        if not self.validate_goal(goal_request):
            self.get_logger().error("Goal validation failed")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle goal cancellation requests"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Main mission execution callback"""
        self.get_logger().info(f"Executing grid search mission: {goal_handle.request.mission_id}")
        
        self.current_goal_handle = goal_handle
        self.current_mission_id = goal_handle.request.mission_id
        
        try:
            # Set up vehicle interface
            self.setup_vehicle_interface(goal_handle.request.vehicle_name)
            
            # Execute mission phases
            result = await self.execute_mission(goal_handle)
            
            # Mark goal as succeeded or failed
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                
            return result
            
        except Exception as e:
            self.get_logger().error(f"Mission execution failed: {str(e)}")
            goal_handle.abort()
            result = GridSearch.Result()
            result.success = False
            result.result_message = f"Mission failed: {str(e)}"
            return result
        
        finally:
            # Clean up
            self.cleanup_mission()

    async def execute_mission(self, goal_handle) -> GridSearch.Result:
        """Execute the complete grid search mission"""
        goal = goal_handle.request
        result = GridSearch.Result()
        
        try:
            # Phase 1: Mission Planning
            self.mission_state = MissionState.PLANNING
            self.publish_feedback(goal_handle, "Planning mission...")
            
            waypoints = await self.plan_mission(goal)
            if not waypoints:
                raise Exception("Failed to generate waypoints")
                
            # Phase 2: Pre-flight checks
            self.publish_feedback(goal_handle, "Performing pre-flight checks...")
            if not await self.preflight_checks(goal):
                raise Exception("Pre-flight checks failed")
                
            # Phase 3: Takeoff
            if goal.auto_start:
                self.publish_feedback(goal_handle, "Taking off...")
                if not await self.takeoff():
                    raise Exception("Takeoff failed")
                    
            # Phase 4: Mission execution
            self.mission_state = MissionState.EXECUTING
            self.publish_feedback(goal_handle, "Executing mission...")
            
            mission_stats = await self.execute_waypoint_mission(goal_handle, waypoints)
            
            # Phase 5: Return home (if requested)
            if goal.return_home_on_completion:
                self.publish_feedback(goal_handle, "Returning home...")
                await self.return_home()
                
            # Phase 6: Landing
            self.publish_feedback(goal_handle, "Landing...")
            await self.land()
            
            # Mission completed successfully
            self.mission_state = MissionState.COMPLETED
            result.success = True
            result.result_message = "Mission completed successfully"
            result.mission_stats = mission_stats
            
        except Exception as e:
            self.get_logger().error(f"Mission execution error: {str(e)}")
            self.mission_state = MissionState.ABORTED
            result.success = False
            result.result_message = str(e)
            
        return result

    def publish_feedback(self, goal_handle, status_message: str):
        """Publish action feedback"""
        feedback = GridSearch.Feedback()
        
        # Create progress message
        progress = MissionProgress()
        progress.header.stamp = self.get_clock().now().to_msg()
        progress.mission_id = self.current_mission_id or ""
        progress.mission_state = self.mission_state.value
        progress.status_message = status_message
        
        # Add current position if available
        with self.mission_lock:
            if 'current_position' in self.mission_data:
                progress.current_position = self.mission_data['current_position']
            if 'battery_percentage' in self.mission_data:
                progress.current_battery_percentage = self.mission_data['battery_percentage']
                
        feedback.progress = progress
        feedback.status_message = status_message
        
        goal_handle.publish_feedback(feedback)

    def publish_status(self):
        """Publish current mission status"""
        status = GridSearchStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mission_id = self.current_mission_id or ""
        
        # Create progress message
        progress = MissionProgress()
        progress.header.stamp = self.get_clock().now().to_msg()
        progress.mission_id = self.current_mission_id or ""
        progress.mission_state = self.mission_state.value
        
        with self.mission_lock:
            if 'current_position' in self.mission_data:
                progress.current_position = self.mission_data['current_position']
                status.current_position = self.mission_data['current_position']
            if 'current_velocity' in self.mission_data:
                status.current_velocity = Twist()
                status.current_velocity.linear = self.mission_data['current_velocity']
                
        status.progress = progress
        self.status_publisher.publish(status)

    def validate_goal(self, goal) -> bool:
        """Validate mission goal parameters"""
        # Basic validation - can be extended
        if not goal.mission_id:
            self.get_logger().error("Mission ID is required")
            return False
            
        if not goal.vehicle_name:
            self.get_logger().error("Vehicle name is required")
            return False
            
        # Validate search area
        search_area = goal.search_area
        if search_area.coordinate_type == search_area.COORDINATE_TYPE_GPS:
            if not (search_area.north_lat and search_area.south_lat and 
                   search_area.east_lon and search_area.west_lon):
                self.get_logger().error("Invalid GPS coordinates in search area")
                return False
        
        return True

    async def plan_mission(self, goal) -> Optional[WaypointArray]:
        """Plan the mission and generate waypoints"""
        try:
            waypoints = self.grid_generator.generate_grid(
                goal.search_area,
                goal.grid_params
            )
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Mission planning failed: {str(e)}")
            return None

    async def preflight_checks(self, goal) -> bool:
        """Perform pre-flight safety checks"""
        return self.safety_monitor.perform_preflight_checks(goal.safety_params)

    async def takeoff(self) -> bool:
        """Execute takeoff procedure"""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Takeoff service not available")
            return False
            
        request = Takeoff.Request()
        request.wait_on_last_task = True
        
        try:
            future = self.takeoff_client.call_async(request)
            # Add timeout for async call
            response = await asyncio.wait_for(future, timeout=30.0)
            success = response.success if hasattr(response, 'success') else True
            if not success:
                self.get_logger().error("Takeoff service returned failure")
            return success
        except asyncio.TimeoutError:
            self.get_logger().error("Takeoff service call timed out")
            return False
        except Exception as e:
            self.get_logger().error(f"Takeoff failed: {str(e)}")
            return False

    async def land(self) -> bool:
        """Execute landing procedure"""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Land service not available")
            return False
            
        request = Land.Request()
        request.wait_on_last_task = True
        
        try:
            future = self.land_client.call_async(request)
            # Add timeout for async call
            response = await asyncio.wait_for(future, timeout=60.0)  # Landing can take longer
            success = response.success if hasattr(response, 'success') else True
            if not success:
                self.get_logger().error("Land service returned failure")
            return success
        except asyncio.TimeoutError:
            self.get_logger().error("Land service call timed out")
            return False
        except Exception as e:
            self.get_logger().error(f"Landing failed: {str(e)}")
            return False

    async def execute_waypoint_mission(self, goal_handle, waypoints: WaypointArray) -> SearchStats:
        """Execute the waypoint navigation mission"""
        stats = SearchStats()
        stats.header.stamp = self.get_clock().now().to_msg()
        stats.mission_id = self.current_mission_id
        stats.mission_start_time = self.get_clock().now().to_msg()
        
        # Use waypoint navigator to execute mission
        success = await self.waypoint_navigator.execute_waypoints(
            goal_handle, waypoints, stats
        )
        
        stats.mission_end_time = self.get_clock().now().to_msg()
        return stats

    async def return_home(self) -> bool:
        """Return to home position"""
        # Implementation depends on how home position is stored
        self.get_logger().info("Returning to home position")
        return True

    def cleanup_mission(self):
        """Clean up after mission completion"""
        with self.mission_lock:
            self.mission_state = MissionState.IDLE
            self.current_mission_id = None
            self.current_goal_handle = None
            self.mission_data.clear()

    # Service callbacks
    def generate_grid_callback(self, request, response):
        """Handle grid generation service requests"""
        try:
            waypoints = self.grid_generator.generate_grid(
                request.search_area,
                request.grid_params
            )
            
            response.success = True
            response.generated_waypoints = waypoints
            response.message = "Grid generated successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Grid generation failed: {str(e)}"
            
        return response

    def update_mission_callback(self, request, response):
        """Handle mission update requests"""
        # Implementation for dynamic mission updates
        response.success = True
        response.message = "Mission update not yet implemented"
        return response

    def get_mission_status_callback(self, request, response):
        """Handle mission status requests"""
        response.success = True
        response.mission_active = (self.mission_state != MissionState.IDLE)
        
        # Create current status
        status = GridSearchStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mission_id = self.current_mission_id or ""
        
        response.mission_status = status
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    grid_search_server = GridSearchServer()
    
    # Use MultiThreadedExecutor for concurrent callback processing
    executor = MultiThreadedExecutor()
    executor.add_node(grid_search_server)
    
    try:
        grid_search_server.get_logger().info("Grid Search Server is running...")
        executor.spin()
    except KeyboardInterrupt:
        grid_search_server.get_logger().info("Grid Search Server shutting down...")
    finally:
        grid_search_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()