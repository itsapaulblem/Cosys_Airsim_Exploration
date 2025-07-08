#!/usr/bin/env python3

"""
Waypoint Navigator Module

Handles waypoint-by-waypoint navigation for autonomous missions.
Provides precise position control and mission progress tracking.

Author: Cosys-Lab
License: MIT
"""

import asyncio
import math
import time
from typing import List, Optional
from enum import Enum

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Header

from airsim_mission_interfaces.msg import (
    Waypoint, WaypointArray, CurrentWaypoint, SearchStats, MissionProgress
)
from airsim_interfaces.srv import SetLocalPosition
from airsim_interfaces.msg import VelCmd


class NavigationState(Enum):
    """Navigation states for waypoint execution"""
    IDLE = 0
    NAVIGATING = 1
    HOVERING = 2
    COMPLETED = 3
    FAILED = 4


class WaypointNavigator:
    """
    Handles precise waypoint navigation for autonomous missions.
    
    Features:
    - Position-based navigation with velocity control fallback
    - Waypoint acceptance radius checking
    - Hover time management
    - Progress tracking and reporting
    """
    
    def __init__(self, grid_search_server):
        self.server = grid_search_server
        self.node = grid_search_server
        self.logger = grid_search_server.get_logger()
        
        # Navigation parameters
        self.position_tolerance = self.node.get_parameter('position_tolerance').value
        self.waypoint_timeout = self.node.get_parameter('timeout_waypoint').value
        self.navigation_frequency = 10.0  # Hz
        
        # Navigation state
        self.current_state = NavigationState.IDLE
        self.current_waypoint_index = 0
        self.mission_waypoints: Optional[WaypointArray] = None
        self.navigation_start_time = 0.0
        self.waypoint_start_time = 0.0
        
        # Position control
        self.target_position: Optional[Point] = None
        self.current_position: Optional[Point] = None
        self.position_controller = SimplePositionController(self.node)
        
    async def execute_waypoints(self, goal_handle, waypoints: WaypointArray, 
                              mission_stats: SearchStats) -> bool:
        """
        Execute a complete waypoint mission.
        
        Args:
            goal_handle: Action goal handle for feedback
            waypoints: Array of waypoints to navigate
            mission_stats: Statistics object to update
            
        Returns:
            bool: True if mission completed successfully
        """
        self.logger.info(f"Starting waypoint navigation with {len(waypoints.waypoints)} waypoints")
        
        self.mission_waypoints = waypoints
        self.current_waypoint_index = 0
        self.navigation_start_time = time.time()
        
        mission_stats.total_waypoints = len(waypoints.waypoints)
        mission_stats.waypoints_completed = 0
        
        try:
            # Execute each waypoint in sequence
            for i, waypoint in enumerate(waypoints.waypoints):
                self.current_waypoint_index = i
                
                # Update current waypoint feedback
                self._publish_current_waypoint_feedback(goal_handle, waypoint)
                
                # Navigate to waypoint
                success = await self._navigate_to_waypoint(waypoint, goal_handle)
                
                if not success:
                    self.logger.error(f"Failed to reach waypoint {i}: {waypoint.waypoint_id}")
                    mission_stats.navigation_errors += 1
                    
                    # Decide whether to continue or abort
                    if not self._should_continue_after_failure(waypoint):
                        return False
                else:
                    mission_stats.waypoints_completed += 1
                    
                # Check for mission cancellation
                if goal_handle.is_cancel_requested:
                    self.logger.info("Mission cancellation requested")
                    return False
                    
                # Update progress
                completion = (i + 1) / len(waypoints.waypoints)
                self._publish_progress_feedback(goal_handle, completion)
                
            self.logger.info("Waypoint navigation completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Waypoint navigation failed: {str(e)}")
            return False
        
        finally:
            self.current_state = NavigationState.IDLE
    
    async def _navigate_to_waypoint(self, waypoint: Waypoint, goal_handle) -> bool:
        """
        Navigate to a single waypoint.
        
        Args:
            waypoint: Target waypoint
            goal_handle: Action goal handle for feedback
            
        Returns:
            bool: True if waypoint reached successfully
        """
        self.logger.info(f"Navigating to waypoint {waypoint.waypoint_id} at "
                        f"({waypoint.position.x:.1f}, {waypoint.position.y:.1f}, {waypoint.position.z:.1f})")
        
        self.current_state = NavigationState.NAVIGATING
        self.target_position = waypoint.position
        self.waypoint_start_time = time.time()
        
        # Set navigation parameters
        acceptance_radius = waypoint.acceptance_radius if waypoint.acceptance_radius > 0 else self.position_tolerance
        max_time = waypoint.max_time_at_waypoint if waypoint.max_time_at_waypoint > 0 else self.waypoint_timeout
        
        try:
            # Use position control service if available, otherwise use velocity control
            if await self._try_position_control(waypoint):
                # Wait for arrival using position service
                arrived = await self._wait_for_arrival_position_service(acceptance_radius, max_time)
            else:
                # Use velocity control
                arrived = await self._navigate_with_velocity_control(waypoint, acceptance_radius, max_time, goal_handle)
                
            if not arrived:
                self.logger.warn(f"Timeout reaching waypoint {waypoint.waypoint_id}")
                return False
            
            # Handle hover time
            if waypoint.hover_time > 0:
                await self._hover_at_waypoint(waypoint.hover_time)
            
            # Handle data collection
            if waypoint.capture_photo or waypoint.record_sensors:
                await self._handle_data_collection(waypoint)
            
            self.logger.info(f"Successfully reached waypoint {waypoint.waypoint_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Navigation to waypoint {waypoint.waypoint_id} failed: {str(e)}")
            return False
    
    async def _try_position_control(self, waypoint: Waypoint) -> bool:
        """Try to use AirSim position control service"""
        if not hasattr(self.server, 'position_client'):
            return False
            
        if not self.server.position_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("Position control service not available, using velocity control")
            return False
        
        try:
            request = SetLocalPosition.Request()
            request.x = waypoint.position.x
            request.y = waypoint.position.y
            request.z = waypoint.position.z
            request.yaw = waypoint.yaw
            request.vehicle_name = self.server.vehicle_name
            request.wait_on_last_task = False  # Don't block
            
            future = self.server.position_client.call_async(request)
            # Add timeout for async call
            response = await asyncio.wait_for(future, timeout=10.0)
            
            success = response.success if hasattr(response, 'success') else True
            if not success and hasattr(response, 'message'):
                self.logger.warn(f"Position control service failed: {response.message}")
            return success
            
        except asyncio.TimeoutError:
            self.logger.warn("Position control service timed out, falling back to velocity control")
            return False
        except Exception as e:
            self.logger.warn(f"Position control failed, falling back to velocity: {str(e)}")
            return False
    
    async def _wait_for_arrival_position_service(self, acceptance_radius: float, max_time: float) -> bool:
        """Wait for arrival when using position control service"""
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            # Check if we're close enough
            distance = self._get_distance_to_target()
            if distance is not None and distance <= acceptance_radius:
                return True
                
            await asyncio.sleep(0.1)  # 10 Hz check rate
            
        return False
    
    async def _navigate_with_velocity_control(self, waypoint: Waypoint, acceptance_radius: float,
                                            max_time: float, goal_handle) -> bool:
        """Navigate using velocity control commands"""
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            # Get current position
            current_pos = self._get_current_position()
            if current_pos is None:
                await asyncio.sleep(0.1)
                continue
            
            # Calculate distance to target
            distance = self._calculate_distance(current_pos, waypoint.position)
            
            if distance <= acceptance_radius:
                # Arrived at waypoint
                self._send_stop_command()
                return True
            
            # Calculate velocity command
            vel_cmd = self.position_controller.calculate_velocity_command(
                current_pos, waypoint.position, waypoint.speed_ms
            )
            
            # Send velocity command
            self._send_velocity_command(vel_cmd)
            
            # Update feedback
            self._publish_navigation_feedback(goal_handle, current_pos, waypoint.position, distance)
            
            await asyncio.sleep(1.0 / self.navigation_frequency)
        
        # Timeout - stop the vehicle
        self._send_stop_command()
        return False
    
    async def _hover_at_waypoint(self, hover_time: float):
        """Hover at waypoint for specified time"""
        self.logger.info(f"Hovering for {hover_time:.1f} seconds")
        self.current_state = NavigationState.HOVERING
        
        # Send stop command to maintain position
        self._send_stop_command()
        
        await asyncio.sleep(hover_time)
    
    async def _handle_data_collection(self, waypoint: Waypoint):
        """Handle data collection at waypoint"""
        if waypoint.capture_photo:
            self.logger.info(f"Capturing photo at waypoint {waypoint.waypoint_id}")
            # TODO: Implement photo capture
            
        if waypoint.record_sensors:
            self.logger.info(f"Recording sensor data at waypoint {waypoint.waypoint_id}")
            # TODO: Implement sensor data recording
    
    def _get_current_position(self) -> Optional[Point]:
        """Get current vehicle position from mission data"""
        with self.server.mission_lock:
            if 'current_position' in self.server.mission_data:
                return self.server.mission_data['current_position']
        return None
    
    def _get_distance_to_target(self) -> Optional[float]:
        """Get distance to current target position"""
        current_pos = self._get_current_position()
        if current_pos is None or self.target_position is None:
            return None
        return self._calculate_distance(current_pos, self.target_position)
    
    def _calculate_distance(self, pos1: Point, pos2: Point) -> float:
        """Calculate 3D distance between two points"""
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        dz = pos2.z - pos1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def _send_velocity_command(self, velocity: Twist):
        """Send velocity command to vehicle"""
        if hasattr(self.server, 'vel_cmd_publisher'):
            vel_msg = VelCmd()
            vel_msg.twist = velocity
            self.server.vel_cmd_publisher.publish(vel_msg)
    
    def _send_stop_command(self):
        """Send stop command to vehicle"""
        stop_vel = Twist()
        # All velocities are zero by default
        self._send_velocity_command(stop_vel)
    
    def _should_continue_after_failure(self, waypoint: Waypoint) -> bool:
        """Decide whether to continue mission after waypoint failure"""
        # For now, continue with mission unless it's a critical waypoint
        if waypoint.waypoint_type == Waypoint.TYPE_LAND:
            return False  # Landing waypoint failure is critical
        return True
    
    def _publish_current_waypoint_feedback(self, goal_handle, waypoint: Waypoint):
        """Publish current waypoint feedback"""
        current_wp_msg = CurrentWaypoint()
        current_wp_msg.header.stamp = self.node.get_clock().now().to_msg()
        current_wp_msg.current_waypoint = waypoint
        current_wp_msg.waypoint_index = self.current_waypoint_index
        current_wp_msg.total_waypoints = len(self.mission_waypoints.waypoints) if self.mission_waypoints else 0
        
        # Calculate distance and time to waypoint
        current_pos = self._get_current_position()
        if current_pos:
            current_wp_msg.distance_to_waypoint = self._calculate_distance(current_pos, waypoint.position)
            # Estimate time based on speed
            if waypoint.speed_ms > 0:
                current_wp_msg.time_to_waypoint = current_wp_msg.distance_to_waypoint / waypoint.speed_ms
        
        # Update goal feedback
        feedback = self.server.current_goal_handle.feedback if hasattr(self.server, 'current_goal_handle') else None
        if feedback:
            feedback.current_waypoint = current_wp_msg
            goal_handle.publish_feedback(feedback)
    
    def _publish_navigation_feedback(self, goal_handle, current_pos: Point, target_pos: Point, distance: float):
        """Publish navigation progress feedback"""
        if not hasattr(self.server, 'current_goal_handle'):
            return
            
        # Update mission progress
        progress = MissionProgress()
        progress.header.stamp = self.node.get_clock().now().to_msg()
        progress.current_position = current_pos
        
        # Calculate progress percentage for current waypoint
        if self.mission_waypoints and self.current_waypoint_index < len(self.mission_waypoints.waypoints):
            waypoint_progress = min(1.0, max(0.0, 1.0 - distance / 10.0))  # Assume 10m approach distance
            overall_progress = (self.current_waypoint_index + waypoint_progress) / len(self.mission_waypoints.waypoints)
            progress.completion_percentage = overall_progress * 100.0
    
    def _publish_progress_feedback(self, goal_handle, completion_percentage: float):
        """Publish overall mission progress"""
        if hasattr(self.server, 'publish_feedback'):
            self.server.publish_feedback(goal_handle, f"Progress: {completion_percentage*100:.1f}%")


class SimplePositionController:
    """
    Simple position controller for velocity-based navigation.
    
    Uses proportional control with velocity limiting.
    """
    
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        # Control gains
        self.kp_xy = 1.0      # Proportional gain for XY
        self.kp_z = 0.8       # Proportional gain for Z
        self.max_vel_xy = 5.0 # Maximum XY velocity (m/s)
        self.max_vel_z = 2.0  # Maximum Z velocity (m/s)
        
    def calculate_velocity_command(self, current_pos: Point, target_pos: Point, 
                                 target_speed: float) -> Twist:
        """
        Calculate velocity command to reach target position.
        
        Args:
            current_pos: Current position
            target_pos: Target position
            target_speed: Desired speed (used to scale velocities)
            
        Returns:
            Twist: Velocity command
        """
        # Calculate position errors
        error_x = target_pos.x - current_pos.x
        error_y = target_pos.y - current_pos.y
        error_z = target_pos.z - current_pos.z
        
        # Calculate desired velocities
        vel_x = self.kp_xy * error_x
        vel_y = self.kp_xy * error_y
        vel_z = self.kp_z * error_z
        
        # Apply velocity limits
        vel_xy_mag = math.sqrt(vel_x*vel_x + vel_y*vel_y)
        if vel_xy_mag > self.max_vel_xy:
            scale = self.max_vel_xy / vel_xy_mag
            vel_x *= scale
            vel_y *= scale
        
        vel_z = max(-self.max_vel_z, min(self.max_vel_z, vel_z))
        
        # Scale by target speed if specified
        if target_speed > 0:
            speed_scale = min(1.0, target_speed / 5.0)  # Assume 5 m/s nominal speed
            vel_x *= speed_scale
            vel_y *= speed_scale
            vel_z *= speed_scale
        
        # Create velocity command
        vel_cmd = Twist()
        vel_cmd.linear.x = vel_x
        vel_cmd.linear.y = vel_y
        vel_cmd.linear.z = vel_z
        # Angular velocities remain zero for simple position control
        
        return vel_cmd