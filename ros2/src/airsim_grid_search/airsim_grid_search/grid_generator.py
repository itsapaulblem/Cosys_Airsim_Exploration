#!/usr/bin/env python3

"""
Grid Generator Module

Generates waypoint grids for search missions using various patterns:
- Boustrophedon (lawnmower) pattern
- Spiral pattern (future implementation)
- Adaptive pattern (future implementation)

Author: Cosys-Lab
License: MIT
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from geometry_msgs.msg import Point
from std_msgs.msg import Header

from airsim_mission_interfaces.msg import (
    SearchArea, GridParams, Waypoint, WaypointArray
)


@dataclass
class GridBounds:
    """Grid boundary definition in local coordinates"""
    north: float
    south: float
    east: float
    west: float
    min_alt: float
    max_alt: float
    default_alt: float


class GridGenerator:
    """
    Generates waypoint grids for autonomous search missions.
    
    Supports multiple search patterns and coordinate systems.
    """
    
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        # GPS conversion parameters (simplified - real implementation would use geodetic libs)
        self.earth_radius = 6371000.0  # meters
        
    def generate_grid(self, search_area: SearchArea, grid_params: GridParams) -> WaypointArray:
        """
        Generate waypoint grid based on search area and parameters.
        
        Args:
            search_area: Search area definition (GPS or local coordinates)
            grid_params: Grid generation parameters
            
        Returns:
            WaypointArray: Generated waypoints for the mission
        """
        self.logger.info(f"Generating grid with pattern type: {grid_params.pattern_type}")
        
        # Convert search area to local grid bounds
        bounds = self._convert_to_local_bounds(search_area)
        
        # Generate waypoints based on pattern type
        if grid_params.pattern_type == GridParams.PATTERN_BOUSTROPHEDON:
            waypoints = self._generate_boustrophedon_pattern(bounds, grid_params)
        elif grid_params.pattern_type == GridParams.PATTERN_SPIRAL:
            waypoints = self._generate_spiral_pattern(bounds, grid_params)
        else:
            raise ValueError(f"Unsupported pattern type: {grid_params.pattern_type}")
            
        # Create waypoint array message
        waypoint_array = WaypointArray()
        waypoint_array.header = Header()
        waypoint_array.header.stamp = self.node.get_clock().now().to_msg()
        waypoint_array.header.frame_id = "map"
        waypoint_array.waypoints = waypoints
        waypoint_array.total_waypoints = len(waypoints)
        
        # Calculate mission metrics
        total_distance, flight_time = self._calculate_mission_metrics(waypoints, grid_params)
        waypoint_array.total_distance = total_distance
        waypoint_array.estimated_flight_time = flight_time
        
        self.logger.info(f"Generated {len(waypoints)} waypoints, "
                        f"distance: {total_distance:.1f}m, time: {flight_time:.1f}s")
        
        return waypoint_array
    
    def _convert_to_local_bounds(self, search_area: SearchArea) -> GridBounds:
        """Convert search area to local coordinate bounds"""
        
        if search_area.coordinate_type == SearchArea.COORDINATE_TYPE_LOCAL:
            return GridBounds(
                north=search_area.north_m,
                south=search_area.south_m,
                east=search_area.east_m,
                west=search_area.west_m,
                min_alt=search_area.min_altitude,
                max_alt=search_area.max_altitude,
                default_alt=search_area.default_altitude
            )
        
        elif search_area.coordinate_type == SearchArea.COORDINATE_TYPE_GPS:
            # Convert GPS to local NED coordinates
            # This is a simplified conversion - real implementation should use proper geodetic libraries
            center_lat = (search_area.north_lat + search_area.south_lat) / 2.0
            center_lon = (search_area.east_lon + search_area.west_lon) / 2.0
            
            # Convert to meters (approximate)
            lat_to_m = self.earth_radius * math.pi / 180.0
            lon_to_m = lat_to_m * math.cos(math.radians(center_lat))
            
            north_m = (search_area.north_lat - center_lat) * lat_to_m
            south_m = (search_area.south_lat - center_lat) * lat_to_m
            east_m = (search_area.east_lon - center_lon) * lon_to_m
            west_m = (search_area.west_lon - center_lon) * lon_to_m
            
            return GridBounds(
                north=north_m,
                south=south_m,
                east=east_m,
                west=west_m,
                min_alt=search_area.min_altitude,
                max_alt=search_area.max_altitude,
                default_alt=search_area.default_altitude
            )
        
        else:
            raise ValueError(f"Unsupported coordinate type: {search_area.coordinate_type}")
    
    def _generate_boustrophedon_pattern(self, bounds: GridBounds, params: GridParams) -> List[Waypoint]:
        """
        Generate boustrophedon (lawnmower) pattern waypoints.
        
        This creates parallel flight lines with alternating directions.
        """
        waypoints = []
        waypoint_id = 0
        
        # Calculate flight line parameters
        flight_direction = params.initial_heading
        perpendicular_direction = flight_direction + math.pi / 2
        
        # Determine primary and secondary axes based on flight direction
        if abs(math.cos(flight_direction)) > abs(math.sin(flight_direction)):
            # Primarily north-south flight lines
            primary_axis = 'ns'
            line_start = bounds.west
            line_end = bounds.east
            line_min = bounds.south
            line_max = bounds.north
        else:
            # Primarily east-west flight lines  
            primary_axis = 'ew'
            line_start = bounds.south
            line_end = bounds.north
            line_min = bounds.west
            line_max = bounds.east
        
        # Calculate number of flight lines
        area_width = abs(line_max - line_min)
        num_lines = max(1, int(area_width / params.lane_spacing) + 1)
        
        # Generate flight lines
        for line_idx in range(num_lines):
            # Calculate line position
            if num_lines == 1:
                line_pos = (line_min + line_max) / 2
            else:
                line_pos = line_min + (line_idx * area_width / (num_lines - 1))
            
            # Determine flight direction for this line
            if params.reverse_direction and line_idx % 2 == 1:
                start_pos = line_end
                end_pos = line_start
            else:
                start_pos = line_start
                end_pos = line_end
            
            # Generate waypoints along this line
            line_waypoints = self._generate_line_waypoints(
                primary_axis, line_pos, start_pos, end_pos,
                bounds.default_alt, params, waypoint_id
            )
            
            waypoints.extend(line_waypoints)
            waypoint_id += len(line_waypoints)
            
            # Add turn waypoint if not the last line
            if line_idx < num_lines - 1 and params.turn_radius > 0:
                turn_waypoint = self._generate_turn_waypoint(
                    waypoints[-1], line_idx, params, bounds, waypoint_id
                )
                if turn_waypoint:
                    waypoints.append(turn_waypoint)
                    waypoint_id += 1
        
        # Optimize path if requested
        if params.optimize_path:
            waypoints = self._optimize_waypoint_path(waypoints, params)
        
        return waypoints
    
    def _generate_line_waypoints(self, primary_axis: str, line_pos: float,
                                start_pos: float, end_pos: float, altitude: float,
                                params: GridParams, start_id: int) -> List[Waypoint]:
        """Generate waypoints along a single flight line"""
        waypoints = []
        
        # Calculate waypoint positions along line
        line_length = abs(end_pos - start_pos)
        if params.waypoint_spacing <= 0:
            num_waypoints = 2  # Just start and end
        else:
            num_waypoints = max(2, int(line_length / params.waypoint_spacing) + 1)
        
        for i in range(num_waypoints):
            # Calculate position along line
            if num_waypoints == 1:
                pos_along_line = (start_pos + end_pos) / 2
            else:
                t = i / (num_waypoints - 1)
                pos_along_line = start_pos + t * (end_pos - start_pos)
            
            # Create waypoint
            waypoint = Waypoint()
            waypoint.waypoint_id = start_id + i
            waypoint.waypoint_type = Waypoint.TYPE_NAVIGATION
            
            # Set position based on primary axis
            if primary_axis == 'ns':
                waypoint.position = Point(x=pos_along_line, y=line_pos, z=-altitude)
            else:
                waypoint.position = Point(x=line_pos, y=pos_along_line, z=-altitude)
            
            waypoint.yaw = 0.0  # Will be calculated based on flight direction
            waypoint.speed_ms = params.speed_ms
            waypoint.acceptance_radius = 2.0  # Default 2m acceptance radius
            waypoint.capture_photo = True  # Capture photos during search
            waypoint.record_sensors = True
            
            waypoints.append(waypoint)
        
        return waypoints
    
    def _generate_turn_waypoint(self, last_waypoint: Waypoint, line_idx: int,
                               params: GridParams, bounds: GridBounds,
                               waypoint_id: int) -> Optional[Waypoint]:
        """Generate a turn waypoint between flight lines"""
        if params.turn_radius <= 0:
            return None
        
        # Simple turn waypoint - could be enhanced with proper turn geometry
        turn_waypoint = Waypoint()
        turn_waypoint.waypoint_id = waypoint_id
        turn_waypoint.waypoint_type = Waypoint.TYPE_NAVIGATION
        
        # Position slightly offset from last waypoint for turning
        turn_waypoint.position = Point()
        turn_waypoint.position.x = last_waypoint.position.x
        turn_waypoint.position.y = last_waypoint.position.y
        turn_waypoint.position.z = last_waypoint.position.z
        
        turn_waypoint.speed_ms = params.speed_ms * 0.7  # Slower for turns
        turn_waypoint.acceptance_radius = params.turn_radius
        turn_waypoint.hover_time = 0.5  # Brief pause for turn
        
        return turn_waypoint
    
    def _generate_spiral_pattern(self, bounds: GridBounds, params: GridParams) -> List[Waypoint]:
        """Generate spiral pattern waypoints (future implementation)"""
        # Placeholder for spiral pattern
        self.logger.warn("Spiral pattern not yet implemented, using boustrophedon")
        return self._generate_boustrophedon_pattern(bounds, params)
    
    def _optimize_waypoint_path(self, waypoints: List[Waypoint], params: GridParams) -> List[Waypoint]:
        """Optimize waypoint path for efficiency"""
        if not params.minimize_turns:
            return waypoints
        
        # Simple optimization - could be enhanced with more sophisticated algorithms
        # For now, just ensure proper yaw angles for smooth flight
        for i in range(len(waypoints) - 1):
            current_wp = waypoints[i]
            next_wp = waypoints[i + 1]
            
            # Calculate yaw angle to next waypoint
            dx = next_wp.position.x - current_wp.position.x
            dy = next_wp.position.y - current_wp.position.y
            
            if abs(dx) > 0.1 or abs(dy) > 0.1:  # Avoid division by zero
                yaw = math.atan2(dy, dx)
                current_wp.yaw = yaw
        
        return waypoints
    
    def _calculate_mission_metrics(self, waypoints: List[Waypoint], params: GridParams) -> Tuple[float, float]:
        """Calculate total distance and estimated flight time"""
        if len(waypoints) < 2:
            return 0.0, 0.0
        
        total_distance = 0.0
        
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            dx = wp2.position.x - wp1.position.x
            dy = wp2.position.y - wp1.position.y
            dz = wp2.position.z - wp1.position.z
            
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += distance
        
        # Estimate flight time (simplified)
        average_speed = params.speed_ms if params.speed_ms > 0 else 5.0
        estimated_time = total_distance / average_speed
        
        # Add time for hover points, turns, etc.
        hover_time = sum(wp.hover_time for wp in waypoints if hasattr(wp, 'hover_time'))
        estimated_time += hover_time
        
        return total_distance, estimated_time


def main():
    """Standalone test function"""
    import rclpy
    from rclpy.node import Node
    
    rclpy.init()
    node = Node('grid_generator_test')
    
    generator = GridGenerator(node)
    
    # Test with sample data
    search_area = SearchArea()
    search_area.coordinate_type = SearchArea.COORDINATE_TYPE_LOCAL
    search_area.north_m = 100.0
    search_area.south_m = 0.0
    search_area.east_m = 100.0
    search_area.west_m = 0.0
    search_area.default_altitude = 20.0
    
    grid_params = GridParams()
    grid_params.pattern_type = GridParams.PATTERN_BOUSTROPHEDON
    grid_params.lane_spacing = 20.0
    grid_params.waypoint_spacing = 10.0
    grid_params.speed_ms = 5.0
    grid_params.initial_heading = 0.0  # North
    grid_params.reverse_direction = True
    
    waypoints = generator.generate_grid(search_area, grid_params)
    
    print(f"Generated {len(waypoints.waypoints)} waypoints")
    print(f"Total distance: {waypoints.total_distance:.1f}m")
    print(f"Estimated time: {waypoints.estimated_flight_time:.1f}s")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()