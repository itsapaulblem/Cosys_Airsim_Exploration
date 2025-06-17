#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser
import cosysairsim as airsim

class GridSurveyMission(GenericMission):
    """
    GridSurveyMission implements professional survey patterns for mapping and photogrammetry.
    Supports boustrophedon (back-and-forth) and parallel flight line patterns.
    Uses takeoff position as default reference point, with option to specify custom start point.
    """
    
    def __init__(self, width=800, height=600, altitude=20, speed=5, 
                 grid_spacing=50, photo_overlap=30, pattern="boustrophedon", 
                 grid_rotation=0, start_lat=None, start_lon=None):
        """
        Initialize the grid survey mission.
        
        Args:
            width: Survey area width in meters
            height: Survey area height in meters
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            grid_spacing: Distance between parallel flight lines in meters
            photo_overlap: Photo overlap percentage (30% recommended)
            pattern: "boustrophedon" (back-and-forth) or "parallel" (all same direction)
            grid_rotation: Grid rotation angle in degrees (0 = aligned with X-axis)
            start_lat: Optional starting latitude (if None, uses takeoff position)
            start_lon: Optional starting longitude (if None, uses takeoff position)
        """
        super().__init__(altitude, speed)
        self.width = width
        self.height = height
        self.grid_spacing = grid_spacing
        self.photo_overlap = photo_overlap
        self.pattern = pattern.lower()
        self.grid_rotation = math.radians(grid_rotation)  # Convert to radians
        
        # Store custom start coordinates if provided
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.custom_start_position = None  # Will be set after takeoff if lat/lon provided
        
        # Mission tracking
        self.total_photos = 0
        self.flight_lines_completed = 0
        self.coverage_area = 0
        self.last_photo_time = 0
        
        # Calculate photo interval based on overlap
        self.photo_interval = self._calculate_photo_interval()
        
        # Validate pattern
        if self.pattern not in ["boustrophedon", "parallel"]:
            raise ValueError("Pattern must be 'boustrophedon' or 'parallel'")
        
        # Calculate flight lines
        self.flight_lines = self._calculate_flight_lines()
    
    def _connect_and_setup(self):
        """Connect to AirSim and perform initial setup with custom start point handling."""
        super()._connect_and_setup()  # Call parent method first
        
        # If custom start coordinates provided, convert to local coordinates
        if self.start_lat is not None and self.start_lon is not None:
            print(f"\nConverting custom start coordinates to local position...")
            self.custom_start_position = self._convert_latlon_to_local(
                self.start_lat, self.start_lon)
            print(f"Custom start position: ({self.custom_start_position.x_val:.1f}, {self.custom_start_position.y_val:.1f})")
    
    def _convert_latlon_to_local(self, lat, lon):
        """Convert latitude/longitude coordinates to local coordinates relative to takeoff position."""
        try:
            # Get the home position (takeoff point)
            home_lat = self.client.getHomeGeoPoint().latitude
            home_lon = self.client.getHomeGeoPoint().longitude
            
            # Calculate local position using AirSim's coordinate conversion
            local_position = self.client.getLocalPositionFromGeoPoint(
                airsim.GeoPoint(lat, lon, 0))
            
            return local_position
            
        except Exception as e:
            error_msg = f"Failed to convert lat/lon coordinates: {str(e)}"
            print(f"  {error_msg}")
            self.errors.append(error_msg)
            # Fall back to takeoff position
            return self.home_position
    
    def _get_reference_position(self):
        """Get the reference position for the survey (either custom start or takeoff position)."""
        if self.custom_start_position is not None:
            return self.custom_start_position
        return self.home_position
    
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\n  GRID SURVEY MISSION PARAMETERS")
        print(f"Survey Area: {self.width}m x {self.height}m")
        print(f"Grid Pattern: {self.pattern.title()}")
        print(f"Grid Spacing: {self.grid_spacing}m")
        print(f"Grid Rotation: {math.degrees(self.grid_rotation):.1f}°")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Photo Overlap: {self.photo_overlap}%")
        print(f"Photo Interval: {self.photo_interval:.1f}s")
        
        # Print start point information
        if self.start_lat is not None and self.start_lon is not None:
            print(f"\nStart Point: Custom coordinates")
            print(f"Latitude: {self.start_lat:.6f}")
            print(f"Longitude: {self.start_lon:.6f}")
        else:
            print(f"\nStart Point: Takeoff position")
        
        # Calculate survey characteristics
        num_lines = len(self.flight_lines)
        total_length = sum(self._calculate_line_length(line) for line in self.flight_lines)
        coverage_area = self.width * self.height
        estimated_photos = int(total_length / self.speed / self.photo_interval)
        
        print(f"\n SURVEY CHARACTERISTICS")
        print(f"Number of Flight Lines: {num_lines}")
        print(f"Total Flight Length: {total_length:.0f}m")
        print(f"Coverage Area: {coverage_area/10000:.1f} hectares")
        print(f"Estimated Photos: {estimated_photos}")
    
    def execute_mission_logic(self, target_z):
        """Execute the grid survey mission."""
        print(f"  Executing {self.pattern} grid survey pattern")
        
        if self.pattern == "boustrophedon":
            self._execute_boustrophedon_pattern(target_z)
        else:
            self._execute_parallel_pattern(target_z)
        
        print(f" Grid survey completed!")
        print(f" Mission Summary: {self.flight_lines_completed} flight lines, {self.total_photos} photos taken")
    
    def _execute_boustrophedon_pattern(self, target_z):
        """Execute boustrophedon (back-and-forth) survey pattern."""
        print(" Executing boustrophedon survey pattern...")
        
        for i, flight_line in enumerate(self.flight_lines):
            print(f"\n  Flight line {i+1}/{len(self.flight_lines)}")
            
            start_point, end_point = flight_line
            
            # Move to start of flight line
            print(f"   Moving to start of flight line...")
            abs_start_x = self.home_position.x_val + start_point[0]
            abs_start_y = self.home_position.y_val + start_point[1]
            self._fly_to_position(abs_start_x, abs_start_y, target_z)
            
            # Fly the flight line smoothly
            self._fly_flight_line_smoothly(start_point, end_point, target_z)
            
            self.flight_lines_completed += 1
            
            # If not the last line, move to start of next line
            if i < len(self.flight_lines) - 1:
                next_start = self.flight_lines[i + 1][0]
                next_start_x = self.home_position.x_val + next_start[0]
                next_start_y = self.home_position.y_val + next_start[1]
                print(f"   Moving to start of next flight line...")
                self._fly_to_position(next_start_x, next_start_y, target_z)
        
        print(f" Boustrophedon pattern completed: {self.flight_lines_completed} flight lines")
    
    def _execute_parallel_pattern(self, target_z):
        """Execute parallel survey pattern (all lines in same direction)."""
        print(" Executing parallel survey pattern...")
        
        for i, flight_line in enumerate(self.flight_lines):
            print(f"\n  Flight line {i+1}/{len(self.flight_lines)}")
            
            start_point, end_point = flight_line
            
            # Move to start of flight line
            print(f"   Moving to start of flight line...")
            abs_start_x = self.home_position.x_val + start_point[0]
            abs_start_y = self.home_position.y_val + start_point[1]
            self._fly_to_position(abs_start_x, abs_start_y, target_z)
            
            # Fly the flight line
            self._fly_flight_line_smoothly(start_point, end_point, target_z)
            
            self.flight_lines_completed += 1
            
            # If not the last line, move to start of next line
            if i < len(self.flight_lines) - 1:
                next_start = self.flight_lines[i + 1][0]
                next_start_x = self.home_position.x_val + next_start[0]
                next_start_y = self.home_position.y_val + next_start[1]
                print(f"   Moving to start of next flight line...")
                self._fly_to_position(next_start_x, next_start_y, target_z)
        
        print(f" Parallel pattern completed: {self.flight_lines_completed} flight lines")
    
    def _fly_to_position(self, x, y, z):
        """Fly to a specific position using smooth movement."""
        try:
            print(f"   Flying to position ({x:.1f}, {y:.1f}, {z:.1f})")
            
            # Use position-based movement for accuracy
            self.client.moveToPositionAsync(x, y, z, self.speed).join()
            
            # Verify position
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            distance_error = math.sqrt(
                (current_pos.x_val - x)**2 + 
                (current_pos.y_val - y)**2 + 
                (current_pos.z_val - z)**2
            )
            
            if distance_error > 5:  # 5 meter tolerance
                print(f"   Position error: {distance_error:.1f}m")
                self.errors.append(f"Position error at ({x:.1f}, {y:.1f}): {distance_error:.1f}m")
            else:
                print(f"   Arrived at position (error: {distance_error:.1f}m)")
            
        except Exception as e:
            error_msg = f"Failed to fly to position ({x:.1f}, {y:.1f}): {str(e)}"
            print(f"   {error_msg}")
            self.errors.append(error_msg)
    
    def _fly_flight_line_smoothly(self, start_point, end_point, target_z):
        """Fly a flight line smoothly with continuous photo capture."""
        # Get reference position (either custom start or takeoff position)
        ref_pos = self._get_reference_position()
        
        # Convert to absolute coordinates
        abs_start_x = ref_pos.x_val + start_point[0]
        abs_start_y = ref_pos.y_val + start_point[1]
        abs_end_x = ref_pos.x_val + end_point[0]
        abs_end_y = ref_pos.y_val + end_point[1]
        
        # Calculate flight line parameters
        dx = abs_end_x - abs_start_x
        dy = abs_end_y - abs_start_y
        line_length = math.sqrt(dx*dx + dy*dy)
        
        if line_length == 0:
            return
        
        print(f"   Flying survey line ({line_length:.1f}m) with photo capture...")
        
        # Calculate number of steps for smooth flight
        steps = max(20, int(line_length / 5))  # At least 20 steps, or one every 5m
        
        # Calculate velocity components for smooth motion
        velocity_x = (dx / line_length) * self.speed
        velocity_y = (dy / line_length) * self.speed
        
        # Start time for photo interval tracking
        start_time = time.time()
        last_photo_time = start_time
        
        # Fly the line using continuous velocity control
        for step in range(steps + 1):
            progress = step / steps
            
            # Calculate current position along the line
            current_x = abs_start_x + dx * progress
            current_y = abs_start_y + dy * progress
            
            # Apply velocity command for smooth motion
            self.client.moveByVelocityAsync(velocity_x, velocity_y, 0, 0.1).join()
            
            # Take photos at intervals
            current_time = time.time()
            if current_time - last_photo_time >= self.photo_interval:
                photo_result = self.take_photo(photo_dir="grid_survey_photos", 
                               filename_prefix=f"survey_{self.pattern}")
                if photo_result:
                    self.total_photos += 1
                    last_photo_time = current_time
                    print(f"     Photo {self.total_photos} taken at {progress*100:.0f}% of line")
            
            # Update tracking
            self.update_distance_tracking()
            
            # Small delay for smooth motion
            time.sleep(0.05)
        
        # Gentle stop at end of line
        self.client.moveByVelocityAsync(0, 0, 0, 0.2).join()
        print(f"   Flight line completed")
    
    def _calculate_flight_lines(self):
        """Calculate flight lines based on survey area and parameters."""
        flight_lines = []
        
        # Calculate number of flight lines
        num_lines = int(self.height / self.grid_spacing) + 1
        
        # Calculate flight line endpoints
        for i in range(num_lines):
            # Y position for this flight line
            y = -self.height/2 + i * self.grid_spacing
            
            # X endpoints (full width of survey area)
            x_start = -self.width/2
            x_end = self.width/2
            
            # For boustrophedon pattern, alternate direction
            if self.pattern == "boustrophedon" and i % 2 == 1:
                x_start, x_end = x_end, x_start  # Reverse direction
            
            # Apply rotation if specified
            if self.grid_rotation != 0:
                start_point = self._rotate_point((x_start, y), self.grid_rotation)
                end_point = self._rotate_point((x_end, y), self.grid_rotation)
            else:
                start_point = (x_start, y)
                end_point = (x_end, y)
            
            flight_lines.append((start_point, end_point))
        
        return flight_lines
    
    def _rotate_point(self, point, angle):
        """Rotate a point around the origin by the specified angle."""
        x, y = point
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        new_x = x * cos_a - y * sin_a
        new_y = x * sin_a + y * cos_a
        
        return (new_x, new_y)
    
    def _calculate_line_length(self, flight_line):
        """Calculate the length of a flight line."""
        start_point, end_point = flight_line
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_photo_interval(self):
        """Calculate photo interval based on desired overlap percentage."""
        # Simplified calculation: assuming camera covers certain ground area
        # In practice, this would depend on camera specs and altitude
        
        # Example: if we want 30% overlap and flying at 5 m/s
        # Photo interval = (1 - overlap/100) * (photo_footprint / speed)
        # Assuming 20m photo footprint at this altitude
        photo_footprint = 20  # meters
        overlap_factor = 1 - (self.photo_overlap / 100)
        
        interval = overlap_factor * (photo_footprint / self.speed)
        return max(0.5, interval)  # Minimum 0.5 second interval
    
    def _take_photo_if_time(self):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            self.take_photo(photo_dir="grid_survey_photos", 
                           filename_prefix=f"survey_{self.pattern}")
            self.total_photos += 1
            self.last_photo_time = current_time
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        # Sum of all flight line lengths
        total_flight_lines = sum(self._calculate_line_length(line) for line in self.flight_lines)
        
        # Add transition distances between flight lines (for boustrophedon, minimal; for parallel, more)
        if self.pattern == "parallel":
            # For parallel, need to return to start of each line
            transition_distance = (len(self.flight_lines) - 1) * self.height
        else:
            # For boustrophedon, transitions are minimal
            transition_distance = 0
        
        # Add buffer for takeoff/landing and positioning
        buffer_distance = 300
        
        return total_flight_lines + transition_distance + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        coverage_area = self.width * self.height
        
        return {
            "Survey Pattern": f"{self.pattern.title()}",
            "Flight Lines": f"{self.flight_lines_completed}/{len(self.flight_lines)}",
            "Survey Area": f"{self.width}m x {self.height}m",
            "Coverage Area": f"{coverage_area/10000:.1f} hectares",
            "Total Photos": self.total_photos,
            "Grid Spacing": f"{self.grid_spacing}m",
            "Photo Overlap": f"{self.photo_overlap}%"
        }


def main():
    """Main function to run the grid survey mission."""
    # Create argument parser with standard parameters
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Grid Survey Mission for Mapping and Photogrammetry"
    
    # Add grid-specific parameters
    parser.add_argument('--width', type=float, default=800,
                       help='Survey area width in meters (default: 800)')
    parser.add_argument('--height', type=float, default=600,
                       help='Survey area height in meters (default: 600)')
    parser.add_argument('--grid_spacing', type=float, default=50,
                       help='Distance between flight lines in meters (default: 50)')
    parser.add_argument('--photo_overlap', type=float, default=30,
                       help='Photo overlap percentage (default: 30)')
    parser.add_argument('--pattern', type=str, choices=['boustrophedon', 'parallel'], 
                       default='boustrophedon',
                       help='Flight pattern: boustrophedon (back-and-forth) or parallel (same direction)')
    parser.add_argument('--grid_rotation', type=float, default=0,
                       help='Grid rotation angle in degrees (default: 0)')
    parser.add_argument('--start_lat', type=float,
                       help='Custom start latitude (if not provided, uses takeoff position)')
    parser.add_argument('--start_lon', type=float,
                       help='Custom start longitude (if not provided, uses takeoff position)')
    
    args = parser.parse_args()
    
    # Validate lat/lon arguments
    if (args.start_lat is None) != (args.start_lon is None):
        parser.error("Both --start_lat and --start_lon must be provided together")
    
    # Create and configure mission
    mission = GridSurveyMission(
        width=args.width,
        height=args.height,
        altitude=args.altitude,
        speed=args.speed,
        grid_spacing=args.grid_spacing,
        photo_overlap=args.photo_overlap,
        pattern=args.pattern,
        grid_rotation=args.grid_rotation,
        start_lat=args.start_lat,
        start_lon=args.start_lon
    )
    
    # Preview mode
    if args.preview:
        print(" MISSION PREVIEW MODE")
        mission.print_mission_parameters()
        
        planned_distance = mission.get_planned_distance()
        estimated_time = planned_distance / args.speed
        estimated_photos = int(estimated_time / mission.photo_interval)
        
        print(f"\n MISSION ESTIMATES")
        print(f"Planned Distance: {planned_distance:.0f}m")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        print(f"Estimated Photos: {estimated_photos}")
        
        print(f"\n To execute the mission, run without --preview flag")
        return
    
    # Execute mission
    print(f" Starting {args.pattern.title()} Grid Survey Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 