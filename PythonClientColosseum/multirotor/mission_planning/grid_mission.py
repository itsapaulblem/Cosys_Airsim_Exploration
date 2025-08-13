#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class GridSurveyMission(GenericMission):
    """
    GridSurveyMission implements professional survey patterns for mapping and photogrammetry.
    Supports boustrophedon (back-and-forth) and parallel flight line patterns.
    """
    
    def __init__(self, width=800, height=600, altitude=20, speed=5, 
                 grid_spacing=50, photo_overlap=30, pattern="boustrophedon", 
                 grid_rotation=0):
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
        """
        super().__init__(altitude, speed)
        self.width = width
        self.height = height
        self.grid_spacing = grid_spacing
        self.photo_overlap = photo_overlap
        self.pattern = pattern.lower()
        self.grid_rotation = math.radians(grid_rotation)  # Convert to radians
        
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
            
            # Fly the flight line smoothly
            self._fly_flight_line_smoothly(start_point, end_point, target_z)
            
            self.flight_lines_completed += 1
        
        print(f" Boustrophedon pattern completed: {self.flight_lines_completed} flight lines")
    
    def _execute_parallel_pattern(self, target_z):
        """Execute parallel survey pattern (all lines in same direction)."""
        print(" Executing parallel survey pattern...")
        
        for i, flight_line in enumerate(self.flight_lines):
            print(f"\n  Flight line {i+1}/{len(self.flight_lines)}")
            
            start_point, end_point = flight_line
            
            # For parallel pattern, always use the same start direction
            # Move to start of flight line
            print(f"   Moving to start of flight line...")
            abs_start_x = self.home_position.x_val + start_point[0]
            abs_start_y = self.home_position.y_val + start_point[1]
            self.client.moveToPositionAsync(abs_start_x, abs_start_y, target_z, self.speed).join()
            
            # Fly the flight line
            self._fly_flight_line_smoothly(start_point, end_point, target_z)
            
            self.flight_lines_completed += 1
        
        print(f" Parallel pattern completed: {self.flight_lines_completed} flight lines")
    
    def _fly_flight_line_smoothly(self, start_point, end_point, target_z):
        """Fly a flight line smoothly with continuous photo capture."""
        # Convert to absolute coordinates
        abs_start_x = self.home_position.x_val + start_point[0]
        abs_start_y = self.home_position.y_val + start_point[1]
        abs_end_x = self.home_position.x_val + end_point[0]
        abs_end_y = self.home_position.y_val + end_point[1]
        
        # Calculate flight line parameters
        dx = abs_end_x - abs_start_x
        dy = abs_end_y - abs_start_y
        line_length = math.sqrt(dx*dx + dy*dy)
        
        if line_length == 0:
            return
        
        # Move to start of flight line
        self.client.moveToPositionAsync(abs_start_x, abs_start_y, target_z, self.speed).join()
        
        # Fly the line using smooth velocity control
        print(f"   Flying survey line with photo capture...")
        
        # Calculate number of steps for smooth flight
        steps = max(20, int(line_length / 5))  # At least 20 steps, or one every 5m
        
        for step in range(steps + 1):
            progress = step / steps
            
            # Calculate current position along the line
            current_x = abs_start_x + dx * progress
            current_y = abs_start_y + dy * progress
            
            # Move to position
            self.client.moveToPositionAsync(current_x, current_y, target_z, self.speed).join()
            
            # Take photos at intervals
            self._take_photo_if_time()
            
            # Update tracking
            self.update_distance_tracking()
            
            # Small delay for smooth motion
            time.sleep(0.1)
        
        print(f"   Flight line completed")
    
    def _calculate_flight_lines(self):
        """Calculate flight lines based on survey area and parameters."""
        flight_lines = []
        
        # Calculate number of flight lines
        num_lines = int(self.width / self.grid_spacing) + 1
        
        # Calculate flight line endpoints
        for i in range(num_lines):
            # X position for this flight line
            x = -self.width/2 + i * self.grid_spacing
            
            # Y endpoints (full height of survey area)
            y_start = -self.height/2
            y_end = self.height/2
            
            # For boustrophedon pattern, alternate direction
            if self.pattern == "boustrophedon" and i % 2 == 1:
                y_start, y_end = y_end, y_start  # Reverse direction
            
            # Apply rotation if specified
            if self.grid_rotation != 0:
                start_point = self._rotate_point((x, y_start), self.grid_rotation)
                end_point = self._rotate_point((x, y_end), self.grid_rotation)
            else:
                start_point = (x, y_start)
                end_point = (x, y_end)
            
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
    
    args = parser.parse_args()
    
    # Create and configure mission
    mission = GridSurveyMission(
        width=args.width,
        height=args.height,
        altitude=args.altitude,
        speed=args.speed,
        grid_spacing=args.grid_spacing,
        photo_overlap=args.photo_overlap,
        pattern=args.pattern,
        grid_rotation=args.grid_rotation
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