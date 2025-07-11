#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class SpiralSearchMission(GenericMission):
    """
    SpiralSearchMission implements Archimedes spiral patterns for systematic area search.
    Supports both outward (center to edge) and inward (edge to center) spiral patterns.
    """
    
    def __init__(self, max_radius=500, altitude=20, speed=5, spiral_spacing=25, 
                 pattern="outward", photo_interval=2.0):
        """
        Initialize the spiral search mission.
        
        Args:
            max_radius: Maximum radius of the spiral in meters
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            spiral_spacing: Distance between spiral arms in meters
            pattern: "outward" (center to edge) or "inward" (edge to center)
            photo_interval: Time interval between photos in seconds
        """
        super().__init__(altitude, speed)
        self.max_radius = max_radius
        self.spiral_spacing = spiral_spacing
        self.pattern = pattern.lower()
        self.photo_interval = photo_interval
        
        # Mission tracking
        self.total_photos = 0
        self.spiral_turns = 0
        self.coverage_area = 0
        self.last_photo_time = 0
        
        # Validate pattern
        if self.pattern not in ["outward", "inward"]:
            raise ValueError("Pattern must be 'outward' or 'inward'")
    
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\n SPIRAL SEARCH MISSION PARAMETERS")
        print(f"Search Pattern: {self.pattern.title()} Spiral")
        print(f"Maximum Radius: {self.max_radius}m")
        print(f"Spiral Spacing: {self.spiral_spacing}m")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Photo Interval: {self.photo_interval}s")
        
        # Calculate spiral characteristics
        total_turns = self.max_radius / self.spiral_spacing
        total_length = self._calculate_spiral_length()
        coverage_area = math.pi * self.max_radius ** 2
        
        print(f"\n SPIRAL CHARACTERISTICS")
        print(f"Total Spiral Turns: {total_turns:.1f}")
        print(f"Spiral Length: {total_length:.0f}m")
        print(f"Coverage Area: {coverage_area/10000:.1f} hectares")
        print(f"Estimated Photos: {int(total_length / self.speed / self.photo_interval)}")
    
    def execute_mission_logic(self, target_z):
        """Execute the spiral search mission."""
        print(f" Executing {self.pattern} spiral search pattern")
        
        if self.pattern == "outward":
            self._execute_outward_spiral(target_z)
        else:
            self._execute_inward_spiral(target_z)
        
        print(f" Spiral search completed!")
        print(f" Mission Summary: {self.spiral_turns:.1f} turns, {self.total_photos} photos taken")
    
    def _execute_outward_spiral(self, target_z):
        """Execute outward spiral (center to edge)."""
        print(" Starting from center, spiraling outward...")
        
        # Move to center position
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        # Spiral parameters for Archimedes spiral: r = a * θ
        # where 'a' determines the spacing between spiral arms
        a = self.spiral_spacing / (2 * math.pi)  # Spiral constant
        angle = 0  # Starting angle
        angle_step = 0.1  # Radians per step (smaller = smoother)
        
        while True:
            # Calculate current radius using Archimedes spiral formula
            radius = a * angle
            
            # Stop if we've reached maximum radius
            if radius > self.max_radius:
                break
            
            # Calculate position
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # Move to position using smooth velocity control
            self._move_smoothly_to_position(x, y, target_z)
            
            # Take photos at intervals
            self._take_photo_if_time()
            
            # Update tracking
            self.update_distance_tracking()
            self.spiral_turns = angle / (2 * math.pi)
            
            # Increment angle
            angle += angle_step
            
            # Small delay for smooth motion
            time.sleep(0.05)
        
        print(f" Outward spiral completed: {self.spiral_turns:.1f} turns")
    
    def _execute_inward_spiral(self, target_z):
        """Execute inward spiral (edge to center)."""
        print(" Starting from edge, spiraling inward...")
        
        # Move to starting position at edge
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        # Start from the edge
        start_x = center_x + self.max_radius
        start_y = center_y
        
        print(f" Moving to spiral start position...")
        self.client.moveToPositionAsync(start_x, start_y, target_z, self.speed).join()
        
        # Spiral parameters for inward spiral
        a = self.spiral_spacing / (2 * math.pi)
        max_angle = self.max_radius / a  # Maximum angle corresponding to max radius
        angle = max_angle  # Start from maximum angle
        angle_step = -0.1  # Negative for inward spiral
        
        while angle > 0:
            # Calculate current radius
            radius = a * angle
            
            # Calculate position
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # Move to position using smooth velocity control
            self._move_smoothly_to_position(x, y, target_z)
            
            # Take photos at intervals
            self._take_photo_if_time()
            
            # Update tracking
            self.update_distance_tracking()
            self.spiral_turns = (max_angle - angle) / (2 * math.pi)
            
            # Increment angle (decrement for inward)
            angle += angle_step
            
            # Small delay for smooth motion
            time.sleep(0.05)
        
        # Final move to center
        self.client.moveToPositionAsync(center_x, center_y, target_z, self.speed).join()
        
        print(f" Inward spiral completed: {self.spiral_turns:.1f} turns")
    
    def _move_smoothly_to_position(self, x, y, z):
        """Move smoothly to position using velocity-based control."""
        try:
            # Get current position
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate direction vector
            dx = x - current_pos.x_val
            dy = y - current_pos.y_val
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 1.0:  # Only move if distance is significant
                # Normalize direction and scale by speed
                vx = (dx / distance) * self.speed
                vy = (dy / distance) * self.speed
                
                # Calculate time to reach position
                move_time = distance / self.speed
                
                # Move using velocity control
                self.client.moveByVelocityAsync(vx, vy, 0, min(move_time, 0.5)).join()
            
        except Exception as e:
            self.errors.append(f"Smooth movement error: {str(e)}")
    
    def _take_photo_if_time(self):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            self.take_photo(photo_dir="spiral_search_photos", 
                           filename_prefix=f"spiral_{self.pattern}")
            self.total_photos += 1
            self.last_photo_time = current_time
    
    def _calculate_spiral_length(self):
        """Calculate the approximate length of the spiral."""
        # For Archimedes spiral: L ≈ (a/2) * [θ * sqrt(1 + θ²) + ln(θ + sqrt(1 + θ²))]
        # Simplified approximation: L ≈ (1/2) * a * θ²
        a = self.spiral_spacing / (2 * math.pi)
        max_angle = self.max_radius / a
        
        # More accurate formula for spiral length
        length = (a / 2) * (max_angle * math.sqrt(1 + max_angle**2) + 
                           math.log(max_angle + math.sqrt(1 + max_angle**2)))
        
        return length
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        spiral_length = self._calculate_spiral_length()
        
        # Add buffer for takeoff/landing and positioning
        buffer_distance = 200
        
        return spiral_length + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        coverage_area = math.pi * self.max_radius ** 2
        
        return {
            "Search Pattern": f"{self.pattern.title()} Spiral",
            "Spiral Turns": f"{self.spiral_turns:.1f}",
            "Coverage Area": f"{coverage_area/10000:.1f} hectares",
            "Total Photos": self.total_photos,
            "Max Radius": f"{self.max_radius}m",
            "Spiral Spacing": f"{self.spiral_spacing}m"
        }


def main():
    """Main function to run the spiral search mission."""
    # Create argument parser with standard parameters
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Spiral Search Mission"
    
    # Add spiral-specific parameters
    parser.add_argument('--max_radius', type=float, default=500,
                       help='Maximum spiral radius in meters (default: 500)')
    parser.add_argument('--spiral_spacing', type=float, default=25,
                       help='Distance between spiral arms in meters (default: 25)')
    parser.add_argument('--pattern', type=str, choices=['outward', 'inward'], 
                       default='outward',
                       help='Spiral pattern: outward (center to edge) or inward (edge to center)')
    parser.add_argument('--photo_interval', type=float, default=2.0,
                       help='Time interval between photos in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    # Create and configure mission
    mission = SpiralSearchMission(
        max_radius=args.max_radius,
        altitude=args.altitude,
        speed=args.speed,
        spiral_spacing=args.spiral_spacing,
        pattern=args.pattern,
        photo_interval=args.photo_interval
    )
    
    # Preview mode
    if args.preview:
        print(" MISSION PREVIEW MODE")
        mission.print_mission_parameters()
        
        planned_distance = mission.get_planned_distance()
        estimated_time = planned_distance / args.speed
        estimated_photos = int(estimated_time / args.photo_interval)
        
        print(f"\n MISSION ESTIMATES")
        print(f"Planned Distance: {planned_distance:.0f}m")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        print(f"Estimated Photos: {estimated_photos}")
        
        print(f"\n To execute the mission, run without --preview flag")
        return
    
    # Execute mission
    print(f" Starting {args.pattern.title()} Spiral Search Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 