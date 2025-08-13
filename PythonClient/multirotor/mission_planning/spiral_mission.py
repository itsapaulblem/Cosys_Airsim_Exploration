#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class SpiralMission(GenericMission):
    """
    SpiralMission implements Archimedes spiral patterns for systematic area search.
    Supports both outward (center to edge) and inward (edge to center) spiral patterns.
    Includes comprehensive collision detection and recovery mechanisms.
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
        print(f"\n SPIRAL MISSION PARAMETERS")
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
        print(f"\n COLLISION DETECTION")
        print(f"Collision Detection: {'Enabled' if self.collision_detection_enabled else 'Disabled'}")
        print(f"Max Collisions Before Landing: {self.max_collisions}")
        print(f"Landscape Collisions Ignored: {'Yes' if self.ignore_landscape_collisions else 'No'}")
    
    def execute_mission_logic(self, target_z):
        """Execute the spiral search mission with collision monitoring."""
        print(f" Executing {self.pattern} spiral search pattern")
        print(f" Collision monitoring active - mission will abort on building impacts")
        
        try:
            if self.pattern == "outward":
                self._execute_outward_spiral(target_z)
            else:
                self._execute_inward_spiral(target_z)
            
            print(f" Spiral search completed!")
            print(f" Mission Summary: {self.spiral_turns:.1f} turns, {self.total_photos} photos taken")
            
            if self.collision_count > 0:
                print(f"   WARNING: {self.collision_count} collision(s) detected during mission")
                
        except Exception as e:
            if "collision" in str(e).lower():
                print(f"  Mission aborted due to collision: {str(e)}")
                self.errors.append(f"Mission aborted: {str(e)}")
            else:
                print(f"  Mission failed: {str(e)}")
                self.errors.append(f"Mission execution error: {str(e)}")
            raise
    
    def _execute_outward_spiral(self, target_z):
        """Execute outward spiral (center to edge) with collision monitoring."""
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
            # Check for collisions before each movement
            if self._check_collision():
                if self.emergency_landing_triggered:
                    raise Exception("Critical collision detected - emergency landing initiated")
                # Continue if recoverable collision
            
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
            
            # Small delay for smooth motion and collision checking
            time.sleep(0.05)
        
        print(f" Outward spiral completed: {self.spiral_turns:.1f} turns")
    
    def _execute_inward_spiral(self, target_z):
        """Execute inward spiral (edge to center) with collision monitoring."""
        print(" Starting from edge, spiraling inward...")
        
        # Move to starting position at edge
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        # Start from the edge
        start_x = center_x + self.max_radius
        start_y = center_y
        
        print(f" Moving to spiral start position...")
        
        # Check for collisions during initial move
        if self._check_collision():
            if self.emergency_landing_triggered:
                raise Exception("Critical collision detected during initial positioning")
        
        self.client.moveToPositionAsync(start_x, start_y, target_z, self.speed).join()
        
        # Spiral parameters for inward spiral
        a = self.spiral_spacing / (2 * math.pi)
        max_angle = self.max_radius / a  # Maximum angle corresponding to max radius
        angle = max_angle  # Start from maximum angle
        angle_step = -0.1  # Negative for inward spiral
        
        while angle > 0:
            # Check for collisions before each movement
            if self._check_collision():
                if self.emergency_landing_triggered:
                    raise Exception("Critical collision detected - emergency landing initiated")
            
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
            
            # Small delay for smooth motion and collision checking
            time.sleep(0.05)
        
        # Final move to center with collision check
        if not self._check_collision():
            self.client.moveToPositionAsync(center_x, center_y, target_z, self.speed).join()
        elif self.emergency_landing_triggered:
            raise Exception("Critical collision detected during final approach to center")
        
        print(f" Inward spiral completed: {self.spiral_turns:.1f} turns")
    
    def _move_smoothly_to_position(self, x, y, z):
        """Move smoothly to position using velocity-based control with collision monitoring."""
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
            # Check if this was a collision-related error
            if "collision" in str(e).lower():
                print(f"   Movement interrupted - possible collision detected")
                self._check_collision()  # Force collision check
    
    def _take_photo_if_time(self):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            self.take_photo(photo_dir="spiral_photos", 
                          filename_prefix=f"spiral_{self.pattern}")
            self.total_photos += 1
            self.last_photo_time = current_time
    
    def _calculate_spiral_length(self):
        """Calculate the approximate length of the spiral path."""
        # For Archimedes spiral, approximate length calculation
        # Using the formula for spiral arc length
        a = self.spiral_spacing / (2 * math.pi)
        max_angle = self.max_radius / a
        
        # Simplified length approximation
        # More accurate would require integration
        return a * max_angle * max_angle / 2
    
    def get_planned_distance(self):
        """Return the planned flight distance for this mission."""
        spiral_length = self._calculate_spiral_length()
        # Add distance to reach spiral start (for inward pattern) and return home
        overhead_distance = self.max_radius * 2 if self.pattern == "inward" else 0
        return spiral_length + overhead_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics for reporting."""
        coverage_area = math.pi * self.max_radius ** 2
        
        return {
            "spiral_pattern": self.pattern,
            "spiral_turns": self.spiral_turns,
            "photos_taken": self.total_photos,
            "coverage_area_sqm": coverage_area,
            "coverage_area_hectares": coverage_area / 10000,
            "max_radius": self.max_radius,
            "spiral_spacing": self.spiral_spacing,
            "photo_interval": self.photo_interval,
            "collision_count": self.collision_count,
            "collisions_per_km": (self.collision_count / max(self.total_distance/1000, 0.1)) if self.total_distance > 0 else 0
        }

def main():
    """Main execution function with command line argument parsing."""
    parser = create_standard_argument_parser()
    
    # Add spiral-specific arguments
    parser.add_argument('--max_radius', type=float, default=50.0,
                       help='Maximum spiral radius in meters (default: 50)')
    parser.add_argument('--spiral_spacing', type=float, default=10.0,
                       help='Distance between spiral arms in meters (default: 10)')
    parser.add_argument('--pattern', choices=['outward', 'inward'], default='outward',
                       help='Spiral pattern: outward (center to edge) or inward (edge to center)')
    parser.add_argument('--photo_interval', type=float, default=2.0,
                       help='Time interval between photos in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    # Create and run the mission
    mission = SpiralMission(
        max_radius=args.max_radius,
        altitude=args.altitude,
        speed=args.speed,
        spiral_spacing=args.spiral_spacing,
        pattern=args.pattern,
        photo_interval=args.photo_interval
    )
    
    mission.run_mission()

if __name__ == "__main__":
    main() 