#!/usr/bin/env python3

import math
import time
import random
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class StarPatternMission(GenericMission):
    """
    StarPatternMission flies radiating spoke patterns from a central point for systematic area coverage.
    Features multiple pattern modes and configurable return-to-center behavior.
    """
    
    def __init__(self, num_spokes=8, spoke_length=400, altitude=20, speed=5,
                 spoke_pattern="sequential", return_to_center=True, 
                 hover_at_endpoints=True, hover_time=3.0, photo_interval=2.0):
        """
        Initialize the star pattern mission.
        
        Args:
            num_spokes: Number of radiating spokes (like compass directions)
            spoke_length: Length of each spoke in meters
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            spoke_pattern: Pattern order - "sequential", "random", "alternating", "crisscross"
            return_to_center: Return to center between each spoke
            hover_at_endpoints: Hover at the end of each spoke
            hover_time: Time to hover at endpoints in seconds
            photo_interval: Time interval between photos in seconds
        """
        super().__init__(altitude, speed)
        self.num_spokes = num_spokes
        self.spoke_length = spoke_length
        self.spoke_pattern = spoke_pattern.lower()
        self.return_to_center = return_to_center
        self.hover_at_endpoints = hover_at_endpoints
        self.hover_time = hover_time
        self.photo_interval = photo_interval
        
        # Mission tracking
        self.total_photos = 0
        self.spokes_completed = 0
        self.center_returns = 0
        self.last_photo_time = 0
        
        # Validate pattern
        valid_patterns = ["sequential", "random", "alternating", "crisscross"]
        if self.spoke_pattern not in valid_patterns:
            print(f" Invalid pattern '{spoke_pattern}'. Using 'sequential'.")
            self.spoke_pattern = "sequential"
        
        # Calculate spoke angles and pattern order
        self.spoke_angles = self._calculate_spoke_angles()
        self.flight_order = self._calculate_flight_order()
        
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\n STAR PATTERN MISSION PARAMETERS")
        print(f"Number of Spokes: {self.num_spokes}")
        print(f"Spoke Length: {self.spoke_length}m")
        print(f"Flight Pattern: {self.spoke_pattern.title()}")
        print(f"Return to Center: {'Yes' if self.return_to_center else 'No'}")
        print(f"Hover at Endpoints: {'Yes' if self.hover_at_endpoints else 'No'}")
        if self.hover_at_endpoints:
            print(f"Hover Time: {self.hover_time}s")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Photo Interval: {self.photo_interval}s")
        
        # Calculate pattern characteristics
        total_spoke_distance = self.spoke_length * self.num_spokes
        if self.return_to_center:
            total_distance = total_spoke_distance * 2  # Out and back
        else:
            total_distance = total_spoke_distance
        
        print(f"\n PATTERN CHARACTERISTICS")
        print(f"Total Spoke Distance: {total_spoke_distance:.0f}m")
        print(f"Total Flight Distance: {total_distance:.0f}m")
        print(f"Coverage Radius: {self.spoke_length}m")
        print(f"Coverage Area: {math.pi * self.spoke_length**2 / 10000:.1f} hectares")
        
        print(f"\n SPOKE DIRECTIONS:")
        for i, angle in enumerate(self.spoke_angles):
            compass = self._angle_to_compass(angle)
            order_index = self.flight_order.index(i) + 1
            print(f"  {order_index:2d}. Spoke {i+1}: {math.degrees(angle):6.1f}° ({compass})")
    
    def execute_mission_logic(self, target_z):
        """Execute the star pattern mission."""
        print(f" Executing {self.spoke_pattern} star pattern mission")
        
        # Get center position
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        print(f" Center position: ({center_x:.1f}, {center_y:.1f})")
        
        # Execute spokes in specified order
        for flight_index, spoke_index in enumerate(self.flight_order):
            print(f"\n Flying spoke {spoke_index + 1}/{self.num_spokes} (order: {flight_index + 1})")
            
            try:
                self._execute_single_spoke(center_x, center_y, target_z, spoke_index, flight_index + 1)
                self.spokes_completed += 1
                
            except Exception as e:
                print(f" Error in spoke {spoke_index + 1}: {str(e)}")
                if "Mission aborted" in str(e):
                    raise
                continue
        
        # Final return to center
        if not self.return_to_center:
            print(f"\n Final return to center")
            self._fly_to_center(center_x, center_y, target_z)
        
        print(f" Star pattern completed!")
        print(f" Mission Summary: {self.spokes_completed} spokes, {self.center_returns} center returns, {self.total_photos} photos")
    
    def _execute_single_spoke(self, center_x, center_y, target_z, spoke_index, flight_order):
        """Execute a single spoke of the star pattern."""
        angle = self.spoke_angles[spoke_index]
        compass = self._angle_to_compass(angle)
        
        print(f"   Spoke direction: {math.degrees(angle):.1f}° ({compass})")
        
        # Calculate endpoint
        end_x = center_x + self.spoke_length * math.cos(angle)
        end_y = center_y + self.spoke_length * math.sin(angle)
        
        # Fly outward along spoke
        print(f"   Flying outward to ({end_x:.1f}, {end_y:.1f})")
        self._fly_spoke_smoothly(center_x, center_y, end_x, end_y, target_z, spoke_index, "outward")
        
        # Hover at endpoint if enabled
        if self.hover_at_endpoints:
            print(f"    Hovering at endpoint for {self.hover_time}s")
            self.client.hoverAsync().join()
            
            # Take photo at endpoint
            photo_result = self.take_photo(
                photo_dir="star_mission_photos",
                filename_prefix=f"spoke_{spoke_index+1}_endpoint"
            )
            if photo_result:
                self.total_photos += 1
                print(f"     Endpoint photo taken")
            
            time.sleep(self.hover_time)
        
        # Return to center if enabled
        if self.return_to_center:
            print(f"   Returning to center")
            self._fly_spoke_smoothly(end_x, end_y, center_x, center_y, target_z, spoke_index, "return")
            self.center_returns += 1
        
        print(f"   Spoke {spoke_index + 1} completed")
    
    def _fly_spoke_smoothly(self, start_x, start_y, end_x, end_y, target_z, spoke_index, direction):
        """Fly along a spoke with smooth velocity control and photo capture."""
        # Calculate spoke parameters
        dx = end_x - start_x
        dy = end_y - start_y
        spoke_distance = math.sqrt(dx*dx + dy*dy)
        
        if spoke_distance < 1.0:
            return
        
        # Calculate flight time and steps
        flight_time = spoke_distance / self.speed
        steps = max(20, int(spoke_distance / 5))  # Step every 5m or minimum 20 steps
        
        print(f"    Distance: {spoke_distance:.1f}m, Time: {flight_time:.1f}s")
        
        # Smooth flight along spoke
        for step in range(steps + 1):
            # Check for emergency landing
            if self.emergency_landing_triggered:
                print(f"     Emergency landing triggered - aborting spoke")
                break
            
            progress = step / steps
            
            # Calculate current position along spoke
            current_x = start_x + dx * progress
            current_y = start_y + dy * progress
            
            # Move to position using smooth velocity control
            self._move_to_position_smooth(current_x, current_y, target_z)
            
            # Take photos at intervals
            self._take_photo_if_time(spoke_index, direction)
            
            # Update tracking
            self.update_distance_tracking()
            time.sleep(0.1)  # Smooth motion delay
    
    def _move_to_position_smooth(self, target_x, target_y, target_z):
        """Move to position using smooth velocity control."""
        try:
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate velocity vector
            dx = target_x - current_pos.x_val
            dy = target_y - current_pos.y_val
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0.5:  # Only move if distance is significant
                # Calculate velocity components
                vx = (dx / distance) * self.speed
                vy = (dy / distance) * self.speed
                
                # Apply velocity control
                self.client.moveByVelocityAsync(vx, vy, 0, 0.2)
            
        except Exception as e:
            self.errors.append(f"Smooth movement error: {str(e)}")
    
    def _fly_to_center(self, center_x, center_y, target_z):
        """Fly directly to center position."""
        self.client.moveToPositionAsync(center_x, center_y, target_z, self.speed).join()
    
    def _take_photo_if_time(self, spoke_index, direction):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            photo_result = self.take_photo(
                photo_dir="star_mission_photos",
                filename_prefix=f"spoke_{spoke_index+1}_{direction}"
            )
            if photo_result:
                self.total_photos += 1
                self.last_photo_time = current_time
    
    def _calculate_spoke_angles(self):
        """Calculate angles for each spoke evenly distributed around 360°."""
        angles = []
        for i in range(self.num_spokes):
            angle = (i * 2 * math.pi) / self.num_spokes
            angles.append(angle)
        return angles
    
    def _calculate_flight_order(self):
        """Calculate the order to fly spokes based on pattern type."""
        spoke_indices = list(range(self.num_spokes))
        
        if self.spoke_pattern == "sequential":
            return spoke_indices
        
        elif self.spoke_pattern == "random":
            random.shuffle(spoke_indices)
            return spoke_indices
        
        elif self.spoke_pattern == "alternating":
            # Alternate between opposite sides
            order = []
            for i in range(self.num_spokes // 2):
                order.append(i)
                opposite = (i + self.num_spokes // 2) % self.num_spokes
                order.append(opposite)
            # Add any remaining spokes for odd numbers
            if self.num_spokes % 2 == 1:
                order.append(self.num_spokes - 1)
            return order
        
        elif self.spoke_pattern == "crisscross":
            # Jump to opposite or diagonal spokes
            order = []
            step = max(2, self.num_spokes // 3)
            current = 0
            for _ in range(self.num_spokes):
                order.append(current)
                current = (current + step) % self.num_spokes
                # Avoid duplicates
                while current in order and len(order) < self.num_spokes:
                    current = (current + 1) % self.num_spokes
            return order
        
        return spoke_indices  # Fallback to sequential
    
    def _angle_to_compass(self, angle_rad):
        """Convert angle in radians to compass direction."""
        angle_deg = math.degrees(angle_rad) % 360
        
        directions = [
            "E", "ENE", "NE", "NNE", "N", "NNW", "NW", "WNW",
            "W", "WSW", "SW", "SSW", "S", "SSE", "SE", "ESE"
        ]
        
        index = int((angle_deg + 11.25) / 22.5) % 16
        return directions[index]
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        total_spoke_distance = self.spoke_length * self.num_spokes
        
        if self.return_to_center:
            total_distance = total_spoke_distance * 2  # Out and back
        else:
            total_distance = total_spoke_distance
        
        # Add buffer for takeoff/landing and positioning
        buffer_distance = 200
        
        return total_distance + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        return {
            "Spokes Completed": f"{self.spokes_completed}/{self.num_spokes}",
            "Center Returns": self.center_returns,
            "Total Photos": self.total_photos,
            "Pattern Type": self.spoke_pattern.title(),
            "Coverage Radius": f"{self.spoke_length}m",
            "Coverage Area": f"{math.pi * self.spoke_length**2 / 10000:.1f} hectares"
        }


def main():
    """Main function to run the star pattern mission."""
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Star Pattern Mission for Radial Area Coverage"
    
    # Add star pattern specific parameters
    parser.add_argument('--num_spokes', type=int, default=8,
                       help='Number of radiating spokes (default: 8)')
    parser.add_argument('--spoke_length', type=float, default=400,
                       help='Length of each spoke in meters (default: 400)')
    parser.add_argument('--spoke_pattern', type=str, default='sequential',
                       choices=['sequential', 'random', 'alternating', 'crisscross'],
                       help='Pattern order for flying spokes (default: sequential)')
    parser.add_argument('--return_to_center', action='store_true',
                       help='Return to center between each spoke (default: False)')
    parser.add_argument('--hover_at_endpoints', action='store_true',
                       help='Hover at the end of each spoke (default: False)')
    parser.add_argument('--hover_time', type=float, default=3.0,
                       help='Time to hover at endpoints in seconds (default: 3.0)')
    parser.add_argument('--photo_interval', type=float, default=2.0,
                       help='Time interval between photos in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    # Create and configure mission
    mission = StarPatternMission(
        num_spokes=args.num_spokes,
        spoke_length=args.spoke_length,
        altitude=args.altitude,
        speed=args.speed,
        spoke_pattern=args.spoke_pattern,
        return_to_center=args.return_to_center,
        hover_at_endpoints=args.hover_at_endpoints,
        hover_time=args.hover_time,
        photo_interval=args.photo_interval
    )
    
    # Preview mode
    if args.preview:
        print(" MISSION PREVIEW MODE")
        mission.print_mission_parameters()
        
        planned_distance = mission.get_planned_distance()
        estimated_time = planned_distance / args.speed
        
        print(f"\n MISSION ESTIMATES")
        print(f"Planned Distance: {planned_distance:.0f}m")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        print(f"Expected Photos: {int(estimated_time / args.photo_interval)}")
        
        if args.hover_at_endpoints:
            total_hover_time = args.num_spokes * args.hover_time
            print(f"Total Hover Time: {total_hover_time/60:.1f} minutes")
        
        print(f"\n To execute the mission, run without --preview flag")
        return
    
    # Execute mission
    print(" Starting Star Pattern Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 