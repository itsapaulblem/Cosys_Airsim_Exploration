#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class Figure8Mission(GenericMission):
    """
    Figure8Mission flies continuous figure-8 (infinity symbol) patterns for surveillance and tracking.
    Features smooth velocity-based motion and photo capture at pattern crossings.
    """
    
    def __init__(self, loop_width=300, loop_height=200, altitude=20, speed=5, 
                 crossing_angle=45, loops_count=3, photo_at_crossings=True, 
                 photo_interval=5.0):
        """
        Initialize the figure-8 mission.
        
        Args:
            loop_width: Width of each loop in meters
            loop_height: Height of each loop in meters  
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            crossing_angle: Angle between loops in degrees (0-90)
            loops_count: Number of complete figure-8 patterns to fly
            photo_at_crossings: Take photos at center crossings
            photo_interval: Time interval between photos in seconds
        """
        super().__init__(altitude, speed)
        self.loop_width = loop_width
        self.loop_height = loop_height
        self.crossing_angle = math.radians(crossing_angle)
        self.loops_count = loops_count
        self.photo_at_crossings = photo_at_crossings
        self.photo_interval = photo_interval
        
        # Mission tracking
        self.total_photos = 0
        self.loops_completed = 0
        self.crossings_completed = 0
        self.last_photo_time = 0
        
        # Calculate figure-8 parameters
        self.semi_width = loop_width / 2
        self.semi_height = loop_height / 2
        
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\n FIGURE-8 MISSION PARAMETERS")
        print(f"Loop Dimensions: {self.loop_width}m x {self.loop_height}m")
        print(f"Crossing Angle: {math.degrees(self.crossing_angle):.1f}°")
        print(f"Number of Loops: {self.loops_count}")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Photo at Crossings: {'Yes' if self.photo_at_crossings else 'No'}")
        print(f"Photo Interval: {self.photo_interval}s")
        
        # Calculate pattern characteristics
        single_loop_length = self._calculate_single_loop_length()
        total_length = single_loop_length * self.loops_count
        estimated_time = total_length / self.speed
        
        print(f"\n PATTERN CHARACTERISTICS")
        print(f"Single Loop Length: {single_loop_length:.0f}m")
        print(f"Total Pattern Length: {total_length:.0f}m")
        print(f"Estimated Flight Time: {estimated_time/60:.1f} minutes")
        print(f"Expected Crossings: {self.loops_count}")
        
    def execute_mission_logic(self, target_z):
        """Execute the figure-8 mission pattern."""
        print(f" Executing Figure-8 infinity pattern mission")
        
        # Move to starting position (center)
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        print(f" Moving to pattern center position")
        self.client.moveToPositionAsync(center_x, center_y, target_z, self.speed).join()
        
        # Execute figure-8 loops
        for loop_num in range(self.loops_count):
            print(f"\n∞ Starting Figure-8 loop {loop_num + 1}/{self.loops_count}")
            
            try:
                self._execute_single_figure8(center_x, center_y, target_z, loop_num + 1)
                self.loops_completed += 1
                
            except Exception as e:
                print(f" Error in loop {loop_num + 1}: {str(e)}")
                if "Mission aborted" in str(e):
                    raise
                continue
        
        print(f" Figure-8 pattern completed!")
        print(f" Mission Summary: {self.loops_completed} loops, {self.crossings_completed} crossings, {self.total_photos} photos")
    
    def _execute_single_figure8(self, center_x, center_y, target_z, loop_num):
        """Execute a single figure-8 pattern using smooth parametric equations."""
        print(f"   Flying smooth figure-8 pattern...")
        
        # Figure-8 parametric equations: 
        # x = a * sin(t)
        # y = b * sin(t) * cos(t) = (b/2) * sin(2t)
        
        # Calculate time for one complete figure-8
        single_loop_length = self._calculate_single_loop_length()
        loop_duration = single_loop_length / self.speed
        
        print(f"    Loop duration: {loop_duration:.1f}s")
        
        start_time = time.time()
        crossing_detected = False
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if loop is complete
            if elapsed_time >= loop_duration:
                break
            
            # Check for emergency landing
            if self.emergency_landing_triggered:
                print(f"   Emergency landing triggered - aborting figure-8")
                break
            
            # Calculate parameter t (0 to 4π for complete figure-8)
            t = (elapsed_time / loop_duration) * 4 * math.pi
            
            # Figure-8 parametric equations with crossing angle
            x_offset = self.semi_width * math.sin(t)
            y_offset = self.semi_height * math.sin(t) * math.cos(t)
            
            # Apply crossing angle rotation
            cos_angle = math.cos(self.crossing_angle)
            sin_angle = math.sin(self.crossing_angle)
            
            rotated_x = x_offset * cos_angle - y_offset * sin_angle
            rotated_y = x_offset * sin_angle + y_offset * cos_angle
            
            # Calculate target position
            target_x = center_x + rotated_x
            target_y = center_y + rotated_y
            
            # Get current position for velocity calculation
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate velocity vector for smooth motion
            dx = target_x - current_pos.x_val
            dy = target_y - current_pos.y_val
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0.5:  # Only apply velocity if distance is significant
                # Calculate velocity components
                vx = (dx / distance) * self.speed
                vy = (dy / distance) * self.speed
                
                # Apply smooth velocity control
                self.client.moveByVelocityAsync(vx, vy, 0, 0.1)
            
            # Detect crossing (when we're near center)
            center_distance = math.sqrt(
                (current_pos.x_val - center_x)**2 + 
                (current_pos.y_val - center_y)**2
            )
            
            # Check for crossing point and photo opportunity
            if center_distance < 5.0 and not crossing_detected:  # Within 5m of center
                crossing_detected = True
                self.crossings_completed += 1
                print(f"     Crossing detected at center (#{self.crossings_completed})")
                
                if self.photo_at_crossings:
                    photo_result = self.take_photo(
                        photo_dir="figure8_mission_photos",
                        filename_prefix=f"loop_{loop_num}_crossing_{self.crossings_completed}"
                    )
                    if photo_result:
                        self.total_photos += 1
                        print(f"     Crossing photo taken")
            elif center_distance > 15.0:  # Reset crossing detection when far from center
                crossing_detected = False
            
            # Take interval photos
            if not self.photo_at_crossings:
                self._take_photo_if_time(loop_num)
            
            # High-frequency control loop
            time.sleep(0.02)  # 50Hz for smooth motion
            self.update_distance_tracking()
        
        # Gentle stop at end of loop
        self.client.moveByVelocityAsync(0, 0, 0, 0.2).join()
        print(f"   Figure-8 loop {loop_num} completed")
    
    def _take_photo_if_time(self, loop_num):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            photo_result = self.take_photo(
                photo_dir="figure8_mission_photos",
                filename_prefix=f"loop_{loop_num}_interval"
            )
            if photo_result:
                self.total_photos += 1
                self.last_photo_time = current_time
    
    def _calculate_single_loop_length(self):
        """Calculate approximate length of a single figure-8 loop."""
        # Approximate length using ellipse perimeter formula
        # For figure-8, it's roughly 2 times an ellipse perimeter
        a = self.semi_width
        b = self.semi_height
        
        # Ramanujan's approximation for ellipse perimeter
        ellipse_perimeter = math.pi * (3*(a + b) - math.sqrt((3*a + b)*(a + 3*b)))
        
        # Figure-8 is approximately 2.5 times ellipse perimeter
        return ellipse_perimeter * 2.5
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        single_loop_length = self._calculate_single_loop_length()
        total_pattern_distance = single_loop_length * self.loops_count
        
        # Add buffer for takeoff/landing and positioning
        buffer_distance = 200
        
        return total_pattern_distance + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        return {
            "Loops Completed": f"{self.loops_completed}/{self.loops_count}",
            "Crossings Detected": self.crossings_completed,
            "Total Photos": self.total_photos,
            "Loop Dimensions": f"{self.loop_width}m x {self.loop_height}m",
            "Crossing Angle": f"{math.degrees(self.crossing_angle):.1f}°"
        }


def main():
    """Main function to run the figure-8 mission."""
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Figure-8 Mission for Surveillance and Tracking"
    
    # Add figure-8 specific parameters
    parser.add_argument('--loop_width', type=float, default=300,
                       help='Width of each loop in meters (default: 300)')
    parser.add_argument('--loop_height', type=float, default=200,
                       help='Height of each loop in meters (default: 200)')
    parser.add_argument('--crossing_angle', type=float, default=45,
                       help='Angle between loops in degrees (default: 45)')
    parser.add_argument('--loops_count', type=int, default=3,
                       help='Number of complete figure-8 patterns (default: 3)')
    parser.add_argument('--photo_at_crossings', action='store_true',
                       help='Take photos at pattern crossings (default: False)')
    parser.add_argument('--photo_interval', type=float, default=5.0,
                       help='Time interval between photos in seconds (default: 5.0)')
    
    args = parser.parse_args()
    
    # Create and configure mission
    mission = Figure8Mission(
        loop_width=args.loop_width,
        loop_height=args.loop_height,
        altitude=args.altitude,
        speed=args.speed,
        crossing_angle=args.crossing_angle,
        loops_count=args.loops_count,
        photo_at_crossings=args.photo_at_crossings,
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
        
        if args.photo_at_crossings:
            print(f"Expected Photos: {args.loops_count} (at crossings)")
        else:
            print(f"Expected Photos: {int(estimated_time / args.photo_interval)} (interval-based)")
        
        print(f"\n To execute the mission, run without --preview flag")
        return
    
    # Execute mission
    print(" Starting Figure-8 Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 