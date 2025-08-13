#!/usr/bin/env python3

import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class WaveMission(GenericMission):
    """
    WaveMission flies sinusoidal wave patterns for smooth area coverage.
    Features configurable wave parameters and direction for beautiful curved flight paths.
    """
    
    def __init__(self, area_width=800, area_height=400, altitude=20, speed=5,
                 wave_amplitude=50, wave_frequency=3, wave_direction="horizontal",
                 photo_interval=3.0, return_passes=1):
        """
        Initialize the wave pattern mission.
        
        Args:
            area_width: Total area width in meters
            area_height: Total area height in meters
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            wave_amplitude: Height of wave peaks in meters
            wave_frequency: Number of complete waves across area
            wave_direction: "horizontal" or "vertical" wave orientation
            photo_interval: Time interval between photos in seconds
            return_passes: Number of return passes (1 = single pass, 2 = there and back)
        """
        super().__init__(altitude, speed)
        self.area_width = area_width
        self.area_height = area_height
        self.wave_amplitude = wave_amplitude
        self.wave_frequency = wave_frequency
        self.wave_direction = wave_direction.lower()
        self.photo_interval = photo_interval
        self.return_passes = return_passes
        
        # Mission tracking
        self.total_photos = 0
        self.waves_completed = 0
        self.passes_completed = 0
        self.last_photo_time = 0
        
        # Validate direction
        if self.wave_direction not in ["horizontal", "vertical"]:
            print(f" Invalid direction '{wave_direction}'. Using 'horizontal'.")
            self.wave_direction = "horizontal"
        
        # Calculate wave parameters
        self._calculate_wave_parameters()
        
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\n WAVE PATTERN MISSION PARAMETERS")
        print(f"Coverage Area: {self.area_width}m x {self.area_height}m")
        print(f"Wave Direction: {self.wave_direction.title()}")
        print(f"Wave Amplitude: {self.wave_amplitude}m")
        print(f"Wave Frequency: {self.wave_frequency} waves")
        print(f"Return Passes: {self.return_passes}")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Photo Interval: {self.photo_interval}s")
        
        # Calculate pattern characteristics
        total_distance = self._calculate_total_distance()
        estimated_time = total_distance / self.speed
        
        print(f"\n WAVE CHARACTERISTICS")
        print(f"Wave Length: {self.wave_length:.1f}m")
        print(f"Wave Period: {self.wave_period:.1f}m")
        print(f"Total Distance: {total_distance:.0f}m")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        print(f"Coverage Efficiency: {self._calculate_coverage_efficiency():.1f}%")
        
    def execute_mission_logic(self, target_z):
        """Execute the wave pattern mission."""
        print(f" Executing {self.wave_direction} wave pattern mission")
        
        # Calculate starting position (corner of area)
        center_x = self.home_position.x_val
        center_y = self.home_position.y_val
        
        start_x = center_x - self.area_width / 2
        start_y = center_y - self.area_height / 2
        
        print(f" Moving to wave pattern start: ({start_x:.1f}, {start_y:.1f})")
        self.client.moveToPositionAsync(start_x, start_y, target_z, self.speed).join()
        
        # Execute wave passes
        for pass_num in range(self.return_passes):
            print(f"\n Starting wave pass {pass_num + 1}/{self.return_passes}")
            
            try:
                if pass_num % 2 == 0:  # Forward pass
                    self._execute_wave_pass(start_x, start_y, target_z, pass_num + 1, "forward")
                else:  # Reverse pass
                    self._execute_wave_pass(start_x, start_y, target_z, pass_num + 1, "reverse")
                
                self.passes_completed += 1
                
            except Exception as e:
                print(f" Error in pass {pass_num + 1}: {str(e)}")
                if "Mission aborted" in str(e):
                    raise
                continue
        
        print(f" Wave pattern completed!")
        print(f" Mission Summary: {self.passes_completed} passes, {self.waves_completed:.1f} waves, {self.total_photos} photos")
    
    def _execute_wave_pass(self, start_x, start_y, target_z, pass_num, direction):
        """Execute a single wave pass across the area."""
        print(f"   Flying {direction} wave pass...")
        
        # Calculate wave parameters based on direction
        if self.wave_direction == "horizontal":
            primary_distance = self.area_width
            primary_start = start_x
            secondary_start = start_y
            wave_axis = "x"
        else:  # vertical
            primary_distance = self.area_height
            primary_start = start_y
            secondary_start = start_x
            wave_axis = "y"
        
        # Calculate total time for the pass
        pass_time = primary_distance / self.speed
        
        # Adjust direction for reverse passes
        if direction == "reverse":
            primary_start = primary_start + primary_distance
            primary_distance = -primary_distance
        
        print(f"    Pass distance: {abs(primary_distance):.1f}m, Time: {pass_time:.1f}s")
        
        start_time = time.time()
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if pass is complete
            if elapsed_time >= pass_time:
                break
            
            # Check for emergency landing
            if self.emergency_landing_triggered:
                print(f"   Emergency landing triggered - aborting wave pass")
                break
            
            # Calculate progress along primary axis (0 to 1)
            progress = elapsed_time / pass_time
            
            # Calculate position along primary axis
            primary_pos = primary_start + (primary_distance * progress)
            
            # Calculate wave offset using sinusoidal function
            wave_phase = progress * self.wave_frequency * 2 * math.pi
            wave_offset = self.wave_amplitude * math.sin(wave_phase)
            
            # Calculate secondary axis position (with wave offset)
            secondary_pos = secondary_start + (self.area_height / 2) + wave_offset
            
            # Convert to x,y coordinates based on wave direction
            if self.wave_direction == "horizontal":
                target_x = primary_pos
                target_y = secondary_pos
            else:  # vertical
                target_x = secondary_pos
                target_y = primary_pos
            
            # Move to target position using smooth velocity control
            self._move_to_wave_position(target_x, target_y, target_z)
            
            # Take photos at intervals
            self._take_photo_if_time(pass_num, direction)
            
            # Update wave completion tracking
            current_waves = progress * self.wave_frequency
            self.waves_completed = max(self.waves_completed, current_waves)
            
            # High-frequency control for smooth waves
            time.sleep(0.02)  # 50Hz for smooth sinusoidal motion
            self.update_distance_tracking()
        
        print(f"   {direction.title()} wave pass completed")
    
    def _move_to_wave_position(self, target_x, target_y, target_z):
        """Move to wave position using smooth velocity control."""
        try:
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate velocity vector for smooth wave motion
            dx = target_x - current_pos.x_val
            dy = target_y - current_pos.y_val
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0.3:  # Only move if distance is significant
                # Calculate velocity components
                vx = (dx / distance) * self.speed
                vy = (dy / distance) * self.speed
                
                # Apply smooth velocity control with shorter duration for responsive waves
                self.client.moveByVelocityAsync(vx, vy, 0, 0.05)
            
        except Exception as e:
            self.errors.append(f"Wave movement error: {str(e)}")
    
    def _take_photo_if_time(self, pass_num, direction):
        """Take a photo if enough time has passed since the last photo."""
        current_time = time.time()
        
        if current_time - self.last_photo_time >= self.photo_interval:
            photo_result = self.take_photo(
                photo_dir="wave_mission_photos",
                filename_prefix=f"pass_{pass_num}_{direction}"
            )
            if photo_result:
                self.total_photos += 1
                self.last_photo_time = current_time
    
    def _calculate_wave_parameters(self):
        """Calculate wave-specific parameters."""
        if self.wave_direction == "horizontal":
            self.wave_length = self.area_width / self.wave_frequency
        else:  # vertical
            self.wave_length = self.area_height / self.wave_frequency
        
        # Wave period is the distance for one complete wave cycle
        self.wave_period = self.wave_length
    
    def _calculate_total_distance(self):
        """Calculate total distance for the wave pattern."""
        # Base distance is the primary axis distance for each pass
        if self.wave_direction == "horizontal":
            base_distance = self.area_width * self.return_passes
        else:
            base_distance = self.area_height * self.return_passes
        
        # Add extra distance for wave undulations (approximate)
        wave_factor = 1 + (self.wave_amplitude / 100)  # Rough approximation
        total_distance = base_distance * wave_factor
        
        return total_distance
    
    def _calculate_coverage_efficiency(self):
        """Calculate coverage efficiency compared to straight line."""
        straight_distance = (self.area_width if self.wave_direction == "horizontal" 
                            else self.area_height) * self.return_passes
        wave_distance = self._calculate_total_distance()
        
        return (straight_distance / wave_distance) * 100
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        total_wave_distance = self._calculate_total_distance()
        
        # Add buffer for takeoff/landing and positioning
        buffer_distance = 200
        
        return total_wave_distance + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        return {
            "Passes Completed": f"{self.passes_completed}/{self.return_passes}",
            "Waves Completed": f"{self.waves_completed:.1f}/{self.wave_frequency * self.return_passes}",
            "Total Photos": self.total_photos,
            "Wave Direction": self.wave_direction.title(),
            "Wave Amplitude": f"{self.wave_amplitude}m",
            "Coverage Area": f"{self.area_width}m x {self.area_height}m",
            "Coverage Efficiency": f"{self._calculate_coverage_efficiency():.1f}%"
        }


def main():
    """Main function to run the wave pattern mission."""
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Wave Pattern Mission for Smooth Area Coverage"
    
    # Add wave pattern specific parameters
    parser.add_argument('--area_width', type=float, default=800,
                       help='Total area width in meters (default: 800)')
    parser.add_argument('--area_height', type=float, default=400,
                       help='Total area height in meters (default: 400)')
    parser.add_argument('--wave_amplitude', type=float, default=50,
                       help='Height of wave peaks in meters (default: 50)')
    parser.add_argument('--wave_frequency', type=int, default=3,
                       help='Number of complete waves across area (default: 3)')
    parser.add_argument('--wave_direction', type=str, default='horizontal',
                       choices=['horizontal', 'vertical'],
                       help='Wave orientation: horizontal or vertical (default: horizontal)')
    parser.add_argument('--return_passes', type=int, default=1,
                       help='Number of passes (1=single, 2=there and back) (default: 1)')
    parser.add_argument('--photo_interval', type=float, default=3.0,
                       help='Time interval between photos in seconds (default: 3.0)')
    
    args = parser.parse_args()
    
    # Create and configure mission
    mission = WaveMission(
        area_width=args.area_width,
        area_height=args.area_height,
        altitude=args.altitude,
        speed=args.speed,
        wave_amplitude=args.wave_amplitude,
        wave_frequency=args.wave_frequency,
        wave_direction=args.wave_direction,
        photo_interval=args.photo_interval,
        return_passes=args.return_passes
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
        print(f"Wave Pattern: {args.wave_frequency} waves, {args.wave_amplitude}m amplitude")
        
        print(f"\n To execute the mission, run without --preview flag")
        return
    
    # Execute mission
    print(" Starting Wave Pattern Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 