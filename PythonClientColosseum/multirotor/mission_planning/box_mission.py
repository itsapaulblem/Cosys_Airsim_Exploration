#!/usr/bin/env python3
import math
import time
import argparse
from generic_mission import GenericMission, create_standard_argument_parser

class BoxMission(GenericMission):
    """
    BoxMission flies a rectangular box pattern with orbital photography at each vertex.
    Includes smooth velocity-based movement and proper box completion.
    """
    
    def __init__(self, box_size=400, altitude=20, speed=5, orbit_radius=50, 
                 orbit_speed=3, photos_per_orbit=8, orbit_mode="velocity", max_collisions=5, 
                 enable_orbits=True):
        """
        Initialize the box mission.
        
        Args:
            box_size: Size of the box in meters
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            orbit_radius: Radius for orbital photography at vertices
            orbit_speed: Speed during orbital photography
            photos_per_orbit: Number of photos to take during each orbit
            orbit_mode: Orbital motion mode - "waypoint" (stable) or "velocity" (smooth)
            max_collisions: Maximum collisions before emergency landing
            enable_orbits: Whether to perform orbital photography at vertices (True) or just fly box pattern (False)
        """
        super().__init__(altitude, speed)
        self.box_size = box_size
        self.orbit_radius = orbit_radius
        self.orbit_speed = orbit_speed
        self.photos_per_orbit = photos_per_orbit
        self.orbit_mode = orbit_mode.lower()
        self.enable_orbits = enable_orbits
        
        # Set collision limit
        self.max_collisions = max_collisions
        
        # Validate orbit mode
        if self.orbit_mode not in ["waypoint", "velocity"]:
            print(f"Invalid orbit mode '{orbit_mode}'. Using 'waypoint' mode.")
            self.orbit_mode = "waypoint"
        
        # Mission tracking
        self.vertices_visited = 0
        self.total_photos = 0
        self.orbits_completed = 0
        
        # Calculate box vertices (relative to home position)
        half_size = box_size / 2
        self.vertices = [
            (-half_size, -half_size),  # Bottom-Left
            (half_size, -half_size),   # Bottom-Right
            (half_size, half_size),    # Top-Right
            (-half_size, half_size)    # Top-Left
        ]
        
        self.vertex_names = ["Bottom-Left", "Bottom-Right", "Top-Right", "Top-Left"]
        
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\nBOX MISSION PARAMETERS")
        print(f"Box Size: {self.box_size}m x {self.box_size}m")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Orbital Photography: {'ENABLED' if self.enable_orbits else 'DISABLED'}")
        
        if self.enable_orbits:
            print(f"Orbit Radius: {self.orbit_radius}m")
            print(f"Orbit Speed: {self.orbit_speed}m/s")
            print(f"Orbit Mode: {self.orbit_mode.title()} ({'Smooth continuous motion' if self.orbit_mode == 'velocity' else 'Stable waypoint transitions'})")
            print(f"Photos per Orbit: {self.photos_per_orbit}")
            print(f"Collision Detection: Enabled for transit, disabled during orbital photography")
        else:
            print(f"Mission Type: Simple box pattern flight (no orbital photography)")
            print(f"Collision Detection: Enabled throughout mission")
        
        print(f"Collision Safety Limit: {self.max_collisions} collisions")
        
        print(f"\nPLANNED VERTICES:")
        for i, (x, y) in enumerate(self.vertices):
            print(f"  {i+1}. {self.vertex_names[i]}: ({x:+.1f}, {y:+.1f})")
    
    def execute_mission_logic(self, target_z):
        """Execute the box mission: fly to each vertex and perform orbital photography."""
        print(f"Executing box pattern mission")
        
        # Fly to each vertex and perform orbital photography
        for i, (vertex_x, vertex_y) in enumerate(self.vertices):
            # Check for emergency landing before each vertex
            if self.emergency_landing_triggered:
                print(f"\nEmergency landing triggered - aborting mission at vertex {i+1}")
                break
            
            print(f"\nFlying to vertex {i+1}/4: {self.vertex_names[i]}")
            
            # Calculate absolute position (relative to home)
            abs_x = self.home_position.x_val + vertex_x
            abs_y = self.home_position.y_val + vertex_y
            
            try:
                # Fly to vertex using smooth movement
                self._fly_to_position(abs_x, abs_y, target_z)
                
                # Check again before potential orbital photography
                if self.emergency_landing_triggered:
                    print(f"Emergency landing triggered - skipping vertex {i+1}")
                    break
                
                # Perform orbital photography if enabled
                if self.enable_orbits:
                    self._perform_orbital_photography(abs_x, abs_y, target_z, i+1)
                else:
                    print(f"  Vertex {i+1} reached - continuing to next vertex (orbital photography disabled)")
                
                self.vertices_visited += 1
                self.update_distance_tracking()
                
            except Exception as vertex_error:
                print(f"Error at vertex {i+1}: {str(vertex_error)}")
                # If this is a mission abort exception, re-raise it
                if "Mission aborted" in str(vertex_error):
                    raise
                # Otherwise, try to continue to next vertex
                continue
        
        # Complete the box by returning to first vertex
        print(f"\nCompleting box pattern - returning to first vertex")
        first_vertex_x = self.home_position.x_val + self.vertices[0][0]
        first_vertex_y = self.home_position.y_val + self.vertices[0][1]
        self._fly_to_position(first_vertex_x, first_vertex_y, target_z)
        
        print(f"Box pattern completed!")
        if self.enable_orbits:
            print(f"Mission Summary: {self.vertices_visited} vertices visited, {self.orbits_completed} orbits completed, {self.total_photos} photos taken")
        else:
            print(f"Mission Summary: {self.vertices_visited} vertices visited, simple box pattern flight completed")
    
    def _fly_to_position(self, x, y, z):
        """Fly to a specific position using smooth velocity-based movement."""
        try:
            print(f"  Flying to position ({x:.1f}, {y:.1f}, {z:.1f})")
            
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
                print(f"Position error: {distance_error:.1f}m")
                self.errors.append(f"Position error at ({x:.1f}, {y:.1f}): {distance_error:.1f}m")
            else:
                print(f"  Arrived at position (error: {distance_error:.1f}m)")
            
        except Exception as e:
            error_msg = f"Failed to fly to position ({x:.1f}, {y:.1f}): {str(e)}"
            print(f"  {error_msg}")
            self.errors.append(error_msg)
    
    def _perform_orbital_photography(self, center_x, center_y, center_z, vertex_num):
        """Perform orbital photography around a vertex using selected mode."""
        print(f"  Starting orbital photography at vertex {vertex_num} ({self.orbit_mode} mode)")
        
        try:
            # Move to orbit starting position first (with collision detection enabled)
            start_x = center_x + self.orbit_radius
            start_y = center_y
            print(f"  Moving to orbit start position")
            self._fly_to_position(start_x, start_y, center_z)
            
            # DISABLE collision detection for orbital maneuvers
            print(f"  Disabling collision detection for orbital photography")
            self.disable_collision_detection()
            
            # Choose orbital method based on mode
            if self.orbit_mode == "waypoint":
                self._perform_waypoint_orbit(center_x, center_y, center_z, vertex_num)
            else:  # velocity mode
                self._perform_velocity_orbit(center_x, center_y, center_z, vertex_num)
            
        except Exception as e:
            error_msg = f"Orbital photography failed at vertex {vertex_num}: {str(e)}"
            print(f"  {error_msg}")
            self.errors.append(error_msg)
        finally:
            # ALWAYS re-enable collision detection after orbital photography
            print(f"  Re-enabling collision detection")
            self.enable_collision_detection()
    
    def _perform_waypoint_orbit(self, center_x, center_y, center_z, vertex_num):
        """Perform waypoint-based orbital photography with smooth transitions (stable, no tilting)."""
        # Calculate orbital parameters
        orbit_circumference = 2 * math.pi * self.orbit_radius
        
        # Use more waypoints for smoother motion, but not too many to avoid jerkiness
        base_waypoints = max(16, int(orbit_circumference / 8))  # Waypoint every ~8 meters for smoother motion
        num_waypoints = min(base_waypoints, 32)  # Cap at 32 waypoints to avoid excessive stopping
        photo_waypoints = max(1, num_waypoints // self.photos_per_orbit)  # Photo interval
        
        print(f"Starting SMOOTH WAYPOINT orbital motion with {num_waypoints} waypoints")
        print(f"Using slower speed and curved transitions for smoothness")
        
        photos_taken = 0
        
        # Use slower speed for smoother waypoint transitions
        smooth_orbit_speed = min(self.orbit_speed * 0.7, 2.0)  # 30% slower, max 2 m/s
        
        # Generate orbital waypoints
        for i in range(num_waypoints + 1):  # +1 to complete full circle
            # Check for emergency landing (collision detection is disabled, but check emergency state)
            if self.emergency_landing_triggered:
                print(f"  Emergency landing triggered - aborting waypoint orbit")
                break
            
            # Calculate angle for this waypoint
            angle = (i / num_waypoints) * 2 * math.pi
            
            # Calculate waypoint position
            waypoint_x = center_x + self.orbit_radius * math.cos(angle)
            waypoint_y = center_y + self.orbit_radius * math.sin(angle)
            
            # Move to waypoint using smooth position control
            if i % 4 == 0 or photos_taken < self.photos_per_orbit:  # Log every 4th waypoint or when taking photos
                print(f"Waypoint {i+1}/{num_waypoints}: ({waypoint_x:.1f}, {waypoint_y:.1f})")
            
            try:
                # Use smooth position-based movement with gentle speed
                self.client.moveToPositionAsync(
                    waypoint_x, waypoint_y, center_z, 
                    smooth_orbit_speed
                ).join()
                
                # Point towards center with smooth yaw transition
                self._point_towards_center_stable(waypoint_x, waypoint_y, center_x, center_y)
                
                # Take photo at specified intervals
                if (i % photo_waypoints == 0 and 
                    photos_taken < self.photos_per_orbit and 
                    not self.emergency_landing_triggered):
                    
                    # Small stabilization pause before photo
                    time.sleep(0.3)
                    
                    photo_result = self.take_photo(photo_dir="box_mission_photos", 
                                   filename_prefix=f"vertex_{vertex_num}_photo_{photos_taken+1}")
                    if photo_result:
                        photos_taken += 1
                        self.total_photos += 1
                        print(f"    Photo {photos_taken}/{self.photos_per_orbit} taken at waypoint {i+1}")
                
                # Minimal pause for smooth flow (reduced from 0.2s)
                time.sleep(0.05)
                self.update_distance_tracking()
                
            except Exception as waypoint_error:
                print(f"    Waypoint {i+1} movement failed: {str(waypoint_error)}")
                # If waypoint movement fails, try to continue with next waypoint
                continue
        
        self.orbits_completed += 1
        print(f"  SMOOTH WAYPOINT orbital photography completed: {photos_taken} photos taken")
    
    def _perform_velocity_orbit(self, center_x, center_y, center_z, vertex_num):
        """Perform velocity-based orbital photography (smooth continuous motion)."""
        # Calculate orbital parameters for smooth motion
        orbit_circumference = 2 * math.pi * self.orbit_radius
        orbit_duration = orbit_circumference / self.orbit_speed  # Time for one complete orbit
        photo_interval = orbit_duration / self.photos_per_orbit
        
        print(f"  Starting SMOOTH VELOCITY orbital motion")
        print(f"    Orbit duration: {orbit_duration:.1f}s, Photo interval: {photo_interval:.1f}s")
        
        photos_taken = 0
        start_time = time.time()
        last_photo_time = start_time
        
        # Initialize smooth orbital motion
        initial_angle = 0.0
        
        # Continuous velocity-based orbital motion with enhanced smoothness
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if orbit is complete (add small buffer for completion)
            if elapsed_time >= (orbit_duration + 0.5):
                break
            
            # Check for emergency landing (collision detection is disabled during orbit)
            if self.emergency_landing_triggered:
                print(f"  Emergency landing triggered - aborting orbit")
                break
            
            # Calculate smooth orbital progression
            progress = min(elapsed_time / orbit_duration, 1.0)  # Clamp to 1.0
            angle = initial_angle + (progress * 2 * math.pi)
            
            # Calculate smooth orbital position
            target_x = center_x + self.orbit_radius * math.cos(angle)
            target_y = center_y + self.orbit_radius * math.sin(angle)
            
            # Get current position for smooth velocity calculation
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate position error for smooth correction
            error_x = target_x - current_pos.x_val
            error_y = target_y - current_pos.y_val
            
            # Calculate tangential velocity (primary motion)
            tangent_vx = -self.orbit_speed * math.sin(angle)
            tangent_vy = self.orbit_speed * math.cos(angle)
            
            # Add position correction (secondary, for accuracy)
            correction_gain = 0.3  # Gentle correction
            correction_vx = error_x * correction_gain
            correction_vy = error_y * correction_gain
            
            # Combine tangential motion with gentle position correction
            final_vx = tangent_vx + correction_vx
            final_vy = tangent_vy + correction_vy
            
            # Limit maximum velocity for smoothness
            max_velocity = self.orbit_speed * 1.5
            velocity_magnitude = math.sqrt(final_vx**2 + final_vy**2)
            if velocity_magnitude > max_velocity:
                scale = max_velocity / velocity_magnitude
                final_vx *= scale
                final_vy *= scale
            
            # Apply smooth velocity command
            self.client.moveByVelocityAsync(final_vx, final_vy, 0, 0.05)  # Shorter duration for smoother control
            
            # Smooth camera pointing - less frequent updates for stability
            if int(elapsed_time * 4) % 8 == 0:  # Update every 2 seconds
                self._point_towards_center_smooth(target_x, target_y, center_x, center_y)
            
            # Take photos at specified intervals
            if ((current_time - last_photo_time) >= photo_interval and 
                photos_taken < self.photos_per_orbit and 
                not self.emergency_landing_triggered):
                
                photo_result = self.take_photo(photo_dir="box_mission_photos", 
                               filename_prefix=f"vertex_{vertex_num}_photo_{photos_taken+1}")
                if photo_result:
                    photos_taken += 1
                    self.total_photos += 1
                    last_photo_time = current_time
                    print(f"    Photo {photos_taken}/{self.photos_per_orbit} taken at {progress*100:.0f}% orbit")
            
            # High-frequency control loop for ultra-smooth motion
            time.sleep(0.01)  # 100Hz update rate for maximum smoothness
            self.update_distance_tracking()
        
        # Gentle stop - gradually reduce velocity
        print(f"  Completing orbit - gentle stop")
        for i in range(10):
            decel_factor = (10 - i) / 10  # Gradual deceleration
            self.client.moveByVelocityAsync(final_vx * decel_factor, final_vy * decel_factor, 0, 0.05)
            time.sleep(0.05)
        
        # Final stop
        self.client.moveByVelocityAsync(0, 0, 0, 0.1).join()
        
        self.orbits_completed += 1
        print(f"  SMOOTH orbital photography completed: {photos_taken} photos taken")
    
    def _point_towards_center_stable(self, current_x, current_y, center_x, center_y):
        """Point the drone towards the center with stable yaw control (waypoint mode)."""
        try:
            # Calculate angle to center
            dx = center_x - current_x
            dy = center_y - current_y
            target_yaw = math.degrees(math.atan2(dy, dx))
            
            # Use rotateToYawAsync for precise, stable yaw control
            self.client.rotateToYawAsync(target_yaw).join()
            
        except Exception as e:
            self.errors.append(f"Stable yaw rotation error: {str(e)}")
    
    def _point_towards_center_velocity(self, current_x, current_y, center_x, center_y):
        """Point the drone towards the center with smooth yaw control (velocity mode)."""
        try:
            # Calculate angle to center
            dx = center_x - current_x
            dy = center_y - current_y
            target_yaw = math.degrees(math.atan2(dy, dx))
            
            # Get current yaw
            current_state = self.client.getMultirotorState()
            current_yaw = math.degrees(current_state.kinematics_estimated.orientation.z_val)
            
            # Calculate yaw difference and normalize to [-180, 180]
            yaw_diff = target_yaw - current_yaw
            while yaw_diff > 180:
                yaw_diff -= 360
            while yaw_diff < -180:
                yaw_diff += 360
            
            # Only adjust yaw if difference is significant
            if abs(yaw_diff) > 15:  # Threshold for stability
                # Use gentle yaw rate for smooth motion
                yaw_rate = yaw_diff * 0.3  # Reduced gain
                self.client.rotateByYawRateAsync(yaw_rate, 0.3)  # No .join() for continuous motion
            
        except Exception as e:
            self.errors.append(f"Velocity yaw rotation error: {str(e)}")
    
    def _point_towards_center_smooth(self, current_x, current_y, center_x, center_y):
        """Point the drone towards the center with ultra-smooth yaw control (enhanced velocity mode)."""
        try:
            # Calculate angle to center
            dx = center_x - current_x
            dy = center_y - current_y
            target_yaw = math.degrees(math.atan2(dy, dx))
            
            # Get current yaw from orientation quaternion (more precise)
            current_state = self.client.getMultirotorState()
            orientation = current_state.kinematics_estimated.orientation
            
            # Convert quaternion to yaw angle (more accurate than z_val)
            # For NED coordinate system
            yaw_rad = math.atan2(
                2 * (orientation.w_val * orientation.z_val + orientation.x_val * orientation.y_val),
                1 - 2 * (orientation.y_val**2 + orientation.z_val**2)
            )
            current_yaw = math.degrees(yaw_rad)
            
            # Calculate yaw difference and normalize to [-180, 180]
            yaw_diff = target_yaw - current_yaw
            while yaw_diff > 180:
                yaw_diff -= 360
            while yaw_diff < -180:
                yaw_diff += 360
            
            # Ultra-smooth yaw control with smaller threshold and gentler response
            if abs(yaw_diff) > 8:  # Smaller threshold for smoother pointing
                # Use very gentle yaw rate with progressive scaling
                max_yaw_rate = 20  # Maximum degrees per second
                yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_diff * 0.15))  # Very gentle gain
                
                # Apply smooth yaw rate command (no join for continuous motion)
                self.client.rotateByYawRateAsync(yaw_rate, 0.2)
                
        except Exception as e:
            self.errors.append(f"Smooth yaw rotation error: {str(e)}")
    
    def get_planned_distance(self):
        """Calculate the planned mission distance."""
        # Box perimeter
        box_perimeter = 4 * self.box_size
        
        # Orbital photography distance (circumference * number of orbits) - only if enabled
        if self.enable_orbits:
            orbit_circumference = 2 * math.pi * self.orbit_radius
            total_orbit_distance = orbit_circumference * 4  # 4 vertices
        else:
            total_orbit_distance = 0
        
        # Add some buffer for takeoff/landing and transitions
        buffer_distance = 100
        
        return box_perimeter + total_orbit_distance + buffer_distance
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics."""
        metrics = {
            "Vertices Visited": f"{self.vertices_visited}/4",
            "Box Size": f"{self.box_size}m x {self.box_size}m",
            "Orbital Photography": "Enabled" if self.enable_orbits else "Disabled"
        }
        
        if self.enable_orbits:
            metrics.update({
                "Orbits Completed": self.orbits_completed,
                "Total Photos": self.total_photos,
                "Orbit Radius": f"{self.orbit_radius}m"
            })
        
        return metrics


def main():
    """Main function to run the box mission."""
    # Create argument parser with standard parameters
    parser = create_standard_argument_parser()
    parser.description = "Autonomous Box Mission with Orbital Photography"
    
    # Add box-specific parameters
    parser.add_argument('--box_size', type=float, default=400,
                       help='Box size in meters (default: 400)')
    parser.add_argument('--orbit_radius', type=float, default=50,
                       help='Orbit radius for photography in meters (default: 50)')
    parser.add_argument('--orbit_speed', type=float, default=3,
                       help='Orbit speed in m/s (default: 3)')
    parser.add_argument('--photos_per_orbit', type=int, default=8,
                       help='Number of photos per orbit (default: 8)')
    parser.add_argument('--orbit_mode', type=str, default='velocity', 
                       choices=['waypoint', 'velocity'],
                       help='Orbital motion mode: velocity (smooth continuous) or waypoint (stable transitions) (default: velocity)')
    parser.add_argument('--enable_orbits', action='store_true', default=True,
                       help='Enable orbital photography at vertices (default: True)')
    parser.add_argument('--disable_orbits', action='store_true',
                       help='Disable orbital photography - just fly box pattern')
    
    args = parser.parse_args()
    
    # Handle orbit enable/disable logic
    enable_orbits = args.enable_orbits and not args.disable_orbits
    
    # Create and configure mission
    mission = BoxMission(
        box_size=args.box_size,
        altitude=args.altitude,
        speed=args.speed,
        orbit_radius=args.orbit_radius,
        orbit_speed=args.orbit_speed,
        photos_per_orbit=args.photos_per_orbit,
        orbit_mode=args.orbit_mode,
        max_collisions=args.max_collisions,
        enable_orbits=enable_orbits
    )
    
    # Preview mode
    if args.preview:
        print("MISSION PREVIEW MODE")
        mission.print_mission_parameters()
        
        planned_distance = mission.get_planned_distance()
        estimated_time = planned_distance / args.speed
        
        print(f"\nMISSION ESTIMATES")
        print(f"Planned Distance: {planned_distance:.0f}m")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        if enable_orbits:
            print(f"Total Photos: {args.photos_per_orbit * 4}")
            print(f"Orbit Mode: {args.orbit_mode.title()} ({'Smooth continuous motion' if args.orbit_mode == 'velocity' else 'Stable waypoint transitions'})")
        else:
            print(f"Total Photos: 0 (orbital photography disabled)")
            print(f"Mission Type: Simple box pattern flight")
        
        print(f"\nTo execute the mission, run without --preview flag")
        return
        
    # Execute mission
    print("Starting Box Mission...")
    mission.run_mission()


if __name__ == "__main__":
    main() 