#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse
from datetime import datetime

# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import cosysairsim as airsim
from mission_planning.generic_mission import GenericMission, create_standard_argument_parser

class MultiDroneOrbitMission(GenericMission):
    """
    Multi-Drone Orbit Mission: Each drone orbits through all other drone positions.
    
    Mission Flow:
    1. All drones takeoff from their initial positions
    2. Each drone moves in a circular orbit pattern, visiting the position of each other drone
    3. At each orbit position, drones stop for 2 seconds before continuing
    4. All drones return to their original starting positions
    
    Key Features:
    - Supports variable number of drones (auto-detects available drones)
    - Synchronized orbital movement
    - Position stopping mechanism for observation/photo opportunities
    - Collision detection and safety features from GenericMission
    - Comprehensive mission reporting
    """
    
    def __init__(self, num_drones=None, altitude=20, speed=5, orbit_radius=2, 
                 stop_duration=2, disable_collision_detection=False):
        """
        Initialize Multi-Drone Orbit Mission.
        
        Args:
            num_drones: Number of drones to use (None = auto-detect)
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            orbit_radius: Radius around each position for smooth orbital approach
            stop_duration: Time to stop at each orbit position in seconds
            disable_collision_detection: Whether to disable collision detection
        """
        super().__init__(altitude, speed, disable_collision_detection)
        
        # Mission-specific parameters
        self.num_drones = num_drones
        self.orbit_radius = orbit_radius
        self.stop_duration = stop_duration
        
        # Multi-drone management
        self.drone_names = []
        self.initial_positions = {}
        self.current_positions = {}
        self.orbit_clients = {}  # Store individual client connections for each drone
        
        # Mission tracking
        self.total_orbit_positions = 0
        self.completed_orbit_cycles = 0
        self.photos_taken = 0
        
        # Custom metrics for this mission
        self.custom_metrics = {
            'drones_used': 0,
            'orbit_positions_visited': 0,
            'total_stops_made': 0,
            'photos_captured': 0,
            'average_cycle_time': 0,
            'orbit_efficiency': 0
        }
    
    def print_mission_parameters(self):
        """Print mission-specific parameters."""
        print(f"\nMULTI-DRONE ORBIT MISSION PARAMETERS")
        print(f"Target Drone Count: {self.num_drones if self.num_drones else 'Auto-detect'}")
        print(f"Flight Altitude: {self.altitude}m")
        print(f"Flight Speed: {self.speed}m/s")
        print(f"Orbit Radius: {self.orbit_radius}m")
        print(f"Stop Duration: {self.stop_duration}s per position")
        print(f"Collision Detection: {'Disabled' if self.collision_detection_disabled_globally else 'Enabled'}")
    
    def _discover_available_drones(self):
        """
        Discover available drones in the simulation.
        Uses common naming patterns to find drones.
        """
        print("\n🔍 Discovering available drones in simulation...")
        
        # Common drone naming patterns to check
        drone_patterns = [
            ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6"],
            ["PX4_Drone1", "PX4_Drone2", "PX4_Drone3", "PX4_Drone4", "PX4_Drone5", "PX4_Drone6"],
            ["UAV1", "UAV2", "UAV3", "UAV4", "UAV5", "UAV6"],
            ["Multirotor1", "Multirotor2", "Multirotor3", "Multirotor4", "Multirotor5", "Multirotor6"]
        ]
        
        available_drones = []
        
        # Try each pattern
        for pattern in drone_patterns:
            temp_client = airsim.MultirotorClient()  # Temporary client for discovery
            temp_client.confirmConnection()
            
            pattern_drones = []
            for drone_name in pattern:
                try:
                    # Try to get the drone state - if successful, drone exists
                    state = temp_client.getMultirotorState(vehicle_name=drone_name)
                    pattern_drones.append(drone_name)
                    print(f"  ✅ Found: {drone_name}")
                except:
                    # Drone doesn't exist, continue
                    pass
            
            # If we found drones with this pattern, use them
            if pattern_drones:
                available_drones = pattern_drones
                print(f"  Using drone pattern: {pattern[0].replace('1', 'X')}")
                break
        
        if not available_drones:
            # Try generic "SimDrone" pattern as fallback
            temp_client = airsim.MultirotorClient()
            temp_client.confirmConnection()
            for i in range(1, 11):  # Check up to 10 drones
                drone_name = f"SimDrone{i}"
                try:
                    state = temp_client.getMultirotorState(vehicle_name=drone_name)
                    available_drones.append(drone_name)
                    print(f"  ✅ Found: {drone_name}")
                except:
                    pass
        
        if self.num_drones:
            # Limit to requested number
            available_drones = available_drones[:self.num_drones]
        
        if not available_drones:
            raise Exception("No drones found in simulation! Check settings.json configuration.")
        
        print(f"  🎯 Total drones available: {len(available_drones)}")
        return available_drones
    
    def _setup_multi_drone_connections(self):
        """Setup individual client connections for each drone."""
        print("\n🔗 Setting up multi-drone connections...")
        
        for drone_name in self.drone_names:
            try:
                # Create individual client for each drone (some versions need this)
                client = airsim.MultirotorClient()
                client.confirmConnection()
                
                # Enable API control and arm
                client.enableApiControl(True, drone_name)
                client.armDisarm(True, drone_name)
                
                # Store client reference
                self.orbit_clients[drone_name] = client
                
                print(f"  ✅ {drone_name}: Connected and armed")
                
            except Exception as e:
                error_msg = f"Failed to setup {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        print(f"✅ Multi-drone setup completed for {len(self.orbit_clients)} drones")
    
    def _record_initial_positions(self):
        """Record the initial positions of all drones."""
        print("\n📍 Recording initial drone positions...")
        
        for drone_name in self.drone_names:
            try:
                client = self.orbit_clients[drone_name]
                state = client.getMultirotorState(vehicle_name=drone_name)
                position = state.kinematics_estimated.position
                
                self.initial_positions[drone_name] = {
                    'x': position.x_val,
                    'y': position.y_val,
                    'z': position.z_val
                }
                
                self.current_positions[drone_name] = self.initial_positions[drone_name].copy()
                
                print(f"  {drone_name}: ({position.x_val:.1f}, {position.y_val:.1f}, {position.z_val:.1f})")
                
            except Exception as e:
                error_msg = f"Failed to get position for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        self.total_orbit_positions = len(self.drone_names) * len(self.drone_names)  # Each drone visits all positions
        print(f"📊 Total orbit positions to visit: {self.total_orbit_positions}")
    
    def execute_mission_logic(self, target_z):
        """
        Execute the core multi-drone orbit mission logic.
        
        This is the main mission execution method that:
        1. Discovers and sets up all drones
        2. Records initial positions
        3. Executes synchronized takeoff
        4. Performs orbital movement between positions
        5. Manages stopping and photo-taking at each position
        6. Returns all drones to starting positions
        """
        print(f"\n🚁 Starting Multi-Drone Orbit Mission Logic")
        
        # Step 1: Discover available drones
        self.drone_names = self._discover_available_drones()
        self.custom_metrics['drones_used'] = len(self.drone_names)
        
        if len(self.drone_names) < 2:
            raise Exception(f"Need at least 2 drones for orbit mission, found {len(self.drone_names)}")
        
        # Step 2: Setup multi-drone connections
        self._setup_multi_drone_connections()
        
        # Step 3: Record initial positions
        self._record_initial_positions()
        
        # Step 4: Synchronized takeoff
        self._execute_synchronized_takeoff(target_z)
        
        # Step 5: Execute orbital movement pattern
        self._execute_orbital_pattern(target_z)
        
        # Step 6: Return to home positions
        self._return_all_drones_home(target_z)
        
        print("✅ Multi-drone orbit mission logic completed successfully!")
    
    def _execute_synchronized_takeoff(self, target_z):
        """Execute synchronized takeoff for all drones."""
        print(f"\n🚀 Executing synchronized takeoff to {abs(target_z)}m...")
        
        takeoff_futures = []
        
        # Start takeoff for all drones simultaneously
        for drone_name in self.drone_names:
            try:
                client = self.orbit_clients[drone_name]
                future = client.takeoffAsync(vehicle_name=drone_name)
                takeoff_futures.append((drone_name, future))
                print(f"  🚁 {drone_name}: Takeoff initiated")
                
            except Exception as e:
                error_msg = f"Takeoff failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        # Wait for all takeoffs to complete
        for drone_name, future in takeoff_futures:
            try:
                future.join()
                print(f"  ✅ {drone_name}: Takeoff completed")
            except Exception as e:
                error_msg = f"Takeoff completion failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        time.sleep(2)  # Stabilization time
        
        # Move all drones to target altitude
        altitude_futures = []
        for drone_name in self.drone_names:
            try:
                client = self.orbit_clients[drone_name]
                future = client.moveToZAsync(target_z, self.speed, vehicle_name=drone_name)
                altitude_futures.append((drone_name, future))
                print(f"  📈 {drone_name}: Climbing to {abs(target_z)}m")
                
            except Exception as e:
                error_msg = f"Altitude climb failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        # Wait for altitude changes
        for drone_name, future in altitude_futures:
            try:
                future.join()
                print(f"  ✅ {drone_name}: Reached target altitude")
            except Exception as e:
                error_msg = f"Altitude completion failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        print("✅ Synchronized takeoff completed for all drones")
    
    def _execute_orbital_pattern(self, target_z):
        """
        Execute the orbital pattern where each drone visits all other drone positions.
        
        The orbital pattern works as follows:
        1. Each drone has a sequence of positions to visit (all other drones' positions)
        2. Drones move simultaneously to their next target positions
        3. At each position, drones perform orbital approach and stop for specified duration
        4. Process continues until all drones complete full orbit cycle
        """
        print(f"\n🔄 Executing orbital pattern...")
        print(f"  Each drone will visit {len(self.drone_names)-1} other positions")
        print(f"  Stop duration: {self.stop_duration}s per position")
        print(f"  Orbit radius: {self.orbit_radius}m")
        
        # Create orbit sequences for each drone
        orbit_sequences = {}
        for i, drone_name in enumerate(self.drone_names):
            # Each drone visits all other drones' positions in order
            sequence = []
            for j in range(len(self.drone_names)):
                if i != j:  # Don't visit own position
                    target_drone = self.drone_names[j]
                    sequence.append(target_drone)
            orbit_sequences[drone_name] = sequence
        
        # Print orbit plan
        print(f"\n📋 Orbit Plan:")
        for drone_name, sequence in orbit_sequences.items():
            print(f"  {drone_name} → {' → '.join(sequence)} → HOME")
        
        cycle_start_time = time.time()
        
        # Execute orbit cycles
        max_sequence_length = max(len(seq) for seq in orbit_sequences.values())
        
        for step in range(max_sequence_length):
            print(f"\n🎯 Orbit Step {step + 1}/{max_sequence_length}")
            
            # Move all drones to their next target position simultaneously
            move_futures = []
            step_targets = {}
            
            for drone_name in self.drone_names:
                if step < len(orbit_sequences[drone_name]):
                    target_drone = orbit_sequences[drone_name][step]
                    target_pos = self.initial_positions[target_drone]
                    step_targets[drone_name] = target_drone
                    
                    # Calculate orbital approach position (slightly offset for smooth approach)
                    orbital_x = target_pos['x'] + self.orbit_radius * math.cos(step * math.pi / 4)
                    orbital_y = target_pos['y'] + self.orbit_radius * math.sin(step * math.pi / 4)
                    
                    try:
                        client = self.orbit_clients[drone_name]
                        future = client.moveToPositionAsync(
                            orbital_x, orbital_y, target_z, self.speed, 
                            vehicle_name=drone_name
                        )
                        move_futures.append((drone_name, future))
                        
                        print(f"  🚁 {drone_name} → {target_drone} position via orbit approach")
                        
                    except Exception as e:
                        error_msg = f"Orbit movement failed for {drone_name}: {str(e)}"
                        self.errors.append(error_msg)
                        print(f"  ❌ {error_msg}")
            
            # Wait for all movements to complete
            for drone_name, future in move_futures:
                try:
                    future.join()
                    print(f"  ✅ {drone_name}: Reached orbit position")
                    
                    # Update distance tracking
                    self.update_distance_tracking()
                    
                except Exception as e:
                    error_msg = f"Orbit movement completion failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Now move to exact target positions and stop
            exact_move_futures = []
            for drone_name in step_targets:
                target_drone = step_targets[drone_name]
                target_pos = self.initial_positions[target_drone]
                
                try:
                    client = self.orbit_clients[drone_name]
                    future = client.moveToPositionAsync(
                        target_pos['x'], target_pos['y'], target_z, self.speed * 0.5,  # Slower for precision
                        vehicle_name=drone_name
                    )
                    exact_move_futures.append((drone_name, future))
                    
                except Exception as e:
                    error_msg = f"Exact positioning failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Wait for exact positioning
            for drone_name, future in exact_move_futures:
                try:
                    future.join()
                    self.custom_metrics['orbit_positions_visited'] += 1
                    
                except Exception as e:
                    error_msg = f"Exact positioning completion failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Stop and hover at positions
            print(f"  ⏸️  Stopping at positions for {self.stop_duration}s...")
            
            # Hover all drones
            hover_futures = []
            for drone_name in step_targets:
                try:
                    client = self.orbit_clients[drone_name]
                    future = client.hoverAsync(vehicle_name=drone_name)
                    hover_futures.append((drone_name, future))
                    
                except Exception as e:
                    error_msg = f"Hover failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Wait for hover to engage
            for drone_name, future in hover_futures:
                try:
                    future.join()
                except Exception as e:
                    error_msg = f"Hover engagement failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Take photos during stop (optional)
            for drone_name in step_targets:
                try:
                    target_drone = step_targets[drone_name]
                    photo_filename = f"{drone_name}_at_{target_drone}_position_step_{step+1}"
                    photo_path = self.take_photo(photo_filename)
                    if photo_path:
                        self.photos_taken += 1
                        self.custom_metrics['photos_captured'] += 1
                        print(f"  📸 {drone_name}: Photo captured at {target_drone} position")
                    
                except Exception as e:
                    error_msg = f"Photo capture failed for {drone_name}: {str(e)}"
                    self.errors.append(error_msg)
                    print(f"  ❌ {error_msg}")
            
            # Stop duration
            time.sleep(self.stop_duration)
            self.custom_metrics['total_stops_made'] += len(step_targets)
            
            print(f"  ✅ Step {step + 1} completed")
            
            # Monitor for collisions during mission
            self._monitor_collision_during_mission()
        
        cycle_end_time = time.time()
        cycle_duration = cycle_end_time - cycle_start_time
        self.custom_metrics['average_cycle_time'] = cycle_duration
        self.completed_orbit_cycles = 1
        
        print(f"✅ Orbital pattern completed in {cycle_duration:.1f}s")
    
    def _return_all_drones_home(self, target_z):
        """Return all drones to their initial starting positions."""
        print(f"\n🏠 Returning all drones to home positions...")
        
        return_futures = []
        
        # Start return movements for all drones simultaneously
        for drone_name in self.drone_names:
            try:
                home_pos = self.initial_positions[drone_name]
                client = self.orbit_clients[drone_name]
                
                future = client.moveToPositionAsync(
                    home_pos['x'], home_pos['y'], target_z, self.speed,
                    vehicle_name=drone_name
                )
                return_futures.append((drone_name, future))
                
                print(f"  🏠 {drone_name}: Returning to ({home_pos['x']:.1f}, {home_pos['y']:.1f}, {abs(target_z):.1f})")
                
            except Exception as e:
                error_msg = f"Return home failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        # Wait for all returns to complete
        for drone_name, future in return_futures:
            try:
                future.join()
                print(f"  ✅ {drone_name}: Returned home")
                
                # Update distance tracking
                self.update_distance_tracking()
                
            except Exception as e:
                error_msg = f"Return completion failed for {drone_name}: {str(e)}"
                self.errors.append(error_msg)
                print(f"  ❌ {error_msg}")
        
        print("✅ All drones returned to home positions")
    
    def get_planned_distance(self):
        """Calculate the total planned distance for all drones."""
        if len(self.drone_names) < 2:
            return 0
        
        # Calculate distance each drone travels
        total_planned = 0
        
        for i, drone_name in enumerate(self.drone_names):
            drone_distance = 0
            current_pos = self.initial_positions[drone_name]
            
            # Distance to visit all other drone positions
            for j, other_drone in enumerate(self.drone_names):
                if i != j:
                    other_pos = self.initial_positions[other_drone]
                    dx = other_pos['x'] - current_pos['x']
                    dy = other_pos['y'] - current_pos['y']
                    distance = math.sqrt(dx*dx + dy*dy)
                    drone_distance += distance
                    current_pos = other_pos  # Update current position for next calculation
            
            # Distance back to home
            home_pos = self.initial_positions[drone_name]
            dx = home_pos['x'] - current_pos['x']
            dy = home_pos['y'] - current_pos['y']
            drone_distance += math.sqrt(dx*dx + dy*dy)
            
            # Add altitude changes (takeoff/landing)
            drone_distance += self.altitude * 2  # Up and down
            
            total_planned += drone_distance
        
        return total_planned
    
    def get_mission_specific_metrics(self):
        """Return mission-specific metrics for reporting."""
        # Calculate efficiency
        planned_distance = self.get_planned_distance()
        if planned_distance > 0 and self.total_distance > 0:
            self.custom_metrics['orbit_efficiency'] = (planned_distance / self.total_distance) * 100
        
        return {
            'Drones Used': self.custom_metrics['drones_used'],
            'Orbit Positions Visited': f"{self.custom_metrics['orbit_positions_visited']}/{self.total_orbit_positions}",
            'Total Stops Made': self.custom_metrics['total_stops_made'],
            'Photos Captured': self.custom_metrics['photos_captured'],
            'Orbit Cycles Completed': self.completed_orbit_cycles,
            'Average Cycle Time': f"{self.custom_metrics['average_cycle_time']:.1f}s",
            'Orbit Efficiency': f"{self.custom_metrics['orbit_efficiency']:.1f}%"
        }


def main():
    """Main execution function with command line argument parsing."""
    parser = create_standard_argument_parser()
    
    # Add mission-specific arguments
    parser.add_argument('--num_drones', type=int, default=None,
                       help='Number of drones to use (default: auto-detect all available)')
    parser.add_argument('--orbit_radius', type=float, default=2,
                       help='Orbit radius around each position in meters (default: 2)')
    parser.add_argument('--stop_duration', type=float, default=2,
                       help='Stop duration at each orbit position in seconds (default: 2)')
    
    args = parser.parse_args()
    
    if args.preview:
        print("MISSION PREVIEW MODE")
        print("=" * 50)
        mission = MultiDroneOrbitMission(
            num_drones=args.num_drones,
            altitude=args.altitude,
            speed=args.speed,
            orbit_radius=args.orbit_radius,
            stop_duration=args.stop_duration,
            disable_collision_detection=args.disable_collision_detection
        )
        mission.print_mission_parameters()
        return
    
    # Create and run the mission
    try:
        mission = MultiDroneOrbitMission(
            num_drones=args.num_drones,
            altitude=args.altitude,
            speed=args.speed,
            orbit_radius=args.orbit_radius,
            stop_duration=args.stop_duration,
            disable_collision_detection=args.disable_collision_detection
        )
        
        # Configure collision detection settings
        if args.max_collisions:
            mission.max_collisions = args.max_collisions
        
        if not args.include_landscape_collisions:
            mission.set_landscape_collision_detection(False)
        
        if args.ignore_objects:
            for obj in args.ignore_objects:
                mission.add_ignored_collision_object(obj)
        
        mission.run_mission()
        
    except KeyboardInterrupt:
        print("\nMission interrupted by user!")
    except Exception as e:
        print(f"\nMission failed: {str(e)}")


if __name__ == "__main__":
    main()