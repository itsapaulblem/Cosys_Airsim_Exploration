import cv2
import numpy as np
import os
import pprint
import tempfile
import sys
import time
import math

# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim


# Use below in settings.json with Blocks environment for 3 PX4 drones
"""
{
    "SeeDocsAt": "https://cosys-lab.github.io/Cosys-AirSim/settings/",
    "SettingsVersion": 2.0,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "ApiServerPort": 41451,
    "OriginGeopoint": {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 10
    },
    "Vehicles": {
        "PX4_Drone1": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlIp": "127.0.0.1",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "LocalHostIp": "127.0.0.1",
            "X": 0, "Y": 0, "Z": -2
        },
        "PX4_Drone2": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4561,
            "ControlIp": "127.0.0.1",
            "ControlPortLocal": 14541,
            "ControlPortRemote": 14581,
            "LocalHostIp": "127.0.0.1",
            "X": 5, "Y": 0, "Z": -2
        },
        "PX4_Drone3": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4562,
            "ControlIp": "127.0.0.1",
            "ControlPortLocal": 14542,
            "ControlPortRemote": 14582,
            "LocalHostIp": "127.0.0.1",
            "X": -5, "Y": 0, "Z": -2
        }
    }
}
"""

def print_status(message):
    """Print status message with timestamp"""
    print(f"[{time.strftime('%H:%M:%S')}] {message}")

def get_circle_position(center_x, center_y, radius, angle_degrees, altitude):
    """Calculate position on circle given center, radius, angle and altitude"""
    angle_rad = math.radians(angle_degrees)
    x = center_x + radius * math.cos(angle_rad)
    y = center_y + radius * math.sin(angle_rad)
    return x, y, altitude

def calculate_orbit_parameters(num_vehicles):
    """
    Calculate orbit parameters dynamically based on number of vehicles
    
    Args:
        num_vehicles (int): Number of vehicles in the formation
        
    Returns:
        dict: Dictionary containing orbit parameters
    """
    # Base parameters for single vehicle
    base_radius = 10
    base_altitude = 20
    base_speed = 3
    
    # Dynamic scaling based on number of vehicles
    if num_vehicles == 1:
        # Single vehicle - smaller, faster orbit
        radius = base_radius
        altitude = base_altitude
        speed = base_speed + 1
        orbits = 3
    elif num_vehicles == 2:
        # Two vehicles - medium orbit
        radius = base_radius + 5
        altitude = base_altitude + 2
        speed = base_speed
        orbits = 2
    elif num_vehicles == 3:
        # Three vehicles - larger orbit for spacing
        radius = base_radius + 8
        altitude = base_altitude + 5
        speed = base_speed - 0.5
        orbits = 2
    else:
        # 4+ vehicles - scale up significantly
        radius = base_radius + (num_vehicles * 4)
        altitude = base_altitude + (num_vehicles * 2)
        speed = max(2, base_speed - (num_vehicles * 0.3))
        orbits = 1 + (1 if num_vehicles <= 5 else 0)
    
    # Calculate safe minimum separation distance
    circumference = 2 * math.pi * radius
    min_separation = circumference / num_vehicles
    
    # Safety check: ensure minimum 5m separation between vehicles
    min_safe_separation = 5.0
    if min_separation < min_safe_separation:
        # Increase radius to maintain safe separation
        required_circumference = num_vehicles * min_safe_separation
        radius = required_circumference / (2 * math.pi)
        min_separation = min_safe_separation
        print(f"Radius increased to {radius:.1f}m for safe {min_safe_separation}m separation")
    
    # Calculate dynamic motion step for smoother orbit motion
    # Use smaller steps for more vehicles, but keep it reasonable for orbit motion
    if num_vehicles == 1:
        motion_step = 3  # Smooth motion for single vehicle
    elif num_vehicles == 2:
        motion_step = 2  # Medium steps for two vehicles
    else:
        motion_step = max(1, 4 - num_vehicles)  # Smaller steps for more vehicles
    
    return {
        'radius': radius,
        'altitude': -altitude,  # Negative Z for AirSim
        'speed': speed,
        'orbits': orbits,
        'min_separation': min_separation,
        'motion_step': motion_step
    }

def main():
    # Base orbit configuration
    center_x, center_y = 0, 0  # Center of orbit
    
    print_status("Multi-Agent Orbit Demo Starting...")
    
    # Connect to AirSim
    print_status("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print_status("Connected successfully!")
    
    # Get available vehicles dynamically
    print_status("Discovering available vehicles...")
    available_vehicles = client.listVehicles()
    print_status(f"Found vehicles: {available_vehicles}")
    
    # Filter for PX4 drones (or use all vehicles if no PX4 prefix)
    drone_names = [name for name in available_vehicles if name.startswith("PX4_") or len(available_vehicles) <= 3]
    if not drone_names:
        # If no PX4 prefixed vehicles, use all available vehicles
        drone_names = available_vehicles
    
    if len(drone_names) == 0:
        print_status("ERROR: No vehicles found! Check your AirSim settings.json")
        return
    elif len(drone_names) > 3:
        print_status(f"WARNING: Found {len(drone_names)} vehicles, using first 3 for orbit demo")
        drone_names = drone_names[:3]
    
    print_status(f"Using vehicles: {drone_names}")
    
    # Calculate dynamic orbit parameters based on number of vehicles
    orbit_params = calculate_orbit_parameters(len(drone_names))
    orbit_radius = orbit_params['radius']
    orbit_altitude = orbit_params['altitude']
    orbit_speed = orbit_params['speed']
    num_orbits = orbit_params['orbits']
    min_separation = orbit_params['min_separation']
    motion_step = orbit_params['motion_step']
    
    print_status(f"Dynamic orbit configuration for {len(drone_names)} vehicles:")
    print_status(f"  Radius: {orbit_radius}m")
    print_status(f"  Altitude: {-orbit_altitude}m")
    print_status(f"  Speed: {orbit_speed}m/s")
    print_status(f"  Orbits: {num_orbits}")
    print_status(f"  Min separation: {min_separation:.1f}m")
    
    # Calculate initial positions (evenly spaced around circle)
    formation_spacing = 360 / len(drone_names)
    initial_angles = [i * formation_spacing for i in range(len(drone_names))]
    print_status(f"Formation: {len(drone_names)} drones spaced {formation_spacing:.1f}° apart")
    
    try:
        # Enable API control for all drones
        print_status("Enabling API control...")
        for drone in drone_names:
            client.enableApiControl(True, drone)
            print_status(f"  {drone}: API control enabled")
        
        # Arm all drones
        print_status("Arming drones...")
        for drone in drone_names:
            client.armDisarm(True, drone)
            print_status(f"  {drone}: Armed")
        
        input("Press Enter to takeoff...")
        
        # Takeoff all drones
        print_status("Taking off...")
        takeoff_futures = []
        for drone in drone_names:
            future = client.takeoffAsync(vehicle_name=drone)
            takeoff_futures.append(future)
        
        # Wait for all takeoffs to complete
        for i, future in enumerate(takeoff_futures):
            future.join()
            print_status(f"  {drone_names[i]}: Takeoff complete")
        
        time.sleep(2)  # Brief pause after takeoff
        
        # Move to initial orbit positions
        print_status("Moving to initial orbit positions...")
        move_futures = []
        for i, drone in enumerate(drone_names):
            x, y, z = get_circle_position(center_x, center_y, orbit_radius, initial_angles[i], orbit_altitude)
            print_status(f"  {drone}: Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
            future = client.moveToPositionAsync(x, y, z, orbit_speed, vehicle_name=drone)
            move_futures.append(future)
        
        # Wait for all moves to complete
        for i, future in enumerate(move_futures):
            future.join()
            print_status(f"  {drone_names[i]}: Reached initial position")
        
        time.sleep(2)  # Brief pause
        
        input("Press Enter to start orbiting...")
        
        # Orbit in formation
        print_status(f"Starting orbit pattern ({num_orbits} complete circles)...")
        
        # Calculate orbit parameters
        circumference = 2 * math.pi * orbit_radius
        orbit_time = circumference / orbit_speed  # Time for one complete orbit
        time_step = (motion_step / 360.0) * orbit_time  # Time per step
        total_steps = int((360 * num_orbits) / motion_step)
        
        print_status(f"Orbit details: circumference={circumference:.1f}m, time_per_orbit={orbit_time:.1f}s")
        print_status(f"Motion: {motion_step}° steps, {time_step:.2f}s per step, {total_steps} total steps")
        print_status(f"Estimated total mission time: {(total_steps * time_step + 30):.1f}s")
        
        for step in range(total_steps):
            current_angles = [(initial_angles[i] + step * motion_step) % 360 for i in range(len(drone_names))]
            
            move_futures = []
            for i, drone in enumerate(drone_names):
                x, y, z = get_circle_position(center_x, center_y, orbit_radius, current_angles[i], orbit_altitude)
                future = client.moveToPositionAsync(x, y, z, orbit_speed, vehicle_name=drone)
                move_futures.append(future)
            
            # Wait for all moves to complete
            for future in move_futures:
                future.join()
            
            # Progress update - adjust frequency based on motion step
            progress_interval = max(1, int(45 / motion_step)) if motion_step <= 45 else 1
            if step % progress_interval == 0:
                progress = (step / total_steps) * 100
                orbits_completed = step * motion_step / 360.0
                print_status(f"Progress: {progress:.1f}% ({orbits_completed:.2f} orbits completed)")
            
            time.sleep(0.1)  # Small delay between waypoints
        
        print_status("Orbit pattern completed!")
        time.sleep(2)
        
        input("Press Enter to land...")
        
        # Land all drones
        print_status("Landing drones...")
        land_futures = []
        for drone in drone_names:
            future = client.landAsync(vehicle_name=drone)
            land_futures.append(future)
        
        # Wait for all landings to complete
        for i, future in enumerate(land_futures):
            future.join()
            print_status(f"  {drone_names[i]}: Landed")
        
        time.sleep(2)
        
        # Disarm all drones
        print_status("Disarming drones...")
        for drone in drone_names:
            client.armDisarm(False, drone)
            print_status(f"  {drone}: Disarmed")
        
        print_status("Mission completed successfully!")
        
        # Get final states
        print_status("Final drone states:")
        for drone in drone_names:
            state = client.getMultirotorState(vehicle_name=drone)
            pos = state.kinematics_estimated.position
            print_status(f"  {drone}: Position({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
        
    except Exception as e:
        print_status(f"Error occurred: {str(e)}")
        print_status("Attempting emergency landing...")
        
        # Emergency procedures
        try:
            for drone in drone_names:
                client.landAsync(vehicle_name=drone)
                client.armDisarm(False, drone)
        except:
            pass
    
    finally:
        # Cleanup
        print_status("Cleaning up...")
        try:
            for drone in drone_names:
                client.enableApiControl(False, drone)
            print_status("API control disabled for all drones")
        except:
            pass

if __name__ == "__main__":
    main() 