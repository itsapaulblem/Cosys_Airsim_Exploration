#!/usr/bin/env python3
"""
Enhanced Multi-Agent Drone Script for Cosys-AirSim with Auto Vehicle Discovery
============================================================================

This script uses the Cosys-AirSim client.listVehicles() function to automatically
discover all available vehicles and make them take off.

Features:
- Uses native listVehicles() API for reliable vehicle discovery
- Batch takeoff for all discovered vehicles  
- Error handling for individual vehicles
- Enhanced sensor and state monitoring
- Support for all Cosys-AirSim vehicle types
"""

import cosysairsim as airsim
import cv2
import numpy as np
import os
import pprint
import sys
import tempfile
import time

# Add the parent directory to the path to import setup_path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def setup_vehicle(client, vehicle_name, vehicle_type="MultiRotor"):
    """
    Setup a single vehicle for API control.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle to setup
        vehicle_type (str): Type of vehicle (MultiRotor, Car, ComputerVision)
        
    Returns:
        bool: True if setup successful, False otherwise
    """
    try:
        print(f"⚙️  Setting up '{vehicle_name}' ({vehicle_type})...")
        
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        
        # Only arm multirotor vehicles
        if vehicle_type.lower() in ["multirotor", "simpleflight"]:
            client.armDisarm(True, vehicle_name)
        
        print(f"✅ '{vehicle_name}' setup complete")
        return True
        
    except Exception as e:
        print(f"❌ Failed to setup '{vehicle_name}': {str(e)}")
        return False

def takeoff_vehicle(client, vehicle_name, vehicle_type="MultiRotor"):
    """
    Make a single vehicle take off asynchronously (only for multirotor).
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle to takeoff
        vehicle_type (str): Type of vehicle
        
    Returns:
        Future or None: Takeoff future if successful, None otherwise
    """
    try:
        if vehicle_type.lower() not in ["multirotor", "simpleflight"]:
            print(f"⚠️  Skipping takeoff for '{vehicle_name}' - not a multirotor")
            return None
            
        print(f"🚁 Starting takeoff for '{vehicle_name}'...")
        future = client.takeoffAsync(vehicle_name=vehicle_name)
        return future
        
    except Exception as e:
        print(f"❌ Failed to start takeoff for '{vehicle_name}': {str(e)}")
        return None

def get_vehicle_state(client, vehicle_name, vehicle_type="MultiRotor"):
    """
    Get and print vehicle state based on vehicle type.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle
        vehicle_type (str): Type of vehicle
    """
    try:
        print(f"\n📊 State for '{vehicle_name}' ({vehicle_type}):")
        
        if vehicle_type.lower() in ["multirotor", "simpleflight"]:
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            print(f"   Position: x={state.kinematics_estimated.position.x_val:.2f}, "
                  f"y={state.kinematics_estimated.position.y_val:.2f}, "
                  f"z={state.kinematics_estimated.position.z_val:.2f}")
            print(f"   Landed: {state.landed_state}")
            print(f"   Armed: {state.armed}")
            
        elif vehicle_type.lower() == "car":
            state = client.getCarState(vehicle_name=vehicle_name)
            print(f"   Position: x={state.kinematics_estimated.position.x_val:.2f}, "
                  f"y={state.kinematics_estimated.position.y_val:.2f}, "
                  f"z={state.kinematics_estimated.position.z_val:.2f}")
            print(f"   Speed: {state.speed:.2f} m/s")
            print(f"   Gear: {state.gear}")
            
        elif vehicle_type.lower() == "computervision":
            state = client.getComputerVisionState(vehicle_name=vehicle_name)
            print(f"   Position: x={state.kinematics_estimated.position.x_val:.2f}, "
                  f"y={state.kinematics_estimated.position.y_val:.2f}, "
                  f"z={state.kinematics_estimated.position.z_val:.2f}")
            
    except Exception as e:
        print(f"❌ Failed to get state for '{vehicle_name}': {str(e)}")

def move_vehicle_to_formation(client, vehicle_name, vehicle_type, index, total_vehicles):
    """
    Move a vehicle to a formation position based on its type.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle
        vehicle_type (str): Type of vehicle
        index (int): Vehicle index in the list
        total_vehicles (int): Total number of vehicles
        
    Returns:
        Future or None: Movement future if applicable
    """
    try:
        if vehicle_type.lower() in ["multirotor", "simpleflight"]:
            # Create aerial formation
            x = -5 + (index * 4)  # Spread vehicles along X axis
            y = 5 - (index * 3)   # Stagger along Y axis  
            z = -10 - (index * 2) # Stagger altitude
            
            print(f"📍 Moving '{vehicle_name}' to aerial position ({x}, {y}, {z})")
            return client.moveToPositionAsync(x, y, z, 5, vehicle_name=vehicle_name)
            
        elif vehicle_type.lower() == "car":
            # Create ground formation
            x = index * 5  # Spread cars along X axis
            y = 0          # Keep on ground level
            z = 0          # Ground level
            
            print(f"📍 Moving '{vehicle_name}' to ground position ({x}, {y}, {z})")
            # For cars, use different movement API
            car_controls = airsim.CarControls()
            car_controls.throttle = 0.5
            car_controls.steering = 0.0
            client.setCarControls(car_controls, vehicle_name)
            
            # Return a simple future-like object for consistency
            return None
            
        else:
            print(f"⚠️  Formation movement not implemented for '{vehicle_type}'")
            return None
            
    except Exception as e:
        print(f"❌ Failed to move '{vehicle_name}': {str(e)}")
        return None

def cleanup_vehicles(client, vehicles_info):
    """
    Clean up all vehicles by disarming and disabling API control.
    
    Args:
        client: AirSim client instance
        vehicles_info (list): List of (vehicle_name, vehicle_type) tuples
    """
    print("\n🧹 Cleaning up vehicles...")
    
    for vehicle_name, vehicle_type in vehicles_info:
        try:
            if vehicle_type.lower() in ["multirotor", "simpleflight"]:
                client.armDisarm(False, vehicle_name)
            client.enableApiControl(False, vehicle_name)
            print(f"✅ Cleaned up '{vehicle_name}'")
        except Exception as e:
            print(f"⚠️  Failed to cleanup '{vehicle_name}': {str(e)}")

def main():
    """
    Main function to run the multi-agent demo with Cosys-AirSim listVehicles().
    """
    print("🚁 Cosys-AirSim Multi-Agent Auto-Discovery Demo")
    print("=" * 55)
    
    # Connect to AirSim
    print("🔗 Connecting to Cosys-AirSim...")
    client = airsim.MultirotorClient()
    
    try:
        client.confirmConnection()
        print("✅ Connected to Cosys-AirSim successfully")
    except Exception as e:
        print(f"❌ Failed to connect to AirSim: {str(e)}")
        print("Make sure AirSim is running and accessible on localhost:41451")
        return
    
    # Use the native listVehicles() function
    print("\n🔍 Discovering vehicles using listVehicles()...")
    
    try:
        vehicle_names = client.listVehicles()
        print(f"✅ Found {len(vehicle_names)} vehicle(s): {vehicle_names}")
    except Exception as e:
        print(f"❌ Failed to list vehicles: {str(e)}")
        print("This might be because you're using an older AirSim version without listVehicles()")
        return
    
    if not vehicle_names:
        print("❌ No vehicles found. Make sure your settings.json contains vehicle definitions.")
        print("📝 Example settings.json structure:")
        print("""
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 0, "Y": 0, "Z": -2
        },
        "Drone2": {
            "VehicleType": "SimpleFlight", 
            "X": 4, "Y": 0, "Z": -2
        }
    }
}
        """)
        return
    
    # Determine vehicle types (for Cosys-AirSim, we'll assume MultiRotor unless specified)
    # In a real implementation, you might want to query vehicle types from settings
    vehicles_info = []
    
    for vehicle_name in vehicle_names:
        # For this demo, we'll assume all are multirotors
        # In practice, you might parse settings.json or use other APIs to determine type
        vehicle_type = "MultiRotor"  # Default assumption
        vehicles_info.append((vehicle_name, vehicle_type))
    
    # Setup all vehicles
    print(f"\n⚙️  Setting up {len(vehicles_info)} vehicle(s)...")
    active_vehicles = []
    
    for vehicle_name, vehicle_type in vehicles_info:
        success = setup_vehicle(client, vehicle_name, vehicle_type)
        if success:
            active_vehicles.append((vehicle_name, vehicle_type))
    
    if not active_vehicles:
        print("❌ No vehicles could be setup successfully. Exiting.")
        return
    
    print(f"✅ Successfully setup {len(active_vehicles)} vehicle(s)")
    
    # Wait for user input before takeoff
    multirotor_count = sum(1 for _, vtype in active_vehicles if vtype.lower() in ["multirotor", "simpleflight"])
    print(f"\n🚁 Ready to takeoff {multirotor_count} multirotor vehicle(s)")
    airsim.wait_key('Press any key to initiate takeoff sequence...')
    
    # Start takeoff for multirotor vehicles
    print("\n🚀 Initiating takeoff sequence...")
    takeoff_futures = []
    
    for vehicle_name, vehicle_type in active_vehicles:
        future = takeoff_vehicle(client, vehicle_name, vehicle_type)
        if future:
            takeoff_futures.append((vehicle_name, future))
    
    # Wait for all takeoffs to complete
    if takeoff_futures:
        print(f"⏳ Waiting for {len(takeoff_futures)} vehicle(s) to complete takeoff...")
        
        for vehicle_name, future in takeoff_futures:
            try:
                future.join()
                print(f"✅ '{vehicle_name}' takeoff completed")
            except Exception as e:
                print(f"❌ '{vehicle_name}' takeoff failed: {str(e)}")
        
        # Give vehicles a moment to stabilize
        print("⏳ Allowing vehicles to stabilize...")
        time.sleep(2)
    
    # Display states of all vehicles
    print("\n📊 Vehicle States After Setup:")
    print("-" * 40)
    
    for vehicle_name, vehicle_type in active_vehicles:
        get_vehicle_state(client, vehicle_name, vehicle_type)
    
    # Optional: Move vehicles to formation
    airsim.wait_key('\nPress any key to move vehicles to formation...')
    
    print("\n🎯 Moving vehicles to formation...")
    move_futures = []
    
    for i, (vehicle_name, vehicle_type) in enumerate(active_vehicles):
        future = move_vehicle_to_formation(client, vehicle_name, vehicle_type, i, len(active_vehicles))
        if future:
            move_futures.append((vehicle_name, future))
    
    # Wait for formation movements to complete
    if move_futures:
        print("⏳ Waiting for formation completion...")
        
        for vehicle_name, future in move_futures:
            try:
                future.join()
                print(f"✅ '{vehicle_name}' reached target position")
            except Exception as e:
                print(f"❌ '{vehicle_name}' movement failed: {str(e)}")
    
    # Final state check
    print("\n📊 Final Vehicle States:")
    print("-" * 40)
    
    for vehicle_name, vehicle_type in active_vehicles:
        get_vehicle_state(client, vehicle_name, vehicle_type)
    
    # Optional: Capture images from all vehicles
    airsim.wait_key('\nPress any key to capture images from all vehicles...')
    
    print("\n📸 Capturing images from all vehicles...")
    
    tmp_dir = os.path.join(tempfile.gettempdir(), "cosys_airsim_multi_agent")
    print(f"💾 Saving images to {tmp_dir}")
    
    try:
        os.makedirs(tmp_dir, exist_ok=True)
    except OSError as e:
        print(f"❌ Failed to create directory: {e}")
    
    for vehicle_name, vehicle_type in active_vehicles:
        try:
            # Request images from each vehicle
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                airsim.ImageRequest("0", airsim.ImageType.DepthVis)
            ], vehicle_name=vehicle_name)
            
            print(f"📷 Captured {len(responses)} images from '{vehicle_name}'")
            
            for idx, response in enumerate(responses):
                filename = os.path.join(tmp_dir, f"{vehicle_name}_{idx}")
                
                if response.compress:  # PNG format
                    airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
                else:  # Uncompressed array
                    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                    img_rgb = img1d.reshape(response.height, response.width, 3)
                    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)
                    
        except Exception as e:
            print(f"❌ Failed to capture images from '{vehicle_name}': {str(e)}")
    
    # Cleanup
    airsim.wait_key('\nPress any key to reset and cleanup...')
    
    cleanup_vehicles(client, active_vehicles)
    client.reset()
    
    print("\n🎉 Cosys-AirSim multi-agent demo completed successfully!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        print("👋 Goodbye!")