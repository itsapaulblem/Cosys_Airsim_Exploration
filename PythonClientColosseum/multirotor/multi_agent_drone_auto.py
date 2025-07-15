#!/usr/bin/env python3
"""
Enhanced Multi-Agent Drone Script with Auto Vehicle Discovery
===========================================================

This script automatically discovers available vehicles in the AirSim simulation
and makes them take off. Since listVehicles() is not available in Colosseum,
it uses a discovery method to detect active vehicles.

Features:
- Automatic vehicle discovery
- Batch takeoff for all discovered vehicles
- Error handling for individual vehicles
- Configurable vehicle detection
"""

import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import time

def discover_vehicles(client, max_vehicles=10):
    """
    Discover available vehicles by trying common vehicle names and checking their state.
    
    Args:
        client: AirSim client instance
        max_vehicles (int): Maximum number of vehicles to check for
        
    Returns:
        list[str]: List of discovered vehicle names
    """
    discovered_vehicles = []
    
    # Common vehicle naming patterns to try
    vehicle_patterns = [
        # Standard patterns
        ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5"],
        ["drone1", "drone2", "drone3", "drone4", "drone5"], 
        ["UAV1", "UAV2", "UAV3", "UAV4", "UAV5"],
        ["Quadcopter1", "Quadcopter2", "Quadcopter3"],
        ["Vehicle1", "Vehicle2", "Vehicle3", "Vehicle4", "Vehicle5"],
        # Default empty name (primary vehicle)
        [""]
    ]
    
    print("🔍 Discovering available vehicles...")
    
    # Flatten the patterns into a single list
    all_patterns = []
    for pattern_group in vehicle_patterns:
        all_patterns.extend(pattern_group)
    
    for vehicle_name in all_patterns[:max_vehicles]:
        try:
            # Try to get the vehicle state to check if it exists
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            
            # If we get here without exception, the vehicle exists
            display_name = vehicle_name if vehicle_name else "Default Vehicle"
            print(f"✅ Found vehicle: '{display_name}'")
            discovered_vehicles.append(vehicle_name)
            
        except Exception as e:
            # Vehicle doesn't exist or isn't accessible, skip silently
            pass
    
    if not discovered_vehicles:
        print("❌ No vehicles discovered. Make sure AirSim is running with vehicles configured.")
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
    else:
        print(f"🎯 Discovered {len(discovered_vehicles)} vehicle(s): {discovered_vehicles}")
    
    return discovered_vehicles

def setup_vehicle(client, vehicle_name):
    """
    Setup a single vehicle for API control.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle to setup
        
    Returns:
        bool: True if setup successful, False otherwise
    """
    try:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        print(f"⚙️  Setting up '{display_name}'...")
        
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        
        # Arm the vehicle
        client.armDisarm(True, vehicle_name)
        
        print(f"✅ '{display_name}' setup complete")
        return True
        
    except Exception as e:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        print(f"❌ Failed to setup '{display_name}': {str(e)}")
        return False

def takeoff_vehicle(client, vehicle_name):
    """
    Make a single vehicle take off asynchronously.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle to takeoff
        
    Returns:
        Future or None: Takeoff future if successful, None otherwise
    """
    try:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        print(f"🚁 Starting takeoff for '{display_name}'...")
        
        future = client.takeoffAsync(vehicle_name=vehicle_name)
        return future
        
    except Exception as e:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        print(f"❌ Failed to start takeoff for '{display_name}': {str(e)}")
        return None

def get_vehicle_state(client, vehicle_name):
    """
    Get and print vehicle state.
    
    Args:
        client: AirSim client instance
        vehicle_name (str): Name of the vehicle
    """
    try:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        print(f"\n📊 State for '{display_name}':")
        print(f"   Position: x={state.kinematics_estimated.position.x_val:.2f}, "
              f"y={state.kinematics_estimated.position.y_val:.2f}, "
              f"z={state.kinematics_estimated.position.z_val:.2f}")
        print(f"   Landed: {state.landed_state}")
        print(f"   Armed: {state.armed}")
        
    except Exception as e:
        display_name = vehicle_name if vehicle_name else "Default Vehicle"
        print(f"❌ Failed to get state for '{display_name}': {str(e)}")

def cleanup_vehicles(client, vehicle_names):
    """
    Clean up all vehicles by disarming and disabling API control.
    
    Args:
        client: AirSim client instance
        vehicle_names (list[str]): List of vehicle names to cleanup
    """
    print("\n🧹 Cleaning up vehicles...")
    
    for vehicle_name in vehicle_names:
        try:
            display_name = vehicle_name if vehicle_name else "Default Vehicle"
            client.armDisarm(False, vehicle_name)
            client.enableApiControl(False, vehicle_name)
            print(f"✅ Cleaned up '{display_name}'")
        except Exception as e:
            display_name = vehicle_name if vehicle_name else "Default Vehicle"
            print(f"⚠️  Failed to cleanup '{display_name}': {str(e)}")

def main():
    """
    Main function to run the multi-agent drone demo with auto-discovery.
    """
    print("🚁 Multi-Agent Drone Auto-Discovery Demo")
    print("=" * 50)
    
    # Connect to AirSim
    print("🔗 Connecting to AirSim...")
    client = airsim.MultirotorClient()
    
    try:
        client.confirmConnection()
        print("✅ Connected to AirSim successfully")
    except Exception as e:
        print(f"❌ Failed to connect to AirSim: {str(e)}")
        print("Make sure AirSim is running and accessible on localhost:41451")
        return
    
    # Discover available vehicles
    vehicles = discover_vehicles(client)
    
    if not vehicles:
        print("❌ No vehicles found. Exiting.")
        return
    
    # Setup all vehicles
    print(f"\n⚙️  Setting up {len(vehicles)} vehicle(s)...")
    setup_success = []
    
    for vehicle in vehicles:
        success = setup_vehicle(client, vehicle)
        setup_success.append(success)
    
    # Filter to only successfully setup vehicles
    active_vehicles = [vehicle for vehicle, success in zip(vehicles, setup_success) if success]
    
    if not active_vehicles:
        print("❌ No vehicles could be setup successfully. Exiting.")
        return
    
    print(f"✅ Successfully setup {len(active_vehicles)} vehicle(s)")
    
    # Wait for user input before takeoff
    print(f"\n🚁 Ready to takeoff {len(active_vehicles)} vehicle(s)")
    airsim.wait_key('Press any key to initiate takeoff sequence...')
    
    # Start takeoff for all vehicles
    print("\n🚀 Initiating takeoff sequence...")
    takeoff_futures = []
    
    for vehicle in active_vehicles:
        future = takeoff_vehicle(client, vehicle)
        if future:
            takeoff_futures.append((vehicle, future))
    
    # Wait for all takeoffs to complete
    print(f"⏳ Waiting for {len(takeoff_futures)} vehicle(s) to complete takeoff...")
    
    for vehicle, future in takeoff_futures:
        try:
            future.join()
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"✅ '{display_name}' takeoff completed")
        except Exception as e:
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"❌ '{display_name}' takeoff failed: {str(e)}")
    
    # Give vehicles a moment to stabilize
    print("⏳ Allowing vehicles to stabilize...")
    time.sleep(2)
    
    # Display states of all vehicles
    print("\n📊 Vehicle States After Takeoff:")
    print("-" * 40)
    
    for vehicle in active_vehicles:
        get_vehicle_state(client, vehicle)
    
    # Optional: Move vehicles to different positions
    airsim.wait_key('\nPress any key to move vehicles to formation...')
    
    print("\n🎯 Moving vehicles to formation...")
    move_futures = []
    
    for i, vehicle in enumerate(active_vehicles):
        try:
            # Create a simple formation pattern
            x = -5 + (i * 3)  # Spread vehicles along X axis
            y = 5 - (i * 2)   # Stagger along Y axis  
            z = -10           # Fixed altitude
            
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"📍 Moving '{display_name}' to position ({x}, {y}, {z})")
            
            future = client.moveToPositionAsync(x, y, z, 5, vehicle_name=vehicle)
            move_futures.append((vehicle, future))
            
        except Exception as e:
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"❌ Failed to move '{display_name}': {str(e)}")
    
    # Wait for all movements to complete
    print("⏳ Waiting for formation completion...")
    
    for vehicle, future in move_futures:
        try:
            future.join()
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"✅ '{display_name}' reached target position")
        except Exception as e:
            display_name = vehicle if vehicle else "Default Vehicle"
            print(f"❌ '{display_name}' movement failed: {str(e)}")
    
    # Final state check
    print("\n📊 Final Vehicle States:")
    print("-" * 40)
    
    for vehicle in active_vehicles:
        get_vehicle_state(client, vehicle)
    
    # Cleanup
    airsim.wait_key('\nPress any key to reset and cleanup...')
    
    cleanup_vehicles(client, active_vehicles)
    client.reset()
    
    print("\n🎉 Multi-agent drone demo completed successfully!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {str(e)}")
    finally:
        print("👋 Goodbye!")