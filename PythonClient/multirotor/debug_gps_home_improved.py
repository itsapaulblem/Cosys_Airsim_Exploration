#!/usr/bin/env python3
"""
Debug script for troubleshooting "waiting for valid GPS home" error in AirSim PX4
This script monitors GPS messages and home position status to help diagnose issues.
"""
import setup_path
import cosysairsim as airsim
import time
import sys

def list_vehicles(client):
    """List all available vehicles in the simulation"""
    try:
        # Try to get vehicle list using simListSceneObjects
        vehicles = []
        scene_objects = client.simListSceneObjects()
        
        # Common vehicle name patterns
        vehicle_patterns = ['PX4', 'Drone', 'drone', 'multirotor', 'Multirotor', 
                          'SimpleFlight', 'ArduCopter', 'PX4_', 'Vehicle']
        
        for obj in scene_objects:
            for pattern in vehicle_patterns:
                if pattern in obj:
                    vehicles.append(obj)
                    break
        
        return vehicles
    except:
        return []

def get_default_vehicle_name(client):
    """Try to determine the default vehicle name"""
    # First try empty string (default vehicle)
    try:
        state = client.getMultirotorState()
        return ""  # Empty string means default vehicle
    except:
        pass
    
    # Try common names
    common_names = ["", "PX4", "SimpleFlight", "Drone1", "PX4_1"]
    for name in common_names:
        try:
            client.getMultirotorState(vehicle_name=name)
            return name
        except:
            continue
    
    return None

def main():
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    print("Connected to AirSim")
    print("=" * 50)
    
    # Try to find available vehicles
    print("Searching for vehicles...")
    vehicles = list_vehicles(client)
    if vehicles:
        print(f"Found possible vehicles: {vehicles}")
    
    # Get default vehicle name
    vehicle_name = get_default_vehicle_name(client)
    
    if vehicle_name is None:
        print("\nERROR: No vehicles found in the simulation!")
        print("\nPossible reasons:")
        print("1. AirSim is not running or not fully loaded")
        print("2. No vehicles are configured in settings.json")
        print("3. Vehicle type mismatch (e.g., trying to access multirotor API for a car)")
        print("\nPlease check your settings.json and ensure it contains a vehicle configuration.")
        return
    
    if vehicle_name == "":
        print(f"\nUsing default vehicle")
    else:
        print(f"\nUsing vehicle: '{vehicle_name}'")
    
    try:
        # Enable API control (but don't arm yet)
        client.enableApiControl(True, vehicle_name)
        print(f"API control enabled")
    except Exception as e:
        print(f"Error enabling API control: {e}")
        print("\nTrying to continue without API control...")
    
    # Monitor GPS and home position
    print("\nMonitoring GPS and Home Position Status...")
    print("Press Ctrl+C to stop monitoring\n")
    
    try:
        while True:
            try:
                # Get GPS data
                gps_data = client.getGpsData(vehicle_name=vehicle_name)
                
                # Get home geo point
                home_point = client.getHomeGeoPoint(vehicle_name=vehicle_name)
                
                # Get vehicle state
                state = client.getMultirotorState(vehicle_name=vehicle_name)
                
                # Clear screen for better readability
                print("\033[2J\033[H")  # Clear screen and move cursor to top
                
                print("GPS STATUS MONITOR")
                print("=" * 50)
                print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                if vehicle_name:
                    print(f"Vehicle: {vehicle_name}")
                else:
                    print("Vehicle: Default")
                print()
                
                # GPS Information
                print("GPS Data:")
                print(f"  Latitude:  {gps_data.gnss.geo_point.latitude:.8f}")
                print(f"  Longitude: {gps_data.gnss.geo_point.longitude:.8f}")
                print(f"  Altitude:  {gps_data.gnss.geo_point.altitude:.2f} m")
                print(f"  Fix Type:  {gps_data.gnss.fix_type}")
                print(f"  Valid:     {gps_data.is_valid}")
                print(f"  EPH:       {gps_data.gnss.eph:.2f}")
                print(f"  EPV:       {gps_data.gnss.epv:.2f}")
                print()
                
                # Home Position
                print("Home Position:")
                if home_point.latitude != 0 or home_point.longitude != 0:
                    print(f"  Latitude:  {home_point.latitude:.8f}")
                    print(f"  Longitude: {home_point.longitude:.8f}")
                    print(f"  Altitude:  {home_point.altitude:.2f} m")
                    print("  Status:    SET ✓")
                else:
                    print("  Status:    NOT SET ✗")
                print()
                
                # Vehicle State
                print("Vehicle State:")
                print(f"  Armed:     {state.is_armable}")
                print(f"  Position:  X={state.kinematics_estimated.position.x_val:.2f}, "
                      f"Y={state.kinematics_estimated.position.y_val:.2f}, "
                      f"Z={state.kinematics_estimated.position.z_val:.2f}")
                print()
                
                # Recommendations
                print("Troubleshooting:")
                if not gps_data.is_valid:
                    print("  ⚠ GPS data is not valid. Check GPS sensor settings.")
                if gps_data.gnss.fix_type < 3:
                    print("  ⚠ GPS fix type is less than 3D fix. Waiting for better GPS lock.")
                if home_point.latitude == 0 and home_point.longitude == 0:
                    print("  ⚠ Home position not set. Check OriginGeopoint in settings.json")
                if gps_data.gnss.eph > 5:
                    print("  ⚠ GPS horizontal accuracy is poor (EPH > 5)")
                
            except Exception as e:
                print(f"\nError reading vehicle data: {e}")
                print("Retrying in 5 seconds...")
                time.sleep(5)
                continue
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Disable API control
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

if __name__ == "__main__":
    main()