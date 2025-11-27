#!/usr/bin/env python3
"""
Simple diagnostic script to test AirSim position APIs without complex dependencies
"""

import sys
import json
import os

# Add AirSim Python client to path - we're in PythonClient/multirotor
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

print("AIRSIM SPAWN POSITION ANALYSIS")
print("=" * 40)

try:
    import cosysairsim as airsim
    print("Using cosysairsim client")
except ImportError:
    print("cosysairsim not available, trying standard airsim")
    try:
        import airsim
        print("Using standard airsim client") 
    except ImportError:
        print("ERROR: No AirSim Python client available")
        print("Please install numpy in your environment:")
        print("pip install numpy")
        sys.exit(1)

def main():
    # Connect to AirSim
    try:
        client = airsim.MultirotorClient(ip='localhost', port=41451)
        client.confirmConnection()
        print("Connected to AirSim server")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
    
    # Get vehicle list
    try:
        vehicles = client.listVehicles()
        print(f"Vehicles: {vehicles}")
    except Exception as e:
        print(f"Failed to get vehicles: {e}")
        return
    
    print(f"\nTesting position APIs:")
    print("-" * 30)
    
    for vehicle_name in vehicles:
        print(f"\n{vehicle_name}:")
        
        try:
            # Test getMultirotorState 
            state = client.getMultirotorState(vehicle_name)
            pos = state.kinematics_estimated.pose.position
            print(f"  getMultirotorState: [{pos.x_val:.3f}, {pos.y_val:.3f}, {pos.z_val:.3f}]")
            
            # Test simGetVehiclePose
            pose = client.simGetVehiclePose(vehicle_name)
            pos2 = pose.position
            print(f"  simGetVehiclePose:  [{pos2.x_val:.3f}, {pos2.y_val:.3f}, {pos2.z_val:.3f}]")
            
            # Test GPS
            gps = client.getGpsData("", vehicle_name)
            print(f"  GPS: lat={gps.gnss.geo_point.latitude:.6f}, lon={gps.gnss.geo_point.longitude:.6f}")
            
            # Calculate GPS differences
            try:
                origin_gps = client.getHomeGeoPoint("")
                lat_diff = gps.gnss.geo_point.latitude - origin_gps.latitude
                lon_diff = gps.gnss.geo_point.longitude - origin_gps.longitude
                
                import math
                lat_rad = origin_gps.latitude * math.pi / 180.0
                ned_x = lat_diff * 111320.0
                ned_y = lon_diff * 111320.0 * math.cos(lat_rad)
                
                print(f"  GPS->NED calc:      [{ned_x:.3f}, {ned_y:.3f}, ?] (this should be spawn offset)")
                
            except Exception as e:
                print(f"  GPS calc failed: {e}")
                
        except Exception as e:
            print(f"  Failed for {vehicle_name}: {e}")
    
    # Check settings
    print(f"\nSettings from AirSim server:")
    print("-" * 30)
    try:
        settings_str = client.getSettingsString()
        if settings_str:
            settings = json.loads(settings_str)
            if "Vehicles" in settings:
                for name, config in settings["Vehicles"].items():
                    x = config.get("X", "default")
                    y = config.get("Y", "default") 
                    z = config.get("Z", "default")
                    print(f"  {name}: X={x}, Y={y}, Z={z}")
            else:
                print("  No Vehicles section found")
        else:
            print("  Empty settings")
    except Exception as e:
        print(f"  Settings error: {e}")

    print(f"\nCONCLUSION:")
    print("If all vehicles show [0,0,height] but different GPS coordinates,")
    print("then AirSim IS positioning correctly but using LOCAL coordinates by design.")
    print("The GPS-based spawn_offset calculation is the correct approach.")

if __name__ == "__main__":
    main()