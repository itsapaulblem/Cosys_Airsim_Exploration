#!/usr/bin/env python3
"""
Diagnose vehicle positions in AirSim using the same API as ROS nodes
"""

import sys
import os

# Add AirSim Python client to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

try:
    import cosysairsim as airsim
    print("Using cosysairsim client")
except ImportError:
    try:
        import airsim
        print("Using standard airsim client")
    except ImportError:
        print("ERROR: No AirSim Python client found!")
        sys.exit(1)

def main():
    # Connect to AirSim
    client = airsim.MultirotorClient()
    
    try:
        client.confirmConnection()
        print("Connected!")
        
        # Check client/server versions (simplified)
        try:
            client_ver = client.getClientVersion()
            server_ver = client.getServerVersion()
            print(f"Client Ver: {client_ver}, Server Ver: {server_ver}")
        except:
            print("Version info not available, continuing...")
        print()
        
        # Get list of vehicles
        vehicles = client.listVehicles()
        print(f"Discovered vehicles: {vehicles}")
        print()
        
        # Test each vehicle position using ROS-compatible API
        for vehicle_name in vehicles:
            try:
                # Use same API as ROS multirotor_node.cpp
                state = client.getMultirotorState(vehicle_name)
                
                # Access position using Python API (different from C++ ROS API)
                kinematics = state.kinematics_estimated
                pos = kinematics.position
                
                print(f"{vehicle_name}:")
                print(f"  Raw position (NED): [{pos.x_val:.3f}, {pos.y_val:.3f}, {pos.z_val:.3f}]")
                
                # Apply same NED->ENU conversion as ROS
                enu_x = pos.x_val
                enu_y = -pos.y_val  
                enu_z = -pos.z_val
                print(f"  Converted (ENU):    [{enu_x:.3f}, {enu_y:.3f}, {enu_z:.3f}]")
                
                # Check if vehicle is armed/active
                try:
                    is_armed = state.armed
                    print(f"  Armed: {is_armed}")
                except:
                    print(f"  Armed: Unknown")
                    
                print()
                
            except Exception as e:
                print(f"{vehicle_name}: ERROR - {e}")
                print()
        
        # Also test GPS data access
        print("=== GPS Data Test ===")
        for vehicle_name in vehicles:
            try:
                gps_data = client.getGpsData("", vehicle_name)
                lat = gps_data.gnss.geo_point.latitude
                lon = gps_data.gnss.geo_point.longitude
                alt = gps_data.gnss.geo_point.altitude
                print(f"{vehicle_name} GPS: [{lat:.6f}, {lon:.6f}, {alt:.3f}]")
            except Exception as e:
                print(f"{vehicle_name} GPS: ERROR - {e}")
                
    except Exception as e:
        print(f"Connection failed: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())