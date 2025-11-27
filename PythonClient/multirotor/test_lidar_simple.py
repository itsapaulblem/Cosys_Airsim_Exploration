#!/usr/bin/env python3
"""
Simple LiDAR test without numpy dependency
"""

import sys
import numpy as np
import os
import time
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    import cosysairsim as airsim
    print("Using cosysairsim")
except ImportError:
    try:
        import airsim
        print("Using airsim")
    except ImportError:
        print("Error: Neither cosysairsim nor airsim package found")
        sys.exit(1)

def test_lidar_basic():
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim")
    
    # Get list of vehicles
    vehicles = client.listVehicles()
    print(f"Available vehicles: {vehicles}")
    
    # Test Drone1 with LidarSensor1
    if "Drone1" in vehicles:
        try:
            print("\nTesting Drone1/LidarSensor1...")
            lidar_data = client.getLidarData("LidarSensor1", "Drone1")
            
            if lidar_data.point_cloud:
                print(f"Point cloud size: {len(lidar_data.point_cloud)}")
                print(f"Number of points: {len(lidar_data.point_cloud) // 3}")
                
                # Check first 10 values
                print(f"First 10 values: {lidar_data.point_cloud[:10]}")
                
                # Check if all values are zero
                non_zero_count = sum(1 for x in lidar_data.point_cloud if x != 0.0)
                print(f"Non-zero values: {non_zero_count} / {len(lidar_data.point_cloud)}")
                
                if non_zero_count > 0:
                    print("✅ SUCCESS: LiDAR returning valid data!")
                    
                    # Find min/max values manually
                    min_val = min(lidar_data.point_cloud)
                    max_val = max(lidar_data.point_cloud)
                    print(f"Value range: {min_val:.3f} to {max_val:.3f}")
                else:
                    print("❌ FAIL: All values are still zeros")
                    
            else:
                print("❌ Point cloud is empty")
                
        except Exception as e:
            print(f"❌ Error: {e}")
    else:
        print("Drone1 not found in vehicle list")

if __name__ == "__main__":
    test_lidar_basic()