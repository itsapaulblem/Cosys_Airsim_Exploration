#!/usr/bin/env python3
"""
Advanced LiDAR debugging to check if sensor is working
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

def test_lidar_detailed():
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim")

    # Get list of all available drones
    vehicles = client.listVehicles()
    print(f"Found {len(vehicles)} vehicles: {vehicles}")

    drone_names = []
    for vehicle_name in vehicles:
        try:
            # Try to get multirotor state - if this works, it's a multirotor
            client.getMultirotorState(vehicle_name=vehicle_name)
            drone_names.append(vehicle_name)
            print(f"✓ {vehicle_name} is a multirotor drone")
        except:
            print(f"✗ {vehicle_name} is not a multirotor (skipping)")

    if not drone_names:
        print("No multirotor drones found! Please check your settings.json configuration.")
        sys.exit(1)

    print(f"\nPreparing {len(drone_names)} drones: {drone_names}")

    # Enable API control and arm all drones
    # for drone_name in drone_names:
    drone_name = drone_names[0]
    print(f"Enabling API control for {drone_name}...")
    client.enableApiControl(True, drone_name)
    print(f"Arming {drone_name}...")
    client.armDisarm(True, drone_name)

    airsim.wait_key('Press any key to takeoff drones one by one')
    
    # drone_count = 0
    # Enable API control for Drone1
    # for i, drone_name in enumerate(drone_names):
        # print(f"Taking off drone {i+1}/{len(drone_names)}: {drone_name}")
    # vehicle_name = drone_name[i]
    takeoff_future = client.takeoffAsync(vehicle_name=drone_name)
    takeoff_future.join()  # Wait for this drone to complete takeoff before starting next
    print(f"✓ {drone_name} takeoff completed")

    print("\nAll drones have taken off successfully!")
    
    
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)
    
    print(f"\nTesting {drone_name}")
    print("="*60)
    
    # Get drone state
    state = client.getMultirotorState(drone_name)
    print(f"Drone position: {state.kinematics_estimated.position}")
    print(f"Drone orientation: {state.kinematics_estimated.orientation}")
    
    # Try different sensor names
    sensor_names = ["LidarSensor1", "LidarSensor", ""]  # Empty string for default
    
    for sensor_name in sensor_names:
        print(f"\n{'='*60}")
        print(f"Testing sensor: '{sensor_name}' (empty = default)")
        print('='*60)
        
        try:
            # Get LiDAR data
            if sensor_name:
                lidar_data = client.getLidarData(sensor_name, drone_name)
            else:
                lidar_data = client.getLidarData()  # Use default
            
            print(f"Time stamp: {lidar_data.time_stamp}")
            print(f"Sensor pose position: {lidar_data.pose.position}")
            print(f"Sensor pose orientation: {lidar_data.pose.orientation}")
            print(f"Raw point cloud size: {len(lidar_data.point_cloud)}")
            print(f"Number of points: {len(lidar_data.point_cloud) // 3}")
            
            if len(lidar_data.point_cloud) > 0:
                # Convert to numpy array
                points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
                
                # Analyze the data
                print(f"\nPoint cloud analysis:")
                print(f"  Shape: {points.shape}")
                print(f"  Data type: {points.dtype}")
                
                # Check for origin points
                origin_mask = np.all(np.abs(points) < 0.001, axis=1)
                num_origin = np.sum(origin_mask)
                print(f"  Points at origin (0,0,0): {num_origin}/{len(points)}")
                
                # Check for valid points
                non_origin_mask = ~origin_mask
                if np.any(non_origin_mask):
                    valid_points = points[non_origin_mask]
                    print(f"\n  Valid (non-origin) points: {len(valid_points)}")
                    print(f"  X range: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
                    print(f"  Y range: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
                    print(f"  Z range: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
                    
                    # Show first few valid points
                    print(f"\n  First 5 valid points:")
                    for i, pt in enumerate(valid_points[:5]):
                        print(f"    {i}: ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
                        # Also show distance from origin
                        dist = np.linalg.norm(pt)
                        print(f"       Distance from sensor: {dist:.3f}")
                else:
                    print(f"\n  WARNING: All {len(points)} points are at origin!")
                    
                    # Check raw data
                    print(f"\n  First 10 raw values from point_cloud:")
                    for i in range(min(30, len(lidar_data.point_cloud))):
                        if i % 3 == 0:
                            print(f"    Point {i//3}: ", end="")
                        print(f"{lidar_data.point_cloud[i]:.6f}", end="")
                        if i % 3 == 2:
                            print()  # New line after Z coordinate
                        else:
                            print(", ", end="")
                            
                    # Check if it's a pattern
                    if len(lidar_data.point_cloud) >= 12:
                        # Check if all points are exactly (0, 0, 0)
                        all_zero = all(v == 0.0 for v in lidar_data.point_cloud)
                        print(f"\n  All values exactly 0.0: {all_zero}")
                        
                        # Check for any non-zero values
                        non_zero_indices = [i for i, v in enumerate(lidar_data.point_cloud) if v != 0.0]
                        if non_zero_indices:
                            print(f"  Found {len(non_zero_indices)} non-zero values at indices: {non_zero_indices[:10]}")
                
            else:
                print("No point cloud data returned")
                
        except Exception as e:
            print(f"Error getting LiDAR data: {e}")
            import traceback
            traceback.print_exc()
            
    # Try to fly up and test again
    print(f"\n{'='*60}")
    print("Taking off and testing at altitude...")
    print('='*60)
    
    # Take off
    print("Taking off...")
    client.takeoffAsync(vehicle_name=drone_name).join()
    time.sleep(3)
    
    # Move up to ensure we're above ground
    print("Moving to altitude -10 (10m up in NED)...")
    client.moveToZAsync(-10, 2, vehicle_name=drone_name).join()
    time.sleep(2)
    
    # Test LiDAR again at altitude
    print("\nTesting LiDAR at altitude...")
    try:
        lidar_data = client.getLidarData("LidarSensor1", drone_name)
        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        
        origin_mask = np.all(np.abs(points) < 0.001, axis=1)
        num_origin = np.sum(origin_mask)
        print(f"Points at origin: {num_origin}/{len(points)}")
        
        if num_origin < len(points):
            valid_points = points[~origin_mask]
            print(f"Valid points found! Count: {len(valid_points)}")
            print(f"Distance range: [{np.min(np.linalg.norm(valid_points, axis=1)):.3f}, "
                  f"{np.max(np.linalg.norm(valid_points, axis=1)):.3f}]")
        else:
            print("Still all points at origin even at altitude!")
            
    except Exception as e:
        print(f"Error: {e}")
    
    # Land
    print("\nLanding...")
    client.landAsync(vehicle_name=drone_name).join()
    
    # Reset
    client.armDisarm(False, drone_name)
    client.enableApiControl(False, drone_name)
    print("\nDone!")

if __name__ == "__main__":
    test_lidar_detailed()