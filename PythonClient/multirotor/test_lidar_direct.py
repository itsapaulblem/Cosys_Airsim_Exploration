#!/usr/bin/env python3
"""
Direct test of LiDAR sensor to diagnose zero values issue
"""

import sys
import numpy as np

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

def test_lidar():
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim")
    
    # Get list of vehicles
    vehicles = client.listVehicles()
    print(f"\nAvailable vehicles: {vehicles}")
    
    # Test each vehicle with LiDAR
    test_vehicles = ["Drone1", "Drone2", "Drone3"]  # These have LiDAR configured
    
    for vehicle_name in test_vehicles:
        if vehicle_name in vehicles:
            print(f"\n{'='*50}")
            print(f"Testing {vehicle_name}")
            print('='*50)
            
            try:
                # Get LiDAR data
                lidar_data = client.getLidarData("LidarSensor1", vehicle_name)
                
                # Check if we have data
                if lidar_data.point_cloud:
                    points = np.array(lidar_data.point_cloud, dtype=np.float32)
                    num_points = len(points) // 3
                    
                    if num_points > 0:
                        points_3d = points.reshape(num_points, 3)
                        
                        # Calculate statistics
                        distances = np.linalg.norm(points_3d, axis=1)
                        non_zero_points = points_3d[np.any(points_3d != 0, axis=1)]
                        
                        print(f"Total points: {num_points}")
                        print(f"Non-zero points: {len(non_zero_points)}")
                        print(f"Zero points: {num_points - len(non_zero_points)}")
                        
                        if len(non_zero_points) > 0:
                            print(f"Distance range: {distances[distances > 0].min():.2f}m - {distances.max():.2f}m")
                            print(f"First 5 non-zero points:\n{non_zero_points[:5]}")
                        else:
                            print("WARNING: All points are zeros!")
                            print(f"First 10 raw values: {points[:30]}")
                            
                        # Check sensor pose
                        print(f"\nSensor pose: {lidar_data.pose}")
                        print(f"Time stamp: {lidar_data.time_stamp}")
                        
                        # Check if sensor might be underground or in wrong position
                        if hasattr(lidar_data, 'segmentation'):
                            print(f"Segmentation available: {len(lidar_data.segmentation) if lidar_data.segmentation else 0}")
                    else:
                        print("No points in point cloud!")
                else:
                    print("Point cloud is empty!")
                    
            except Exception as e:
                print(f"Error getting LiDAR data: {e}")
                
            # Also check vehicle position
            try:
                state = client.getMultirotorState(vehicle_name)
                pos = state.kinematics_estimated.position
                print(f"\nVehicle position: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
                
                # Check if vehicle is on ground (might affect LiDAR)
                if abs(pos.z_val) < 0.5:
                    print("WARNING: Vehicle is very close to ground, LiDAR might be blocked")
                    
            except Exception as e:
                print(f"Error getting vehicle state: {e}")

    # Test with raw API call to understand the data format
    print(f"\n{'='*50}")
    print("Raw API Test")
    print('='*50)
    
    try:
        import msgpackrpc
        
        # Direct RPC call
        rpc_client = msgpackrpc.Client(msgpackrpc.Address("localhost", 41451))
        result = rpc_client.call('getLidarData', 'LidarSensor1', 'Drone1')
        print(f"Raw RPC result type: {type(result)}")
        if result and 'point_cloud' in result:
            raw_points = result['point_cloud']
            print(f"Raw point cloud length: {len(raw_points) if raw_points else 0}")
            if raw_points and len(raw_points) > 0:
                print(f"First 10 raw values: {raw_points[:10]}")
    except Exception as e:
        print(f"Raw RPC test failed: {e}")

if __name__ == "__main__":
    test_lidar()