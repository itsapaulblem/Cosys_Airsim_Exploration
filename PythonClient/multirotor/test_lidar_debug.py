#!/usr/bin/env python3
"""
Simple LiDAR data retrieval for comparison with ROS2
Outputs data in a format easy to compare with ros2 topic echo
"""

import numpy as np
import struct
import sys
import argparse
import os
import setup_path 

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

def get_lidar_data_simple(vehicle_name="Drone_1", sensor_name="LidarSensor1", show_bytes=False):
    """Get LiDAR data and display in comparable format"""
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Get LiDAR data
        lidar_data = client.getLidarData(sensor_name, vehicle_name)
        
        # Header info
        print("="*60)
        print(f"Python RPC LiDAR Data")
        print("="*60)
        print(f"Vehicle: {vehicle_name}")
        print(f"Sensor: {sensor_name}")
        print(f"Timestamp: {lidar_data.time_stamp}")
        print(f"Sensor pose:")
        print(f"  Position: ({lidar_data.pose.position.x_val:.3f}, {lidar_data.pose.position.y_val:.3f}, {lidar_data.pose.position.z_val:.3f})")
        print(f"  Orientation (wxyz): ({lidar_data.pose.orientation.w_val:.3f}, {lidar_data.pose.orientation.x_val:.3f}, {lidar_data.pose.orientation.y_val:.3f}, {lidar_data.pose.orientation.z_val:.3f})")
        
        # Point cloud info
        print(f"\nPoint Cloud:")
        print(f"  Raw array size: {len(lidar_data.point_cloud)} floats")
        print(f"  Number of points: {len(lidar_data.point_cloud) // 3}")
        
        if len(lidar_data.point_cloud) == 0:
            print("  ❌ No points returned!")
            return None
            
        # Convert to numpy array
        points = np.array(lidar_data.point_cloud, dtype=np.float32)
        points = points.reshape(-1, 3)
        
        # Statistics
        origin_mask = np.all(np.abs(points) < 0.001, axis=1)
        valid_mask = ~origin_mask
        num_origin = np.sum(origin_mask)
        num_valid = np.sum(valid_mask)
        
        print(f"  Points at origin: {num_origin}")
        print(f"  Valid points: {num_valid}")
        
        if num_valid > 0:
            valid_points = points[valid_mask]
            print(f"\nValid point statistics:")
            print(f"  X range: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
            print(f"  Y range: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
            print(f"  Z range: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
            
            distances = np.linalg.norm(valid_points, axis=1)
            print(f"  Distance range: [{np.min(distances):.3f}, {np.max(distances):.3f}]")
            
            # Show first few valid points (same format as ROS2 comparison)
            print(f"\nFirst 20 valid points (AirSim NED coordinates):")
            print(f"{'Index':<7} {'X':>10} {'Y':>10} {'Z':>10} {'Distance':>10}")
            print("-" * 50)
            for i in range(min(20, len(valid_points))):
                pt = valid_points[i]
                dist = np.linalg.norm(pt)
                print(f"[{i:4d}]: {pt[0]:10.4f} {pt[1]:10.4f} {pt[2]:10.4f} {dist:10.4f}")
        
        # Show bytes if requested (for exact comparison with ROS2)
        if show_bytes:
            print(f"\nFirst 120 bytes as integers (10 points * 3 floats * 4 bytes):")
            byte_data = points.flatten()[:10*3].tobytes()
            byte_ints = list(byte_data)
            
            for i in range(0, min(120, len(byte_ints)), 12):
                # Show one point per line (12 bytes = 3 floats)
                point_bytes = byte_ints[i:i+12]
                print(f"Point {i//12}: {point_bytes}")
                
                # Decode and show the float values
                if len(point_bytes) == 12:
                    x = struct.unpack('<f', bytes(point_bytes[0:4]))[0]
                    y = struct.unpack('<f', bytes(point_bytes[4:8]))[0]
                    z = struct.unpack('<f', bytes(point_bytes[8:12]))[0]
                    print(f"         = ({x:.4f}, {y:.4f}, {z:.4f})")
        
        # Expected ROS2 transformation
        print(f"\n" + "="*60)
        print("Expected ROS2 Transformation:")
        print("="*60)
        print("ROS2 should apply: Y_ros = -Y_airsim")
        print("\nFirst 5 valid points after expected ROS2 transform:")
        if num_valid > 0:
            for i in range(min(5, len(valid_points))):
                pt = valid_points[i]
                pt_ros = np.array([pt[0], -pt[1], pt[2]])  # Y-flip
                print(f"  AirSim: ({pt[0]:8.4f}, {pt[1]:8.4f}, {pt[2]:8.4f})")
                print(f"  ROS2:   ({pt_ros[0]:8.4f}, {pt_ros[1]:8.4f}, {pt_ros[2]:8.4f})")
                print()
        
        return points
        
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def main():
    parser = argparse.ArgumentParser(description='Get LiDAR data from AirSim for ROS2 comparison')
    parser.add_argument('--vehicle', default='Drone_1', help='Vehicle name (default: Drone_1)')
    parser.add_argument('--sensor', default='LidarSensor1', help='Sensor name (default: LidarSensor1)')
    parser.add_argument('--bytes', action='store_true', help='Show byte representation')
    
    args = parser.parse_args()
    
    points = get_lidar_data_simple(args.vehicle, args.sensor, args.bytes)
    
    if points is not None:
        print(f"\n✅ Successfully retrieved {len(points)} points")
        print("\nTo compare with ROS2:")
        print(f"  1. Run: ros2 topic echo /{args.vehicle}/{args.sensor}/points")
        print(f"  2. Look for 'width:' field (should match {len(points)})")
        print(f"  3. Check 'data:' array for byte values")
        print(f"  4. Verify Y-axis is flipped in ROS2 data")
    else:
        print("\n❌ Failed to retrieve LiDAR data")
        print("Check that:")
        print(f"  - Vehicle '{args.vehicle}' exists in settings.json")
        print(f"  - Sensor '{args.sensor}' is configured for the vehicle")
        print(f"  - AirSim is running")

if __name__ == "__main__":
    main()
