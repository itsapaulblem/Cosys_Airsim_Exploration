#!/usr/bin/env python3
"""
Simple script to decode ROS2 PointCloud2 data and compare with Python RPC
"""

import struct
import numpy as np

def decode_ros2_bytes(byte_list):
    """Decode ROS2 PointCloud2 byte data to points"""
    # Convert list of integers to bytes
    byte_data = bytes(byte_list)
    
    # Each point is 12 bytes (3 floats of 4 bytes each)
    point_step = 12
    num_points = len(byte_data) // point_step
    
    points = []
    for i in range(num_points):
        offset = i * point_step
        if offset + 12 <= len(byte_data):
            # Extract x, y, z as floats
            x = struct.unpack('<f', byte_data[offset:offset+4])[0]  # Little-endian float
            y = struct.unpack('<f', byte_data[offset+4:offset+8])[0]
            z = struct.unpack('<f', byte_data[offset+8:offset+12])[0]
            points.append([x, y, z])
    
    return np.array(points, dtype=np.float32)

def analyze_pointcloud(points, source_name="Data"):
    """Analyze point cloud statistics"""
    print(f"\n{source_name} Analysis:")
    print(f"  Total points: {len(points)}")
    
    # Filter out points at origin (0,0,0) or (-0,-0,-0)
    origin_mask = np.all(np.abs(points) < 0.001, axis=1)
    num_origin = np.sum(origin_mask)
    
    print(f"  Points at origin: {num_origin}")
    
    # Get valid (non-origin) points
    valid_points = points[~origin_mask]
    print(f"  Valid points: {len(valid_points)}")
    
    if len(valid_points) > 0:
        # Statistics
        print(f"\n  Valid point ranges:")
        print(f"    X: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
        print(f"    Y: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
        print(f"    Z: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
        
        # Distance statistics
        distances = np.linalg.norm(valid_points, axis=1)
        print(f"    Distance: [{np.min(distances):.3f}, {np.max(distances):.3f}]")
        
        # Show first few valid points
        print(f"\n  First 10 valid points:")
        for i in range(min(10, len(valid_points))):
            pt = valid_points[i]
            dist = np.linalg.norm(pt)
            print(f"    [{i:3d}]: ({pt[0]:8.3f}, {pt[1]:8.3f}, {pt[2]:8.3f}) | dist={dist:6.3f}")
    
    return valid_points

# Example ROS2 data from your output
# The pattern [0, 0, 0, 128] represents -0.0 in IEEE 754 (sign bit set)
# Later values like [71, 203, 91, 193] represent actual measurements

# Sample of your ROS2 data
ros2_sample_bytes = [
    0, 0, 0, 128, 0, 0, 0, 128, 0, 0, 0, 128,  # Point 1: (-0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 2: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 3: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 4: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 5: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 6: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 7: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 8: (0, -0, -0)
    0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128,     # Point 9: (0, -0, -0)
    71, 203, 91, 193, 0, 0, 0, 128, 255, 213, 153, 191,  # Point 10: Real data!
    88, 71, 3, 193, 0, 0, 0, 128                          # Part of Point 11
]

print("="*60)
print("ROS2 PointCloud2 Data Decoder")
print("="*60)

# Decode the sample
points = decode_ros2_bytes(ros2_sample_bytes)
print(f"\nDecoded {len(points)} points from ROS2 data")

# Show raw decoded values
print("\nRaw decoded points:")
for i, pt in enumerate(points[:12]):  # Show first 12 points
    print(f"  Point {i:2d}: ({pt[0]:10.6f}, {pt[1]:10.6f}, {pt[2]:10.6f})")

# Analyze the data
valid_points = analyze_pointcloud(points, "ROS2 Sample")

print("\n" + "="*60)
print("Interpretation:")
print("="*60)
print("\n1. The byte pattern [0, 0, 0, 128] decodes to -0.0 (negative zero)")
print("   This is a special IEEE 754 float value with the sign bit set")
print("   These points are typically invalid/no-return LiDAR readings")
print("\n2. Points with actual distance data have different byte patterns")
print("   For example: [71, 203, 91, 193] decodes to a real coordinate")
print("\n3. Your ROS2 data shows width=8192, meaning 8192 points total")
print("   Many appear to be at origin, which is normal for LiDAR sensors")
print("   (no return = origin point)")

print("\n" + "="*60)
print("To verify against Python RPC:")
print("="*60)
print("\n1. Run the Python test to get RPC data:")
print("   python test_lidar_debug_advanced.py")
print("\n2. Look for:")
print("   - Number of points (should be 8192)")
print("   - Number of valid (non-origin) points")
print("   - Coordinate ranges")
print("\n3. Note the coordinate transformation:")
print("   - AirSim uses NED (North-East-Down)")
print("   - ROS2 typically flips Y axis: ROS_Y = -AirSim_Y")
print("\n4. Use the full verification script:")
print("   python verify_ros2_lidar.py --vehicle Drone_1")