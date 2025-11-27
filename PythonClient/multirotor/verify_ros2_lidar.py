#!/usr/bin/env python3
"""
Verification script to compare ROS2 PointCloud2 messages with Python RPC LiDAR data
This helps verify that the ROS2 wrapper is correctly transforming and publishing data
"""

import sys
import numpy as np
import os
import struct
import time
import subprocess
import json
import threading
from datetime import datetime

# Setup path for AirSim
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

class LiDARVerifier:
    def __init__(self, vehicle_name="Drone1", sensor_name="LidarSensor1"):
        self.vehicle_name = vehicle_name
        self.sensor_name = sensor_name
        self.ros2_data = None
        self.rpc_data = None
        self.client = None
        
    def connect_airsim(self):
        """Connect to AirSim via RPC"""
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print(f"Connected to AirSim for {self.vehicle_name}")
        
        # Enable API control
        self.client.enableApiControl(True, self.vehicle_name)
        self.client.armDisarm(True, self.vehicle_name)
        
    def get_rpc_data(self):
        """Get LiDAR data via Python RPC"""
        try:
            lidar_data = self.client.getLidarData(self.sensor_name, self.vehicle_name)
            
            # Convert to numpy array
            points = np.array(lidar_data.point_cloud, dtype=np.float32)
            if len(points) > 0:
                points = points.reshape(-1, 3)
            
            self.rpc_data = {
                'timestamp': lidar_data.time_stamp,
                'pose': {
                    'position': [lidar_data.pose.position.x_val, 
                                lidar_data.pose.position.y_val, 
                                lidar_data.pose.position.z_val],
                    'orientation': [lidar_data.pose.orientation.w_val,
                                  lidar_data.pose.orientation.x_val,
                                  lidar_data.pose.orientation.y_val,
                                  lidar_data.pose.orientation.z_val]
                },
                'points': points,
                'num_points': len(points),
                'raw_size': len(lidar_data.point_cloud)
            }
            
            print(f"\n{'='*60}")
            print("RPC DATA COLLECTED")
            print(f"{'='*60}")
            print(f"Timestamp: {self.rpc_data['timestamp']}")
            print(f"Number of points: {self.rpc_data['num_points']}")
            print(f"Raw data size: {self.rpc_data['raw_size']} floats")
            print(f"Sensor position: {self.rpc_data['pose']['position']}")
            
            if len(points) > 0:
                # Analyze point cloud
                origin_mask = np.all(np.abs(points) < 0.001, axis=1)
                valid_points = points[~origin_mask]
                
                print(f"\nPoint Analysis:")
                print(f"  Points at origin: {np.sum(origin_mask)}")
                print(f"  Valid points: {len(valid_points)}")
                
                if len(valid_points) > 0:
                    print(f"  X range: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
                    print(f"  Y range: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
                    print(f"  Z range: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
                    
                    print(f"\n  First 5 valid points (RPC - AirSim NED):")
                    for i in range(min(5, len(valid_points))):
                        pt = valid_points[i]
                        print(f"    [{i}]: ({pt[0]:7.3f}, {pt[1]:7.3f}, {pt[2]:7.3f}) dist={np.linalg.norm(pt):.3f}")
                        
            return True
            
        except Exception as e:
            print(f"Error getting RPC data: {e}")
            return False
            
    def get_ros2_data(self):
        """Get LiDAR data from ROS2 topic"""
        topic_name = f"/{self.vehicle_name}/{self.sensor_name}/points"
        
        print(f"\n{'='*60}")
        print("ROS2 DATA COLLECTION")
        print(f"{'='*60}")
        print(f"Listening to topic: {topic_name}")
        print("Waiting for message (timeout 5 seconds)...")
        
        try:
            # Use ros2 topic echo to get one message
            cmd = f"ros2 topic echo {topic_name} sensor_msgs/msg/PointCloud2 --once"
            
            # Run with timeout
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            
            if result.returncode != 0:
                print(f"Error: {result.stderr}")
                print("\nMake sure:")
                print(f"  1. ROS2 nodes are running (check with: ros2 node list)")
                print(f"  2. Topic exists (check with: ros2 topic list | grep {self.sensor_name})")
                return False
                
            # Parse the YAML output
            output = result.stdout
            
            # Extract key information
            lines = output.split('\n')
            
            # Parse header
            header_info = {}
            data_info = {}
            fields_info = []
            data_values = []
            
            parsing_mode = None
            field_index = 0
            
            for line in lines:
                line = line.strip()
                
                if 'stamp:' in line and parsing_mode is None:
                    parsing_mode = 'header'
                elif 'height:' in line:
                    data_info['height'] = int(line.split(':')[1].strip())
                elif 'width:' in line:
                    data_info['width'] = int(line.split(':')[1].strip())
                elif 'fields:' in line:
                    parsing_mode = 'fields'
                elif 'point_step:' in line:
                    data_info['point_step'] = int(line.split(':')[1].strip())
                elif 'row_step:' in line:
                    data_info['row_step'] = int(line.split(':')[1].strip())
                elif 'data:' in line and '- ' not in line:
                    parsing_mode = 'data'
                    # Extract data array start
                    data_str = line.split('[')[1] if '[' in line else ''
                elif parsing_mode == 'fields' and '- name:' in line:
                    field_name = line.split(':')[1].strip().strip("'\"")
                    fields_info.append({'name': field_name})
                elif parsing_mode == 'data' and line:
                    # Parse data values
                    if line.endswith(']'):
                        line = line[:-1]
                    values = [int(v.strip()) for v in line.split(',') if v.strip()]
                    data_values.extend(values)
                    
            print(f"\nROS2 Message Info:")
            print(f"  Width (num points): {data_info.get('width', 'N/A')}")
            print(f"  Height: {data_info.get('height', 'N/A')}")
            print(f"  Point step: {data_info.get('point_step', 'N/A')} bytes")
            print(f"  Row step: {data_info.get('row_step', 'N/A')} bytes")
            print(f"  Fields: {[f['name'] for f in fields_info]}")
            print(f"  Data array size: {len(data_values)} bytes")
            
            # Convert byte array to points
            if data_values and data_info.get('point_step'):
                points = self.parse_pointcloud2_data(
                    bytes(data_values), 
                    data_info.get('width', 0),
                    data_info.get('point_step', 12)
                )
                
                self.ros2_data = {
                    'points': points,
                    'num_points': len(points),
                    'width': data_info.get('width', 0),
                    'point_step': data_info.get('point_step', 0)
                }
                
                if len(points) > 0:
                    # Analyze ROS2 points
                    origin_mask = np.all(np.abs(points) < 0.001, axis=1)
                    valid_points = points[~origin_mask]
                    
                    print(f"\nPoint Analysis:")
                    print(f"  Points at origin: {np.sum(origin_mask)}")
                    print(f"  Valid points: {len(valid_points)}")
                    
                    if len(valid_points) > 0:
                        print(f"  X range: [{np.min(valid_points[:, 0]):.3f}, {np.max(valid_points[:, 0]):.3f}]")
                        print(f"  Y range: [{np.min(valid_points[:, 1]):.3f}, {np.max(valid_points[:, 1]):.3f}]")
                        print(f"  Z range: [{np.min(valid_points[:, 2]):.3f}, {np.max(valid_points[:, 2]):.3f}]")
                        
                        print(f"\n  First 5 valid points (ROS2 - After transformation):")
                        for i in range(min(5, len(valid_points))):
                            pt = valid_points[i]
                            print(f"    [{i}]: ({pt[0]:7.3f}, {pt[1]:7.3f}, {pt[2]:7.3f}) dist={np.linalg.norm(pt):.3f}")
                
                return True
                
        except subprocess.TimeoutExpired:
            print("Timeout waiting for ROS2 message")
            print("\nTroubleshooting:")
            print(f"  1. Check if topic exists: ros2 topic list | grep {self.sensor_name}")
            print(f"  2. Check topic frequency: ros2 topic hz {topic_name}")
            print(f"  3. Check if nodes are running: ros2 node list")
            return False
        except Exception as e:
            print(f"Error getting ROS2 data: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def parse_pointcloud2_data(self, data, width, point_step):
        """Parse PointCloud2 binary data"""
        points = []
        
        for i in range(width):
            offset = i * point_step
            if offset + 12 <= len(data):
                # Extract x, y, z as floats (4 bytes each)
                x = struct.unpack('f', data[offset:offset+4])[0]
                y = struct.unpack('f', data[offset+4:offset+8])[0]
                z = struct.unpack('f', data[offset+8:offset+12])[0]
                points.append([x, y, z])
                
        return np.array(points, dtype=np.float32)
        
    def compare_data(self):
        """Compare RPC and ROS2 data"""
        print(f"\n{'='*60}")
        print("DATA COMPARISON")
        print(f"{'='*60}")
        
        if self.rpc_data is None or self.ros2_data is None:
            print("ERROR: Missing data for comparison")
            if self.rpc_data is None:
                print("  - RPC data not collected")
            if self.ros2_data is None:
                print("  - ROS2 data not collected")
            return
            
        # Compare point counts
        print(f"\nPoint Count Comparison:")
        print(f"  RPC points:  {self.rpc_data['num_points']}")
        print(f"  ROS2 points: {self.ros2_data['num_points']}")
        
        if self.rpc_data['num_points'] != self.ros2_data['num_points']:
            print(f"  ⚠️  WARNING: Point count mismatch!")
        else:
            print(f"  ✅ Point counts match")
            
        # Get valid points (non-origin)
        rpc_points = self.rpc_data['points']
        ros2_points = self.ros2_data['points']
        
        if len(rpc_points) > 0 and len(ros2_points) > 0:
            # Filter out origin points
            rpc_valid = rpc_points[~np.all(np.abs(rpc_points) < 0.001, axis=1)]
            ros2_valid = ros2_points[~np.all(np.abs(ros2_points) < 0.001, axis=1)]
            
            print(f"\nValid Points (non-origin):")
            print(f"  RPC:  {len(rpc_valid)}")
            print(f"  ROS2: {len(ros2_valid)}")
            
            if len(rpc_valid) > 0 and len(ros2_valid) > 0:
                # Check coordinate transformation
                # ROS2 applies transformation: fixPointCloud with axis flip on Y
                print(f"\nCoordinate System Check:")
                print(f"  AirSim uses NED (North-East-Down)")
                print(f"  ROS2 should transform to ROS convention")
                print(f"  Expected transformation: Y' = -Y (flip Y axis)")
                
                # Compare first few points
                num_compare = min(5, len(rpc_valid), len(ros2_valid))
                print(f"\nDetailed Point Comparison (first {num_compare} valid points):")
                print(f"{'Index':<7} {'RPC (NED)':<30} {'ROS2':<30} {'Match?':<10}")
                print("-" * 80)
                
                for i in range(num_compare):
                    rpc_pt = rpc_valid[i]
                    ros2_pt = ros2_valid[i]
                    
                    # Check if ROS2 applied Y-axis flip
                    expected_ros2 = np.array([rpc_pt[0], -rpc_pt[1], rpc_pt[2]])
                    
                    # Check if points match (with small tolerance)
                    match = np.allclose(ros2_pt, expected_ros2, atol=0.01)
                    
                    print(f"{i:<7} ({rpc_pt[0]:7.3f},{rpc_pt[1]:7.3f},{rpc_pt[2]:7.3f}) "
                          f"({ros2_pt[0]:7.3f},{ros2_pt[1]:7.3f},{ros2_pt[2]:7.3f}) "
                          f"{'✅' if match else '❌'}")
                    
                    if not match:
                        print(f"        Expected ROS2: ({expected_ros2[0]:7.3f},{expected_ros2[1]:7.3f},{expected_ros2[2]:7.3f})")
                        
                # Statistical comparison
                print(f"\nStatistical Comparison:")
                
                # Check if transformation is consistent
                if len(rpc_valid) == len(ros2_valid):
                    # Apply expected transformation to RPC data
                    rpc_transformed = rpc_valid.copy()
                    rpc_transformed[:, 1] = -rpc_transformed[:, 1]  # Flip Y
                    
                    # Calculate differences
                    diffs = ros2_valid - rpc_transformed
                    
                    print(f"  Mean difference after transformation:")
                    print(f"    X: {np.mean(diffs[:, 0]):.6f}")
                    print(f"    Y: {np.mean(diffs[:, 1]):.6f}")
                    print(f"    Z: {np.mean(diffs[:, 2]):.6f}")
                    
                    print(f"  Max absolute difference:")
                    print(f"    X: {np.max(np.abs(diffs[:, 0])):.6f}")
                    print(f"    Y: {np.max(np.abs(diffs[:, 1])):.6f}")
                    print(f"    Z: {np.max(np.abs(diffs[:, 2])):.6f}")
                    
                    # Check if transformation is correct
                    if np.allclose(ros2_valid, rpc_transformed, atol=0.01):
                        print(f"\n✅ SUCCESS: ROS2 data matches RPC data with correct transformation!")
                    else:
                        print(f"\n⚠️  WARNING: Data mismatch detected. Check transformation implementation.")
                        
        print(f"\n{'='*60}")
        
    def run_verification(self):
        """Run the complete verification process"""
        print(f"\n{'#'*60}")
        print(f"LiDAR DATA VERIFICATION")
        print(f"Vehicle: {self.vehicle_name}")
        print(f"Sensor: {self.sensor_name}")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"{'#'*60}")
        
        # Connect to AirSim
        self.connect_airsim()
        
        # Take off for better sensor readings
        print("\nTaking off to get clear sensor readings...")
        self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        time.sleep(2)
        
        # Move to altitude
        print("Moving to altitude -5m...")
        self.client.moveToZAsync(-5, 2, vehicle_name=self.vehicle_name).join()
        time.sleep(2)
        
        # Collect RPC data
        if not self.get_rpc_data():
            print("Failed to get RPC data")
            return
            
        # Small delay
        time.sleep(1)
        
        # Collect ROS2 data
        if not self.get_ros2_data():
            print("Failed to get ROS2 data")
            print("\nMake sure ROS2 nodes are running:")
            print("  ros2 launch airsim_ros_pkgs airsim_node.launch.py")
            print("    OR")
            print("  ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py")
            
        # Compare data
        self.compare_data()
        
        # Land
        print("\nLanding...")
        self.client.landAsync(vehicle_name=self.vehicle_name).join()
        
        # Cleanup
        self.client.armDisarm(False, self.vehicle_name)
        self.client.enableApiControl(False, self.vehicle_name)
        
        print("\n✅ Verification complete!")
        
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Verify ROS2 LiDAR data against Python RPC')
    parser.add_argument('--vehicle', default='Drone_1', help='Vehicle name (default: Drone_1)')
    parser.add_argument('--sensor', default='LidarSensor1', help='Sensor name (default: LidarSensor1)')
    args = parser.parse_args()
    
    verifier = LiDARVerifier(args.vehicle, args.sensor)
    verifier.run_verification()
    
if __name__ == "__main__":
    main()