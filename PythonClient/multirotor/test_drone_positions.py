#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import setup_path
from datetime import datetime

# Add the parent directory to the path to import cosysairsim
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def test_single_drone_position_methods(client, drone_name):
    """Test all available methods to get position for a single drone."""
    print(f"\n{'='*60}")
    print(f"Testing Position Methods for: {drone_name}")
    print(f"{'='*60}")
    
    results = {}
    
    # Method 1: getMultirotorState - kinematics_estimated
    print("\nMethod 1: getMultirotorState().kinematics_estimated.position")
    try:
        state = client.getMultirotorState(vehicle_name=drone_name)
        pos = state.kinematics_estimated.position
        orientation = state.kinematics_estimated.orientation
        results['kinematics_estimated'] = {
            'position': (pos.x_val, pos.y_val, pos.z_val),
            'orientation': (orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val),
            'success': True,
            'armed': state.armed,
            'landed_state': str(state.landed_state)
        }
        print(f"  SUCCESS: Position: ({pos.x_val:.3f}, {pos.y_val:.3f}, {pos.z_val:.3f})")
        print(f"  Orientation (WXYZ): ({orientation.w_val:.3f}, {orientation.x_val:.3f}, {orientation.y_val:.3f}, {orientation.z_val:.3f})")
        print(f"  Armed: {state.armed}, Landed: {state.landed_state}")
    except Exception as e:
        results['kinematics_estimated'] = {'success': False, 'error': str(e)}
        print(f"  FAILED: {str(e)}")
    
    # Method 2: simGetVehiclePose
    print("\nMethod 2: simGetVehiclePose()")
    try:
        pose = client.simGetVehiclePose(vehicle_name=drone_name)
        pos = pose.position
        orientation = pose.orientation
        results['vehicle_pose'] = {
            'position': (pos.x_val, pos.y_val, pos.z_val),
            'orientation': (orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val),
            'success': True
        }
        print(f"  SUCCESS: Position: ({pos.x_val:.3f}, {pos.y_val:.3f}, {pos.z_val:.3f})")
        print(f"  Orientation (WXYZ): ({orientation.w_val:.3f}, {orientation.x_val:.3f}, {orientation.y_val:.3f}, {orientation.z_val:.3f})")
    except Exception as e:
        results['vehicle_pose'] = {'success': False, 'error': str(e)}
        print(f"  FAILED: {str(e)}")
    
    # Method 3: simGetGroundTruthKinematics
    print("\nMethod 3: simGetGroundTruthKinematics()")
    try:
        kinematics = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
        pos = kinematics.position
        orientation = kinematics.orientation
        velocity = kinematics.linear_velocity
        angular_vel = kinematics.angular_velocity
        results['ground_truth'] = {
            'position': (pos.x_val, pos.y_val, pos.z_val),
            'orientation': (orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val),
            'linear_velocity': (velocity.x_val, velocity.y_val, velocity.z_val),
            'angular_velocity': (angular_vel.x_val, angular_vel.y_val, angular_vel.z_val),
            'success': True
        }
        print(f"  SUCCESS: Position: ({pos.x_val:.3f}, {pos.y_val:.3f}, {pos.z_val:.3f})")
        print(f"  Orientation (WXYZ): ({orientation.w_val:.3f}, {orientation.x_val:.3f}, {orientation.y_val:.3f}, {orientation.z_val:.3f})")
        print(f"  Linear Velocity: ({velocity.x_val:.3f}, {velocity.y_val:.3f}, {velocity.z_val:.3f})")
        print(f"  Angular Velocity: ({angular_vel.x_val:.3f}, {angular_vel.y_val:.3f}, {angular_vel.z_val:.3f})")
    except Exception as e:
        results['ground_truth'] = {'success': False, 'error': str(e)}
        print(f"  FAILED: {str(e)}")
    
    # Method 4: GPS Data
    print("\nMethod 4: getGpsData()")
    try:
        gps = client.getGpsData(vehicle_name=drone_name)
        results['gps'] = {
            'latitude': gps.gnss.geo_point.latitude,
            'longitude': gps.gnss.geo_point.longitude,
            'altitude': gps.gnss.geo_point.altitude,
            'time_stamp': gps.time_stamp,
            'success': True
        }
        print(f"  SUCCESS: GPS Lat/Lon/Alt: ({gps.gnss.geo_point.latitude:.6f}, {gps.gnss.geo_point.longitude:.6f}, {gps.gnss.geo_point.altitude:.3f})")
        print(f"  Timestamp: {gps.time_stamp}")
    except Exception as e:
        results['gps'] = {'success': False, 'error': str(e)}
        print(f"  FAILED: {str(e)}")
    
    return results

def compare_position_methods(results):
    """Compare position results from different methods."""
    print(f"\n{'='*60}")
    print("POSITION COMPARISON ANALYSIS")
    print(f"{'='*60}")
    
    # Extract positions from successful methods
    positions = {}
    for method, data in results.items():
        if data.get('success') and 'position' in data:
            positions[method] = data['position']
    
    if len(positions) < 2:
        print("WARNING: Not enough position methods succeeded for comparison")
        return
    
    print("\nPosition Comparison:")
    for method, pos in positions.items():
        print(f"  {method:20}: ({pos[0]:8.3f}, {pos[1]:8.3f}, {pos[2]:8.3f})")
    
    # Calculate differences between methods
    print("\nPosition Differences:")
    methods = list(positions.keys())
    for i in range(len(methods)):
        for j in range(i+1, len(methods)):
            method1, method2 = methods[i], methods[j]
            pos1, pos2 = positions[method1], positions[method2]
            
            diff_x = abs(pos1[0] - pos2[0])
            diff_y = abs(pos1[1] - pos2[1])
            diff_z = abs(pos1[2] - pos2[2])
            distance = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)**0.5
            
            print(f"  {method1} vs {method2}:")
            print(f"    Delta X: {diff_x:6.3f}m, Delta Y: {diff_y:6.3f}m, Delta Z: {diff_z:6.3f}m")
            print(f"    3D Distance: {distance:6.3f}m")
            
            if distance < 0.001:
                print(f"    RESULT: IDENTICAL positions")
            elif distance < 0.1:
                print(f"    RESULT: Very close positions (< 10cm)")
            elif distance < 1.0:
                print(f"    RESULT: Close positions (< 1m)")
            else:
                print(f"    RESULT: Significant difference (> 1m)")

def main():
    print("COMPREHENSIVE DRONE POSITION TESTING SUITE")
    print("=" * 60)
    print("This script will rigorously test all available methods")
    print("for getting drone positions in AirSim multi-agent setup.")
    print("=" * 60)
    
    # Define drone names to test
    drone_names = ["Drone1", "Drone2", "Drone3", "Drone4"]
    
    # Connect to AirSim
    print("\nConnecting to AirSim...")
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("SUCCESS: Connected to AirSim")
        print(f"Client Version: {client.getClientVersion()}")
        print(f"Server Version: {client.getServerVersion()}")
    except Exception as e:
        print(f"FAILED: Failed to connect to AirSim: {str(e)}")
        return
    
    # Check which drones are available
    print("\nChecking available drones...")
    available_drones = []
    for drone in drone_names:
        try:
            client.getMultirotorState(vehicle_name=drone)
            available_drones.append(drone)
            print(f"  SUCCESS: {drone} is available")
        except Exception as e:
            print(f"  FAILED: {drone} not available: {str(e)}")
    
    if not available_drones:
        print("ERROR: No drones available for testing!")
        return
    
    print(f"\nTesting {len(available_drones)} available drones: {available_drones}")
    
    # Test each drone individually
    all_results = {}
    for drone in available_drones:
        results = test_single_drone_position_methods(client, drone)
        all_results[drone] = results
        compare_position_methods(results)
        
        # Add a small delay between drones
        time.sleep(1)
    
    # Cross-drone comparison
    print(f"\n{'='*60}")
    print("CROSS-DRONE POSITION ANALYSIS")
    print(f"{'='*60}")
    
    # Compare positions across all drones
    print("\nAll Drone Positions (using simGetVehiclePose):")
    drone_positions = {}
    for drone in available_drones:
        if all_results[drone].get('vehicle_pose', {}).get('success'):
            pos = all_results[drone]['vehicle_pose']['position']
            drone_positions[drone] = pos
            print(f"  {drone:10}: ({pos[0]:8.3f}, {pos[1]:8.3f}, {pos[2]:8.3f})")
    
    # Check if all drones are at origin
    origin_drones = []
    positioned_drones = []
    for drone, pos in drone_positions.items():
        if abs(pos[0]) < 0.5 and abs(pos[1]) < 0.5:
            origin_drones.append(drone)
        else:
            positioned_drones.append(drone)
    
    print(f"\nPosition Analysis Summary:")
    print(f"  Drones at origin (0,0): {len(origin_drones)} - {origin_drones}")
    print(f"  Drones positioned: {len(positioned_drones)} - {positioned_drones}")
    
    if len(origin_drones) == len(available_drones):
        print("  CONCLUSION: ALL DRONES AT ORIGIN - Settings.json may not be loaded or physics not synced")
    elif len(positioned_drones) == len(available_drones):
        print("  CONCLUSION: ALL DRONES PROPERLY POSITIONED")
    else:
        print("  CONCLUSION: MIXED POSITIONING - Some drones positioned, others at origin")
    
    # Final summary
    print(f"\n{'='*60}")
    print("FINAL TEST SUMMARY")
    print(f"{'='*60}")
    
    for drone in available_drones:
        print(f"\n{drone} Results:")
        results = all_results[drone]
        
        successful_methods = [method for method, data in results.items() if data.get('success')]
        failed_methods = [method for method, data in results.items() if not data.get('success')]
        
        print(f"  Successful methods ({len(successful_methods)}): {', '.join(successful_methods)}")
        if failed_methods:
            print(f"  Failed methods ({len(failed_methods)}): {', '.join(failed_methods)}")
    
    print(f"\nPosition testing completed!")
    print(f"Tested {len(available_drones)} drones with {len(all_results[available_drones[0]]) if available_drones else 0} different methods each")

if __name__ == "__main__":
    main() 