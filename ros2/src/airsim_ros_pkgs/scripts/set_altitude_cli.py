#!/usr/bin/env python3

"""
AirSim SetAltitude Command Line Tool

Usage:
    python3 set_altitude_cli.py -z -10.0 -v 5.0 --drone Drone1 --wait
    
This tool provides a simple command-line interface to the SetAltitude service.
"""

import rclpy
from rclpy.node import Node
from airsim_interfaces.srv import SetAltitude
import argparse
import sys

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Set drone altitude using AirSim ROS2 service')
    parser.add_argument('-z', '--altitude', type=float, required=True,
                       help='Target altitude in meters (NED coordinates: negative is up, e.g., -10 = 10m high)')
    parser.add_argument('-v', '--velocity', type=float, default=5.0,
                       help='Movement velocity in m/s (default: 5.0)')
    parser.add_argument('--drone', type=str, default='Drone1',
                       help='Drone name (default: Drone1)')
    parser.add_argument('--wait', action='store_true',
                       help='Wait for the altitude change to complete')
    parser.add_argument('--no-wait', action='store_true',
                       help='Send command asynchronously (don\'t wait for completion)')
    
    args = parser.parse_args()
    
    # Determine wait behavior
    if args.wait and args.no_wait:
        print("Error: Cannot specify both --wait and --no-wait")
        sys.exit(1)
    
    wait_on_task = args.wait or not args.no_wait  # Default to waiting
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node and service client
    node = Node('altitude_cli_client')
    client = node.create_client(SetAltitude, f'/airsim_node/{args.drone}/set_altitude')
    
    # Wait for service
    print(f"Waiting for SetAltitude service for {args.drone}...")
    if not client.wait_for_service(timeout_sec=10.0):
        print(f"Error: SetAltitude service for {args.drone} not available after 10 seconds")
        sys.exit(1)
    
    # Create and send request
    request = SetAltitude.Request()
    request.z = args.altitude
    request.velocity = args.velocity
    request.vehicle_name = args.drone
    request.wait_on_last_task = wait_on_task
    
    print(f"Setting altitude for {args.drone}:")
    print(f"  Target Z: {args.altitude}m (NED coordinates)")
    print(f"  Velocity: {args.velocity}m/s")
    print(f"  Wait for completion: {wait_on_task}")
    
    try:
        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        response = future.result()
        
        if response.success:
            print(f"✅ Success: {response.message}")
            sys.exit(0)
        else:
            print(f"❌ Failed: {response.message}")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ Service call failed: {str(e)}")
        sys.exit(1)
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 