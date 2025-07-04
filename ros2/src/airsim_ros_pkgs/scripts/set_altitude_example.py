#!/usr/bin/env python3

"""
AirSim SetAltitude Service Example

This script demonstrates how to use the new SetAltitude service to control drone altitude.
"""

import rclpy
from rclpy.node import Node
from airsim_interfaces.srv import SetAltitude
import time

class AltitudeServiceClient(Node):
    def __init__(self):
        super().__init__('altitude_service_client')
        
        # Create service client
        self.client = self.create_client(SetAltitude, '/airsim_node/Drone1/set_altitude')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetAltitude service not available, waiting...')
        
        self.get_logger().info('SetAltitude service is available!')

    def set_altitude(self, z_position, velocity=5.0, vehicle_name='Drone1', wait_on_task=True):
        """
        Set the altitude of a drone using the SetAltitude service.
        
        Args:
            z_position (float): Target Z position (meters, NED coordinates - negative is up)
            velocity (float): Movement velocity (m/s)
            vehicle_name (str): Name of the vehicle
            wait_on_task (bool): Whether to wait for the task completion
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Create service request
        request = SetAltitude.Request()
        request.z = float(z_position)
        request.velocity = float(velocity)
        request.vehicle_name = vehicle_name
        request.wait_on_last_task = wait_on_task
        
        self.get_logger().info(f'Requesting altitude change: z={z_position}m, velocity={velocity}m/s, vehicle={vehicle_name}')
        
        try:
            # Call service
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed: {response.message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False

def main():
    rclpy.init()
    
    altitude_client = AltitudeServiceClient()
    
    try:
        # Example usage: different altitude commands
        
        # 1. Move to 10 meters altitude (z = -10 in NED coordinates)
        print("\n=== Example 1: Moving to 10m altitude ===")
        success = altitude_client.set_altitude(-10.0, velocity=3.0, wait_on_task=True)
        if success:
            print("✅ Successfully moved to 10m altitude")
            time.sleep(2)  # Wait for movement to complete
        
        # 2. Move to 20 meters altitude with faster velocity
        print("\n=== Example 2: Moving to 20m altitude (faster) ===")
        success = altitude_client.set_altitude(-20.0, velocity=8.0, wait_on_task=True)
        if success:
            print("✅ Successfully moved to 20m altitude")
            time.sleep(2)
        
        # 3. Return to lower altitude
        print("\n=== Example 3: Returning to 5m altitude ===")
        success = altitude_client.set_altitude(-5.0, velocity=5.0, wait_on_task=True)
        if success:
            print("✅ Successfully returned to 5m altitude")
        
        # 4. Example of async call (don't wait for completion)
        print("\n=== Example 4: Async altitude change ===")
        success = altitude_client.set_altitude(-15.0, velocity=4.0, wait_on_task=False)
        if success:
            print("✅ Async altitude command sent successfully")
    
    except KeyboardInterrupt:
        print("\nShutting down due to keyboard interrupt")
    
    finally:
        altitude_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 