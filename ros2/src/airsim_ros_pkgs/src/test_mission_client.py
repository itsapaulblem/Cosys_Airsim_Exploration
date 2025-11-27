#!/usr/bin/env python3
"""
Simple test client for MissionMultirotorNode action servers
Validates that mission interfaces work correctly with ultra-clean naming
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Polygon, Point32
from mission_search_interfaces.action import SearchArea
import time

class MissionTestClient(Node):
    def __init__(self):
        super().__init__('mission_test_client')
        
        # Test different vehicles with ultra-clean naming
        self.test_vehicles = ['Droan1', 'PX4_Drone2', 'SimpleFlight3']
        self.results = {}
        
        self.get_logger().info("Mission Test Client initialized - Testing ultra-clean naming architecture")
    
    def test_search_area_action(self, vehicle_name):
        """Test SearchArea action server availability and goal acceptance"""
        
        # Create action client with ultra-clean naming: /VehicleName/actions/search_area
        action_name = f'/{vehicle_name}/actions/search_area'
        action_client = ActionClient(self, SearchArea, action_name)
        
        self.get_logger().info(f"Testing action server: {action_name}")
        
        # Wait for action server with timeout
        if not action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"Action server {action_name} not available")
            return False
        
        # Create test goal - small search area
        goal_msg = SearchArea.Goal()
        goal_msg.search_boundary.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=10.0, y=0.0, z=0.0),
            Point32(x=10.0, y=10.0, z=0.0),
            Point32(x=0.0, y=10.0, z=0.0)
        ]
        goal_msg.search_pattern = "spiral"
        goal_msg.search_altitude = 20.0
        goal_msg.search_speed = 3.0
        goal_msg.pattern_spacing = 5.0
        goal_msg.enable_detection = True
        goal_msg.detection_confidence_threshold = 0.5
        
        self.get_logger().info(f"Sending test goal to {vehicle_name}")
        
        # Send goal (we'll cancel it immediately for testing)
        future = action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info(f"✅ Goal accepted by {vehicle_name}")
                
                # Immediately cancel for testing
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                
                if cancel_future.result() is not None:
                    self.get_logger().info(f"✅ Goal cancellation successful for {vehicle_name}")
                    return True
                else:
                    self.get_logger().warn(f"⚠️  Goal cancellation failed for {vehicle_name}")
                    return False
            else:
                self.get_logger().warn(f"❌ Goal rejected by {vehicle_name}")
                return False
        else:
            self.get_logger().warn(f"❌ No response from {vehicle_name} action server")
            return False
    
    def test_all_vehicles(self):
        """Test action servers for all configured vehicles"""
        
        self.get_logger().info("=== Starting Mission Interface Validation Tests ===")
        
        for vehicle_name in self.test_vehicles:
            success = self.test_search_area_action(vehicle_name)
            self.results[vehicle_name] = success
            time.sleep(1)  # Small delay between tests
        
        # Print summary
        self.get_logger().info("=== Test Results Summary ===")
        successful = 0
        for vehicle, result in self.results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            self.get_logger().info(f"{vehicle}: {status}")
            if result:
                successful += 1
        
        self.get_logger().info(f"Tests completed: {successful}/{len(self.test_vehicles)} passed")
        return successful == len(self.test_vehicles)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_client = MissionTestClient()
        
        # Run tests
        all_passed = test_client.test_all_vehicles()
        
        if all_passed:
            test_client.get_logger().info("🎉 All mission interface tests PASSED!")
        else:
            test_client.get_logger().error("❌ Some mission interface tests FAILED!")
            
    except KeyboardInterrupt:
        test_client.get_logger().info("Test interrupted by user")
    finally:
        test_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()