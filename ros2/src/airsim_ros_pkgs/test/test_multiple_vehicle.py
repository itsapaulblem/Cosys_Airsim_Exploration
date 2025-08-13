#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import subprocess 
import sys
from std_srvs.srv import SetBool
from airsim_interfaces.srv import Reset, Takeoff, Land

class MultiVehicleTestNode(Node):
    def __init__(self):
        super().__init__('multi_vehicle_test')

        self.reset_client = self.create_client(Reset, 'reset_all')
        self.takeoff_client = self.create_client(Takeoff, 'takeoff_all')
        self.land_client = self.create_client(SetBool, 'pause_simulation')

        self.get_logger().info("Multi-vehicle test node initialized")
    
    def wait_for_services(self):
        """Wait for all coordination services to be available"""
        services = [
            (self.reset_client, 'reset_all'),
            (self.takeoff_client, 'takeoff_all'), 
            (self.land_client, 'land_all'),
            (self.pause_client, 'pause_simulation')
        ]
        
        for client, name in services:
            self.get_logger().info(f"Waiting for service {name}...")
            if not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f"Service {name} not available!")
                return False
        
        self.get_logger().info("All services available")
        return True
    
    def test_reset(self):
        """Test system reset"""
        self.get_logger().info("Testing system reset...")
        req = Reset.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Reset test passed")
            return True
        else:
            self.get_logger().error("❌ Reset test failed")
            return False
    
    def test_takeoff(self):
        """Test coordinated takeoff"""
        self.get_logger().info("Testing coordinated takeoff...")
        req = Takeoff.Request()
        req.wait_on_last_task = True
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Takeoff test passed")
            return True
        else:
            self.get_logger().error("❌ Takeoff test failed")
            return False
    
    def test_landing(self):
        """Test coordinated landing"""
        self.get_logger().info("Testing coordinated landing...")
        req = Land.Request()
        req.wait_on_last_task = True
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Landing test passed")
            return True
        else:
            self.get_logger().error("❌ Landing test failed")
            return False
    
    def test_pause_resume(self):
        """Test simulation pause/resume"""
        self.get_logger().info("Testing pause/resume...")
        
        # Pause
        req = SetBool.Request()
        req.data = True
        future = self.pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not (future.result() and future.result().success):
            self.get_logger().error("❌ Pause test failed")
            return False
        
        time.sleep(2)
        
        # Resume
        req.data = False
        future = self.pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Pause/resume test passed")
            return True
        else:
            self.get_logger().error("❌ Resume test failed")
            return False
    
    def run_tests(self):
        """Run all tests"""
        if not self.wait_for_services():
            return False
        
        tests = [
            self.test_reset,
            self.test_pause_resume,
            self.test_takeoff,
            self.test_landing,
        ]
        
        passed = 0
        for test in tests:
            if test():
                passed += 1
            time.sleep(2)  # Wait between tests
        
        self.get_logger().info(f"Tests completed: {passed}/{len(tests)} passed")
        return passed == len(tests)

def main():
    rclpy.init()
    test_node = MultiVehicleTestNode()
    
    try:
        success = test_node.run_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()