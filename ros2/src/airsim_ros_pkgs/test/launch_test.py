#!/usr/bin/env python3

import os
import sys
import time
import subprocess
import signal
import rclpy
from rclpy.node import Node

class LaunchTest(Node):
    def __init__(self):
        super().__init__('launch_test')
        self.processes = []
        
    def run_test(self):
        """Test the complete launch system"""
        try:
            # Test 1: Launch multi-vehicle system
            self.get_logger().info("Starting multi-vehicle launch test...")
            
            launch_cmd = [
                'ros2', 'launch', 'airsim_ros_pkgs', 'multi_vehicle.launch.py',
                'max_vehicles:=2'
            ]
            
            process = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(process)
            
            # Wait for system to start
            time.sleep(10)
            
            # Test 2: Check if nodes are running
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            nodes = result.stdout.strip().split('\n')
            
            expected_nodes = [
                '/airsim_coordination',
                '/drone1/airsim_drone1',
                '/drone2/airsim_drone2'
            ]
            
            for expected in expected_nodes:
                if expected in nodes:
                    self.get_logger().info(f"✅ Node {expected} running")
                else:
                    self.get_logger().error(f"❌ Node {expected} not found")
                    return False
            
            # Test 3: Check services
            result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True)
            services = result.stdout.strip().split('\n')
            
            expected_services = ['/reset_all', '/takeoff_all', '/land_all']
            for service in expected_services:
                if service in services:
                    self.get_logger().info(f"✅ Service {service} available")
                else:
                    self.get_logger().error(f"❌ Service {service} not found")
                    return False
            
            # Test 4: Check topics
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            topics = result.stdout.strip().split('\n')
            
            expected_topics = ['/drone1/odom_local_ned', '/drone2/odom_local_ned', '/system_status']
            for topic in expected_topics:
                if topic in topics:
                    self.get_logger().info(f"✅ Topic {topic} available")
                else:
                    self.get_logger().error(f"❌ Topic {topic} not found")
                    return False
            
            self.get_logger().info("🎉 All launch tests passed!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Test failed: {e}")
            return False
        finally:
            # Cleanup
            for process in self.processes:
                process.terminate()
                process.wait()

def main():
    rclpy.init()
    test = LaunchTest()
    
    try:
        success = test.run_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        test.get_logger().info("Test interrupted")
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()