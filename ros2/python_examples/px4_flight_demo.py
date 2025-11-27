#!/usr/bin/env python3
"""
PX4 AirSim Flight Demonstration

This script demonstrates the correct sequence for PX4 velocity commands:
1. Takeoff (REQUIRED)
2. Continuous velocity commands
3. Land

This addresses the specific issues found with PX4 velocity commands:
- 0.05 second command duration (solved with continuous commands)
- Need for takeoff before velocity commands work
- PX4 timeout requirements

Usage:
    python3 px4_flight_demo.py --vehicle PX4_Drone1
    python3 px4_flight_demo.py --vehicle PX4_Drone1 --pattern square
    python3 px4_flight_demo.py --vehicle PX4_Drone1 --pattern circle

Author: Cosys-AirSim ROS2 Integration
"""

import argparse
import rclpy
from rclpy.node import Node
import time
import math
from velocity_command_helper import VelocityCommandPublisher


class PX4FlightDemo(Node):
    """
    Demonstration of proper PX4 flight patterns using velocity commands.
    """
    
    def __init__(self, vehicle_name: str = "PX4_Drone1"):
        super().__init__('px4_flight_demo')
        self.vehicle_name = vehicle_name
        
        # Create velocity command publisher with takeoff/land capability
        self.cmd_publisher = VelocityCommandPublisher(vehicle_name, use_body_frame=True)
        
        self.get_logger().info(f'PX4 Flight Demo initialized for {vehicle_name}')
    
    def safe_flight_sequence(self, pattern: str = "simple") -> bool:
        """
        Execute a complete safe flight sequence.
        
        Args:
            pattern: Flight pattern to execute ('simple', 'square', 'circle')
            
        Returns:
            True if sequence completed successfully
        """
        try:
            # Step 1: Takeoff (REQUIRED for PX4)
            self.get_logger().info("🚀 Starting flight sequence...")
            self.get_logger().info("📍 Step 1: Taking off...")
            
            if not self.cmd_publisher.takeoff():
                self.get_logger().error("❌ Takeoff failed - aborting sequence")
                return False
            
            self.get_logger().info("✅ Takeoff completed - drone is airborne")
            
            # Step 2: Execute flight pattern
            self.get_logger().info(f"📍 Step 2: Executing {pattern} pattern...")
            
            if pattern == "simple":
                success = self.simple_pattern()
            elif pattern == "square":
                success = self.square_pattern()
            elif pattern == "circle":
                success = self.circle_pattern()
            else:
                self.get_logger().error(f"Unknown pattern: {pattern}")
                success = False
            
            if not success:
                self.get_logger().error("❌ Flight pattern failed")
            else:
                self.get_logger().info("✅ Flight pattern completed successfully")
            
            # Step 3: Hover before landing
            self.get_logger().info("📍 Step 3: Hovering before landing...")
            self.hover_for_duration(2.0)
            
            # Step 4: Land
            self.get_logger().info("📍 Step 4: Landing...")
            if not self.cmd_publisher.land():
                self.get_logger().error("❌ Landing failed")
                return False
            
            self.get_logger().info("✅ Flight sequence completed successfully!")
            return True
            
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Flight sequence interrupted by user")
            self.emergency_landing()
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Flight sequence failed: {e}")
            self.emergency_landing()
            return False
    
    def simple_pattern(self) -> bool:
        """Execute a simple forward-hover-back pattern."""
        try:
            # Move forward for 3 seconds
            self.get_logger().info("   → Moving forward...")
            self.send_velocity_for_duration(1.0, 0.0, 0.0, duration=3.0)
            
            # Hover for 2 seconds  
            self.get_logger().info("   → Hovering...")
            self.hover_for_duration(2.0)
            
            # Move backward for 3 seconds
            self.get_logger().info("   → Moving backward...")
            self.send_velocity_for_duration(-1.0, 0.0, 0.0, duration=3.0)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Simple pattern failed: {e}")
            return False
    
    def square_pattern(self) -> bool:
        """Execute a square flight pattern."""
        try:
            movements = [
                ("forward", 1.0, 0.0, 0.0),
                ("right", 0.0, 1.0, 0.0),
                ("backward", -1.0, 0.0, 0.0),
                ("left", 0.0, -1.0, 0.0)
            ]
            
            for direction, vx, vy, vz in movements:
                self.get_logger().info(f"   → Moving {direction}...")
                self.send_velocity_for_duration(vx, vy, vz, duration=3.0)
                
                # Brief hover between movements
                self.hover_for_duration(1.0)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Square pattern failed: {e}")
            return False
    
    def circle_pattern(self) -> bool:
        """Execute a circular flight pattern."""
        try:
            self.get_logger().info("   → Flying in circle...")
            
            # Circular motion: forward velocity + rotation
            # This creates a circle by moving forward while rotating
            duration = 10.0  # Total circle time
            rate = 10.0  # Command rate (Hz)
            total_steps = int(duration * rate)
            
            for step in range(total_steps):
                # Move forward while rotating (creates circular motion)
                self.cmd_publisher.send_velocity_command(
                    vx=0.8,  # Forward speed
                    vy=0.0,
                    vz=0.0,
                    yaw_rate=0.4  # Rotation rate
                )
                
                # Sleep for rate control
                time.sleep(1.0 / rate)
                
                # Spin ROS2 to keep everything alive
                rclpy.spin_once(self, timeout_sec=0.001)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Circle pattern failed: {e}")
            return False
    
    def send_velocity_for_duration(self, vx: float, vy: float, vz: float, 
                                 duration: float, rate: float = 10.0) -> None:
        """
        Send velocity commands for a specific duration.
        
        This solves the 0.05 second duration issue by sending continuous commands.
        """
        total_steps = int(duration * rate)
        
        for _ in range(total_steps):
            self.cmd_publisher.send_velocity_command(vx, vy, vz)
            time.sleep(1.0 / rate)
            rclpy.spin_once(self, timeout_sec=0.001)
    
    def hover_for_duration(self, duration: float, rate: float = 10.0) -> None:
        """Hover in place for specified duration."""
        self.send_velocity_for_duration(0.0, 0.0, 0.0, duration, rate)
    
    def emergency_landing(self) -> None:
        """Emergency landing procedure."""
        self.get_logger().warn("🚨 Executing emergency landing...")
        try:
            # Stop all movement first
            self.hover_for_duration(1.0)
            # Then land
            self.cmd_publisher.land()
        except Exception as e:
            self.get_logger().error(f"Emergency landing failed: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='PX4 AirSim flight demonstration'
    )
    
    parser.add_argument(
        '--vehicle', '-v',
        default='PX4_Drone1',
        help='Vehicle name (default: PX4_Drone1)'
    )
    
    parser.add_argument(
        '--pattern', '-p',
        choices=['simple', 'square', 'circle'],
        default='simple',
        help='Flight pattern to execute (default: simple)'
    )
    
    parser.add_argument(
        '--diagnostics',
        action='store_true',
        help='Run diagnostics before flight'
    )
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Run diagnostics if requested
        if args.diagnostics:
            print("🔧 Running pre-flight diagnostics...")
            import subprocess
            result = subprocess.run([
                'python3', 'airsim_diagnostics.py', 
                '--vehicle', args.vehicle, '--check-topics'
            ])
            if result.returncode != 0:
                print("❌ Diagnostics failed - check setup before flying")
                return
        
        # Create demo node
        demo = PX4FlightDemo(args.vehicle)
        
        # Display important information
        print("\n" + "="*60)
        print("🚁 PX4 AirSim Flight Demonstration")
        print("="*60)
        print(f"Vehicle: {args.vehicle}")
        print(f"Pattern: {args.pattern}")
        print("\n🔑 This demo addresses PX4-specific issues:")
        print("• ✅ Takeoff before velocity commands (REQUIRED)")
        print("• ✅ Continuous velocity commands (solves 0.05s duration)")
        print("• ✅ Proper PX4 timeout handling")
        print("• ✅ Safe landing sequence")
        print("\n🚨 Safety: Press Ctrl+C to emergency land")
        print("="*60)
        
        # Wait for user confirmation
        input("\nPress Enter to start flight sequence (or Ctrl+C to abort)...")
        
        # Execute flight sequence
        success = demo.safe_flight_sequence(args.pattern)
        
        if success:
            print("\n🎉 Flight demonstration completed successfully!")
        else:
            print("\n❌ Flight demonstration failed - check logs")
            
    except KeyboardInterrupt:
        print("\n🛑 Flight demonstration aborted by user")
    except Exception as e:
        print(f"\n💥 Flight demonstration error: {e}")
    finally:
        # Cleanup
        try:
            demo.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()