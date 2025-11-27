#!/usr/bin/env python3
"""
AirSim ROS2 Velocity Command Helper

This script provides an easy interface for sending velocity commands to AirSim drones
via ROS2. It handles the complete message structure including twist, drivetrain, 
and yaw_mode fields.

Usage:
    python3 velocity_command_helper.py --vehicle PX4_Drone1 --forward 1.0
    python3 velocity_command_helper.py --vehicle PX4_Drone1 --x 1.0 --y 0.5 --z -0.5
    python3 velocity_command_helper.py --vehicle PX4_Drone1 --hover
    python3 velocity_command_helper.py --vehicle PX4_Drone1 --stop

Author: Cosys-AirSim ROS2 Integration
"""

import argparse
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from geometry_msgs.msg import Twist
from airsim_interfaces.msg import VelCmd, YawMode
from airsim_interfaces.srv import Takeoff, Land
import time


class VelocityCommandPublisher(Node):
    """
    ROS2 node for publishing velocity commands to AirSim drones.
    
    This handles the complete VelCmd message structure including:
    - Twist (linear and angular velocities)
    - Drivetrain type
    - Yaw mode configuration
    """
    
    def __init__(self, vehicle_name="PX4_Drone1", use_body_frame=True):
        super().__init__('velocity_command_publisher')
        
        self.vehicle_name = vehicle_name
        self.use_body_frame = use_body_frame
        
        # Determine topic based on frame type
        if use_body_frame:
            topic_name = f'/{vehicle_name}/vel_cmd_body_frame'
        else:
            topic_name = f'/{vehicle_name}/vel_cmd_world_frame'
            
        # Create publisher
        self.publisher = self.create_publisher(VelCmd, topic_name, 10)
        
        # Create service clients for takeoff and land
        self.takeoff_client = self.create_client(Takeoff, f'/{vehicle_name}/takeoff')
        self.land_client = self.create_client(Land, f'/{vehicle_name}/land')
        
        self.get_logger().info(f'Velocity command publisher initialized for {vehicle_name}')
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        
    def send_velocity_command(self, vx=0.0, vy=0.0, vz=0.0, 
                            yaw_rate=0.0, maintain_heading=True,
                            drivetrain_type=0):
        """
        Send a velocity command to the drone.
        
        Args:
            vx: Forward velocity (m/s) - positive = forward
            vy: Sideways velocity (m/s) - positive = right
            vz: Vertical velocity (m/s) - positive = down (NED coordinates)
            yaw_rate: Yaw rotation rate (rad/s)
            maintain_heading: If True, maintains current heading
            drivetrain_type: 0 = MaxDegreeOfFreedom, 1 = ForwardOnly
        """
        msg = VelCmd()
        
        # Set twist velocities
        msg.twist = Twist()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = 0.0  # Roll rate (usually not used)
        msg.twist.angular.y = 0.0  # Pitch rate (usually not used)
        msg.twist.angular.z = yaw_rate
        
        # Set drivetrain type
        msg.drivetrain = drivetrain_type
        
        # Set yaw mode
        msg.yaw_mode = YawMode()
        if maintain_heading:
            msg.yaw_mode.is_rate = True
            msg.yaw_mode.yaw_or_rate = yaw_rate
        else:
            msg.yaw_mode.is_rate = False
            msg.yaw_mode.yaw_or_rate = 0.0  # Could be set to desired angle
        
        # Publish the message
        self.publisher.publish(msg)
        
        frame_type = "body frame" if self.use_body_frame else "world frame"
        self.get_logger().info(
            f'Sent velocity command ({frame_type}): '
            f'vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, '
            f'yaw_rate={yaw_rate:.2f} rad/s'
        )
        
    def hover(self):
        """Send command to hover in place (zero velocities)."""
        self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Hovering in place')
        
    def stop(self):
        """Alias for hover - stops all movement."""
        self.hover()
        
    def move_forward(self, speed=1.0):
        """Move forward at specified speed."""
        self.send_velocity_command(vx=speed)
        
    def move_backward(self, speed=1.0):
        """Move backward at specified speed."""
        self.send_velocity_command(vx=-speed)
        
    def move_right(self, speed=1.0):
        """Move right at specified speed."""
        self.send_velocity_command(vy=speed)
        
    def move_left(self, speed=1.0):
        """Move left at specified speed."""
        self.send_velocity_command(vy=-speed)
        
    def move_up(self, speed=1.0):
        """Move up at specified speed (negative z in NED)."""
        self.send_velocity_command(vz=-speed)
        
    def move_down(self, speed=1.0):
        """Move down at specified speed (positive z in NED)."""
        self.send_velocity_command(vz=speed)
        
    def rotate_clockwise(self, rate=0.5):
        """Rotate clockwise at specified rate (rad/s)."""
        self.send_velocity_command(yaw_rate=rate)
        
    def rotate_counter_clockwise(self, rate=0.5):
        """Rotate counter-clockwise at specified rate (rad/s)."""
        self.send_velocity_command(yaw_rate=-rate)
        
    def call_service_sync(self, client: Client, request, service_name: str, timeout: float = 10.0):
        """Call a service synchronously with timeout."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{service_name} service not available')
            return None
            
        future = client.call_async(request)
        
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'{service_name} service call failed: {e}')
                    return None
            
            if time.time() - start_time > timeout:
                self.get_logger().error(f'{service_name} service call timed out')
                return None
        
        return None
    
    def takeoff(self, wait_for_completion: bool = True) -> bool:
        """
        Command the drone to takeoff.
        
        Args:
            wait_for_completion: If True, waits for takeoff to complete
            
        Returns:
            True if takeoff command was successful
        """
        self.get_logger().info('Commanding takeoff...')
        
        request = Takeoff.Request()
        response = self.call_service_sync(self.takeoff_client, request, 'takeoff')
        
        if response is not None:
            self.get_logger().info('Takeoff command sent successfully')
            if wait_for_completion:
                self.get_logger().info('Waiting for takeoff to complete (5 seconds)...')
                time.sleep(5.0)
            return True
        else:
            self.get_logger().error('Takeoff command failed')
            return False
    
    def land(self, wait_for_completion: bool = True) -> bool:
        """
        Command the drone to land.
        
        Args:
            wait_for_completion: If True, waits for landing to complete
            
        Returns:
            True if land command was successful
        """
        self.get_logger().info('Commanding landing...')
        
        request = Land.Request()
        response = self.call_service_sync(self.land_client, request, 'land')
        
        if response is not None:
            self.get_logger().info('Land command sent successfully')
            if wait_for_completion:
                self.get_logger().info('Waiting for landing to complete (10 seconds)...')
                time.sleep(10.0)
            return True
        else:
            self.get_logger().error('Land command failed')
            return False


def main():
    parser = argparse.ArgumentParser(
        description='Send velocity commands to AirSim drones via ROS2'
    )
    
    # Vehicle selection
    parser.add_argument(
        '--vehicle', '-v',
        default='PX4_Drone1',
        help='Vehicle name (default: PX4_Drone1)'
    )
    
    # Frame selection
    parser.add_argument(
        '--world-frame', '-w',
        action='store_true',
        help='Use world frame instead of body frame'
    )
    
    # Velocity components
    parser.add_argument('--x', type=float, default=0.0,
                       help='X velocity (forward/north)')
    parser.add_argument('--y', type=float, default=0.0,
                       help='Y velocity (right/east)')
    parser.add_argument('--z', type=float, default=0.0,
                       help='Z velocity (down)')
    parser.add_argument('--yaw', type=float, default=0.0,
                       help='Yaw rate (rad/s)')
    
    # Convenience commands
    parser.add_argument('--forward', type=float,
                       help='Move forward at specified speed')
    parser.add_argument('--backward', type=float,
                       help='Move backward at specified speed')
    parser.add_argument('--right', type=float,
                       help='Move right at specified speed')
    parser.add_argument('--left', type=float,
                       help='Move left at specified speed')
    parser.add_argument('--up', type=float,
                       help='Move up at specified speed')
    parser.add_argument('--down', type=float,
                       help='Move down at specified speed')
    parser.add_argument('--hover', action='store_true',
                       help='Hover in place (zero velocities)')
    parser.add_argument('--stop', action='store_true',
                       help='Stop all movement (alias for hover)')
    
    # Takeoff and land commands  
    parser.add_argument('--takeoff', action='store_true',
                       help='Command drone to takeoff')
    parser.add_argument('--land', action='store_true',
                       help='Command drone to land')
    
    # Continuous mode
    parser.add_argument(
        '--continuous', '-c',
        action='store_true',
        help='Send commands continuously (use Ctrl+C to stop)'
    )
    
    parser.add_argument(
        '--rate', '-r',
        type=float,
        default=10.0,
        help='Command publishing rate in Hz (default: 10.0)'
    )
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create publisher node
    use_body_frame = not args.world_frame
    node = VelocityCommandPublisher(args.vehicle, use_body_frame)
    
    try:
        # Handle service commands first (takeoff/land)
        if args.takeoff:
            success = node.takeoff()
            if success:
                node.get_logger().info('Takeoff completed - drone is now airborne')
            else:
                node.get_logger().error('Takeoff failed')
            return
        elif args.land:
            success = node.land()
            if success:
                node.get_logger().info('Landing completed')
            else:
                node.get_logger().error('Landing failed')
            return
        
        # Handle velocity commands
        if args.hover or args.stop:
            command = lambda: node.hover()
        elif args.forward is not None:
            command = lambda: node.move_forward(args.forward)
        elif args.backward is not None:
            command = lambda: node.move_backward(args.backward)
        elif args.right is not None:
            command = lambda: node.move_right(args.right)
        elif args.left is not None:
            command = lambda: node.move_left(args.left)
        elif args.up is not None:
            command = lambda: node.move_up(args.up)
        elif args.down is not None:
            command = lambda: node.move_down(args.down)
        else:
            # Use explicit x, y, z, yaw values
            command = lambda: node.send_velocity_command(
                args.x, args.y, args.z, args.yaw
            )
        
        if args.continuous:
            # Continuous mode - send commands at specified rate
            node.get_logger().info(
                f'Sending commands continuously at {args.rate} Hz. '
                'Press Ctrl+C to stop.'
            )
            
            timer = node.create_timer(1.0 / args.rate, command)
            rclpy.spin(node)
        else:
            # Single command mode
            command()
            # Give time for the message to be sent
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info('Stopping velocity commands')
        node.hover()  # Stop the drone before exiting
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()