#!/usr/bin/env python3
"""
Simple AirSim ROS2 Subscriber Example

This is a minimal example showing how to subscribe to AirSim ROS2 topics
and send basic commands. Based on the ROS2 tutorial pattern from:
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

Author: Generated for Cosys-AirSim ROS2 Integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS2 messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

# AirSim custom messages
from airsim_interfaces.msg import VelCmd, Environment

class SimpleAirSimSubscriber(Node):
    """
    Simple subscriber following ROS2 tutorial pattern
    """
    
    def __init__(self):
        super().__init__('simple_airsim_subscriber')
        
        # Configure QoS for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom_local',
            self.odom_listener_callback,
            qos_profile)
        
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/global_gps',
            self.gps_listener_callback,
            qos_profile)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_listener_callback,
            qos_profile)
        
        # Create publisher for velocity commands
        self.vel_publisher = self.create_publisher(VelCmd, '/vel_cmd_world_frame', 10)
        
        # Create timer for periodic commands
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # State variables
        self.current_position = None
        self.current_gps = None
        self.command_counter = 0
        
        self.get_logger().info('Simple AirSim Subscriber has been started')
    
    def odom_listener_callback(self, msg):
        """
        Callback for odometry messages
        Similar to ROS2 tutorial callback pattern
        """
        self.current_position = msg.pose.pose.position
        
        self.get_logger().info(
            f'Received odometry: position=({self.current_position.x:.2f}, '
            f'{self.current_position.y:.2f}, {self.current_position.z:.2f})')
    
    def gps_listener_callback(self, msg):
        """
        Callback for GPS messages
        """
        self.current_gps = msg
        
        self.get_logger().info(
            f'Received GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, '
            f'alt={msg.altitude:.2f}m')
    
    def imu_listener_callback(self, msg):
        """
        Callback for IMU messages
        """
        accel = msg.linear_acceleration
        self.get_logger().info(
            f'Received IMU: accel=({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f})')
    
    def timer_callback(self):
        """
        Timer callback for periodic actions
        Similar to ROS2 tutorial timer pattern
        """
        self.command_counter += 1
        
        # Send different commands based on counter
        if self.command_counter % 4 == 1:
            # Move forward
            self.send_velocity_command(1.0, 0.0, 0.0, 0.0)
            self.get_logger().info('Sent command: Move forward')
        elif self.command_counter % 4 == 2:
            # Move right
            self.send_velocity_command(0.0, 1.0, 0.0, 0.0)
            self.get_logger().info('Sent command: Move right')
        elif self.command_counter % 4 == 3:
            # Move backward
            self.send_velocity_command(-1.0, 0.0, 0.0, 0.0)
            self.get_logger().info('Sent command: Move backward')
        else:
            # Stop
            self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('Sent command: Stop')
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        """
        Send velocity command to AirSim
        """
        msg = VelCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.vehicle_name = ""  # Empty means all vehicles
        msg.vx = vx
        msg.vy = vy
        msg.vz = vz
        msg.yaw_rate = yaw_rate
        
        self.vel_publisher.publish(msg)


def main(args=None):
    """
    Main function following ROS2 tutorial pattern
    """
    rclpy.init(args=args)
    
    simple_airsim_subscriber = SimpleAirSimSubscriber()
    
    try:
        rclpy.spin(simple_airsim_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    simple_airsim_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()