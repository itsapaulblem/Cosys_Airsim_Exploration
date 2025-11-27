#!/usr/bin/env python3
"""
Mission Coordination Demonstration Launch File
Launches working ROS2 mission coordination system with C++ nodes

This demonstrates actual ROS2 mission orchestration with:
- Mission-capable vehicle nodes (/Droan1, /PX4_Drone2)
- Mission coordination node (/mission_coordinator)
- Real action/service/topic communication
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generate launch description for mission coordination demonstration
    """
    
    # Launch arguments
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='172.28.240.1',
        description='IP address of AirSim server'
    )
    
    host_port_arg = DeclareLaunchArgument(
        'host_port', 
        default_value='41451',
        description='Port of AirSim server'
    )
    
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    
    # Mission-capable vehicle nodes with ultra-clean naming
    vehicle_nodes = []
    
    # Mission-capable vehicle: Droan1
    droan1_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',  # Use mission-capable node
        name='Droan1',  # Ultra-clean naming: /Droan1
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': 'Droan1',
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    vehicle_nodes.append(droan1_node)
    
    # Mission-capable vehicle: PX4_Drone2
    px4_drone2_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',  # Use mission-capable node
        name='PX4_Drone2',  # Ultra-clean naming: /PX4_Drone2
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': 'PX4_Drone2',
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    vehicle_nodes.append(px4_drone2_node)
    
    # Mission coordination node (global orchestration)
    mission_coordination_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_coordination_node',
        name='mission_coordinator',  # Ultra-clean naming: /mission_coordinator
        namespace='',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # Demonstration client (starts after nodes are ready)
    mission_demo_client = TimerAction(
        period=5.0,  # Wait 5 seconds for nodes to start
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='mission_coordination_demo_client.py',
                name='mission_demo_client',
                output='screen',
                arguments=['--ros-args', '--log-level', 'INFO']
            )
        ]
    )
    
    return LaunchDescription([
        host_ip_arg,
        host_port_arg,
        
        # Start vehicle nodes
        *vehicle_nodes,
        
        # Start mission coordination node
        mission_coordination_node,
        
        # Start demonstration client after delay
        mission_demo_client,
    ])