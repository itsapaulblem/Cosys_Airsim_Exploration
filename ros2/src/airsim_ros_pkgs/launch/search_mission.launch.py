#!/usr/bin/env python3
"""
Search Mission Launch File - Search and Rescue Operations
Configurable launch file for various search and rescue scenarios

Supports multiple mission types:
- Single vehicle search patterns (spiral, grid, lawnmower)
- Multi-vehicle coordinated search operations
- GPS coordinate-based search areas
- Emergency response missions

Ultra-Clean Architecture: /VehicleName pattern with mission-capable nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from typing import List

def generate_launch_description():
    """Generate launch description for search and rescue missions"""
    
    # === CORE ARGUMENTS ===
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='172.28.240.1',
        description='IP address of AirSim server (auto-detected for WSL2)'
    )
    
    host_port_arg = DeclareLaunchArgument(
        'host_port',
        default_value='41451', 
        description='Port of AirSim server'
    )
    
    # === MISSION CONFIGURATION ===
    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='single_vehicle_search',
        choices=['single_vehicle_search', 'multi_vehicle_search', 'gps_search', 'spiral_search'],
        description='Type of search mission to execute'
    )
    
    search_pattern_arg = DeclareLaunchArgument(
        'search_pattern',
        default_value='spiral',
        choices=['spiral', 'grid', 'lawnmower'],
        description='Search pattern for systematic coverage'
    )
    
    primary_vehicle_arg = DeclareLaunchArgument(
        'primary_vehicle',
        default_value='Droan1',
        description='Primary vehicle for single vehicle missions or mission lead'
    )
    
    secondary_vehicle_arg = DeclareLaunchArgument(
        'secondary_vehicle',
        default_value='PX4_Drone2', 
        description='Secondary vehicle for multi-vehicle missions'
    )
    
    enable_coordination_arg = DeclareLaunchArgument(
        'enable_coordination',
        default_value='true',
        choices=['true', 'false'],
        description='Enable mission coordination node for multi-vehicle operations'
    )
    
    # === SEARCH PARAMETERS ===
    search_altitude_arg = DeclareLaunchArgument(
        'search_altitude',
        default_value='25.0',
        description='Search altitude in meters'
    )
    
    search_speed_arg = DeclareLaunchArgument(
        'search_speed', 
        default_value='5.0',
        description='Search speed in m/s'
    )
    
    pattern_spacing_arg = DeclareLaunchArgument(
        'pattern_spacing',
        default_value='15.0',
        description='Pattern spacing in meters (distance between search lines)'
    )
    
    detection_threshold_arg = DeclareLaunchArgument(
        'detection_threshold',
        default_value='0.7',
        description='Target detection confidence threshold (0.0-1.0)'
    )
    
    # === GPS SEARCH AREA (for GPS-based missions) ===
    gps_waypoints_arg = DeclareLaunchArgument(
        'gps_waypoints',
        default_value='47.641468,-122.140165 47.642468,-122.140165 47.642468,-122.139165 47.641468,-122.139165',
        description='GPS waypoints for search area (space-separated lat,lon pairs)'
    )
    
    # === SPIRAL SEARCH PARAMETERS ===
    spiral_center_x_arg = DeclareLaunchArgument(
        'spiral_center_x',
        default_value='0.0',
        description='Spiral search center X coordinate (meters)'
    )
    
    spiral_center_y_arg = DeclareLaunchArgument(
        'spiral_center_y', 
        default_value='0.0',
        description='Spiral search center Y coordinate (meters)'
    )
    
    spiral_max_radius_arg = DeclareLaunchArgument(
        'spiral_max_radius',
        default_value='100.0',
        description='Maximum spiral radius in meters'
    )
    
    # === MISSION TIMING ===
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='1800',  # 30 minutes
        description='Mission timeout in seconds'
    )
    
    auto_start_delay_arg = DeclareLaunchArgument(
        'auto_start_delay',
        default_value='10.0',
        description='Delay before auto-starting mission (seconds, 0 to disable)'
    )
    
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    mission_type = LaunchConfiguration('mission_type')
    primary_vehicle = LaunchConfiguration('primary_vehicle')
    secondary_vehicle = LaunchConfiguration('secondary_vehicle')
    enable_coordination = LaunchConfiguration('enable_coordination')
    
    # === MISSION-CAPABLE VEHICLE NODES ===
    
    # Primary vehicle (always started)
    primary_vehicle_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name=[primary_vehicle],  # Ultra-clean naming: /VehicleName
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': primary_vehicle,
            'search_altitude': LaunchConfiguration('search_altitude'),
            'search_speed': LaunchConfiguration('search_speed'),
            'pattern_spacing': LaunchConfiguration('pattern_spacing'),
            'detection_threshold': LaunchConfiguration('detection_threshold'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        remappings=[\n            # Ultra-clean topic remapping\n            ('mission/status', [primary_vehicle, '/mission/status']),\n            ('detections/target', [primary_vehicle, '/detections/target'])\n        ]
    )
    
    # Secondary vehicle (for multi-vehicle missions)
    secondary_vehicle_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name=[secondary_vehicle],
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': secondary_vehicle,
            'search_altitude': LaunchConfiguration('search_altitude'),
            'search_speed': LaunchConfiguration('search_speed'),
            'pattern_spacing': LaunchConfiguration('pattern_spacing'),
            'detection_threshold': LaunchConfiguration('detection_threshold'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(PythonExpression([\n            \"'\", mission_type, \"' in ['multi_vehicle_search']\"\n        ])),
        remappings=[\n            ('mission/status', [secondary_vehicle, '/mission/status']),\n            ('detections/target', [secondary_vehicle, '/detections/target'])\n        ]
    )
    
    # === MISSION COORDINATION NODE ===
    mission_coordination_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_coordination_node',
        name='search_mission_coordinator',
        namespace='',
        output='screen',
        parameters=[{
            'mission_type': mission_type,
            'primary_vehicle': primary_vehicle,
            'secondary_vehicle': secondary_vehicle,
            'search_pattern': LaunchConfiguration('search_pattern'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(enable_coordination)\n    )
    
    # === MISSION EXECUTION CLIENTS ===
    
    # Single vehicle search client
    single_vehicle_search_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='test_mission_client.py',
                name='single_vehicle_search_client',
                output='screen',
                parameters=[{
                    'target_vehicle': primary_vehicle,
                    'search_pattern': LaunchConfiguration('search_pattern'),
                    'search_altitude': LaunchConfiguration('search_altitude'),
                    'search_speed': LaunchConfiguration('search_speed'),
                    'pattern_spacing': LaunchConfiguration('pattern_spacing')\n                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([\n                    \"'\", mission_type, \"' == 'single_vehicle_search' and \",\n                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"\n                ]))\n            )\n        ],\n        condition=IfCondition(PythonExpression([\n            \"'\", mission_type, \"' == 'single_vehicle_search'\"\n        ]))\n    )
    
    # GPS-based search client
    gps_search_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='searchtrack_mission_ros2.py',
                name='gps_search_client',
                output='screen',
                parameters=[{
                    'vehicle': primary_vehicle,
                    'altitude': LaunchConfiguration('search_altitude'),
                    'speed': LaunchConfiguration('search_speed'),
                    'pattern': LaunchConfiguration('search_pattern'),
                    'spacing': LaunchConfiguration('pattern_spacing')\n                }],
                arguments=[\n                    '--waypoints', LaunchConfiguration('gps_waypoints'),\n                    '--vehicle', primary_vehicle,\n                    '--altitude', LaunchConfiguration('search_altitude'),\n                    '--speed', LaunchConfiguration('search_speed'),\n                    '--pattern', LaunchConfiguration('search_pattern'),\n                    '--spacing', LaunchConfiguration('pattern_spacing'),\n                    '--ros-args', '--log-level', 'INFO'\n                ],\n                condition=IfCondition(PythonExpression([\n                    \"'\", mission_type, \"' == 'gps_search' and \",\n                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"\n                ]))\n            )\n        ],\n        condition=IfCondition(PythonExpression([\n            \"'\", mission_type, \"' == 'gps_search'\"\n        ]))\n    )
    
    # Spiral search client
    spiral_search_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='spiral_search_ros2.py',
                name='spiral_search_client',
                output='screen',
                arguments=[\n                    '--vehicle', primary_vehicle,\n                    '--center_x', LaunchConfiguration('spiral_center_x'),\n                    '--center_y', LaunchConfiguration('spiral_center_y'),\n                    '--max_radius', LaunchConfiguration('spiral_max_radius'),\n                    '--altitude', LaunchConfiguration('search_altitude'),\n                    '--speed', LaunchConfiguration('search_speed'),\n                    '--spacing', LaunchConfiguration('pattern_spacing'),\n                    '--pattern', 'outward',\n                    '--ros-args', '--log-level', 'INFO'\n                ],\n                condition=IfCondition(PythonExpression([\n                    \"'\", mission_type, \"' == 'spiral_search' and \",\n                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"\n                ]))\n            )\n        ],\n        condition=IfCondition(PythonExpression([\n            \"'\", mission_type, \"' == 'spiral_search'\"\n        ]))\n    )
    
    # Multi-vehicle coordination client
    multi_vehicle_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='multi_vehicle_coordinator',
                output='screen',
                parameters=[{
                    'vehicles': [primary_vehicle, secondary_vehicle],
                    'search_pattern': LaunchConfiguration('search_pattern'),
                    'coordination_enabled': enable_coordination\n                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([\n                    \"'\", mission_type, \"' == 'multi_vehicle_search' and \",\n                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"\n                ]))\n            )\n        ],\n        condition=IfCondition(PythonExpression([\n            \"'\", mission_type, \"' == 'multi_vehicle_search'\"\n        ]))\n    )
    
    return LaunchDescription([\n        # Launch arguments\n        host_ip_arg,\n        host_port_arg,\n        mission_type_arg,\n        search_pattern_arg,\n        primary_vehicle_arg,\n        secondary_vehicle_arg,\n        enable_coordination_arg,\n        search_altitude_arg,\n        search_speed_arg,\n        pattern_spacing_arg,\n        detection_threshold_arg,\n        gps_waypoints_arg,\n        spiral_center_x_arg,\n        spiral_center_y_arg,\n        spiral_max_radius_arg,\n        mission_timeout_arg,\n        auto_start_delay_arg,\n        \n        # Vehicle nodes (mission-capable)\n        primary_vehicle_node,\n        secondary_vehicle_node,\n        \n        # Mission coordination\n        mission_coordination_node,\n        \n        # Mission execution clients\n        single_vehicle_search_client,\n        gps_search_client,\n        spiral_search_client,\n        multi_vehicle_client,\n    ])