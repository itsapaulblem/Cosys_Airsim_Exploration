#!/usr/bin/env python3
"""
Multi-Vehicle Mission Launch File - Coordinated Multi-Drone Operations
Advanced coordination for complex multi-vehicle missions

Mission Types:
- Synchronized area coverage with zone assignment
- Leader-follower formations for convoy escort
- Distributed search with real-time coordination
- Swarm-based surveillance and reconnaissance
- Emergency response with role specialization

Features:
- Intelligent zone allocation based on vehicle capabilities
- Real-time mission replanning and adaptation
- Collision avoidance and spatial coordination
- Load balancing and failure recovery
- Cross-vehicle communication and data sharing

Ultra-Clean Architecture: /VehicleName with centralized mission coordination
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from typing import List

def generate_launch_description():
    """Generate launch description for multi-vehicle coordinated missions"""
    
    # === CORE ARGUMENTS ===
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
    
    # === MISSION TYPE CONFIGURATION ===
    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='coordinated_search',
        choices=['coordinated_search', 'formation_flight', 'distributed_survey', 'swarm_surveillance', 'emergency_response'],
        description='Type of multi-vehicle mission'
    )
    
    coordination_strategy_arg = DeclareLaunchArgument(
        'coordination_strategy',
        default_value='zone_allocation',
        choices=['zone_allocation', 'leader_follower', 'distributed', 'hierarchical'],
        description='Strategy for multi-vehicle coordination'
    )
    
    # === VEHICLE CONFIGURATION ===
    num_vehicles_arg = DeclareLaunchArgument(
        'num_vehicles',
        default_value='3',
        description='Number of vehicles to deploy (2-6 supported)'
    )
    
    vehicle_names_arg = DeclareLaunchArgument(
        'vehicle_names',
        default_value='Droan1,PX4_Drone2,SimpleFlight3',
        description='Comma-separated vehicle names for ultra-clean architecture'
    )
    
    leader_vehicle_arg = DeclareLaunchArgument(
        'leader_vehicle',
        default_value='Droan1',
        description='Lead vehicle for leader-follower coordination'
    )
    
    # === VEHICLE ROLE SPECIALIZATION ===
    vehicle_roles_arg = DeclareLaunchArgument(
        'vehicle_roles',
        default_value='leader,scout,support',
        description='Comma-separated roles: leader, scout, support, surveillance, emergency'
    )
    
    # === MISSION AREA DEFINITION ===
    mission_area_corners_arg = DeclareLaunchArgument(
        'mission_area_corners',
        default_value='0,0 300,0 300,200 0,200',
        description='Mission area corners in NED coordinates (automatically divided into zones)'
    )
    
    zone_overlap_arg = DeclareLaunchArgument(
        'zone_overlap',
        default_value='20.0',
        description='Overlap between vehicle zones in meters (for coordination boundaries)'
    )
    
    # === COORDINATION PARAMETERS ===
    formation_spacing_arg = DeclareLaunchArgument(
        'formation_spacing',
        default_value='50.0',
        description='Spacing between vehicles in formation (meters)'
    )
    
    formation_type_arg = DeclareLaunchArgument(
        'formation_type',
        default_value='line',
        choices=['line', 'triangle', 'diamond', 'circle', 'custom'],
        description='Formation pattern for formation flight missions'
    )
    
    coordination_altitude_arg = DeclareLaunchArgument(
        'coordination_altitude',
        default_value='30.0',
        description='Base coordination altitude (vehicles may use different altitudes for separation)'
    )
    
    altitude_separation_arg = DeclareLaunchArgument(
        'altitude_separation',
        default_value='10.0',
        description='Vertical separation between vehicles (meters)'
    )
    
    # === MISSION EXECUTION PARAMETERS ===
    mission_speed_arg = DeclareLaunchArgument(
        'mission_speed',
        default_value='6.0',
        description='Coordinated mission speed in m/s'
    )
    
    search_pattern_arg = DeclareLaunchArgument(
        'search_pattern',
        default_value='grid',
        choices=['grid', 'spiral', 'lawnmower', 'custom'],
        description='Search pattern for coordinated search missions'
    )
    
    pattern_spacing_arg = DeclareLaunchArgument(
        'pattern_spacing',
        default_value='20.0',
        description='Pattern spacing within each vehicle zone'
    )
    
    # === SYNCHRONIZATION AND TIMING ===
    sync_waypoints_arg = DeclareLaunchArgument(
        'sync_waypoints',
        default_value='true',
        choices=['true', 'false'],
        description='Synchronize vehicles at waypoints for coordinated movement'
    )
    
    checkpoint_interval_arg = DeclareLaunchArgument(
        'checkpoint_interval',
        default_value='60.0',
        description='Interval for coordination checkpoints (seconds)'
    )
    
    max_separation_distance_arg = DeclareLaunchArgument(
        'max_separation_distance',
        default_value='500.0',
        description='Maximum allowed separation between coordinated vehicles'
    )
    
    # === COMMUNICATION AND DATA SHARING ===
    enable_data_fusion_arg = DeclareLaunchArgument(
        'enable_data_fusion',
        default_value='true',
        choices=['true', 'false'],
        description='Enable cross-vehicle data fusion and sharing'
    )
    
    communication_range_arg = DeclareLaunchArgument(
        'communication_range',
        default_value='1000.0',
        description='Effective communication range between vehicles (meters)'
    )
    
    # === SAFETY AND COLLISION AVOIDANCE ===
    collision_avoidance_arg = DeclareLaunchArgument(
        'collision_avoidance',
        default_value='true',
        choices=['true', 'false'],
        description='Enable active collision avoidance between vehicles'
    )
    
    safety_bubble_radius_arg = DeclareLaunchArgument(
        'safety_bubble_radius',
        default_value='25.0',
        description='Safety bubble radius around each vehicle (meters)'
    )
    
    emergency_landing_zones_arg = DeclareLaunchArgument(
        'emergency_landing_zones',
        default_value='50,50 150,150 250,50',
        description='Emergency landing zones (space-separated x,y coordinates)'
    )
    
    # === MISSION TIMING ===
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='3600',  # 60 minutes
        description='Mission timeout in seconds'
    )
    
    coordination_delay_arg = DeclareLaunchArgument(
        'coordination_delay',
        default_value='20.0',
        description='Delay for coordination setup before mission start'
    )
    
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    num_vehicles = LaunchConfiguration('num_vehicles')
    vehicle_names = LaunchConfiguration('vehicle_names')
    mission_type = LaunchConfiguration('mission_type')
    
    # === DYNAMIC VEHICLE NODE GENERATION ===
    # Note: In a real implementation, this would dynamically generate nodes based on num_vehicles
    # For this template, we'll create nodes for the standard 3 vehicles
    
    vehicle_1_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name='Droan1',  # Ultra-clean naming
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': 'Droan1',
            'mission_type': 'multi_vehicle',
            'vehicle_role': 'leader',
            'coordination_altitude': LaunchConfiguration('coordination_altitude'),
            'mission_speed': LaunchConfiguration('mission_speed'),
            'formation_spacing': LaunchConfiguration('formation_spacing'),
            'collision_avoidance': LaunchConfiguration('collision_avoidance'),
            'safety_bubble_radius': LaunchConfiguration('safety_bubble_radius'),
            'communication_range': LaunchConfiguration('communication_range'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        remappings=[
            ('mission/status', 'Droan1/mission/status'),
            ('coordination/status', 'Droan1/coordination/status'),
            ('formation/position', 'Droan1/formation/position')
        ]
    )
    
    vehicle_2_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name='PX4_Drone2',
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': 'PX4_Drone2',
            'mission_type': 'multi_vehicle',
            'vehicle_role': 'scout',
            'coordination_altitude': PythonExpression([
                LaunchConfiguration('coordination_altitude'), ' + ', 
                LaunchConfiguration('altitude_separation')
            ]),
            'mission_speed': LaunchConfiguration('mission_speed'),
            'formation_spacing': LaunchConfiguration('formation_spacing'),
            'collision_avoidance': LaunchConfiguration('collision_avoidance'),
            'safety_bubble_radius': LaunchConfiguration('safety_bubble_radius'),
            'communication_range': LaunchConfiguration('communication_range'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(PythonExpression([
            \"int('\", num_vehicles, \"') >= 2\"
        ])),
        remappings=[
            ('mission/status', 'PX4_Drone2/mission/status'),
            ('coordination/status', 'PX4_Drone2/coordination/status'),
            ('formation/position', 'PX4_Drone2/formation/position')
        ]
    )
    
    vehicle_3_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name='SimpleFlight3',
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': 'SimpleFlight3',
            'mission_type': 'multi_vehicle',
            'vehicle_role': 'support',
            'coordination_altitude': PythonExpression([
                LaunchConfiguration('coordination_altitude'), ' + 2 * ', 
                LaunchConfiguration('altitude_separation')
            ]),
            'mission_speed': LaunchConfiguration('mission_speed'),
            'formation_spacing': LaunchConfiguration('formation_spacing'),
            'collision_avoidance': LaunchConfiguration('collision_avoidance'),
            'safety_bubble_radius': LaunchConfiguration('safety_bubble_radius'),
            'communication_range': LaunchConfiguration('communication_range'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(PythonExpression([
            \"int('\", num_vehicles, \"') >= 3\"
        ])),
        remappings=[
            ('mission/status', 'SimpleFlight3/mission/status'),
            ('coordination/status', 'SimpleFlight3/coordination/status'),
            ('formation/position', 'SimpleFlight3/formation/position')
        ]
    )
    
    # === MULTI-VEHICLE COORDINATION NODE ===
    multi_vehicle_coordinator = Node(
        package='airsim_ros_pkgs',
        executable='mission_coordination_node',
        name='multi_vehicle_coordinator',
        namespace='',
        output='screen',
        parameters=[{
            'mission_type': mission_type,
            'coordination_strategy': LaunchConfiguration('coordination_strategy'),
            'num_vehicles': num_vehicles,
            'vehicle_names': vehicle_names,
            'vehicle_roles': LaunchConfiguration('vehicle_roles'),
            'leader_vehicle': LaunchConfiguration('leader_vehicle'),
            'mission_area': LaunchConfiguration('mission_area_corners'),
            'zone_overlap': LaunchConfiguration('zone_overlap'),
            'formation_type': LaunchConfiguration('formation_type'),
            'formation_spacing': LaunchConfiguration('formation_spacing'),
            'sync_waypoints': LaunchConfiguration('sync_waypoints'),
            'checkpoint_interval': LaunchConfiguration('checkpoint_interval'),
            'max_separation_distance': LaunchConfiguration('max_separation_distance'),
            'collision_avoidance': LaunchConfiguration('collision_avoidance'),
            'enable_data_fusion': LaunchConfiguration('enable_data_fusion'),
            'emergency_landing_zones': LaunchConfiguration('emergency_landing_zones'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # === ZONE ALLOCATION SERVICE ===
    zone_allocation_service = Node(
        package='airsim_ros_pkgs',
        executable='simple_mission_coordination_demo.py',
        name='zone_allocation_service',
        output='screen',
        parameters=[{
            'service_type': 'zone_allocation',
            'mission_area': LaunchConfiguration('mission_area_corners'),
            'num_vehicles': num_vehicles,
            'vehicle_capabilities': vehicle_names,
            'zone_overlap': LaunchConfiguration('zone_overlap'),
            'optimization_strategy': 'capability_based'
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # === MISSION EXECUTION CLIENTS ===
    
    # Coordinated search mission
    coordinated_search_client = TimerAction(
        period=LaunchConfiguration('coordination_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='coordinated_search_client',
                output='screen',
                parameters=[{
                    'mission_type': 'coordinated_search',
                    'vehicles': vehicle_names,
                    'search_pattern': LaunchConfiguration('search_pattern'),
                    'pattern_spacing': LaunchConfiguration('pattern_spacing'),
                    'coordination_enabled': 'true',
                    'zone_allocation': 'automatic'
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", mission_type, \"' == 'coordinated_search'\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", mission_type, \"' == 'coordinated_search'\"
        ]))
    )
    
    # Formation flight mission
    formation_flight_client = TimerAction(
        period=LaunchConfiguration('coordination_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='formation_flight_client',
                output='screen',
                parameters=[{
                    'mission_type': 'formation_flight',
                    'vehicles': vehicle_names,
                    'leader_vehicle': LaunchConfiguration('leader_vehicle'),
                    'formation_type': LaunchConfiguration('formation_type'),
                    'formation_spacing': LaunchConfiguration('formation_spacing'),
                    'sync_waypoints': LaunchConfiguration('sync_waypoints')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", mission_type, \"' == 'formation_flight'\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", mission_type, \"' == 'formation_flight'\"
        ]))
    )
    
    # Distributed survey mission
    distributed_survey_client = TimerAction(
        period=LaunchConfiguration('coordination_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='distributed_survey_client',
                output='screen',
                parameters=[{
                    'mission_type': 'distributed_survey',
                    'vehicles': vehicle_names,
                    'mission_area': LaunchConfiguration('mission_area_corners'),
                    'pattern_spacing': LaunchConfiguration('pattern_spacing'),
                    'altitude_separation': LaunchConfiguration('altitude_separation'),
                    'enable_data_fusion': LaunchConfiguration('enable_data_fusion')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", mission_type, \"' == 'distributed_survey'\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", mission_type, \"' == 'distributed_survey'\"
        ]))
    )
    
    # Swarm surveillance mission
    swarm_surveillance_client = TimerAction(
        period=LaunchConfiguration('coordination_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='swarm_surveillance_client',
                output='screen',
                parameters=[{
                    'mission_type': 'swarm_surveillance',
                    'vehicles': vehicle_names,
                    'coordination_strategy': LaunchConfiguration('coordination_strategy'),
                    'surveillance_altitude': LaunchConfiguration('coordination_altitude'),
                    'patrol_speed': LaunchConfiguration('mission_speed'),
                    'enable_data_fusion': LaunchConfiguration('enable_data_fusion')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", mission_type, \"' == 'swarm_surveillance'\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", mission_type, \"' == 'swarm_surveillance'\"
        ]))
    )
    
    # Emergency response mission
    emergency_response_client = TimerAction(
        period=LaunchConfiguration('coordination_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='emergency_response_client',
                output='screen',
                parameters=[{
                    'mission_type': 'emergency_response',
                    'vehicles': vehicle_names,
                    'vehicle_roles': LaunchConfiguration('vehicle_roles'),
                    'emergency_area': LaunchConfiguration('mission_area_corners'),
                    'emergency_landing_zones': LaunchConfiguration('emergency_landing_zones'),
                    'rapid_deployment': 'true'
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", mission_type, \"' == 'emergency_response'\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", mission_type, \"' == 'emergency_response'\"
        ]))
    )
    
    # === DATA FUSION AND MONITORING ===
    multi_vehicle_monitor = Node(
        package='airsim_ros_pkgs',
        executable='simple_mission_coordination_demo.py',
        name='multi_vehicle_monitor',
        output='screen',
        parameters=[{
            'monitor_vehicles': vehicle_names,
            'enable_data_fusion': LaunchConfiguration('enable_data_fusion'),
            'log_coordination_data': 'true',
            'real_time_analysis': 'true'
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(LaunchConfiguration('enable_data_fusion'))
    )
    
    return LaunchDescription([
        # Launch arguments
        host_ip_arg,
        host_port_arg,
        mission_type_arg,
        coordination_strategy_arg,
        num_vehicles_arg,
        vehicle_names_arg,
        leader_vehicle_arg,
        vehicle_roles_arg,
        mission_area_corners_arg,
        zone_overlap_arg,
        formation_spacing_arg,
        formation_type_arg,
        coordination_altitude_arg,
        altitude_separation_arg,
        mission_speed_arg,
        search_pattern_arg,
        pattern_spacing_arg,
        sync_waypoints_arg,
        checkpoint_interval_arg,
        max_separation_distance_arg,
        enable_data_fusion_arg,
        communication_range_arg,
        collision_avoidance_arg,
        safety_bubble_radius_arg,
        emergency_landing_zones_arg,
        mission_timeout_arg,
        coordination_delay_arg,
        
        # Vehicle nodes (mission-capable with coordination)
        vehicle_1_node,
        vehicle_2_node,
        vehicle_3_node,
        
        # Multi-vehicle coordination
        multi_vehicle_coordinator,
        zone_allocation_service,
        
        # Mission execution clients
        coordinated_search_client,
        formation_flight_client,
        distributed_survey_client,
        swarm_surveillance_client,
        emergency_response_client,
        
        # Data fusion and monitoring
        multi_vehicle_monitor,
    ])