"""
Mission Testing System Launch File
Demonstrates ultra-clean mission architecture with multiple vehicles
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('host_ip', default_value='172.28.240.1',
                            description='AirSim server IP address'),
        DeclareLaunchArgument('host_port', default_value='41451',
                            description='AirSim server port'),
        DeclareLaunchArgument('enable_coordination', default_value='true',
                            description='Enable coordination node'),
        DeclareLaunchArgument('run_tests', default_value='true',
                            description='Run automated mission tests'),
        DeclareLaunchArgument('test_vehicles', default_value='Droan1,PX4_Drone2',
                            description='Comma-separated list of test vehicles'),
        
        # Information message
        LogInfo(msg="[EMOJI] Launching Mission Test System with Ultra-Clean Architecture"),
        LogInfo(msg="Vehicle-Name-As-Node-Name pattern: /Droan1, /PX4_Drone2, etc."),
        LogInfo(msg="Action servers: /VehicleName/actions/search_area"),
        LogInfo(msg="Services: /VehicleName/services/set_search_pattern"),
        LogInfo(msg="Topics: /VehicleName/mission/status, /VehicleName/detections/target"),
        
        # Mission-capable multirotor nodes with ultra-clean naming
        Node(
            package='airsim_ros_pkgs',
            executable='mission_multirotor_node',
            name='Droan1',
            namespace='',  # No namespace - vehicle name IS the node name
            parameters=[{
                'vehicle_name': 'Droan1',
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'use_sim_time': False,
                'mission_enabled': True,
                'search_pattern': 'spiral',
                'search_altitude': 20.0,
                'search_speed': 5.0
            }],
            output='screen',
            emulate_tty=True,
            prefix='echo "Starting Mission Node: Droan1" &&'
        ),
        
        Node(
            package='airsim_ros_pkgs',
            executable='mission_multirotor_node',
            name='PX4_Drone2', 
            namespace='',  # No namespace - vehicle name IS the node name
            parameters=[{
                'vehicle_name': 'PX4_Drone2',
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'use_sim_time': False,
                'mission_enabled': True,
                'search_pattern': 'lawnmower',
                'search_altitude': 25.0,
                'search_speed': 4.0
            }],
            output='screen',
            emulate_tty=True,
            prefix='echo "Starting Mission Node: PX4_Drone2" &&'
        ),
        
        # Coordination node for mission orchestration (optional)
        Node(
            package='airsim_ros_pkgs',
            executable='coordination_node',
            name='mission_coordinator',
            namespace='',
            parameters=[{
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'coordination_mode': 'mission_orchestration',
                'enable_mission_planning': True,
                'enable_zone_assignment': True
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_coordination')),
            prefix='echo "Starting Mission Coordinator" &&'
        ),
        
        # Launch test client after nodes have time to start
        TimerAction(
            period=15.0,  # Wait 15 seconds for nodes to initialize
            actions=[
                LogInfo(msg="[EMOJI] Starting Mission Interface Tests..."),
                Node(
                    package='airsim_ros_pkgs',
                    executable='test_mission_client.py',
                    name='mission_test_client',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('run_tests')),
                    prefix='echo "Running Mission Tests" &&'
                )
            ]
        ),
        
        # Status monitoring
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="STATS Mission System Status: Vehicle nodes active with ultra-clean naming")
            ]
        ),
    ])