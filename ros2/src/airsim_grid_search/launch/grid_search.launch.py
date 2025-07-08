#!/usr/bin/env python3

"""
Grid Search Launch File

Launches the complete grid search mission system including:
- Grid search action server
- Mission monitoring tools
- Optional visualization tools

Author: Cosys-Lab
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for grid search system"""
    
    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='drone_1',
        description='Name of the vehicle to control'
    )
    
    launch_monitor_arg = DeclareLaunchArgument(
        'launch_monitor',
        default_value='true',
        description='Launch mission monitor node'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    safety_check_frequency_arg = DeclareLaunchArgument(
        'safety_check_frequency',
        default_value='2.0',
        description='Safety check frequency in Hz'
    )
    
    position_tolerance_arg = DeclareLaunchArgument(
        'position_tolerance',
        default_value='2.0',
        description='Position tolerance for waypoint acceptance (meters)'
    )
    
    timeout_waypoint_arg = DeclareLaunchArgument(
        'timeout_waypoint',
        default_value='30.0',
        description='Timeout for waypoint navigation (seconds)'
    )
    
    # Grid search server node
    grid_search_server = Node(
        package='airsim_grid_search',
        executable='grid_search_server',
        name='grid_search_server',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'safety_check_frequency': LaunchConfiguration('safety_check_frequency'),
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'timeout_waypoint': LaunchConfiguration('timeout_waypoint'),
            'max_concurrent_missions': 1,
            'default_takeoff_altitude': 10.0,
            'default_flight_speed': 5.0,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Mission monitor node (optional)
    mission_monitor = Node(
        package='airsim_grid_search',
        executable='mission_monitor',
        name='mission_monitor',
        parameters=[{
            'update_frequency': 1.0,
            'log_mission_progress': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_monitor')),
        emulate_tty=True,
    )
    
    # RViz2 for visualization (optional)
    rviz_config_file = os.path.join(
        FindPackageShare('airsim_grid_search').find('airsim_grid_search'),
        'config',
        'grid_search.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )
    
    # Group all nodes
    grid_search_group = GroupAction([
        LogInfo(msg=['Starting Grid Search Mission System']),
        LogInfo(msg=['Vehicle: ', LaunchConfiguration('vehicle_name')]),
        LogInfo(msg=['Safety check frequency: ', LaunchConfiguration('safety_check_frequency'), ' Hz']),
        
        grid_search_server,
        mission_monitor,
        rviz_node,
        
        LogInfo(msg=['Grid Search Mission System ready!']),
    ])
    
    return LaunchDescription([
        vehicle_name_arg,
        launch_monitor_arg,
        launch_rviz_arg,
        safety_check_frequency_arg,
        position_tolerance_arg,
        timeout_waypoint_arg,
        
        grid_search_group,
    ])