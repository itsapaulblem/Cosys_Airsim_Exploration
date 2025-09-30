import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    drone_names_arg = DeclareLaunchArgument(
        'drone_names',
        default_value='["Drone1", "Drone2", "Drone3"]',
        description='List of drone names to manage'
    )
    
    leader_drone_arg = DeclareLaunchArgument(
        'leader_drone',
        default_value='Drone1',
        description='Name of the leader drone'
    )
    
    formation_type_arg = DeclareLaunchArgument(
        'formation_type',
        default_value='triangle',
        description='Formation type: triangle, line, or box'
    )
    
    formation_spacing_arg = DeclareLaunchArgument(
        'formation_spacing',
        default_value='5.0',
        description='Distance between drones in formation (meters)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization output'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='Minimum confidence for target detection'
    )
    
    search_pattern_arg = DeclareLaunchArgument(
        'search_pattern',
        default_value='grid',
        description='Search pattern: grid, spiral, or random'
    )
    
    # Get launch configurations
    drone_names = LaunchConfiguration('drone_names')
    leader_drone = LaunchConfiguration('leader_drone')
    formation_type = LaunchConfiguration('formation_type')
    formation_spacing = LaunchConfiguration('formation_spacing')
    enable_visualization = LaunchConfiguration('enable_visualization')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    search_pattern = LaunchConfiguration('search_pattern')
    
    # Multi-drone coordinator node
    coordinator_node = Node(
        package='airsim_ros_pkgs',
        executable='multi_drone_coordinator.py',
        name='multi_drone_coordinator',
        output='screen',
        parameters=[{
            'drone_names': drone_names,
            'leader_drone': leader_drone,
            'formation_type': formation_type,
            'formation_spacing': formation_spacing,
            'enable_visualization': enable_visualization,
            'confidence_threshold': confidence_threshold,
            'search_pattern': search_pattern,
            'coordination_frequency': 10.0,
            'enable_swarm_intelligence': True,
            'takeoff_height': 3.0,
            'follow_distance': 2.0,
            'target_assignment_method': 'closest'
        }]
    )
    
    # Individual drone nodes (multirotor_node for each drone)
    drone_nodes = []
    drone_names_list = ['Drone1', 'Drone2', 'Drone3']  # Default list
    
    for drone_name in drone_names_list:
        # Individual multirotor node for each drone
        multirotor_node = Node(
            package='airsim_ros_pkgs',
            executable='multirotor_node',
            name=f'{drone_name.lower()}_multirotor_node',
            namespace=drone_name.lower(),
            output='screen',
            parameters=[{
                'vehicle_name': drone_name,
                'host_ip': '127.0.0.1',
                'host_port': 41451
            }]
        )
        
        # Motion detection node for each drone
        motion_detection_node = Node(
            package='airsim_ros_pkgs',
            executable='motion_detection_node.py',
            name=f'{drone_name.lower()}_motion_detection',
            namespace=drone_name.lower(),
            output='screen',
            parameters=[{
                'vehicle_name': drone_name,
                'confidence_threshold': confidence_threshold,
                'enable_visualization': enable_visualization,
                'enable_following': False,  # Disable individual following - coordinator handles this
                'motion_threshold': 15.0,
                'trail_length': 30,
                'takeoff_height': 3.0,
                'image_width': 640,
                'image_height': 480
            }]
        )
        
        drone_nodes.extend([multirotor_node, motion_detection_node])
    
    # Group all nodes
    multi_drone_group = GroupAction([
        coordinator_node
    ] + drone_nodes)
    
    return LaunchDescription([
        drone_names_arg,
        leader_drone_arg,
        formation_type_arg,
        formation_spacing_arg,
        enable_visualization_arg,
        confidence_threshold_arg,
        search_pattern_arg,
        multi_drone_group
    ])