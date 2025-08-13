from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """Simple multi-drone launch for development/testing"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('num_drones', default_value='2'),
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('host_port', default_value='41451'),
        
        # Drone 1
        GroupAction([
            PushRosNamespace('drone1'),
            Node(
                package='airsim_ros_pkgs',
                executable='multirotor_node',
                name='airsim_drone1',
                output='screen',
                parameters=[{
                    'vehicle_name': 'Drone1',
                    'host_ip': LaunchConfiguration('host_ip'),
                    'host_port': LaunchConfiguration('host_port'),
                    'world_frame_id': 'world_ned',
                    'odom_frame_id': 'drone1_odom_local_ned',
                    'state_timer_freq': 0.01,
                    'image_timer_freq': 0.05,
                    'lidar_timer_freq': 0.01,
                }],
                arguments=['Drone1']
            )
        ]),
        
        # Drone 2  
        GroupAction([
            PushRosNamespace('drone2'),
            Node(
                package='airsim_ros_pkgs',
                executable='multirotor_node',
                name='airsim_drone2',
                output='screen',
                parameters=[{
                    'vehicle_name': 'Drone2',
                    'host_ip': LaunchConfiguration('host_ip'),
                    'host_port': LaunchConfiguration('host_port'),
                    'world_frame_id': 'world_ned',
                    'odom_frame_id': 'drone2_odom_local_ned',
                    'state_timer_freq': 0.01,
                    'image_timer_freq': 0.05,
                    'lidar_timer_freq': 0.01,
                }],
                arguments=['Drone2']
            )
        ]),
        
        # Coordination Node
        Node(
            package='airsim_ros_pkgs',
            executable='coordination_node',
            name='airsim_coordination',
            output='screen',
            parameters=[{
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'world_frame_id': 'world_ned',
                'vehicle_names': ['Drone1', 'Drone2'],
            }]
        ),
    ])