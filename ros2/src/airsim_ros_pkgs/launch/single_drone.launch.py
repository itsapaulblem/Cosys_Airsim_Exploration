from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('host_port', default_value='41451'),

<<<<<<< HEAD
        # Drone 1
=======
        # Drone 1 with enhanced parallel processing
>>>>>>> main
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
<<<<<<< HEAD
                    'state_timer_freq': 0.01,
                    'image_timer_freq': 0.05,
                    'lidar_timer_freq': 0.01,
=======
                    # Enhanced parallel processing frequencies
                    'state_timer_freq': 0.02,   # 50Hz for responsive control
                    'image_timer_freq': 0.033,  # 30Hz for smooth video
                    'lidar_timer_freq': 0.05,   # 20Hz for lidar data
                    'echo_timer_freq': 0.1,     # 10Hz for other sensors
>>>>>>> main
                }],
                arguments=['Drone1']
            )
        ]),

        # Coordination Node (only manages Drone1)
        Node(
            package='airsim_ros_pkgs',
            executable='coordination_node',
            name='airsim_coordination',
            output='screen',
            parameters=[{
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'world_frame_id': 'world_ned',
                'vehicle_names': ['Drone1'],
            }]
        ),
    ])