from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_name', default_value='Drone1'),
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('host_port', default_value='41451'),
        
        # Simple single drone
        GroupAction([
            PushRosNamespace('drone1'),
            Node(
                package='airsim_ros_pkgs',
                executable='simple_multirotor_node',
                name='simple_drone',
                output='screen',
                parameters=[{
                    'vehicle_name': LaunchConfiguration('vehicle_name'),
                    'host_ip': LaunchConfiguration('host_ip'),
                    'host_port': LaunchConfiguration('host_port'),
                }],
                arguments=[LaunchConfiguration('vehicle_name')]
            )
        ]),
    ])