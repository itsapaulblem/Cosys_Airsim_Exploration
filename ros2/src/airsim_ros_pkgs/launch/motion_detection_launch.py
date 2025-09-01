import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='Drone1',
        description='Name of the vehicle'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for object detection'
    )

    motion_threshold_arg = DeclareLaunchArgument(
        'motion_threshold', default_value='15.0',
        description='Minimum movement in pixels to be considered moving'
    )

    # Motion Detection Node
    motion_detection_node = Node(
        package='airsim_ros_pkgs',
        executable='motion_detection_node.py',
        name='motion_detection_node',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'camera_topic': '/drone1/camera0/image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'motion_threshold': LaunchConfiguration('motion_threshold')
        }],
        remappings=[
            ('target_detection', '/drone1/target_detection'),
        ],  
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        confidence_threshold_arg,
        motion_threshold_arg, 
        motion_detection_node
    ])