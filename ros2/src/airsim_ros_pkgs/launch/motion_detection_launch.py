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
        'motion_threshold', 
        default_value='15.0',
        description='Minimum movement in pixels to be considered moving'
    )

    # Person following parameters
    enable_following_arg = DeclareLaunchArgument(
        'enable_following',
        default_value='true',
        description='Enable person following behavior'
    )

    follow_distance_arg = DeclareLaunchArgument(
        'follow_distance',
        default_value='5.0',
        description='Desired following distance in meters'
    )

    max_follow_speed_arg = DeclareLaunchArgument(
        'max_follow_speed',
        default_value='2.0',
        description='Maximum following speed in m/s'
    )

    follow_height_arg = DeclareLaunchArgument(
        'follow_height',
        default_value='3.0',
        description='Following height in meters'
    )

    # Motion Detection Node with Person Following
    motion_detection_node = Node(
        package='airsim_ros_pkgs',
        executable='motion_detection_node.py',
        name='motion_detection_node',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'camera_topic': '/drone1/camera0/image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'motion_threshold': LaunchConfiguration('motion_threshold'),
            'enable_following': LaunchConfiguration('enable_following'),
            'follow_distance': LaunchConfiguration('follow_distance'),
            'max_follow_speed': LaunchConfiguration('max_follow_speed'),
            'follow_height': LaunchConfiguration('follow_height'),
            'image_width': 640,
            'image_height': 480
        }],
        remappings=[
            ('target_detection', '/drone1/target_detection'),
            ('detection_visualization', '/drone1/detection_visualization'),
        ],  
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        confidence_threshold_arg,
        motion_threshold_arg,
        enable_following_arg,
        follow_distance_arg,
        max_follow_speed_arg,
        follow_height_arg,
        motion_detection_node
    ])