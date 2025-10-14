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

    # Target class configuration (key difference from motion_detection_launch)
    target_class_id_arg = DeclareLaunchArgument(
        'target_class_id',
        default_value='0',
        description='Target class ID to track (0=person, 2=car, 3=motorcycle, 4=airplane/drone, etc.)'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.08',
        description='Confidence threshold for object detection'
    )

    motion_threshold_arg = DeclareLaunchArgument(
        'motion_threshold', 
        default_value='15.0',
        description='Minimum movement in pixels to be considered moving'
    )

    # Object following parameters
    enable_following_arg = DeclareLaunchArgument(
        'enable_following',
        default_value='true',
        description='Enable target following behavior'
    )

    follow_distance_arg = DeclareLaunchArgument(
        'follow_distance',
        default_value='3.0',
        description='Desired following distance in meters'
    )

    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )

    # Image quality parameters
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Image width for processing and visualization'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height', 
        default_value='720',
        description='Image height for processing and visualization'
    )
    
    image_quality_arg = DeclareLaunchArgument(
        'image_quality',
        default_value='95',
        description='Image quality for visualization (0-100)'
    )

    # Generalised Object Tracking Node
    object_tracking_node = Node(
        package='airsim_ros_pkgs',
        executable='generalised_object_tracking_node.py',
        name='generalised_object_tracking_node',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'target_class_id': LaunchConfiguration('target_class_id'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'motion_threshold': LaunchConfiguration('motion_threshold'),
            'enable_following': LaunchConfiguration('enable_following'),
            'follow_distance': LaunchConfiguration('follow_distance'),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'image_quality': LaunchConfiguration('image_quality'),
            'enable_image_resize': True,
            'enable_visualization': True,
            'iou_threshold': 0.45,
            'trail_length': 30
        }],
        remappings=[
            ('target_detection', '/drone1/target_detection'),
        ],  
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        target_class_id_arg,
        confidence_threshold_arg,
        motion_threshold_arg,
        enable_following_arg,
        follow_distance_arg,
        takeoff_height_arg,
        image_width_arg,
        image_height_arg,
        image_quality_arg,
        object_tracking_node
    ])