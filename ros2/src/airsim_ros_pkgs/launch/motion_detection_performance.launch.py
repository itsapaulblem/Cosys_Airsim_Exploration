#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for motion detection with optimized performance settings
    Reduces lag in rqt_image_view by limiting FPS and resolution
    """
    
    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='Drone1',
        description='Name of the vehicle to use for motion detection'
    )
    
    # Performance optimized motion detection node
    motion_detection_node = Node(
        package='airsim_ros_pkgs',
        executable='motion_detection_node.py',
        name='multi_camera_motion_detection_node',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'enable_following': True,
            'enable_vis': True,
            'num_cameras': 4,
            
            # Performance optimization parameters
            'enable_all_camera_viz': False,  # Only primary camera visualization
            'viz_resolution_scale': 0.5,     # Half resolution for visualization 
            'detection_fps_limit': 10.0,     # Limit detection to 10 FPS
            'viz_fps_limit': 5.0,            # Limit visualization to 5 FPS
            'skip_secondary_frames': 2,      # Skip every other frame for secondary cameras
            
            # Reduced image processing parameters
            'image_width': 320,              # Reduced from 640 for performance
            'image_height': 240,             # Reduced from 480 for performance
            'image_quality': 60,             # Reduced from 95 for performance
            'enable_image_resize': True,
            
            # Detection parameters
            'confidence_threshold': 0.3,     # Slightly higher for stability
            'iou_threshold': 0.45,
            'motion_threshold': 15.0,
            
            # Following parameters
            'follow_distance': 0.5,
            'max_follow_speed': 3.0,         # Reduced for smoother following
            'follow_height': 3.0,
            'takeoff_height': 3.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2
    )
    
    return LaunchDescription([
        vehicle_name_arg,
        motion_detection_node
    ])