import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def load_airsim_settings(settings_file_path):
    """Load and parse AirSim settings.json file"""
    vehicles = {}
    
    if settings_file_path and os.path.exists(settings_file_path):
        try:
            with open(settings_file_path, 'r') as f:
                settings = json.load(f)
                vehicles = settings.get('Vehicles', {})
                print(f"[INFO] Loaded {len(vehicles)} vehicles from {settings_file_path}")
        except Exception as e:
            print(f"[WARNING] Could not load settings file {settings_file_path}: {e}")
    
    # If no vehicles found, create robust default configuration
    if not vehicles:
        vehicles = {
            "Drone1": {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True,
                "X": 0, "Y": 0, "Z": -2,
                "Yaw": 0
            },
            "Drone2": {
                "VehicleType": "SimpleFlight", 
                "AutoCreate": True,
                "X": 5, "Y": 0, "Z": -2,
                "Yaw": 0
            }
        }
        print("[INFO] Using default 2-drone configuration")
    
    return vehicles

def create_vehicle_node(vehicle_name, vehicle_config, global_params):
    """Create a vehicle node based on configuration with robust error handling"""
    
    # Determine executable based on vehicle type
    vehicle_type = vehicle_config.get('VehicleType', 'SimpleFlight')
    
    if vehicle_type in ['SimpleFlight', 'PX4Multirotor']:
        executable = 'multirotor_node'
    else:
        # Default to multirotor for unknown types
        executable = 'multirotor_node'
        print(f"[WARNING] Unknown vehicle type {vehicle_type}, defaulting to multirotor_node")
    
    # Vehicle-specific parameters with robust defaults
    vehicle_params = {
        'vehicle_name': vehicle_name,
        'host_ip': global_params['host_ip'],
        'host_port': global_params['host_port'],
        'world_frame_id': global_params['world_frame_id'],
        'odom_frame_id': f'{vehicle_name.lower()}_odom_local_ned',
        'base_frame_id': f'{vehicle_name.lower()}_base_link',
    }
    
    # Add position if specified
    if 'X' in vehicle_config:
        vehicle_params['initial_x'] = float(vehicle_config['X'])
        vehicle_params['initial_y'] = float(vehicle_config['Y'])  
        vehicle_params['initial_z'] = float(vehicle_config['Z'])
    
    if 'Yaw' in vehicle_config:
        vehicle_params['initial_yaw'] = float(vehicle_config['Yaw'])
    
    # Timer frequencies optimized for multi-vehicle performance
    vehicle_params.update({
        'state_timer_freq': 0.02,     # 50Hz - Reduced for multi-vehicle
        'image_timer_freq': 0.1,      # 10Hz - Conservative for bandwidth
        'lidar_timer_freq': 0.05,     # 20Hz - Balanced performance
        'gpulidar_timer_freq': 0.05,  # 20Hz
        'echo_timer_freq': 0.1,       # 10Hz
    })
    
    # Create node with proper namespace isolation
    return GroupAction([
        PushRosNamespace(vehicle_name.lower()),
        Node(
            package='airsim_ros_pkgs',
            executable=executable,
            name=f'airsim_{vehicle_name.lower()}',
            output='screen',
            parameters=[vehicle_params],
            arguments=[vehicle_name],
            # Multi-threaded execution for better performance
            prefix=f'nice -n 0',  # Normal priority
            respawn=True,          # Auto-restart on failure
            respawn_delay=2.0,     # Wait 2 seconds before restart
        )
    ])

def generate_vehicle_nodes_from_settings(context, *args, **kwargs):
    """Generate vehicle nodes based on AirSim settings with comprehensive error handling"""
    
    # Get launch configuration values
    settings_file = LaunchConfiguration('settings_file').perform(context)
    host_ip = LaunchConfiguration('host_ip').perform(context)
    host_port = LaunchConfiguration('host_port').perform(context)
    world_frame_id = LaunchConfiguration('world_frame_id').perform(context)
    max_vehicles = int(LaunchConfiguration('max_vehicles').perform(context))
    
    # Global parameters shared across all vehicles
    global_params = {
        'host_ip': host_ip,
        'host_port': int(host_port),
        'world_frame_id': world_frame_id,
    }
    
    # Load vehicle configurations
    vehicles = load_airsim_settings(settings_file)
    
    # Limit number of vehicles for performance
    if len(vehicles) > max_vehicles:
        print(f"[WARNING] Limiting vehicles to {max_vehicles} (found {len(vehicles)})")
        vehicles = dict(list(vehicles.items())[:max_vehicles])
    
    nodes = []
    
    # Create vehicle nodes with error handling
    for vehicle_name, vehicle_config in vehicles.items():
        try:
            node_group = create_vehicle_node(vehicle_name, vehicle_config, global_params)
            nodes.append(node_group)
            print(f"[INFO] Created node for {vehicle_name}")
        except Exception as e:
            print(f"[ERROR] Failed to create node for {vehicle_name}: {e}")
            continue
    
    # Add coordination services
    coordination_node = Node(
        package='airsim_ros_pkgs',
        executable='coordination_node',
        name='airsim_coordination',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': int(host_port),
            'world_frame_id': world_frame_id,
            'vehicle_names': list(vehicles.keys()),
        }],
        respawn=True,
        respawn_delay=5.0,  # Longer delay for coordination node
    )
    nodes.append(coordination_node)
    
    print(f"[INFO] Launch system ready with {len(vehicles)} vehicles")
    return nodes

def generate_launch_description():
    """Generate the complete launch description for production use"""
    
    # Default AirSim settings path
    default_settings = '/mnt/c/Users/UserAdmin/Documents/AirSim/settings.json'

    return LaunchDescription([
        # Launch arguments with robust defaults
        DeclareLaunchArgument(
            'settings_file',
            default_value=default_settings,
            description='Path to AirSim settings.json file'
        ),
        DeclareLaunchArgument(
            'host_ip',
            default_value='localhost',
            description='AirSim host IP address'
        ),
        DeclareLaunchArgument(
            'host_port', 
            default_value='41451',
            description='AirSim host port'
        ),
        DeclareLaunchArgument(
            'world_frame_id',
            default_value='world_ned',
            description='World frame ID for TF tree'
        ),
        DeclareLaunchArgument(
            'max_vehicles',
            default_value='8',
            description='Maximum number of vehicles to launch'
        ),
        
        # Generate nodes dynamically
        OpaqueFunction(function=generate_vehicle_nodes_from_settings)
    ])