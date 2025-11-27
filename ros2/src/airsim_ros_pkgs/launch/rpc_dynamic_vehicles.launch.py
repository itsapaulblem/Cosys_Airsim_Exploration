#!/usr/bin/env python3
"""
RPC-Based Dynamic AirSim Vehicle Discovery Launch File
Queries running AirSim server via RPC to discover active vehicles and create namespaced nodes
"""

import os
import sys
import json
import time
import platform
from pathlib import Path
from typing import List, Dict, Any, Optional

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.logging import get_logger

# Add AirSim Python client to path
def add_airsim_to_path():
    """Add AirSim Python client to Python path"""
    possible_paths = [
        "/airsim_ros2_ws/PythonClient",  # Docker environment
        Path(__file__).parent.parent.parent.parent.parent / "PythonClient",  # Local development
        Path.home() / "AirSim" / "PythonClient",  # Standard AirSim installation
    ]
    
    for path in possible_paths:
        if Path(path).exists():
            sys.path.insert(0, str(path))
            get_logger("rpc_dynamic").info(f"Added AirSim Python path: {path}")
            return True
    
    get_logger("rpc_dynamic").warn("Could not find AirSim Python client in common locations")
    return False


def discover_vehicles_via_rpc(host_ip: str, host_port: int, timeout_sec: float = 10.0) -> List[Dict[str, Any]]:
    """
    Discover vehicles by querying running AirSim server via RPC
    Supports both Python client and direct RPC approaches
    
    Args:
        host_ip: AirSim server IP address
        host_port: AirSim server port
        timeout_sec: Connection timeout in seconds
        
    Returns:
        List of vehicle configurations discovered via RPC
    """
    # Try Method 1: Direct RPC (no Python client needed)
    vehicles = discover_vehicles_direct_rpc(host_ip, host_port, timeout_sec)
    if vehicles:
        return vehicles
    
    # Try Method 2: cosysairsim Python client
    try:
        import cosysairsim as airsim
        get_logger("rpc_dynamic").info(f"Using cosysairsim client to connect to {host_ip}:{host_port}...")
        return discover_with_python_client(airsim, host_ip, host_port, timeout_sec, "cosysairsim")
    except ImportError:
        get_logger("rpc_dynamic").warn("cosysairsim Python client not available")
    
    # Try Method 3: Standard airsim Python client  
    try:
        import airsim
        get_logger("rpc_dynamic").info(f"Using airsim client to connect to {host_ip}:{host_port}...")
        return discover_with_python_client(airsim, host_ip, host_port, timeout_sec, "airsim")
    except ImportError:
        get_logger("rpc_dynamic").warn("airsim Python client not available")
    
    # All methods failed
    get_logger("rpc_dynamic").warn("All RPC methods failed, using fallback")
    raise Exception("No RPC method available")


def discover_vehicles_direct_rpc(host_ip: str, host_port: int, timeout_sec: float) -> List[Dict[str, Any]]:
    """
    Direct RPC communication without Python client dependencies
    Uses raw msgpack-rpc over TCP
    """
    try:
        import socket
        import msgpack
        
        get_logger("rpc_dynamic").info(f"Attempting direct RPC to {host_ip}:{host_port} (timeout: {timeout_sec}s)...")
        
        # Connect via TCP with timeout
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout_sec)
        sock.connect((host_ip, host_port))
        
        # Query 1: Get vehicle list
        try:
            vehicles = direct_rpc_call(sock, "listVehicles", [])
            get_logger("rpc_dynamic").info(f"Direct RPC discovered vehicles: {vehicles}")
        except Exception as e:
            sock.close()
            raise Exception(f"Failed to get vehicle list: {e}")
        
        sock.close()
        
        # Parse vehicle configurations (skip settings for now - assume multirotor)
        vehicle_configs = []
        for vehicle_name in vehicles:
            vehicle_configs.append({
                "name": vehicle_name,
                "type": "multirotor",  # Default to multirotor
                "active": True,
                "source": "direct_rpc"
            })
            
            get_logger("rpc_dynamic").info(f"Direct RPC discovered: {vehicle_name} (type: multirotor)")
        
        return vehicle_configs
        
    except ImportError as e:
        get_logger("rpc_dynamic").warn(f"Direct RPC failed - missing dependency: {e}. Install with: pip install msgpack")
        return []
    except Exception as e:
        get_logger("rpc_dynamic").warn(f"Direct RPC failed: {e}")
        return []


def direct_rpc_call(sock, method: str, params: list):
    """Make a direct msgpack-rpc call"""
    import msgpack
    
    call_id = 1
    # msgpack-rpc format: [type, msgid, method, params]
    request = msgpack.packb([0, call_id, method, params])
    
    sock.send(request)
    
    # Receive response
    response = sock.recv(8192)  # Increase buffer for settings
    unpacked = msgpack.unpackb(response, raw=False)
    
    # Response format: [type, msgid, error, result]
    if len(unpacked) >= 4 and unpacked[2] is None:  # No error
        return unpacked[3]  # Return result
    else:
        raise Exception(f"RPC error: {unpacked[2] if len(unpacked) > 2 else 'Unknown error'}")


def discover_with_python_client(airsim_module, host_ip: str, host_port: int, timeout_sec: float, client_type: str) -> List[Dict[str, Any]]:
    """
    Discover vehicles using Python client (airsim or cosysairsim)
    """
    try:
        # Create client (timeout_sec not supported in all versions)
        client = airsim_module.MultirotorClient(ip=host_ip, port=host_port)
        client.confirmConnection()
        
        # Get live vehicle list
        vehicle_names = client.listVehicles()
        get_logger("rpc_dynamic").info(f"{client_type} discovered vehicles: {vehicle_names}")
        
        # Get settings for type detection
        settings_json = {}
        try:
            settings_string = client.getSettingsString()
            if settings_string:
                settings_json = json.loads(settings_string)
        except Exception as e:
            get_logger("rpc_dynamic").warn(f"Could not get settings: {e}")
        
        # Parse configurations
        vehicles = []
        for vehicle_name in vehicle_names:
            vehicle_type = detect_vehicle_type_from_settings(vehicle_name, settings_json)
            
            vehicles.append({
                "name": vehicle_name,
                "type": vehicle_type,
                "active": True,
                "source": client_type
            })
        
        return vehicles
        
    except Exception as e:
        get_logger("rpc_dynamic").error(f"{client_type} discovery failed: {e}")
        raise


def detect_vehicle_type_from_settings(vehicle_name: str, settings_json: Dict) -> str:
    """
    Detect vehicle type from settings JSON (no client needed)
    """
    # Check settings.json from server
    if "Vehicles" in settings_json and vehicle_name in settings_json["Vehicles"]:
        vehicle_config = settings_json["Vehicles"][vehicle_name]
        airsim_type = vehicle_config.get("VehicleType", "").lower()
        
        if any(x in airsim_type for x in ["multirotor", "px4", "arducopter", "simpleflight"]):
            return "multirotor"
        elif any(x in airsim_type for x in ["car", "physx", "rover"]):
            return "car"
        elif "computervision" in airsim_type:
            return "computervision"
    
    # Default assumption for unknown vehicles
    get_logger("rpc_dynamic").info(f"Could not determine type for {vehicle_name}, assuming multirotor")
    return "multirotor"


def detect_vehicle_type(client, vehicle_name: str, settings_json: Dict) -> str:
    """
    Detect vehicle type using multiple methods
    
    Args:
        client: AirSim client instance  
        vehicle_name: Name of vehicle to detect type for
        settings_json: Parsed settings from AirSim server
        
    Returns:
        Vehicle type string ("multirotor", "car", or "computervision")
    """
    # Method 1: Check settings.json from server
    if "Vehicles" in settings_json and vehicle_name in settings_json["Vehicles"]:
        vehicle_config = settings_json["Vehicles"][vehicle_name]
        airsim_type = vehicle_config.get("VehicleType", "").lower()
        
        if any(x in airsim_type for x in ["multirotor", "px4", "arducopter", "simpleflight"]):
            return "multirotor"
        elif any(x in airsim_type for x in ["car", "physx", "rover"]):
            return "car"
        elif "computervision" in airsim_type:
            return "computervision"
    
    # Method 2: Try RPC API calls to detect type
    try:
        # Test multirotor-specific API
        state = client.getMultirotorState(vehicle_name)
        if state:
            return "multirotor"
    except:
        pass
    
    try:
        # Test car-specific API
        state = client.getCarState(vehicle_name)  
        if state:
            return "car"
    except:
        pass
    
    # Method 3: Default assumption
    get_logger("rpc_dynamic").info(f"Could not determine type for {vehicle_name}, assuming multirotor")
    return "multirotor"


def fallback_vehicle_discovery(settings_file: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Fallback vehicle discovery when RPC fails
    
    Args:
        settings_file: Optional path to settings.json file
        
    Returns:
        List of vehicle configurations from fallback methods
    """
    get_logger("rpc_dynamic").warn("Using fallback vehicle discovery")
    
    # Try to parse settings file if provided
    if settings_file and os.path.exists(settings_file):
        try:
            with open(settings_file, 'r') as f:
                settings = json.load(f)
            
            if "Vehicles" in settings:
                vehicles = []
                for name, config in settings["Vehicles"].items():
                    vehicle_type = "multirotor"
                    airsim_type = config.get("VehicleType", "").lower()
                    
                    if any(x in airsim_type for x in ["car", "physx", "rover"]):
                        vehicle_type = "car"
                    elif "computervision" in airsim_type:
                        vehicle_type = "computervision"
                    
                    vehicles.append({
                        "name": name,
                        "type": vehicle_type,
                        "active": False,  # Unknown if active
                        "source": "settings_file"
                    })
                
                get_logger("rpc_dynamic").info(f"Fallback found {len(vehicles)} vehicles in settings file")
                return vehicles
        except Exception as e:
            get_logger("rpc_dynamic").warn(f"Could not parse settings file: {e}")
    
    # Ultimate fallback: default vehicle
    get_logger("rpc_dynamic").warn("Using default vehicle configuration")
    return [{
        "name": "DefaultDrone",
        "type": "multirotor",
        "active": False,
        "source": "default"
    }]


def create_vehicle_node(vehicle_info: Dict[str, Any], host_ip: str, host_port: int, 
                       mission_mode: bool = False, mission_parameters: Dict[str, Any] = None) -> Node:
    """
    Create a namespaced ROS2 node for a vehicle discovered via RPC
    Enhanced with mission mode support
    """
    vehicle_name = vehicle_info["name"]
    vehicle_type = vehicle_info["type"]
    
    if mission_parameters is None:
        mission_parameters = {}
    
    # Choose appropriate executable based on vehicle type and mission mode
    if mission_mode:
        executable_map = {
            "multirotor": "mission_multirotor_node",  # Mission-capable node with action servers
            "car": "simple_multirotor_node",  # TODO: Add dedicated mission car node
            "computervision": "simple_multirotor_node"  # TODO: Add dedicated mission CV node
        }
    else:
        executable_map = {
            "multirotor": "multirotor_node",  # Standard node for sensors and basic flight
            "car": "simple_multirotor_node",  # TODO: Add dedicated car node
            "computervision": "simple_multirotor_node"  # TODO: Add dedicated CV node
        }

    
    executable = executable_map.get(vehicle_type, "multirotor_node")

    # Base parameters (with performance tuning)
    node_parameters = {
        'host_ip': host_ip,
        'host_port': int(host_port),
        'vehicle_name': vehicle_name,
        'image_timer_freq': 0.033,  # 30Hz camera rate (diagnostic: increased from 10Hz default)
    }
    
    # Add mission parameters if in mission mode
    if mission_mode:
        node_parameters.update({
            'mission_mode': True,
            'search_altitude': mission_parameters.get('search_altitude', 25.0),
            'search_speed': mission_parameters.get('search_speed', 5.0),
            'pattern_spacing': mission_parameters.get('pattern_spacing', 15.0),
            'detection_threshold': mission_parameters.get('detection_threshold', 0.7),
            'mission_timeout_seconds': mission_parameters.get('mission_timeout', 1800),
            'enable_coordination': mission_parameters.get('enable_coordination', True)
        })
    
    # Create appropriate remappings for mission mode
    remappings = []
    if mission_mode:
        remappings = [
            ('mission/status', f'{vehicle_name}/mission/status'),
            ('detections/target', f'{vehicle_name}/detections/target'),
            ('coordination/status', f'{vehicle_name}/coordination/status')
        ]

    # NOTE: This duplicated node_parameters block will be removed in cleanup
    # Currently keeping for compatibility but first block at line 339 takes precedence

    # Add mission parameters if in mission mode
    if mission_mode:
        node_parameters.update({
            'mission_mode': True,
            'search_altitude': mission_parameters.get('search_altitude', 25.0),
            'search_speed': mission_parameters.get('search_speed', 5.0),
            'pattern_spacing': mission_parameters.get('pattern_spacing', 15.0),
            'detection_threshold': mission_parameters.get('detection_threshold', 0.7),
            'mission_timeout_seconds': mission_parameters.get('mission_timeout', 1800),
            'enable_coordination': mission_parameters.get('enable_coordination', True)
        })
    
    # Create appropriate remappings for mission mode
    remappings = []
    if mission_mode:
        remappings = [
            ('mission/status', f'{vehicle_name}/mission/status'),
            ('detections/target', f'{vehicle_name}/detections/target'),
            ('coordination/status', f'{vehicle_name}/coordination/status')
        ]
    
    return Node(
        package='airsim_ros_pkgs',
        executable=executable,
        name=vehicle_name,  # Node name IS vehicle name: /VehicleName
        namespace='',  # No namespace - topics prefixed in C++ code
        output='screen',
        parameters=[node_parameters],
        remappings=remappings,
        # Arguments for debugging
        arguments=['--ros-args', '--log-level', 'INFO']
    )


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction to dynamically create nodes based on RPC vehicle discovery
    Enhanced with mission mode support
    """
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip').perform(context)
    host_port = LaunchConfiguration('host_port').perform(context)
    enable_coordination = LaunchConfiguration('enable_coordination').perform(context)
    fallback_settings = LaunchConfiguration('fallback_settings').perform(context)
    rpc_timeout = float(LaunchConfiguration('rpc_timeout').perform(context))
    debug = LaunchConfiguration('debug').perform(context).lower() in ['true', '1', 'yes']
    enable_localization = LaunchConfiguration('enable_localization').perform(context).lower() in ['true', '1', 'yes']
    
    # Mission mode configuration
    mission_mode = LaunchConfiguration('mission_mode').perform(context).lower() in ['true', '1', 'yes']
    
    # Mission parameters
    mission_parameters = {}
    if mission_mode:
        mission_parameters = {
            'search_altitude': float(LaunchConfiguration('search_altitude').perform(context)),
            'search_speed': float(LaunchConfiguration('search_speed').perform(context)),
            'pattern_spacing': float(LaunchConfiguration('pattern_spacing').perform(context)),
            'detection_threshold': float(LaunchConfiguration('detection_threshold').perform(context)),
            'mission_timeout': float(LaunchConfiguration('mission_timeout').perform(context)),
            'enable_coordination': enable_coordination.lower() in ['true', '1', 'yes']
        }
    
    # Mission mode configuration
    mission_mode = LaunchConfiguration('mission_mode').perform(context).lower() in ['true', '1', 'yes']
    
    # Mission parameters
    mission_parameters = {}
    if mission_mode:
        mission_parameters = {
            'search_altitude': float(LaunchConfiguration('search_altitude').perform(context)),
            'search_speed': float(LaunchConfiguration('search_speed').perform(context)),
            'pattern_spacing': float(LaunchConfiguration('pattern_spacing').perform(context)),
            'detection_threshold': float(LaunchConfiguration('detection_threshold').perform(context)),
            'mission_timeout': float(LaunchConfiguration('mission_timeout').perform(context)),
            'enable_coordination': enable_coordination.lower() in ['true', '1', 'yes']
        }
    
    # Debug mode - just log additional info
    if debug:
        get_logger("rpc_dynamic").info("Debug mode enabled")
    
    # Add AirSim Python client to path (optional - we can use direct RPC)
    airsim_available = add_airsim_to_path()
    if not airsim_available:
        get_logger("rpc_dynamic").info("AirSim Python client not found, but direct RPC is available")
    
    # Primary: RPC-based vehicle discovery (try all methods)
    vehicles = []
    try:
        vehicles = discover_vehicles_via_rpc(host_ip, int(host_port), rpc_timeout)
        get_logger("rpc_dynamic").info(f"RPC discovery successful: {len(vehicles)} vehicles")
        
    except Exception as e:
        get_logger("rpc_dynamic").warn(f"RPC discovery failed: {e}")
        vehicles = []  # Clear any partial results
    
    if not vehicles:
        # Fallback: Use settings file or defaults
        fallback_file = fallback_settings if fallback_settings != "none" else None
        vehicles = fallback_vehicle_discovery(fallback_file)
        get_logger("rpc_dynamic").info(f"Fallback discovery: {len(vehicles)} vehicles")
    
    # Create nodes list
    nodes = []
    
    # Add coordination node if enabled
    if enable_coordination.lower() in ['true', '1', 'yes']:
        # Choose coordination node based on mission mode
        if mission_mode:
            coordination_executable = 'mission_coordination_node'
            coordination_name = 'mission_coordinator'
        else:
            coordination_executable = 'coordination_node'
            coordination_name = 'airsim_coordination_node'
            
        coordination_node = Node(
            package='airsim_ros_pkgs',
            executable=coordination_executable,
            name=coordination_name,
            namespace='',  # Global namespace
            output='screen',
            parameters=[{
                'host_ip': host_ip,
                'host_port': int(host_port),
                'vehicle_names': [v["name"] for v in vehicles],  # Pass discovered vehicle names
                'mission_mode': mission_mode,
                **mission_parameters  # Add mission parameters if in mission mode
            }]
        )
        nodes.append(coordination_node)
        
        coordination_type = "Mission Coordination" if mission_mode else "Basic Coordination"
        get_logger("rpc_dynamic").info(f"Added {coordination_type} node: {coordination_name}")
    
    # Create vehicle nodes
    for vehicle_info in vehicles:
        vehicle_node = create_vehicle_node(vehicle_info, host_ip, host_port, mission_mode, mission_parameters)
        nodes.append(vehicle_node)
        
        # REP 105 COMPLIANCE: Add localization node for each vehicle (if enabled)
        # This ensures proper map→odom transform authority per REP 105 specification
        if enable_localization:
            localization_node = Node(
                package='airsim_ros_pkgs',
                executable='localization_node',
                name=f'localization_{vehicle_info["name"]}',
                namespace='',  # No namespace - follows vehicle naming convention
                output='screen',
                parameters=[{
                    'vehicle_name': vehicle_info['name'],
                    'host_ip': host_ip,
                    'host_port': int(host_port)
                }],
                arguments=['--ros-args', '--log-level', 'INFO']
            )
            nodes.append(localization_node)
        
        source_info = f" (source: {vehicle_info.get('source', 'unknown')})"
        mode_info = " [MISSION MODE]" if mission_mode else " [STANDARD MODE]"
        get_logger("rpc_dynamic").info(
            f"Creating node for vehicle '{vehicle_info['name']}' "
            f"of type '{vehicle_info['type']}'{source_info}{mode_info}"
        )

        if enable_localization:
            get_logger("rpc_dynamic").info(
                f"REP 105 Localization node created: localization_{vehicle_info['name']} "
                f"(publishes map→{vehicle_info['name']}/odom)"
            )

        
    # Log mission mode status
    if mission_mode:
        get_logger("rpc_dynamic").info("🚁 MISSION MODE ENABLED")
        get_logger("rpc_dynamic").info(f"Mission Parameters: altitude={mission_parameters.get('search_altitude')}m, "
                                     f"speed={mission_parameters.get('search_speed')}m/s, "
                                     f"spacing={mission_parameters.get('pattern_spacing')}m")
    else:
        get_logger("rpc_dynamic").info("📡 STANDARD MODE - sensors and basic flight only")
    
    # Log REP 105 compliance status
    if enable_localization:
        get_logger("rpc_dynamic").info("REP 105 COMPLIANCE: Localization nodes enabled for proper frame authorities")
        get_logger("rpc_dynamic").info("Transform Chain: map → {vehicle}/odom → {vehicle}/base_link1 → sensors")
    else:
        get_logger("rpc_dynamic").warn("REP 105 WARNING: Localization disabled - map→odom transforms missing!")
        get_logger("rpc_dynamic").warn("Consider enabling localization for complete transform chain")

    # ===================
    # OCTOMAP 3D MAPPING
    # ===================
    enable_octomap = LaunchConfiguration('enable_octomap').perform(context).lower() in ['true', '1', 'yes']
    enable_octomap_rviz = LaunchConfiguration('enable_octomap_rviz').perform(context).lower() in ['true', '1', 'yes']

    if enable_octomap and len(vehicles) > 0:
        get_logger("rpc_dynamic").info(f"🗺️  OCTOMAP 3D MAPPING ENABLED ({len(vehicles)} drones)")

        # Configuration files
        octomap_params_file = PathJoinSubstitution([
            FindPackageShare('airsim_ros_pkgs'),
            'config',
            'octomap_params.yaml'
        ])

        # Point Cloud Merger (DYNAMIC drone count)
        merger_node = Node(
            package='airsim_ros_pkgs',
            executable='pointcloud_merger_node',
            name='pointcloud_merger',
            namespace='',
            output='screen',
            parameters=[{
                'num_drones': len(vehicles),  # Auto-configures for discovered count
                'merge_rate_hz': 10.0,
                'enable_downsampling': False,  # Disabled: allows unlimited range, no spatial limits
                'voxel_leaf_size': float(LaunchConfiguration('voxel_leaf_size').perform(context))
            }]
        )
        nodes.append(merger_node)
        get_logger("rpc_dynamic").info(f"  - Point Cloud Merger: {len(vehicles)} drone LiDAR streams → /merged_pointcloud")

        # OctoMap Server
        octomap_node = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            namespace='',
            output='screen',
            parameters=[
                str(octomap_params_file.perform(context)),
                {
                    'resolution': float(LaunchConfiguration('octomap_resolution').perform(context))
                }
            ],
            remappings=[
                ('cloud_in', '/merged_pointcloud'),
                ('octomap_binary', '/octomap_binary'),
                ('octomap_full', '/octomap_full')
            ]
        )
        nodes.append(octomap_node)
        get_logger("rpc_dynamic").info(f"  - OctoMap Server: resolution={LaunchConfiguration('octomap_resolution').perform(context)}m")

        # Optional RViz (with LD_PRELOAD workaround for octomap_rviz_plugins bug)
        if enable_octomap_rviz:
            rviz_config_file = PathJoinSubstitution([
                FindPackageShare('airsim_ros_pkgs'),
                'rviz',
                'lidar_visualization.rviz'
            ])

            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace='',
                output='screen',
                arguments=['-d', str(rviz_config_file.perform(context))],
                # Workaround for octomap_rviz_plugins symbol resolution bug in ROS2 Humble
                additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'}
            )
            nodes.append(rviz_node)
            get_logger("rpc_dynamic").info("  - RViz2: OctoMap visualization enabled")

        get_logger("rpc_dynamic").info("OctoMap topics: /merged_pointcloud, /octomap_binary, /octomap_full")

    return nodes


def generate_launch_description():
    """
    Generate launch description with RPC-based dynamic vehicle discovery
    Enhanced with mission mode support
    """
    # === CORE LAUNCH ARGUMENTS ===
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value=os.getenv('AIRSIM_HOST_IP', 'host.docker.internal'),  # Docker-friendly default
        description='IP address of AirSim server'
    )
    
    host_port_arg = DeclareLaunchArgument(
        'host_port',
        default_value='41451',
        description='Port of AirSim server'
    )
    
    enable_coordination_arg = DeclareLaunchArgument(
        'enable_coordination',
        default_value='true',
        description='Enable global coordination node'
    )
    
    fallback_settings_arg = DeclareLaunchArgument(
        'fallback_settings',
        default_value='none',
        description='Fallback settings.json file path (none = use defaults)'
    )
    
    rpc_timeout_arg = DeclareLaunchArgument(
        'rpc_timeout',
        default_value='10.0',
        description='RPC connection timeout in seconds'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    

    enable_localization_arg = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        choices=['true', 'false'],
        description='Enable REP 105 compliant localization nodes (recommended for proper transform chain)'
    )
    
    # === MISSION MODE ARGUMENTS ===
    mission_mode_arg = DeclareLaunchArgument(
        'mission_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable mission mode (action servers, services, coordination)'
    )
    
    search_altitude_arg = DeclareLaunchArgument(
        'search_altitude',
        default_value='25.0',
        description='Default search altitude in meters (mission mode)'
    )
    
    search_speed_arg = DeclareLaunchArgument(
        'search_speed',
        default_value='5.0',
        description='Default search speed in m/s (mission mode)'
    )
    
    pattern_spacing_arg = DeclareLaunchArgument(
        'pattern_spacing',
        default_value='15.0',
        description='Default pattern spacing in meters (mission mode)'
    )
    
    detection_threshold_arg = DeclareLaunchArgument(
        'detection_threshold',
        default_value='0.7',
        description='Target detection confidence threshold 0.0-1.0 (mission mode)'
    )
    
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='1800.0',
        description='Mission timeout in seconds (mission mode)'
    )

    # === OCTOMAP 3D MAPPING ARGUMENTS ===
    enable_octomap_arg = DeclareLaunchArgument(
        'enable_octomap',
        default_value='false',
        choices=['true', 'false'],
        description='Enable OctoMap 3D mapping with automatic multi-drone LiDAR merging'
    )

    octomap_resolution_arg = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.1',
        description='OctoMap voxel resolution in meters (0.05=detailed, 0.1=balanced, 0.2=fast)'
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.5',
        description='VoxelGrid downsampling leaf size in meters (0.5m allows 500m span, 0.2m allows 200m span)'
    )

    enable_octomap_rviz_arg = DeclareLaunchArgument(
        'enable_octomap_rviz',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RViz with OctoMap visualization (requires X11 or VNC)'
    )

    # Opaque function for dynamic node creation
    dynamic_nodes = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        # Core arguments
        host_ip_arg,
        host_port_arg,
        enable_coordination_arg,
        fallback_settings_arg,
        rpc_timeout_arg,
        debug_arg,
        enable_localization_arg,


        # Mission mode arguments
        mission_mode_arg,
        search_altitude_arg,
        search_speed_arg,
        pattern_spacing_arg,
        detection_threshold_arg,
        mission_timeout_arg,

        # OctoMap 3D mapping arguments
        enable_octomap_arg,
        octomap_resolution_arg,
        voxel_leaf_size_arg,
        enable_octomap_rviz_arg,

        # Dynamic node creation
        dynamic_nodes
    ])