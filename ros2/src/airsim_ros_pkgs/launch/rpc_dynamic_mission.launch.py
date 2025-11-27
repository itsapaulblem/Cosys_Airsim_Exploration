#!/usr/bin/env python3
"""
RPC-Based Dynamic Mission Vehicle Discovery Launch File
Combines dynamic vehicle discovery via RPC with mission coordination capabilities

This launch file:
1. Queries running AirSim server via RPC to discover active vehicles
2. Creates mission-capable nodes for each discovered vehicle
3. Launches mission coordination node for orchestration
4. Optionally includes demonstration client

Usage:
    ros2 launch airsim_ros_pkgs rpc_dynamic_mission.launch.py
    ros2 launch airsim_ros_pkgs rpc_dynamic_mission.launch.py enable_demo:=true
    ros2 launch airsim_ros_pkgs rpc_dynamic_mission.launch.py host_ip:=192.168.1.100
"""

import os
import sys
import json
import time
import platform
from pathlib import Path
from typing import List, Dict, Any, Optional

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.logging import get_logger

# Add AirSim Python client to path
def add_airsim_to_path():
    """Add AirSim Python client to Python path"""
    possible_paths = [
        "/airsim_ros2_ws/PythonClientColosseum",  # Docker environment
        Path(__file__).parent.parent.parent.parent.parent / "PythonClientColosseum",  # Local development
        Path.home() / "AirSim" / "PythonClient",  # Standard AirSim installation
    ]
    
    for path in possible_paths:
        if Path(path).exists():
            sys.path.insert(0, str(path))
            get_logger("rpc_dynamic_mission").info(f"Added AirSim Python path: {path}")
            return True
    
    get_logger("rpc_dynamic_mission").warn("Could not find AirSim Python client in common locations")
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
        get_logger("rpc_dynamic_mission").info(f"Using cosysairsim client to connect to {host_ip}:{host_port}...")
        return discover_with_python_client(airsim, host_ip, host_port, timeout_sec, "cosysairsim")
    except ImportError:
        pass
    
    # Try Method 3: Standard airsim Python client  
    try:
        import airsim
        get_logger("rpc_dynamic_mission").info(f"Using airsim client to connect to {host_ip}:{host_port}...")
        return discover_with_python_client(airsim, host_ip, host_port, timeout_sec, "airsim")
    except ImportError:
        pass
    
    # All methods failed
    get_logger("rpc_dynamic_mission").warn("All RPC methods failed, using fallback")
    raise Exception("No RPC method available")


def discover_vehicles_direct_rpc(host_ip: str, host_port: int, timeout_sec: float) -> List[Dict[str, Any]]:
    """
    Direct RPC communication without Python client dependencies
    Uses raw msgpack-rpc over TCP
    """
    try:
        import socket
        import msgpack
        
        get_logger("rpc_dynamic_mission").info(f"Attempting direct RPC to {host_ip}:{host_port}...")
        
        # Connect via TCP with timeout
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout_sec)
        sock.connect((host_ip, host_port))
        
        # Query 1: Get vehicle list
        try:
            vehicles = direct_rpc_call(sock, "listVehicles", [])
            get_logger("rpc_dynamic_mission").info(f"Direct RPC discovered vehicles: {vehicles}")
        except Exception as e:
            sock.close()
            raise Exception(f"Failed to get vehicle list: {e}")
        
        sock.close()
        
        # Parse vehicle configurations (assume multirotor for mission capability)
        vehicle_configs = []
        for vehicle_name in vehicles:
            vehicle_configs.append({
                "name": vehicle_name,
                "type": "multirotor",  # Default to multirotor for mission support
                "active": True,
                "source": "direct_rpc"
            })
            
            get_logger("rpc_dynamic_mission").info(f"Direct RPC discovered: {vehicle_name} (type: multirotor)")
        
        return vehicle_configs
        
    except Exception as e:
        get_logger("rpc_dynamic_mission").warn(f"Direct RPC failed: {e}")
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
        get_logger("rpc_dynamic_mission").info(f"{client_type} discovered vehicles: {vehicle_names}")
        
        # Get settings for type detection
        settings_json = {}
        try:
            settings_string = client.getSettingsString()
            if settings_string:
                settings_json = json.loads(settings_string)
        except Exception as e:
            get_logger("rpc_dynamic_mission").warn(f"Could not get settings: {e}")
        
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
        get_logger("rpc_dynamic_mission").error(f"{client_type} discovery failed: {e}")
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
    get_logger("rpc_dynamic_mission").info(f"Could not determine type for {vehicle_name}, assuming multirotor")
    return "multirotor"


def fallback_vehicle_discovery(settings_file: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Fallback vehicle discovery when RPC fails
    
    Args:
        settings_file: Optional path to settings.json file
        
    Returns:
        List of vehicle configurations from fallback methods
    """
    get_logger("rpc_dynamic_mission").warn("Using fallback vehicle discovery")
    
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
                
                get_logger("rpc_dynamic_mission").info(f"Fallback found {len(vehicles)} vehicles in settings file")
                return vehicles
        except Exception as e:
            get_logger("rpc_dynamic_mission").warn(f"Could not parse settings file: {e}")
    
    # Ultimate fallback: default vehicles for mission testing
    get_logger("rpc_dynamic_mission").warn("Using default mission vehicle configuration")
    return [
        {
            "name": "Droan1",
            "type": "multirotor",
            "active": False,
            "source": "default"
        },
        {
            "name": "PX4_Drone2",
            "type": "multirotor",
            "active": False,
            "source": "default"
        }
    ]


def create_vehicle_node(vehicle_info: Dict[str, Any], host_ip: str, host_port: int, enable_missions: bool) -> Node:
    """
    Create a namespaced ROS2 node for a vehicle discovered via RPC
    
    Args:
        vehicle_info: Vehicle information dict
        host_ip: AirSim server IP
        host_port: AirSim server port
        enable_missions: Whether to use mission-capable nodes
    """
    vehicle_name = vehicle_info["name"]
    vehicle_type = vehicle_info["type"]
    
    # Choose appropriate executable based on vehicle type and mission support
    if enable_missions and vehicle_type == "multirotor":
        # Use mission-capable node for multirotors when missions are enabled
        executable = "mission_multirotor_node"
        get_logger("rpc_dynamic_mission").info(
            f"Creating mission-capable node for {vehicle_name}"
        )
    else:
        # Use standard nodes when missions are disabled or for non-multirotor vehicles
        executable_map = {
            "multirotor": "multirotor_node",  # Full sensor node
            "car": "simple_multirotor_node",  # TODO: Add dedicated car node
            "computervision": "simple_multirotor_node"  # TODO: Add dedicated CV node
        }
        executable = executable_map.get(vehicle_type, "simple_multirotor_node")
        get_logger("rpc_dynamic_mission").info(
            f"Creating standard node for {vehicle_name} (type: {vehicle_type})"
        )
    
    return Node(
        package='airsim_ros_pkgs',
        executable=executable,
        name=vehicle_name,  # Node name IS vehicle name: /VehicleName
        namespace='',  # No namespace - topics prefixed in C++ code
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': int(host_port),
            'vehicle_name': vehicle_name,
        }],
        # Arguments for debugging
        arguments=['--ros-args', '--log-level', 'INFO']
    )


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction to dynamically create nodes based on RPC vehicle discovery
    """
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip').perform(context)
    host_port = LaunchConfiguration('host_port').perform(context)
    enable_missions = LaunchConfiguration('enable_missions').perform(context)
    enable_demo = LaunchConfiguration('enable_demo').perform(context)
    fallback_settings = LaunchConfiguration('fallback_settings').perform(context)
    rpc_timeout = float(LaunchConfiguration('rpc_timeout').perform(context))
    debug = LaunchConfiguration('debug').perform(context).lower() in ['true', '1', 'yes']
    
    # Convert string booleans
    enable_missions_bool = enable_missions.lower() in ['true', '1', 'yes']
    enable_demo_bool = enable_demo.lower() in ['true', '1', 'yes']
    
    # Debug mode - just log additional info
    if debug:
        get_logger("rpc_dynamic_mission").info("Debug mode enabled")
        get_logger("rpc_dynamic_mission").info(f"Mission mode: {enable_missions_bool}")
        get_logger("rpc_dynamic_mission").info(f"Demo client: {enable_demo_bool}")
    
    # Add AirSim Python client to path (optional - we can use direct RPC)
    airsim_available = add_airsim_to_path()
    if not airsim_available:
        get_logger("rpc_dynamic_mission").info("AirSim Python client not found, but direct RPC is available")
    
    # Primary: RPC-based vehicle discovery (try all methods)
    vehicles = []
    try:
        vehicles = discover_vehicles_via_rpc(host_ip, int(host_port), rpc_timeout)
        get_logger("rpc_dynamic_mission").info(f"RPC discovery successful: {len(vehicles)} vehicles")
        
    except Exception as e:
        get_logger("rpc_dynamic_mission").warn(f"RPC discovery failed: {e}")
        vehicles = []  # Clear any partial results
    
    if not vehicles:
        # Fallback: Use settings file or defaults
        fallback_file = fallback_settings if fallback_settings != "none" else None
        vehicles = fallback_vehicle_discovery(fallback_file)
        get_logger("rpc_dynamic_mission").info(f"Fallback discovery: {len(vehicles)} vehicles")
    
    # Create nodes list
    nodes = []
    
    # Add mission coordination node if missions are enabled
    if enable_missions_bool:
        get_logger("rpc_dynamic_mission").info("Adding mission coordination node")
        
        mission_coordination_node = Node(
            package='airsim_ros_pkgs',
            executable='mission_coordination_node',
            name='mission_coordinator',
            namespace='',  # Global namespace: /mission_coordinator
            output='screen',
            parameters=[{
                'host_ip': host_ip,
                'host_port': int(host_port),
                'vehicle_names': [v["name"] for v in vehicles]  # Pass discovered vehicle names
            }],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
        nodes.append(mission_coordination_node)
    else:
        # Add standard coordination node if missions are disabled
        coordination_node = Node(
            package='airsim_ros_pkgs',
            executable='coordination_node',
            name='airsim_coordination_node',
            namespace='',  # Global namespace: /airsim_coordination_node
            output='screen',
            parameters=[{
                'host_ip': host_ip,
                'host_port': int(host_port),
                'vehicle_names': [v["name"] for v in vehicles]  # Pass discovered vehicle names
            }]
        )
        nodes.append(coordination_node)
    
    # Create vehicle nodes
    for vehicle_info in vehicles:
        vehicle_node = create_vehicle_node(vehicle_info, host_ip, host_port, enable_missions_bool)
        nodes.append(vehicle_node)
        
        source_info = f" (source: {vehicle_info.get('source', 'unknown')})"
        capability_info = " [mission-capable]" if enable_missions_bool and vehicle_info['type'] == 'multirotor' else ""
        get_logger("rpc_dynamic_mission").info(
            f"Creating node for vehicle '{vehicle_info['name']}' "
            f"of type '{vehicle_info['type']}'{source_info}{capability_info}"
        )
    
    # Add demo client if requested
    if enable_demo_bool and enable_missions_bool:
        get_logger("rpc_dynamic_mission").info("Adding mission demonstration client (5 second delay)")
        
        # Demonstration client (starts after nodes are ready)
        mission_demo_client = TimerAction(
            period=5.0,  # Wait 5 seconds for nodes to start
            actions=[
                Node(
                    package='airsim_ros_pkgs',
                    executable='mission_coordination_demo_client.py',
                    name='mission_demo_client',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'INFO']
                )
            ]
        )
        nodes.append(mission_demo_client)
    elif enable_demo_bool and not enable_missions_bool:
        get_logger("rpc_dynamic_mission").warn(
            "Demo client requires mission mode (enable_missions:=true)"
        )
    
    # Summary log
    get_logger("rpc_dynamic_mission").info("=" * 60)
    get_logger("rpc_dynamic_mission").info(f"🚁 Dynamic Mission Launch Configuration:")
    get_logger("rpc_dynamic_mission").info(f"   Vehicles discovered: {len(vehicles)}")
    get_logger("rpc_dynamic_mission").info(f"   Mission mode: {'Enabled' if enable_missions_bool else 'Disabled'}")
    get_logger("rpc_dynamic_mission").info(f"   Coordination: {'/mission_coordinator' if enable_missions_bool else '/airsim_coordination_node'}")
    if enable_missions_bool:
        get_logger("rpc_dynamic_mission").info(f"   Mission capabilities:")
        for v in vehicles:
            if v['type'] == 'multirotor':
                get_logger("rpc_dynamic_mission").info(f"      /{v['name']}/actions/search_area")
                get_logger("rpc_dynamic_mission").info(f"      /{v['name']}/actions/navigate_to_target")
                get_logger("rpc_dynamic_mission").info(f"      /{v['name']}/actions/track_target")
    get_logger("rpc_dynamic_mission").info(f"   Demo client: {'Enabled' if enable_demo_bool else 'Disabled'}")
    get_logger("rpc_dynamic_mission").info("=" * 60)
    
    return nodes


def generate_launch_description():
    """
    Generate launch description with RPC-based dynamic vehicle discovery and mission support
    """
    # Launch arguments
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='172.28.240.1',  # Docker default
        description='IP address of AirSim server'
    )
    
    host_port_arg = DeclareLaunchArgument(
        'host_port',
        default_value='41451',
        description='Port of AirSim server'
    )
    
    enable_missions_arg = DeclareLaunchArgument(
        'enable_missions',
        default_value='true',
        description='Enable mission-capable nodes with search/rescue actions'
    )
    
    enable_demo_arg = DeclareLaunchArgument(
        'enable_demo',
        default_value='false',
        description='Launch mission demonstration client after startup'
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
    
    # Opaque function for dynamic node creation
    dynamic_nodes = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        host_ip_arg,
        host_port_arg,
        enable_missions_arg,
        enable_demo_arg,
        fallback_settings_arg,
        rpc_timeout_arg,
        debug_arg,
        dynamic_nodes
    ])