#!/usr/bin/env python3
"""
Unified AirSim Configuration Generator
Generates both settings.json and docker-compose.yml from a single configuration
"""

import json
import sys
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field, asdict
from enum import Enum
import argparse

# Optional yaml import for docker-compose generation
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False
    print("⚠️  PyYAML not installed. Docker-compose files will be generated as text.")
    print("   To install: pip install PyYAML")

def write_yaml_file(data: dict, filepath: Path, description: str = "file"):
    """Write data to YAML file, with fallback if PyYAML not available"""
    if YAML_AVAILABLE:
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    else:
        # Fallback: Generate basic YAML manually for docker-compose
        if "services" in data:  # Docker compose file
            write_docker_compose_manually(data, filepath)
        else:  # Other YAML (like RViz config)
            print(f"⚠️  Skipping {description} generation (requires PyYAML)")
            return False
    return True

def write_docker_compose_manually(compose_data: dict, filepath: Path):
    """Manually write docker-compose.yml without PyYAML dependency"""
    with open(filepath, 'w') as f:
        f.write(f"version: '{compose_data.get('version', '3.8')}'\n\n")
        
        # Write services
        if 'services' in compose_data:
            f.write("services:\n")
            for service_name, service_config in compose_data['services'].items():
                f.write(f"  {service_name}:\n")
                
                # Build section
                if 'build' in service_config:
                    f.write("    build:\n")
                    build_config = service_config['build']
                    if isinstance(build_config, str):
                        f.write(f"      context: {build_config}\n")
                    else:
                        f.write(f"      context: {build_config.get('context', '.')}\n")
                        if 'dockerfile' in build_config:
                            f.write(f"      dockerfile: {build_config['dockerfile']}\n")
                
                # Simple string fields
                for field in ['container_name', 'hostname', 'image']:
                    if field in service_config:
                        f.write(f"    {field}: {service_config[field]}\n")
                
                # Environment
                if 'environment' in service_config:
                    f.write("    environment:\n")
                    for env_var in service_config['environment']:
                        f.write(f"    - {env_var}\n")
                
                # Ports
                if 'ports' in service_config:
                    f.write("    ports:\n")
                    for port in service_config['ports']:
                        f.write(f"    - \"{port}\"\n")
                
                # Networks
                if 'networks' in service_config:
                    f.write("    networks:\n")
                    networks = service_config['networks']
                    if isinstance(networks, list):
                        for network in networks:
                            f.write(f"    - {network}\n")
                    elif isinstance(networks, dict):
                        for net_name, net_config in networks.items():
                            f.write(f"      {net_name}:\n")
                            if 'ipv4_address' in net_config:
                                f.write(f"        ipv4_address: {net_config['ipv4_address']}\n")
                
                # Volumes
                if 'volumes' in service_config:
                    f.write("    volumes:\n")
                    for volume in service_config['volumes']:
                        f.write(f"    - {volume}\n")
                
                # Other simple fields
                for field in ['restart', 'tty', 'stdin_open']:
                    if field in service_config:
                        value = str(service_config[field]).lower()
                        f.write(f"    {field}: {value}\n")
                
                # Command
                if 'command' in service_config:
                    f.write("    command:\n")
                    command = service_config['command']
                    if isinstance(command, list):
                        for cmd_line in command:
                            f.write(f"    - {cmd_line}\n")
                    else:
                        f.write(f"    - {command}\n")
                
                f.write("\n")
        
        # Write networks
        if 'networks' in compose_data:
            f.write("networks:\n")
            for net_name, net_config in compose_data['networks'].items():
                f.write(f"  {net_name}:\n")
                if 'driver' in net_config:
                    f.write(f"    driver: {net_config['driver']}\n")
                if 'ipam' in net_config:
                    f.write("    ipam:\n")
                    ipam = net_config['ipam']
                    if 'config' in ipam:
                        f.write("      config:\n")
                        for config_item in ipam['config']:
                            f.write("      - subnet: {}\n".format(config_item['subnet']))
            f.write("\n")
        
        # Write volumes
        if 'volumes' in compose_data:
            f.write("volumes:\n")
            for vol_name, vol_config in compose_data['volumes'].items():
                f.write(f"  {vol_name}:\n")
                if isinstance(vol_config, dict):
                    for key, value in vol_config.items():
                        f.write(f"    {key}: {value}\n")
                f.write("\n")

# ===== Configuration Classes =====

@dataclass
class CameraConfig:
    name: str
    image_types: List[int] = field(default_factory=lambda: [0])  # 0=RGB, 1=Depth, 5=Segmentation
    width: int = 640
    height: int = 480
    fov_degrees: float = 90.0
    position: "Position" = field(default_factory=lambda: Position(x=0.50, y=0.00, z=0.10))
    enabled: bool = True

class VehicleType(Enum):
    PX4_MULTIROTOR = "PX4Multirotor"
    ARDUPILOT_COPTER = "ArduCopter"
    SIMPLE_FLIGHT = "SimpleFlight"
    PHYSX_CAR = "PhysXCar"
    COMPUTER_VISION = "ComputerVision"

@dataclass
class GPSConfig:
    enabled: bool = True
    start_latitude: float = 47.641468
    start_longitude: float = -122.140165
    start_altitude: float = 122.0
    eph_time_constant: float = 0.9
    epv_time_constant: float = 0.9
    eph_initial: float = 25.0
    epv_initial: float = 25.0
    eph_final: float = 0.1
    epv_final: float = 0.1
    eph_min_3d: float = 3.0
    eph_min_2d: float = 4.0
    update_latency: float = 0.2
    update_frequency: int = 50
    startup_delay: int = 1

@dataclass
class SensorConfig:
    gps: Optional[GPSConfig] = None
    magnetometer_enabled: bool = True
    barometer_enabled: bool = True
    imu_enabled: bool = True
    lidar_enabled: bool = False
    distance_enabled: bool = False

@dataclass
class NetworkPorts:
    tcp_port: int
    control_port_local: int
    control_port_remote: int
    qgc_port: int
    api_server_port: int = 41451

@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = -2.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

@dataclass
class VehicleConfig:
    name: str
    vehicle_type: VehicleType
    position: Position
    sensors: SensorConfig
    ports: NetworkPorts
    cameras: List[CameraConfig] = field(default_factory=list)
    pawn_path: Optional[str] = None
    docker_enabled: bool = True
    container_ip: Optional[str] = None

@dataclass
class SimulationConfig:
    vehicles: List[VehicleConfig] = field(default_factory=list)
    sim_mode: str = "Multirotor"
    clock_type: str = "SteppableClock"
    api_server_endpoint: str = "0.0.0.0:41451"
    origin_geopoint: Dict[str, float] = field(default_factory=lambda: {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 10
    })
    docker_network_subnet: str = "172.30.0.0/16"
    pawn_paths: Dict[str, Dict[str, str]] = field(default_factory=dict)

# ===== Generator Functions =====

class ConfigGenerator:
    def __init__(self, config: SimulationConfig, enable_tcp_ports: bool = False):
        self.config = config
        self.enable_tcp_ports = enable_tcp_ports
        self.base_tcp_port = 4560
        self.base_control_local = 14540
        self.base_control_remote = 14580
        self.base_qgc_port = 14549
        self.base_container_ip = 10
        
    def calculate_ports(self, index: int) -> NetworkPorts:
        """Calculate unique ports for a vehicle based on its index"""
        return NetworkPorts(
            tcp_port=self.base_tcp_port + index,
            control_port_local=self.base_control_local + index,
            control_port_remote=self.base_control_remote + index,
            qgc_port=self.base_qgc_port + index
        )
    
    def calculate_position(self, index: int, layout: str = "grid") -> Position:
        """Calculate vehicle position based on index and layout type"""
        if layout == "line":
            return Position(x=(index - 1) * 5, y=0, z=-2)
        elif layout == "grid":
            row_size = 5
            x = ((index - 1) % row_size) * 5
            y = ((index - 1) // row_size) * 5
            return Position(x=x, y=y, z=-2)
        elif layout == "circle":
            import math
            radius = 10
            angle = (2 * math.pi * (index - 1)) / len(self.config.vehicles)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            return Position(x=x, y=y, z=-2)
        else:
            raise ValueError(f"Unknown layout type: {layout}")
    
    def generate_airsim_settings(self) -> Dict:
        """Generate AirSim settings.json configuration"""
        settings = {
            "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
            "SettingsVersion": 2.0,
            "SimMode": self.config.sim_mode,
            "ClockType": self.config.clock_type,
            "ApiServerEndpoint": self.config.api_server_endpoint,
            "OriginGeopoint": self.config.origin_geopoint,
            "Vehicles": {}
        }
        
        # Add pawn paths if specified
        if self.config.pawn_paths:
            settings["PawnPaths"] = self.config.pawn_paths
        
        # Generate vehicle configurations
        all_subwindows = []
        for vehicle in self.config.vehicles:
            vehicle_settings = self._generate_vehicle_settings(vehicle)
            settings["Vehicles"][vehicle.name] = vehicle_settings
            
            # Generate SubWindows for cameras
            if vehicle.cameras:
                vehicle_subwindows = self._generate_subwindows(vehicle)
                all_subwindows.extend(vehicle_subwindows)
        
        # Add SubWindows to settings if any cameras are configured
        if all_subwindows:
            settings["SubWindows"] = all_subwindows
            
        return settings
    
    def _generate_vehicle_settings(self, vehicle: VehicleConfig) -> Dict:
        """Generate settings for a single vehicle"""
        settings = {
            "VehicleType": vehicle.vehicle_type.value,
            "X": vehicle.position.x,
            "Y": vehicle.position.y,
            "Z": vehicle.position.z,
            "Pitch": vehicle.position.pitch,
            "Roll": vehicle.position.roll,
            "Yaw": vehicle.position.yaw
        }
        
        # Add PX4-specific settings
        if vehicle.vehicle_type == VehicleType.PX4_MULTIROTOR:
            settings.update({
                "UseSerial": False,
                "UseTcp": True,
                "TcpPort": vehicle.ports.tcp_port,
                "ControlPortLocal": vehicle.ports.control_port_local,
                "ControlPortRemote": vehicle.ports.control_port_remote,
                "LocalHostIp": "127.0.0.1",
                "LockStep": True,
                "Sensors": self._generate_sensor_config(vehicle.sensors),
                "Parameters": {
                    "NAV_RCL_ACT": 0,
                    "NAV_DLL_ACT": 0,
                    "COM_OBL_ACT": 1,
                    "LPE_LAT": self.config.origin_geopoint["Latitude"],
                    "LPE_LON": self.config.origin_geopoint["Longitude"],
                    "COM_ARM_WO_GPS": 0,
                    "EKF2_AID_MASK": 1,
                    "EKF2_HGT_MODE": 0,
                    "EKF2_GPS_CHECK": 31
                }
            })
        
        # Add pawn path if specified
        if vehicle.pawn_path:
            settings["PawnPath"] = vehicle.pawn_path
            
        # Add cameras if configured
        camera_settings = self._generate_camera_settings(vehicle)
        if camera_settings:
            settings["Cameras"] = camera_settings
            
        return settings
    
    def _generate_sensor_config(self, sensors: SensorConfig) -> Dict:
        """Generate sensor configuration"""
        sensor_dict = {}
        
        if sensors.gps and sensors.gps.enabled:
            sensor_dict["Gps"] = {
                "SensorType": 3,
                "Enabled": True,
                "EphTimeConstant": sensors.gps.eph_time_constant,
                "EpvTimeConstant": sensors.gps.epv_time_constant,
                "EphInitial": sensors.gps.eph_initial,
                "EpvInitial": sensors.gps.epv_initial,
                "EphFinal": sensors.gps.eph_final,
                "EpvFinal": sensors.gps.epv_final,
                "EphMin3d": sensors.gps.eph_min_3d,
                "EphMin2d": sensors.gps.eph_min_2d,
                "UpdateLatency": sensors.gps.update_latency,
                "UpdateFrequency": sensors.gps.update_frequency,
                "StartupDelay": sensors.gps.startup_delay
            }
            
        if sensors.magnetometer_enabled:
            sensor_dict["Magnetometer"] = {
                "SensorType": 4,
                "Enabled": True
            }
            
        if sensors.barometer_enabled:
            sensor_dict["Barometer"] = {
                "SensorType": 1,
                "Enabled": True,
                "PressureFactorSigma": 0.0001825
            }
            
        if sensors.imu_enabled:
            sensor_dict["Imu"] = {
                "SensorType": 2,
                "Enabled": True
            }
            
        return sensor_dict
    
    def _generate_subwindows(self, vehicle: VehicleConfig) -> List[Dict]:
        """Generate SubWindows configuration for vehicle cameras"""
        subwindows = []
        window_id = 0
        
        for camera in vehicle.cameras:
            for image_type in camera.image_types:
                subwindow = {
                    "WindowID": window_id,
                    "CameraName": camera.name,
                    "ImageType": image_type,
                    "VehicleName": vehicle.name,
                    "Visible": True
                }
                subwindows.append(subwindow)
                window_id += 1
                
        return subwindows
    
    def _generate_camera_settings(self, vehicle: VehicleConfig) -> Dict:
        """Generate camera configuration for a vehicle"""
        if not vehicle.cameras:
            return {}
            
        cameras = {}
        for camera in vehicle.cameras:
            camera_config = {
                "CaptureSettings": [
                    {
                        "ImageType": image_type,
                        "Width": camera.width,
                        "Height": camera.height,
                        "FOV_Degrees": camera.fov_degrees
                    }
                    for image_type in camera.image_types
                ],
                "X": camera.position.x,
                "Y": camera.position.y,
                "Z": camera.position.z,
                "Pitch": camera.position.pitch,
                "Roll": camera.position.roll,
                "Yaw": camera.position.yaw
            }
            cameras[camera.name] = camera_config
            
        return cameras
    
    def generate_docker_compose(self) -> Dict:
        """Generate docker-compose.yml configuration"""
        compose = {
            "version": "3.8",
            "services": {},
            "networks": {
                "airsim-network": {
                    "driver": "bridge",
                    "ipam": {
                        "config": [{
                            "subnet": self.config.docker_network_subnet
                        }]
                    }
                }
            },
            "volumes": {
                "px4-shared-data": {
                    "driver": "local"
                }
            }
        }
        
        # Generate service configurations for docker-enabled vehicles
        for idx, vehicle in enumerate(self.config.vehicles, 1):
            if vehicle.docker_enabled and vehicle.vehicle_type == VehicleType.PX4_MULTIROTOR:
                service = self._generate_docker_service(vehicle, idx)
                compose["services"][vehicle.name.lower().replace("_", "-")] = service
                
        return compose
    
    def _generate_docker_service(self, vehicle: VehicleConfig, index: int) -> Dict:
        """Generate Docker service configuration for a vehicle"""
        container_ip = f"172.30.0.{self.base_container_ip + index - 1}"
        
        return {
            "build": {
                "context": "../common",
                "dockerfile": "Dockerfile"
            },
            "container_name": vehicle.name.lower().replace("_", "-"),
            "hostname": vehicle.name.lower().replace("_", "-"),
            "environment": [
                f"PX4_HOME_LAT={self.config.origin_geopoint['Latitude']}",
                f"PX4_HOME_LON={self.config.origin_geopoint['Longitude']}",
                f"PX4_HOME_ALT={self.config.origin_geopoint['Altitude']}",
                "PX4_SYS_AUTOSTART=10016",
                "PX4_SIM_HOSTNAME=host.docker.internal",
                "PX4_SIM_MODEL=iris",
                f"PX4_INSTANCE={index}"
            ],
            "ports": self._generate_port_mappings(vehicle),
            "networks": {
                "airsim-network": {
                    "ipv4_address": container_ip
                }
            },
            "volumes": [
                "px4-shared-data:/px4_data",
                "../common/scripts:/scripts"
            ],
            "restart": "unless-stopped",
            "command": ["/Scripts/run_airsim_sitl_final.sh", str(index)]
        }
    
    def _generate_port_mappings(self, vehicle: VehicleConfig) -> List[str]:
        """Generate port mappings for a vehicle based on configuration"""
        ports = [
            f"{vehicle.ports.qgc_port}:14550/udp",  # QGroundControl
            f"{vehicle.ports.control_port_local}:{vehicle.ports.control_port_local}/udp",  # MAVLink control local
            f"{vehicle.ports.control_port_remote}:{vehicle.ports.control_port_remote}/udp"  # MAVLink control remote
        ]
        
        # Add TCP port mapping if enabled (needed for some Docker networking setups)
        if self.enable_tcp_ports:
            ports.append(f"{vehicle.ports.tcp_port}:{vehicle.ports.tcp_port}/tcp")
        
        return ports
    
    def generate_launcher_script(self, compose_file: str, platform: str = "windows") -> str:
        """Generate launcher script for the configuration"""
        num_vehicles = len([v for v in self.config.vehicles if v.docker_enabled])
        
        if platform == "windows":
            return self._generate_windows_launcher(compose_file, num_vehicles)
        else:
            return self._generate_linux_launcher(compose_file, num_vehicles)
    
    def _generate_windows_launcher(self, compose_file: str, num_vehicles: int) -> str:
        """Generate Windows batch launcher script"""
        port_info = ""
        for vehicle in self.config.vehicles:
            if vehicle.docker_enabled:
                port_info += f'echo   {vehicle.name}: TCP={vehicle.ports.tcp_port} ^| '
                port_info += f'MAVLink={vehicle.ports.control_port_local}/{vehicle.ports.control_port_remote} ^| '
                port_info += f'QGC={vehicle.ports.qgc_port}\n'
        
        return f"""@echo off
echo ================================================
echo    Unified AirSim Docker Launcher
echo    Generated for {num_vehicles} vehicle(s)
echo ================================================
echo.

echo [INFO] Creating shared data directory...
if not exist "shared_data" mkdir shared_data

echo [INFO] Building Docker images...
docker-compose -f {compose_file} build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker images!
    pause
    exit /b 1
)

echo [INFO] Cleaning up existing containers...
docker-compose -f {compose_file} down > nul 2>&1

echo [INFO] Starting {num_vehicles} vehicle instance(s)...
docker-compose -f {compose_file} up -d

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start vehicle instances!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    {num_vehicles} Vehicle(s) Started Successfully!
echo ================================================
echo.

echo Port Configuration:
{port_info}
echo.
echo Management Commands:
echo   View logs:        docker-compose -f {compose_file} logs
echo   Stop all:         docker-compose -f {compose_file} down
echo   Container status: docker-compose -f {compose_file} ps
echo.
pause
"""
    
    def _generate_linux_launcher(self, compose_file: str, num_vehicles: int) -> str:
        """Generate Linux shell launcher script"""
        port_info = ""
        for vehicle in self.config.vehicles:
            if vehicle.docker_enabled:
                port_info += f'echo "  {vehicle.name}: TCP={vehicle.ports.tcp_port} | '
                port_info += f'MAVLink={vehicle.ports.control_port_local}/{vehicle.ports.control_port_remote} | '
                port_info += f'QGC={vehicle.ports.qgc_port}"\n'
        
        return f"""#!/bin/bash
echo "================================================"
echo "   Unified AirSim Docker Launcher"
echo "   Generated for {num_vehicles} vehicle(s)"
echo "================================================"
echo

echo "[INFO] Creating shared data directory..."
mkdir -p shared_data

echo "[INFO] Building Docker images..."
docker-compose -f {compose_file} build

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to build Docker images!"
    exit 1
fi

echo "[INFO] Cleaning up existing containers..."
docker-compose -f {compose_file} down > /dev/null 2>&1

echo "[INFO] Starting {num_vehicles} vehicle instance(s)..."
docker-compose -f {compose_file} up -d

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to start vehicle instances!"
    exit 1
fi

echo
echo "================================================"
echo "   {num_vehicles} Vehicle(s) Started Successfully!"
echo "================================================"
echo

echo "Port Configuration:"
{port_info}
echo
echo "Management Commands:"
echo "  View logs:        docker-compose -f {compose_file} logs"
echo "  Stop all:         docker-compose -f {compose_file} down"
echo "  Container status: docker-compose -f {compose_file} ps"
echo
"""

    def generate_rviz_config(self, output_path: Path) -> None:
        """Generate RViz2 configuration file for camera visualization"""
        displays = []
        
        # Add basic displays
        displays.extend([
            {
                "Alpha": 0.5,
                "Cell Size": 1,
                "Class": "rviz_default_plugins/Grid",
                "Color": "160; 160; 164",
                "Enabled": True,
                "Name": "Grid"
            },
            {
                "Class": "rviz_default_plugins/TF",
                "Enabled": True,
                "Name": "TF"
            }
        ])
        
        # Add camera displays
        for vehicle in self.config.vehicles:
            for camera in vehicle.cameras:
                for image_type in camera.image_types:
                    image_type_name = self._get_image_type_name(image_type)
                    display = {
                        "Class": "rviz_default_plugins/Image",
                        "Enabled": True,
                        "Name": f"{vehicle.name} {camera.name} {image_type_name}",
                        "Topic": {
                            "Value": f"/airsim_node/{vehicle.name}/{camera.name}/{image_type_name}"
                        },
                        "Transport Hint": "raw"
                    }
                    displays.append(display)
        
        rviz_config = {
            "Panels": [
                {
                    "Class": "rviz_common/Displays",
                    "Name": "Displays"
                }
            ],
            "Visualization Manager": {
                "Class": "",
                "Displays": displays,
                "Global Options": {
                    "Background Color": "48; 48; 48",
                    "Fixed Frame": "world_ned"
                },
                "Tools": [
                    {"Class": "rviz_default_plugins/Interact"},
                    {"Class": "rviz_default_plugins/MoveCamera"}
                ]
            }
        }
        
        rviz_file = output_path / "airsim_cameras.rviz"
        if write_yaml_file(rviz_config, rviz_file, "RViz config"):
            print(f"✅ Generated: {rviz_file}")
        else:
            print(f"⚠️  Skipped RViz config generation (install PyYAML to enable)")
    
    def _get_image_type_name(self, image_type: int) -> str:
        """Convert image type number to ROS topic name"""
        type_mapping = {
            0: "Scene",
            1: "DepthPlanar",
            2: "DepthPerspective",
            3: "DepthVis",
            4: "DisparityNormalized",
            5: "Segmentation",
            6: "SurfaceNormals",
            7: "Infrared"
        }
        return type_mapping.get(image_type, f"ImageType{image_type}")

# ===== Camera Presets =====

def get_camera_presets() -> Dict[str, CameraConfig]:
    """Get predefined camera configurations"""
    return {
        "front_rgb": CameraConfig(
            name="front_rgb",
            image_types=[0],  # RGB
            width=1920,
            height=1080,
            position=Position(x=0.50, y=0.00, z=0.10)
        ),
        "front_depth": CameraConfig(
            name="front_depth",
            image_types=[1],  # Depth
            width=640,
            height=480,
            position=Position(x=0.50, y=0.00, z=0.10)
        ),
        "downward_cam": CameraConfig(
            name="downward_cam",
            image_types=[0],  # RGB
            width=640,
            height=480,
            position=Position(x=0.00, y=0.00, z=0.10, pitch=-90.0)
        ),
        "segmentation_cam": CameraConfig(
            name="segmentation_cam",
            image_types=[5],  # Segmentation
            width=640,
            height=480,
            position=Position(x=0.50, y=0.00, z=0.10)
        ),
        "stereo_left": CameraConfig(
            name="stereo_left",
            image_types=[0],  # RGB
            width=640,
            height=480,
            position=Position(x=0.50, y=-0.10, z=0.10)
        ),
        "stereo_right": CameraConfig(
            name="stereo_right",
            image_types=[0],  # RGB
            width=640,
            height=480,
            position=Position(x=0.50, y=0.10, z=0.10)
        ),
        "gimbal_cam": CameraConfig(
            name="gimbal_cam",
            image_types=[0],  # RGB
            width=1920,
            height=1080,
            position=Position(x=0.30, y=0.00, z=0.20, pitch=-45.0)
        )
    }

# ===== Configuration Templates =====

def create_single_drone_config(cameras: List[str] = None, enable_tcp_ports: bool = False) -> SimulationConfig:
    """Create configuration for a single drone"""
    config = SimulationConfig()
    
    vehicle = VehicleConfig(
        name="PX4_Drone1",
        vehicle_type=VehicleType.PX4_MULTIROTOR,
        position=Position(x=0, y=0, z=-2),
        sensors=SensorConfig(
            gps=GPSConfig(enabled=True),
            magnetometer_enabled=True,
            barometer_enabled=True,
            imu_enabled=True
        ),
        ports=NetworkPorts(
            tcp_port=4561,
            control_port_local=14541,
            control_port_remote=14581,
            qgc_port=14550
        ),
        docker_enabled=True
    )
    
    # Add cameras if specified
    if cameras:
        camera_presets = get_camera_presets()
        for camera_name in cameras:
            if camera_name in camera_presets:
                vehicle.cameras.append(camera_presets[camera_name])
    
    config.vehicles.append(vehicle)
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    return config

def create_multi_drone_config(num_drones: int, layout: str = "grid", cameras: List[str] = None, enable_tcp_ports: bool = False, 
                             swarm_id: int = 1, enable_ultra_swarm: bool = False) -> SimulationConfig:
    """Create configuration for multiple drones with ultra-swarm support"""
    config = SimulationConfig()
    generator = ConfigGenerator(config, enable_tcp_ports=enable_tcp_ports)
    
    # Adjust port allocation for ultra-swarm mode
    if enable_ultra_swarm:
        generator.base_tcp_port = 4560 + (swarm_id - 1) * 10
        generator.base_control_local = 14549 + (swarm_id - 1) * 10  
        generator.base_control_remote = 18569 + (swarm_id - 1) * 10
        generator.base_qgc_port = 14549 + (swarm_id - 1) * 10
        
        # Adjust GPS location per swarm
        config.origin_geopoint["Latitude"] += (swarm_id - 1) * 0.001
        config.origin_geopoint["Longitude"] += (swarm_id - 1) * 0.001
    
    for i in range(1, num_drones + 1):
        vehicle_name = f"PX4_Swarm{swarm_id}_Drone{i}" if enable_ultra_swarm else f"PX4_Drone{i}"
        
        vehicle = VehicleConfig(
            name=vehicle_name,
            vehicle_type=VehicleType.PX4_MULTIROTOR,
            position=generator.calculate_position(i, layout),
            sensors=SensorConfig(
                gps=GPSConfig(enabled=True),
                magnetometer_enabled=True,
                barometer_enabled=True,
                imu_enabled=True
            ),
            ports=generator.calculate_ports(i),
            docker_enabled=True
        )
        
        # Add cameras if specified
        if cameras:
            camera_presets = get_camera_presets()
            for camera_name in cameras:
                if camera_name in camera_presets:
                    vehicle.cameras.append(camera_presets[camera_name])
        
        config.vehicles.append(vehicle)
    
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    return config

def create_ultra_swarm_config(num_swarms: int = 3, drones_per_swarm: int = 9, 
                             cameras: List[str] = None, enable_tcp_ports: bool = False) -> SimulationConfig:
    """Create ultra-swarm configuration with up to 27 drones (3 swarms x 9 drones)"""
    config = SimulationConfig()
    
    # GPS home locations for each swarm (Seattle area)
    swarm_gps_origins = {
        1: {"lat": 47.641468, "lon": -122.140165, "name": "Blue Team"},  # Seattle
        2: {"lat": 47.642468, "lon": -122.139165, "name": "Red Team"},   # Bellevue  
        3: {"lat": 47.643468, "lon": -122.138165, "name": "Green Team"}  # Redmond
    }
    
    all_vehicles = []
    
    for swarm_id in range(1, num_swarms + 1):
        # Create a temporary config for this swarm
        swarm_config = SimulationConfig()
        swarm_config.origin_geopoint = {
            "Latitude": swarm_gps_origins[swarm_id]["lat"],
            "Longitude": swarm_gps_origins[swarm_id]["lon"],
            "Altitude": 10
        }
        
        # Generate swarm configuration
        swarm_vehicles = create_multi_drone_config(
            num_drones=drones_per_swarm,
            layout="grid",
            cameras=cameras,
            enable_tcp_ports=enable_tcp_ports,
            swarm_id=swarm_id,
            enable_ultra_swarm=True
        )
        
        # Add all vehicles from this swarm
        all_vehicles.extend(swarm_vehicles.vehicles)
    
    # Combine all vehicles into main config
    config.vehicles = all_vehicles
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    # Use the first swarm's GPS origin as the global origin
    config.origin_geopoint = {
        "Latitude": swarm_gps_origins[1]["lat"],
        "Longitude": swarm_gps_origins[1]["lon"],
        "Altitude": 10
    }
    
    return config

def create_simpleflight_config(num_drones: int = 1, layout: str = "grid", cameras: List[str] = None) -> SimulationConfig:
    """Create SimpleFlight configuration for 1-27 drones (no Docker required)"""
    if num_drones < 1 or num_drones > 27:
        raise ValueError("SimpleFlight supports 1-27 drones")
    
    config = SimulationConfig()
    config.sim_mode = "Multirotor"
    
    # SimpleFlight uses different port scheme (no external PX4)
    generator = ConfigGenerator(config, enable_tcp_ports=False)
    
    for i in range(1, num_drones + 1):
        vehicle = VehicleConfig(
            name=f"Drone{i}",
            vehicle_type=VehicleType.SIMPLE_FLIGHT,
            position=generator.calculate_position(i, layout),
            sensors=SensorConfig(
                gps=GPSConfig(enabled=True),
                magnetometer_enabled=True,
                barometer_enabled=True,
                imu_enabled=True
            ),
            ports=NetworkPorts(
                tcp_port=4561,  # SimpleFlight drones all use same AirSim server
                control_port_local=14540 + i,  # For logging/debugging
                control_port_remote=14580 + i,  # For logging/debugging  
                qgc_port=14550 + i - 1        # For potential QGC monitoring
            ),
            docker_enabled=False  # SimpleFlight runs inside AirSim, no external containers
        )
        
        # Add cameras if specified
        if cameras:
            camera_presets = get_camera_presets()
            for camera_name in cameras:
                if camera_name in camera_presets:
                    vehicle.cameras.append(camera_presets[camera_name])
        
        config.vehicles.append(vehicle)
    
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    return config

def create_mixed_vehicle_config(enable_tcp_ports: bool = False) -> SimulationConfig:
    """Create configuration with mixed vehicle types"""
    config = SimulationConfig()
    
    # Add a multirotor
    drone = VehicleConfig(
        name="Drone1",
        vehicle_type=VehicleType.PX4_MULTIROTOR,
        position=Position(x=0, y=0, z=-2),
        sensors=SensorConfig(
            gps=GPSConfig(enabled=True),
            magnetometer_enabled=True,
            barometer_enabled=True,
            imu_enabled=True
        ),
        ports=NetworkPorts(
            tcp_port=4561,
            control_port_local=14541,
            control_port_remote=14581,
            qgc_port=14550
        ),
        docker_enabled=True
    )
    
    # Add a car
    car = VehicleConfig(
        name="Car1",
        vehicle_type=VehicleType.PHYSX_CAR,
        position=Position(x=10, y=0, z=0),
        sensors=SensorConfig(
            gps=GPSConfig(enabled=True),
            magnetometer_enabled=False,
            barometer_enabled=False,
            imu_enabled=True
        ),
        ports=NetworkPorts(
            tcp_port=4562,
            control_port_local=14542,
            control_port_remote=14582,
            qgc_port=14551
        ),
        docker_enabled=False  # Cars typically don't use PX4 Docker
    )
    
    config.vehicles.extend([drone, car])
    config.sim_mode = "Multirotor"  # Can be overridden if needed
    
    return config

# ===== Main CLI =====

def main():
    parser = argparse.ArgumentParser(
        description="Unified AirSim Configuration Generator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate single drone configuration
  python unified_generator.py single

  # Generate 5 drones in grid layout
  python unified_generator.py multi --num-drones 5 --layout grid

  # Generate single drone with cameras and RViz config
  python unified_generator.py single --cameras front_rgb front_depth --create-rviz

  # Generate multi-drone with cameras
  python unified_generator.py multi --num-drones 3 --cameras front_rgb downward_cam --create-rviz

  # Generate ultra-swarm configuration (27 drones: 3 swarms x 9 drones)
  python unified_generator.py ultra-swarm

  # Generate partial ultra-swarm (2 swarms x 5 drones each)
  python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 5

  # Generate SimpleFlight configuration (1-27 drones, no Docker required)
  python unified_generator.py simpleflight --num-drones 10

  # Generate SimpleFlight with cameras  
  python unified_generator.py simpleflight --num-drones 5 --cameras front_rgb downward_cam

  # Generate mixed vehicle configuration
  python unified_generator.py mixed

  # Custom output directory
  python unified_generator.py single --output-dir ./my_configs

  # Generate only settings.json
  python unified_generator.py single --settings-only

  # Generate only docker-compose.yml
  python unified_generator.py single --docker-only

  # Generate with TCP port mappings (for Docker networking issues)
  python unified_generator.py multi --num-drones 2 --enable-tcp-ports
        """
    )
    
    parser.add_argument(
        "mode",
        choices=["single", "multi", "mixed", "ultra-swarm", "simpleflight", "custom"],
        help="Configuration mode"
    )
    
    parser.add_argument(
        "--num-drones",
        type=int,
        default=3,
        help="Number of drones for multi mode (default: 3)"
    )
    
    parser.add_argument(
        "--num-swarms",
        type=int,
        default=3,
        choices=[1, 2, 3],
        help="Number of swarms for ultra-swarm mode (default: 3, max: 3)"
    )
    
    parser.add_argument(
        "--drones-per-swarm",
        type=int,
        default=9,
        choices=range(1, 10),
        help="Number of drones per swarm for ultra-swarm mode (default: 9, max: 9)"
    )
    
    parser.add_argument(
        "--layout",
        choices=["line", "grid", "circle"],
        default="grid",
        help="Drone layout for multi mode (default: grid)"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Output directory for generated files (default: current directory)"
    )
    
    parser.add_argument(
        "--settings-only",
        action="store_true",
        help="Generate only settings.json"
    )
    
    parser.add_argument(
        "--docker-only",
        action="store_true",
        help="Generate only docker-compose.yml"
    )
    
    parser.add_argument(
        "--platform",
        choices=["windows", "linux"],
        default="windows" if os.name == 'nt' else "linux",
        help="Platform for launcher script"
    )
    
    parser.add_argument(
        "--enable-tcp-ports",
        action="store_true",
        help="Enable TCP port mappings in docker-compose (needed for some Docker networking setups)"
    )
    
    parser.add_argument(
        "--gps-location",
        nargs=3,
        metavar=("LAT", "LON", "ALT"),
        type=float,
        help="GPS origin (latitude longitude altitude)"
    )
    
    parser.add_argument(
        "--cameras",
        nargs="*",
        choices=list(get_camera_presets().keys()),
        help="Camera presets to add to vehicles"
    )
    
    parser.add_argument(
        "--create-rviz",
        action="store_true",
        help="Generate RViz2 configuration file for camera visualization"
    )
    
    args = parser.parse_args()
    
    # Create configuration based on mode
    if args.mode == "single":
        config = create_single_drone_config(args.cameras, args.enable_tcp_ports)
    elif args.mode == "multi":
        if args.num_drones < 1 or args.num_drones > 10:
            print("Error: Number of drones must be between 1 and 10")
            sys.exit(1)
        config = create_multi_drone_config(args.num_drones, args.layout, args.cameras, args.enable_tcp_ports)
    elif args.mode == "mixed":
        config = create_mixed_vehicle_config(args.enable_tcp_ports)
    elif args.mode == "ultra-swarm":
        total_drones = args.num_swarms * args.drones_per_swarm
        if total_drones > 27:
            print(f"Error: Total drones ({total_drones}) exceeds maximum of 27")
            sys.exit(1)
        config = create_ultra_swarm_config(args.num_swarms, args.drones_per_swarm, args.cameras, args.enable_tcp_ports)
    elif args.mode == "simpleflight":
        if args.num_drones < 1 or args.num_drones > 27:
            print("Error: SimpleFlight supports 1-27 drones")
            sys.exit(1)
        config = create_simpleflight_config(args.num_drones, args.layout, args.cameras)
    else:
        print("Error: Custom mode not yet implemented")
        sys.exit(1)
    
    # Override GPS location if specified
    if args.gps_location:
        config.origin_geopoint = {
            "Latitude": args.gps_location[0],
            "Longitude": args.gps_location[1],
            "Altitude": args.gps_location[2]
        }
    
    # Determine output directory based on mode
    if args.output_dir == ".":
        # Default behavior: create in appropriate docker_clean subdirectory
        script_dir = Path(__file__).parent.parent.parent  # docker_clean directory
        if args.mode == "single":
            output_path = script_dir / "single_drone"
        elif args.mode == "multi":
            output_path = script_dir / "multi_drone"
        elif args.mode == "ultra-swarm":
            output_path = script_dir / "ultra_swarm"
        elif args.mode == "simpleflight":
            output_path = script_dir / "simpleflight"
        else:
            output_path = script_dir / f"{args.mode}_config"
    else:
        # Custom output directory specified
        output_path = Path(args.output_dir)
    
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Initialize generator
    generator = ConfigGenerator(config, enable_tcp_ports=args.enable_tcp_ports)
    
    print("🚁 Unified AirSim Configuration Generator")
    print("=" * 50)
    
    # Generate settings.json
    if not args.docker_only:
        settings = generator.generate_airsim_settings()
        settings_file = output_path / "settings.json"
        
        with open(settings_file, 'w') as f:
            json.dump(settings, f, indent=2)
            
        print(f"✅ Generated: {settings_file}")
        
        # Copy to default AirSim location
        default_settings = Path.home() / "Documents" / "AirSim" / "settings.json"
        default_settings.parent.mkdir(parents=True, exist_ok=True)
        
        with open(default_settings, 'w') as f:
            json.dump(settings, f, indent=2)
            
        print(f"✅ Copied to: {default_settings}")
    
    # Generate docker-compose.yml (only for Docker-enabled configs)
    if not args.settings_only and args.mode != "simpleflight":
        compose = generator.generate_docker_compose()
        compose_file = output_path / "docker-compose.yml"
        
        if write_yaml_file(compose, compose_file, "docker-compose.yml"):
            print(f"✅ Generated: {compose_file}")
        else:
            print(f"❌ Failed to generate docker-compose.yml")
        
        # Generate launcher script
        launcher_ext = ".bat" if args.platform == "windows" else ".sh"
        launcher_file = output_path / f"launch{launcher_ext}"
        launcher_content = generator.generate_launcher_script(
            "docker-compose.yml", 
            args.platform
        )
        
        with open(launcher_file, 'w') as f:
            f.write(launcher_content)
            
        if args.platform == "linux":
            launcher_file.chmod(0o755)
            
        print(f"✅ Generated: {launcher_file}")
    elif args.mode == "simpleflight":
        # Generate SimpleFlight-specific usage instructions
        usage_file = output_path / "README.txt"
        usage_content = f"""SimpleFlight Configuration Generated
=====================================

This SimpleFlight configuration includes {len(config.vehicles)} drone(s).
SimpleFlight is AirSim's built-in flight controller - no external Docker containers required.

Usage:
1. Copy settings.json to your AirSim Documents folder (already done automatically)
2. Launch AirSim (Unreal Engine environment)
3. All {len(config.vehicles)} drones will be available for control via AirSim's Python/C++ APIs

Connection Info:
- All drones connect to AirSim server on localhost:4561
- Vehicle names: {', '.join([v.name for v in config.vehicles])}
- Use these names when connecting via airsim.MultirotorClient()

Python Client Example:
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
for drone_name in {[v.name for v in config.vehicles]}:
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)
    client.takeoffAsync(vehicle_name=drone_name)

Benefits of SimpleFlight:
✅ No Docker setup required
✅ Fast startup and lightweight  
✅ Perfect for development and testing
✅ Supports up to 27 drones
✅ Built-in physics simulation
"""
        
        with open(usage_file, 'w') as f:
            f.write(usage_content)
            
        print(f"✅ Generated: {usage_file}")
    
    # Generate RViz configuration if requested
    if args.create_rviz and any(vehicle.cameras for vehicle in config.vehicles):
        generator.generate_rviz_config(output_path)
    
    # Print summary
    print("\n📋 Configuration Summary")
    print("=" * 50)
    print(f"Mode: {args.mode}")
    print(f"Vehicles: {len(config.vehicles)}")
    
    if args.mode == "ultra-swarm":
        print(f"Swarms: {args.num_swarms}")
        print(f"Drones per swarm: {args.drones_per_swarm}")
        print(f"Total drones: {args.num_swarms * args.drones_per_swarm}")
        
        # Show swarm-level summary
        for swarm_id in range(1, args.num_swarms + 1):
            swarm_vehicles = [v for v in config.vehicles if f"Swarm{swarm_id}" in v.name]
            if swarm_vehicles:
                print(f"\n🔹 Swarm {swarm_id} ({'Blue' if swarm_id == 1 else 'Red' if swarm_id == 2 else 'Green'} Team):")
                print(f"   Drones: {len(swarm_vehicles)}")
                print(f"   AirSim TCP ports: {swarm_vehicles[0].ports.tcp_port}-{swarm_vehicles[-1].ports.tcp_port}")
    else:
        for vehicle in config.vehicles:
            print(f"\n{vehicle.name}:")
            print(f"  Type: {vehicle.vehicle_type.value}")
            print(f"  Position: X={vehicle.position.x}, Y={vehicle.position.y}, Z={vehicle.position.z}")
            if vehicle.cameras:
                print(f"  Cameras: {', '.join([cam.name for cam in vehicle.cameras])}")
            if vehicle.docker_enabled:
                print(f"  Ports: TCP={vehicle.ports.tcp_port}, QGC={vehicle.ports.qgc_port}")
    
    print("\n💡 Next Steps:")
    print(f"1. Navigate to: {output_path}")
    if args.mode == "single":
        print("2. Start Docker: ./launch.bat (or ./launch.sh)")
    elif args.mode == "multi":
        print("2. Start Docker: ./launch.bat (or ./launch.sh)")
    print("3. Launch AirSim/Unreal Engine")
    print("4. Vehicles will connect automatically via HIL!")
    print(f"\n📁 Files created in: {output_path}")
    print("   - docker-compose.yml")
    print("   - launch.bat/.sh")
    if not args.docker_only:
        print("   - settings.json (also copied to ~/Documents/AirSim/)")
    if args.create_rviz and any(vehicle.cameras for vehicle in config.vehicles):
        print("   - airsim_cameras.rviz (RViz2 configuration)")
    
if __name__ == "__main__":
    main()