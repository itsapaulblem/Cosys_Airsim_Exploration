#!/usr/bin/env python3
"""
Unified AirSim Configuration Generator
Generates both settings.json and docker-compose.yml from a single configuration
"""

import json
import yaml
import sys
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field, asdict
from enum import Enum
import argparse

# ===== Configuration Classes =====

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
    docker_network_subnet: str = "172.25.0.0/16"
    docker_network_gateway: str = "172.25.0.1"
    pawn_paths: Dict[str, Dict[str, str]] = field(default_factory=dict)

# ===== Generator Functions =====

class ConfigGenerator:
    def __init__(self, config: SimulationConfig):
        self.config = config
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
        for vehicle in self.config.vehicles:
            vehicle_settings = self._generate_vehicle_settings(vehicle)
            settings["Vehicles"][vehicle.name] = vehicle_settings
            
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
    
    def generate_docker_compose(self) -> Dict:
        """Generate docker-compose.yml configuration"""
        compose = {
            "version": "3.8",
            "services": {},
            "networks": {
                "airsim-network": {
                    "name": "airsim-network",
                    "driver": "bridge",
                    "ipam": {
                        "driver": "default",
                        "config": [{
                            "subnet": self.config.docker_network_subnet,
                            "gateway": self.config.docker_network_gateway
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
        container_ip = f"172.25.0.{self.base_container_ip + index - 1}"
        
        return {
            "build": {
                "context": ".",
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
            "ports": [
                f"{vehicle.ports.control_port_local}:{vehicle.ports.control_port_local}/udp",
                f"{vehicle.ports.control_port_remote}:{vehicle.ports.control_port_remote}/udp",
                f"{vehicle.ports.qgc_port}:{vehicle.ports.qgc_port}/udp"
            ],
            "networks": {
                "airsim-network": {
                    "ipv4_address": container_ip
                }
            },
            "volumes": [
                "px4-shared-data:/px4_data"
            ],
            "restart": "unless-stopped",
            "command": ["/Scripts/run_airsim_sitl_final.sh", str(index)]
        }
    
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

# ===== Configuration Templates =====

def create_single_drone_config() -> SimulationConfig:
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
    
    config.vehicles.append(vehicle)
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    return config

def create_multi_drone_config(num_drones: int, layout: str = "grid") -> SimulationConfig:
    """Create configuration for multiple drones"""
    config = SimulationConfig()
    generator = ConfigGenerator(config)
    
    for i in range(1, num_drones + 1):
        vehicle = VehicleConfig(
            name=f"PX4_Drone{i}",
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
        config.vehicles.append(vehicle)
    
    config.pawn_paths["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
    
    return config

def create_mixed_vehicle_config() -> SimulationConfig:
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

  # Generate mixed vehicle configuration
  python unified_generator.py mixed

  # Custom output directory
  python unified_generator.py single --output-dir ./my_configs

  # Generate only settings.json
  python unified_generator.py single --settings-only

  # Generate only docker-compose.yml
  python unified_generator.py single --docker-only
        """
    )
    
    parser.add_argument(
        "mode",
        choices=["single", "multi", "mixed", "custom"],
        help="Configuration mode"
    )
    
    parser.add_argument(
        "--num-drones",
        type=int,
        default=3,
        help="Number of drones for multi mode (default: 3)"
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
        "--gps-location",
        nargs=3,
        metavar=("LAT", "LON", "ALT"),
        type=float,
        help="GPS origin (latitude longitude altitude)"
    )
    
    args = parser.parse_args()
    
    # Create configuration based on mode
    if args.mode == "single":
        config = create_single_drone_config()
    elif args.mode == "multi":
        if args.num_drones < 1 or args.num_drones > 10:
            print("Error: Number of drones must be between 1 and 10")
            sys.exit(1)
        config = create_multi_drone_config(args.num_drones, args.layout)
    elif args.mode == "mixed":
        config = create_mixed_vehicle_config()
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
    
    # Create output directory
    output_path = Path(args.output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Initialize generator
    generator = ConfigGenerator(config)
    
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
    
    # Generate docker-compose.yml
    if not args.settings_only:
        compose = generator.generate_docker_compose()
        compose_file = output_path / "docker-compose.yml"
        
        with open(compose_file, 'w') as f:
            yaml.dump(compose, f, default_flow_style=False, sort_keys=False)
            
        print(f"✅ Generated: {compose_file}")
        
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
    
    # Print summary
    print("\n📋 Configuration Summary")
    print("=" * 50)
    print(f"Mode: {args.mode}")
    print(f"Vehicles: {len(config.vehicles)}")
    
    for vehicle in config.vehicles:
        print(f"\n{vehicle.name}:")
        print(f"  Type: {vehicle.vehicle_type.value}")
        print(f"  Position: X={vehicle.position.x}, Y={vehicle.position.y}, Z={vehicle.position.z}")
        if vehicle.docker_enabled:
            print(f"  Ports: TCP={vehicle.ports.tcp_port}, QGC={vehicle.ports.qgc_port}")
    
    print("\n💡 Next Steps:")
    print("1. Start Docker containers: ./launch.bat (or ./launch.sh)")
    print("2. Launch AirSim/Unreal Engine")
    print("3. Vehicles will connect automatically!")
    
if __name__ == "__main__":
    main()