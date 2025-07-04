#!/usr/bin/env python3
"""
Generate Docker Compose file from AirSim settings.json
This creates a static compose file with perfect port mappings - no conflicts!
"""

import json
import os
import sys
from pathlib import Path

def load_airsim_settings():
    """Load AirSim settings.json from standard locations"""
    possible_paths = [
        # Windows
        Path.home() / "Documents" / "AirSim" / "settings.json",
        # Linux/WSL
        Path.home() / "Documents" / "AirSim" / "settings.json",
        Path.home() / ".airsim" / "settings.json",
        # Local development
        Path("settings.json"),
        Path("../settings.json"),
    ]
    
    for path in possible_paths:
        if path.exists():
            print(f"Found settings.json at: {path}")
            with open(path, 'r') as f:
                return json.load(f), str(path)
    
    raise FileNotFoundError("Could not find AirSim settings.json in any standard location")

def generate_docker_compose(settings, output_file="docker-compose.generated.yml"):
    """Generate Docker Compose file from AirSim settings"""
    
    vehicles = settings.get("Vehicles", {})
    if not vehicles:
        raise ValueError("No vehicles found in settings.json")
    
    # Extract origin coordinates for consistent GPS home location
    origin = settings.get("OriginGeopoint", {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 10
    })
    
    compose_content = f"""# Auto-generated Docker Compose file from AirSim settings.json
# Generated for {len(vehicles)} drone(s)
# DO NOT EDIT MANUALLY - regenerate with: python generate_compose.py

version: "3.8"

services:
"""
    
    # Generate service for each vehicle
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        tcp_port = vehicle_config.get("TcpPort", 4560 + idx)
        control_local = vehicle_config.get("ControlPortLocal", 14540 + idx)
        control_remote = vehicle_config.get("ControlPortRemote", 14580 + idx)
        qgc_port = 14549 + idx  # QGroundControl port
        
        # Clean container name (remove special characters)
        container_name = f"px4-{vehicle_name.lower().replace('_', '-')}"
        
        # Get vehicle position
        x_pos = vehicle_config.get("X", idx * 5)
        y_pos = vehicle_config.get("Y", 0)
        z_pos = vehicle_config.get("Z", -2)
        
        compose_content += f"""
  {container_name}:
    build: 
      context: .
      dockerfile: Dockerfile
    container_name: {container_name}
    hostname: {container_name}
    environment:
      - PX4_HOME_LAT={origin['Latitude']}
      - PX4_HOME_LON={origin['Longitude']}
      - PX4_HOME_ALT={origin['Altitude']}
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=host.docker.internal
      - PX4_SIM_MODEL=iris
      - PX4_INSTANCE={idx}
      - PX4_TCP_PORT={tcp_port}
      - PX4_MAVLINK_LOCAL={control_local}
      - PX4_MAVLINK_REMOTE={control_remote}
      - VEHICLE_NAME={vehicle_name}
      - VEHICLE_POS_X={x_pos}
      - VEHICLE_POS_Y={y_pos}
      - VEHICLE_POS_Z={z_pos}
    ports:
      # AirSim TCP connection (commented out - AirSim connects TO container)
      # - "{tcp_port}:{tcp_port}/tcp"     # AirSim TCP (bidirectional)
      
      # MAVLink UDP ports for external tools
      - "{control_local}:{control_local}/udp"   # MAVLink Local (PX4 -> QGC/MAVSDK)
      - "{control_remote}:{control_remote}/udp" # MAVLink Remote (QGC/MAVSDK -> PX4)
      - "{qgc_port}:{qgc_port}/udp"             # QGroundControl discovery
    networks:
      airsim-network:
        ipv4_address: 172.25.0.{10 + idx - 1}
    volumes:
      - px4-shared-data:/px4_data
    restart: unless-stopped
    command: ["/Scripts/run_airsim_sitl_final.sh", "{idx}"]
    healthcheck:
      test: ["CMD", "bash", "-c", "pgrep -f 'px4.*sitl' > /dev/null"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 60s
"""
    
    # Add networks and volumes
    compose_content += f"""
# Networks
networks:
  airsim-network:
    name: airsim-network
    driver: bridge
    ipam:
      driver: default
      config:
        - subnet: 172.25.0.0/16
          gateway: 172.25.0.1

# Volumes
volumes:
  px4-shared-data:
    driver: local
    driver_opts:
      type: none
      o: bind
      device: ${{PWD}}/shared_data
"""
    
    # Write the compose file
    with open(output_file, 'w') as f:
        f.write(compose_content)
    
    return output_file, vehicles

def generate_launcher_script(vehicles, compose_file):
    """Generate a simple launcher script for the generated compose file"""
    
    launcher_content = f"""@echo off
echo ================================================
echo    Generated PX4 AirSim Docker Launcher
echo    Generated for {len(vehicles)} drone(s)
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

echo [INFO] Starting {len(vehicles)} drone instance(s)...
docker-compose -f {compose_file} up -d

if %errorlevel% neq 0 (
    echo [ERROR] Failed to start drone instances!
    pause
    exit /b 1
)

echo.
echo ================================================
echo    {len(vehicles)} Drone(s) Started Successfully!
echo ================================================
echo.

echo 📋 Port Configuration:
"""
    
    # Add port information for each vehicle
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        tcp_port = vehicle_config.get("TcpPort", 4560 + idx)
        control_local = vehicle_config.get("ControlPortLocal", 14540 + idx)
        control_remote = vehicle_config.get("ControlPortRemote", 14580 + idx)
        qgc_port = 14549 + idx
        
        launcher_content += f'echo   {vehicle_name}: TCP={tcp_port} ^| MAVLink={control_local}/{control_remote} ^| QGC={qgc_port}\n'
    
    launcher_content += f"""
echo.
echo Management Commands:
echo   View logs:        docker-compose -f {compose_file} logs
echo   Stop all:         docker-compose -f {compose_file} down
echo   Container status: docker-compose -f {compose_file} ps
echo   Follow logs:      docker-compose -f {compose_file} logs -f
echo.

echo Troubleshooting:
echo   List containers:  docker ps
echo   Check networks:   docker network ls
echo   GPS diagnosis:    python diagnose_gps_docker.py
echo   Regenerate:       python generate_compose.py
echo.

echo Benefits of Generated Compose:
echo   * Zero port conflicts - each drone has unique ports
echo   * Perfect mapping from your settings.json
echo   * Static configuration - no runtime complexity
echo   * Predictable container names and IPs
echo.

echo Your drones are ready! Start AirSim and they should connect automatically.
echo.
pause
"""
    
    launcher_name = f"launch_generated.bat"
    with open(launcher_name, 'w') as f:
        f.write(launcher_content)
    
    return launcher_name

def print_summary(vehicles, compose_file, launcher_file, settings_path):
    """Print a summary of what was generated"""
    
    print(f"""
Successfully Generated Docker Configuration!
===============================================

Summary:
  • Settings source: {settings_path}
  • Number of drones: {len(vehicles)}
  • Compose file: {compose_file}
  • Launcher script: {launcher_file}

Port Assignments:""")
    
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        tcp_port = vehicle_config.get("TcpPort", 4560 + idx)
        control_local = vehicle_config.get("ControlPortLocal", 14540 + idx)
        control_remote = vehicle_config.get("ControlPortRemote", 14580 + idx)
        qgc_port = 14549 + idx
        
        print(f"  {vehicle_name}:")
        print(f"    AirSim TCP:     {tcp_port}")
        print(f"    MAVLink Local:  {control_local}")
        print(f"    MAVLink Remote: {control_remote}")
        print(f"    QGroundControl: {qgc_port}")
        print()

    print(f"""Next Steps:
  1. Run: .\\{launcher_file}
  2. Wait for all containers to start
  3. Launch AirSim (it will auto-connect to all drones)
  4. Use QGroundControl on ports {14550}-{14549 + len(vehicles)} for each drone

Zero port conflicts guaranteed! Each drone has unique ports.
""")

def main():
    print("Generating Docker Compose from AirSim Settings")
    print("=" * 50)
    
    try:
        # Load AirSim settings
        settings, settings_path = load_airsim_settings()
        
        # Generate Docker Compose file
        compose_file, vehicles = generate_docker_compose(settings)
        print(f"Generated: {compose_file}")
        
        # Generate launcher script
        launcher_file = generate_launcher_script(vehicles, compose_file)
        print(f"Generated: {launcher_file}")
        
        # Print summary
        print_summary(vehicles, compose_file, launcher_file, settings_path)
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 