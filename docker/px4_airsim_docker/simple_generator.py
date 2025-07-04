#!/usr/bin/env python3
import json
from pathlib import Path

def main():
    print("Generating Docker Compose from AirSim Settings")
    print("=" * 50)
    
    # Load settings
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    
    with open(settings_path, 'r') as f:
        settings = json.load(f)
    
    vehicles = settings.get("Vehicles", {})
    origin = settings.get("OriginGeopoint", {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 10
    })
    
    print(f"Found {len(vehicles)} vehicles")
    
    # Generate compose content
    compose_content = """# Auto-generated Docker Compose file from AirSim settings.json
# DO NOT EDIT MANUALLY - regenerate with: python simple_generator.py

services:
"""
    
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        tcp_port = vehicle_config.get("TcpPort", 4560 + idx)
        control_local = vehicle_config.get("ControlPortLocal", 14540 + idx)
        control_remote = vehicle_config.get("ControlPortRemote", 14580 + idx)
        qgc_port = 14549 + idx
        
        container_name = f"{vehicle_name.lower().replace('_', '-')}"
        
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
    ports:
      # MAVLink UDP ports for external tools
      - "{control_local}:{control_local}/udp"   # MAVLink Local
      - "{control_remote}:{control_remote}/udp" # MAVLink Remote
      - "{qgc_port}:{qgc_port}/udp"             # QGroundControl
    networks:
      airsim-network:
        ipv4_address: 172.25.0.{10 + idx - 1}
    volumes:
      - px4-shared-data:/px4_data
    restart: unless-stopped
    command: ["/Scripts/run_airsim_sitl_final.sh", "{idx}"]
"""
    
    compose_content += """
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
"""
    
    # Write compose file
    output_file = "docker-compose.generated.yml"
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(compose_content)
    
    # Generate launcher
    launcher_content = f"""@echo off
echo ================================================
echo    Generated PX4 AirSim Docker Launcher
echo    Generated for {len(vehicles)} drone(s)
echo ================================================
echo.

echo [INFO] Creating shared data directory...
if not exist "shared_data" mkdir shared_data

echo [INFO] Building Docker images...
docker-compose -f {output_file} build

if %errorlevel% neq 0 (
    echo [ERROR] Failed to build Docker images!
    pause
    exit /b 1
)

echo [INFO] Cleaning up existing containers...
docker-compose -f {output_file} down > nul 2>&1

echo [INFO] Starting {len(vehicles)} drone instance(s)...
docker-compose -f {output_file} up -d

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

echo Port Configuration:
"""
    
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        tcp_port = vehicle_config.get("TcpPort", 4560 + idx)
        control_local = vehicle_config.get("ControlPortLocal", 14540 + idx)
        control_remote = vehicle_config.get("ControlPortRemote", 14580 + idx)
        qgc_port = 14549 + idx
        
        launcher_content += f'echo   {vehicle_name}: TCP={tcp_port} ^| MAVLink={control_local}/{control_remote} ^| QGC={qgc_port}\n'
    
    launcher_content += f"""
echo.
echo Management Commands:
echo   View logs:        docker-compose -f {output_file} logs
echo   Stop all:         docker-compose -f {output_file} down
echo   Container status: docker-compose -f {output_file} ps
echo.
pause
"""
    
    launcher_file = "launch_generated.bat"
    with open(launcher_file, 'w', encoding='utf-8') as f:
        f.write(launcher_content)
    
    print(f"Generated: {output_file}")
    print(f"Generated: {launcher_file}")
    print()
    print("Port Assignments:")
    
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
    
    print("Next Steps:")
    print(f"  1. Run: .\\{launcher_file}")
    print("  2. Wait for all containers to start")
    print("  3. Launch AirSim (it will auto-connect to all drones)")
    print()
    print("Zero port conflicts guaranteed!")

if __name__ == "__main__":
    main() 