#!/usr/bin/env python3
"""
Ultra-Swarm Docker Compose Generator
Dynamically generates docker-compose files for variable swarm configurations
Supports 1-3 swarms with 1-9 drones each and GPS configuration
"""

import argparse
import json
import os
import sys
from datetime import datetime
from typing import Dict, List, Tuple

class UltraSwarmComposeGenerator:
    def __init__(self):
        self.swarm_colors = {
            1: {"name": "Blue Team", "emoji": "🔵"},
            2: {"name": "Red Team", "emoji": "🔴"},
            3: {"name": "Green Team", "emoji": "🟢"}
        }
        
        self.base_gps_locations = {
            1: {"lat": 47.641468, "lon": -122.140165, "location": "Seattle area"},
            2: {"lat": 47.642468, "lon": -122.139165, "location": "Bellevue area"},
            3: {"lat": 47.643468, "lon": -122.138165, "location": "Redmond area"}
        }
    
    def generate_port_allocation(self, num_swarms: int, drones_per_swarm: int) -> Dict:
        """Generate port allocation for the given configuration"""
        port_config = {}
        
        for swarm_id in range(1, num_swarms + 1):
            base_airsim = 4560 + (swarm_id - 1) * 10
            base_qgc = 14549 + (swarm_id - 1) * 10  
            base_mavlink = 18569 + (swarm_id - 1) * 10
            
            port_config[f"swarm_{swarm_id}"] = {
                "airsim_tcp_range": f"{base_airsim + 1}-{base_airsim + drones_per_swarm}",
                "qgc_udp_range": f"{base_qgc + 1}-{base_qgc + drones_per_swarm}",
                "mavlink_udp_range": f"{base_mavlink + 1}-{base_mavlink + drones_per_swarm}"
            }
        
        return port_config
    
    def generate_gps_config(self, num_swarms: int, drones_per_swarm: int) -> Dict:
        """Generate GPS configuration for all drones"""
        gps_config = {
            "description": f"GPS home locations for {num_swarms}-swarm configuration",
            "coordinate_system": "WGS84",
            "altitude_reference": "MSL",
            "swarms": {},
            "port_allocation": self.generate_port_allocation(num_swarms, drones_per_swarm)
        }
        
        for swarm_id in range(1, num_swarms + 1):
            base_location = self.base_gps_locations[swarm_id]
            swarm_info = self.swarm_colors[swarm_id]
            
            swarm_config = {
                "name": swarm_info["name"],
                "description": f"{base_location['location']} - {'Primary' if swarm_id == 1 else 'Secondary' if swarm_id == 2 else 'Tertiary'} swarm location",
                "home": {
                    "latitude": base_location["lat"],
                    "longitude": base_location["lon"],
                    "altitude": 0.0
                },
                "formation_spacing": {
                    "x_offset_per_drone": 0.0001,
                    "y_offset_per_drone": 0.0001,
                    "description": "Each drone offset by ~11m in X and Y"
                },
                "drones": {}
            }
            
            # Generate individual drone positions in a 3x3 grid
            for drone_id in range(1, drones_per_swarm + 1):
                row = (drone_id - 1) // 3
                col = (drone_id - 1) % 3
                
                lat_offset = -row * 0.0001  # Move south for each row
                lon_offset = col * 0.0001   # Move east for each column
                
                swarm_config["drones"][str(drone_id)] = {
                    "lat": base_location["lat"] + lat_offset,
                    "lon": base_location["lon"] + lon_offset,
                    "alt": 0.0
                }
            
            gps_config["swarms"][str(swarm_id)] = swarm_config
        
        gps_config["notes"] = {
            "coordinate_offsets": f"Each swarm is offset by ~100m from the others",
            "drone_spacing": "Drones within each swarm are spaced ~11m apart",
            "altitude": "All drones start at sea level (0m MSL)",
            "usage": "Use these coordinates in AirSim settings.json and PX4 parameters"
        }
        
        return gps_config
    
    def generate_compose_header(self, num_swarms: int, drones_per_swarm: int) -> str:
        """Generate the header section of the docker-compose file"""
        total_drones = num_swarms * drones_per_swarm
        
        port_ranges = []
        for swarm_id in range(1, num_swarms + 1):
            base_port = 4560 + (swarm_id - 1) * 10
            port_ranges.append(f"Swarm {swarm_id}: AirSim TCP {base_port + 1}-{base_port + drones_per_swarm}")
        
        header = f"""# Ultra-Swarm Docker Compose Configuration
# Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# Supports {total_drones} drones: {drones_per_swarm} drones per swarm across {num_swarms} swarm{'s' if num_swarms > 1 else ''}
# Includes GPS home location fixes and proper port allocation
# 
# Port allocation:
"""
        
        for swarm_id in range(1, num_swarms + 1):
            base_airsim = 4560 + (swarm_id - 1) * 10
            base_qgc = 14549 + (swarm_id - 1) * 10
            base_mavlink = 18569 + (swarm_id - 1) * 10
            
            header += f"#   Swarm {swarm_id}: AirSim TCP {base_airsim + 1}-{base_airsim + drones_per_swarm}, QGC UDP {base_qgc + 1}-{base_qgc + drones_per_swarm}, MAVLink UDP {base_mavlink + 1}-{base_mavlink + drones_per_swarm}\n"
        
        return header
    
    def generate_networks_and_volumes(self) -> str:
        """Generate networks and volumes section"""
        return """
networks:
  ultra-swarm-network:
    name: ultra-swarm-network
    driver: bridge
    ipam:
      driver: default
      config:
        - subnet: 172.27.0.0/16
          gateway: 172.27.0.1

volumes:
  ultra-swarm-data:
    driver: local
  qgc-config:
    driver: local

x-px4-common: &px4-common
  build:
    context: .
    dockerfile: Dockerfile.ultra-swarm
  image: px4-ultra-swarm:latest
  environment: &px4-env
    PX4_HOME_LAT: 47.641468
    PX4_HOME_LON: -122.140165
    PX4_HOME_ALT: 0.0
    PX4_SYS_AUTOSTART: 10016
    PX4_SIM_HOSTNAME: host.docker.internal
    PX4_SIM_MODEL: none_iris
    MAX_SWARMS: 3
    MAX_DRONES_PER_SWARM: 9
  networks:
    - ultra-swarm-network
  volumes:
    - ultra-swarm-data:/px4_data
    - ./logs:/px4_workspace/PX4-Autopilot/build/px4_sitl_default/logs
  restart: unless-stopped
  tty: true
  stdin_open: true
  deploy:
    resources:
      limits:
        cpus: '0.8'
        memory: 512M
"""
    
    def generate_swarm_services(self, num_swarms: int, drones_per_swarm: int) -> str:
        """Generate all swarm services"""
        services = "\nservices:\n"
        
        for swarm_id in range(1, num_swarms + 1):
            swarm_info = self.swarm_colors[swarm_id]
            
            services += f"  # ==========================================\n"
            services += f"  # SWARM {swarm_id} - {swarm_info['name'].upper()} ({drones_per_swarm} drones: "
            
            base_port = 4560 + (swarm_id - 1) * 10
            services += f"{base_port + 1}-{base_port + drones_per_swarm})\n"
            services += f"  # ==========================================\n"
            services += f"  \n"
            
            for drone_id in range(1, drones_per_swarm + 1):
                services += self.generate_drone_service(swarm_id, drone_id, swarm_info, drones_per_swarm)
                services += "\n"
        
        return services
    
    def generate_drone_service(self, swarm_id: int, drone_id: int, swarm_info: Dict, drones_per_swarm: int) -> str:
        """Generate a single drone service definition"""
        container_name = f"px4-swarm-{swarm_id}-drone-{drone_id}"
        
        # Calculate ports
        base_airsim = 4560 + (swarm_id - 1) * 10
        base_qgc = 14549 + (swarm_id - 1) * 10
        base_mavlink = 18569 + (swarm_id - 1) * 10
        
        airsim_port = base_airsim + drone_id
        qgc_port = base_qgc + drone_id
        mavlink_port = base_mavlink + drone_id
        
        service = f"""  {container_name}:
    <<: *px4-common
    container_name: {container_name}
    environment:
      <<: *px4-env
      SWARM_ID: {swarm_id}
      PX4_INSTANCE: {drone_id}
      SWARM_SIZE: {drones_per_swarm}
    expose:
      - "{airsim_port}/tcp"
      - "{qgc_port}/udp"
      - "{mavlink_port}/udp"
    command: >
      bash -c "
        echo '{swarm_info["emoji"]} Swarm {swarm_id} - Drone {drone_id} starting on AirSim port {airsim_port}...'
        /Scripts/run_ultra_swarm.sh
      \""""
        
        # Add profiles for swarms 2 and 3
        if swarm_id > 1:
            service += f"""
    profiles:
      - swarm{swarm_id}"""
        
        return service
    
    def generate_support_services(self) -> str:
        """Generate support services (QGC, debug)"""
        return """
  # ==========================================
  # SUPPORT SERVICES
  # ==========================================

  # Ground Control Station for monitoring all swarms
  qgroundcontrol:
    image: linuxserver/qgroundcontrol:latest
    container_name: qgc-ultra-swarm
    environment:
      - PUID=1000
      - PGID=1000
      - TZ=UTC
    ports:
      - "8080:8080"  # Web interface
    volumes:
      - qgc-config:/config
    networks:
      - ultra-swarm-network
    profiles:
      - gcs

  # Debug container for troubleshooting ultra-swarms
  px4-debug:
    <<: *px4-common
    container_name: px4-debug-ultra-swarm
    command: ["/bin/bash"]
    environment:
      <<: *px4-env
      PX4_INSTANCE: 999
      SWARM_ID: 99
    profiles:
      - debug
"""
    
    def generate_compose_file(self, num_swarms: int, drones_per_swarm: int) -> str:
        """Generate the complete docker-compose file"""
        compose_content = ""
        compose_content += self.generate_compose_header(num_swarms, drones_per_swarm)
        compose_content += self.generate_networks_and_volumes()
        compose_content += self.generate_swarm_services(num_swarms, drones_per_swarm)
        compose_content += self.generate_support_services()
        
        return compose_content
    
    def save_files(self, compose_content: str, gps_config: Dict, output_dir: str, filename_prefix: str):
        """Save generated files to disk"""
        os.makedirs(output_dir, exist_ok=True)
        
        # Save docker-compose file
        compose_file = os.path.join(output_dir, f"{filename_prefix}.yml")
        with open(compose_file, 'w') as f:
            f.write(compose_content)
        
        # Save GPS configuration
        gps_file = os.path.join(output_dir, f"{filename_prefix}_gps.json")
        with open(gps_file, 'w') as f:
            json.dump(gps_config, f, indent=2)
        
        return compose_file, gps_file

def main():
    parser = argparse.ArgumentParser(
        description="Generate Ultra-Swarm Docker Compose configurations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --swarms 1 --drones 3                    # Single swarm, 3 drones
  %(prog)s --swarms 2 --drones 9                    # Two swarms, 9 drones each
  %(prog)s --swarms 3 --drones 9 --output custom/   # Full ultra-swarm in custom directory
  %(prog)s --preset single                          # Use preset: single drone
  %(prog)s --preset test                             # Use preset: 3-drone test
  %(prog)s --preset ultra-max                       # Use preset: full 27-drone configuration
        """
    )
    
    # Configuration options
    parser.add_argument('--swarms', type=int, choices=[1, 2, 3], default=1,
                       help='Number of swarms (1-3)')
    parser.add_argument('--drones', type=int, choices=range(1, 10), default=1,
                       help='Number of drones per swarm (1-9)')
    
    # Preset configurations
    parser.add_argument('--preset', choices=['single', 'test', 'full-swarm', 'dual-swarms', 'ultra-max'],
                       help='Use predefined configuration preset')
    
    # Output options
    parser.add_argument('--output', default='.',
                       help='Output directory (default: current directory)')
    parser.add_argument('--filename', default='docker-compose-generated',
                       help='Output filename prefix (default: docker-compose-generated)')
    
    # Additional options
    parser.add_argument('--dry-run', action='store_true',
                       help='Show configuration without generating files')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed output')
    
    args = parser.parse_args()
    
    # Handle presets
    if args.preset:
        preset_configs = {
            'single': (1, 1),
            'test': (1, 3),
            'full-swarm': (1, 9),
            'dual-swarms': (2, 9),
            'ultra-max': (3, 9)
        }
        args.swarms, args.drones = preset_configs[args.preset]
        if args.verbose:
            print(f"Using preset '{args.preset}': {args.swarms} swarm(s), {args.drones} drones per swarm")
    
    # Validate configuration
    total_drones = args.swarms * args.drones
    if total_drones > 27:
        print(f"❌ Error: Configuration exceeds maximum of 27 drones (requested: {total_drones})")
        sys.exit(1)
    
    # Initialize generator
    generator = UltraSwarmComposeGenerator()
    
    # Show configuration
    print(f"🚁 Ultra-Swarm Configuration Generator")
    print(f"   Swarms: {args.swarms}")
    print(f"   Drones per swarm: {args.drones}")
    print(f"   Total drones: {total_drones}")
    print(f"   Output directory: {args.output}")
    
    if args.dry_run:
        print("   Mode: Dry run (no files will be generated)")
        print("\n📊 Port Allocation Preview:")
        port_config = generator.generate_port_allocation(args.swarms, args.drones)
        for swarm, ports in port_config.items():
            swarm_num = int(swarm.split('_')[1])
            color_info = generator.swarm_colors[swarm_num]
            print(f"   {color_info['emoji']} Swarm {swarm_num}: AirSim {ports['airsim_tcp_range']}")
        return
    
    # Generate configuration
    print("\n🔧 Generating configuration files...")
    
    compose_content = generator.generate_compose_file(args.swarms, args.drones)
    gps_config = generator.generate_gps_config(args.swarms, args.drones)
    
    # Save files
    compose_file, gps_file = generator.save_files(
        compose_content, gps_config, args.output, args.filename
    )
    
    print(f"✅ Files generated successfully:")
    print(f"   📄 Docker Compose: {compose_file}")
    print(f"   📍 GPS Config: {gps_file}")
    
    print(f"\n🚀 Usage:")
    print(f"   cd {args.output}")
    print(f"   docker-compose -f {os.path.basename(compose_file)} up")
    
    if args.swarms > 1:
        print(f"\n   # Start additional swarms:")
        for swarm_id in range(2, args.swarms + 1):
            print(f"   docker-compose -f {os.path.basename(compose_file)} --profile swarm{swarm_id} up")

if __name__ == "__main__":
    main()