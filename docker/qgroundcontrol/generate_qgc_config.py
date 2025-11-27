#!/usr/bin/env python3
"""
QGroundControl Configuration Generator
Automatically creates QGC comm links based on PX4 drones in docker-compose.yml

This script parses docker-compose-px4.yml and generates a QGroundControl.ini
file with pre-configured UDP communication links for each PX4 drone.

Usage:
    python3 generate_qgc_config.py

Output:
    qgc_config/QGroundControl.ini - Ready to mount into QGC container
"""

import yaml
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

class QGCConfigGenerator:
    """Generate QGroundControl configuration from docker-compose files"""

    def __init__(self, compose_file: str):
        self.compose_file = compose_file
        self.px4_drones: List[Dict] = []

    def parse_compose_file(self) -> None:
        """Parse docker-compose file and extract PX4 drone configurations"""
        print(f"Parsing {self.compose_file}...")

        with open(self.compose_file, 'r') as f:
            compose_data = yaml.safe_load(f)

        services = compose_data.get('services', {})

        for service_name, service_config in services.items():
            # Only process PX4 drone services
            if service_name.startswith('px4-bridge-drone-') or service_name.startswith('px4-drone-'):
                drone_info = self._extract_drone_info(service_name, service_config)
                if drone_info:
                    self.px4_drones.append(drone_info)
                    print(f"  Found: {service_name} at {drone_info['ip']}:{drone_info['port']}")

        print(f"Found {len(self.px4_drones)} PX4 drones\n")

    def _extract_drone_info(self, service_name: str, config: Dict) -> Dict:
        """Extract IP address and MAVLink port from service configuration"""
        drone_info = {
            'name': service_name,
            'ip': None,
            'port': 14550,  # Default MAVLink GCS port
            'instance': 0
        }

        # Extract IP address from networks configuration
        networks = config.get('networks', {})
        if isinstance(networks, dict):
            for network_name, network_config in networks.items():
                if isinstance(network_config, dict):
                    drone_info['ip'] = network_config.get('ipv4_address')
                    break

        # Extract PX4_INSTANCE from environment
        env_vars = config.get('environment', {})
        if isinstance(env_vars, dict):
            drone_info['instance'] = env_vars.get('PX4_INSTANCE', 0)

        # Extract MAVLink port from exposed ports
        # PX4 drones typically expose 14540+instance for GCS
        ports = config.get('ports', [])
        for port_mapping in ports:
            if isinstance(port_mapping, str):
                # Format: "14540:14540/udp" or "14541:14541/udp"
                if '/udp' in port_mapping and ('1454' in port_mapping or '1455' in port_mapping):
                    container_port = port_mapping.split(':')[1].split('/')[0]
                    drone_info['port'] = int(container_port)
                    break

        return drone_info if drone_info['ip'] else None

    def generate_qgc_ini(self, output_dir: str) -> str:
        """Generate QGroundControl.ini with comm links for all drones using Qt INI format"""
        os.makedirs(output_dir, exist_ok=True)
        config_file = os.path.join(output_dir, 'QGroundControl.ini')

        print(f"Generating QGroundControl config at: {config_file}")

        with open(config_file, 'w') as f:
            # Write header
            f.write("; QGroundControl Configuration\n")
            f.write("; Auto-generated for multi-drone PX4/AirSim setup\n")
            f.write("; DO NOT EDIT MANUALLY - regenerate using generate_qgc_config.py\n\n")

            # General settings (Qt INI format)
            f.write("[General]\n")
            f.write("SettingsVersion=9\n")
            f.write("version=1\n\n")

            # LinkConfigurations section (Qt INI format with Link0\, Link1\, etc.)
            f.write("[LinkConfigurations]\n")

            for idx, drone in enumerate(self.px4_drones):
                # Qt INI format uses Link0\key notation (backslash escaped to \\)
                f.write(f"Link{idx}\\name={drone['name']}\n")
                f.write(f"Link{idx}\\type=1\n")  # 1 = UDP link type
                f.write(f"Link{idx}\\auto=true\n")
                f.write(f"Link{idx}\\autoConnectAllowed=true\n")
                f.write(f"Link{idx}\\high_latency=false\n")

                # UDP specific settings
                f.write(f"Link{idx}\\host={drone['ip']}\n")
                f.write(f"Link{idx}\\port={drone['port']}\n")
                f.write(f"Link{idx}\\localPort=14550\n")  # QGC listens on 14550
                f.write(f"Link{idx}\\hostCount=1\n")

            f.write(f"count={len(self.px4_drones)}\n\n")

            # LinkManager section
            f.write("[LinkManager]\n")
            f.write(f"count={len(self.px4_drones)}\n\n")

        print(f"✓ Configuration written successfully\n")
        return config_file

    def print_summary(self) -> None:
        """Print configuration summary"""
        print("=" * 60)
        print("QGroundControl Comm Links Configuration Summary")
        print("=" * 60)

        for idx, drone in enumerate(self.px4_drones, start=1):
            print(f"{idx}. {drone['name']}")
            print(f"   IP: {drone['ip']}")
            print(f"   Port: {drone['port']}")
            print(f"   Instance: {drone['instance']}")
            print()

        print("=" * 60)
        print()
        print("⚠️  IMPORTANT: BRIDGE NETWORK MODE LIMITATIONS")
        print("=" * 60)
        print()
        print("This configuration is for BRIDGE network mode only.")
        print()
        print("Bridge Network Limitations:")
        print("  ❌ UDP broadcast auto-discovery does NOT work")
        print("  ❌ Each drone must be manually configured")
        print("  ❌ QGC connects via direct IP (172.20.0.x)")
        print("  ❌ Higher latency (~0.5-1.0ms)")
        print()
        print("=" * 60)
        print()
        print("✅ RECOMMENDED: Use Hybrid Network Mode Instead")
        print("=" * 60)
        print()
        print("Hybrid Mode Benefits:")
        print("  ✓ 70-80% lower network latency (0.1-0.3ms)")
        print("  ✓ 30-40% lower CPU usage")
        print("  ✓ UDP broadcast auto-discovery works natively")
        print("  ✓ Zero configuration needed - drones appear automatically")
        print("  ✓ Better ROS2 DDS multicast support")
        print()
        print("To switch to hybrid mode:")
        print("  cd /home/mnsuser/Cosys_Airsim_Exploration/docker")
        print("  ./toggle-network-mode.sh hybrid")
        print()
        print("Then start with hybrid override:")
        print("  docker compose -f docker-compose-master.yml \\")
        print("                 -f docker-compose-hybrid-override.yml \\")
        print("                 --profile linux-integrated up")
        print()
        print("=" * 60)
        print()
        print("If you still want to use bridge mode:")
        print("  1. Configuration already generated at:")
        print("     ./qgroundcontrol/qgc_config/QGroundControl.ini")
        print("  2. Config will be auto-injected on QGC startup (read-only mount)")
        print("  3. Rebuild QGC: docker compose build qgroundcontrol-x11")
        print("  4. Start: docker compose --profile linux-integrated up")
        print()
        print("=" * 60)


def main():
    """Main execution function"""
    # Determine paths
    script_dir = Path(__file__).parent
    docker_dir = script_dir.parent
    compose_file = docker_dir / "px4_airsim_docker" / "docker-compose-px4.yml"
    output_dir = script_dir / "qgc_config"

    # Check if compose file exists
    if not compose_file.exists():
        print(f"ERROR: {compose_file} not found!")
        print(f"Current directory: {os.getcwd()}")
        print(f"Script directory: {script_dir}")
        sys.exit(1)

    # Generate configuration
    generator = QGCConfigGenerator(str(compose_file))
    generator.parse_compose_file()

    if not generator.px4_drones:
        print("WARNING: No PX4 drones found in docker-compose file")
        sys.exit(1)

    config_file = generator.generate_qgc_ini(str(output_dir))
    generator.print_summary()

    print(f"\n✓ Configuration generated successfully: {config_file}")


if __name__ == '__main__':
    main()
