#!/usr/bin/env python3
"""
Network diagnostic tool for PX4-AirSim Docker setup
Analyzes IP configurations and provides recommendations
"""

import json
import subprocess
import socket
from pathlib import Path

def get_docker_host_ip():
    """Get the actual IP address that containers should use to reach the host"""
    try:
        # Try to get the Docker bridge gateway IP
        result = subprocess.run(
            ['docker', 'network', 'inspect', 'airsim-network'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            import json
            network_info = json.loads(result.stdout)
            gateway = network_info[0]['IPAM']['Config'][0]['Gateway']
            print(f"Docker airsim-network gateway: {gateway}")
            return gateway
    except Exception as e:
        print(f"Could not get Docker network info: {e}")
    
    # Fallback: get the host's primary IP
    try:
        # Connect to a remote address to determine the local IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            host_ip = s.getsockname()[0]
            print(f"Host primary IP: {host_ip}")
            return host_ip
    except Exception as e:
        print(f"Could not determine host IP: {e}")
        return "127.0.0.1"

def check_current_settings():
    """Check current AirSim settings configuration"""
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    
    if not settings_path.exists():
        print("❌ No settings.json found!")
        return None
    
    with open(settings_path, 'r') as f:
        settings = json.load(f)
    
    print("📋 Current AirSim Settings Analysis")
    print("=" * 50)
    
    vehicles = settings.get("Vehicles", {})
    print(f"Number of vehicles: {len(vehicles)}")
    
    for name, config in vehicles.items():
        print(f"\n🚁 {name}:")
        print(f"  TcpPort: {config.get('TcpPort', 'Not set')}")
        print(f"  ControlIp: {config.get('ControlIp', 'Not set')}")
        print(f"  LocalHostIp: {config.get('LocalHostIp', 'Not set')}")
        print(f"  ControlPortLocal: {config.get('ControlPortLocal', 'Not set')}")
        print(f"  ControlPortRemote: {config.get('ControlPortRemote', 'Not set')}")
    
    return settings

def check_docker_containers():
    """Check running PX4 containers and their network configuration"""
    try:
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=px4', '--format', 'table {{.Names}}\t{{.Status}}\t{{.Networks}}'],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            print("\n🐳 Docker Container Status")
            print("=" * 50)
            print(result.stdout)
        
        # Get detailed network info for first container
        container_result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=px4', '--format', '{{.Names}}'],
            capture_output=True, text=True
        )
        
        if container_result.returncode == 0:
            containers = container_result.stdout.strip().split('\n')
            if containers and containers[0]:
                container_name = containers[0]
                
                # Get container IP
                ip_result = subprocess.run(
                    ['docker', 'inspect', '-f', '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}', container_name],
                    capture_output=True, text=True
                )
                
                if ip_result.returncode == 0:
                    container_ip = ip_result.stdout.strip()
                    print(f"\n📍 Container {container_name} IP: {container_ip}")
                    
                    # Test connectivity from container to host
                    ping_result = subprocess.run(
                        ['docker', 'exec', container_name, 'ping', '-c', '1', 'host.docker.internal'],
                        capture_output=True, text=True
                    )
                    
                    if ping_result.returncode == 0:
                        print("✅ Container can reach host.docker.internal")
                    else:
                        print("❌ Container cannot reach host.docker.internal")
                        
    except Exception as e:
        print(f"Error checking Docker containers: {e}")

def analyze_mavlink_configuration():
    """Analyze MAVLink configuration issues"""
    print("\n🔗 MAVLink Configuration Analysis")
    print("=" * 50)
    
    # Get Docker bridge IP
    docker_gateway = get_docker_host_ip()
    
    print(f"""
📝 Current Network Architecture:
  • PX4 containers are on airsim-network (172.25.0.0/16)
  • Container gateway: {docker_gateway}
  • AirSim runs on host machine
  • host.docker.internal resolves to: 192.168.65.254

🔍 The Issue:
  PX4 containers are configured to connect to 'host.docker.internal' for AirSim TCP,
  but your settings.json has ControlIp and LocalHostIp set to 127.0.0.1,
  which means AirSim is trying to connect to MAVLink on localhost instead
  of the container's IP address.

💡 Solution Options:""")

def recommend_solution():
    """Provide recommended solution"""
    print("""
🎯 RECOMMENDED SOLUTION:

Option 1: Update settings.json IP addresses (RECOMMENDED)
--------------------------------------------------------
Update your settings.json to use the Docker gateway IP instead of 127.0.0.1:

For each vehicle in settings.json, change:
  "ControlIp": "172.25.0.1"        # Docker gateway IP
  "LocalHostIp": "172.25.0.1"      # Docker gateway IP

This tells AirSim to connect to MAVLink on the Docker network instead of localhost.

Option 2: Use host networking (ALTERNATIVE)
------------------------------------------
Modify Docker Compose to use host networking, but this removes network isolation.

Option 3: Port forwarding (COMPLEX)
-----------------------------------
Set up port forwarding rules, but this is more complex and error-prone.

🔧 IMPLEMENTATION:

1. Stop current containers:
   docker-compose -f docker-compose.generated.yml down

2. Update settings with new script:
   python fix_network_settings.py

3. Restart containers:
   .\launch_generated.bat

4. Test GPS and armDisarm functionality
""")

def main():
    print("🔍 PX4-AirSim Network Diagnostic Tool")
    print("=" * 60)
    
    # Check current settings
    settings = check_current_settings()
    
    # Check Docker setup
    check_docker_containers()
    
    # Analyze MAVLink configuration
    analyze_mavlink_configuration()
    
    # Provide recommendations
    recommend_solution()

if __name__ == "__main__":
    main() 