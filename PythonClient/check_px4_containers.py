#!/usr/bin/env python3

import socket
import subprocess
import sys
import time

def check_port_connectivity(host, port, timeout=2):
    """Check if a port is reachable"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception:
        return False

def check_docker_containers():
    """Check if Docker containers are running"""
    try:
        result = subprocess.run(['docker', 'ps'], capture_output=True, text=True, check=True)
        return result.stdout
    except subprocess.CalledProcessError:
        return None
    except FileNotFoundError:
        print("❌ Docker command not found. Is Docker installed?")
        return None

def main():
    print("🐳 PX4 Container Connectivity Check")
    print("=" * 50)
    
    # Check MAVLink ports
    mavlink_ports = [14541, 14542, 14543, 14544, 14545]
    host = "127.0.0.1"
    
    print(f"\n📡 Checking MAVLink ports on {host}...")
    reachable_ports = []
    
    for i, port in enumerate(mavlink_ports):
        if check_port_connectivity(host, port):
            print(f"  ✅ PX4_Drone{i+1} (port {port}) - REACHABLE")
            reachable_ports.append(port)
        else:
            print(f"  ❌ PX4_Drone{i+1} (port {port}) - NOT REACHABLE")
    
    # Check Docker containers
    print(f"\n🐳 Checking Docker containers...")
    docker_output = check_docker_containers()
    
    if docker_output is None:
        print("❌ Could not check Docker containers")
    else:
        px4_containers = []
        lines = docker_output.split('\n')
        
        for line in lines:
            if 'px4' in line.lower() or 'airsim' in line.lower():
                px4_containers.append(line)
        
        if px4_containers:
            print("✅ Found PX4/AirSim related containers:")
            for container in px4_containers:
                print(f"  {container}")
        else:
            print("❌ No PX4/AirSim containers found running")
    
    # Summary and recommendations
    print(f"\n📋 Summary")
    print("-" * 30)
    print(f"Reachable MAVLink ports: {len(reachable_ports)}/5")
    
    if len(reachable_ports) == 0:
        print("\n❌ No PX4 containers appear to be running")
        print("\n🔧 Troubleshooting Steps:")
        print("1. Start your Docker containers:")
        print("   cd docker/px4_airsim_docker")
        print("   docker-compose up -d")
        print()
        print("2. Or if using a specific launcher:")
        print("   ./launch_fixed.bat explicit 3")
        print()
        print("3. Check container status:")
        print("   docker ps")
        print()
        print("4. Check container logs if they're failing:")
        print("   docker logs <container_name>")
        print()
        print("5. Verify port mappings in docker-compose.yml")
        
    elif len(reachable_ports) < 5:
        print(f"\n⚠️  Only {len(reachable_ports)} out of 5 PX4 containers are reachable")
        print("Some containers may not have started properly.")
        print("\nCheck logs for failed containers:")
        print("docker logs <container_name>")
        
    else:
        print("\n✅ All PX4 containers appear to be running!")
        print("You can now run the GPS home position fix scripts:")
        print("  python fix_gps_home_mavlink.py")
        print("  python diagnose_gps_home_issue.py")
    
    # Test AirSim connection
    print(f"\n📡 Testing AirSim connection...")
    if check_port_connectivity("127.0.0.1", 41451):
        print("  ✅ AirSim (port 41451) - REACHABLE")
    else:
        print("  ❌ AirSim (port 41451) - NOT REACHABLE")
        print("     Make sure AirSim container is running")

if __name__ == "__main__":
    main() 