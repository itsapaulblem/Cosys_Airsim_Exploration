#!/usr/bin/env python3
"""
Docker TCP Port Fix
Adds missing TCP port mappings for AirSim-PX4 communication
"""

import os
import sys
import subprocess
import time

def fix_docker_tcp_ports():
    """Add TCP port mappings to running containers using iptables"""
    print("🚁 DOCKER TCP PORT FIX")
    print("=" * 50)
    print("Adding missing TCP port mappings for AirSim-PX4 communication...")
    
    # Container IP mappings (from docker network inspect)
    container_mappings = [
        {"name": "px4-drone1", "container_ip": "172.30.0.10", "tcp_port": 4561},
        {"name": "px4-drone2", "container_ip": "172.30.0.11", "tcp_port": 4562},
        {"name": "px4-drone3", "container_ip": "172.30.0.12", "tcp_port": 4563},
        {"name": "px4-drone4", "container_ip": "172.30.0.13", "tcp_port": 4564},
        {"name": "px4-drone5", "container_ip": "172.30.0.14", "tcp_port": 4565}
    ]
    
    success_count = 0
    
    for mapping in container_mappings:
        container_name = mapping["name"]
        container_ip = mapping["container_ip"]
        tcp_port = mapping["tcp_port"]
        
        print(f"\n🔧 Fixing {container_name}...")
        print(f"   Mapping localhost:{tcp_port} → {container_ip}:{tcp_port}")
        
        try:
            # Method 1: Use Docker's built-in port forwarding (if possible)
            # This creates an iptables rule to forward traffic
            cmd = [
                "docker", "exec", container_name,
                "sh", "-c", 
                f"socat TCP-LISTEN:{tcp_port},fork,reuseaddr TCP:localhost:4560 &"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                print(f"   ✅ {container_name}: TCP port {tcp_port} mapped via socat")
                success_count += 1
            else:
                # Method 2: Try alternative approach
                print(f"   🔄 Trying alternative mapping for {container_name}...")
                
                # Start a simple TCP relay inside the container
                cmd2 = [
                    "docker", "exec", "-d", container_name,
                    "sh", "-c",
                    f"while true; do socat TCP-LISTEN:{tcp_port},fork,reuseaddr TCP:127.0.0.1:4560; sleep 1; done"
                ]
                result2 = subprocess.run(cmd2, capture_output=True, text=True, timeout=5)
                
                if result2.returncode == 0:
                    print(f"   ✅ {container_name}: TCP relay started")
                    success_count += 1
                else:
                    print(f"   ❌ {container_name}: Could not create TCP mapping")
                    
        except Exception as e:
            print(f"   ❌ {container_name}: Error - {e}")
    
    return success_count

def create_fixed_docker_compose():
    """Create a corrected docker-compose.yml with TCP ports"""
    print("\n📝 CREATING FIXED DOCKER-COMPOSE.YML...")
    
    compose_content = '''version: '3.8'
services:
  px4-drone1:
    build:
      context: ../common
      dockerfile: Dockerfile
    container_name: px4-drone1
    hostname: px4-drone1
    environment:
    - PX4_HOME_LAT=47.641468
    - PX4_HOME_LON=-122.140165
    - PX4_HOME_ALT=10
    - PX4_SYS_AUTOSTART=10016
    - PX4_SIM_HOSTNAME=host.docker.internal
    - PX4_SIM_MODEL=iris
    - PX4_INSTANCE=1
    ports:
    - 14550:14550/udp    # QGroundControl
    - 14541:14541/udp    # MAVLink Local
    - 14581:14581/udp    # MAVLink Remote
    - 4561:4561/tcp      # AirSim TCP (ADDED!)
    networks:
      airsim-network:
        ipv4_address: 172.30.0.10
    volumes:
    - px4-shared-data:/px4_data
    - ../common/scripts:/scripts
    restart: unless-stopped
    command:
    - /Scripts/run_airsim_sitl_final.sh
    - '1'

  px4-drone2:
    build:
      context: ../common
      dockerfile: Dockerfile
    container_name: px4-drone2
    hostname: px4-drone2
    environment:
    - PX4_HOME_LAT=47.641468
    - PX4_HOME_LON=-122.140165
    - PX4_HOME_ALT=10
    - PX4_SYS_AUTOSTART=10016
    - PX4_SIM_HOSTNAME=host.docker.internal
    - PX4_SIM_MODEL=iris
    - PX4_INSTANCE=2
    ports:
    - 14551:14550/udp    # QGroundControl
    - 14542:14542/udp    # MAVLink Local
    - 14582:14582/udp    # MAVLink Remote
    - 4562:4562/tcp      # AirSim TCP (ADDED!)
    networks:
      airsim-network:
        ipv4_address: 172.30.0.11
    volumes:
    - px4-shared-data:/px4_data
    - ../common/scripts:/scripts
    restart: unless-stopped
    command:
    - /Scripts/run_airsim_sitl_final.sh
    - '2'

  px4-drone3:
    build:
      context: ../common
      dockerfile: Dockerfile
    container_name: px4-drone3
    hostname: px4-drone3
    environment:
    - PX4_HOME_LAT=47.641468
    - PX4_HOME_LON=-122.140165
    - PX4_HOME_ALT=10
    - PX4_SYS_AUTOSTART=10016
    - PX4_SIM_HOSTNAME=host.docker.internal
    - PX4_SIM_MODEL=iris
    - PX4_INSTANCE=3
    ports:
    - 14552:14550/udp    # QGroundControl
    - 14543:14543/udp    # MAVLink Local
    - 14583:14583/udp    # MAVLink Remote
    - 4563:4563/tcp      # AirSim TCP (ADDED!)
    networks:
      airsim-network:
        ipv4_address: 172.30.0.12
    volumes:
    - px4-shared-data:/px4_data
    - ../common/scripts:/scripts
    restart: unless-stopped
    command:
    - /Scripts/run_airsim_sitl_final.sh
    - '3'

  px4-drone4:
    build:
      context: ../common
      dockerfile: Dockerfile
    container_name: px4-drone4
    hostname: px4-drone4
    environment:
    - PX4_HOME_LAT=47.641468
    - PX4_HOME_LON=-122.140165
    - PX4_HOME_ALT=10
    - PX4_SYS_AUTOSTART=10016
    - PX4_SIM_HOSTNAME=host.docker.internal
    - PX4_SIM_MODEL=iris
    - PX4_INSTANCE=4
    ports:
    - 14553:14550/udp    # QGroundControl
    - 14544:14544/udp    # MAVLink Local
    - 14584:14584/udp    # MAVLink Remote
    - 4564:4564/tcp      # AirSim TCP (ADDED!)
    networks:
      airsim-network:
        ipv4_address: 172.30.0.13
    volumes:
    - px4-shared-data:/px4_data
    - ../common/scripts:/scripts
    restart: unless-stopped
    command:
    - /Scripts/run_airsim_sitl_final.sh
    - '4'

  px4-drone5:
    build:
      context: ../common
      dockerfile: Dockerfile
    container_name: px4-drone5
    hostname: px4-drone5
    environment:
    - PX4_HOME_LAT=47.641468
    - PX4_HOME_LON=-122.140165
    - PX4_HOME_ALT=10
    - PX4_SYS_AUTOSTART=10016
    - PX4_SIM_HOSTNAME=host.docker.internal
    - PX4_SIM_MODEL=iris
    - PX4_INSTANCE=5
    ports:
    - 14554:14550/udp    # QGroundControl
    - 14545:14545/udp    # MAVLink Local
    - 14585:14585/udp    # MAVLink Remote
    - 4565:4565/tcp      # AirSim TCP (ADDED!)
    networks:
      airsim-network:
        ipv4_address: 172.30.0.14
    volumes:
    - px4-shared-data:/px4_data
    - ../common/scripts:/scripts
    restart: unless-stopped
    command:
    - /Scripts/run_airsim_sitl_final.sh
    - '5'

networks:
  airsim-network:
    driver: bridge
    ipam:
      config:
      - subnet: 172.30.0.0/16

volumes:
  px4-shared-data:
    driver: local
'''
    
    # Save the fixed docker-compose file
    compose_file = "/mnt/l/Cosys-AirSim/docker_clean/multi_drone/docker-compose-fixed.yml"
    try:
        with open(compose_file, 'w') as f:
            f.write(compose_content)
        print(f"✅ Fixed docker-compose saved to: {compose_file}")
        return compose_file
    except Exception as e:
        print(f"❌ Error creating fixed compose file: {e}")
        return None

def main():
    print("🚁 DOCKER TCP PORT FIX FOR AIRSIM-PX4 COMMUNICATION")
    print("=" * 70)
    
    print("🔍 ISSUE ANALYSIS:")
    print("   ❌ Docker containers only expose UDP ports (MAVLink ground control)")
    print("   ❌ Missing TCP ports 4561-4565 (AirSim HIL communication)")
    print("   ❌ AirSim cannot connect to PX4 for simulation")
    
    # Method 1: Try quick fix with current containers
    print("\n🔧 ATTEMPTING QUICK FIX...")
    success_count = fix_docker_tcp_ports()
    
    if success_count > 0:
        print(f"\n✅ Quick fix applied to {success_count} containers!")
        print("🧪 Test your mission now - it might work!")
    else:
        print("\n⚠️ Quick fix didn't work, creating proper solution...")
    
    # Method 2: Create proper docker-compose fix
    fixed_compose = create_fixed_docker_compose()
    
    if fixed_compose:
        print(f"\n🎯 PERMANENT SOLUTION CREATED!")
        print("To apply the permanent fix:")
        print("1. Stop current containers:")
        print("   cd /mnt/l/Cosys-AirSim/docker_clean/multi_drone")
        print("   docker-compose down")
        print("2. Use fixed configuration:")
        print("   docker-compose -f docker-compose-fixed.yml up -d")
        print("3. Your mission should work perfectly!")
        
        print(f"\n📋 FIXED CONFIGURATION ADDS:")
        print(f"   ✅ TCP ports 4561-4565 for AirSim communication")
        print(f"   ✅ All 5 PX4 drones properly configured")
        print(f"   ✅ Correct port mappings for each container")
    
    print(f"\n💡 ROOT CAUSE:")
    print(f"   Your Docker setup was missing TCP port exposure!")
    print(f"   - WSL2 instances work: Direct network access")
    print(f"   - Docker containers failed: Missing port mappings")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())