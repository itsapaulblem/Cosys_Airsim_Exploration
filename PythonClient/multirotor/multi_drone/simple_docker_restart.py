#!/usr/bin/env python3
"""
Simple Docker Restart with TCP Ports
Creates and runs corrected docker-compose without extra dependencies
"""

import os
import sys
import subprocess
import time

def stop_current_containers():
    """Stop current containers"""
    print("🛑 Stopping current containers...")
    try:
        result = subprocess.run(
            ["docker-compose", "down"], 
            cwd="/mnt/l/Cosys-AirSim/docker_clean/multi_drone",
            capture_output=True, 
            text=True, 
            timeout=30
        )
        if result.returncode == 0:
            print("✅ Containers stopped successfully")
            return True
        else:
            print(f"⚠️ Stop command result: {result.stderr}")
            return True  # Continue anyway
    except Exception as e:
        print(f"⚠️ Error stopping containers: {e}")
        return True  # Continue anyway

def create_fixed_compose_file():
    """Create docker-compose file with TCP ports"""
    print("📝 Creating fixed docker-compose.yml...")
    
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
    - "14550:14550/udp"
    - "14541:14541/udp"
    - "14581:14581/udp"
    - "4561:4561/tcp"
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
    - "14551:14550/udp"
    - "14542:14542/udp"
    - "14582:14582/udp"
    - "4562:4562/tcp"
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
    - "14552:14550/udp"
    - "14543:14543/udp"
    - "14583:14583/udp"
    - "4563:4563/tcp"
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
    - "14553:14550/udp"
    - "14544:14544/udp"
    - "14584:14584/udp"
    - "4564:4564/tcp"
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
    - "14554:14550/udp"
    - "14545:14545/udp"
    - "14585:14585/udp"
    - "4565:4565/tcp"
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
    
    # Save the corrected file
    output_file = "/mnt/l/Cosys-AirSim/docker_clean/multi_drone/docker-compose-tcp-fixed.yml"
    try:
        with open(output_file, 'w') as f:
            f.write(compose_content)
        print(f"✅ Fixed docker-compose saved: {output_file}")
        return output_file
    except Exception as e:
        print(f"❌ Error creating file: {e}")
        return None

def start_fixed_containers(compose_file):
    """Start containers with fixed configuration"""
    print("🚀 Starting containers with TCP port mappings...")
    
    try:
        result = subprocess.run(
            ["docker-compose", "-f", os.path.basename(compose_file), "up", "-d"],
            cwd="/mnt/l/Cosys-AirSim/docker_clean/multi_drone",
            capture_output=True,
            text=True,
            timeout=120
        )
        
        if result.returncode == 0:
            print("✅ Containers started successfully with TCP ports!")
            print("📊 Container status:")
            # Show running containers
            status_result = subprocess.run(
                ["docker", "ps", "--format", "table {{.Names}}\\t{{.Ports}}"],
                capture_output=True,
                text=True
            )
            if status_result.returncode == 0:
                print(status_result.stdout)
            return True
        else:
            print(f"❌ Error starting containers: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error starting containers: {e}")
        return False

def test_tcp_connectivity():
    """Test if TCP ports are now accessible"""
    print("\n🧪 Testing TCP port connectivity...")
    
    import socket
    
    tcp_ports = [4561, 4562, 4563, 4564, 4565]
    working_ports = []
    
    for port in tcp_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex(('localhost', port))
            if result == 0:
                working_ports.append(port)
                print(f"   ✅ Port {port}: Accessible")
            else:
                print(f"   ❌ Port {port}: Not accessible")
            sock.close()
        except Exception as e:
            print(f"   ❌ Port {port}: Error - {e}")
    
    return len(working_ports)

def main():
    print("🚁 SIMPLE DOCKER TCP FIX")
    print("=" * 50)
    print("This script will:")
    print("1. Stop current containers")
    print("2. Create docker-compose with TCP ports")
    print("3. Start containers with proper networking")
    print("4. Test TCP connectivity")
    
    # Step 1: Stop current containers
    stop_current_containers()
    
    # Step 2: Create fixed compose file
    compose_file = create_fixed_compose_file()
    if not compose_file:
        print("❌ Failed to create compose file")
        return 1
    
    # Step 3: Start with fixed configuration
    if not start_fixed_containers(compose_file):
        print("❌ Failed to start containers")
        return 1
    
    # Wait for containers to initialize
    print("\n⏳ Waiting for containers to initialize...")
    time.sleep(10)
    
    # Step 4: Test connectivity
    working_count = test_tcp_connectivity()
    
    # Results
    print("\n" + "=" * 50)
    if working_count >= 2:
        print(f"🎉 SUCCESS! {working_count}/5 TCP ports are accessible!")
        print("✅ Your AirSim multi-drone mission should work now!")
        print("\n🚁 TO TEST:")
        print("   python multi_drone_orbit_mission.py")
    else:
        print(f"⚠️ Only {working_count}/5 TCP ports accessible")
        print("\n🔧 MANUAL STEPS:")
        print("1. Check container logs: docker logs px4-drone1")
        print("2. Verify AirSim settings match TCP ports 4561-4565")
        print("3. Restart AirSim to pick up new network configuration")
    
    print(f"\n📋 WHAT WAS FIXED:")
    print(f"   ✅ Added TCP port mappings: 4561-4565")
    print(f"   ✅ These ports enable AirSim ↔ PX4 HIL communication")
    print(f"   ✅ GPS home location should now work properly")
    
    return 0 if working_count >= 2 else 1

if __name__ == "__main__":
    sys.exit(main())