#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math
import subprocess
import socket
import json
import threading
import struct
from datetime import datetime

def get_docker_network_info():
    """Get detailed Docker network information"""
    print("🐳 Docker Network Analysis")
    print("-" * 30)
    
    try:
        # Try multiple methods to get container IP
        container_ip = None
        
        # Method 1: Direct IPAddress
        result = subprocess.run(['docker', 'inspect', 'px4-single', '--format={{.NetworkSettings.IPAddress}}'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0 and result.stdout.strip():
            container_ip = result.stdout.strip()
            print(f"📍 Container IP (default): {container_ip}")
        
        # Method 2: Check all networks
        if not container_ip:
            result = subprocess.run(['docker', 'inspect', 'px4-single', '--format={{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0 and result.stdout.strip():
                container_ip = result.stdout.strip()
                print(f"📍 Container IP (networks): {container_ip}")
        
        # Method 3: Get specific network details
        result = subprocess.run(['docker', 'inspect', 'px4-single'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            try:
                import json
                inspect_data = json.loads(result.stdout)
                networks = inspect_data[0]['NetworkSettings']['Networks']
                print(f"🌐 Network Details:")
                for network_name, network_info in networks.items():
                    net_ip = network_info.get('IPAddress', 'N/A')
                    print(f"   {network_name}: {net_ip}")
                    if net_ip and not container_ip:
                        container_ip = net_ip
            except Exception as e:
                print(f"   Failed to parse network details: {e}")
        
        # Get port mappings
        result = subprocess.run(['docker', 'port', 'px4-single'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("🔌 Port Mappings:")
            for line in result.stdout.strip().split('\n'):
                if line:
                    print(f"   {line}")
        
        if container_ip:
            print(f"✅ Final Container IP: {container_ip}")
        else:
            print("❌ Could not determine container IP")
        
        return container_ip
        
    except Exception as e:
        print(f"❌ Docker network check failed: {e}")
        return None

def test_advanced_connectivity(container_ip=None):
    """Advanced connectivity testing"""
    print("\n🌐 Advanced Connectivity Testing")
    print("-" * 35)
    
    test_configs = [
        ("localhost", "127.0.0.1"),
        ("docker-host", "host.docker.internal") if container_ip else None,
        ("container-ip", container_ip) if container_ip else None,
        ("docker-gateway", "172.25.0.1"),
    ]
    
    # Filter out None entries
    test_configs = [config for config in test_configs if config is not None]
    
    ports_to_test = [4561, 14541, 14581, 14550]
    
    connectivity_results = {}
    
    for config_name, ip in test_configs:
        print(f"\n🔍 Testing {config_name} ({ip}):")
        connectivity_results[config_name] = {}
        
        for port in ports_to_test:
            try:
                # TCP test
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2)
                tcp_result = sock.connect_ex((ip, port))
                sock.close()
                
                # UDP test (just try to create socket)
                try:
                    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    udp_sock.settimeout(1)
                    udp_sock.sendto(b'test', (ip, port))
                    udp_sock.close()
                    udp_status = "✅"
                except:
                    udp_status = "❓"
                
                tcp_status = "✅" if tcp_result == 0 else "❌"
                print(f"   Port {port}: TCP {tcp_status} UDP {udp_status}")
                connectivity_results[config_name][port] = (tcp_result == 0, udp_status == "✅")
                
            except Exception as e:
                print(f"   Port {port}: ❌ Error - {e}")
                connectivity_results[config_name][port] = (False, False)
    
    return connectivity_results

def monitor_mavlink_traffic(duration=10):
    """Monitor MAVLink traffic on UDP port 14581"""
    print(f"\n📡 Monitoring MAVLink Traffic (Port 14581) for {duration}s")
    print("-" * 50)
    
    messages_received = []
    
    def udp_listener():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('', 14581))
            sock.settimeout(1)
            
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    data, addr = sock.recvfrom(1024)
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    messages_received.append((timestamp, addr, len(data)))
                    print(f"📨 {timestamp} - From {addr}: {len(data)} bytes")
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ UDP error: {e}")
                    break
            
            sock.close()
            
        except Exception as e:
            print(f"❌ Could not bind to port 14581: {e}")
    
    # Start listener in background
    listener_thread = threading.Thread(target=udp_listener)
    listener_thread.daemon = True
    listener_thread.start()
    
    # Wait for monitoring to complete
    listener_thread.join(duration + 1)
    
    if messages_received:
        print(f"✅ Received {len(messages_received)} MAVLink messages")
        return True
    else:
        print("❌ No MAVLink messages received")
        return False

def test_different_configurations(container_ip):
    """Test different AirSim configuration approaches"""
    print("\n🧪 Testing Different Configurations")
    print("-" * 40)
    
    test_configs = [
        {
            "name": "Localhost with port forwarding",
            "ControlIp": "127.0.0.1",
            "LocalHostIp": "127.0.0.1"
        },
        {
            "name": "Direct container IP",
            "ControlIp": container_ip,
            "LocalHostIp": "127.0.0.1"
        } if container_ip else None,
        {
            "name": "Docker host gateway",
            "ControlIp": "172.25.0.1",
            "LocalHostIp": "127.0.0.1"
        },
        {
            "name": "Docker internal hostname",
            "ControlIp": "host.docker.internal",
            "LocalHostIp": "127.0.0.1"
        }
    ]
    
    # Filter out None entries
    test_configs = [config for config in test_configs if config is not None]
    
    results = {}
    
    for config in test_configs:
        print(f"\n🔧 Testing: {config['name']}")
        print(f"   ControlIp: {config['ControlIp']}")
        
        # Create test settings file
        settings = create_test_settings(config['ControlIp'], config['LocalHostIp'])
        
        try:
            # Write temporary settings
            with open("temp_test_settings.json", "w") as f:
                json.dump(settings, f, indent=2)
            
            # Test connectivity to the ControlIp
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(2)
                sock.sendto(b'test', (config['ControlIp'], 14581))
                sock.close()
                connectivity = True
                print("   ✅ UDP connectivity: OK")
            except Exception as e:
                connectivity = False
                print(f"   ❌ UDP connectivity: {e}")
            
            results[config['name']] = {
                'connectivity': connectivity,
                'config': config
            }
            
        except Exception as e:
            print(f"   ❌ Configuration test failed: {e}")
            results[config['name']] = {'connectivity': False, 'error': str(e)}
    
    return results

def create_test_settings(control_ip, local_host_ip):
    """Create test settings with specified IPs"""
    return {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
        "SettingsVersion": 2.0,
        "SimMode": "Multirotor",
        "ClockType": "SteppableClock",
        "OriginGeopoint": {
            "Latitude": 47.641468,
            "Longitude": -122.140165,
            "Altitude": 122.0
        },
        "Vehicles": {
            "PX4_Drone1": {
                "VehicleType": "PX4Multirotor",
                "UseSerial": False,
                "LockStep": True,
                "UseTcp": True,
                "TcpPort": 4561,
                "ControlIp": control_ip,
                "ControlPortLocal": 14541,
                "ControlPortRemote": 14581,
                "LocalHostIp": local_host_ip,
                "UdpIp": local_host_ip,
                "UdpPort": 14560,
                "UseUdp": True,
                "Sensors": {
                    "Gps": {
                        "SensorType": 3,
                        "Enabled": True,
                        "EphTimeConstant": 0.9,
                        "EpvTimeConstant": 0.9,
                        "EphInitial": 5.0,
                        "EpvInitial": 5.0,
                        "EphFinal": 0.1,
                        "EpvFinal": 0.1,
                        "EphMin3d": 1.0,
                        "EphMin2d": 2.0,
                        "UpdateLatency": 0.1,
                        "UpdateFrequency": 50,
                        "StartupDelay": 1
                    }
                },
                "Parameters": {
                    "EKF2_GPS_CHECK": 5,
                    "EKF2_REQ_EPH": 5.0,
                    "EKF2_REQ_EPV": 8.0
                }
            }
        }
    }

def check_px4_mavlink_config():
    """Check PX4's MAVLink configuration inside container"""
    print("\n🚁 PX4 MAVLink Configuration Check")
    print("-" * 40)
    
    try:
        # Check PX4 processes
        result = subprocess.run(['docker', 'exec', 'px4-single', 'ps', 'aux'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            px4_processes = [line for line in result.stdout.split('\n') if 'px4' in line.lower()]
            print("🔍 PX4 Processes:")
            for proc in px4_processes:
                print(f"   {proc}")
        
        # Check network interfaces in container
        result = subprocess.run(['docker', 'exec', 'px4-single', 'ip', 'addr'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("\n🌐 Container Network Interfaces:")
            for line in result.stdout.split('\n'):
                if 'inet ' in line:
                    print(f"   {line.strip()}")
        
        # Check listening ports in container
        result = subprocess.run(['docker', 'exec', 'px4-single', 'netstat', '-ln'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("\n🔌 Container Listening Ports:")
            for line in result.stdout.split('\n'):
                if any(port in line for port in ['4561', '14541', '14581', '14550']):
                    print(f"   {line.strip()}")
        
        # Get recent PX4 logs with MAVLink info
        result = subprocess.run(['docker', 'logs', 'px4-single', '--tail', '50'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("\n📋 Recent PX4 Logs (MAVLink related):")
            for line in result.stdout.split('\n'):
                if any(keyword in line.lower() for keyword in ['mavlink', 'simulator', 'gps', 'home']):
                    print(f"   {line}")
        
    except Exception as e:
        print(f"❌ PX4 configuration check failed: {e}")

def test_port_connectivity():
    """Test if the forwarded ports are actually reachable"""
    print("🌐 Testing Port Connectivity")
    print("-" * 30)
    
    ports_to_test = [4561, 14541, 14581, 14550]
    
    for port in ports_to_test:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex(('127.0.0.1', port))
            sock.close()
            
            if result == 0:
                print(f"✅ Port {port}: Reachable")
            else:
                print(f"❌ Port {port}: Not reachable")
        except Exception as e:
            print(f"❌ Port {port}: Error - {e}")
    
    print()

def check_docker_containers():
    """Check if Docker containers are running properly"""
    print("🐳 Docker Container Status")
    print("-" * 30)
    
    try:
        result = subprocess.run(['docker', 'ps', '--filter', 'name=px4'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            if len(lines) > 1:  # Header + at least one container
                print("✅ Docker containers running:")
                for line in lines[1:]:  # Skip header
                    print(f"   {line}")
            else:
                print("❌ No PX4 containers running")
                print("💡 Start with: docker-compose --profile single up")
        else:
            print(f"❌ Docker command failed: {result.stderr}")
    except Exception as e:
        print(f"❌ Docker check failed: {e}")
    
    print()

def debug_docker_gps():
    """Comprehensive Docker GPS debugging"""
    print("🔍 Docker GPS Home Location Debug")
    print("=" * 45)
    
    # Phase 1: Infrastructure checks
    check_docker_containers()
    container_ip = get_docker_network_info()
    test_port_connectivity()
    connectivity_results = test_advanced_connectivity(container_ip)
    
    # Phase 2: PX4 configuration check
    check_px4_mavlink_config()
    
    # Phase 3: MAVLink monitoring
    print("\n📡 Starting MAVLink monitoring...")
    print("   (This will check if AirSim is sending GPS data)")
    mavlink_detected = monitor_mavlink_traffic(10)
    
    # Phase 4: Configuration testing
    if container_ip:
        config_results = test_different_configurations(container_ip)
    
    # Phase 5: AirSim connection test
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("\n✅ Connected to AirSim")
        
        # Check active settings
        settings_string = client.getSettingsString()
        print(f"🔧 Current AirSim settings contain: {len(settings_string)} characters")
        
        # GPS monitoring
        print("\n🔍 GPS Status Monitoring")
        print("-" * 25)
        
        for i in range(15):  # 15 second test
            try:
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                if i % 3 == 0:  # Every 3 seconds
                    print(f"⏱️  {i+1:2d}s:")
                    print(f"   GPS Fix: {gps_data.gnss.fix_type}")
                    print(f"   GPS Pos: ({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f})")
                    print(f"   Home:    ({home_location.latitude}, {home_location.longitude})")
                    
                    if not math.isnan(home_location.latitude):
                        print(f"\n🎉 SUCCESS! GPS home location set after {i+1} seconds!")
                        return True
                    print()
                
            except Exception as e:
                print(f"   Error at {i+1}s: {e}")
            
            time.sleep(1)
        
        print("❌ GPS home location not set after 15 seconds")
        return False
        
    except Exception as e:
        print(f"❌ AirSim connection failed: {e}")
        return False

def show_comprehensive_analysis():
    """Show detailed analysis and recommendations"""
    print("\n📊 Comprehensive Analysis & Recommendations")
    print("=" * 50)
    
    print("🔍 Root Cause Analysis:")
    print("1. Docker port forwarding may not be bidirectional for UDP")
    print("2. PX4 MAVLink configuration might not match AirSim expectations")
    print("3. Firewall or network isolation issues")
    print("4. AirSim GPS data transmission timing issues")
    print()
    
    print("🔧 Troubleshooting Steps:")
    print("1. Verify Docker UDP port forwarding:")
    print("   docker exec px4-single netstat -ln | grep 14581")
    print()
    print("2. Test direct container communication:")
    print("   Use container IP instead of localhost")
    print()
    print("3. Check Windows firewall:")
    print("   Allow UDP traffic on ports 14541, 14581")
    print()
    print("4. Monitor Docker logs for MAVLink messages:")
    print("   docker logs px4-single --follow | grep -i mavlink")
    print()
    
    print("🚀 Alternative Solutions:")
    print("1. Use host networking: --network host (Linux only)")
    print("2. Use WSL2 with direct IP communication")
    print("3. Use PX4 parameters to adjust GPS requirements")
    print("4. Use bridge network with custom configuration")

if __name__ == "__main__":
    print("🔍 Comprehensive Docker GPS Troubleshooter")
    print("=" * 50)
    
    success = debug_docker_gps()
    
    if not success:
        show_comprehensive_analysis()
        
        print("\n🎯 Next Steps:")
        print("1. Review the network connectivity results above")
        print("2. Try the configuration with best connectivity")
        print("3. Check Docker logs for MAVLink startup messages")
        print("4. Consider switching to WSL2 if Docker issues persist") 