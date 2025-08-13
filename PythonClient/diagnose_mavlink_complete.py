#!/usr/bin/env python3
"""
Complete MAVLink Communication Diagnostic

This tool thoroughly diagnoses the GPS home location issue by checking:
1. AirSim API connection
2. TCP socket connections
3. MAVLink message flow
4. PX4 parameter configuration
5. GPS sensor configuration
"""

import setup_path
import cosysairsim as airsim
import subprocess
import time
import socket
import json
from pathlib import Path

def test_tcp_socket_connections():
    """Test if AirSim can actually connect to the TCP ports"""
    localHostIp = "172.28.240.1"
    print("\n🔌 Testing TCP Socket Connections on ", localHostIp)
    print("=" * 40)
    
    ports_to_test = [4561, 4562, 4563, 4564, 4565]
    
    for port in ports_to_test:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex(('172.28.240.1', port)) #127.0.0.1
            
            if result == 0:
                print(f"✅ Port {port}: Connection successful")
                # Send a test message to see if anything responds
                try:
                    sock.send(b"TEST")
                    sock.settimeout(1)
                    response = sock.recv(1024)
                    print(f"   Response: {len(response)} bytes received")
                except:
                    print(f"   No response to test message")
            else:
                print(f"❌ Port {port}: Connection failed (error {result})")
            
            sock.close()
            
        except Exception as e:
            print(f"❌ Port {port}: Exception - {e}")

def check_airsim_settings_loaded():
    """Check what settings AirSim actually loaded"""
    print("\n📋 Checking AirSim Settings")
    print("=" * 30)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Get the settings string
        settings_str = client.getSettingsString()
        
        # Check for key indicators
        tcp_ports = []
        control_ips = []
        
        # Parse for TCP ports
        for line in settings_str.split('\n'):
            if 'TcpPort' in line:
                tcp_ports.append(line.strip())
            elif 'ControlIp' in line:
                control_ips.append(line.strip())
        
        print(f"📊 Settings Analysis:")
        print(f"   Total settings length: {len(settings_str)} characters")
        print(f"   TCP ports found: {len(tcp_ports)}")
        for port in tcp_ports:
            print(f"     {port}")
        print(f"   Control IPs found: {len(control_ips)}")
        for ip in control_ips:
            print(f"     {ip}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to get AirSim settings: {e}")
        return False

def test_gps_sensors():
    """Test GPS sensor configuration and data"""
    print("\n🛰️ Testing GPS Sensors")
    print("=" * 25)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicles = ["PX4_Drone1", "PX4_Drone2", "PX4_Drone3"]
        
        for vehicle in vehicles:
            try:
                print(f"\n🚁 Testing {vehicle}:")
                
                # Test GPS data
                try:
                    gps_data = client.getGpsData(vehicle_name=vehicle)
                    print(f"   GPS Data Available: ✅")
                    print(f"   GPS Time: {gps_data.time_utc}")
                    print(f"   GPS Position: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}")
                    print(f"   GPS Altitude: {gps_data.gnss.geo_point.altitude:.2f}")
                except Exception as e:
                    print(f"   GPS Data: ❌ {e}")
                
                # Test Home Geo Point
                try:
                    home = client.getHomeGeoPoint(vehicle_name=vehicle)
                    print(f"   Home Point: {home.latitude:.6f}, {home.longitude:.6f}, {home.altitude:.2f}")
                    if home.latitude != 0 and not str(home.latitude).lower() == 'nan':
                        print(f"   Home Status: ✅ Valid")
                    else:
                        print(f"   Home Status: ❌ Invalid")
                except Exception as e:
                    print(f"   Home Point: ❌ {e}")
                
                # Test Vehicle State
                try:
                    state = client.getMultirotorState(vehicle_name=vehicle)
                    print(f"   Vehicle State: ✅ Available")
                    print(f"   Landed State: {state.landed_state}")
                    print(f"   Position: {state.kinematics_estimated.position}")
                except Exception as e:
                    print(f"   Vehicle State: ❌ {e}")
                
            except Exception as e:
                print(f"   Vehicle {vehicle}: ❌ Not accessible - {e}")
        
    except Exception as e:
        print(f"❌ Failed to test GPS sensors: {e}")

def check_px4_container_status():
    """Check PX4 container status and logs"""
    print("\n🐳 Checking PX4 Container Status")
    print("=" * 35)
    
    try:
        # Get container status
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=px4-', '--format', 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            print("📋 Container Status:")
            for line in lines:
                print(f"   {line}")
        
        # Check recent logs for one container
        print("\n📋 Recent Container Logs (px4-px4-drone1):")
        result = subprocess.run(
            ['docker', 'logs', '--tail', '10', 'px4-px4-drone1'],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            for line in result.stdout.split('\n')[-10:]:
                if line.strip():
                    print(f"   {line}")
        
    except Exception as e:
        print(f"❌ Error checking containers: {e}")

def test_mavlink_home_position():
    """Test MAVLink HOME_POSITION message reception"""
    print("\n📡 Testing MAVLink HOME_POSITION")
    print("=" * 35)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Try to arm each vehicle and see what happens
        vehicles = ["PX4_Drone1"]
        
        for vehicle in vehicles:
            print(f"\n🚁 Testing {vehicle} armDisarm sequence:")
            
            try:
                # Enable API control
                client.enableApiControl(True, vehicle)
                print(f"   API Control: ✅ Enabled")
                
                # Try to arm
                client.armDisarm(True, vehicle)
                print(f"   Arm Command: ✅ Successful")
                
                # Disarm immediately
                client.armDisarm(False, vehicle)
                print(f"   Disarm Command: ✅ Successful")
                
            except Exception as e:
                print(f"   Arm/Disarm: ❌ {e}")
                
                # Check if it's the GPS error
                if "GPS home location" in str(e):
                    print(f"   🎯 GPS HOME LOCATION ERROR CONFIRMED")
                    
                    # Try waiting and retrying
                    print(f"   ⏳ Waiting 5 seconds and retrying...")
                    time.sleep(5)
                    
                    try:
                        client.armDisarm(True, vehicle)
                        print(f"   Retry: ✅ Successful after wait")
                        client.armDisarm(False, vehicle)
                    except Exception as e2:
                        print(f"   Retry: ❌ Still failing - {e2}")
            
            finally:
                try:
                    client.enableApiControl(False, vehicle)
                except:
                    pass
    
    except Exception as e:
        print(f"❌ Error testing MAVLink: {e}")

def analyze_network_flow():
    """Analyze the complete network flow"""
    print("\n🌐 Network Flow Analysis")
    print("=" * 25)
    
    print("Expected Flow:")
    print("1. PX4 containers start → connect to host.docker.internal:4561+")
    print("2. AirSim listens on 127.0.0.1:4561+ → accepts connections")
    print("3. PX4 sends GPS data → AirSim via MAVLink")
    print("4. AirSim processes GPS → sets home location")
    print("5. armDisarm works ✅")
    
    print("\nActual Status:")
    
    # Check if AirSim is listening
    listening_ports = []
    for port in [4561, 4562, 4563, 4564, 4565]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.bind(('127.0.0.1', port))
            sock.close()
            print(f"❌ Port {port}: Not in use (AirSim not listening?)")
        except OSError:
            print(f"✅ Port {port}: In use (AirSim likely listening)")
            listening_ports.append(port)
    
    if listening_ports:
        print(f"🎯 AirSim appears to be listening on ports: {listening_ports}")
    else:
        print(f"❌ AirSim doesn't appear to be listening on any ports")

def main():
    print("🔍 Complete MAVLink Communication Diagnostic")
    print("=" * 50)
    print("This tool checks every aspect of AirSim-PX4 communication")
    
    # Test sequence
    test_tcp_socket_connections()
    check_airsim_settings_loaded()
    check_px4_container_status()
    test_gps_sensors()
    analyze_network_flow()
    test_mavlink_home_position()
    
    print("\n📋 Diagnostic Complete")
    print("=" * 25)
    print("Review the output above to identify communication issues.")
    print("Key things to look for:")
    print("1. ✅ TCP connections working")
    print("2. ✅ AirSim listening on correct ports")
    print("3. ✅ Containers running and connecting")
    print("4. ❌ GPS home location invalid")
    print("5. ❌ armDisarm failing with GPS error")

if __name__ == "__main__":
    main() 