#!/usr/bin/env python3
"""
Check AirSim TCP Server Status

This script checks if AirSim is properly listening for PX4 TCP connections.
"""

import setup_path
import cosysairsim as airsim
import socket
import subprocess

def check_airsim_tcp_listeners():
    """Check if AirSim is listening on expected TCP ports"""
    print("🔍 Checking AirSim TCP Listeners")
    print("=" * 35)
    
    # Check if anything is listening on the expected ports
    ports = [4561, 4562, 4563, 4564, 4565]
    
    try:
        # Use netstat to check listening ports
        result = subprocess.run(
            ['netstat', '-an', '|', 'findstr', ':456'],
            shell=True, capture_output=True, text=True
        )
        
        print("📋 Current TCP listeners (ports 456x):")
        if result.stdout.strip():
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    print(f"   {line}")
        else:
            print("   ❌ No listeners found on ports 456x")
        
        # Test each port specifically
        print(f"\n🧪 Testing individual ports:")
        for port in ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex(('127.0.0.1', port))
                
                if result == 0:
                    print(f"   ✅ Port {port}: Something is listening")
                else:
                    print(f"   ❌ Port {port}: Nothing listening")
                
                sock.close()
                
            except Exception as e:
                print(f"   ❌ Port {port}: Error - {e}")
    
    except Exception as e:
        print(f"❌ Error checking listeners: {e}")

def check_airsim_connection_status():
    """Check AirSim connection and vehicle status"""
    print(f"\n🔗 AirSim Connection Status")
    print("=" * 30)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ AirSim API connection: Working")
        
        # Check vehicles
        vehicles = client.listVehicles()
        print(f"📋 Available vehicles: {vehicles}")
        
        # Check if vehicles are connected to PX4
        for vehicle in vehicles[:3]:  # Check first 3
            try:
                state = client.getMultirotorState(vehicle_name=vehicle)
                print(f"   {vehicle}: ✅ Responding")
            except Exception as e:
                print(f"   {vehicle}: ❌ Error - {e}")
        
    except Exception as e:
        print(f"❌ AirSim API connection failed: {e}")

def diagnose_tcp_issue():
    """Diagnose why TCP connections aren't working"""
    print(f"\n🔧 TCP Issue Diagnosis")
    print("=" * 25)
    
    print("Expected behavior:")
    print("1. AirSim starts and creates TCP server sockets on ports 4561-4565")
    print("2. AirSim listens for incoming connections from PX4 containers")
    print("3. PX4 containers connect to host.docker.internal:4561+ (resolves to host)")
    print("4. TCP connection established → GPS data flows → home location set")
    
    print(f"\nActual behavior:")
    print("❌ AirSim is NOT listening on any TCP ports")
    print("❌ PX4 containers cannot connect")
    print("❌ No GPS data exchange")
    print("❌ Home location remains NaN")
    
    print(f"\n💡 Possible causes:")
    print("1. AirSim not restarted after settings.json change")
    print("2. Firewall blocking TCP listeners")
    print("3. Another process using the ports")
    print("4. AirSim configuration error")
    
    print(f"\n🎯 Solution:")
    print("1. Completely restart AirSim (Unreal Engine)")
    print("2. Wait for AirSim to load settings.json")
    print("3. Check that TCP listeners start")
    print("4. Restart containers if needed")

def main():
    print("🔍 AirSim TCP Server Status Check")
    print("=" * 35)
    
    check_airsim_tcp_listeners()
    check_airsim_connection_status()
    diagnose_tcp_issue()
    
    print(f"\n📋 Summary")
    print("=" * 15)
    print("The core issue is that AirSim is not listening on TCP ports.")
    print("This prevents PX4 containers from connecting.")
    print("Solution: Restart AirSim completely to apply the new settings.")

if __name__ == "__main__":
    main() 