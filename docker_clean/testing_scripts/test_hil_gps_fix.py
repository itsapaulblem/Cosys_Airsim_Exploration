#!/usr/bin/env python3
"""
Test script to verify the HIL communication and GPS home location fix.
This script tests both issues:
1. HIL communication with proper Docker networking
2. GPS home location establishment and armDisarm() functionality
"""

import sys
import time
import socket
import subprocess
import os

# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'PythonClient')))

try:
    import cosysairsim as airsim
    AIRSIM_AVAILABLE = True
except ImportError:
    print("⚠️  AirSim Python client not found - testing network connectivity only")
    AIRSIM_AVAILABLE = False

def test_docker_container_status():
    """Check if PX4 containers are running"""
    print("🐳 Checking Docker container status...")
    
    try:
        result = subprocess.run(['docker', 'ps', '--format', 'table {{.Names}}\t{{.Status}}'], 
                              capture_output=True, text=True, check=True)
        
        lines = result.stdout.strip().split('\n')
        px4_containers = [line for line in lines if 'px4-drone' in line.lower()]
        
        if not px4_containers:
            print("❌ No PX4 containers found running")
            return False
            
        for container in px4_containers:
            print(f"  ✅ {container}")
        
        return True
        
    except Exception as e:
        print(f"❌ Docker check failed: {e}")
        return False

def test_hil_port_connectivity(port_start=4561, num_ports=5):
    """Test HIL port connectivity (should connect but then reset)"""
    print(f"\n🔌 Testing HIL port connectivity (ports {port_start}-{port_start+num_ports-1})...")
    
    results = {}
    
    for i in range(num_ports):
        port = port_start + i
        print(f"\n  Testing port {port}...")
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            
            start_time = time.time()
            sock.connect(('localhost', port))
            connect_time = time.time() - start_time
            
            print(f"    ✅ Connected in {connect_time:.3f}s")
            
            # Send test message
            sock.send(b"HIL_TEST\n")
            
            # Try to receive (should timeout or get reset)
            try:
                sock.settimeout(2)
                response = sock.recv(1024)
                print(f"    📨 Response: {response}")
                results[port] = "connected_with_response"
            except socket.timeout:
                print("    ⏳ No response (normal for PX4 waiting for AirSim)")
                results[port] = "connected_no_response"
            except ConnectionResetError:
                print("    🔄 Connection reset (normal - not AirSim protocol)")
                results[port] = "connected_reset"
                
            sock.close()
            
        except ConnectionRefusedError:
            print(f"    ❌ Connection refused on port {port}")
            results[port] = "refused"
        except Exception as e:
            print(f"    ❌ Error: {e}")
            results[port] = f"error: {e}"
    
    return results

def test_airsim_client_connection():
    """Test AirSim client connection and GPS/arming functionality"""
    if not AIRSIM_AVAILABLE:
        print("\n⚠️  Skipping AirSim client tests - cosysairsim not available")
        return False
        
    print("\n🚁 Testing AirSim client connection...")
    
    try:
        # Test basic connection
        client = airsim.MultirotorClient()
        print("  📡 Connecting to AirSim...")
        client.confirmConnection()
        print("  ✅ AirSim connection established")
        
        # Test vehicle discovery
        print("  🔍 Discovering vehicles...")
        
        # Try common vehicle names
        test_vehicles = ["Drone1", "PX4_Drone1", "UAV1", "Multirotor1"]
        found_vehicle = None
        
        for vehicle_name in test_vehicles:
            try:
                state = client.getMultirotorState(vehicle_name=vehicle_name)
                found_vehicle = vehicle_name
                print(f"  ✅ Found vehicle: {vehicle_name}")
                break
            except:
                continue
        
        if not found_vehicle:
            print("  ⚠️  No vehicles found - testing with default vehicle")
            found_vehicle = ""
        
        # Test GPS and arming
        return test_gps_and_arming(client, found_vehicle)
        
    except Exception as e:
        print(f"  ❌ AirSim connection failed: {e}")
        return False

def test_gps_and_arming(client, vehicle_name=""):
    """Test GPS home location and arming functionality"""
    print(f"\n🛰️  Testing GPS and arming for vehicle: '{vehicle_name}'")
    
    try:
        # Get initial state
        print("  📊 Getting vehicle state...")
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        
        # Check GPS
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        print(f"  🛰️  GPS Fix: {gps_data.fix_type}")
        print(f"  📍 GPS Location: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}")
        
        if gps_data.fix_type == 0:
            print("  ⚠️  No GPS fix - this may cause arming issues")
        else:
            print("  ✅ GPS fix established")
        
        # Test API control
        print("  🎮 Enabling API control...")
        client.enableApiControl(True, vehicle_name)
        
        if client.isApiControlEnabled(vehicle_name):
            print("  ✅ API control enabled")
        else:
            print("  ❌ API control failed")
            return False
        
        # Test arming (the critical test)
        print("  🔐 Testing arming...")
        try:
            success = client.armDisarm(True, vehicle_name)
            if success:
                print("  ✅ Vehicle armed successfully!")
                
                # Disarm for safety
                time.sleep(1)
                client.armDisarm(False, vehicle_name)
                print("  🔓 Vehicle disarmed")
                
                return True
            else:
                print("  ❌ Arming failed (returned False)")
                return False
                
        except Exception as arm_error:
            print(f"  ❌ Arming error: {arm_error}")
            if "GPS home location" in str(arm_error):
                print("  💡 This indicates HIL communication is not working properly")
            return False
        
    except Exception as e:
        print(f"  ❌ GPS/Arming test failed: {e}")
        return False
    
    finally:
        # Cleanup
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

def main():
    print("=" * 80)
    print("  HIL COMMUNICATION AND GPS HOME LOCATION FIX TEST")
    print("=" * 80)
    
    all_tests_passed = True
    
    # Test 1: Docker containers
    if not test_docker_container_status():
        print("\n❌ Docker container test failed")
        all_tests_passed = False
    
    # Test 2: HIL port connectivity  
    hil_results = test_hil_port_connectivity()
    
    connected_ports = [port for port, result in hil_results.items() 
                      if 'connected' in result]
    
    if connected_ports:
        print(f"\n✅ HIL ports accessible: {connected_ports}")
    else:
        print(f"\n❌ No HIL ports accessible")
        all_tests_passed = False
    
    # Test 3: AirSim client functionality
    if test_airsim_client_connection():
        print("\n✅ AirSim client tests passed")
    else:
        print("\n❌ AirSim client tests failed")
        all_tests_passed = False
    
    # Summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    
    if all_tests_passed:
        print("🎉 ALL TESTS PASSED!")
        print("✅ HIL communication is working properly")
        print("✅ GPS home location is being established") 
        print("✅ armDisarm() functionality is working")
        print("\n💡 The Docker networking fix has resolved both issues!")
        return 0
    else:
        print("❌ SOME TESTS FAILED")
        print("\nTroubleshooting:")
        print("1. Ensure PX4 containers are running")
        print("2. Ensure AirSim/Unreal is running")
        print("3. Check that vehicles are properly configured in settings.json")
        print("4. Verify no firewall is blocking connections")
        return 1

if __name__ == "__main__":
    sys.exit(main())