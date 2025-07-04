#!/usr/bin/env python3
"""
GPS Troubleshooting Script for Docker PX4-AirSim Setup
Diagnoses GPS data transmission issues between AirSim and PX4 in containers
"""

import sys
import os
import time
import math
import socket

# Add AirSim Python client to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'PythonClient'))

try:
    import cosysairsim as airsim
except ImportError:
    print("❌ AirSim Python client not found!")
    print("   Make sure you're running from the correct directory")
    sys.exit(1)

def test_network_connectivity():
    """Test network connectivity to AirSim"""
    print("🌐 Testing Network Connectivity")
    print("-" * 30)
    
    # Test AirSim API connection
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ AirSim API connected successfully")
        return client
    except Exception as e:
        print(f"❌ AirSim API connection failed: {e}")
        
        # Test if AirSim port is reachable
        try:
            sock = socket.create_connection(("127.0.0.1", 41451), timeout=5)
            sock.close()
            print("✅ AirSim port 41451 is reachable")
            print("💡 Connection issue may be temporary, retrying...")
            time.sleep(2)
            
            client = airsim.MultirotorClient()
            client.confirmConnection()
            print("✅ AirSim API connected on retry")
            return client
        except Exception as port_e:
            print(f"❌ AirSim port 41451 not reachable: {port_e}")
            print("💡 Make sure AirSim is running with correct settings")
            return None

def test_px4_connectivity():
    """Test connectivity to PX4 containers"""
    print("\n🚁 Testing PX4 Container Connectivity")
    print("-" * 35)
    
    # Test common PX4 ports
    px4_ports = {
        14550: "QGroundControl",
        14541: "MAVLink Local",
        14581: "MAVLink Remote",
        4561: "AirSim TCP"
    }
    
    for port, description in px4_ports.items():
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=2)
            sock.close()
            print(f"✅ {description} (port {port}) - Connected")
        except Exception:
            print(f"❌ {description} (port {port}) - Not reachable")

def diagnose_gps_transmission(client, test_duration=30):
    """Diagnose GPS data transmission issues"""
    print(f"\n🛰️ GPS Data Transmission Diagnosis ({test_duration}s)")
    print("-" * 45)
    
    gps_issues = []
    
    try:
        print("📊 Real-time GPS monitoring:")
        print("Time | Fix | GPS Coordinates        | Home Coordinates       | Status")
        print("-" * 75)
        
        for i in range(test_duration):
            try:
                # Get GPS and home data
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                # Format coordinates
                gps_coords = f"({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f})"
                home_coords = f"({home_location.latitude}, {home_location.longitude})"
                
                # Check GPS fix type
                fix_type = gps_data.gnss.fix_type
                if fix_type < 3:
                    gps_issues.append(f"Poor GPS fix (type {fix_type}) at {i+1}s")
                
                # Check home location validity
                home_valid = not (math.isnan(home_location.latitude) or math.isnan(home_location.longitude))
                
                if home_valid:
                    status = "✅ HOME_SET"
                    print(f"{i+1:3d}s | {fix_type}   | {gps_coords:22s} | {home_coords:22s} | {status}")
                    
                    # Test arming when home is set
                    if i >= 5:  # Wait a bit before testing arm
                        print(f"\n🔧 Testing ARM command (GPS home established)...")
                        try:
                            client.enableApiControl(True)
                            client.armDisarm(True)
                            print(f"✅ ARM successful! GPS data flow is working correctly.")
                            client.armDisarm(False)
                            client.enableApiControl(False)
                            return True, gps_issues
                        except Exception as arm_e:
                            if "GPS home location" in str(arm_e):
                                print(f"❌ ARM failed - GPS home issue: {arm_e}")
                                gps_issues.append(f"ARM failed due to GPS: {arm_e}")
                            else:
                                print(f"❌ ARM failed - other issue: {arm_e}")
                            client.enableApiControl(False)
                else:
                    status = "❌ NO_HOME"
                    if i % 5 == 0:  # Show every 5 seconds when no home
                        print(f"{i+1:3d}s | {fix_type}   | {gps_coords:22s} | {home_coords:22s} | {status}")
                
            except Exception as e:
                error_msg = f"Data error at {i+1}s: {e}"
                print(f"{i+1:3d}s | ERR | {error_msg:45s}")
                gps_issues.append(error_msg)
            
            time.sleep(1)
        
        print(f"\n❌ GPS home location not established after {test_duration} seconds")
        return False, gps_issues
        
    except Exception as e:
        print(f"❌ GPS diagnosis failed: {e}")
        return False, [f"Diagnosis failed: {e}"]

def provide_solutions(gps_issues):
    """Provide solutions based on identified issues"""
    print(f"\n💡 Troubleshooting Solutions")
    print("-" * 25)
    
    if not gps_issues:
        print("✅ No GPS issues detected!")
        return
    
    print("Detected issues:")
    for issue in gps_issues:
        print(f"  • {issue}")
    
    print("\n🔧 Recommended Solutions:")
    
    # Check for common issues
    has_fix_issues = any("GPS fix" in issue for issue in gps_issues)
    has_home_issues = any("home" in issue.lower() for issue in gps_issues)
    has_arm_issues = any("ARM failed" in issue for issue in gps_issues)
    
    if has_fix_issues:
        print("📡 GPS Fix Issues:")
        print("  1. Check AirSim environment has GPS enabled")
        print("  2. Verify drone is in outdoor environment (GPS requires 'sky view')")
        print("  3. Check AirSim settings.json for GPS sensor configuration")
    
    if has_home_issues:
        print("🏠 GPS Home Location Issues:")
        print("  1. Ensure PX4 and AirSim are on same Docker network")
        print("  2. Check TCP port 4561 connectivity between containers")
        print("  3. Verify PX4_SIM_HOSTNAME=host.docker.internal in PX4 container")
        print("  4. Try restarting both containers with clean state")
    
    if has_arm_issues:
        print("🔧 Arming Issues:")
        print("  1. Wait longer for GPS home establishment (can take 30-60s)")
        print("  2. Check PX4 parameters: EKF2_AID_MASK, EKF2_HGT_MODE")
        print("  3. Use PX4 console: 'commander status' to check GPS state")
    
    print("\n🐳 Docker-Specific Solutions:")
    print("  1. Use improved Docker Compose with unified network")
    print("  2. Ensure all containers use same network (172.30.0.0/16)")
    print("  3. Check container-to-container connectivity")
    print("  4. Use 'host.docker.internal' for AirSim-PX4 communication")

def main():
    print("🔍 Docker PX4-AirSim GPS Troubleshooting Tool")
    print("=" * 50)
    
    # Test network connectivity
    client = test_network_connectivity()
    if not client:
        print("\n❌ Cannot proceed without AirSim connection")
        print("💡 Start AirSim first, then run this script")
        return False
    
    # Test PX4 connectivity
    test_px4_connectivity()
    
    # Diagnose GPS transmission
    success, issues = diagnose_gps_transmission(client, 30)
    
    # Provide solutions
    provide_solutions(issues)
    
    print(f"\n{'='*50}")
    if success:
        print("🎉 GPS system is working correctly!")
    else:
        print("❌ GPS issues detected - follow solutions above")
    
    return success

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 GPS diagnosis interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1) 