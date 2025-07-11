#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math
import subprocess

def debug_gps_transmission():
    """Comprehensive debugging for GPS home location transmission"""
    print("🔍 GPS Home Location Transmission Debug")
    print("=" * 45)
    
    # Check current settings
    try:
        with open("c:\\Users\\Admin\\Documents\\AirSim\\settings.json", 'r') as f:
            settings_content = f.read()
            print("📁 Current AirSim settings.json:")
            if "172.28.240.1" in settings_content:
                print("✅ Settings contain correct Windows host IP (172.28.240.1)")
            else:
                print("❌ Settings do not contain Windows host IP")
                print("💡 Check if settings.json was properly updated")
    except Exception as e:
        print(f"❌ Could not read settings: {e}")
    
    print()
    
    # Connect to AirSim
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        # Get detailed sensor information
        print("\n🔍 Detailed Sensor Analysis:")
        print("-" * 30)
        
        # GPS Data Analysis
        gps_data = client.getGpsData()
        print(f"📡 GPS Sensor:")
        print(f"   Fix Type: {gps_data.gnss.fix_type}")
        print(f"   Coordinates: ({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f})")
        print(f"   Altitude: {gps_data.gnss.geo_point.altitude:.1f}m")
        print(f"   Time: {gps_data.time_stamp}")
        
        # Home Location Analysis
        home_location = client.getHomeGeoPoint()
        print(f"\n🏠 Home Location:")
        print(f"   Lat: {home_location.latitude}")
        print(f"   Lon: {home_location.longitude}")
        print(f"   Alt: {home_location.altitude}")
        
        # Vehicle State Analysis
        state = client.getMultirotorState()
        print(f"\n🚁 Vehicle State:")
        print(f"   Armed: {state.landed_state}")
        print(f"   API Control: {client.isApiControlEnabled()}")
        
        # Settings Analysis
        settings_string = client.getSettingsString()
        print(f"\n⚙️ Active Settings Analysis:")
        if "PX4Multirotor" in settings_string:
            print("✅ PX4Multirotor vehicle type detected")
        if "172.28.240.1" in settings_string:
            print("✅ Correct Windows host IP in active settings")
        else:
            print("❌ Windows host IP not found in active settings")
            print("💡 AirSim may not have loaded the updated settings")
        
        # Network connectivity test
        print(f"\n🌐 Network Connectivity Test:")
        try:
            # Test if we can reach the PX4 MAVLink ports
            result = subprocess.run(['wsl', 'bash', '-c', 'netstat -tuln | grep -E "14541|14581"'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and result.stdout.strip():
                print("✅ PX4 MAVLink ports are listening in WSL2")
                print(f"   Ports: {result.stdout.strip()}")
            else:
                print("❌ PX4 MAVLink ports not found")
        except Exception as e:
            print(f"❌ Network test failed: {e}")
        
        # Extended GPS monitoring
        print(f"\n⏳ Extended GPS Home Location Monitor (30s):")
        print("Time | GPS Fix | GPS Coordinates         | Home Coordinates        | Status")
        print("-" * 80)
        
        for i in range(30):
            try:
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                gps_coords = f"({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f})"
                home_coords = f"({home_location.latitude}, {home_location.longitude})"
                
                if not math.isnan(home_location.latitude):
                    status = "✅ SUCCESS!"
                    print(f"{i+1:3d}s | Fix {gps_data.gnss.fix_type}   | {gps_coords:22s} | {home_coords:22s} | {status}")
                    
                    # Test arming immediately when home is set
                    print("\n🔧 Testing ARM with valid home location...")
                    client.enableApiControl(True)
                    try:
                        result = client.armDisarm(True)
                        print(f"✅ ARM SUCCESS! Result: {result}")
                        client.armDisarm(False)
                        print("✅ Disarmed safely")
                        return True
                    except Exception as arm_e:
                        print(f"❌ ARM failed even with valid home: {arm_e}")
                        return False
                else:
                    status = "❌ NaN"
                    
                if i % 5 == 0 or i == 29:  # Show every 5 seconds and at end
                    print(f"{i+1:3d}s | Fix {gps_data.gnss.fix_type}   | {gps_coords:22s} | {home_coords:22s} | {status}")
                
            except Exception as e:
                print(f"{i+1:3d}s | ERROR: {e}")
            
            time.sleep(1)
        
        print("\n❌ GPS home location not transmitted after 30 seconds")
        return False
        
    except Exception as e:
        print(f"❌ Debug failed: {e}")
        return False

def show_advanced_fixes():
    """Show advanced troubleshooting steps"""
    print("\n🔧 Advanced Troubleshooting")
    print("=" * 30)
    
    print("1. 🔍 Verify Settings Loading:")
    print("   - Check if AirSim console shows any settings errors")
    print("   - Verify settings.json syntax is valid")
    print()
    
    print("2. 🌐 Network Configuration:")
    print("   - Windows Firewall: Allow AirSim ports 4561, 14541, 14581")
    print("   - WSL2 networking: May need Windows host file updates")
    print()
    
    print("3. 🐧 PX4 Configuration:")
    print("   - Check PX4 logs for GPS parameter warnings")
    print("   - Verify EKF2 parameters are correct")
    print()
    
    print("4. 🔄 Reset Sequence:")
    print("   - Stop AirSim")
    print("   - In WSL2: pkill -f px4")
    print("   - Delete temp files: rm -rf /tmp/px4*")
    print("   - Restart PX4 with: make px4_sitl_default none_iris")
    print("   - Start AirSim")
    print()
    
    print("5. 🎯 Alternative Test:")
    print("   - Try connecting QGroundControl to verify MAVLink GPS")
    print("   - Use: UDP connection to 127.0.0.1:14550")

if __name__ == "__main__":
    success = debug_gps_transmission()
    
    if not success:
        show_advanced_fixes()
        
        # Show current WSL2 and Windows IPs for reference
        try:
            wsl_ip = subprocess.run(['wsl', 'hostname', '-I'], capture_output=True, text=True)
            if wsl_ip.returncode == 0:
                print(f"\n📍 Current WSL2 IP: {wsl_ip.stdout.strip()}")
            
            host_ip = subprocess.run(['wsl', 'bash', '-c', 'ip route show | grep default'], 
                                   capture_output=True, text=True)
            if host_ip.returncode == 0 and 'via' in host_ip.stdout:
                ip = host_ip.stdout.split('via')[1].split()[0]
                print(f"📍 Windows Host IP from WSL2: {ip}")
        except:
            pass 