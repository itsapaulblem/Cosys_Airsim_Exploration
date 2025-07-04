#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math
import subprocess

def get_wsl2_ip():
    """Get the current WSL2 IP address"""
    try:
        result = subprocess.run(['wsl', 'hostname', '-I'], capture_output=True, text=True)
        if result.returncode == 0:
            ip = result.stdout.strip()
            print(f"📍 WSL2 IP Address: {ip}")
            return ip
        else:
            print("❌ Failed to get WSL2 IP")
            return None
    except Exception as e:
        print(f"❌ Error getting WSL2 IP: {e}")
        return None

def test_wsl2_px4_connection():
    """Test connection to PX4 running in WSL2"""
    print("🐧 Testing WSL2 PX4 Connection & GPS Home Location")
    print("=" * 55)
    
    # Get WSL2 IP
    wsl_ip = get_wsl2_ip()
    if not wsl_ip:
        return False
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("✅ Connected to AirSim")
        
        # Test connection for 60 seconds (WSL2 can be slower to initialize)
        print("⏳ Testing GPS home location transmission (60 seconds)...")
        
        for i in range(60):
            try:
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                # Show progress every 10 seconds
                if i % 10 == 0 or not math.isnan(home_location.latitude):
                    print(f"  {i+1:2d}s: GPS Fix={gps_data.gnss.fix_type}, "
                          f"GPS=({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}), "
                          f"Home=({home_location.latitude}, {home_location.longitude})")
                
                # Check if home location is now valid
                if not math.isnan(home_location.latitude):
                    print(f"\n✅ SUCCESS! GPS home location transmitted after {i+1} seconds!")
                    print(f"   Home: {home_location.latitude:.6f}, {home_location.longitude:.6f}, {home_location.altitude:.1f}")
                    
                    # Test arming
                    print("\n🔧 Testing arm with valid home location...")
                    client.enableApiControl(True)
                    
                    try:
                        result = client.armDisarm(True)
                        print(f"✅ ARM SUCCESS! Result: {result}")
                        
                        # Disarm for safety
                        client.armDisarm(False)
                        print("✅ Disarmed. WSL2 GPS transmission working correctly!")
                        return True
                        
                    except Exception as arm_e:
                        print(f"❌ Arm failed: {arm_e}")
                        return False
                
            except Exception as e:
                print(f"  {i+1:2d}s: Error: {e}")
            
            time.sleep(1)
        
        print("❌ GPS home location not transmitted after 60 seconds")
        return False
        
    except Exception as e:
        print(f"❌ Connection test failed: {e}")
        return False

def check_wsl2_px4_status():
    """Check if PX4 is running in WSL2"""
    print("\n🔍 Checking WSL2 PX4 Status")
    print("-" * 30)
    
    try:
        # Check if PX4 process is running in WSL2
        result = subprocess.run(['wsl', 'pgrep', '-f', 'px4'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ PX4 process found in WSL2")
            print(f"   PIDs: {result.stdout.strip()}")
        else:
            print("❌ PX4 process not found in WSL2")
            print("   Make sure PX4 SITL is running in WSL2")
        
        # Check network connectivity
        wsl_ip = get_wsl2_ip()
        if wsl_ip:
            # Test if port 4561 is open
            result = subprocess.run(['wsl', 'netstat', '-ln'], capture_output=True, text=True)
            if '4561' in result.stdout:
                print("✅ Port 4561 is listening in WSL2")
            else:
                print("❌ Port 4561 not found in WSL2")
                print("   PX4 may not be configured for AirSim connection")
        
    except Exception as e:
        print(f"❌ WSL2 status check failed: {e}")

def show_wsl2_fixes():
    """Show fixes for WSL2 PX4 issues"""
    print("\n🔧 WSL2 PX4 Setup Fixes")
    print("-" * 25)
    
    wsl_ip = get_wsl2_ip()
    
    print("If GPS home location is not working with WSL2:")
    print()
    print("1. 📁 Update AirSim settings.json:")
    print(f"   ControlIp: '{wsl_ip}'")
    print(f"   LocalHostIp: '{wsl_ip}'")
    print("   (WSL2 IP can change on reboot!)")
    print()
    print("2. 🐧 In WSL2, make sure PX4 is configured for AirSim:")
    print("   export PX4_SIM_HOSTNAME=172.28.248.1  # Windows host IP from WSL2")
    print("   make px4_sitl_default none_iris")
    print()
    print("3. 🔄 Restart sequence:")
    print("   - Stop AirSim")
    print("   - In WSL2: pkill -f px4")
    print("   - Restart PX4 SITL in WSL2")
    print("   - Update settings.json with current WSL2 IP")
    print("   - Start AirSim")
    print()
    print("4. 🌐 Check Windows Firewall:")
    print("   - Allow AirSim through Windows Firewall")
    print("   - Allow ports 4561, 14541, 14581")

if __name__ == "__main__":
    success = test_wsl2_px4_connection()
    
    if not success:
        check_wsl2_px4_status()
        show_wsl2_fixes()
        
        print("\n🎯 Quick Fix Steps:")
        print("1. Copy WSL2 settings: Copy-Item 'wsl2_settings.json' 'c:\\Users\\Admin\\Documents\\AirSim\\settings.json' -Force")
        print("2. Restart AirSim")
        print("3. Make sure PX4 is running in WSL2")
        print("4. Re-run this test")
    else:
        print("\n🎉 WSL2 PX4 GPS transmission working!")
        print("✅ You can now use your takeoff scripts.") 