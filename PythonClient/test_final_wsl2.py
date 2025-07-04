#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math

def test_final_wsl2_setup():
    """Final test for WSL2 PX4 GPS home location transmission"""
    print("🏁 Final WSL2 GPS Home Location Test")
    print("=" * 40)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        # Wait a bit for initialization
        print("⏳ Waiting for GPS home location transmission...")
        
        for i in range(30):  # Test for 30 seconds
            try:
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                print(f"  {i+1:2d}s: GPS Fix={gps_data.gnss.fix_type}, "
                      f"GPS=({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}), "
                      f"Home=({home_location.latitude}, {home_location.longitude})")
                
                # Check if home location is now valid
                if not math.isnan(home_location.latitude):
                    print(f"\n🎉 SUCCESS! GPS home location transmitted after {i+1} seconds!")
                    print(f"   📍 Home: {home_location.latitude:.6f}, {home_location.longitude:.6f}, {home_location.altitude:.1f}")
                    
                    # Test arming
                    print("\n🔧 Testing arm with valid home location...")
                    client.enableApiControl(True)
                    
                    try:
                        result = client.armDisarm(True)
                        print(f"✅ ARM SUCCESS! Result: {result}")
                        print("🎯 WSL2 GPS transmission working perfectly!")
                        
                        # Disarm for safety
                        client.armDisarm(False)
                        print("✅ Disarmed safely")
                        return True
                        
                    except Exception as arm_e:
                        print(f"❌ Arm failed: {arm_e}")
                        print("💡 Check PX4 parameters and EKF2 settings")
                        return False
                
            except Exception as e:
                print(f"  {i+1:2d}s: Error: {e}")
            
            time.sleep(1)
        
        print("❌ GPS home location still not transmitted after 30 seconds")
        print("💡 You may need to restart AirSim for settings to take effect")
        return False
        
    except Exception as e:
        print(f"❌ Connection test failed: {e}")
        return False

if __name__ == "__main__":
    print("🐧 Testing WSL2 PX4 with corrected Windows host IP (172.28.240.1)")
    print("📋 Make sure:")
    print("   - AirSim is restarted after settings update")
    print("   - PX4 is running in WSL2") 
    print("   - PX4 shows 'Simulator connected on TCP port 4561'")
    print()
    
    success = test_final_wsl2_setup()
    
    if success:
        print("\n🎉 WSL2 Setup Complete!")
        print("✅ Your takeoff scripts should now work")
    else:
        print("\n🔧 Troubleshooting Tips:")
        print("1. Restart AirSim completely")
        print("2. Check PX4 shows: 'using TCP on remote host 172.28.240.1 port 4561'")
        print("3. Verify no firewall blocking port 4561")
        print("4. Re-run this test") 