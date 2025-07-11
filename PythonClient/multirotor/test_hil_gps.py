#!/usr/bin/env python3
"""
HIL GPS Test Script
==================

Tests if AirSim is properly sending HIL_GPS messages to PX4
"""

import sys
import os
import time
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def test_hil_gps_flow():
    """Test HIL GPS message flow"""
    print("🔍 Testing HIL GPS Message Flow")
    print("=" * 40)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        vehicle_name = "PX4_Drone1"
        
        # 1. Check if AirSim is generating GPS data
        print(f"\n📡 Checking GPS data generation for {vehicle_name}...")
        for i in range(5):
            gps_data = client.getGpsData(vehicle_name=vehicle_name)
            print(f"   GPS {i+1}: Lat={gps_data.gnss.geo_point.latitude:.8f}, "
                  f"Lon={gps_data.gnss.geo_point.longitude:.8f}, "
                  f"Alt={gps_data.gnss.geo_point.altitude:.2f}, "
                  f"Valid={gps_data.is_valid}")
            time.sleep(1)
        
        # 2. Check GPS home location over time
        print(f"\n🏠 Monitoring GPS home location establishment...")
        for i in range(10):
            try:
                home = client.getHomeGeoPoint(vehicle_name=vehicle_name)
                import math
                
                if not math.isnan(home.latitude) and not math.isnan(home.longitude):
                    print(f"✅ GPS Home Location SET!")
                    print(f"   Lat: {home.latitude:.8f}")
                    print(f"   Lon: {home.longitude:.8f}")
                    print(f"   Alt: {home.altitude:.2f}")
                    return True
                else:
                    print(f"   Attempt {i+1}/10: GPS home still NaN")
                    
            except Exception as e:
                print(f"   Attempt {i+1}/10: Error getting home: {e}")
                
            time.sleep(2)
        
        print("❌ GPS home location never established")
        return False
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def check_px4_hil_mode():
    """Instructions to check PX4 HIL mode"""
    print("\n🔧 PX4 HIL Mode Verification")
    print("=" * 40)
    print("In your PX4 console, run these commands:")
    print("1. param show SYS_HITL")
    print("   Should return: SYS_HITL: curr: 1")
    print("")
    print("2. mavlink status")
    print("   Should show HIL connection with rx > 0 B/s")
    print("")
    print("3. listener HIL_GPS")
    print("   Should show incoming HIL_GPS messages from AirSim")
    print("")
    print("If any of these fail, PX4 is not in proper HIL mode")

def main():
    print("🛰️ HIL GPS Message Flow Test")
    print("=" * 50)
    
    success = test_hil_gps_flow()
    
    if not success:
        print("\n💡 DIAGNOSIS: HIL GPS messages not working properly")
        print("\nPossible causes:")
        print("1. PX4 not started in HIL mode")
        print("2. Network connectivity issues (WSL2 ↔ Windows)")
        print("3. AirSim not sending HIL_GPS messages")
        print("4. PX4 not accepting simulated GPS data")
        
        check_px4_hil_mode()
        
        print("\n🔧 SOLUTIONS:")
        print("1. Restart PX4 with HIL mode:")
        print("   PX4_SYS_AUTOSTART=4001 HIL_MODE=1 ./Tools/simulation/sitl/sitl_bin.sh ...")
        print("")
        print("2. Force HIL mode in PX4 console:")
        print("   param set SYS_HITL 1")
        print("   reboot")
        print("")
        print("3. Check firewall/network issues")
        
    else:
        print("🎉 HIL GPS flow is working!")
        print("Try arming your drone now.")

if __name__ == "__main__":
    main() 