#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math

def simple_gps_fix():
    """Simple fix for PX4 home location issue"""
    print("🔧 Simple GPS Home Location Fix")
    print("=" * 40)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("🔄 Step 1: Reset and wait for initialization...")
        client.reset()
        time.sleep(10)  # Longer wait
        
        print("🔄 Step 2: Enable API control...")
        client.enableApiControl(True)
        time.sleep(2)
        
        print("🔄 Step 3: Check GPS status...")
        gps_data = client.getGpsData()
        print(f"  GPS Fix: {gps_data.gnss.fix_type}")
        print(f"  GPS Coords: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}")
        
        print("🔄 Step 4: Waiting for home location (up to 60 seconds)...")
        for attempt in range(60):
            home_location = client.getHomeGeoPoint()
            if not math.isnan(home_location.latitude):
                print(f"✅ SUCCESS! Home location set after {attempt+1} seconds")
                print(f"  Home: {home_location.latitude:.6f}, {home_location.longitude:.6f}")
                
                # Test arm
                print("🔧 Testing arm...")
                try:
                    result = client.armDisarm(True)
                    print(f"✅ Arm successful: {result}")
                    
                    # Disarm for safety
                    client.armDisarm(False)
                    print("✅ Disarmed. GPS fix complete!")
                    return True
                    
                except Exception as e:
                    print(f"❌ Arm failed even with home location: {e}")
                    return False
                    
            if attempt % 10 == 0:
                print(f"  Waiting... {attempt+1}/60 seconds")
                
            time.sleep(1)
        
        print("❌ Home location not set after 60 seconds")
        return False
        
    except Exception as e:
        print(f"❌ Fix failed: {e}")
        return False

if __name__ == "__main__":
    success = simple_gps_fix()
    
    if success:
        print("\n🎉 GPS Home Location Fixed!")
        print("✅ You can now run your takeoff scripts")
        print("\n💡 To test:")
        print("   cd multirotor")
        print("   python takeoff.py")
    else:
        print("\n❌ GPS fix failed. Try these solutions:")
        print("1. Restart AirSim completely")
        print("2. Restart PX4 container: docker restart px4-single")  
        print("3. Wait even longer (some setups need 2-3 minutes)")
        print("4. Check PX4 is actually connected to AirSim") 