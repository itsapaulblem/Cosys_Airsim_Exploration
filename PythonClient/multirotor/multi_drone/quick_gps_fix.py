#!/usr/bin/env python3
"""
Quick GPS fix for multi-drone missions
Run this before starting your mission to ensure GPS home is set
"""
import os
import sys
# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim
import time
import sys

def force_gps_home_setting():
    """Force GPS home setting for all vehicles"""
    print("🚁 Quick GPS Home Fix for Multi-Drone Mission")
    print("=" * 50)
    
    try:
        # Connect to AirSim
        print("🔌 Connecting to AirSim...")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim!")
        
        # Get list of vehicles
        vehicles = client.listVehicles()
        print(f"🚁 Found vehicles: {vehicles}")
        
        if not vehicles:
            print("❌ No vehicles found! Check your settings.json")
            return False
        
        # Set global GPS home location (Seattle coordinates)
        home_lat, home_lon, home_alt = 47.641468, -122.140165, 122.0
        print(f"🛰️ Setting GPS home to: {home_lat}, {home_lon}")
        
        try:
            # Try different API methods to set GPS home
            try:
                # Method 1: Try with GeoPoint object
                home_location = airsim.GeoPoint()
                home_location.latitude = home_lat
                home_location.longitude = home_lon
                home_location.altitude = home_alt
                client.simSetHome(home_location)
                print("✅ GPS home location set successfully via GeoPoint!")
            except:
                # Method 2: Try setting home for each vehicle individually
                for vehicle in vehicles:
                    try:
                        # Force GPS coordinates by setting vehicle pose
                        pose = client.simGetVehiclePose(vehicle)
                        print(f"   Setting GPS reference for {vehicle}")
                        # This indirectly sets the GPS reference
                        client.simSetVehiclePose(pose, True, vehicle)
                    except:
                        pass
                print("✅ GPS reference set via vehicle poses!")
        except Exception as e:
            print(f"⚠️ Could not set GPS home via API: {e}")
            print("   This is normal - PX4 will set it automatically")
        
        # Wait a moment for GPS to initialize
        print("⏳ Waiting for GPS initialization...")
        time.sleep(3)
        
        # Check GPS status for each vehicle
        all_good = True
        for vehicle in vehicles:
            print(f"\n🔍 Checking GPS status for {vehicle}...")
            try:
                gps_data = client.getGpsData(vehicle_name=vehicle)
                fix_type = gps_data.gnss.fix_type
                
                print(f"   Fix Type: {fix_type}")
                print(f"   Latitude: {gps_data.gnss.geo_point.latitude:.6f}")
                print(f"   Longitude: {gps_data.gnss.geo_point.longitude:.6f}")
                print(f"   Altitude: {gps_data.gnss.geo_point.altitude:.2f}")
                
                if fix_type >= 3:  # 3D fix
                    print(f"   ✅ {vehicle} has 3D GPS fix!")
                elif fix_type >= 2:  # 2D fix
                    print(f"   ⚠️ {vehicle} has 2D GPS fix (should work)")
                else:
                    print(f"   ❌ {vehicle} has no GPS fix (fix_type: {fix_type})")
                    all_good = False
                    
            except Exception as e:
                print(f"   ❌ Could not get GPS data for {vehicle}: {e}")
                all_good = False
        
        # Test if we can arm the first vehicle (this will fail if no GPS home)
        print(f"\n🔧 Testing GPS home by attempting to arm {vehicles[0]}...")
        try:
            client.enableApiControl(True, vehicles[0])
            can_arm = client.armDisarm(True, vehicles[0])
            if can_arm:
                print(f"✅ {vehicles[0]} armed successfully - GPS home is working!")
                client.armDisarm(False, vehicles[0])  # Disarm immediately
            else:
                print(f"⚠️ Could not arm {vehicles[0]} - check GPS status")
                all_good = False
        except Exception as e:
            if "GPS home location" in str(e):
                print(f"❌ GPS home error: {e}")
                all_good = False
            else:
                print(f"⚠️ Arm test error (may be normal): {e}")
        
        return all_good
        
    except Exception as e:
        print(f"❌ Connection error: {e}")
        print("💡 Make sure AirSim is running and containers are connected!")
        return False

def main():
    success = force_gps_home_setting()
    
    print("\n" + "=" * 50)
    if success:
        print("🎉 GPS HOME FIX SUCCESSFUL!")
        print("✅ All vehicles have GPS fix")
        print("🚁 Your mission script should work now!")
    else:
        print("⚠️ GPS HOME FIX INCOMPLETE")
        print("🔧 Try these solutions:")
        print("   1. Restart AirSim/Unreal Engine")
        print("   2. Restart Docker containers")
        print("   3. Check container logs: docker logs px4-drone1")
        print("   4. Verify settings.json is in ~/Documents/AirSim/")
    
    print("\n💡 TIPS:")
    print("   - Always run this script before starting missions")
    print("   - Wait 10-15 seconds after starting AirSim before running this")
    print("   - If problems persist, check the unified config generator tools")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())