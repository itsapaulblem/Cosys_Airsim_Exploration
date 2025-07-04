import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def set_gps_home_manual(client, vehicle_name):
    """Manually set GPS home location for a drone"""
    try:
        # Get current GPS data
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        lat = gps_data.gnss.geo_point.latitude
        lon = gps_data.gnss.geo_point.longitude
        alt = gps_data.gnss.geo_point.altitude
        
        print(f"📍 {vehicle_name} GPS: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}")
        
        # Try to set home position (this might work in some PX4 versions)
        try:
            # This is a theoretical call - might not be available in all AirSim versions
            client.setHomeGeoPoint(lat, lon, alt, vehicle_name=vehicle_name)
            print(f"✅ GPS home set manually for {vehicle_name}")
            return True
        except:
            print(f"⚠️ Manual GPS home setting not available via API")
            
        # Alternative: Just wait and let PX4 establish home naturally
        print(f"⏳ Waiting for {vehicle_name} to establish GPS home naturally...")
        
        for i in range(20):  # Wait up to 60 seconds
            try:
                client.armDisarm(True, vehicle_name)
                client.armDisarm(False, vehicle_name)
                print(f"✅ {vehicle_name} GPS home established!")
                return True
            except Exception as e:
                if "GPS home location" in str(e):
                    print(f"⏳ Still waiting... ({i*3}s)")
                    time.sleep(3)
                else:
                    print(f"❌ Different error: {e}")
                    return False
        
        return False
        
    except Exception as e:
        print(f"❌ Error setting GPS home for {vehicle_name}: {e}")
        return False

def main():
    print("🛰️ GPS Home Location Setup Script")
    print("=================================")
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected to AirSim")
    
    # Test drones
    drones = ["PX4_Drone1", "PX4_Drone2"]
    
    for drone in drones:
        print(f"\n🔧 Setting up GPS home for {drone}...")
        client.enableApiControl(True, drone)
        
        if set_gps_home_manual(client, drone):
            print(f"🎯 {drone} is ready!")
        else:
            print(f"❌ {drone} GPS home setup failed")
            
        client.enableApiControl(False, drone)
    
    print("\n✅ GPS home setup complete!")

if __name__ == "__main__":
    main() 