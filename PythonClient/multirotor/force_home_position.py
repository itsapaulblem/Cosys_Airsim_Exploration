#!/usr/bin/env python3
"""
Force set home position for PX4 in AirSim when GPS is working but home is NaN
"""
import setup_path
import cosysairsim as airsim
import time
import math

def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    print("🏠 Force Home Position Setter")
    print("=" * 50)
    
    # Get default vehicle
    vehicle_name = ""
    try:
        state = client.getMultirotorState(vehicle_name)
        print(f"Using default vehicle")
    except:
        vehicle_name = "PX4_Drone1"
        print(f"Using vehicle: {vehicle_name}")
    
    try:
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Get current GPS position
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        
        print(f"\n📍 Current GPS Position:")
        print(f"  Latitude:  {gps_data.gnss.geo_point.latitude:.8f}")
        print(f"  Longitude: {gps_data.gnss.geo_point.longitude:.8f}")
        print(f"  Altitude:  {gps_data.gnss.geo_point.altitude:.2f} m")
        print(f"  Valid:     {gps_data.is_valid}")
        print(f"  Fix Type:  {gps_data.gnss.fix_type}")
        
        if not gps_data.is_valid or gps_data.gnss.fix_type < 3:
            print("❌ GPS not ready for home position setting")
            return
            
        # Force set home position using current GPS coordinates
        home_geopoint = airsim.GeoPoint(
            latitude=gps_data.gnss.geo_point.latitude,
            longitude=gps_data.gnss.geo_point.longitude,
            altitude=gps_data.gnss.geo_point.altitude
        )
        
        print(f"\n🎯 Setting home position to current GPS location...")
        
        # Try to set home position
        try:
            # Method 1: Try direct home position setting (if available)
            client.setHomePosition(
                gps_data.gnss.geo_point.latitude,
                gps_data.gnss.geo_point.longitude,
                gps_data.gnss.geo_point.altitude,
                vehicle_name
            )
            print("✅ Home position set using setHomePosition()")
        except Exception as e:
            print(f"⚠ setHomePosition() not available: {e}")
            
            # Method 2: Try moving to current position to establish home
            try:
                print("🔄 Attempting to establish home by moving to current position...")
                client.moveToPositionAsync(0, 0, -2, 1, vehicle_name=vehicle_name).join()
                time.sleep(2)
                print("✅ Position command sent")
            except Exception as e2:
                print(f"⚠ Position command failed: {e2}")
        
        # Wait a moment for PX4 to process
        print("\n⏳ Waiting for PX4 to establish home position...")
        for i in range(10):
            time.sleep(1)
            home_point = client.getHomeGeoPoint(vehicle_name=vehicle_name)
            
            if not (math.isnan(home_point.latitude) or math.isnan(home_point.longitude)):
                print("✅ SUCCESS! Home position established:")
                print(f"  Latitude:  {home_point.latitude:.8f}")
                print(f"  Longitude: {home_point.longitude:.8f}")
                print(f"  Altitude:  {home_point.altitude:.2f} m")
                break
            else:
                print(f"   Attempt {i+1}/10: Still waiting...")
        else:
            print("❌ Home position still not set after 10 seconds")
            print("\n💡 Potential solutions:")
            print("   1. Restart PX4 Docker container")
            print("   2. Check PX4 parameter COM_HOME_IN_AIR=1")
            print("   3. Wait longer - some PX4 versions need more time")
            print("   4. Try arming/disarming the vehicle")
            
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

if __name__ == "__main__":
    main()