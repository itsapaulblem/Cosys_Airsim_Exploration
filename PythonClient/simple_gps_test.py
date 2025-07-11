#!/usr/bin/env python3
"""
Simple GPS Home Location Test

This script does a basic test to understand why GPS home location isn't being set.
"""

import setup_path
import cosysairsim as airsim
import time

def detailed_gps_check():
    """Check GPS data in detail"""
    print("🛰️ Detailed GPS Check")
    print("=" * 25)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicle = "PX4_Drone1"
        print(f"🚁 Testing {vehicle}:")
        
        # Check GPS data structure
        try:
            gps_data = client.getGpsData(vehicle_name=vehicle)
            print(f"   GPS Data Type: {type(gps_data)}")
            print(f"   GPS Data Dir: {dir(gps_data)}")
            
            # Try to access GPS fields
            if hasattr(gps_data, 'gnss'):
                print(f"   GNSS available: ✅")
                gnss = gps_data.gnss
                
                if hasattr(gnss, 'geo_point'):
                    geo = gnss.geo_point
                    print(f"   Lat: {geo.latitude}")
                    print(f"   Lon: {geo.longitude}")
                    print(f"   Alt: {geo.altitude}")
                else:
                    print(f"   No geo_point in GNSS")
                    
            else:
                print(f"   No GNSS data available")
            
        except Exception as e:
            print(f"   GPS Data Error: {e}")
        
        # Check sensor data directly
        try:
            print(f"\n📊 All Sensor Data:")
            
            # Get barometer
            baro = client.getBarometerData(vehicle_name=vehicle)
            print(f"   Barometer: {baro.altitude:.2f}m, {baro.pressure:.2f}Pa")
            
            # Get IMU
            imu = client.getImuData(vehicle_name=vehicle)
            print(f"   IMU Available: ✅")
            
            # Get magnetometer
            mag = client.getMagnetometerData(vehicle_name=vehicle)
            print(f"   Magnetometer Available: ✅")
            
        except Exception as e:
            print(f"   Sensor Error: {e}")
        
        # Check multiple times to see if GPS data changes
        print(f"\n🕐 GPS Data Over Time:")
        for i in range(5):
            try:
                home = client.getHomeGeoPoint(vehicle_name=vehicle)
                print(f"   T+{i*2}s: {home.latitude:.6f}, {home.longitude:.6f}")
                time.sleep(2)
            except Exception as e:
                print(f"   T+{i*2}s: Error - {e}")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")

def test_manual_home_setting():
    """Try to manually set home position"""
    print(f"\n🏠 Manual Home Setting Test")
    print("=" * 30)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicle = "PX4_Drone1"
        
        # Try to set home manually using the origin from settings
        lat = 47.641468
        lon = -122.140165
        alt = 10.0
        
        print(f"🚁 Attempting to set home for {vehicle}:")
        print(f"   Lat: {lat}")
        print(f"   Lon: {lon}")  
        print(f"   Alt: {alt}")
        
        # Create GeoPoint
        home_point = airsim.GeoPoint()
        home_point.latitude = lat
        home_point.longitude = lon
        home_point.altitude = alt
        
        try:
            # This might not work, but let's try
            result = client.setHomePosition(lat, lon, alt, vehicle_name=vehicle)
            print(f"   Manual set result: {result}")
        except Exception as e:
            print(f"   Manual set failed: {e}")
        
        # Check if it worked
        try:
            home = client.getHomeGeoPoint(vehicle_name=vehicle)
            print(f"   After manual set: {home.latitude:.6f}, {home.longitude:.6f}")
        except Exception as e:
            print(f"   Check after manual set failed: {e}")
    
    except Exception as e:
        print(f"❌ Manual home test failed: {e}")

def test_arm_with_workaround():
    """Try different approaches to enable arming"""
    print(f"\n⚔️ Arming Workaround Test")
    print("=" * 25)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicle = "PX4_Drone1"
        
        print(f"🚁 Testing arming workarounds for {vehicle}:")
        
        # Enable API control
        client.enableApiControl(True, vehicle)
        print(f"   API Control: ✅")
        
        # Try arming with different approaches
        approaches = [
            ("Standard armDisarm", lambda: client.armDisarm(True, vehicle)),
            ("Move to position first", lambda: client.moveToPositionAsync(0, 0, -1, 1, vehicle_name=vehicle)),
            ("Takeoff command", lambda: client.takeoffAsync(vehicle_name=vehicle)),
        ]
        
        for name, func in approaches:
            try:
                print(f"   Trying {name}...")
                result = func()
                if hasattr(result, 'join'):  # Async result
                    result.join()
                print(f"   {name}: ✅ Success")
                
                # If takeoff worked, land
                if "takeoff" in name.lower():
                    client.landAsync(vehicle_name=vehicle).join()
                elif "arm" in name.lower():
                    client.armDisarm(False, vehicle)
                
                break
                
            except Exception as e:
                print(f"   {name}: ❌ {e}")
        
        # Disable API control
        client.enableApiControl(False, vehicle)
        
    except Exception as e:
        print(f"❌ Arming test failed: {e}")

def main():
    print("🔍 Simple GPS Home Location Test")
    print("=" * 35)
    
    detailed_gps_check()
    test_manual_home_setting()
    test_arm_with_workaround()
    
    print(f"\n📋 Analysis")
    print("=" * 15)
    print("This test helps identify:")
    print("1. What GPS data AirSim actually receives")
    print("2. Whether manual home setting works")
    print("3. Alternative approaches to arming")

if __name__ == "__main__":
    main() 