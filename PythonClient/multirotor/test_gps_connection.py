#!/usr/bin/env python3
import setup_path
import cosysairsim as airsim
import time
import sys

def test_gps_connection():
    """Test GPS connection and home location setup"""
    
    print("🔧 Testing AirSim GPS Connection and Home Location")
    print("=" * 60)
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("✅ Connected to AirSim")
        
        # Try to get home location
        try:
            home_location = client.getHomeGeoPoint()
            print(f"✅ Home Location: Lat={home_location.latitude}, Lon={home_location.longitude}, Alt={home_location.altitude}")
        except Exception as e:
            print(f"❌ Failed to get home location: {e}")
            return False
        
        # Get GPS data
        try:
            gps_data = client.getGpsData()
            print(f"✅ GPS Data: Lat={gps_data.gnss.geo_point.latitude}, Lon={gps_data.gnss.geo_point.longitude}")
            print(f"   GPS Fix Type: {gps_data.gnss.fix_type}")
            print(f"   GPS Time: {gps_data.time_stamp}")
        except Exception as e:
            print(f"❌ Failed to get GPS data: {e}")
            return False
        
        # Check if vehicle state is available
        try:
            state = client.getMultirotorState()
            print(f"✅ Vehicle State: {state.landed_state}")
        except Exception as e:
            print(f"❌ Failed to get vehicle state: {e}")
            return False
        
        # Enable API control
        try:
            client.enableApiControl(True)
            print("✅ API Control enabled")
        except Exception as e:
            print(f"❌ Failed to enable API control: {e}")
            return False
        
        # Try to arm
        try:
            result = client.armDisarm(True)
            print(f"✅ Arm command sent successfully: {result}")
            
            # Wait a bit and check if armed
            time.sleep(2)
            state = client.getMultirotorState()
            print(f"✅ Armed state: {state.armed}")
            
            # Disarm for safety
            client.armDisarm(False)
            print("✅ Disarmed for safety")
            
        except Exception as e:
            print(f"❌ Failed to arm: {e}")
            print("   This is the error you're experiencing!")
            return False
        
        return True
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def check_settings_file():
    """Check if settings file has GPS sensor"""
    import json
    import os
    
    settings_path = os.path.expanduser("~\\Documents\\AirSim\\settings.json")
    
    print(f"\n🔍 Checking settings file: {settings_path}")
    
    try:
        with open(settings_path, 'r') as f:
            settings = json.load(f)
        
        # Check if GPS sensor is configured
        vehicles = settings.get('Vehicles', {})
        
        for vehicle_name, vehicle_config in vehicles.items():
            sensors = vehicle_config.get('Sensors', {})
            has_gps = 'Gps' in sensors
            has_origin = 'OriginGeopoint' in settings
            
            print(f"  Vehicle: {vehicle_name}")
            print(f"    GPS Sensor: {'✅ Found' if has_gps else '❌ Missing'}")
            print(f"    Origin Point: {'✅ Found' if has_origin else '❌ Missing'}")
            
            if has_gps:
                gps_config = sensors['Gps']
                print(f"    GPS Enabled: {'✅' if gps_config.get('Enabled', False) else '❌'}")
        
    except Exception as e:
        print(f"❌ Failed to read settings: {e}")

if __name__ == "__main__":
    check_settings_file()
    
    print("\n" + "=" * 60)
    print("Starting connection test...")
    print("Make sure AirSim is running and PX4 is connected!")
    print("=" * 60)
    
    success = test_gps_connection()
    
    if success:
        print("\n🎉 All tests passed! Your setup should work correctly.")
    else:
        print("\n❌ Some tests failed. Check the errors above.")
        print("\n🔧 Troubleshooting steps:")
        print("1. Make sure AirSim is running")
        print("2. Make sure PX4 Docker container is running and connected")
        print("3. Check that settings.json has GPS sensor configured")
        print("4. Wait a few seconds after starting for GPS to initialize") 