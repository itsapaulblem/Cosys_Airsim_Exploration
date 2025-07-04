import sys
import os
import time
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def test_gps_data_flow(client, vehicle_name):
    """Test comprehensive GPS data flow"""
    print(f"\n🔍 Testing GPS data flow for {vehicle_name}")
    print("=" * 50)
    
    # 1. Test basic AirSim connection and vehicle state
    try:
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        pos = state.kinematics_estimated.position
        print(f"✅ Vehicle state available")
        print(f"   Position: X={pos.x_val:.2f}, Y={pos.y_val:.2f}, Z={pos.z_val:.2f}")
        
        # Check if vehicle is in simulation
        print(f"   Landed State: {state.landed_state}")
        print(f"   RC Connected: {state.rc_data.is_connected if hasattr(state, 'rc_data') else 'Unknown'}")
        
    except Exception as e:
        print(f"❌ Vehicle state error: {e}")
        return False
        
    # 2. Test GPS sensor data from AirSim
    try:
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        print(f"\n🛰️ AirSim GPS Data:")
        print(f"   Valid: {gps_data.is_valid}")
        print(f"   Time: {gps_data.time_stamp}")
        print(f"   Latitude: {gps_data.gnss.geo_point.latitude:.8f}")
        print(f"   Longitude: {gps_data.gnss.geo_point.longitude:.8f}")
        print(f"   Altitude: {gps_data.gnss.geo_point.altitude:.2f}")
        print(f"   Fix Type: {gps_data.gnss.fix_type}")
        print(f"   Velocity: ({gps_data.gnss.velocity.x_val:.2f}, {gps_data.gnss.velocity.y_val:.2f}, {gps_data.gnss.velocity.z_val:.2f})")
        print(f"   EPH: {gps_data.gnss.eph:.2f}")
        print(f"   EPV: {gps_data.gnss.epv:.2f}")
        
        # Check if GPS data looks reasonable
        if abs(gps_data.gnss.geo_point.latitude) < 0.001:
            print("⚠️  GPS latitude is suspiciously close to 0")
        if abs(gps_data.gnss.geo_point.longitude) < 0.001:
            print("⚠️  GPS longitude is suspiciously close to 0")
        if gps_data.gnss.geo_point.altitude < -1000 or gps_data.gnss.geo_point.altitude > 10000:
            print("⚠️  GPS altitude seems unrealistic")
            
    except Exception as e:
        print(f"❌ GPS data error: {e}")
        return False
        
    # 3. Test home location
    try:
        home = client.getHomeGeoPoint(vehicle_name=vehicle_name)
        print(f"\n🏠 Home Location:")
        print(f"   Latitude: {home.latitude:.8f}")
        print(f"   Longitude: {home.longitude:.8f}")
        print(f"   Altitude: {home.altitude:.2f}")
        
        # Check if home is set (not NaN)
        import math
        if math.isnan(home.latitude) or math.isnan(home.longitude):
            print("⚠️  Home location not set (NaN values)")
            return False
        else:
            print("✅ Home location is properly set")
            
    except Exception as e:
        print(f"❌ Home location error: {e}")
        return False
        
    # 4. Test API control and arming (the critical test)
    try:
        print(f"\n🔧 Testing API control and arming...")
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Small delay to let things settle
        time.sleep(2)
        
        # Try to arm - this is where GPS home location issues manifest
        print("🔧 Attempting to arm drone...")
        client.armDisarm(True, vehicle_name)
        print("✅ Arming successful! GPS data flow is working.")
        
        # Immediately disarm for safety
        client.armDisarm(False, vehicle_name)
        print("✅ Disarmed successfully")
        
        client.enableApiControl(False, vehicle_name)
        return True
        
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        if "GPS home location" in str(e):
            print("💡 This indicates GPS data is not properly flowing from AirSim to PX4")
            print("   Possible causes:")
            print("   - PX4 not receiving GPS data from AirSim")
            print("   - GPS sensor not properly configured in settings.json")
            print("   - MAVLink communication issues")
            print("   - PX4 GPS home location timeout")
        client.enableApiControl(False, vehicle_name)
        return False

def monitor_gps_over_time(client, vehicle_name, duration=30):
    """Monitor GPS data changes over time"""
    print(f"\n📊 Monitoring GPS data for {vehicle_name} over {duration} seconds...")
    
    start_time = time.time()
    last_lat = None
    last_lon = None
    update_count = 0
    
    while time.time() - start_time < duration:
        try:
            gps_data = client.getGpsData(vehicle_name=vehicle_name)
            lat = gps_data.gnss.geo_point.latitude
            lon = gps_data.gnss.geo_point.longitude
            
            if last_lat is not None:
                lat_change = abs(lat - last_lat)
                lon_change = abs(lon - last_lon)
                if lat_change > 1e-8 or lon_change > 1e-8:
                    update_count += 1
                    print(f"📍 GPS update {update_count}: Lat={lat:.8f}, Lon={lon:.8f}")
            
            last_lat = lat
            last_lon = lon
            time.sleep(1)
            
        except Exception as e:
            print(f"❌ GPS monitoring error: {e}")
            break
    
    print(f"📊 GPS updates detected: {update_count} over {duration} seconds")
    if update_count == 0:
        print("⚠️  No GPS coordinate changes detected - GPS might be static or not updating")

def main():
    print("🛰️ AirSim ↔ PX4 GPS Data Flow Diagnostics")
    print("=" * 60)
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim")
        
        # Get list of vehicles to test
        vehicles = client.listVehicles()
        print(f"📋 Available vehicles: {vehicles}")
        
        if not vehicles:
            # Fallback to known vehicle names
            vehicles = ["PX4_Drone1", "PX4_Drone2"]
            print(f"📋 Using fallback vehicle list: {vehicles}")
        
        # Test each vehicle
        for vehicle_name in vehicles:
            success = test_gps_data_flow(client, vehicle_name)
            
            if success:
                print(f"🎯 {vehicle_name}: GPS data flow WORKING correctly")
                
                # Optional: Monitor GPS over time to see if it's updating
                monitor_choice = input(f"\nMonitor {vehicle_name} GPS for 30 seconds? (y/N): ").lower()
                if monitor_choice == 'y':
                    monitor_gps_over_time(client, vehicle_name)
            else:
                print(f"❌ {vehicle_name}: GPS data flow has ISSUES")
                
                # Provide debugging suggestions
                print(f"\n💡 Debugging suggestions for {vehicle_name}:")
                print("   1. Check if PX4 is properly connected to AirSim")
                print("   2. Verify GPS sensor is enabled in settings.json")
                print("   3. Check MAVLink communication logs")
                print("   4. Ensure PX4 is in simulation mode")
                print("   5. Wait longer for GPS lock (sometimes takes 30+ seconds)")
        
        print("\n🏁 GPS diagnostics complete!")
        print("\nKey findings:")
        print("- If arming works: GPS data flow is good")
        print("- If arming fails with 'GPS home location': Data flow issue")
        print("- Check PX4 logs: mavlink start -x -u <port> -r 4000000")
        
    except Exception as e:
        print(f"❌ Diagnostics failed: {e}")

if __name__ == "__main__":
    main() 