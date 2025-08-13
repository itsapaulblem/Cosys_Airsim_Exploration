#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math

def test_gps_transmission():
    """Test GPS data transmission from AirSim to PX4"""
    print("🛰️ Testing GPS Data Transmission (AirSim → PX4)")
    print("=" * 55)
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        print("✅ Connected to AirSim")
        
        # Wait for GPS to initialize
        print("⏳ Waiting for GPS initialization (30 seconds)...")
        for i in range(30):
            try:
                gps_data = client.getGpsData()
                home_location = client.getHomeGeoPoint()
                
                print(f"  {i+1:2d}s: GPS Fix={gps_data.gnss.fix_type}, "
                      f"GPS=({gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}), "
                      f"Home=({home_location.latitude}, {home_location.longitude})")
                
                # Check if home location is now valid
                if not math.isnan(home_location.latitude):
                    print(f"\n✅ SUCCESS! Home location transmitted after {i+1} seconds!")
                    print(f"   Home: {home_location.latitude:.6f}, {home_location.longitude:.6f}, {home_location.altitude:.1f}")
                    
                    # Test arming now that home is set
                    print("\n🔧 Testing arm with valid home location...")
                    client.enableApiControl(True)
                    
                    try:
                        result = client.armDisarm(True)
                        print(f"✅ ARM SUCCESS! Result: {result}")
                        
                        # Disarm for safety
                        client.armDisarm(False)
                        print("✅ Disarmed. GPS transmission working correctly!")
                        return True
                        
                    except Exception as arm_e:
                        print(f"❌ Arm failed despite home location: {arm_e}")
                        return False
                
            except Exception as e:
                print(f"  {i+1:2d}s: Error getting GPS data: {e}")
            
            time.sleep(1)
        
        print("❌ Home location never transmitted after 30 seconds")
        return False
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def diagnose_gps_transmission_issues():
    """Diagnose why GPS data might not be transmitting"""
    print("\n🔍 Diagnosing GPS Transmission Issues")
    print("-" * 40)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Check network connectivity
        print("1. 🌐 Network connectivity:")
        try:
            ping_result = client.ping()
            print(f"   AirSim ping: {ping_result}")
        except Exception as e:
            print(f"   ❌ Ping failed: {e}")
        
        # Check GPS sensor data
        print("\n2. 🛰️ GPS sensor data:")
        try:
            gps_data = client.getGpsData()
            print(f"   Fix type: {gps_data.gnss.fix_type}")
            print(f"   Latitude: {gps_data.gnss.geo_point.latitude:.6f}")
            print(f"   Longitude: {gps_data.gnss.geo_point.longitude:.6f}")
            print(f"   Altitude: {gps_data.gnss.geo_point.altitude:.1f}")
            print(f"   Timestamp: {gps_data.time_stamp}")
        except Exception as e:
            print(f"   ❌ GPS data failed: {e}")
        
        # Check home location
        print("\n3. 🏠 Home location:")
        try:
            home = client.getHomeGeoPoint()
            print(f"   Latitude: {home.latitude}")
            print(f"   Longitude: {home.longitude}")
            print(f"   Altitude: {home.altitude}")
            
            if math.isnan(home.latitude):
                print("   ❌ Home location is NaN - GPS data not being transmitted to PX4!")
            else:
                print("   ✅ Home location is valid")
        except Exception as e:
            print(f"   ❌ Home location failed: {e}")
        
        # Check vehicle state
        print("\n4. 🚁 Vehicle state:")
        try:
            state = client.getMultirotorState()
            print(f"   Landed state: {state.landed_state}")
            print(f"   Position: {state.kinematics_estimated.position}")
        except Exception as e:
            print(f"   ❌ Vehicle state failed: {e}")
            
    except Exception as e:
        print(f"❌ Diagnosis failed: {e}")

def show_transmission_fixes():
    """Show potential fixes for GPS transmission issues"""
    print("\n🔧 GPS Transmission Fixes")
    print("-" * 30)
    
    print("If GPS data is not transmitting to PX4, try these fixes:")
    print()
    print("1. 📁 Update settings.json with correct network configuration:")
    print("   - ControlIp: '172.25.0.10' (Docker container IP)")
    print("   - LocalHostIp: '172.25.0.10'")
    print("   - Complete GPS sensor configuration")
    print()
    print("2. 🔄 Restart sequence:")
    print("   - Stop AirSim")
    print("   - Restart PX4 container: docker restart px4-single")
    print("   - Start AirSim")
    print("   - Wait 30-60 seconds for GPS initialization")
    print()
    print("3. 🛠️ Check PX4 parameters:")
    print("   docker exec px4-single /Scripts/px4_shell.sh 'param show EKF2_GPS_CHECK'")
    print("   docker exec px4-single /Scripts/px4_shell.sh 'param show COM_ARM_WO_GPS'")
    print()
    print("4. 📡 Verify MAVLink connection:")
    print("   docker logs px4-single --tail 20")
    print("   Look for MAVLink heartbeat messages")

if __name__ == "__main__":
    success = test_gps_transmission()
    
    if not success:
        diagnose_gps_transmission_issues()
        show_transmission_fixes()
        
        print("\n🎯 Quick Fix:")
        print("1. Copy the corrected settings: Copy-Item 'fixed_gps_transmission_settings.json' 'c:\\Users\\Admin\\Documents\\AirSim\\settings.json' -Force")
        print("2. Restart AirSim")
        print("3. Wait 60 seconds for GPS data transmission")
        print("4. Re-run this test")
    else:
        print("\n🎉 GPS transmission working correctly!")
        print("✅ You can now use your takeoff scripts successfully.") 