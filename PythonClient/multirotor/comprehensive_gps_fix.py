#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math

def wait_for_px4_connection():
    """Wait for PX4 to properly connect to AirSim"""
    print("🔗 Waiting for PX4 connection...")
    
    client = airsim.MultirotorClient()
    
    for attempt in range(30):
        try:
            client.confirmConnection()
            gps_data = client.getGpsData()
            
            # Check if we're getting valid GPS data
            if (gps_data.gnss.fix_type >= 3 and 
                not math.isnan(gps_data.gnss.geo_point.latitude) and
                abs(gps_data.gnss.geo_point.latitude) > 0.1):
                
                print(f"✅ PX4 connected! GPS Fix: {gps_data.gnss.fix_type}")
                return client
                
        except Exception as e:
            print(f"  Attempt {attempt+1}/30: {e}")
            
        time.sleep(2)
    
    raise Exception("❌ PX4 connection timeout")

def force_home_location_set(client):
    """Try multiple methods to force home location setting"""
    print("\n🏠 Attempting to force home location setting...")
    
    # Method 1: Reset and enable API control multiple times
    print("🔄 Method 1: Reset sequence...")
    for i in range(3):
        client.reset()
        time.sleep(5)
        client.enableApiControl(True)
        time.sleep(3)
        
        home = client.getHomeGeoPoint()
        if not math.isnan(home.latitude):
            print(f"✅ Home set after reset {i+1}!")
            return True
    
    # Method 2: Manual position setting
    print("🔄 Method 2: Manual position setting...")
    try:
        from cosysairsim.types import Pose, Vector3r, Quaternionr
        
        # Set vehicle to exact origin
        pose = Pose()
        pose.position = Vector3r(0, 0, 0)
        pose.orientation = Quaternionr(0, 0, 0, 1)
        
        client.simSetVehiclePose(pose, True)
        time.sleep(5)
        
        home = client.getHomeGeoPoint()
        if not math.isnan(home.latitude):
            print("✅ Home set after manual positioning!")
            return True
            
    except Exception as e:
        print(f"  Manual positioning failed: {e}")
    
    # Method 3: Kinematics reset
    print("🔄 Method 3: Kinematics reset...")
    try:
        from cosysairsim.types import KinematicsState, Vector3r, Quaternionr
        
        state = KinematicsState()
        state.position = Vector3r(0, 0, 0)
        state.orientation = Quaternionr(0, 0, 0, 1)
        state.linear_velocity = Vector3r(0, 0, 0)
        state.angular_velocity = Vector3r(0, 0, 0)
        
        client.simSetKinematics(state, True)
        time.sleep(5)
        
        home = client.getHomeGeoPoint()
        if not math.isnan(home.latitude):
            print("✅ Home set after kinematics reset!")
            return True
            
    except Exception as e:
        print(f"  Kinematics reset failed: {e}")
    
    return False

def comprehensive_gps_fix():
    """Comprehensive GPS fix procedure"""
    print("🚁 Comprehensive GPS Home Location Fix")
    print("=" * 50)
    
    try:
        # Step 1: Ensure PX4 connection
        client = wait_for_px4_connection()
        
        # Step 2: Initial status check
        gps_data = client.getGpsData()
        home_location = client.getHomeGeoPoint()
        
        print(f"\n📍 Initial Status:")
        print(f"  GPS: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}")
        print(f"  Fix Type: {gps_data.gnss.fix_type}")
        print(f"  Home: {home_location.latitude}, {home_location.longitude}")
        
        # Step 3: Check if home is already valid
        if not math.isnan(home_location.latitude):
            print("✅ Home location already valid!")
        else:
            # Step 4: Try to force home location
            if not force_home_location_set(client):
                print("❌ Could not set home location with standard methods")
                
                # Step 5: Last resort - wait very long
                print("🔄 Last resort: Extended wait (120 seconds)...")
                for i in range(120):
                    home = client.getHomeGeoPoint()
                    if not math.isnan(home.latitude):
                        print(f"✅ Home finally set after {i+1} seconds!")
                        break
                    if i % 20 == 0:
                        print(f"  Still waiting... {i+1}/120 seconds")
                    time.sleep(1)
                else:
                    print("❌ Home location never set - this indicates a deeper PX4 configuration issue")
                    return False
        
        # Step 6: Test arming
        print("\n🔧 Testing arm sequence...")
        client.enableApiControl(True)
        
        try:
            result = client.armDisarm(True)
            print(f"✅ ARM SUCCESS! Result: {result}")
            
            # Test basic takeoff
            print("🚀 Testing takeoff...")
            takeoff_task = client.takeoffAsync(timeout_sec=15)
            takeoff_task.join()
            print("✅ TAKEOFF SUCCESS!")
            
            # Land and disarm
            print("🛬 Landing...")
            land_task = client.landAsync()
            land_task.join()
            
            client.armDisarm(False)
            print("✅ COMPLETE SUCCESS!")
            return True
            
        except Exception as e:
            print(f"❌ Arm failed: {e}")
            
            # Final diagnostics
            print("\n🔍 Final Diagnostics:")
            try:
                home = client.getHomeGeoPoint()
                gps = client.getGpsData()
                state = client.getMultirotorState()
                
                print(f"  Home: {home.latitude}, {home.longitude}, {home.altitude}")
                print(f"  GPS: {gps.gnss.geo_point.latitude:.6f}, {gps.gnss.geo_point.longitude:.6f}")
                print(f"  GPS Fix: {gps.gnss.fix_type}")
                print(f"  Vehicle State: {state.landed_state}")
                
            except Exception as diag_e:
                print(f"  Diagnostics failed: {diag_e}")
                
            return False
    
    except Exception as e:
        print(f"❌ Comprehensive fix failed: {e}")
        return False

if __name__ == "__main__":
    success = comprehensive_gps_fix()
    
    if success:
        print("\n🎉 GPS HOME LOCATION COMPLETELY FIXED!")
        print("✅ Your drone is ready for takeoff")
        print("\n💡 Next steps:")
        print("   cd multirotor")
        print("   python takeoff.py")
    else:
        print("\n❌ GPS fix unsuccessful. This suggests a fundamental PX4 configuration issue.")
        print("\n🔧 Advanced troubleshooting needed:")
        print("1. Check PX4 parameters: EKF2_AID_MASK, EKF2_HGT_MODE")
        print("2. Verify PX4 is receiving MAVLink heartbeat from AirSim")
        print("3. Consider using PX4 parameter: COM_ARM_WO_GPS")
        print("4. Check if PX4 SITL is properly configured for external simulator")
        print("\n📝 Please share the diagnostic output for further analysis.") 