#!/usr/bin/env python3
"""
Test AirSim API directly to check GPS home location availability
This replicates the exact issue you're experiencing with orbit.py
"""

import sys
import os
import time
import traceback

# Add the PythonClient directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'PythonClient'))

try:
    import cosysairsim as airsim
    print("✅ cosysairsim module imported successfully")
except ImportError as e:
    print(f"❌ Failed to import cosysairsim: {e}")
    print("Make sure you're running this from the correct environment")
    sys.exit(1)

def test_basic_connection():
    """Test basic AirSim connection"""
    print("\n🔗 Testing Basic AirSim Connection")
    print("=" * 40)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim successfully")
        
        # Get client and server versions
        try:
            client_version = client.getClientVersion()
            server_version = client.getServerVersion()
            print(f"   Client Version: {client_version}")
            print(f"   Server Version: {server_version}")
        except:
            print("   Version info not available")
        
        return client
        
    except Exception as e:
        print(f"❌ Failed to connect to AirSim: {e}")
        print("   Make sure AirSim is running!")
        return None

def test_vehicle_list(client):
    """Test vehicle enumeration"""
    print("\n🚁 Testing Vehicle List")
    print("=" * 40)
    
    try:
        vehicles = client.listVehicles()
        print(f"Available vehicles: {vehicles}")
        
        if not vehicles:
            print("❌ No vehicles found - check AirSim settings.json")
            return None
        
        vehicle_name = vehicles[0] if vehicles else ""
        print(f"✅ Using vehicle: {vehicle_name}")
        return vehicle_name
        
    except Exception as e:
        print(f"❌ Failed to list vehicles: {e}")
        return None

def test_gps_data(client, vehicle_name):
    """Test GPS data retrieval - this should work"""
    print(f"\n🛰️ Testing GPS Data for {vehicle_name}")
    print("=" * 40)
    
    try:
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        
        print(f"GPS Fix Type: {gps_data.fix_type}")
        print(f"GPS Time: {gps_data.time_utc}")
        print(f"Latitude: {gps_data.gnss.geo_point.latitude}")
        print(f"Longitude: {gps_data.gnss.geo_point.longitude}")
        print(f"Altitude: {gps_data.gnss.geo_point.altitude}")
        
        # Check GPS fix quality
        if gps_data.fix_type >= 3:
            print("✅ GPS has 3D fix or better")
            return True
        else:
            print(f"⚠️ GPS fix insufficient: {gps_data.fix_type}")
            return False
            
    except Exception as e:
        print(f"❌ Failed to get GPS data: {e}")
        return False

def test_home_geo_point(client, vehicle_name):
    """Test home location retrieval - this is where the issue might be"""
    print(f"\n🏠 Testing Home Geo Point for {vehicle_name}")
    print("=" * 40)
    
    try:
        home_geo = client.getHomeGeoPoint(vehicle_name=vehicle_name)
        
        print(f"Home Latitude: {home_geo.latitude}")
        print(f"Home Longitude: {home_geo.longitude}")
        print(f"Home Altitude: {home_geo.altitude}")
        
        # Check if coordinates are valid
        lat_valid = not (str(home_geo.latitude).lower() in ['nan', 'inf']) and home_geo.latitude != 0
        lon_valid = not (str(home_geo.longitude).lower() in ['nan', 'inf']) and home_geo.longitude != 0
        
        if lat_valid and lon_valid:
            print("✅ Home location appears valid")
            return True
        else:
            print("❌ Home location invalid (NaN, Inf, or 0,0)")
            print("   This is likely why armDisarm fails!")
            return False
            
    except Exception as e:
        print(f"❌ Failed to get home geo point: {e}")
        return False

def test_vehicle_state(client, vehicle_name):
    """Test vehicle state information"""
    print(f"\n📊 Testing Vehicle State for {vehicle_name}")
    print("=" * 40)
    
    try:
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        
        print(f"Landed State: {state.landed_state}")
        
        # Get position
        pos = state.kinematics_estimated.position
        print(f"Position: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
        
        # Get orientation (quaternion)
        ori = state.kinematics_estimated.orientation
        print(f"Orientation: w={ori.w_val:.3f}, x={ori.x_val:.3f}, y={ori.y_val:.3f}, z={ori.z_val:.3f}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to get vehicle state: {e}")
        return False

def test_api_control(client, vehicle_name):
    """Test API control enabling"""
    print(f"\n🎮 Testing API Control for {vehicle_name}")
    print("=" * 40)
    
    try:
        # Check current API control status
        try:
            enabled = client.isApiControlEnabled(vehicle_name=vehicle_name)
            print(f"API Control currently enabled: {enabled}")
        except:
            print("Cannot check API control status")
        
        # Enable API control
        client.enableApiControl(True, vehicle_name=vehicle_name)
        print("✅ API control enabled successfully")
        
        # Verify it's enabled
        time.sleep(0.5)
        try:
            enabled = client.isApiControlEnabled(vehicle_name=vehicle_name)
            print(f"API Control verified: {enabled}")
        except:
            print("Cannot verify API control status")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to enable API control: {e}")
        return False

def test_arm_disarm_exact_replication(client, vehicle_name):
    """Test the exact armDisarm call that's failing in orbit.py"""
    print(f"\n⚔️ Testing armDisarm (Exact Replication) for {vehicle_name}")
    print("=" * 40)
    print("This replicates the exact call that fails in orbit.py line 72")
    print()
    
    try:
        print("Attempting: client.armDisarm(True)")
        
        # This is the exact call from orbit.py line 72
        result = client.armDisarm(True, vehicle_name)
        
        print(f"✅ armDisarm succeeded! Result: {result}")
        
        # Disarm for safety
        time.sleep(1)
        disarm_result = client.armDisarm(False, vehicle_name)
        print(f"✅ Disarmed successfully! Result: {disarm_result}")
        
        return True
        
    except Exception as e:
        print(f"❌ armDisarm failed with exception:")
        print(f"   Exception type: {type(e).__name__}")
        print(f"   Exception message: {str(e)}")
        
        # Check if it's the specific GPS home location error
        if "GPS home location" in str(e):
            print("\n🎯 CONFIRMED: This is the GPS home location error!")
            print("   PX4 may have GPS home set, but AirSim API cannot see it")
            print("   This indicates a MAVLink communication issue")
        
        return False

def run_comprehensive_test():
    """Run all tests to diagnose the issue"""
    print("🔍 AirSim GPS Home Location API Test")
    print("=" * 60)
    print("This replicates your orbit.py failure to diagnose the root cause")
    print()
    
    # Test basic connection
    client = test_basic_connection()
    if not client:
        print("\n❌ Cannot proceed without AirSim connection")
        return
    
    # Test vehicle list
    vehicle_name = test_vehicle_list(client)
    if not vehicle_name:
        print("\n❌ Cannot proceed without available vehicles")
        return
    
    print(f"\n🎯 Running tests with vehicle: '{vehicle_name}'")
    
    # Run all diagnostic tests
    gps_ok = test_gps_data(client, vehicle_name)
    home_ok = test_home_geo_point(client, vehicle_name)
    state_ok = test_vehicle_state(client, vehicle_name)
    api_ok = test_api_control(client, vehicle_name)
    
    # The critical test - exact replication of your issue
    arm_ok = test_arm_disarm_exact_replication(client, vehicle_name)
    
    # Summary and analysis
    print(f"\n📋 Test Results Summary")
    print("=" * 40)
    print(f"GPS Data Available: {'✅' if gps_ok else '❌'}")
    print(f"Home Geo Point Valid: {'✅' if home_ok else '❌'}")
    print(f"Vehicle State OK: {'✅' if state_ok else '❌'}")
    print(f"API Control OK: {'✅' if api_ok else '❌'}")
    print(f"armDisarm Success: {'✅' if arm_ok else '❌'}")
    
    print(f"\n🔍 Root Cause Analysis")
    print("=" * 40)
    
    if not arm_ok:
        if not home_ok:
            print("🎯 ROOT CAUSE: Home Geo Point Invalid")
            print("   getHomeGeoPoint() returns NaN/0 coordinates")
            print("   This means AirSim API cannot see PX4's GPS home location")
            print("   SOLUTION: Fix MAVLink HOME_POSITION message transmission")
            
        elif not gps_ok:
            print("🎯 ROOT CAUSE: GPS Data Issues")
            print("   GPS fix is insufficient for navigation")
            print("   SOLUTION: Check GPS sensor configuration and fix type")
            
        else:
            print("🎯 ROOT CAUSE: Unknown")
            print("   GPS and home data look good individually")
            print("   SOLUTION: Check MAVLink timing and parameter synchronization")
    else:
        print("✅ ALL TESTS PASSED")
        print("   armDisarm should work normally")
        print("   Your issue may have been resolved!")

    # Always show next steps
    print(f"\n💡 Next Steps")
    print("=" * 40)
    print("1. If home geo point is invalid:")
    print("   - Run: python diagnose_mavlink_home.py")
    print("   - Check MAVLink HOME_POSITION message flow")
    print("   - Verify network configuration (ControlIp/LocalHostIp)")
    print()
    print("2. If all tests pass but orbit.py still fails:")
    print("   - Check vehicle name differences")
    print("   - Try running orbit.py with enableApiControl() first")
    print("   - Add explicit home location waiting in orbit.py")

if __name__ == "__main__":
    try:
        run_comprehensive_test()
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    except Exception as e:
        print(f"\n💥 Unexpected error during testing:")
        print(f"   {type(e).__name__}: {e}")
        traceback.print_exc() 