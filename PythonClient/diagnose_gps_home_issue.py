#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
from pymavlink import mavutil
import sys

def diagnose_vehicle_gps_home(vehicle_name, mavlink_port):
    """
    Comprehensive diagnosis of GPS home position issue for a single vehicle
    """
    print(f"\n🔍 Diagnosing GPS Home Issue for {vehicle_name}")
    print("=" * 60)
    
    # Connect to AirSim first
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=5)
        client.confirmConnection()
        print("✅ AirSim connection successful")
    except Exception as e:
        print(f"❌ AirSim connection failed: {e}")
        return False
    
    # Step 1: Check AirSim GPS status
    print("\n1️⃣ AirSim GPS Status")
    print("-" * 30)
    try:
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        
        print(f"GPS Fix Type: {gps_data.gnss.fix_type}")
        print(f"Satellites Visible: {gps_data.gnss.satellites_visible}")
        print(f"GPS Coordinates: {gps_data.gnss.geo_point.latitude:.6f}, {gps_data.gnss.geo_point.longitude:.6f}, {gps_data.gnss.geo_point.altitude:.2f}")
        print(f"Vehicle Position (NED): {state.kinematics_estimated.position.x_val:.2f}, {state.kinematics_estimated.position.y_val:.2f}, {state.kinematics_estimated.position.z_val:.2f}")
        print(f"Armed State: {state.armed}")
        
        # Try to get home position via AirSim
        try:
            home_pos = client.getHomeGeoPoint(vehicle_name=vehicle_name)
            print(f"AirSim Home Position: {home_pos.latitude:.6f}, {home_pos.longitude:.6f}, {home_pos.altitude:.2f}")
            has_airsim_home = True
        except Exception as e:
            print(f"AirSim Home Position: ERROR - {e}")
            has_airsim_home = False
            
    except Exception as e:
        print(f"❌ Error getting AirSim GPS data: {e}")
        return False
    
    # Step 2: Check MAVLink connection and GPS
    print("\n2️⃣ MAVLink Connection and GPS")
    print("-" * 30)
    try:
        mavlink_conn = mavutil.mavlink_connection(f'udpin:172.28.240.1:{mavlink_port}', source_system=1)
        print("Connecting to MAVLink...")
        
        heartbeat = mavlink_conn.wait_heartbeat(timeout=10)
        if not heartbeat:
            print("❌ No MAVLink heartbeat received")
            return False
            
        target_system = heartbeat.get_srcSystem()
        target_component = heartbeat.get_srcComponent()
        print(f"✅ MAVLink connected: System {target_system}, Component {target_component}")
        
        # Request GPS data stream
        mavlink_conn.mav.request_data_stream_send(
            target_system, target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)
        
        # Check GPS via MAVLink
        gps_msg = None
        for _ in range(50):
            msg = mavlink_conn.recv_match(type='GPS_RAW_INT', blocking=False)
            if msg:
                gps_msg = msg
                break
            time.sleep(0.1)
        
        if gps_msg:
            print(f"MAVLink GPS Fix Type: {gps_msg.fix_type}")
            print(f"MAVLink GPS Coordinates: {gps_msg.lat/1e7:.6f}, {gps_msg.lon/1e7:.6f}, {gps_msg.alt/1000:.2f}")
            print(f"MAVLink Satellites: {gps_msg.satellites_visible}")
        else:
            print("❌ No GPS data via MAVLink")
            
    except Exception as e:
        print(f"❌ MAVLink connection error: {e}")
        return False
    
    # Step 3: Check current home position via MAVLink
    print("\n3️⃣ Current Home Position Check")
    print("-" * 30)
    
    # Request current home position
    mavlink_conn.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)
    
    home_msg = None
    for _ in range(30):
        msg = mavlink_conn.recv_match(type='HOME_POSITION', blocking=False)
        if msg:
            home_msg = msg
            break
        time.sleep(0.1)
    
    if home_msg:
        home_lat = home_msg.latitude / 1e7
        home_lon = home_msg.longitude / 1e7
        home_alt = home_msg.altitude / 1000
        print(f"MAVLink Home Position: {home_lat:.6f}, {home_lon:.6f}, {home_alt:.2f}")
        
        # Check if home is valid (not 0,0,0)
        if abs(home_lat) < 0.000001 and abs(home_lon) < 0.000001:
            print("❌ Home position is invalid (0,0,0)")
            has_valid_home = False
        else:
            print("✅ Home position appears valid")
            has_valid_home = True
    else:
        print("❌ No home position response from MAVLink")
        has_valid_home = False
    
    # Step 4: Test arming (this will fail if home is not set)
    print("\n4️⃣ Arm Test (Diagnostic)")
    print("-" * 30)
    
    try:
        arm_result = client.armDisarm(True, vehicle_name=vehicle_name)
        if arm_result:
            print("✅ Arming successful!")
            
            # Immediately disarm for safety
            client.armDisarm(False, vehicle_name=vehicle_name)
            print("✅ Disarmed for safety")
            
        else:
            print("❌ Arming failed - likely due to GPS home position issue")
            
            # Get more detailed status
            try:
                # Request extended system state
                mavlink_conn.mav.request_data_stream_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1)
                
                # Look for statustext messages that might explain the failure
                print("Checking for error messages...")
                for _ in range(20):
                    msg = mavlink_conn.recv_match(type='STATUSTEXT', blocking=False)
                    if msg:
                        print(f"  Status: {msg.text.decode('utf-8', errors='ignore')}")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Could not get detailed status: {e}")
    
    except Exception as e:
        print(f"❌ Arming test error: {e}")
    
    # Step 5: Attempt to fix the GPS home position
    print("\n5️⃣ GPS Home Position Fix Attempt")
    print("-" * 30)
    
    # Use coordinates from AirSim settings.json
    HOME_LAT = 47.641468
    HOME_LON = -122.140165
    HOME_ALT = 10.0
    
    print(f"Setting home position to: {HOME_LAT:.6f}, {HOME_LON:.6f}, {HOME_ALT:.1f}")
    
    # Method 1: MAV_CMD_DO_SET_HOME
    print("Using MAV_CMD_DO_SET_HOME...")
    mavlink_conn.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,          # confirmation
        0,          # param1: 0=use specified coordinates
        0,          # param2: yaw (0=default)
        HOME_LAT,   # param3: latitude
        HOME_LON,   # param4: longitude
        HOME_ALT,   # param5: altitude
        0, 0)       # param6, param7: unused
    
    # Wait for acknowledgment
    ack_received = False
    for _ in range(30):
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("✅ SET_HOME command accepted")
                ack_received = True
            else:
                print(f"❌ SET_HOME command rejected: {msg.result}")
            break
        time.sleep(0.1)
    
    if not ack_received:
        print("⏰ SET_HOME command timeout")
    
    # Method 2: SET_GPS_GLOBAL_ORIGIN
    print("Using SET_GPS_GLOBAL_ORIGIN...")
    mavlink_conn.mav.set_gps_global_origin_send(
        target_system,
        int(HOME_LAT * 1e7),
        int(HOME_LON * 1e7),
        int(HOME_ALT * 1000),
        int(time.time() * 1e6))
    
    # Method 3: SET_HOME_POSITION
    print("Using SET_HOME_POSITION...")
    mavlink_conn.mav.set_home_position_send(
        target_system,
        int(HOME_LAT * 1e7),
        int(HOME_LON * 1e7),
        int(HOME_ALT * 1000),
        0, 0, 0,
        [1, 0, 0, 0],
        0, 0, 0,
        int(time.time() * 1e6))
    
    # Give time for processing
    time.sleep(3)
    
    # Step 6: Verify the fix
    print("\n6️⃣ Verification")
    print("-" * 30)
    
    # Request home position again
    mavlink_conn.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)
    
    home_msg = None
    for _ in range(30):
        msg = mavlink_conn.recv_match(type='HOME_POSITION', blocking=False)
        if msg:
            home_msg = msg
            break
        time.sleep(0.1)
    
    if home_msg:
        new_home_lat = home_msg.latitude / 1e7
        new_home_lon = home_msg.longitude / 1e7
        new_home_alt = home_msg.altitude / 1000
        print(f"Updated Home Position: {new_home_lat:.6f}, {new_home_lon:.6f}, {new_home_alt:.2f}")
        
        # Check if it was set correctly
        lat_diff = abs(new_home_lat - HOME_LAT)
        lon_diff = abs(new_home_lon - HOME_LON)
        alt_diff = abs(new_home_alt - HOME_ALT)
        
        if lat_diff < 0.000001 and lon_diff < 0.000001 and alt_diff < 2.0:
            print("✅ Home position fixed successfully!")
            fixed = True
        else:
            print(f"❌ Home position still incorrect")
            print(f"   Difference: lat={lat_diff:.8f}, lon={lon_diff:.8f}, alt={alt_diff:.2f}")
            fixed = False
    else:
        print("❌ Could not verify updated home position")
        fixed = False
    
    # Final arm test
    if fixed:
        print("\n7️⃣ Final Arm Test")
        print("-" * 30)
        
        try:
            arm_result = client.armDisarm(True, vehicle_name=vehicle_name)
            if arm_result:
                print("✅ Arming successful after fix!")
                
                # Quick takeoff test
                print("Testing quick takeoff...")
                takeoff_future = client.takeoffAsync(vehicle_name=vehicle_name)
                takeoff_future.join()
                
                time.sleep(2)
                state = client.getMultirotorState(vehicle_name=vehicle_name)
                if state.kinematics_estimated.position.z_val < -0.5:
                    print("✅ Takeoff successful!")
                else:
                    print("⚠️  Takeoff may not have worked")
                
                # Land and disarm
                client.landAsync(vehicle_name=vehicle_name).join()
                client.armDisarm(False, vehicle_name=vehicle_name)
                print("✅ Landed and disarmed")
                
            else:
                print("❌ Arming still failed")
                
        except Exception as e:
            print(f"❌ Final test error: {e}")
    
    mavlink_conn.close()
    
    print(f"\n📋 Summary for {vehicle_name}")
    print("-" * 30)
    print(f"GPS Fix: {'✅' if gps_data.gnss.fix_type >= 3 else '❌'}")
    print(f"MAVLink Connection: ✅")
    print(f"Home Position Fixed: {'✅' if fixed else '❌'}")
    print(f"Ready for Operations: {'✅' if fixed else '❌'}")
    
    return fixed

def main():
    print("🏠 GPS Home Position Diagnostic Tool")
    print("=" * 60)
    print("This tool will diagnose and fix GPS home position issues")
    print("that prevent vehicle arming and takeoff.")
    
    # Test with first drone
    vehicle = "PX4_Drone1"
    port = 14541
    
    success = diagnose_vehicle_gps_home(vehicle, port)
    
    if success:
        print(f"\n🎉 SUCCESS! {vehicle} is now ready for operations.")
        print("You can now use standard AirSim commands like:")
        print("  - client.armDisarm(True)")
        print("  - client.takeoffAsync()")
        print("  - client.moveToPositionAsync()")
    else:
        print(f"\n❌ FAILED to fix GPS home position for {vehicle}")
        print("Please check:")
        print("  - Docker networking configuration")
        print("  - AirSim settings.json coordinates")
        print("  - PX4 parameter configuration")

if __name__ == "__main__":
    main() 