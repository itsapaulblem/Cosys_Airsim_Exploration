#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import socket
import sys
import os
import json
from pymavlink import mavutil

def main():
    print("🔍 MAVLink GPS Home Position Debug Tool")
    print("=" * 60)
    
    # Configuration
    AIRSIM_IP = "127.0.0.1"  # Container IP for AirSim
    AIRSIM_PORT = 41451
    
    # GPS coordinates from AirSim settings.json
    HOME_LAT = 47.641468
    HOME_LON = -122.140165
    HOME_ALT = 10.0
    
    vehicles = ["PX4_Drone1", "PX4_Drone2", "PX4_Drone3", "PX4_Drone4", "PX4_Drone5"]
    mavlink_ports = [14541, 14542, 14543, 14544, 14545]
    
    try:
        # 1. Connect to AirSim
        print(f"\n📡 Connecting to AirSim at {AIRSIM_IP}:{AIRSIM_PORT}")
        client = airsim.MultirotorClient(ip=AIRSIM_IP, port=AIRSIM_PORT, timeout_value=10)
        client.confirmConnection()
        print("✅ AirSim connection successful")
        
        # 2. Check vehicle states
        print(f"\n🚁 Checking vehicle states...")
        for vehicle in vehicles:
            try:
                state = client.getMultirotorState(vehicle_name=vehicle)
                gps_data = client.getGpsData(vehicle_name=vehicle)
                print(f"  {vehicle}:")
                print(f"    - Position: ({state.kinematics_estimated.position.x_val:.2f}, {state.kinematics_estimated.position.y_val:.2f}, {state.kinematics_estimated.position.z_val:.2f})")
                
                # Fix GPS data access for cosysairsim
                try:
                    if hasattr(gps_data, 'gnss'):
                        # Check what attributes are available
                        gnss = gps_data.gnss
                        fix_type = getattr(gnss, 'fix_type', 'Unknown')
                        
                        # Try different possible attribute names for satellites
                        satellites = None
                        for attr_name in ['satellites_visible', 'num_satellites', 'satellites']:
                            if hasattr(gnss, attr_name):
                                satellites = getattr(gnss, attr_name)
                                break
                        
                        if satellites is None:
                            satellites = "Unknown"
                        
                        # Try to get GPS coordinates
                        if hasattr(gnss, 'geo_point'):
                            geo_point = gnss.geo_point
                            lat = getattr(geo_point, 'latitude', 0.0)
                            lon = getattr(geo_point, 'longitude', 0.0) 
                            alt = getattr(geo_point, 'altitude', 0.0)
                        else:
                            lat = lon = alt = 0.0
                            
                        print(f"    - GPS Fix: {fix_type}, Satellites: {satellites}")
                        print(f"    - GPS Coords: ({lat:.6f}, {lon:.6f}, {alt:.2f})")
                    else:
                        print(f"    - GPS Data: No GNSS data available")
                        print(f"    - Available GPS attributes: {dir(gps_data)}")
                        
                except Exception as gps_error:
                    print(f"    - GPS Error: {gps_error}")
                    print(f"    - GPS Data type: {type(gps_data)}")
                    print(f"    - GPS Data attributes: {dir(gps_data)}")
                    
            except Exception as e:
                print(f"    - ERROR: {e}")
        
        # 2.5. Check if PX4 containers are running
        print(f"\n🐳 Checking PX4 container connectivity...")
        for i, port in enumerate(mavlink_ports):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2)
                result = sock.connect_ex(('127.0.0.1', port))
                sock.close()
                
                if result == 0:
                    print(f"  ✅ PX4_Drone{i+1} port {port} is reachable")
                else:
                    print(f"  ❌ PX4_Drone{i+1} port {port} is NOT reachable")
            except Exception as e:
                print(f"  ❌ PX4_Drone{i+1} port {port} error: {e}")
        
        # 3. Test MAVLink connections and GPS home setting
        print(f"\n🔗 Testing MAVLink connections and GPS home setting...")
        
        for i, (vehicle, port) in enumerate(zip(vehicles, mavlink_ports)):
            print(f"\n--- {vehicle} (MAVLink port {port}) ---")
            
            try:
                # Connect to MAVLink with longer timeout
                print("  🔄 Attempting MAVLink connection...")
                mavlink_conn = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=1)
                
                # Wait for heartbeat with longer timeout
                print("  🔄 Waiting for heartbeat...")
                heartbeat = mavlink_conn.wait_heartbeat(timeout=10)
                if heartbeat:
                    print(f"  ✅ Heartbeat received from system {heartbeat.get_srcSystem()}, component {heartbeat.get_srcComponent()}")
                else:
                    print("  ❌ No heartbeat received - PX4 container may not be running")
                    print("     Please check:")
                    print("     - Are PX4 containers started?")
                    print("     - Is the MAVLink port mapping correct?")
                    print("     - Is PX4 configured to output MAVLink on this port?")
                    continue
                
                # Get system and component IDs
                target_system = heartbeat.get_srcSystem()
                target_component = heartbeat.get_srcComponent()
                
                # Check current GPS status
                print("  📡 Checking GPS status...")
                mavlink_conn.mav.request_data_stream_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)
                
                # Wait for GPS_RAW_INT message
                gps_msg = None
                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = mavlink_conn.recv_match(type='GPS_RAW_INT', blocking=False)
                    if msg:
                        gps_msg = msg
                        break
                    time.sleep(0.1)
                
                if gps_msg:
                    print(f"    GPS Fix Type: {gps_msg.fix_type}")
                    print(f"    GPS Coordinates: {gps_msg.lat/1e7:.6f}, {gps_msg.lon/1e7:.6f}, {gps_msg.alt/1000:.2f}m")
                    print(f"    Satellites: {gps_msg.satellites_visible}")
                else:
                    print("    ❌ No GPS data received via MAVLink")
                
                # Check current home position
                print("  🏠 Requesting current home position...")
                mavlink_conn.mav.command_long_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0, 0, 0, 0, 0, 0, 0, 0)
                
                # Wait for HOME_POSITION message
                home_msg = None
                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = mavlink_conn.recv_match(type='HOME_POSITION', blocking=False)
                    if msg:
                        home_msg = msg
                        break
                    time.sleep(0.1)
                
                if home_msg:
                    print(f"    Current Home: {home_msg.latitude/1e7:.6f}, {home_msg.longitude/1e7:.6f}, {home_msg.altitude/1000:.2f}m")
                else:
                    print("    ❌ No home position received")
                
                # Method 1: Set home position using MAV_CMD_DO_SET_HOME
                print("  🏠 Setting home position using MAV_CMD_DO_SET_HOME...")
                mavlink_conn.mav.command_long_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                    0,  # confirmation
                    0,  # param1: Use current location (0=use specified location)
                    0,  # param2: Yaw angle (0=default)
                    HOME_LAT,  # param3: Latitude
                    HOME_LON,  # param4: Longitude
                    HOME_ALT,  # param5: Altitude
                    0, 0)  # param6, param7: unused
                
                # Wait for acknowledgment
                ack_msg = None
                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=False)
                    if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                        ack_msg = msg
                        break
                    time.sleep(0.1)
                
                if ack_msg:
                    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("    ✅ SET_HOME command accepted")
                    else:
                        print(f"    ❌ SET_HOME command failed: {ack_msg.result}")
                else:
                    print("    ⏰ SET_HOME command timeout")
                
                # Method 2: Set GPS global origin
                print("  🌍 Setting GPS global origin...")
                mavlink_conn.mav.set_gps_global_origin_send(
                    target_system,
                    int(HOME_LAT * 1e7),  # latitude in degE7
                    int(HOME_LON * 1e7),  # longitude in degE7
                    int(HOME_ALT * 1000), # altitude in mm
                    int(time.time() * 1e6))  # timestamp in microseconds
                
                # Method 3: Use SET_HOME_POSITION message
                print("  🏠 Setting home position using SET_HOME_POSITION...")
                mavlink_conn.mav.set_home_position_send(
                    target_system,
                    int(HOME_LAT * 1e7),  # latitude in degE7
                    int(HOME_LON * 1e7),  # longitude in degE7
                    int(HOME_ALT * 1000), # altitude in mm
                    0, 0, 0,  # x, y, z local positions
                    [1, 0, 0, 0],  # quaternion (w, x, y, z)
                    0, 0, 0,  # approach vector
                    int(time.time() * 1e6))  # timestamp
                
                # Give some time for processing
                time.sleep(2)
                
                # Verify home position was set
                print("  ✅ Requesting updated home position...")
                mavlink_conn.mav.command_long_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0, 0, 0, 0, 0, 0, 0, 0)
                
                # Wait for updated HOME_POSITION message
                updated_home_msg = None
                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = mavlink_conn.recv_match(type='HOME_POSITION', blocking=False)
                    if msg:
                        updated_home_msg = msg
                        break
                    time.sleep(0.1)
                
                if updated_home_msg:
                    home_lat = updated_home_msg.latitude/1e7
                    home_lon = updated_home_msg.longitude/1e7
                    home_alt = updated_home_msg.altitude/1000
                    print(f"    Updated Home: {home_lat:.6f}, {home_lon:.6f}, {home_alt:.2f}m")
                    
                    # Check if home was set correctly
                    lat_diff = abs(home_lat - HOME_LAT)
                    lon_diff = abs(home_lon - HOME_LON)
                    alt_diff = abs(home_alt - HOME_ALT)
                    
                    if lat_diff < 0.000001 and lon_diff < 0.000001 and alt_diff < 1.0:
                        print("    ✅ Home position set correctly!")
                    else:
                        print(f"    ❌ Home position mismatch! Expected: {HOME_LAT:.6f}, {HOME_LON:.6f}, {HOME_ALT:.2f}")
                        print(f"        Difference: lat={lat_diff:.8f}, lon={lon_diff:.8f}, alt={alt_diff:.2f}")
                else:
                    print("    ❌ No updated home position received")
                
                # Now test arming via AirSim
                print("  🔐 Testing arm/disarm via AirSim...")
                try:
                    # Try to arm
                    result = client.armDisarm(True, vehicle_name=vehicle)
                    if result:
                        print("    ✅ Armed successfully via AirSim")
                        time.sleep(1)
                        
                        # Try takeoff
                        print("  🚁 Testing takeoff...")
                        takeoff_result = client.takeoffAsync(vehicle_name=vehicle)
                        takeoff_result.join()
                        print("    ✅ Takeoff initiated successfully")
                        
                        time.sleep(2)
                        
                        # Check position after takeoff
                        state = client.getMultirotorState(vehicle_name=vehicle)
                        print(f"    Position after takeoff: z = {state.kinematics_estimated.position.z_val:.2f}")
                        
                        # Land and disarm
                        print("  🛬 Landing...")
                        client.landAsync(vehicle_name=vehicle).join()
                        
                        print("  🔐 Disarming...")
                        disarm_result = client.armDisarm(False, vehicle_name=vehicle)
                        if disarm_result:
                            print("    ✅ Disarmed successfully")
                        else:
                            print("    ❌ Disarm failed")
                    else:
                        print("    ❌ Arm failed via AirSim")
                        
                except Exception as e:
                    print(f"    ❌ Arm/takeoff error: {e}")
                
                mavlink_conn.close()
                
            except Exception as e:
                print(f"  ❌ MAVLink connection error: {e}")
            
            print()
        
        # 4. Final status check
        print("\n📋 Final Status Summary")
        print("=" * 40)
        
        for vehicle in vehicles:
            try:
                state = client.getMultirotorState(vehicle_name=vehicle)
                gps_data = client.getGpsData(vehicle_name=vehicle)
                
                print(f"{vehicle}:")
                
                # Fix armed status access for cosysairsim
                armed_status = "Unknown"
                for attr_name in ['armed', 'is_armed', 'arm_state']:
                    if hasattr(state, attr_name):
                        armed_status = getattr(state, attr_name)
                        break
                
                print(f"  - Armed: {armed_status}")
                print(f"  - Position: z = {state.kinematics_estimated.position.z_val:.2f}")
                
                # GPS data
                if hasattr(gps_data, 'gnss') and hasattr(gps_data.gnss, 'fix_type'):
                    print(f"  - GPS Fix: {gps_data.gnss.fix_type}")
                    if hasattr(gps_data.gnss, 'geo_point'):
                        geo = gps_data.gnss.geo_point
                        print(f"  - GPS Coords: ({geo.latitude:.6f}, {geo.longitude:.6f})")
                else:
                    print(f"  - GPS: No data available")
                
                # Try to get home position via AirSim API
                try:
                    home_pos = client.getHomeGeoPoint(vehicle_name=vehicle)
                    print(f"  - Home (AirSim): ({home_pos.latitude:.6f}, {home_pos.longitude:.6f}, {home_pos.altitude:.2f})")
                except Exception as e:
                    print(f"  - Home (AirSim): Error - {e}")
                
                print()
                
            except Exception as e:
                print(f"{vehicle}: ERROR - {e}")
        
        # 5. Troubleshooting guidance
        print("\n🔧 Troubleshooting Guidance")
        print("=" * 40)
        print("If no MAVLink heartbeats were received:")
        print("1. Check if PX4 Docker containers are running:")
        print("   docker ps | grep px4")
        print("2. Verify port mapping in docker-compose.yml")
        print("3. Check PX4 startup logs:")
        print("   docker logs <container_name>")
        print("4. Ensure PX4 is configured for MAVLink output")
        print("5. Try connecting from host system instead of container")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 