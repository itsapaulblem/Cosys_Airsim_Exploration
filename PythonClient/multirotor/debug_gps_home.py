#!/usr/bin/env python3
"""
Debug script for troubleshooting "waiting for valid GPS home" error in AirSim PX4
This script monitors GPS messages and home position status to help diagnose issues.
"""
import setup_path
import cosysairsim as airsim
import time
import sys

def main():
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    print("Connected to AirSim")
    print("=" * 50)
    
    # Get vehicle state
    vehicle_name = "PX4"  # Change this if your vehicle has a different name
    
    try:
        # Enable API control (but don't arm yet)
        client.enableApiControl(True, vehicle_name)
        print(f"API control enabled for {vehicle_name}")
    except Exception as e:
        print(f"Error enabling API control: {e}")
    
    # Monitor GPS and home position
    print("\nMonitoring GPS and Home Position Status...")
    print("Press Ctrl+C to stop monitoring\n")
    
    try:
        while True:
            # Get GPS data
            gps_data = client.getGpsData(vehicle_name=vehicle_name)
            
            # Get home geo point
            home_point = client.getHomeGeoPoint(vehicle_name=vehicle_name)
            
            # Get vehicle state
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            
            # Clear screen for better readability
            print("\033[2J\033[H")  # Clear screen and move cursor to top
            
            print("GPS STATUS MONITOR")
            print("=" * 50)
            print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            print()
            
            # GPS Information
            print("GPS Data:")
            print(f"  Latitude:  {gps_data.gnss.geo_point.latitude:.8f}")
            print(f"  Longitude: {gps_data.gnss.geo_point.longitude:.8f}")
            print(f"  Altitude:  {gps_data.gnss.geo_point.altitude:.2f} m")
            print(f"  Fix Type:  {gps_data.gnss.fix_type}")
            print(f"  Valid:     {gps_data.is_valid}")
            print(f"  EPH:       {gps_data.gnss.eph:.2f}")
            print(f"  EPV:       {gps_data.gnss.epv:.2f}")
            print()
            
            # Home Position
            print("Home Position:")
            if home_point.latitude != 0 or home_point.longitude != 0:
                print(f"  Latitude:  {home_point.latitude:.8f}")
                print(f"  Longitude: {home_point.longitude:.8f}")
                print(f"  Altitude:  {home_point.altitude:.2f} m")
                print("  Status:    SET ✓")
            else:
                print("  Status:    NOT SET ✗")
            print()
            
            # Vehicle State
            print("Vehicle State:")
            print(f"  Armed:     {state.is_armable}")
            print(f"  Position:  X={state.kinematics_estimated.position.x_val:.2f}, "
                  f"Y={state.kinematics_estimated.position.y_val:.2f}, "
                  f"Z={state.kinematics_estimated.position.z_val:.2f}")
            print()
            
            # Recommendations
            print("Troubleshooting:")
            if not gps_data.is_valid:
                print("  ⚠ GPS data is not valid. Check GPS sensor settings.")
            if gps_data.gnss.fix_type < 3:
                print("  ⚠ GPS fix type is less than 3D fix. Waiting for better GPS lock.")
            if home_point.latitude == 0 and home_point.longitude == 0:
                print("  ⚠ Home position not set. Check OriginGeopoint in settings.json")
            if gps_data.gnss.eph > 5:
                print("  ⚠ GPS horizontal accuracy is poor (EPH > 5)")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Disable API control
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

if __name__ == "__main__":
    main()