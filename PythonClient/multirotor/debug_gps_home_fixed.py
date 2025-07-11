#!/usr/bin/env python3
"""
Fixed debug script for troubleshooting "waiting for valid GPS home" error
"""
import setup_path
import cosysairsim as airsim
import time
import math

def get_default_vehicle_name(client):
    """Try to determine the default vehicle name"""
    # First try empty string (default vehicle)
    try:
        state = client.getMultirotorState()
        return ""  # Empty string means default vehicle
    except:
        pass
    
    # Try common names
    common_names = ["PX4_Drone1", "PX4", "SimpleFlight", "Drone1"]
    for name in common_names:
        try:
            client.getMultirotorState(vehicle_name=name)
            return name
        except:
            continue
    
    return None

def main():
    # Connect to the AirSim simulator using Windows host IP
    client = airsim.MultirotorClient(ip="172.28.240.1")
    client.confirmConnection()
    
    print("Connected to AirSim")
    print("=" * 50)
    
    # Get default vehicle name
    vehicle_name = get_default_vehicle_name(client)
    
    if vehicle_name is None:
        print("\nERROR: No vehicles found!")
        return
    
    if vehicle_name == "":
        print(f"\nUsing default vehicle")
    else:
        print(f"\nUsing vehicle: '{vehicle_name}'")
    
    try:
        # Enable API control (but don't arm yet)
        client.enableApiControl(True, vehicle_name)
        print(f"API control enabled")
    except Exception as e:
        print(f"Warning: Could not enable API control: {e}")
    
    print("\nMonitoring GPS and Home Position Status...")
    print("Press Ctrl+C to stop monitoring\n")
    
    try:
        while True:
            try:
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
                if vehicle_name:
                    print(f"Vehicle: {vehicle_name}")
                else:
                    print("Vehicle: Default")
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
                
                # Home Position (fixed NaN detection)
                print("Home Position:")
                if not (math.isnan(home_point.latitude) or math.isnan(home_point.longitude)):
                    print(f"  Latitude:  {home_point.latitude:.8f}")
                    print(f"  Longitude: {home_point.longitude:.8f}")
                    print(f"  Altitude:  {home_point.altitude:.2f} m")
                    print("  Status:    SET ✓")
                else:
                    print(f"  Latitude:  {home_point.latitude}")
                    print(f"  Longitude: {home_point.longitude}")
                    print(f"  Altitude:  {home_point.altitude}")
                    print("  Status:    NOT SET ✗")
                print()
                
                # Vehicle State (fixed API calls)
                print("Vehicle State:")
                try:
                    # Try different ways to check armed status
                    if hasattr(state, 'armed'):
                        print(f"  Armed:     {state.armed}")
                    elif hasattr(state, 'is_armed'):
                        print(f"  Armed:     {state.is_armed}")
                    else:
                        print(f"  Armed:     Unknown (API mismatch)")
                    
                    print(f"  Position:  X={state.kinematics_estimated.position.x_val:.2f}, "
                          f"Y={state.kinematics_estimated.position.y_val:.2f}, "
                          f"Z={state.kinematics_estimated.position.z_val:.2f}")
                    
                    # Check if we can arm
                    try:
                        can_arm = client.isArmed(vehicle_name) == False  # Not armed = can potentially arm
                        print(f"  Can Arm:   {can_arm}")
                    except:
                        print(f"  Can Arm:   Unknown")
                        
                except Exception as e:
                    print(f"  Error getting vehicle state: {e}")
                
                print()
                
                # Recommendations
                print("Troubleshooting:")
                issues_found = False
                
                if not gps_data.is_valid:
                    print("  ⚠ GPS data is not valid. Check GPS sensor settings.")
                    issues_found = True
                    
                if gps_data.gnss.fix_type < 3:
                    print("  ⚠ GPS fix type is less than 3D fix. Waiting for better GPS lock.")
                    issues_found = True
                    
                if math.isnan(home_point.latitude) or math.isnan(home_point.longitude):
                    print("  ⚠ Home position not set (NaN values)!")
                    print("    - Check OriginGeopoint in settings.json")
                    print("    - Restart AirSim after settings changes")
                    print("    - Wait for PX4 to establish home position")
                    issues_found = True
                    
                if gps_data.gnss.eph > 5:
                    print("  ⚠ GPS horizontal accuracy is poor (EPH > 5)")
                    issues_found = True
                
                if not issues_found:
                    print("  ✓ GPS and Home position look good!")
                    print("  ✓ Try arming your drone now")
                
            except Exception as e:
                print(f"\nError reading vehicle data: {e}")
                print("Retrying in 5 seconds...")
                time.sleep(5)
                continue
            
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