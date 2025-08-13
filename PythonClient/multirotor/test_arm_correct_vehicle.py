#!/usr/bin/env python3
"""
Test arming with the correct vehicle name from settings.json
"""
import sys
import setup_path
import cosysairsim as airsim
import time
import math

def main():
    print("🎯 Correct Vehicle Name Arming Test")
    print("=" * 50)
    
    client = airsim.MultirotorClient(ip="172.28.240.1")
    client.confirmConnection()
    
    # Use the correct vehicle name from settings.json
    vehicle_name = "PX4_Drone1"
    
    try:
        print(f"✅ Connected to AirSim")
        print(f"🚁 Using vehicle: {vehicle_name}")
        
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Check GPS and home position with correct vehicle name
        print(f"\n📍 Checking GPS and Home Position...")
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        home_point = client.getHomeGeoPoint(vehicle_name=vehicle_name)
        
        print(f"GPS Valid: {gps_data.is_valid}")
        print(f"GPS Fix: {gps_data.gnss.fix_type}")
        print(f"GPS Lat: {gps_data.gnss.geo_point.latitude:.8f}")
        print(f"GPS Lon: {gps_data.gnss.geo_point.longitude:.8f}")
        
        home_set = not (math.isnan(home_point.latitude) or math.isnan(home_point.longitude))
        print(f"Home Set: {home_set}")
        
        if home_set:
            print(f"Home Lat: {home_point.latitude:.8f}")
            print(f"Home Lon: {home_point.longitude:.8f}")
        
        # Check current armed status
        try:
            is_armed = client.isArmed(vehicle_name)
            print(f"Currently armed: {is_armed}")
        except:
            print("Cannot check armed status")
            is_armed = False
        
        if not is_armed:
            print(f"\n🔐 Attempting to arm {vehicle_name}...")
            
            try:
                # Try to arm with correct vehicle name
                result = client.armDisarm(True, vehicle_name)
                print(f"Arm command result: {result}")
                
                # Wait a moment and check status
                time.sleep(2)
                
                try:
                    is_armed = client.isArmed(vehicle_name)
                    if is_armed:
                        print("✅ SUCCESS! Vehicle is now ARMED!")
                        
                        print(f"\n🚁 Testing takeoff...")
                        client.takeoffAsync(vehicle_name=vehicle_name).join()
                        
                        print("✅ TAKEOFF SUCCESSFUL!")
                        print("🎉 PROBLEM COMPLETELY SOLVED!")
                        print("   - GPS home position working")
                        print("   - Vehicle naming corrected")
                        print("   - Arming and takeoff successful")
                        
                        time.sleep(3)
                        
                        print(f"\n🛬 Landing...")
                        client.landAsync(vehicle_name=vehicle_name).join()
                        
                        print(f"🔓 Disarming...")
                        client.disarmAsync(vehicle_name=vehicle_name).join()
                        
                        print("✅ Full flight test completed successfully!")
                        return True
                        
                    else:
                        print("❌ Vehicle still not armed")
                        
                except Exception as status_error:
                    print(f"Status check error: {status_error}")
                    
            except Exception as arm_error:
                print(f"❌ Arming failed: {arm_error}")
                
                # Parse the error message
                error_str = str(arm_error).lower()
                if "home" in error_str:
                    print("💡 Still a home position issue")
                    if not home_set:
                        print("   The home position is indeed not set for this vehicle")
                    else:
                        print("   Home position seems set but PX4 disagrees")
                elif "gps" in error_str:
                    print("💡 GPS issue detected")
                elif "prearm" in error_str or "pre-arm" in error_str:
                    print("💡 Pre-arm check failed")
                else:
                    print("💡 Unknown arming issue")
                    
                print(f"\nFull error: {arm_error}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    finally:
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass
    
    return False

if __name__ == "__main__":
    success = main()
    if success:
        print(f"\n🏆 MISSION ACCOMPLISHED!")
        print(f"   Your 'waiting for valid GPS home' error is now COMPLETELY FIXED!")
        print(f"   The issue was vehicle naming - use 'PX4_Drone1' instead of default")
    else:
        print(f"\n🔍 Key Insights:")
        print(f"   - Vehicle name matters: use 'PX4_Drone1' from settings.json")
        print(f"   - Check if home position is set for the correct vehicle")