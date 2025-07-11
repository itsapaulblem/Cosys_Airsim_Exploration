#!/usr/bin/env python3
"""
Test arming the drone after GPS and home position are established
"""
import setup_path
import cosysairsim as airsim
import time
import math

def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    print("🚁 PX4 Arming Test")
    print("=" * 50)
    
    # Get default vehicle
    vehicle_name = ""
    try:
        state = client.getMultirotorState(vehicle_name)
        print(f"Using default vehicle")
    except:
        vehicle_name = "PX4_Drone1"
        print(f"Using vehicle: {vehicle_name}")
    
    try:
        # Enable API control
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Check GPS status
        gps_data = client.getGpsData(vehicle_name=vehicle_name)
        home_point = client.getHomeGeoPoint(vehicle_name=vehicle_name)
        
        print(f"\n📊 Pre-arm Status Check:")
        print(f"GPS Valid: {gps_data.is_valid}")
        print(f"GPS Fix Type: {gps_data.gnss.fix_type}")
        print(f"GPS EPH: {gps_data.gnss.eph:.2f}")
        
        home_set = not (math.isnan(home_point.latitude) or math.isnan(home_point.longitude))
        print(f"Home Position Set: {home_set}")
        
        if home_set:
            print(f"Home Lat: {home_point.latitude:.8f}")
            print(f"Home Lon: {home_point.longitude:.8f}")
        
        # Check if already armed
        try:
            is_armed = client.isArmed(vehicle_name)
            print(f"Currently Armed: {is_armed}")
        except:
            print("Cannot check armed status")
            is_armed = False
        
        if not gps_data.is_valid:
            print("❌ Cannot arm: GPS not valid")
            return
            
        if gps_data.gnss.fix_type < 3:
            print("❌ Cannot arm: GPS fix type < 3")
            return
            
        if not home_set:
            print("❌ Cannot arm: Home position not set")
            print("💡 Try running force_home_position.py first")
            return
            
        if is_armed:
            print("ℹ️ Vehicle is already armed")
            
            # Test basic movement
            print("\n🔄 Testing basic movement...")
            client.moveToZAsync(-5, 2, vehicle_name=vehicle_name).join()
            time.sleep(2)
            
            print("✅ Movement test complete")
            
            # Land and disarm
            print("\n🛬 Landing...")
            client.landAsync(vehicle_name=vehicle_name).join()
            
            print("🔓 Disarming...")
            client.disarmAsync(vehicle_name=vehicle_name).join()
            
        else:
            print("\n🔐 Attempting to arm vehicle...")
            
            # Try to arm
            try:
                result = client.armDisarm(True, vehicle_name)
                if result:
                    print("✅ Vehicle armed successfully!")
                    
                    # Test takeoff
                    print("\n🚁 Taking off...")
                    client.takeoffAsync(vehicle_name=vehicle_name).join()
                    
                    time.sleep(3)
                    
                    print("✅ Takeoff successful!")
                    
                    # Land and disarm
                    print("\n🛬 Landing...")
                    client.landAsync(vehicle_name=vehicle_name).join()
                    
                    print("🔓 Disarming...")
                    client.disarmAsync(vehicle_name=vehicle_name).join()
                    
                    print("✅ Full flight test completed successfully!")
                    
                else:
                    print("❌ Failed to arm vehicle")
                    
            except Exception as arm_error:
                print(f"❌ Arming failed: {arm_error}")
                
                # Check for specific error messages
                error_str = str(arm_error).lower()
                if "home" in error_str:
                    print("💡 Home position issue detected")
                elif "gps" in error_str:
                    print("💡 GPS issue detected")
                elif "prearm" in error_str:
                    print("💡 Pre-arm check failed")
                
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            client.enableApiControl(False, vehicle_name)
        except:
            pass

if __name__ == "__main__":
    main()