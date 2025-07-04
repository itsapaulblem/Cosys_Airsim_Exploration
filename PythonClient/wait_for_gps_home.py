#!/usr/bin/env python3

import setup_path
import cosysairsim as airsim
import time
import math
import sys

def wait_for_gps_home(client, vehicle_name, timeout_seconds=120):
    """
    Wait for GPS home position to be naturally established by PX4
    """
    print(f"⏳ Waiting for {vehicle_name} GPS home position to be established...")
    print(f"   Timeout: {timeout_seconds} seconds")
    
    start_time = time.time()
    last_status_time = start_time
    
    while (time.time() - start_time) < timeout_seconds:
        try:
            # Check GPS home position
            home_location = client.getHomeGeoPoint(vehicle_name=vehicle_name)
            
            # Check if home position is valid (not NaN or 0,0,0)
            if (not math.isnan(home_location.latitude) and 
                not math.isnan(home_location.longitude) and
                abs(home_location.latitude) > 0.000001 and 
                abs(home_location.longitude) > 0.000001):
                
                elapsed = time.time() - start_time
                print(f"✅ GPS home position established after {elapsed:.1f} seconds!")
                print(f"   📍 Home: {home_location.latitude:.6f}, {home_location.longitude:.6f}, {home_location.altitude:.2f}")
                return True
            
            # Print status every 10 seconds
            if time.time() - last_status_time >= 10:
                elapsed = time.time() - start_time
                print(f"   Still waiting... {elapsed:.0f}s (Home: {home_location.latitude}, {home_location.longitude})")
                last_status_time = time.time()
            
        except Exception as e:
            # Print status every 10 seconds even if there's an error
            if time.time() - last_status_time >= 10:
                elapsed = time.time() - start_time
                print(f"   Still waiting... {elapsed:.0f}s (Error: {e})")
                last_status_time = time.time()
        
        time.sleep(1)
    
    print(f"❌ GPS home position not established within {timeout_seconds} seconds")
    return False

def test_arm_disarm_cycle(client, vehicle_name):
    """
    Test complete arm, takeoff, land, disarm cycle
    """
    print(f"\n🔧 Testing complete flight cycle for {vehicle_name}...")
    
    try:
        # Enable API control
        print("  📡 Enabling API control...")
        client.enableApiControl(True, vehicle_name)
        
        # Test arming
        print("  🔐 Attempting to arm...")
        arm_result = client.armDisarm(True, vehicle_name)
        
        if arm_result:
            print("  ✅ Armed successfully!")
            
            # Check vehicle state
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            
            # Check armed status using multiple possible attribute names
            armed_status = "Unknown"
            for attr_name in ['armed', 'is_armed', 'arm_state']:
                if hasattr(state, attr_name):
                    armed_status = getattr(state, attr_name)
                    break
            
            print(f"  🔍 Armed status confirmed: {armed_status}")
            
            # Test takeoff
            print("  🚁 Attempting takeoff...")
            takeoff_future = client.takeoffAsync(vehicle_name=vehicle_name)
            takeoff_future.join()
            
            print("  ⏳ Waiting for takeoff to complete...")
            time.sleep(5)
            
            # Check position after takeoff
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            z_pos = state.kinematics_estimated.position.z_val
            print(f"  📊 Position after takeoff: z = {z_pos:.2f}")
            
            if z_pos < -1.0:  # Vehicle should be above ground (negative Z in NED)
                print("  ✅ Takeoff successful - vehicle is airborne!")
                success = True
            else:
                print("  ⚠️  Vehicle may not have taken off properly")
                success = False
            
            # Wait a bit in the air
            if success:
                print("  ⏳ Hovering for 3 seconds...")
                time.sleep(3)
            
            # Land the vehicle
            print("  🛬 Landing...")
            land_future = client.landAsync(vehicle_name=vehicle_name)
            land_future.join()
            
            print("  ⏳ Waiting for landing to complete...")
            time.sleep(3)
            
            # Disarm
            print("  🔐 Disarming...")
            disarm_result = client.armDisarm(False, vehicle_name=vehicle_name)
            if disarm_result:
                print("  ✅ Disarmed successfully")
            else:
                print("  ❌ Disarm failed")
            
            return success
            
        else:
            print("  ❌ Arming failed")
            return False
            
    except Exception as e:
        print(f"  ❌ Error during flight cycle: {e}")
        return False
        
    finally:
        # Always try to disable API control and disarm for safety
        try:
            client.armDisarm(False, vehicle_name)
            client.enableApiControl(False, vehicle_name)
        except:
            pass

def main():
    print("⏳ GPS Home Position Wait & Test Script")
    print("=" * 50)
    
    # Test configuration
    vehicles_to_test = ["PX4_Drone1", "PX4_Drone2", "PX4_Drone3"]  # Test first 3 drones
    timeout_per_vehicle = 60  # seconds to wait for each vehicle
    
    # Connect to AirSim
    print("\n📡 Connecting to AirSim...")
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=10)
        client.confirmConnection()
        print("✅ AirSim connected")
    except Exception as e:
        print(f"❌ AirSim connection failed: {e}")
        return 1
    
    successful_vehicles = []
    failed_vehicles = []
    
    for vehicle in vehicles_to_test:
        print(f"\n{'='*60}")
        print(f"🚁 Testing {vehicle}")
        print(f"{'='*60}")
        
        # Step 1: Wait for GPS home position
        if wait_for_gps_home(client, vehicle, timeout_per_vehicle):
            
            # Step 2: Test arm/disarm cycle
            if test_arm_disarm_cycle(client, vehicle):
                print(f"🎉 {vehicle} - COMPLETE SUCCESS!")
                successful_vehicles.append(vehicle)
            else:
                print(f"⚠️  {vehicle} - GPS home established but flight cycle failed")
                failed_vehicles.append(vehicle)
        else:
            print(f"❌ {vehicle} - GPS home position timeout")
            failed_vehicles.append(vehicle)
        
        # Brief pause between vehicles
        if vehicle != vehicles_to_test[-1]:
            print("\n⏳ Pausing 5 seconds before next vehicle...")
            time.sleep(5)
    
    # Final summary
    print(f"\n{'='*60}")
    print(f"📋 FINAL SUMMARY")
    print(f"{'='*60}")
    print(f"✅ Successful vehicles: {len(successful_vehicles)}")
    for vehicle in successful_vehicles:
        print(f"   - {vehicle}")
    
    print(f"\n❌ Failed vehicles: {len(failed_vehicles)}")
    for vehicle in failed_vehicles:
        print(f"   - {vehicle}")
    
    if len(successful_vehicles) > 0:
        print(f"\n🎉 SUCCESS! At least {len(successful_vehicles)} vehicles are working correctly.")
        print("The GPS home position issue has been resolved for these vehicles.")
        print("You can now use takeoff commands successfully on the working vehicles.")
        return 0
    else:
        print(f"\n❌ No vehicles succeeded. The GPS home position issue persists.")
        print("\n💡 Troubleshooting suggestions:")
        print("1. Check that PX4 containers are running properly:")
        print("   docker ps | grep px4")
        print("2. Check PX4 container logs for errors:")
        print("   docker logs <container_name>")
        print("3. Verify AirSim settings.json GPS configuration")
        print("4. Try restarting the containers:")
        print("   docker-compose down && docker-compose up -d")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code) 