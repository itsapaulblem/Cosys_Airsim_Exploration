import sys
import os
import time
import setup_path 
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def main():
    print("⚡ Quick GPS Home Location Fix")
    print("=============================")
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected to AirSim")
    
    # Test just PX4_Drone1 first
    drone = "PX4_Drone1"
    print(f"\n🔧 Fixing GPS home location for {drone}...")
    
    try:
        # Enable API control
        client.enableApiControl(True, drone)
        print(f"✅ API control enabled")
        
        # Wait a bit for GPS to stabilize
        print("⏳ Waiting 5 seconds for GPS to stabilize...")
        time.sleep(5)
        
        # Try to arm with retries
        max_retries = 10
        for attempt in range(max_retries):
            try:
                print(f"🔧 Attempt {attempt + 1}/{max_retries} to arm...")
                client.armDisarm(True, drone)
                print(f"✅ SUCCESS! {drone} armed successfully!")
                client.armDisarm(False, drone)
                print(f"✅ {drone} disarmed - GPS home location is now established")
                
                # Test takeoff
                print("🚁 Testing takeoff...")
                client.armDisarm(True, drone)
                client.takeoffAsync(vehicle_name=drone).join()
                print("✅ Takeoff successful!")
                
                # Land and disarm
                client.landAsync(vehicle_name=drone).join()
                client.armDisarm(False, drone)
                print("✅ Landing successful!")
                
                print("\n🎯 GPS home location fix COMPLETE!")
                print("You can now run your multi-agent script.")
                return
                
            except Exception as e:
                if "GPS home location" in str(e):
                    print(f"⏳ Still no GPS home, waiting 3 seconds... (attempt {attempt + 1})")
                    time.sleep(3)
                else:
                    print(f"❌ Different error: {e}")
                    break
        
        print(f"❌ Failed to establish GPS home location after {max_retries} attempts")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main() 