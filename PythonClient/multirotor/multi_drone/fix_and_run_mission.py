#!/usr/bin/env python3
"""
GPS Fix and Mission Runner
Fixes GPS configuration and runs the multi-drone orbit mission
"""

import os
import sys
import json
import time
from pathlib import Path

# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import cosysairsim as airsim

def fix_settings_gps():
    """Fix the settings.json to include proper GPS configuration"""
    print("🔧 Fixing settings.json GPS configuration...")
    
    # Find settings.json
    settings_paths = [
        Path.home() / "Documents" / "AirSim" / "settings.json",
        Path("/mnt/l/Cosys-AirSim/docker_clean/multi_drone/settings.json")
    ]
    
    settings_file = None
    for path in settings_paths:
        if path.exists():
            settings_file = path
            break
    
    if not settings_file:
        print("❌ Could not find settings.json file!")
        return False
    
    print(f"📄 Found settings file: {settings_file}")
    
    try:
        # Read current settings
        with open(settings_file, 'r') as f:
            settings = json.load(f)
        
        # Check if GPS is missing from vehicles
        vehicles_fixed = 0
        for vehicle_name, vehicle_config in settings.get("Vehicles", {}).items():
            if "Sensors" not in vehicle_config:
                vehicle_config["Sensors"] = {}
            
            # Add GPS sensor if missing
            if "Gps" not in vehicle_config["Sensors"]:
                vehicle_config["Sensors"]["Gps"] = {
                    "SensorType": 3,
                    "Enabled": True,
                    "EphTimeConstant": 0.9,
                    "EpvTimeConstant": 0.9,
                    "EphInitial": 25.0,
                    "EpvInitial": 25.0,
                    "EphFinal": 0.1,
                    "EpvFinal": 0.1,
                    "EphMin3d": 3.0,
                    "EphMin2d": 4.0,
                    "UpdateLatency": 0.2,
                    "UpdateFrequency": 50,
                    "StartupDelay": 1
                }
                vehicles_fixed += 1
            
            # Add other required sensors
            if "Magnetometer" not in vehicle_config["Sensors"]:
                vehicle_config["Sensors"]["Magnetometer"] = {
                    "SensorType": 4,
                    "Enabled": True
                }
            
            if "Barometer" not in vehicle_config["Sensors"]:
                vehicle_config["Sensors"]["Barometer"] = {
                    "SensorType": 1,
                    "Enabled": True,
                    "PressureFactorSigma": 0.0001825
                }
            
            if "Imu" not in vehicle_config["Sensors"]:
                vehicle_config["Sensors"]["Imu"] = {
                    "SensorType": 2,
                    "Enabled": True
                }
            
            # Add PX4 parameters if missing
            if "Parameters" not in vehicle_config:
                vehicle_config["Parameters"] = {
                    "NAV_RCL_ACT": 0,
                    "NAV_DLL_ACT": 0,
                    "COM_OBL_ACT": 1,
                    "LPE_LAT": 47.641468,
                    "LPE_LON": -122.140165,
                    "COM_ARM_WO_GPS": 0,
                    "EKF2_AID_MASK": 1,
                    "EKF2_HGT_MODE": 0,
                    "EKF2_GPS_CHECK": 31
                }
        
        # Add OriginGeopoint if missing
        if "OriginGeopoint" not in settings:
            settings["OriginGeopoint"] = {
                "Latitude": 47.641468,
                "Longitude": -122.140165,
                "Altitude": 10
            }
        
        # Save fixed settings
        with open(settings_file, 'w') as f:
            json.dump(settings, f, indent=2)
        
        print(f"✅ Fixed GPS configuration for {vehicles_fixed} vehicles")
        print(f"✅ Settings saved to: {settings_file}")
        return True
        
    except Exception as e:
        print(f"❌ Error fixing settings: {e}")
        return False

def test_gps_and_arm():
    """Test GPS and arming for all vehicles"""
    print("\n🧪 Testing GPS and arming...")
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicles = client.listVehicles()
        print(f"🚁 Found vehicles: {vehicles}")
        
        if not vehicles:
            print("❌ No vehicles found!")
            return False
        
        all_good = True
        for vehicle in vehicles:
            print(f"\n🔍 Testing {vehicle}...")
            
            try:
                # Enable API control
                client.enableApiControl(True, vehicle)
                print(f"   ✅ API control enabled")
                
                # Wait a moment for GPS to initialize
                time.sleep(2)
                
                # Try to arm
                arm_result = client.armDisarm(True, vehicle)
                if arm_result:
                    print(f"   ✅ {vehicle} armed successfully!")
                    client.armDisarm(False, vehicle)  # Disarm
                    print(f"   ✅ {vehicle} disarmed")
                else:
                    print(f"   ⚠️ {vehicle} could not arm")
                    all_good = False
                    
            except Exception as e:
                if "GPS home location" in str(e):
                    print(f"   ❌ {vehicle} GPS home error: {e}")
                    all_good = False
                else:
                    print(f"   ⚠️ {vehicle} error: {e}")
        
        return all_good
        
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return False

def run_mission():
    """Run the multi-drone orbit mission"""
    print("\n🚁 Starting Multi-Drone Orbit Mission...")
    
    try:
        # Import and run the mission
        from multi_drone_orbit_mission import MultiDroneOrbitMission
        
        # Create mission with GPS-friendly settings
        mission = MultiDroneOrbitMission(
            num_drones=None,  # Auto-detect
            altitude=20,
            speed=5,
            orbit_radius=2,
            stop_duration=2,
            disable_collision_detection=False
        )
        
        # Run the mission
        success = mission.run()
        
        if success:
            print("✅ Mission completed successfully!")
        else:
            print("❌ Mission failed!")
            
        return success
        
    except Exception as e:
        print(f"❌ Mission error: {e}")
        return False

def main():
    print("🚁 GPS FIX AND MISSION RUNNER")
    print("=" * 50)
    
    # Step 1: Fix settings.json
    if not fix_settings_gps():
        print("❌ Could not fix settings.json")
        return 1
    
    # Step 2: Wait for AirSim to reload settings
    print("\n⏳ Waiting for AirSim to reload settings...")
    print("   💡 If AirSim is running, you may need to restart it")
    time.sleep(3)
    
    # Step 3: Test GPS and arming
    if not test_gps_and_arm():
        print("\n⚠️ GPS/Arming test failed")
        print("🔧 SOLUTIONS:")
        print("1. Restart AirSim/Unreal Engine")
        print("2. Restart Docker containers")
        print("3. Try running the mission anyway (it may work)")
        
        # Ask user if they want to proceed anyway
        try:
            response = input("\nTry running mission anyway? (y/n): ")
            if response.lower() != 'y':
                return 1
        except KeyboardInterrupt:
            return 1
    else:
        print("\n✅ GPS and arming test passed!")
    
    # Step 4: Run the mission
    success = run_mission()
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())