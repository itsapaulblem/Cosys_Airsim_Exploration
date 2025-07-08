#!/usr/bin/env python3
"""
Complete Settings Fix
Fixes the vehicle count mismatch and GPS configuration
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

def create_complete_5_drone_settings():
    """Create complete settings.json with all 5 PX4 drones"""
    print("🔧 CREATING COMPLETE 5-DRONE SETTINGS.JSON")
    print("=" * 50)
    
    settings_file = Path.home() / "Documents" / "AirSim" / "settings.json"
    print(f"📄 Updating: {settings_file}")
    
    # Complete settings for 5 drones
    settings = {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
        "SettingsVersion": 2.0,
        "SimMode": "Multirotor",
        "ClockType": "SteppableClock",
        "ApiServerEndpoint": "0.0.0.0:41451",
        "OriginGeopoint": {
            "Latitude": 47.641468,
            "Longitude": -122.140165,
            "Altitude": 10
        },
        "Vehicles": {},
        "PawnPaths": {
            "DefaultQuadrotor": {
                "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
            }
        },
        "SubWindows": []
    }
    
    # Create 5 identical drone configurations
    for i in range(1, 6):
        drone_name = f"PX4_Drone{i}"
        
        # Position drones in a line with 5m spacing
        x_pos = (i - 1) * 5
        
        # TCP ports: 4561, 4562, 4563, 4564, 4565
        tcp_port = 4560 + i
        
        # Control ports
        control_local = 14540 + i
        control_remote = 14580 + i
        
        drone_config = {
            "VehicleType": "PX4Multirotor",
            "X": x_pos,
            "Y": 0,
            "Z": -2,
            "Pitch": 0.0,
            "Roll": 0.0,
            "Yaw": 0.0,
            "UseSerial": False,
            "UseTcp": True,
            "TcpPort": tcp_port,
            "ControlPortLocal": control_local,
            "ControlPortRemote": control_remote,
            "LocalHostIp": "127.0.0.1",
            "LockStep": True,
            "Sensors": {
                "Gps": {
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
                },
                "Magnetometer": {
                    "SensorType": 4,
                    "Enabled": True
                },
                "Barometer": {
                    "SensorType": 1,
                    "Enabled": True,
                    "PressureFactorSigma": 0.0001825
                },
                "Imu": {
                    "SensorType": 2,
                    "Enabled": True
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165,
                "COM_ARM_WO_GPS": 0,
                "EKF2_AID_MASK": 1,
                "EKF2_HGT_MODE": 0,
                "EKF2_GPS_CHECK": 31
            },
            "Cameras": {
                "front_rgb": {
                    "CaptureSettings": [
                        {
                            "ImageType": 0,
                            "Width": 1920,
                            "Height": 1080,
                            "FOV_Degrees": 90.0
                        }
                    ],
                    "X": 0.5,
                    "Y": 0.0,
                    "Z": 0.1,
                    "Pitch": 0.0,
                    "Roll": 0.0,
                    "Yaw": 0.0
                },
                "segmentation_cam": {
                    "CaptureSettings": [
                        {
                            "ImageType": 5,
                            "Width": 640,
                            "Height": 480,
                            "FOV_Degrees": 90.0
                        }
                    ],
                    "X": 0.5,
                    "Y": 0.0,
                    "Z": 0.1,
                    "Pitch": 0.0,
                    "Roll": 0.0,
                    "Yaw": 0.0
                }
            }
        }
        
        settings["Vehicles"][drone_name] = drone_config
        
        # Add SubWindows for each drone
        settings["SubWindows"].extend([
            {
                "WindowID": 0,
                "CameraName": "front_rgb",
                "ImageType": 0,
                "VehicleName": drone_name,
                "Visible": True if i <= 2 else False  # Only show first 2 to avoid clutter
            },
            {
                "WindowID": 1,
                "CameraName": "segmentation_cam",
                "ImageType": 5,
                "VehicleName": drone_name,
                "Visible": True if i <= 2 else False
            }
        ])
        
        print(f"✅ Created configuration for {drone_name}")
        print(f"   Position: ({x_pos}, 0, -2)")
        print(f"   TCP Port: {tcp_port}")
        print(f"   Control: {control_local}↔{control_remote}")
    
    # Save the settings
    try:
        with open(settings_file, 'w') as f:
            json.dump(settings, f, indent=2)
        print(f"\n✅ Complete 5-drone settings saved to: {settings_file}")
        return True
    except Exception as e:
        print(f"\n❌ Error saving settings: {e}")
        return False

def test_all_vehicles_after_fix():
    """Test all vehicles after the complete fix"""
    print("\n🧪 TESTING ALL VEHICLES AFTER COMPLETE FIX...")
    
    print("⚠️ IMPORTANT: You MUST restart AirSim/Unreal Engine for the new settings to take effect!")
    
    try:
        response = input("\nHave you restarted AirSim? (y/n): ")
        if response.lower() != 'y':
            print("🔄 Please restart AirSim and run this script again!")
            return False
    except KeyboardInterrupt:
        return False
    
    try:
        print("\n🔌 Connecting to AirSim...")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicles = client.listVehicles()
        print(f"🚁 Found vehicles: {vehicles}")
        
        if len(vehicles) != 5:
            print(f"⚠️ Expected 5 vehicles, found {len(vehicles)}")
            print("   Make sure all PX4 containers are running and AirSim is restarted")
        
        working_vehicles = []
        
        for vehicle in vehicles:
            print(f"\n🔍 Testing {vehicle}...")
            try:
                client.enableApiControl(True, vehicle)
                print(f"   ✅ API control enabled")
                
                # Test arming
                can_arm = client.armDisarm(True, vehicle)
                if can_arm:
                    print(f"   🎉 {vehicle} ARMED SUCCESSFULLY!")
                    client.armDisarm(False, vehicle)
                    print(f"   ✅ {vehicle} disarmed safely")
                    working_vehicles.append(vehicle)
                else:
                    print(f"   ❌ {vehicle} could not arm")
                    
            except Exception as e:
                if "GPS home location" in str(e):
                    print(f"   ❌ {vehicle} still has GPS home error")
                else:
                    print(f"   ⚠️ {vehicle} error: {e}")
        
        return len(working_vehicles), len(vehicles)
        
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return 0, 0

def main():
    print("🚁 COMPLETE SETTINGS FIX FOR 5-DRONE SETUP")
    print("=" * 60)
    
    # Step 1: Create complete settings
    if not create_complete_5_drone_settings():
        print("❌ Failed to create settings")
        return 1
    
    # Step 2: Test vehicles (requires restart)
    working_count, total_count = test_all_vehicles_after_fix()
    
    # Results
    print("\n" + "=" * 60)
    if working_count > 0:
        print(f"🎉 SUCCESS! {working_count}/{total_count} vehicles can arm!")
        print("✅ Your multi-drone mission should work now!")
        
        print("\n🚁 TO RUN YOUR MISSION:")
        print("   python multi_drone_orbit_mission.py")
        print(f"   # Will use all {working_count} working drones")
        
        if working_count < total_count:
            print(f"\n⚠️ {total_count - working_count} vehicles still need attention")
            print("   Check Docker logs: docker logs px4-drone3")
    else:
        print("❌ VEHICLES STILL CANNOT ARM")
        print("\n🔧 NEXT STEPS:")
        print("1. Verify all 5 PX4 containers are running:")
        print("   docker ps | grep px4")
        print("2. Check container logs for errors:")
        print("   docker logs px4-drone1")
        print("3. Restart entire Docker stack:")
        print("   docker-compose down && docker-compose up -d")
    
    print(f"\n📋 SETTINGS SUMMARY:")
    print(f"   ✅ 5 vehicles configured: PX4_Drone1 through PX4_Drone5")
    print(f"   ✅ TCP ports: 4561-4565")
    print(f"   ✅ Control ports: 14541-14545 ↔ 14581-14585")
    print(f"   ✅ GPS sensors enabled for all vehicles")
    print(f"   ✅ Positions: (0,0,-2), (5,0,-2), (10,0,-2), (15,0,-2), (20,0,-2)")
    
    return 0 if working_count > 0 else 1

if __name__ == "__main__":
    sys.exit(main())