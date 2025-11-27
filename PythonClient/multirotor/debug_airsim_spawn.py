#!/usr/bin/env python3
"""
Diagnostic script to debug AirSim spawn positions and settings loading
"""

import sys
import json
import time
from pathlib import Path

# Add AirSim Python client to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
# import cosysairsim as airsim

try:
    import cosysairsim as airsim
    print("✅ Using cosysairsim client")
except ImportError:
    try:
        import airsim
        print("✅ Using standard airsim client") 
    except ImportError:
        print("❌ No AirSim Python client available")
        sys.exit(1)

def main():
    """Debug AirSim spawn positions and settings"""
    
    print("🔍 AIRSIM SPAWN POSITION DIAGNOSTIC")
    print("=" * 50)
    
    # Connect to AirSim
    try:
        client = airsim.MultirotorClient(ip='localhost', port=41451)
        client.confirmConnection()
        print("✅ Connected to AirSim server")
    except Exception as e:
        print(f"❌ Failed to connect to AirSim: {e}")
        return
    
    # Get vehicle list
    try:
        vehicles = client.listVehicles()
        print(f"🚁 Discovered vehicles: {vehicles}")
    except Exception as e:
        print(f"❌ Failed to get vehicle list: {e}")
        return
    
    # Get settings from AirSim server
    print("\n📋 SETTINGS FROM AIRSIM SERVER:")
    print("-" * 30)
    try:
        settings_string = client.getSettingsString()
        if settings_string:
            settings = json.loads(settings_string)
            
            if "Vehicles" in settings:
                print("✅ Vehicles found in AirSim settings:")
                for name, config in settings["Vehicles"].items():
                    x = config.get("X", "not specified")
                    y = config.get("Y", "not specified") 
                    z = config.get("Z", "not specified")
                    vehicle_type = config.get("VehicleType", "not specified")
                    
                    print(f"  • {name}: Type={vehicle_type}, Position=[{x}, {y}, {z}]")
            else:
                print("❌ No 'Vehicles' section in AirSim settings")
                print("Raw settings keys:", list(settings.keys()))
        else:
            print("❌ AirSim returned empty settings string")
            
    except Exception as e:
        print(f"❌ Failed to get settings from AirSim: {e}")
    
    # Get actual vehicle positions 
    print("\n🎯 ACTUAL VEHICLE POSITIONS:")
    print("-" * 30)
    for vehicle_name in vehicles:
        try:
            state = client.getMultirotorState(vehicle_name)
            pos = state.kinematics_estimated.pose.position
            print(f"  • {vehicle_name}: AirSim NED=[{pos.x():.3f}, {pos.y():.3f}, {pos.z():.3f}]")
            
            # Check GPS position too
            gps = client.getGpsData("", vehicle_name)
            if gps:
                print(f"    GPS: lat={gps.gnss.geo_point.latitude:.6f}, lon={gps.gnss.geo_point.longitude:.6f}, alt={gps.gnss.geo_point.altitude:.3f}")
                
        except Exception as e:
            print(f"  ❌ Failed to get position for {vehicle_name}: {e}")
    
    # Check local settings.json file
    print("\n📄 LOCAL SETTINGS.JSON FILE:")
    print("-" * 30)
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    if settings_path.exists():
        print(f"✅ Found local settings file: {settings_path}")
        try:
            with open(settings_path, 'r') as f:
                local_settings = json.load(f)
                
            if "Vehicles" in local_settings:
                print("✅ Vehicles in local settings.json:")
                for name, config in local_settings["Vehicles"].items():
                    x = config.get("X", "not specified")
                    y = config.get("Y", "not specified")
                    z = config.get("Z", "not specified") 
                    vehicle_type = config.get("VehicleType", "not specified")
                    
                    print(f"  • {name}: Type={vehicle_type}, Position=[{x}, {y}, {z}]")
            else:
                print("❌ No 'Vehicles' section in local settings.json")
        except Exception as e:
            print(f"❌ Failed to read local settings.json: {e}")
    else:
        print(f"❌ Local settings file not found: {settings_path}")
    
    print("\n🔍 DIAGNOSTIC SUMMARY:")
    print("-" * 30)
    print("1. Check if AirSim settings match local settings.json")
    print("2. Verify if vehicle positions match configured spawn points")
    print("3. If positions are all the same, AirSim may not be loading spawn positions correctly")
    
if __name__ == "__main__":
    main()