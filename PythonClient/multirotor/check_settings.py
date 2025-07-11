#\!/usr/bin/env python3
"""
Check AirSim settings and provide recommendations for GPS home error
"""
import setup_path
import cosysairsim as airsim
import json
import os
import platform

def find_settings_file():
    """Find the AirSim settings.json file"""
    # Common locations for settings.json
    if platform.system() == "Windows":
        user_profile = os.environ.get('USERPROFILE', '')
        settings_path = os.path.join(user_profile, 'Documents', 'AirSim', 'settings.json')
    else:
        home = os.path.expanduser('~')
        settings_path = os.path.join(home, 'Documents', 'AirSim', 'settings.json')
    
    if os.path.exists(settings_path):
        return settings_path
    
    # Check current directory
    if os.path.exists('settings.json'):
        return 'settings.json'
    
    return None

def analyze_settings(settings_path):
    """Analyze settings.json for GPS configuration issues"""
    try:
        with open(settings_path, 'r') as f:
            settings = json.load(f)
    except Exception as e:
        print(f"Error reading settings file: {e}")
        return
    
    print(f"Settings file: {settings_path}")
    print("=" * 50)
    
    # Check SimMode
    sim_mode = settings.get('SimMode', 'Unknown')
    print(f"SimMode: {sim_mode}")
    
    # Check OriginGeopoint
    origin = settings.get('OriginGeopoint')
    if origin:
        print("OriginGeopoint: ✓ Found")
        print(f"  Latitude:  {origin.get('Latitude', 'Not set')}")
        print(f"  Longitude: {origin.get('Longitude', 'Not set')}")
        print(f"  Altitude:  {origin.get('Altitude', 'Not set')}")
    else:
        print("OriginGeopoint: ✗ MISSING - This is likely the cause of your GPS home error\!")
    
    # Check Vehicles
    vehicles = settings.get('Vehicles', {})
    if vehicles:
        print(f"\nVehicles found: {list(vehicles.keys())}")
        
        for vehicle_name, vehicle_config in vehicles.items():
            print(f"\n--- Vehicle: {vehicle_name} ---")
            vehicle_type = vehicle_config.get('VehicleType', 'Unknown')
            print(f"  Type: {vehicle_type}")
            
            # Check GPS sensor
            sensors = vehicle_config.get('Sensors', {})
            gps_sensor = None
            for sensor_name, sensor_config in sensors.items():
                if sensor_config.get('SensorType') == 3:  # GPS sensor type
                    gps_sensor = sensor_config
                    print(f"  GPS Sensor ({sensor_name}): ✓ Found")
                    print(f"    Enabled: {sensor_config.get('Enabled', False)}")
                    break
            
            if not gps_sensor:
                print("  GPS Sensor: ✗ NOT FOUND")
            
            # Check Parameters for PX4
            if 'PX4' in vehicle_type:
                params = vehicle_config.get('Parameters', {})
                if params:
                    print(f"  Parameters: ✓ Found ({len(params)} parameters)")
                    
                    # Check critical GPS parameters
                    critical_params = ['LPE_LAT', 'LPE_LON', 'COM_ARM_WO_GPS']
                    for param in critical_params:
                        if param in params:
                            print(f"    {param}: {params[param]}")
                        else:
                            print(f"    {param}: ✗ MISSING")
                else:
                    print("  Parameters: ✗ NOT FOUND - Add GPS parameters for PX4")
    else:
        print("Vehicles: ✗ NO VEHICLES CONFIGURED")
    
    print("\n" + "=" * 50)

def check_airsim_connection():
    """Check if we can connect to AirSim and get vehicle info"""
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("AirSim Connection: ✓ SUCCESS")
        
        # Try to get default vehicle state
        try:
            state = client.getMultirotorState()
            print("Default Vehicle: ✓ FOUND")
        except Exception as e:
            print(f"Default Vehicle: ✗ ERROR - {e}")
        
        return True
    except Exception as e:
        print(f"AirSim Connection: ✗ FAILED - {e}")
        return False

def main():
    print("AirSim GPS Home Configuration Checker")
    print("=" * 50)
    
    # Check AirSim connection
    check_airsim_connection()
    print()
    
    # Find and analyze settings file
    settings_path = find_settings_file()
    
    if settings_path:
        analyze_settings(settings_path)
    else:
        print("Settings file: ✗ NOT FOUND")
        print("\nSettings.json should be located at:")
        if platform.system() == "Windows":
            print("  Windows: C:\\Users\\[YourName]\\Documents\\AirSim\\settings.json")
        else:
            print("  Linux/Mac: ~/Documents/AirSim/settings.json")
        print("\nCopy the px4_gps_fix_settings.json to this location.")
    
    print("\nRecommendations:")
    print("1. Copy px4_gps_fix_settings.json to your AirSim Documents folder")
    print("2. Restart AirSim after updating settings")
    print("3. Start PX4 SITL and wait 10-30 seconds for GPS lock")
    print("4. Use debug_gps_home_improved.py to monitor GPS status")

if __name__ == "__main__":
    main()
EOF < /dev/null
