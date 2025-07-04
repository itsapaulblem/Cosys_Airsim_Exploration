#!/usr/bin/env python3
"""
AirSim Settings Generator for Scaled PX4 Setup
Automatically generates settings.json for multiple PX4 drones
"""

import json
import sys
import os
from pathlib import Path

def generate_airsim_settings(num_drones, output_file=None):
    """Generate AirSim settings.json for specified number of drones"""
    
    if num_drones < 1 or num_drones > 10:
        raise ValueError("Number of drones must be between 1 and 10")
    
    # Base settings template
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
        "PawnPaths": {
            "DefaultQuadrotor": {
                "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
            }
        },
        "Vehicles": {},
        "CameraDefaults": {
            "CaptureSettings": [
                {
                    "ImageType": 0,
                    "Width": 256,
                    "Height": 144,
                    "FOV_Degrees": 90,
                    "AutoExposureSpeed": 100,
                    "AutoExposureBias": 0,
                    "AutoExposureMaxBrightness": 0.64,
                    "AutoExposureMinBrightness": 0.03,
                    "TargetGamma": 1.0
                }
            ]
        }
    }
    
    # Generate vehicle configurations
    for i in range(1, num_drones + 1):
        drone_name = f"PX4_Drone{i}"
        
        # Calculate ports (same as in Docker setup)
        tcp_port = 4560 + i
        local_port = 14540 + i
        remote_port = 14580 + i
        
        # Calculate position (spread drones in a grid)
        if num_drones <= 5:
            # Single row for 5 or fewer drones
            x_pos = (i - 1) * 5
            y_pos = 0
        else:
            # Grid layout for more than 5 drones
            x_pos = ((i - 1) % 5) * 5
            y_pos = ((i - 1) // 5) * 5
        
        # Create vehicle configuration
        vehicle_config = {
            "VehicleType": "PX4Multirotor",
            "UseSerial": False,
            "LockStep": True,
            "UseTcp": True,
            "TcpPort": tcp_port,
            "ControlIp": "127.0.0.1",
            "ControlPortLocal": local_port,
            "ControlPortRemote": remote_port,
            "LocalHostIp": "127.0.0.1",
            "Sensors": {
                "Barometer": {
                    "SensorType": 1,
                    "Enabled": True,
                    "PressureFactorSigma": 0.0001825
                },
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
                "Imu": {
                    "SensorType": 2,
                    "Enabled": True
                },
                "Magnetometer": {
                    "SensorType": 4,
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
            "X": x_pos,
            "Y": y_pos,
            "Z": -2
        }
        
        settings["Vehicles"][drone_name] = vehicle_config
    
    # Write settings to file
    if output_file is None:
        # Default output location
        if os.name == 'nt':  # Windows
            output_file = os.path.expanduser("~/Documents/AirSim/settings.json")
        else:  # Linux/Mac
            output_file = os.path.expanduser("~/Documents/AirSim/settings.json")
    
    # Create directory if it doesn't exist
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Write the settings file
    with open(output_file, 'w') as f:
        json.dump(settings, f, indent=2)
    
    return settings, output_file

def print_port_summary(num_drones):
    """Print a summary of port assignments"""
    print(f"\n📋 Port Assignment Summary for {num_drones} Drone(s)")
    print("=" * 50)
    
    for i in range(1, num_drones + 1):
        tcp_port = 4560 + i
        local_port = 14540 + i
        remote_port = 14580 + i
        qgc_port = 14549 + i
        
        print(f"PX4_Drone{i}:")
        print(f"  - AirSim TCP:     {tcp_port}")
        print(f"  - MAVLink Local:  {local_port}")
        print(f"  - MAVLink Remote: {remote_port}")
        print(f"  - QGroundControl: {qgc_port}")
        if i < num_drones:
            print()

def main():
    if len(sys.argv) < 2:
        print("Usage: python generate_settings.py <number_of_drones> [output_file]")
        print("\nExamples:")
        print("  python generate_settings.py 1")
        print("  python generate_settings.py 3")
        print("  python generate_settings.py 5 ./my_settings.json")
        print("\nMaximum: 10 drones")
        sys.exit(1)
    
    try:
        num_drones = int(sys.argv[1])
        output_file = sys.argv[2] if len(sys.argv) > 2 else None
        
        print(f"🚁 Generating AirSim settings for {num_drones} drone(s)...")
        
        settings, output_path = generate_airsim_settings(num_drones, output_file)
        
        print(f"✅ Settings generated successfully!")
        print(f"📁 Output file: {output_path}")
        print(f"🔢 Number of vehicles: {len(settings['Vehicles'])}")
        
        # Print port summary
        print_port_summary(num_drones)
        
        # Print vehicle positions
        print(f"\n📍 Vehicle Positions:")
        for name, config in settings['Vehicles'].items():
            print(f"  {name}: X={config['X']}, Y={config['Y']}, Z={config['Z']}")
        
        print(f"\n💡 Next steps:")
        print(f"1. Copy {output_path} to your AirSim Documents folder")
        print(f"2. Start AirSim/Unreal Engine")
        print(f"3. Run: docker-compose -f docker-compose-scaled.yml up --scale px4={num_drones} -d")
        print(f"4. All drones should connect automatically!")
        
    except ValueError as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 