# Example command to run the script for 4 drones (python3 generate_settings.py 4)

#!/usr/bin/env python3
import os
import json
import argparse
import shutil

def load_existing_settings(settings_path):
    """Load existing settings.json file"""
    if os.path.exists(settings_path):
        try:
            with open(settings_path, 'r') as f:
                settings = json.load(f)
            print(f"Loaded existing settings from {settings_path}")
            return settings
        except json.JSONDecodeError as e:
            print(f"Warning: Error parsing existing settings.json: {e}")
            print("Creating new settings configuration...")
            return None
        except Exception as e:
            print(f"Warning: Error reading existing settings.json: {e}")
            print("Creating new settings configuration...")
            return None
    else:
        print(f"No existing settings found at {settings_path}")
        print("Creating new settings configuration...")
        return None

def backup_settings(settings_path):
    """Create backup of existing settings file"""
    if os.path.exists(settings_path):
        backup_path = settings_path + ".backup"
        try:
            shutil.copy2(settings_path, backup_path)
            print(f"Backup created: {backup_path}")
        except Exception as e:
            print(f"Warning: Could not create backup: {e}")

def generate_settings_with_args(args, existing_settings=None):
    """Generate settings with command line arguments override"""
    
    # Default Z position for all drones
    DEFAULT_Z = 30.5

    # Start with existing settings or create new base config
    if existing_settings:
        base_config = existing_settings.copy()
        # Ensure required keys exist
        if "Vehicles" not in base_config:
            base_config["Vehicles"] = {}
        if "PawnPaths" not in base_config:
            base_config["PawnPaths"] = {}
    else:
        base_config = {
            "SettingsVersion": 2,
            "SimMode": "Multirotor",
            "ClockType": "SteppableClock",
            "Vehicles": {},
            "PawnPaths": {}
        }
    
    # Update base settings if provided
    if args.settings_version:
        base_config["SettingsVersion"] = args.settings_version
    if args.sim_mode:
        base_config["SimMode"] = args.sim_mode
    
    # Add/update PawnPaths configuration
    base_config["PawnPaths"]["DefaultQuadrotor"] = {
        "PawnBP": "Class'/AirSim/Blueprints/BP_MyPawn.BP_MyPawn_C'"
    }
    
    # Network settings
    local_host_ip = args.local_host_ip or os.getenv("LOCAL_HOST_IP", "172.22.112.1")
    control_ip = args.control_ip or os.getenv("CONTROL_IP", "remote")
    
    # Drone configuration
    num_drones = args.num_drones or int(os.getenv("NUM_DRONES", "1"))
    base_tcp_port = args.base_tcp_port or int(os.getenv("BASE_TCP_PORT", "4560"))
    base_control_port_local = args.base_control_local or int(os.getenv("BASE_CONTROL_PORT_LOCAL", "14540"))
    base_control_port_remote = args.base_control_remote or int(os.getenv("BASE_CONTROL_PORT_REMOTE", "14580"))
    
    # Clear existing vehicles and generate new ones
    base_config["Vehicles"] = {}
    
    # Generate drone configurations
    for i in range(1, num_drones + 1):
        drone_name = f"Drone{i}"
        
        # Generate positions for multiple drones on same horizontal level, close together
        if num_drones == 1:
            default_pos = {"X": 0, "Y": 0, "Z": DEFAULT_Z}
        else:
            # Get spacing from args or environment variable
            spacing = args.drone_spacing if hasattr(args, 'drone_spacing') else float(os.getenv("DRONE_SPACING", "4.0"))
            total_width = (num_drones - 1) * spacing
            start_x = -total_width / 2
            
            default_pos = {
                "X": start_x + (i - 1) * spacing,  # Keep all drones at same X position
                "Y": 0,  # Spread along Y axis
                "Z": DEFAULT_Z  # Start at 35.5 meters height
            }
        
        # Allow position override via environment variables
        x_pos = float(os.getenv(f"DRONE{i}_X", default_pos["X"]))
        y_pos = float(os.getenv(f"DRONE{i}_Y", default_pos["Y"]))
        z_pos = float(os.getenv(f"DRONE{i}_Z", default_pos["Z"]))
        

        sensors = {
            "Barometer": {
                "SensorType": 1,
                "Enabled": True
            }, 
            "Imu": {
                "SensorType": 2,
                "Enabled": True
            }, 
            "Gps": {
                "SensorType": 3,
                "Enabled": True
            },
            "Magnetometer": {
                "SensorType": 4,
                "Enabled": True
            },
            "Lidar1": {
                "SensorType": 6,
                "Enabled": True,
                "NumberOfChannels": 16,
                "Range": 100,
                "PointsPerSecond": 10000,
                "DrawDebugPoints": False,
                "X": 0, "Y": 0, "Z": 0,
                "Roll": 0, "Pitch": 0, "Yaw": 0
            }
        }

        base_config["Vehicles"][drone_name] = {
            "VehicleType": os.getenv(f"DRONE{i}_VEHICLE_TYPE", "PX4Multirotor"),
            "UseSerial": os.getenv(f"DRONE{i}_USE_SERIAL", "false").lower() == "true",
            "LockStep": os.getenv(f"DRONE{i}_LOCKSTEP", "true").lower() == "true",
            "UseTcp": os.getenv(f"DRONE{i}_USE_TCP", "true").lower() == "true",
            "RpcEnabled": os.getenv(f"DRONE{i}_RPC_ENABLED", "true").lower() == "true",
            "TcpPort": int(os.getenv(f"DRONE{i}_TCP_PORT", base_tcp_port + i - 1)),
            "ControlIp": os.getenv(f"DRONE{i}_CONTROL_IP", control_ip),
            "ControlPortLocal": int(os.getenv(f"DRONE{i}_CONTROL_PORT_LOCAL", base_control_port_local + i - 1)),
            "ControlPortRemote": int(os.getenv(f"DRONE{i}_CONTROL_PORT_REMOTE", base_control_port_remote + i - 1)),
            "LocalHostIp": os.getenv(f"DRONE{i}_LOCAL_HOST_IP", local_host_ip),
            "X": x_pos,
            "Y": y_pos,
            "Z": z_pos,
            "Yaw": float(os.getenv(f"DRONE{i}_YAW", "0")),
            "Sensors": sensors
        }
    
    return base_config

def save_settings(config, output_path):
    """Save the generated configuration to a JSON file"""
    try:
        # Ensure directory exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Settings updated at {output_path}")
        print(f"Configured {len(config['Vehicles'])} drone(s)")
        return True
    except Exception as e:
        print(f"Error saving settings: {e}")
        return False

def get_settings_path():
    """Get the AirSim settings path"""
    # Default Windows path
    windows_path = r"C:\Users\UserAdmin\Documents\AirSim\settings.json"
    
    # Check if running on WSL and convert path
    if os.path.exists("/mnt/c"):
        wsl_path = "/mnt/c/Users/UserAdmin/Documents/AirSim/settings.json"
        return wsl_path
    
    return windows_path

def main():
    parser = argparse.ArgumentParser(description="Update AirSim settings.json with configurable drones")
    parser.add_argument("num_drones", type=int, nargs='?', 
                       help="Number of drones to configure")
    parser.add_argument("-o", "--output", 
                       help="Output file path (default: AirSim settings location)")
    parser.add_argument("--print-only", action="store_true",
                       help="Print configuration without saving")
    parser.add_argument("--create-backup", action="store_true",
                       help="Create backup file before modifying")
    
    # Configuration arguments
    parser.add_argument("--drone-spacing", type=float, default= 10.0,
                       help="Distance between drones in meters (default: 10.0)")
    parser.add_argument("--local-host-ip", 
                       help="Local host IP address")
    parser.add_argument("--control-ip",
                       help="Control IP address")
    parser.add_argument("--base-tcp-port", type=int,
                       help="Base TCP port number")
    parser.add_argument("--base-control-local", type=int,
                       help="Base control port local")
    parser.add_argument("--base-control-remote", type=int,
                       help="Base control port remote")
    parser.add_argument("--settings-version", type=float,
                       help="Settings version")
    parser.add_argument("--sim-mode",
                       help="Simulation mode")
    parser.add_argument("--show-ports", action="store_true",
                       help="Show port assignments for each drone")
    
    args = parser.parse_args()
    
    # Get number of drones from command line or environment
    if not args.num_drones:
        args.num_drones = int(os.getenv("NUM_DRONES", "3"))
        print(f"Using NUM_DRONES from environment: {args.num_drones}")
    
    # Determine output path
    if args.output:
        settings_path = args.output
    else:
        settings_path = get_settings_path()
    
    print(f"Target settings file: {settings_path}")
    
    # Load existing settings
    existing_settings = load_existing_settings(settings_path)
    
    # Create backup if requested
    if args.create_backup and not args.print_only:
        backup_settings(settings_path)
    
    # Generate configuration
    config = generate_settings_with_args(args, existing_settings)
    
    if args.print_only:
        print(json.dumps(config, indent=2))
    else:
        success = save_settings(config, settings_path)
        
        if success and args.show_ports:
            print("\nPort Assignments:")
            print("-" * 50)
            for drone_name, drone_config in config["Vehicles"].items():
                print(f"{drone_name}:")
                print(f"  TCP Port: {drone_config['TcpPort']}")
                print(f"  Control Port Local: {drone_config['ControlPortLocal']}")
                print(f"  Control Port Remote: {drone_config['ControlPortRemote']}")
                print(f"  Position: ({drone_config['X']}, {drone_config['Y']}, {drone_config['Z']})")
                print()

if __name__ == "__main__":
    main()