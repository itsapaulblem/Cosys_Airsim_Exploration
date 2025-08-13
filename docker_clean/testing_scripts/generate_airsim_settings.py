#!/usr/bin/env python3
"""
Unified AirSim Settings Generator
Generates settings.json files for multi-drone Docker PX4 setups with proper pawn configuration.
"""

import json
import os
import sys
import argparse
from pathlib import Path

def create_airsim_settings(num_drones=5, vehicle_spacing=5, use_spirit_pawn=True, 
                          lockstep=True, clock_type="SteppableClock"):
    """
    Generate AirSim settings.json for multi-drone PX4 setup.
    
    Args:
        num_drones: Number of drones to configure (1-5)
        vehicle_spacing: Distance between vehicles in meters
        use_spirit_pawn: Use BP_SpiritPawn instead of default pawn
        lockstep: Enable lockstep simulation for deterministic behavior
        clock_type: Clock type for simulation (SteppableClock or ScalableClock)
    """
    
    # Base settings structure
    settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockType": clock_type,
        "Vehicles": {}
    }
    
    # Add PawnPaths configuration if using Spirit Pawn
    if use_spirit_pawn:
        settings["PawnPaths"] = {
            "DefaultQuadrotor": {
                "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
            }
        }
    
    # Generate vehicle configurations
    positions = [
        (0, 0, 0),      # Drone 1 - Center
        (vehicle_spacing, 0, 0),     # Drone 2 - East
        (-vehicle_spacing, 0, 0),    # Drone 3 - West  
        (0, vehicle_spacing, 0),     # Drone 4 - North
        (0, -vehicle_spacing, 0),    # Drone 5 - South
    ]
    
    for i in range(num_drones):
        drone_num = i + 1
        vehicle_name = f"PX4_Drone{drone_num}"
        control_port = 4560 + drone_num  # 4561, 4562, 4563, 4564, 4565
        
        x, y, z = positions[i]
        
        vehicle_config = {
            "VehicleType": "PX4Multirotor",
            "X": x,
            "Y": y, 
            "Z": z,
            "UseSerial": False,
            "LockStep": lockstep,
            "ControlPort": control_port
        }
        
        settings["Vehicles"][vehicle_name] = vehicle_config
    
    return settings

def get_airsim_settings_path():
    """Get the default AirSim settings path for the current OS."""
    home = Path.home()
    
    if os.name == 'nt':  # Windows
        return home / "Documents" / "AirSim" / "settings.json"
    else:  # Linux/Mac
        return home / "Documents" / "AirSim" / "settings.json"

def save_settings(settings, output_path=None, backup_existing=True):
    """
    Save settings to file with optional backup of existing settings.
    
    Args:
        settings: Settings dictionary to save
        output_path: Custom output path (defaults to AirSim documents folder)
        backup_existing: Create backup of existing settings.json
    """
    
    if output_path is None:
        output_path = get_airsim_settings_path()
    else:
        output_path = Path(output_path)
    
    # Create directory if it doesn't exist
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Backup existing settings if requested
    if backup_existing and output_path.exists():
        backup_path = output_path.with_suffix('.json.backup')
        backup_path.write_text(output_path.read_text())
        print(f"  📁 Backed up existing settings to: {backup_path}")
    
    # Write new settings
    with open(output_path, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"  ✅ Settings saved to: {output_path}")
    return output_path

def validate_settings(settings):
    """Validate the generated settings for common issues."""
    issues = []
    
    # Check required fields
    required_fields = ["SettingsVersion", "SimMode", "Vehicles"]
    for field in required_fields:
        if field not in settings:
            issues.append(f"Missing required field: {field}")
    
    # Check vehicle configurations
    if "Vehicles" in settings:
        for vehicle_name, config in settings["Vehicles"].items():
            if "ControlPort" not in config:
                issues.append(f"Vehicle {vehicle_name} missing ControlPort")
            elif config["ControlPort"] < 4561 or config["ControlPort"] > 4565:
                issues.append(f"Vehicle {vehicle_name} ControlPort {config['ControlPort']} outside expected range 4561-4565")
            
            if "VehicleType" not in config:
                issues.append(f"Vehicle {vehicle_name} missing VehicleType")
            elif config["VehicleType"] != "PX4Multirotor":
                issues.append(f"Vehicle {vehicle_name} VehicleType should be 'PX4Multirotor' for PX4 integration")
    
    # Check PawnPaths if present
    if "PawnPaths" in settings:
        pawn_paths = settings["PawnPaths"]
        if "DefaultQuadrotor" in pawn_paths:
            pawn_bp = pawn_paths["DefaultQuadrotor"].get("PawnBP", "")
            if "BP_SpiritPawn" not in pawn_bp:
                issues.append("PawnBP does not reference BP_SpiritPawn as requested")
    
    return issues

def print_settings_summary(settings):
    """Print a summary of the generated settings."""
    print("\n📋 SETTINGS SUMMARY")
    print("=" * 50)
    print(f"SimMode: {settings.get('SimMode', 'Unknown')}")
    print(f"ClockType: {settings.get('ClockType', 'Unknown')}")
    
    if "PawnPaths" in settings:
        pawn_bp = settings["PawnPaths"]["DefaultQuadrotor"]["PawnBP"]
        print(f"Default Pawn: {pawn_bp}")
    
    print(f"Vehicles: {len(settings.get('Vehicles', {}))}")
    
    if "Vehicles" in settings:
        print("\nVehicle Configuration:")
        for name, config in settings["Vehicles"].items():
            x, y, z = config.get("X", 0), config.get("Y", 0), config.get("Z", 0)
            port = config.get("ControlPort", "Unknown")
            lockstep = config.get("LockStep", False)
            print(f"  {name}: Port {port}, Position ({x}, {y}, {z}), LockStep: {lockstep}")

def main():
    parser = argparse.ArgumentParser(description='Generate AirSim settings.json for multi-drone PX4 setup')
    
    parser.add_argument('--num_drones', type=int, default=5, choices=range(1, 6),
                       help='Number of drones to configure (1-5, default: 5)')
    parser.add_argument('--spacing', type=float, default=5.0,
                       help='Distance between vehicles in meters (default: 5.0)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file path (default: ~/Documents/AirSim/settings.json)')
    parser.add_argument('--no_backup', action='store_true',
                       help='Do not backup existing settings.json')
    parser.add_argument('--no_spirit_pawn', action='store_true',
                       help='Do not use BP_SpiritPawn (use default pawn)')
    parser.add_argument('--no_lockstep', action='store_true', 
                       help='Disable lockstep simulation')
    parser.add_argument('--clock_type', choices=['SteppableClock', 'ScalableClock'], 
                       default='SteppableClock',
                       help='Clock type for simulation (default: SteppableClock)')
    parser.add_argument('--validate_only', action='store_true',
                       help='Only validate existing settings.json, do not generate new one')
    parser.add_argument('--preview', action='store_true',
                       help='Preview generated settings without saving')
    
    args = parser.parse_args()
    
    print("🚁 AIRSIM SETTINGS GENERATOR")
    print("=" * 50)
    
    if args.validate_only:
        # Validate existing settings
        settings_path = get_airsim_settings_path()
        if not settings_path.exists():
            print(f"❌ No settings.json found at {settings_path}")
            return 1
        
        try:
            with open(settings_path, 'r') as f:
                settings = json.load(f)
            
            issues = validate_settings(settings)
            if issues:
                print("❌ Settings validation failed:")
                for issue in issues:
                    print(f"  - {issue}")
                return 1
            else:
                print("✅ Settings validation passed")
                print_settings_summary(settings)
                return 0
        except Exception as e:
            print(f"❌ Error reading settings: {e}")
            return 1
    
    # Generate new settings
    print(f"Generating settings for {args.num_drones} drone(s)...")
    print(f"Vehicle spacing: {args.spacing}m")
    print(f"Spirit Pawn: {'Yes' if not args.no_spirit_pawn else 'No'}")
    print(f"LockStep: {'Yes' if not args.no_lockstep else 'No'}")
    print(f"Clock Type: {args.clock_type}")
    
    settings = create_airsim_settings(
        num_drones=args.num_drones,
        vehicle_spacing=args.spacing,
        use_spirit_pawn=not args.no_spirit_pawn,
        lockstep=not args.no_lockstep,
        clock_type=args.clock_type
    )
    
    # Validate generated settings
    issues = validate_settings(settings)
    if issues:
        print("\n❌ Generated settings have issues:")
        for issue in issues:
            print(f"  - {issue}")
        return 1
    
    print_settings_summary(settings)
    
    if args.preview:
        print("\n📄 GENERATED SETTINGS (PREVIEW)")
        print("=" * 50)
        print(json.dumps(settings, indent=2))
        print("\n💡 Use without --preview to save to file")
        return 0
    
    # Save settings
    print(f"\n💾 SAVING SETTINGS")
    print("=" * 50)
    
    try:
        saved_path = save_settings(
            settings, 
            output_path=args.output,
            backup_existing=not args.no_backup
        )
        
        print(f"\n✅ SUCCESS! Settings generated and saved.")
        print(f"📁 Location: {saved_path}")
        print("\n🚀 Next steps:")
        print("1. Start AirSim/Unreal Engine")
        print("2. Start PX4 Docker containers")
        print("3. Run your missions!")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error saving settings: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())