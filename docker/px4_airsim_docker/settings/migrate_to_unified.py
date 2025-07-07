#!/usr/bin/env python3
"""
Migration script to help users transition from legacy tools to the unified workflow
Converts existing configurations and provides upgrade guidance
"""

import json
import yaml
import sys
from pathlib import Path
import shutil
from datetime import datetime
from typing import Dict, List, Optional

class LegacyMigrator:
    def __init__(self):
        self.backup_dir = Path("./migration_backup")
        self.issues = []
        
    def migrate_legacy_settings(self, settings_file: str) -> Optional[Dict]:
        """Migrate legacy settings.json to new format"""
        try:
            with open(settings_file, 'r') as f:
                settings = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.issues.append(f"Cannot read settings file: {e}")
            return None
        
        # Create backup
        self._backup_file(settings_file)
        
        # Check if already in new format
        if self._is_unified_format(settings):
            print(f"✅ {settings_file} is already in unified format")
            return settings
        
        # Convert legacy format
        converted = self._convert_settings_format(settings)
        
        if converted:
            print(f"🔄 Converted {settings_file} to unified format")
            
            # Write converted settings
            backup_original = settings_file + ".legacy"
            shutil.copy2(settings_file, backup_original)
            
            with open(settings_file, 'w') as f:
                json.dump(converted, f, indent=2)
                
            print(f"📁 Original saved as {backup_original}")
        
        return converted
    
    def migrate_legacy_compose(self, compose_file: str) -> Optional[Dict]:
        """Migrate legacy docker-compose.yml to new format"""
        try:
            with open(compose_file, 'r') as f:
                compose = yaml.safe_load(f)
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.issues.append(f"Cannot read compose file: {e}")
            return None
        
        # Create backup
        self._backup_file(compose_file)
        
        # Check if already in new format
        if self._is_unified_compose_format(compose):
            print(f"✅ {compose_file} is already in unified format")
            return compose
        
        # Convert legacy format
        converted = self._convert_compose_format(compose)
        
        if converted:
            print(f"🔄 Converted {compose_file} to unified format")
            
            # Write converted compose
            backup_original = compose_file + ".legacy"
            shutil.copy2(compose_file, backup_original)
            
            with open(compose_file, 'w') as f:
                yaml.dump(converted, f, default_flow_style=False, sort_keys=False)
                
            print(f"📁 Original saved as {backup_original}")
        
        return converted
    
    def analyze_legacy_setup(self) -> Dict:
        """Analyze current setup and recommend migration strategy"""
        analysis = {
            "files_found": [],
            "legacy_tools": [],
            "unified_tools": [],
            "recommendations": []
        }
        
        # Check for existing files
        files_to_check = [
            "settings.json",
            "docker-compose.yml", 
            "docker-compose.generated.yml",
            "docker-compose-scaled.yml",
            "airsim_px4_single_settings.json",
            "airsim_px4_multi_settings.json"
        ]
        
        for file in files_to_check:
            if Path(file).exists():
                analysis["files_found"].append(file)
        
        # Check for legacy tools
        legacy_tools = [
            "generate_settings.py",
            "simple_generator.py"
        ]
        
        for tool in legacy_tools:
            if Path(tool).exists():
                analysis["legacy_tools"].append(tool)
        
        # Check for unified tools
        unified_tools = [
            "unified_generator.py",
            "config_validator.py"
        ]
        
        for tool in unified_tools:
            if Path(tool).exists():
                analysis["unified_tools"].append(tool)
        
        # Generate recommendations
        if not analysis["unified_tools"]:
            analysis["recommendations"].append(
                "❌ Unified tools not found - ensure you're in the correct directory"
            )
        
        if analysis["legacy_tools"] and analysis["unified_tools"]:
            analysis["recommendations"].append(
                "🔄 Both legacy and unified tools found - migration recommended"
            )
        
        if "settings.json" in analysis["files_found"]:
            analysis["recommendations"].append(
                "📄 Existing settings.json found - will be migrated and backed up"
            )
        
        if any("docker-compose" in f for f in analysis["files_found"]):
            analysis["recommendations"].append(
                "🐳 Docker compose files found - will be migrated to unified format"
            )
        
        return analysis
    
    def _backup_file(self, file_path: str):
        """Create timestamped backup of a file"""
        self.backup_dir.mkdir(exist_ok=True)
        
        source = Path(file_path)
        if source.exists():
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_name = f"{source.stem}_{timestamp}{source.suffix}"
            backup_path = self.backup_dir / backup_name
            
            shutil.copy2(source, backup_path)
            print(f"📁 Backed up {file_path} to {backup_path}")
    
    def _is_unified_format(self, settings: Dict) -> bool:
        """Check if settings are already in unified format"""
        # Unified format indicators
        indicators = [
            settings.get("SettingsVersion") == 2.0,
            "OriginGeopoint" in settings,
            "ClockType" in settings
        ]
        
        # Check vehicle format
        vehicles = settings.get("Vehicles", {})
        for vehicle in vehicles.values():
            if isinstance(vehicle, dict):
                # Look for unified format fields
                if "Parameters" in vehicle and "LockStep" in vehicle:
                    indicators.append(True)
                    break
        
        return sum(indicators) >= 2
    
    def _is_unified_compose_format(self, compose: Dict) -> bool:
        """Check if compose is already in unified format"""
        # Unified format indicators
        indicators = [
            "version" in compose,
            compose.get("networks", {}).get("airsim-network", {}).get("name") == "airsim-network",
            "px4-shared-data" in compose.get("volumes", {})
        ]
        
        return sum(indicators) >= 2
    
    def _convert_settings_format(self, settings: Dict) -> Dict:
        """Convert legacy settings to unified format"""
        # Start with the original settings
        converted = settings.copy()
        
        # Ensure required fields
        converted["SettingsVersion"] = 2.0
        converted["SimMode"] = settings.get("SimMode", "Multirotor")
        converted["ClockType"] = settings.get("ClockType", "SteppableClock")
        converted["ApiServerEndpoint"] = settings.get("ApiServerEndpoint", "0.0.0.0:41451")
        
        # Add OriginGeopoint if missing
        if "OriginGeopoint" not in converted:
            converted["OriginGeopoint"] = {
                "Latitude": 47.641468,
                "Longitude": -122.140165,
                "Altitude": 10
            }
        
        # Convert vehicle configurations
        vehicles = converted.get("Vehicles", {})
        for vehicle_name, vehicle_config in vehicles.items():
            if isinstance(vehicle_config, dict):
                # Add missing PX4 parameters if it's a PX4 vehicle
                if vehicle_config.get("VehicleType") == "PX4Multirotor":
                    if "Parameters" not in vehicle_config:
                        vehicle_config["Parameters"] = {
                            "NAV_RCL_ACT": 0,
                            "NAV_DLL_ACT": 0,
                            "COM_OBL_ACT": 1,
                            "LPE_LAT": converted["OriginGeopoint"]["Latitude"],
                            "LPE_LON": converted["OriginGeopoint"]["Longitude"],
                            "COM_ARM_WO_GPS": 0,
                            "EKF2_AID_MASK": 1,
                            "EKF2_HGT_MODE": 0,
                            "EKF2_GPS_CHECK": 31
                        }
                    
                    # Ensure LockStep is enabled
                    vehicle_config["LockStep"] = True
                    
                    # Update GPS sensor format if needed
                    sensors = vehicle_config.get("Sensors", {})
                    if "Gps" in sensors:
                        gps_config = sensors["Gps"]
                        if "StartLatitude" in gps_config:
                            # Convert old format to new
                            gps_config["EphTimeConstant"] = 0.9
                            gps_config["EpvTimeConstant"] = 0.9
                            gps_config["EphInitial"] = 25.0
                            gps_config["EpvInitial"] = 25.0
                            gps_config["EphFinal"] = 0.1
                            gps_config["EpvFinal"] = 0.1
                            gps_config["EphMin3d"] = 3.0
                            gps_config["EphMin2d"] = 4.0
                            gps_config["UpdateLatency"] = 0.2
                            gps_config["UpdateFrequency"] = 50
                            gps_config["StartupDelay"] = 1
        
        # Add PawnPaths if missing
        if "PawnPaths" not in converted:
            converted["PawnPaths"] = {
                "DefaultQuadrotor": {
                    "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
                }
            }
        
        return converted
    
    def _convert_compose_format(self, compose: Dict) -> Dict:
        """Convert legacy compose to unified format"""
        # Start with basic unified structure
        converted = {
            "version": "3.8",
            "services": {},
            "networks": {
                "airsim-network": {
                    "name": "airsim-network",
                    "driver": "bridge",
                    "ipam": {
                        "driver": "default",
                        "config": [{
                            "subnet": "172.25.0.0/16",
                            "gateway": "172.25.0.1"
                        }]
                    }
                }
            },
            "volumes": {
                "px4-shared-data": {
                    "driver": "local"
                }
            }
        }
        
        # Convert services from legacy format
        legacy_services = compose.get("services", {})
        for service_name, service_config in legacy_services.items():
            if isinstance(service_config, dict) and "px4" in service_name.lower():
                converted_service = self._convert_service_config(service_config)
                converted["services"][service_name] = converted_service
        
        return converted
    
    def _convert_service_config(self, service: Dict) -> Dict:
        """Convert legacy service configuration"""
        converted = {
            "build": {
                "context": ".",
                "dockerfile": "Dockerfile"
            },
            "container_name": service.get("container_name", "px4-container"),
            "hostname": service.get("hostname", service.get("container_name", "px4-container")),
            "environment": service.get("environment", []),
            "ports": service.get("ports", []),
            "networks": {
                "airsim-network": {}
            },
            "volumes": [
                "px4-shared-data:/px4_data"
            ],
            "restart": "unless-stopped"
        }
        
        # Add IP address if specified in legacy format
        legacy_networks = service.get("networks", {})
        if isinstance(legacy_networks, dict):
            for network_name, network_config in legacy_networks.items():
                if isinstance(network_config, dict) and "ipv4_address" in network_config:
                    converted["networks"]["airsim-network"]["ipv4_address"] = network_config["ipv4_address"]
        
        return converted
    
    def generate_migration_report(self) -> str:
        """Generate a migration report"""
        report = f"""
# Migration Report - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Analysis Results
"""
        
        analysis = self.analyze_legacy_setup()
        
        report += f"\n### Files Found ({len(analysis['files_found'])})\n"
        for file in analysis["files_found"]:
            report += f"- {file}\n"
        
        report += f"\n### Legacy Tools ({len(analysis['legacy_tools'])})\n"
        for tool in analysis["legacy_tools"]:
            report += f"- {tool}\n"
        
        report += f"\n### Unified Tools ({len(analysis['unified_tools'])})\n"
        for tool in analysis["unified_tools"]:
            report += f"- {tool}\n"
        
        report += f"\n### Recommendations\n"
        for rec in analysis["recommendations"]:
            report += f"- {rec}\n"
        
        if self.issues:
            report += f"\n### Issues Encountered\n"
            for issue in self.issues:
                report += f"- ❌ {issue}\n"
        
        report += f"""
## Next Steps

1. **Backup Verification**: Check `{self.backup_dir}` for backed up files
2. **Validation**: Run `python config_validator.py --all`
3. **Testing**: Test the migrated configuration with a small setup
4. **Deployment**: Use the unified workflow for new configurations

## Unified Workflow Commands

```bash
# Generate new configurations
python unified_generator.py multi --num-drones 3

# Validate configurations  
python config_validator.py --all

# Example workflow
python workflow_example.py
```
"""
        
        return report

def main():
    """Main migration workflow"""
    print("🔄 LEGACY TO UNIFIED MIGRATION TOOL")
    print("="*80)
    print("This tool helps migrate from legacy AirSim configuration tools")
    print("to the new standardized unified workflow.")
    print("="*80)
    
    migrator = LegacyMigrator()
    
    # Analyze current setup
    print("\n🔍 Analyzing current setup...")
    analysis = migrator.analyze_legacy_setup()
    
    print(f"\n📊 Analysis Results:")
    print(f"Files found: {len(analysis['files_found'])}")
    print(f"Legacy tools: {len(analysis['legacy_tools'])}")
    print(f"Unified tools: {len(analysis['unified_tools'])}")
    
    if not analysis['unified_tools']:
        print("\n❌ Unified tools not found!")
        print("Please ensure you're running this from the settings directory")
        print("and that unified_generator.py and config_validator.py are present.")
        return 1
    
    # Show recommendations
    if analysis['recommendations']:
        print(f"\n💡 Recommendations:")
        for rec in analysis['recommendations']:
            print(f"  {rec}")
    
    # Ask user for migration
    if analysis['files_found']:
        print(f"\n🔄 Migration Options:")
        print("1. Migrate existing settings.json")
        print("2. Migrate existing docker-compose files")
        print("3. Full migration (settings + compose)")
        print("4. Generate report only")
        print("0. Exit")
        
        try:
            choice = input("\nSelect migration option (0-4): ").strip()
        except KeyboardInterrupt:
            print("\n\nExiting...")
            return 0
        
        if choice == "0":
            print("Migration cancelled.")
            return 0
        elif choice == "1":
            # Migrate settings only
            for file in analysis['files_found']:
                if file.endswith('.json'):
                    migrator.migrate_legacy_settings(file)
        elif choice == "2":
            # Migrate compose only
            for file in analysis['files_found']:
                if 'docker-compose' in file and file.endswith(('.yml', '.yaml')):
                    migrator.migrate_legacy_compose(file)
        elif choice == "3":
            # Full migration
            for file in analysis['files_found']:
                if file.endswith('.json'):
                    migrator.migrate_legacy_settings(file)
                elif 'docker-compose' in file and file.endswith(('.yml', '.yaml')):
                    migrator.migrate_legacy_compose(file)
        elif choice == "4":
            # Report only
            pass
        else:
            print("❌ Invalid choice.")
            return 1
    
    # Generate migration report
    report = migrator.generate_migration_report()
    
    report_file = "migration_report.md"
    with open(report_file, 'w') as f:
        f.write(report)
    
    print(f"\n📄 Migration report saved to: {report_file}")
    
    # Final recommendations
    print("\n✅ Migration completed!")
    print("\n💡 Next steps:")
    print("1. Review the migration report")
    print("2. Validate migrated configurations: python config_validator.py --all")
    print("3. Test with: python workflow_example.py")
    print("4. Use unified workflow for future configurations")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())