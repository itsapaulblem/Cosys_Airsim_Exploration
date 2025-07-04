#!/usr/bin/env python3
"""
Fix AirSim settings.json network configuration for Docker containers
Updates IP addresses to use Docker gateway instead of localhost
"""

import json
import subprocess
import shutil
from pathlib import Path
from datetime import datetime

def get_docker_gateway_ip():
    """Get the Docker airsim-network gateway IP"""
    try:
        result = subprocess.run(
            ['docker', 'network', 'inspect', 'airsim-network'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            network_info = json.loads(result.stdout)
            gateway = network_info[0]['IPAM']['Config'][0]['Gateway']
            print(f"✅ Found Docker gateway IP: {gateway}")
            return gateway
    except Exception as e:
        print(f"⚠️ Could not get Docker network info: {e}")
    
    # Default to the expected gateway for our network
    default_gateway = "172.25.0.1"
    print(f"Using default gateway IP: {default_gateway}")
    return default_gateway

def backup_settings(settings_path):
    """Create a backup of the current settings.json"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = settings_path.parent / f"settings_backup_{timestamp}.json"
    
    try:
        shutil.copy2(settings_path, backup_path)
        print(f"📁 Backup created: {backup_path}")
        return backup_path
    except Exception as e:
        print(f"⚠️ Could not create backup: {e}")
        return None

def fix_settings(settings_path, gateway_ip):
    """Fix the network settings in settings.json"""
    
    # Load current settings
    with open(settings_path, 'r') as f:
        settings = json.load(f)
    
    vehicles = settings.get("Vehicles", {})
    if not vehicles:
        print("❌ No vehicles found in settings.json")
        return False
    
    print(f"\n🔧 Updating network configuration for {len(vehicles)} vehicles...")
    
    # Update each vehicle's network configuration
    for vehicle_name, vehicle_config in vehicles.items():
        print(f"\n🚁 Updating {vehicle_name}:")
        
        # Update ControlIp and LocalHostIp to use Docker gateway
        old_control_ip = vehicle_config.get("ControlIp", "127.0.0.1")
        old_local_ip = vehicle_config.get("LocalHostIp", "127.0.0.1")
        
        vehicle_config["ControlIp"] = gateway_ip
        vehicle_config["LocalHostIp"] = gateway_ip
        
        print(f"  ControlIp: {old_control_ip} → {gateway_ip}")
        print(f"  LocalHostIp: {old_local_ip} → {gateway_ip}")
        print(f"  TcpPort: {vehicle_config.get('TcpPort', 'unchanged')}")
        print(f"  ControlPortLocal: {vehicle_config.get('ControlPortLocal', 'unchanged')}")
        print(f"  ControlPortRemote: {vehicle_config.get('ControlPortRemote', 'unchanged')}")
    
    # Write updated settings
    with open(settings_path, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"\n✅ Settings updated successfully!")
    return True

def verify_configuration():
    """Verify the configuration looks correct"""
    print("\n🔍 Verification:")
    print("=" * 40)
    print("Network Flow:")
    print("  1. AirSim (host) → connects to → PX4 containers via TCP (4561+)")
    print("  2. PX4 containers → connect back to → AirSim via MAVLink UDP")
    print("  3. MAVLink traffic flows through Docker gateway (172.25.0.1)")
    print("  4. QGroundControl connects to MAVLink ports (14550+)")
    
    print("\nExpected Result:")
    print("  ✅ GPS home location should be established")
    print("  ✅ armDisarm commands should work")
    print("  ✅ MAVLink connection stable")

def main():
    print("🔧 AirSim Network Settings Fix Tool")
    print("=" * 50)
    
    # Find settings.json
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    
    if not settings_path.exists():
        print("❌ settings.json not found!")
        print(f"Expected location: {settings_path}")
        print("Please generate settings first with: python generate_settings.py N")
        return
    
    print(f"📍 Found settings.json: {settings_path}")
    
    # Get Docker gateway IP
    gateway_ip = get_docker_gateway_ip()
    
    # Create backup
    backup_path = backup_settings(settings_path)
    
    # Fix the settings
    if fix_settings(settings_path, gateway_ip):
        print(f"\n🎉 Network configuration fixed!")
        
        # Provide next steps
        print("\n📋 Next Steps:")
        print("1. Restart Docker containers:")
        print("   docker-compose -f docker-compose.generated.yml down")
        print("   docker-compose -f docker-compose.generated.yml up -d")
        print("")
        print("2. Or use the launcher:")
        print("   .\\launch_generated.bat")
        print("")
        print("3. Start AirSim and test armDisarm functionality")
        print("")
        print("4. Check GPS status:")
        print("   python diagnose_gps_docker.py")
        
        # Show verification info
        verify_configuration()
        
        if backup_path:
            print(f"\n💾 Original settings backed up to: {backup_path}")
    
    else:
        print("❌ Failed to fix settings")

if __name__ == "__main__":
    main() 