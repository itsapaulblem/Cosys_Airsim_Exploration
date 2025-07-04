#!/usr/bin/env python3
"""
Fix AirSim Docker Networking Configuration

This script fixes the networking configuration between AirSim (host) and PX4 containers.

NETWORKING EXPLANATION:
=======================
The key insight is that there are TWO different network connections:

1. AirSim → PX4 (TCP): AirSim (on host) connects TO PX4 container
   - AirSim needs the container's IP address to connect TO it
   - Use container IPs: 172.25.0.10, 172.25.0.11, etc.

2. PX4 → AirSim (MAVLink): PX4 container connects back TO AirSim (on host)  
   - PX4 needs the host IP address to send MAVLink messages TO AirSim
   - Use host.docker.internal or gateway IP: 172.25.0.1

CORRECT SETTINGS:
================
ControlIp: "172.25.0.10"      # Container IP (AirSim connects TO this)
LocalHostIp: "172.25.0.1"     # Gateway IP (PX4 connects back TO this)

WRONG SETTINGS (what causes your issue):
=======================================
ControlIp: "172.25.0.1"       # Gateway IP - AirSim can't find PX4 here!
LocalHostIp: "172.25.0.1"     # This part is correct
"""

import json
import subprocess
import shutil
from pathlib import Path
from datetime import datetime

def get_docker_network_info():
    """Get Docker network information"""
    try:
        result = subprocess.run(
            ['docker', 'network', 'inspect', 'airsim-network'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            network_info = json.loads(result.stdout)
            gateway = network_info[0]['IPAM']['Config'][0]['Gateway']
            subnet = network_info[0]['IPAM']['Config'][0]['Subnet']
            print(f"✅ Docker network found:")
            print(f"   Gateway: {gateway}")
            print(f"   Subnet: {subnet}")
            return gateway, subnet
    except Exception as e:
        print(f"⚠️ Could not get Docker network info: {e}")
    
    # Return defaults
    return "172.25.0.1", "172.25.0.0/16"

def explain_networking():
    """Explain the networking concepts"""
    print("🌐 AirSim-Docker Networking Explanation")
    print("=" * 50)
    print("""
🔄 BIDIRECTIONAL COMMUNICATION:
------------------------------
AirSim (host) ←→ PX4 (container)

📡 CONNECTION 1: AirSim → PX4 (TCP)
   Purpose: AirSim sends commands to PX4
   Direction: Host → Container  
   Setting: ControlIp = Container IP (172.25.0.10)
   
📡 CONNECTION 2: PX4 → AirSim (MAVLink UDP)
   Purpose: PX4 sends telemetry/status to AirSim
   Direction: Container → Host
   Setting: LocalHostIp = Gateway IP (172.25.0.1)

❌ COMMON MISTAKE:
   Setting ControlIp to gateway IP (172.25.0.1)
   This makes AirSim try to connect to the gateway instead of the container!
   
✅ CORRECT CONFIGURATION:
   ControlIp: "172.25.0.10"    # Where AirSim finds PX4
   LocalHostIp: "172.25.0.1"   # Where PX4 finds AirSim
""")

def fix_settings_file(settings_path, gateway_ip):
    """Fix the AirSim settings.json file"""
    
    # Load current settings
    with open(settings_path, 'r') as f:
        settings = json.load(f)
    
    vehicles = settings.get("Vehicles", {})
    if not vehicles:
        print("❌ No vehicles found in settings.json")
        return False
    
    print(f"\n🔧 Fixing network configuration for {len(vehicles)} vehicles...")
    
    # Fix each vehicle's configuration
    for idx, (vehicle_name, vehicle_config) in enumerate(vehicles.items(), 1):
        print(f"\n🚁 Fixing {vehicle_name}:")
        
        # Calculate container IP (172.25.0.10, 172.25.0.11, etc.)
        container_ip = f"172.25.0.{9 + idx}"
        
        # Show old configuration
        old_control_ip = vehicle_config.get("ControlIp", "Not set")
        old_local_ip = vehicle_config.get("LocalHostIp", "Not set")
        
        # Set correct configuration
        vehicle_config["ControlIp"] = container_ip      # Container IP for TCP
        vehicle_config["LocalHostIp"] = gateway_ip      # Gateway IP for MAVLink
        
        print(f"  ControlIp: {old_control_ip} → {container_ip} (container)")
        print(f"  LocalHostIp: {old_local_ip} → {gateway_ip} (gateway)")
        print(f"  TcpPort: {vehicle_config.get('TcpPort', 'unchanged')}")
        
        # Ensure other required settings
        vehicle_config["UseTcp"] = True
        if "ControlPortLocal" not in vehicle_config:
            vehicle_config["ControlPortLocal"] = 14540 + idx
        if "ControlPortRemote" not in vehicle_config:
            vehicle_config["ControlPortRemote"] = 14580 + idx
    
    # Write updated settings
    with open(settings_path, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"\n✅ Settings fixed successfully!")
    return True

def backup_settings(settings_path):
    """Create backup of current settings"""
    if not settings_path.exists():
        return None
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = settings_path.parent / f"settings_backup_{timestamp}.json"
    
    try:
        shutil.copy2(settings_path, backup_path)
        print(f"💾 Backup created: {backup_path}")
        return backup_path
    except Exception as e:
        print(f"⚠️ Could not create backup: {e}")
        return None

def verify_fix():
    """Verify the configuration should work"""
    print("\n🔍 Configuration Verification")
    print("=" * 30)
    print("""
Expected Network Flow:
1. AirSim (host) connects to 172.25.0.10:4561 (PX4 container 1)
2. AirSim (host) connects to 172.25.0.11:4562 (PX4 container 2)
3. PX4 containers send MAVLink to 172.25.0.1 (gateway → host)
4. GPS data flows: PX4 → AirSim → armDisarm works!

🎯 This should fix your "Vehicle does not have a valid GPS home location" error!
""")

def test_connectivity():
    """Test if containers are accessible on their IPs"""
    print("\n🧪 Testing Container Connectivity")
    print("=" * 35)
    
    # Test if containers are running
    try:
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=px4-', '--format', 'table {{.Names}}\t{{.Status}}'],
            capture_output=True, text=True
        )
        
        if result.returncode == 0:
            lines = result.stdout.strip().split('\n')
            if len(lines) > 1:  # Skip header line
                print("🐳 Running PX4 containers:")
                for line in lines[1:]:
                    if line.strip():
                        print(f"   {line}")
            else:
                print("❌ No PX4 containers running")
                print("💡 Start containers first: .\\launch_generated.bat")
                return False
        
        return True
        
    except Exception as e:
        print(f"❌ Error checking containers: {e}")
        return False

def main():
    print("🔧 AirSim Docker Network Configuration Fix")
    print("=" * 45)
    
    # Explain the networking concepts first
    explain_networking()
    
    # Find settings.json
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    
    if not settings_path.exists():
        print("❌ settings.json not found!")
        print(f"Expected location: {settings_path}")
        print("Please make sure AirSim has created settings.json first")
        return
    
    print(f"📍 Found settings.json: {settings_path}")
    
    # Get Docker network info
    gateway_ip, subnet = get_docker_network_info()
    
    # Test container connectivity
    if not test_connectivity():
        print("\n⚠️ Continue anyway? The fix will still be applied. (y/n): ", end="")
        if input().lower() != 'y':
            return
    
    # Create backup
    backup_path = backup_settings(settings_path)
    
    # Fix the settings
    if fix_settings_file(settings_path, gateway_ip):
        print(f"\n🎉 Network configuration fixed!")
        
        # Provide next steps
        print("\n📋 Next Steps:")
        print("1. Restart AirSim (Unreal Engine)")
        print("   - AirSim needs to reload the new settings")
        print("")
        print("2. Test the configuration:")
        print("   python test_gps_home_api.py")
        print("")
        print("3. Try armDisarm:")
        print("   python diagnose_msgpackrpc.py")
        print("")
        print("4. If it works, try your orbit script!")
        
        # Show verification
        verify_fix()
        
        if backup_path:
            print(f"\n💾 Original settings backed up to: {backup_path}")
    
    else:
        print("❌ Failed to fix settings")

if __name__ == "__main__":
    main() 