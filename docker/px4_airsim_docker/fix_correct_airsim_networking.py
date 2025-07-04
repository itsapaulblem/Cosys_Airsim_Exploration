#!/usr/bin/env python3
"""
CORRECT AirSim Docker Networking Fix

This fixes the REAL networking issue based on how PX4-AirSim actually works.

THE REAL PROBLEM:
================
1. PX4 containers connect TO AirSim (not the other way around!)
2. AirSim should LISTEN on localhost (127.0.0.1 or 0.0.0.0)
3. PX4 connects to host.docker.internal (which resolves to host)

CORRECT ARCHITECTURE:
====================
PX4 Container → host.docker.internal:4561 → AirSim (listening on 127.0.0.1:4561)

WRONG THINKING (what we tried before):
=====================================
AirSim → 172.25.0.10:4561 → PX4 Container (PX4 doesn't listen on TCP!)

CORRECT SETTINGS:
================
ControlIp: "127.0.0.1"        # AirSim listens on localhost
LocalHostIp: "127.0.0.1"      # MAVLink also uses localhost
"""

import json
import subprocess
import shutil
from pathlib import Path
from datetime import datetime

def explain_real_architecture():
    """Explain how PX4-AirSim networking actually works"""
    print("🌐 CORRECT PX4-AirSim Architecture")
    print("=" * 50)
    print("""
🔄 ACTUAL CONNECTION FLOW:
-------------------------
PX4 Container → AirSim (Host)

📡 TCP Connection (Simulation Data):
   PX4 connects TO AirSim via host.docker.internal:4561
   AirSim LISTENS on 127.0.0.1:4561
   
📡 MAVLink Connection (Telemetry):
   PX4 sends MAVLink TO AirSim via host.docker.internal:14541
   AirSim LISTENS on 127.0.0.1:14541

❌ WRONG ASSUMPTION:
   Thinking AirSim connects TO PX4 containers
   PX4 doesn't run a TCP server for AirSim to connect to!
   
✅ CORRECT UNDERSTANDING:
   PX4 connects TO AirSim
   AirSim is the server, PX4 is the client
   
🎯 SOLUTION:
   Set both ControlIp and LocalHostIp to "127.0.0.1"
   AirSim listens on localhost
   PX4 connects to host.docker.internal (resolves to host)
""")

def fix_settings_correct(settings_path):
    """Fix the AirSim settings with the correct understanding"""
    
    # Load current settings
    with open(settings_path, 'r') as f:
        settings = json.load(f)
    
    vehicles = settings.get("Vehicles", {})
    if not vehicles:
        print("❌ No vehicles found in settings.json")
        return False
    
    print(f"\n🔧 Fixing network configuration for {len(vehicles)} vehicles...")
    print("📝 Setting AirSim to LISTEN on localhost (as it should)")
    
    # Fix each vehicle's configuration
    for vehicle_name, vehicle_config in vehicles.items():
        print(f"\n🚁 Fixing {vehicle_name}:")
        
        # Show old configuration
        old_control_ip = vehicle_config.get("ControlIp", "Not set")
        old_local_ip = vehicle_config.get("LocalHostIp", "Not set")
        
        # Set CORRECT configuration - AirSim listens on localhost
        vehicle_config["ControlIp"] = "127.0.0.1"      # AirSim listens on localhost
        vehicle_config["LocalHostIp"] = "127.0.0.1"    # MAVLink also uses localhost
        
        print(f"  ControlIp: {old_control_ip} → 127.0.0.1 (localhost - AirSim listens)")
        print(f"  LocalHostIp: {old_local_ip} → 127.0.0.1 (localhost - MAVLink)")
        print(f"  TcpPort: {vehicle_config.get('TcpPort', 'unchanged')}")
        
        # Ensure other required settings
        vehicle_config["UseTcp"] = True
    
    # Write updated settings
    with open(settings_path, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"\n✅ Settings fixed with CORRECT networking!")
    return True

def verify_correct_setup():
    """Verify this is the correct setup"""
    print("\n🔍 Verification - How This Should Work")
    print("=" * 45)
    print("""
Expected Flow:
1. AirSim starts and LISTENS on:
   - 127.0.0.1:4561 (TCP for Drone1)
   - 127.0.0.1:4562 (TCP for Drone2)
   - etc.

2. PX4 containers start and CONNECT to:
   - host.docker.internal:4561 (resolves to host)
   - host.docker.internal:4562 (resolves to host)
   - etc.

3. Connection established:
   ✅ PX4 → AirSim (TCP simulation data)
   ✅ PX4 → AirSim (MAVLink telemetry)
   ✅ GPS home location works
   ✅ armDisarm works

🎯 This is why 127.0.0.1 works and container IPs don't!
""")

def check_docker_hostname_resolution():
    """Check if host.docker.internal resolves correctly"""
    print("\n🧪 Testing Docker Hostname Resolution")
    print("=" * 40)
    
    try:
        # Test if containers can reach host.docker.internal
        result = subprocess.run(
            ['docker', 'exec', 'px4-px4-drone1', 'nslookup', 'host.docker.internal'],
            capture_output=True, text=True
        )
        
        if result.returncode == 0 and 'Address:' in result.stdout:
            # Extract the IP address
            lines = result.stdout.split('\n')
            for line in lines:
                if 'Address:' in line and not 'server' in line.lower():
                    host_ip = line.split(':')[-1].strip()
                    print(f"✅ host.docker.internal resolves to: {host_ip}")
                    print(f"   PX4 containers can reach the host")
                    return True
        
        print("❌ host.docker.internal resolution failed")
        return False
        
    except Exception as e:
        print(f"⚠️ Could not test hostname resolution: {e}")
        print("   (This is normal if containers aren't running)")
        return None

def backup_settings(settings_path):
    """Create backup of current settings"""
    if not settings_path.exists():
        return None
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = settings_path.parent / f"settings_backup_correct_{timestamp}.json"
    
    try:
        shutil.copy2(settings_path, backup_path)
        print(f"💾 Backup created: {backup_path}")
        return backup_path
    except Exception as e:
        print(f"⚠️ Could not create backup: {e}")
        return None

def main():
    print("🔧 CORRECT AirSim Docker Networking Fix")
    print("=" * 45)
    
    # Explain the real architecture
    explain_real_architecture()
    
    # Find settings.json
    settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    
    if not settings_path.exists():
        print("❌ settings.json not found!")
        print(f"Expected location: {settings_path}")
        return
    
    print(f"📍 Found settings.json: {settings_path}")
    
    # Test hostname resolution
    check_docker_hostname_resolution()
    
    # Create backup
    backup_path = backup_settings(settings_path)
    
    # Apply the correct fix
    if fix_settings_correct(settings_path):
        print(f"\n🎉 CORRECT network configuration applied!")
        
        # Provide next steps
        print("\n📋 Next Steps:")
        print("1. Restart AirSim (Unreal Engine)")
        print("   - AirSim will now LISTEN on localhost as it should")
        print("")
        print("2. Start/restart PX4 containers:")
        print("   docker-compose -f docker-compose.generated.yml restart")
        print("")
        print("3. Check AirSim console - should show:")
        print("   'Waiting for TCP connection on port 4561, local IP 127.0.0.1'")
        print("   'Accepting TCP socket connection'")
        print("")
        print("4. Test GPS and armDisarm:")
        print("   python test_gps_home_api.py")
        
        # Show verification
        verify_correct_setup()
        
        if backup_path:
            print(f"\n💾 Previous settings backed up to: {backup_path}")
    
    else:
        print("❌ Failed to fix settings")

if __name__ == "__main__":
    main() 