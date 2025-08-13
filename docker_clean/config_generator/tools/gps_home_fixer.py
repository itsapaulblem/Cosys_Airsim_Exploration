#!/usr/bin/env python3
"""
GPS Home Location Fixer for AirSim/PX4 Multi-Drone Setup
Fixes the common "no valid GPS home location" error
"""

import os
import json
import shutil
import time
from pathlib import Path
import subprocess

def find_airsim_settings_path():
    """Find the correct AirSim settings path for the current system"""
    possible_paths = [
        Path.home() / "Documents" / "AirSim" / "settings.json",
        Path.home() / "Documents" / "Cosys-AirSim" / "settings.json", 
        Path("./settings.json"),
        Path("../settings.json"),
        Path("/tmp/AirSim/settings.json")
    ]
    
    for path in possible_paths:
        if path.parent.exists():
            return path
    
    # Default to creating in Documents/AirSim
    default_path = Path.home() / "Documents" / "AirSim" / "settings.json"
    default_path.parent.mkdir(parents=True, exist_ok=True)
    return default_path

def check_docker_containers():
    """Check if PX4 Docker containers are running"""
    try:
        result = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], 
                              capture_output=True, text=True, check=True)
        containers = result.stdout.strip().split('\n')
        px4_containers = [c for c in containers if 'px4' in c.lower()]
        return px4_containers
    except subprocess.CalledProcessError:
        return []

def copy_settings_file():
    """Copy the multi-drone settings.json to the correct AirSim location"""
    # Look for settings in docker_clean directories
    possible_sources = [
        Path("/mnt/l/Cosys-AirSim/docker_clean/multi_drone/settings.json"),
        Path("/mnt/l/Cosys-AirSim/docker_clean/config_generator/tools/settings.json"),
        Path("./settings.json"),
        Path("../multi_drone/settings.json")
    ]
    
    source_file = None
    for source in possible_sources:
        if source.exists():
            source_file = source
            break
    
    if not source_file:
        print("❌ Could not find a source settings.json file")
        return False
    
    target_path = find_airsim_settings_path()
    
    try:
        shutil.copy2(source_file, target_path)
        print(f"✅ Copied settings from: {source_file}")
        print(f"✅ To AirSim location: {target_path}")
        return True
    except Exception as e:
        print(f"❌ Failed to copy settings: {e}")
        return False

def wait_for_gps_lock(vehicle_name="PX4_Drone1", timeout=30):
    """Wait for GPS lock on a specific vehicle"""
    print(f"🛰️ Waiting for GPS lock on {vehicle_name}...")
    
    # This is a placeholder - in practice you'd connect to AirSim API
    # and check GPS status. For now, we'll just wait a bit.
    for i in range(timeout):
        print(f"   Waiting... {i+1}/{timeout}s", end='\r')
        time.sleep(1)
        
        # In a real implementation, you'd check:
        # client = airsim.MultirotorClient()
        # gps_data = client.getGpsData(vehicle_name)
        # if gps_data.gnss.fix_type >= 3:  # 3D fix
        #     return True
    
    print(f"\n⚠️  Timeout waiting for GPS lock")
    return False

def force_gps_home_set(containers):
    """Send commands to PX4 containers to force GPS home setting"""
    print("🔧 Forcing GPS home location in PX4 containers...")
    
    for container in containers:
        print(f"   Setting GPS home in {container}...")
        try:
            # Force set home position in PX4
            cmd = [
                "docker", "exec", container, 
                "/bin/bash", "-c", 
                "echo 'commander set_home 47.641468 -122.140165 122.0' | nc localhost 5760 || true"
            ]
            subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            # Also try via MAVLink command
            cmd2 = [
                "docker", "exec", container,
                "/bin/bash", "-c",
                "echo 'param set COM_ARM_WO_GPS 1' | nc localhost 5760 || true"  
            ]
            subprocess.run(cmd2, capture_output=True, text=True, timeout=5)
            
        except Exception as e:
            print(f"   ⚠️  Could not set GPS home in {container}: {e}")
        
    print("✅ GPS home setting commands sent to all containers")

def create_gps_test_script():
    """Create a simple GPS test script"""
    script_content = '''#!/usr/bin/env python3
"""
Quick GPS status test for AirSim multi-drone setup
"""
import airsim
import time

def test_gps_status():
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        vehicles = client.listVehicles()
        print(f"Found vehicles: {vehicles}")
        
        for vehicle in vehicles:
            try:
                gps_data = client.getGpsData(vehicle_name=vehicle)
                print(f"\\n{vehicle} GPS Status:")
                print(f"  Fix Type: {gps_data.gnss.fix_type}")
                print(f"  Latitude: {gps_data.gnss.geo_point.latitude}")
                print(f"  Longitude: {gps_data.gnss.geo_point.longitude}")
                print(f"  Altitude: {gps_data.gnss.geo_point.altitude}")
                print(f"  Velocity: {gps_data.gnss.velocity}")
                
                if gps_data.gnss.fix_type >= 3:
                    print(f"  ✅ {vehicle} has 3D GPS fix")
                else:
                    print(f"  ⚠️  {vehicle} does not have GPS fix")
                    
            except Exception as e:
                print(f"  ❌ Error getting GPS data for {vehicle}: {e}")
                
    except Exception as e:
        print(f"❌ Could not connect to AirSim: {e}")
        print("Make sure AirSim is running!")

if __name__ == "__main__":
    test_gps_status()
'''
    
    script_path = Path("./test_gps_status.py")
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    script_path.chmod(0o755)
    print(f"✅ Created GPS test script: {script_path}")
    return script_path

def main():
    print("🚁 GPS Home Location Fixer for AirSim/PX4")
    print("=" * 50)
    
    # Step 1: Check Docker containers
    print("\n1. 🐳 Checking Docker containers...")
    containers = check_docker_containers()
    if containers:
        print(f"✅ Found PX4 containers: {', '.join(containers)}")
    else:
        print("❌ No PX4 containers found. Start your Docker containers first!")
        return 1
    
    # Step 2: Copy settings file
    print("\n2. 📄 Copying settings.json to AirSim...")
    if not copy_settings_file():
        print("⚠️  Could not copy settings file. You may need to do this manually.")
    
    # Step 3: Force GPS home setting in containers
    print("\n3. 🛰️ Setting GPS home in PX4 containers...")
    force_gps_home_set(containers)
    
    # Step 4: Create test script
    print("\n4. 📝 Creating GPS test script...")
    test_script = create_gps_test_script()
    
    # Step 5: Instructions
    print("\n" + "=" * 50)
    print("🎯 NEXT STEPS:")
    print("=" * 50)
    print("1. ✈️  Start AirSim/Unreal Engine now")
    print("2. ⏳ Wait 10-15 seconds for connections to establish")
    print(f"3. 🧪 Run GPS test: python {test_script}")
    print("4. 🚁 If GPS is working, try your mission script again")
    print()
    print("💡 TROUBLESHOOTING:")
    print("- If still no GPS: restart Docker containers")
    print("- Check container logs: docker logs px4-drone1")
    print("- Verify AirSim is using the correct settings.json")
    print()
    print("🔧 MANUAL GPS SETTING:")
    print("If automatic fixing doesn't work, try this in your Python script:")
    print("  client.simSetHome(airsim.GeoPoint(47.641468, -122.140165, 122.0))")
    
    return 0

if __name__ == "__main__":
    exit(main())