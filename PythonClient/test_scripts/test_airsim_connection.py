#!/usr/bin/env python3
"""
AirSim Connection Test

Test script to verify AirSim is properly connected and working.
Run this before launching missions to ensure everything is set up correctly.
"""

import airsim
import time
import sys

def test_connection():
    """Test basic AirSim connection"""
    print(" Testing AirSim Connection...")
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        print(" Client created")
        
        # Confirm connection
        client.confirmConnection()
        print(" Connection confirmed")
        
        # Check if simulation is running
        sim_state = client.getMultirotorState()
        print(f" Simulation state: {sim_state.landed_state}")
        
        return client
        
    except Exception as e:
        print(f" Connection failed: {e}")
        print("\n Troubleshooting:")
        print("1. Make sure UE5 with AirSim is running")
        print("2. Check that AirSim settings.json is configured")
        print("3. Verify no firewall is blocking the connection")
        return None

def test_basic_controls(client):
    """Test basic drone controls"""
    print("\n Testing Basic Controls...")
    
    try:
        # Enable API control
        client.enableApiControl(True)
        print(" API control enabled")
        
        # Arm the drone
        client.armDisarm(True)
        print(" Drone armed")
        
        # Get current position
        pose = client.simGetVehiclePose()
        print(f" Current position: x={pose.position.x_val:.2f}, y={pose.position.y_val:.2f}, z={pose.position.z_val:.2f}")
        
        # Test takeoff
        print(" Testing takeoff...")
        takeoff_result = client.takeoffAsync(timeout_sec=20)
        takeoff_result.join()
        print(" Takeoff successful")
        
        # Hover for a moment
        print(" Hovering for 3 seconds...")
        time.sleep(3)
        
        # Test movement
        print(" Testing movement...")
        client.moveByVelocityAsync(1, 0, 0, 2).join()  # Move forward
        print(" Forward movement successful")
        
        client.moveByVelocityAsync(0, 0, 0, 1).join()  # Stop
        print(" Stop successful")
        
        # Land
        print(" Testing landing...")
        client.landAsync().join()
        print(" Landing successful")
        
        # Disarm
        client.armDisarm(False)
        print(" Drone disarmed")
        
        return True
        
    except Exception as e:
        print(f" Control test failed: {e}")
        return False

def test_settings():
    """Test AirSim settings"""
    print("\n Testing AirSim Settings...")
    
    try:
        # Check for settings.json
        import os
        from pathlib import Path
        
        # Common settings.json locations
        settings_paths = [
            Path.home() / "Documents" / "AirSim" / "settings.json",
            Path("settings.json"),
            Path(__file__).parent / "settings.json"
        ]
        
        settings_found = False
        for path in settings_paths:
            if path.exists():
                print(f" Found settings.json at: {path}")
                settings_found = True
                
                # Try to read and validate
                try:
                    import json
                    with open(path) as f:
                        settings = json.load(f)
                    
                    print(f" Settings loaded successfully")
                    print(f"   Sim Mode: {settings.get('SimMode', 'Not specified')}")
                    
                    if 'Vehicles' in settings:
                        vehicles = settings['Vehicles']
                        print(f"   Vehicles configured: {list(vehicles.keys())}")
                    
                except Exception as e:
                    print(f"  Settings file exists but has issues: {e}")
                
                break
        
        if not settings_found:
            print("  No settings.json found")
            print("   This might cause connection issues")
            
        return settings_found
        
    except Exception as e:
        print(f" Settings test failed: {e}")
        return False

def create_basic_settings():
    """Create a basic settings.json file"""
    print("\n Creating basic settings.json...")
    
    basic_settings = {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "Vehicles": {
            "Drone1": {
                "VehicleType": "SimpleFlight",
                "AutoCreate": True
            }
        }
    }
    
    try:
        import json
        from pathlib import Path
        
        settings_dir = Path.home() / "Documents" / "AirSim"
        settings_dir.mkdir(parents=True, exist_ok=True)
        
        settings_path = settings_dir / "settings.json"
        
        with open(settings_path, 'w') as f:
            json.dump(basic_settings, f, indent=2)
        
        print(f" Created settings.json at: {settings_path}")
        print("  Please restart UE5/AirSim for settings to take effect")
        
        return True
        
    except Exception as e:
        print(f" Failed to create settings.json: {e}")
        return False

def main():
    print(" AirSim Connection Test")
    print("=" * 50)
    
    # Test 1: Settings
    if not test_settings():
        response = input("\nSettings not found. Create basic settings.json? (y/n): ")
        if response.lower() == 'y':
            if create_basic_settings():
                print("\n  Please restart UE5/AirSim and run this test again")
                return
    
    # Test 2: Connection
    client = test_connection()
    if not client:
        print("\n Connection test failed")
        print("\n Steps to fix:")
        print("1. Start UE5 Editor")
        print("2. Open your AirSim project")
        print("3. Click Play to start simulation")
        print("4. Run this test again")
        return
    
    # Test 3: Basic controls
    response = input("\n Test basic drone controls? (y/n): ")
    if response.lower() == 'y':
        if test_basic_controls(client):
            print("\n All tests passed! AirSim is working correctly")
        else:
            print("\n Control tests failed")
    else:
        print("\n Connection test passed")
    
    print("\n Next steps:")
    print("1. If tests passed, your missions should work")
    print("2. If tests failed, fix the issues shown above")
    print("3. Try launching a mission through the API server")

if __name__ == "__main__":
    main() 