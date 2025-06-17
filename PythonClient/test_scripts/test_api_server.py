#!/usr/bin/env python3
"""
Test script for the Mission API Server.
This allows you to test all API endpoints without the Streamlit dashboard.
"""

import requests
import json
import time

API_BASE = "http://localhost:8000"

def test_connection():
    """Test if API server is running"""
    try:
        response = requests.get(f"{API_BASE}/api/status", timeout=5)
        print("✓ API Server is running")
        return True
    except requests.exceptions.RequestException as e:
        print(f"✗ API Server not accessible: {e}")
        print("Make sure to start the API server first with: .\\PythonClient\\start_api.bat")
        return False

def get_available_missions():
    """Get list of available missions"""
    print("\n=== Available Missions ===")
    try:
        response = requests.get(f"{API_BASE}/api/missions")
        missions = response.json()
        
        for mission_id, mission in missions.items():
            print(f"\n{mission_id.upper()}: {mission['name']}")
            print(f"  Description: {mission['description']}")
            print(f"  Script: {mission['script']}")
            print("  Parameters:")
            for param, config in mission['parameters'].items():
                print(f"    --{param}: {config['type']} (default: {config['default']})")
        
        return missions
    except Exception as e:
        print(f"Error getting missions: {e}")
        return {}

def get_status():
    """Get current mission status"""
    print("\n=== Mission Status ===")
    try:
        response = requests.get(f"{API_BASE}/api/status")
        status = response.json()
        
        print(f"Active: {status['active']}")
        print(f"Type: {status['type']}")
        print(f"Phase: {status['phase']}")
        print(f"Progress: {status['progress']}%")
        print(f"Message: {status['message']}")
        
        if status['start_time']:
            elapsed = time.time() - status['start_time']
            print(f"Running for: {elapsed:.1f} seconds")
        
        return status
    except Exception as e:
        print(f"Error getting status: {e}")
        return {}

def launch_mission(mission_type, parameters=None):
    """Launch a mission"""
    print(f"\n=== Launching {mission_type.upper()} Mission ===")
    
    payload = {
        "mission_type": mission_type,
        "parameters": parameters or {}
    }
    
    print(f"Payload: {json.dumps(payload, indent=2)}")
    
    try:
        response = requests.post(
            f"{API_BASE}/api/launch",
            json=payload,
            headers={"Content-Type": "application/json"}
        )
        
        result = response.json()
        
        if response.status_code == 200:
            print(f"✓ {result['message']}")
            print(f"Process ID: {result.get('pid', 'Unknown')}")
            return True
        else:
            print(f"✗ Launch failed: {result}")
            return False
            
    except Exception as e:
        print(f"Error launching mission: {e}")
        return False

def stop_mission():
    """Stop current mission"""
    print("\n=== Stopping Mission ===")
    try:
        response = requests.post(f"{API_BASE}/api/stop")
        result = response.json()
        
        if result.get('success'):
            print(f"✓ {result['message']}")
        else:
            print(f"✗ {result['message']}")
            
        return result.get('success', False)
        
    except Exception as e:
        print(f"Error stopping mission: {e}")
        return False

def monitor_mission(duration=30):
    """Monitor mission progress for specified duration"""
    print(f"\n=== Monitoring Mission (for {duration} seconds) ===")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        status = get_status()
        
        if not status.get('active'):
            print("Mission completed or stopped")
            break
            
        time.sleep(2)  # Check every 2 seconds
    
    print("Monitoring complete")

def interactive_menu():
    """Interactive menu for testing"""
    while True:
        print("\n" + "="*50)
        print("MISSION API SERVER TEST MENU")
        print("="*50)
        print("1. Test Connection")
        print("2. Get Available Missions")
        print("3. Get Status")
        print("4. Launch Box Mission (default params)")
        print("5. Launch Box Mission (custom params)")
        print("6. Launch Spiral Mission")
        print("7. Launch Grid Mission")
        print("8. Stop Mission")
        print("9. Monitor Mission")
        print("10. Launch Box Mission (no orbits - simple pattern)")
        print("0. Exit")
        
        choice = input("\nEnter choice (0-10): ").strip()
        
        if choice == "0":
            break
        elif choice == "1":
            test_connection()
        elif choice == "2":
            get_available_missions()
        elif choice == "3":
            get_status()
        elif choice == "4":
            launch_mission("box")
        elif choice == "5":
            params = {
                "box_size": 200,
                "altitude": 15,
                "speed": 4,
                "orbit_radius": 30,
                "photos_per_orbit": 4
            }
            launch_mission("box", params)
        elif choice == "6":
            params = {
                "max_radius": 300,
                "altitude": 20,
                "speed": 6,
                "spiral_spacing": 20
            }
            launch_mission("spiral", params)
        elif choice == "7":
            params = {
                "width": 400,
                "height": 300,
                "altitude": 25,
                "grid_spacing": 40
            }
            launch_mission("grid", params)
        elif choice == "8":
            stop_mission()
        elif choice == "9":
            duration = input("Monitor duration in seconds (default 30): ").strip()
            duration = int(duration) if duration.isdigit() else 30
            monitor_mission(duration)
        elif choice == "10":
            params = {
                "box_size": 200,
                "altitude": 15,
                "speed": 4,
                "orbit_radius": 30,
                "photos_per_orbit": 4,
                "enable_orbits": False
            }
            launch_mission("box", params)
        else:
            print("Invalid choice")

def main():
    """Main function"""
    print("Mission API Server Tester")
    print("=" * 40)
    
    # Test connection first
    if not test_connection():
        return
    
    # Run interactive menu
    interactive_menu()
    
    print("\nTesting complete!")

if __name__ == "__main__":
    main() 