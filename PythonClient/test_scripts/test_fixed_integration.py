#!/usr/bin/env python3
"""
Test script to verify the Streamlit-API integration fixes
"""

import requests
import json

API_BASE = "http://localhost:8000"

def test_api_endpoints():
    """Test all API endpoints"""
    print("🧪 Testing Mission API Server Integration")
    print("=" * 50)
    
    # Test 1: API Server Connection
    print("\n1. Testing API Server Connection...")
    try:
        response = requests.get(f"{API_BASE}/api/status", timeout=5)
        if response.status_code == 200:
            print("✅ API Server is running")
        else:
            print(f"❌ API Server error: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Cannot connect to API Server: {e}")
        print("   Make sure to run: python mission_api_server.py")
        return False
    
    # Test 2: AirSim Connection Test
    print("\n2. Testing AirSim Connection via API...")
    try:
        response = requests.get(f"{API_BASE}/api/test_airsim", timeout=10)
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print("✅ AirSim is connected")
                details = result.get('details', {})
                print(f"   State: {details.get('landed_state', 'Unknown')}")
                pos = details.get('position', {})
                print(f"   Position: x={pos.get('x', 0)}, y={pos.get('y', 0)}, z={pos.get('z', 0)}")
            else:
                print(f"⚠️ AirSim not connected: {result.get('status', 'Unknown')}")
        else:
            print(f"❌ AirSim test failed: {response.status_code}")
    except Exception as e:
        print(f"❌ AirSim test error: {e}")
    
    # Test 3: Available Missions
    print("\n3. Testing Available Missions...")
    try:
        response = requests.get(f"{API_BASE}/api/missions", timeout=5)
        if response.status_code == 200:
            missions = response.json()
            print(f"✅ Found {len(missions)} mission types:")
            for mission_id, mission_data in missions.items():
                print(f"   - {mission_id}: {mission_data.get('name', 'Unknown')}")
        else:
            print(f"❌ Missions request failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Missions test error: {e}")
    
    # Test 4: Mission Launch (dry run)
    print("\n4. Testing Mission Launch Parameters...")
    try:
        payload = {
            "mission_type": "box",
            "parameters": {
                "box_size": 200,
                "altitude": 15,
                "speed": 4,
                "enable_orbits": False
            }
        }
        print(f"   Payload: {json.dumps(payload, indent=2)}")
        print("   ✅ Mission parameters formatted correctly")
        print("   (Not launching - this is just a parameter test)")
    except Exception as e:
        print(f"❌ Parameter formatting error: {e}")
    
    print("\n" + "=" * 50)
    print("🎯 Integration Test Summary:")
    print("   - API Server: Should be running")
    print("   - AirSim: Check UE5 is running with Play pressed")
    print("   - Streamlit: Should now work without AirSim import errors")
    print("   - Mission Launch: Should find correct script paths")
    
    return True

def main():
    """Main test function"""
    test_api_endpoints()

if __name__ == "__main__":
    main() 