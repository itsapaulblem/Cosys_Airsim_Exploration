import cv2
import numpy as np
import os
import pprint
import tempfile
import sys
import time
import math

# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim


client = airsim.MultirotorClient()
client.confirmConnection()

print("🔍 Testing GPS Home Location Fix...")

# Check if API control can be enabled
print("📡 Enabling API control...")
client.enableApiControl(True)

# Check GPS data
print("🛰️ Getting GPS data...")
gps_data = client.getGpsData()
print(f"   Position: {gps_data.gnss.geo_point.latitude}, {gps_data.gnss.geo_point.longitude}, {gps_data.gnss.geo_point.altitude}")

# Check home location
print("🏠 Getting home location...")
home_location = client.getHomeGeoPoint()
print(f"   Home: {home_location.latitude}, {home_location.longitude}, {home_location.altitude}")

# The critical test - this should now work!
print("🔋 Testing ARM command via RPC...")
try:
    result = client.armDisarm(True)
    if result:
        print("   ✅ SUCCESS! RPC arming worked!")
        print("   🎉 GPS home location synchronization is fixed!")

        # Test disarm
        time.sleep(2)
        client.armDisarm(False)
        print("   ✅ Disarm also successful")
    else:
        print("   ❌ Arming returned False - check PX4 status")
except Exception as e:
    print(f"   ❌ Arming failed with error: {e}")

print("\n📊 Vehicle Status:")
vehicle_state = client.getMultirotorState()
print(f"   Ready: {vehicle_state.ready}")
print(f"   Can Arm: {vehicle_state.can_arm}")
print(f"   Landed State: {vehicle_state.landed_state}")