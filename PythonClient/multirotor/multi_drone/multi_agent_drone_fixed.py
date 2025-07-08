import cv2
import numpy as np
import os
import pprint
# Add parent directories to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import tempfile
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim

def wait_for_gps_and_arm(client, vehicle_name, max_retries=10, delay=3):
    """Wait for GPS home location and then arm the drone"""
    print(f"🛰️ Waiting for {vehicle_name} to establish GPS home location...")
    
    for attempt in range(max_retries):
        try:
            client.armDisarm(True, vehicle_name)
            print(f"✅ {vehicle_name} armed successfully!")
            return True
        except Exception as e:
            if "GPS home location" in str(e):
                print(f"⏳ {vehicle_name} - Attempt {attempt + 1}/{max_retries}, waiting {delay}s for GPS home...")
                time.sleep(delay)
            else:
                print(f"❌ {vehicle_name} error: {e}")
                return False
    
    print(f"❌ {vehicle_name} failed to establish GPS home location after {max_retries} attempts")
    return False

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable API control first
print("🔧 Enabling API control...")
client.enableApiControl(True, "PX4_Drone1")
client.enableApiControl(True, "PX4_Drone2")

# Wait a moment for initialization
print("⏳ Waiting 5 seconds for PX4 initialization...")
time.sleep(5)

# Try to arm with GPS home location handling
print("🔧 Arming drones with GPS home location handling...")
if not wait_for_gps_and_arm(client, "PX4_Drone1"):
    print("❌ Failed to arm PX4_Drone1")
    exit(1)

if not wait_for_gps_and_arm(client, "PX4_Drone2"):
    print("❌ Failed to arm PX4_Drone2")
    exit(1)

print("✅ Both drones armed successfully!")

print("Press any key to takeoff...")
input()

f1 = client.takeoffAsync(vehicle_name="PX4_Drone1")
f2 = client.takeoffAsync(vehicle_name="PX4_Drone2")
f1.join()
f2.join()

state1 = client.getMultirotorState(vehicle_name="PX4_Drone1")
s = pprint.pformat(state1)
print("state: %s" % s)
state2 = client.getMultirotorState(vehicle_name="PX4_Drone2")
s = pprint.pformat(state2)
print("state: %s" % s)

print("Press any key to move vehicles...")
input()

f1 = client.moveToPositionAsync(-5, 5, -10, 5, vehicle_name="PX4_Drone1")
f2 = client.moveToPositionAsync(5, -5, -10, 5, vehicle_name="PX4_Drone2")
f1.join()
f2.join()

print("Press any key to take images...")
input()

# get camera images from the car
responses1 = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="PX4_Drone1")
print('Drone1: Retrieved images: %d' % len(responses1))
responses2 = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="PX4_Drone2")  #scene vision image in uncompressed RGB array
print('Drone2: Retrieved images: %d' % len(responses2))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses1 + responses2):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

print("Press any key to reset to original state...")
input()

client.armDisarm(False, "PX4_Drone1")
client.armDisarm(False, "PX4_Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "PX4_Drone1")
client.enableApiControl(False, "PX4_Drone2") 