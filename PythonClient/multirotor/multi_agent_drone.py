import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim


# Use below in settings.json with Blocks environment
# This script now dynamically discovers all available multirotor drones
"""
Example settings.json:
{
	"SeeDocsAt": "https://cosys-lab.github.io/Cosys-AirSim/settings/",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 8, "Y": 0, "Z": -2
		},
		"PX4_Drone3": {
		  "VehicleType": "PX4Multirotor",
		  "X": -4, "Y": 4, "Z": -2
		}
		// Add as many drones as needed - script will discover them automatically
    }
}
"""

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Get list of all available drones
vehicles = client.listVehicles()
print(f"Found {len(vehicles)} vehicles: {vehicles}")

# Filter for multirotor vehicles only
drone_names = []
for vehicle_name in vehicles:
    try:
        # Try to get multirotor state - if this works, it's a multirotor
        client.getMultirotorState(vehicle_name=vehicle_name)
        drone_names.append(vehicle_name)
        print(f"✓ {vehicle_name} is a multirotor drone")
    except:
        print(f"✗ {vehicle_name} is not a multirotor (skipping)")

if not drone_names:
    print("No multirotor drones found! Please check your settings.json configuration.")
    sys.exit(1)

print(f"\nPreparing {len(drone_names)} drones: {drone_names}")

# Enable API control and arm all drones
for drone_name in drone_names:
    print(f"Enabling API control for {drone_name}...")
    client.enableApiControl(True, drone_name)
    print(f"Arming {drone_name}...")
    client.armDisarm(True, drone_name)

airsim.wait_key('Press any key to takeoff drones one by one')

# Take off drones sequentially
for i, drone_name in enumerate(drone_names):
    print(f"Taking off drone {i+1}/{len(drone_names)}: {drone_name}")
    takeoff_future = client.takeoffAsync(vehicle_name=drone_name)
    takeoff_future.join()  # Wait for this drone to complete takeoff before starting next
    print(f"✓ {drone_name} takeoff completed")

print("\nAll drones have taken off successfully!")

# Get and display states for all drones
for drone_name in drone_names:
    state = client.getMultirotorState(vehicle_name=drone_name)
    s = pprint.pformat(state)
    print(f"State for {drone_name}: {s}")

airsim.wait_key('Press any key to move vehicles')

# Move drones to different positions
move_futures = []
positions = [(-5, 5, -10), (5, -5, -10), (-8, -8, -10), (8, 8, -10), (0, 10, -10)]  # Define various positions

for i, drone_name in enumerate(drone_names):
    # Use different positions for each drone, cycling through available positions
    pos_x, pos_y, pos_z = positions[i % len(positions)]
    print(f"Moving {drone_name} to position ({pos_x}, {pos_y}, {pos_z})")
    move_future = client.moveToPositionAsync(pos_x, pos_y, pos_z, 5, vehicle_name=drone_name)
    move_futures.append(move_future)

# Wait for all movements to complete
for future in move_futures:
    future.join()

print("All drones have reached their target positions!")

# airsim.wait_key('Press any key to take images')

# # Get camera images from all drones
# all_responses = []
# for drone_name in drone_names:
#     print(f"Capturing images from {drone_name}...")
#     responses = client.simGetImages([
#         airsim.ImageRequest("0", airsim.ImageType.DepthVis),  # depth visualization image
#         airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)  # scene vision image in uncompressed RGB array
#     ], vehicle_name=drone_name)
#     print(f'{drone_name}: Retrieved images: {len(responses)}')
#     all_responses.extend(responses)

# tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
# print ("Saving images to %s" % tmp_dir)
# try:
#     os.makedirs(tmp_dir)
# except OSError:
#     if not os.path.isdir(tmp_dir):
#         raise

# for idx, response in enumerate(all_responses):

#     filename = os.path.join(tmp_dir, str(idx))

#     if response.pixels_as_float:
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
#         airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
#     elif response.compress: #png format
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#         airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
#     else: #uncompressed array
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#         img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) #get numpy array
#         img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3
#         cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')

# Disarm all drones
for drone_name in drone_names:
    print(f"Disarming {drone_name}...")
    client.armDisarm(False, drone_name)

client.reset()

# that's enough fun for now. let's quit cleanly
for drone_name in drone_names:
    print(f"Disabling API control for {drone_name}...")
    client.enableApiControl(False, drone_name)

print(f"Successfully completed operations for {len(drone_names)} drones!")


