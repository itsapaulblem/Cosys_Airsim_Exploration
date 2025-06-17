import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import cosysairsim as airsim


# Use below in settings.json with Blocks environment
"""
{
	"SeeDocsAt": "https://cosys-lab.github.io/Cosys-AirSim/settings/",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": -5, "Y": -5, "Z": 0
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 5, "Y": -5, "Z": 0
		},
		"Drone3": {
		  "VehicleType": "SimpleFlight",
		  "X": 5, "Y": 5, "Z": 0
		},
		"Drone4": {
		  "VehicleType": "SimpleFlight",
		  "X": -5, "Y": 5, "Z": 0
		}
    }
}
"""

def reset_drones_to_formation(client, drone_names, expected_positions):
    """Reset all drones to their expected box formation positions."""
    print("\n🔄 Resetting drones to box formation positions...")
    
    # First, ensure all drones are disarmed and API control is disabled
    for drone in drone_names:
        try:
            client.armDisarm(False, drone)
            client.enableApiControl(False, drone)
        except:
            pass  # Ignore errors if already in this state
    
    time.sleep(1)
    
    # Re-enable API control and move to positions
    move_futures = []
    for drone in drone_names:
        try:
            client.enableApiControl(True, drone)
            client.armDisarm(True, drone)
            
            expected_x, expected_y, expected_z = expected_positions[drone]
            print(f"  Moving {drone} to formation position ({expected_x}, {expected_y}, {expected_z})")
            
            future = client.moveToPositionAsync(
                expected_x, expected_y, expected_z, 3,  # Slow speed for positioning
                vehicle_name=drone
            )
            move_futures.append((drone, future))
        except Exception as e:
            print(f"  ⚠️  Failed to setup {drone}: {str(e)}")
    
    # Wait for all moves to complete
    for drone, future in move_futures:
        try:
            future.join()
            print(f"  ✅ {drone} moved to formation position")
        except Exception as e:
            print(f"  ❌ {drone} move failed: {str(e)}")
    
    time.sleep(2)  # Allow stabilization
    print("✅ Formation reset completed")

def take_photo_for_drone(client, drone_name, photo_dir, camera_name="0"):
    """Take a photo for a specific drone and save it."""
    try:
        # Generate timestamp-based filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename_base = f"{drone_name}_photo_{timestamp}"
        
        print(f"📸 Taking photo for {drone_name}...")
        
        # Take photo
        responses = client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
        ], vehicle_name=drone_name)
        
        if responses and len(responses) > 0:
            response = responses[0]
            
            print(f"  Image captured - Type: {response.image_type}, Size: {response.width}x{response.height}")
            print(f"  Pixels as float: {response.pixels_as_float}, Compress: {response.compress}")
            
            # Handle different image formats
            if response.pixels_as_float:
                # Float images - save as PFM
                filepath = os.path.join(photo_dir, filename_base + '.pfm')
                airsim.write_pfm(os.path.normpath(filepath), airsim.get_pfm_array(response))
                print(f"  ✅ Photo saved (PFM): {filename_base}.pfm")
                return filepath
                
            elif response.compress:  # PNG format - compressed
                # Compressed PNG images
                filepath = os.path.join(photo_dir, filename_base + '.png')
                if len(response.image_data_uint8) > 0:
                    airsim.write_file(os.path.normpath(filepath), response.image_data_uint8)
                    if os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                        print(f"  ✅ Photo saved (compressed): {filename_base}.png ({os.path.getsize(filepath)} bytes)")
                        return filepath
                    else:
                        print(f"  ❌ Compressed photo save failed")
                        return None
                else:
                    print(f"  ❌ Empty compressed image data")
                    return None
                    
            else:  # Uncompressed array
                # Uncompressed RGB arrays
                filepath = os.path.join(photo_dir, filename_base + '.png')
                if len(response.image_data_uint8) > 0:
                    # Convert to numpy array and save with cv2
                    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                    img_rgb = img1d.reshape(response.height, response.width, 3)  # H x W x 3
                    
                    # Use cv2 to write PNG
                    success = cv2.imwrite(os.path.normpath(filepath), img_rgb)
                    
                    if success and os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                        print(f"  ✅ Photo saved (uncompressed): {filename_base}.png ({os.path.getsize(filepath)} bytes)")
                        return filepath
                    else:
                        print(f"  ❌ Uncompressed photo save failed")
                        return None
                else:
                    print(f"  ❌ Empty uncompressed image data")
                    return None
        else:
            print(f"  ❌ No image response for {drone_name}")
            return None
            
    except Exception as e:
        print(f"  ❌ Photo capture failed for {drone_name}: {str(e)}")
        return None

def main():
    print("🚁 Multi-Agent Drone Box Formation Flight Mission")
    print("=" * 55)
    
    # Setup photo directory
    photo_dir = os.path.join(os.path.dirname(__file__), "multi_drone_photos")
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    mission_dir = os.path.join(photo_dir, f"box_rotation_mission_{timestamp}")
    os.makedirs(mission_dir, exist_ok=True)
    print(f"📁 Photos will be saved to: {mission_dir}")
    
    # Define drone names in box formation order
    # Box formation (clockwise from bottom-left):
    # Drone4 ---- Drone3
    #   |           |
    #   |           |
    # Drone1 ---- Drone2
    drone_names = ["Drone1", "Drone2", "Drone3", "Drone4"]
    
    print("\n📐 Initial Box Formation:")
    print("  Drone4 (-5,+5) ---- Drone3 (+5,+5)")
    print("      |                   |")
    print("      |                   |")
    print("  Drone1 (-5,-5) ---- Drone2 (+5,-5)")
    print("\n🔄 Rotation Pattern:")
    print("  Drone1 → Drone2 position (bottom-left → bottom-right)")
    print("  Drone2 → Drone3 position (bottom-right → top-right)")
    print("  Drone3 → Drone4 position (top-right → top-left)")
    print("  Drone4 → Drone1 position (top-left → bottom-left)")
    
    # Connect to the AirSim simulator
    print("\n🔗 Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected to AirSim")
    
    # Check what vehicles are available in the simulation
    print("\n🔍 Checking available vehicles...")
    try:
        # Try to list all available vehicles
        available_vehicles = []
        for drone in drone_names:
            try:
                # Try to get state - if this works, the drone exists
                state = client.getMultirotorState(vehicle_name=drone)
                available_vehicles.append(drone)
                print(f"  ✅ {drone} is available in simulation")
            except Exception as e:
                print(f"  ❌ {drone} not available: {str(e)}")
        
        if len(available_vehicles) == 0:
            print("  ⚠️  No drones found! Check if:")
            print("     - settings.json is in the correct location")
            print("     - AirSim is running with the correct settings")
            print("     - Vehicle names match exactly")
            return
        elif len(available_vehicles) < len(drone_names):
            print(f"  ⚠️  Only {len(available_vehicles)}/{len(drone_names)} drones available")
            # Update drone_names to only include available drones
            drone_names[:] = available_vehicles
            print(f"  📝 Continuing with available drones: {drone_names}")
        else:
            print(f"  ✅ All {len(drone_names)} drones are available")
            
    except Exception as e:
        print(f"  ❌ Error checking vehicles: {str(e)}")
        print("  Continuing anyway - will handle errors during mission")
    
    # Enable API control and arm all drones
    print("\n🔧 Setting up drones...")
    for drone in drone_names:
        client.enableApiControl(True, drone)
        client.armDisarm(True, drone)
        print(f"  ✅ {drone} armed and ready")
    
    # Wait a moment for physics to stabilize after arming
    print("⏳ Waiting for physics to stabilize...")
    time.sleep(3)
    
    # Define expected box formation positions (matching settings.json)
    expected_positions = {
        "Drone1": (-5, -5, -2),  # Bottom-Left
        "Drone2": (5, -5, -2),   # Bottom-Right  
        "Drone3": (5, 5, -2),    # Top-Right
        "Drone4": (-5, 5, -2)    # Top-Left
    }
    
    # Get current positions using multiple methods for debugging
    print("\n📍 Checking drone positions using different methods...")
    current_positions = {}
    positions_correct = True
    
    for drone in drone_names:
        print(f"\n🔍 Debugging {drone} position:")
        
        # Method 1: getMultirotorState
        try:
            state = client.getMultirotorState(vehicle_name=drone)
            pos_estimated = state.kinematics_estimated.position
            print(f"  Method 1 (kinematics_estimated): ({pos_estimated.x_val:.1f}, {pos_estimated.y_val:.1f}, {pos_estimated.z_val:.1f})")
        except Exception as e:
            print(f"  Method 1 failed: {e}")
            pos_estimated = None
        
        # Method 2: Try ground truth position
        try:
            pos_ground_truth = state.kinematics_estimated.position  # This is the same as above, let's try a different approach
            print(f"  Method 2 (same as method 1): ({pos_ground_truth.x_val:.1f}, {pos_ground_truth.y_val:.1f}, {pos_ground_truth.z_val:.1f})")
        except Exception as e:
            print(f"  Method 2 failed: {e}")
        
        # Method 3: Try getting pose directly
        try:
            pose = client.simGetVehiclePose(vehicle_name=drone)
            pos_pose = pose.position
            print(f"  Method 3 (simGetVehiclePose): ({pos_pose.x_val:.1f}, {pos_pose.y_val:.1f}, {pos_pose.z_val:.1f})")
        except Exception as e:
            print(f"  Method 3 failed: {e}")
            pos_pose = None
        
        # Method 4: Check if drone is even connected/available
        try:
            # Try to get basic info about the drone
            gps_data = client.getGpsData(vehicle_name=drone)
            print(f"  GPS available: {gps_data is not None}")
        except Exception as e:
            print(f"  GPS check failed: {e}")
        
        # Use the best available position
        if pos_pose is not None:
            # Use pose position if available
            current_pos = (pos_pose.x_val, pos_pose.y_val, pos_pose.z_val)
            print(f"  ✅ Using pose position: ({pos_pose.x_val:.1f}, {pos_pose.y_val:.1f}, {pos_pose.z_val:.1f})")
        elif pos_estimated is not None:
            # Fall back to estimated position
            current_pos = (pos_estimated.x_val, pos_estimated.y_val, pos_estimated.z_val)
            print(f"  ⚠️  Using estimated position: ({pos_estimated.x_val:.1f}, {pos_estimated.y_val:.1f}, {pos_estimated.z_val:.1f})")
        else:
            # No position available
            current_pos = (0.0, 0.0, 0.0)
            print(f"  ❌ No position data available - using (0,0,0)")
        
        current_positions[drone] = current_pos
        expected = expected_positions[drone]
        
        # Check if position is close to expected (within 5 meter tolerance)
        distance = ((current_pos[0] - expected[0])**2 + (current_pos[1] - expected[1])**2)**0.5
        
        if drone == "Drone1":
            corner = "Bottom-Left"
        elif drone == "Drone2":
            corner = "Bottom-Right"
        elif drone == "Drone3":
            corner = "Top-Right"
        else:  # Drone4
            corner = "Top-Left"
        
        print(f"  📍 Final {drone} ({corner}): ({current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f})")
        print(f"    Expected: ({expected[0]}, {expected[1]}, {expected[2]}) - Distance: {distance:.1f}m")
        
        # Check if all drones are at origin (0,0) - this indicates settings.json not loaded
        if abs(current_pos[0]) < 0.5 and abs(current_pos[1]) < 0.5:
            print(f"    ⚠️  {drone} appears to be at origin - settings.json may not be loaded or drone not spawned")
            positions_correct = False
        elif distance > 5.0:  # More than 5 meters away from expected position
            print(f"    ⚠️  {drone} is {distance:.1f}m away from expected position")
            positions_correct = False
        else:
            print(f"    ✅ {drone} position is acceptable (within {distance:.1f}m)")
    
    # If positions are wrong, ask user what to do
    if not positions_correct:
        print("\n🤔 Drone positions don't match expected box formation.")
        
        # Check if all drones are at origin - special case
        all_at_origin = all(abs(pos[0]) < 0.5 and abs(pos[1]) < 0.5 for pos in current_positions.values())
        
        if all_at_origin:
            print("\n📍 All drones report position (0,0) but may be visually positioned correctly in Unreal.")
            print("This often happens when settings.json positions are loaded visually but physics hasn't updated.")
            print("\nOptions:")
            print("  1. Use expected box positions (recommended - assumes drones are visually correct)")
            print("  2. Move drones to expected box formation positions via API")
            print("  3. Use current (0,0) positions as-is")
            print("  4. Abort mission")
        else:
            print("Options:")
            print("  1. Use current positions as-is")
            print("  2. Move drones to expected box formation positions")
            print("  3. Abort mission")
        
        try:
            choice = input("Enter choice (1/2/3/4): ").strip()
        except:
            choice = "1"  # Default to using expected positions when all at origin
        
        # If all drones are at origin, automatically use expected positions
        if all_at_origin and choice not in ["2", "3", "4"]:
            choice = "1"
            print("🔄 Auto-selecting option 1: Using expected positions (drones at origin detected)")
        
        if all_at_origin and choice == "1":
            # Use expected positions when all drones are at origin
            print("✅ Using expected box formation positions (assuming drones are visually correct)")
            current_positions = expected_positions.copy()
            
        elif (all_at_origin and choice == "2") or (not all_at_origin and choice == "2"):
            print("\n🔧 Moving drones to expected box formation positions...")
            
            # Move drones to box formation positions
            move_futures = []
            for drone in drone_names:
                expected_x, expected_y, expected_z = expected_positions[drone]
                print(f"  Moving {drone} to ({expected_x}, {expected_y}, {expected_z})")
                takeoff_future = client.takeoffAsync(vehicle_name=drone)
                future = client.moveToPositionAsync(
                    expected_x, expected_y, expected_z, 3,  # Slow speed for positioning
                    vehicle_name=drone
                )
                move_futures.append((drone, future))
            
            # Wait for all positioning moves to complete
            for drone, future in move_futures:
                future.join()
                print(f"  ✅ {drone} positioned")
            
            print("✅ All drones moved to box formation")
            time.sleep(2)  # Allow stabilization
            
            # Update positions after manual positioning
            for drone in drone_names:
                state = client.getMultirotorState(vehicle_name=drone)
                pos = state.kinematics_estimated.position
                current_positions[drone] = (pos.x_val, pos.y_val, pos.z_val)
                
        elif (all_at_origin and choice == "4") or (not all_at_origin and choice == "3"):
            print("Mission aborted by user")
            return
        else:  # choice == "1" or any other input
            print("✅ Using current drone positions for the mission")
    
    # Use current positions as initial positions for the mission
    initial_positions = current_positions
    
    try:
        # Phase 1: Takeoff all drones
        print("\n🚀 Phase 1: Taking off all drones...")
        takeoff_futures = []
        for drone in drone_names:
            future = client.takeoffAsync(vehicle_name=drone)
            takeoff_futures.append(future)
            print(f"  {drone} taking off...")
        
        # Wait for all takeoffs to complete
        for future in takeoff_futures:
            future.join()
        
        print("✅ All drones have taken off successfully")
        time.sleep(2)  # Allow stabilization
        
        # Phase 2: Box rotation pattern
        print("\n🔄 Phase 2: Executing box rotation pattern...")
        print("  Each drone rotates clockwise to the next corner:")
        print("  Drone1 → Drone2 position (bottom-left → bottom-right)")
        print("  Drone2 → Drone3 position (bottom-right → top-right)")
        print("  Drone3 → Drone4 position (top-right → top-left)")
        print("  Drone4 → Drone1 position (top-left → bottom-left)")
        
        # Create target positions (each drone goes to the next drone's position)
        target_positions = {}
        for i, drone in enumerate(drone_names):
            next_drone = drone_names[(i + 1) % len(drone_names)]  # Circular indexing
            target_positions[drone] = initial_positions[next_drone]
        
        # Move all drones to their target positions simultaneously
        move_futures = []
        flight_altitude = -10  # 10 meters above ground
        flight_speed = 5
        
        for drone in drone_names:
            target_x, target_y, _ = target_positions[drone]
            print(f"  {drone} flying to ({target_x:.1f}, {target_y:.1f}, {flight_altitude})")
            
            future = client.moveToPositionAsync(
                target_x, target_y, flight_altitude, flight_speed, 
                vehicle_name=drone
            )
            move_futures.append((drone, future))
        
        # Wait for all movements to complete
        for drone, future in move_futures:
            future.join()
            print(f"  ✅ {drone} reached target position")
        
        print("✅ All drones have reached their target positions")
        time.sleep(2)  # Allow stabilization
        
        # Phase 3: Take photos at target positions
        print("\n📸 Phase 3: Taking photos at target positions...")
        photo_results = {}
        
        for drone in drone_names:
            photo_path = take_photo_for_drone(client, drone, mission_dir)
            photo_results[drone] = photo_path
            time.sleep(0.5)  # Small delay between photos
        
        # Summary of photos taken
        successful_photos = sum(1 for path in photo_results.values() if path is not None)
        print(f"\n📊 Photo Summary: {successful_photos}/{len(drone_names)} photos captured successfully")
        
        # Phase 4: Return to home and land
        print("\n🏠 Phase 4: Returning to home positions and landing...")
        
        # Return all drones to their initial positions
        return_futures = []
        for drone in drone_names:
            home_x, home_y, _ = initial_positions[drone]
            print(f"  {drone} returning to home ({home_x:.1f}, {home_y:.1f}, {flight_altitude})")
            
            future = client.moveToPositionAsync(
                home_x, home_y, flight_altitude, flight_speed,
                vehicle_name=drone
            )
            return_futures.append((drone, future))
        
        # Wait for all returns to complete
        for drone, future in return_futures:
            future.join()
            print(f"  ✅ {drone} returned home")
        
        time.sleep(2)  # Allow stabilization
        
        # Land all drones
        print("\n🛬 Landing all drones...")
        land_futures = []
        for drone in drone_names:
            future = client.landAsync(vehicle_name=drone)
            land_futures.append(future)
            print(f"  {drone} landing...")
        
        # Wait for all landings to complete
        for future in land_futures:
            future.join()
        
        print("✅ All drones have landed successfully")
        
        # Mission Summary
        print("\n" + "=" * 50)
        print("🎉 MISSION COMPLETED SUCCESSFULLY!")
        print("=" * 50)
        print(f"📁 Photos saved in: {mission_dir}")
        print(f"📸 Photos captured: {successful_photos}/{len(drone_names)}")
        
        for drone, photo_path in photo_results.items():
            if photo_path:
                filename = os.path.basename(photo_path)
                print(f"  ✅ {drone}: {filename}")
            else:
                print(f"  ❌ {drone}: Photo failed")
        
    except Exception as e:
        print(f"\n❌ Mission failed: {str(e)}")
        print("🚨 Attempting emergency landing...")
        
        # Emergency landing for all drones
        for drone in drone_names:
            try:
                client.landAsync(vehicle_name=drone).join()
                print(f"  ✅ {drone} emergency landed")
            except:
                print(f"  ❌ {drone} emergency landing failed")
    
    finally:
        # Cleanup - disarm and disable API control
        print("\n🔧 Cleaning up...")
        for drone in drone_names:
            try:
                client.armDisarm(False, drone)
                client.enableApiControl(False, drone)
                print(f"  ✅ {drone} disarmed and API control disabled")
            except:
                print(f"  ⚠️  {drone} cleanup had issues")
        
        print("✅ Mission cleanup completed")

if __name__ == "__main__":
    main()

