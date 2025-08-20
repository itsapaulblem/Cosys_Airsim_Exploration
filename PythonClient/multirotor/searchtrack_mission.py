

#!/usr/bin/env python3
import math
import time
import argparse
import numpy as np
import setup_path
import cosysairsim as airsim
import sys
import cv2

def parse_coordinates(coord_string):
    try:
        if ',' in coord_string:
            parts = coord_string.split(',')
        else:
            parts = coord_string.split()
        if len(parts) != 2:
            raise ValueError("Invalid coordinate format")
        lat = float(parts[0].strip())
        lon = float(parts[1].strip())
        return lat, lon
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError("Search box requires exactly 4 GPS coordinates")
    return True

def gps_to_ned(lat, lon, home_lat, home_lon):
    R = 6378137.0
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    return north, east

def detect_any_non_white(client, camera_names):
    for cam in camera_names:
        responses = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
        ])
        if not responses or responses[0].width == 0:
            continue

        img_bytes = responses[0].image_data_uint8
        img1d = np.frombuffer(img_bytes, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)

        # Check for any non-white pixel
        non_white_mask = ~((img_rgb[:, :, 0] == 255) & (img_rgb[:, :, 1] == 255) & (img_rgb[:, :, 2] == 255))
        if np.any(non_white_mask):
            return True
    return False

def get_ned_bounds_from_gps(waypoints, client):
    gps_data = client.getGpsData()
    home_lat = gps_data.gnss.geo_point.latitude
    home_lon = gps_data.gnss.geo_point.longitude
    home_pos = client.getMultirotorState().kinematics_estimated.position
    ned_points = []
    for lat, lon in waypoints:
        north, east = gps_to_ned(lat, lon, home_lat, home_lon)
        x = home_pos.x_val + north
        y = home_pos.y_val + east
        ned_points.append((x, y))
    xs = [p[0] for p in ned_points]
    ys = [p[1] for p in ned_points]
    return min(xs), max(xs), min(ys), max(ys)

def search_and_land_on_any_detection(client, ned_bounds, speed, altitude, step):
    min_x, max_x, min_y, max_y = ned_bounds
    direction = 1
    y = min_y
    camera_names = ["bottom", "front", "back", "left", "right"]

    while y <= max_y:
        x_start, x_end = (min_x, max_x) if direction == 1 else (max_x, min_x)
        for x in [x_start, x_end]:
            print(f"Moving to ({x:.1f}, {y:.1f}, {altitude:.1f})")
            client.moveToPositionAsync(x, y, altitude, speed).join()
            time.sleep(1)

            if detect_any_non_white(client, camera_names):
                print("✅ Non-white object detected! Landing now.")
                client.hoverAsync().join()
                time.sleep(1)
                client.landAsync().join()
                return True
            else:
                print("No object detected, continuing...")

        y += step
        direction *= -1
        time.sleep(0.5)

    return False

def main():
    parser = argparse.ArgumentParser(description="Drone Search and Land on Any Object")
    parser.add_argument('--waypoints', type=str, nargs=4, required=True,
                        help="4 GPS coordinates defining search boundary")
    parser.add_argument('--altitude', type=float, default=5,
                        help="Search altitude in meters")
    parser.add_argument('--speed', type=float, default=5,
                        help="Flight speed in m/s")
    parser.add_argument('--step', type=float, default=10,
                        help="Distance between search lines in meters")
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1

    try:
        client = airsim.MultirotorClient(ip="172.22.112.1")
        client.confirmConnection()

        print("Preparing drone...")
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()

        target_z = -abs(args.altitude)
        client.moveToZAsync(target_z, args.speed).join()
        time.sleep(1)

        ned_bounds = get_ned_bounds_from_gps(waypoints, client)

        found = search_and_land_on_any_detection(
            client, ned_bounds, speed=args.speed, altitude=target_z, step=args.step
        )

        client.armDisarm(False)
        client.enableApiControl(False)

        if found:
            print("🎯 Person found and landed.")
            return 0
        else:
            print("❌ Person not found.")
            return 1

    except Exception as e:
        print(f"Mission failed: {e}")
        try:
            client.armDisarm(False)
            client.enableApiControl(False)
        except:
            pass
        return 1

if __name__ == "__main__":
    sys.exit(main())


# #!/usr/bin/env python3
# import math
# import time
# import argparse
# import numpy as np
# import setup_path
# import cosysairsim as airsim
# import sys
# import cv2

# def parse_coordinates(coord_string):
#     try:
#         if ',' in coord_string:
#             parts = coord_string.split(',')
#         else:
#             parts = coord_string.split()
#         if len(parts) != 2:
#             raise ValueError("Invalid coordinate format")
#         lat = float(parts[0].strip())
#         lon = float(parts[1].strip())
#         if not (-90 <= lat <= 90):
#             raise ValueError(f"Invalid latitude: {lat}")
#         if not (-180 <= lon <= 180):
#             raise ValueError(f"Invalid longitude: {lon}")
#         return lat, lon
#     except (ValueError, IndexError) as e:
#         raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

# def validate_search_box(waypoints):
#     if len(waypoints) != 4:
#         raise ValueError(f"Search box requires exactly 4 GPS coordinates, got {len(waypoints)}")
#     lats = [wp[0] for wp in waypoints]
#     lons = [wp[1] for wp in waypoints]
#     lat_range = max(lats) - min(lats)
#     lon_range = max(lons) - min(lons)
#     if lat_range == 0 or lon_range == 0:
#         raise ValueError("Search box coordinates must form a rectangle with non-zero area")
#     print(f"Search box dimensions: {lat_range:.6f}° lat × {lon_range:.6f}° lon")
#     return True

# def gps_to_ned(target_lat, target_lon, home_lat, home_lon):
#     R = 6378137.0
#     lat1 = math.radians(home_lat)
#     lon1 = math.radians(home_lon)
#     lat2 = math.radians(target_lat)
#     lon2 = math.radians(target_lon)
#     dlat = lat2 - lat1
#     dlon = lon2 - lon1
#     north = dlat * R
#     east = dlon * R * math.cos(lat1)
#     return north, east

# def estimate_distance_to_target(center_x, center_y, object_size, cam, altitude):
#     """
#     Estimate approximate distance to target based on camera view and object size
#     """
#     if cam == "bottom":
#         horizontal_fov = 150
#         vertical_fov = 150
#     else:
#         horizontal_fov = 90
#         vertical_fov = 90
#     img_width = 256
#     img_height = 144
#     pixels_per_degree_x = img_width / horizontal_fov
#     pixels_per_degree_y = img_height / vertical_fov
#     center_offset_x = (center_x - img_width/2) / pixels_per_degree_x
#     center_offset_y = (center_y - img_height/2) / pixels_per_degree_y
#     if cam == "bottom":
#         distance_x = altitude * math.tan(math.radians(abs(center_offset_x)))
#         distance_y = altitude * math.tan(math.radians(abs(center_offset_y)))
#         horizontal_distance = math.sqrt(distance_x**2 + distance_y**2)
#     else:
#         if object_size > 1000:
#             horizontal_distance = 5
#         elif object_size > 500:
#             horizontal_distance = 10
#         elif object_size > 200:
#             horizontal_distance = 20
#         else:
#             horizontal_distance = 30
#     return horizontal_distance, center_offset_x, center_offset_y

# def detect_nonwhite_circle_all_cameras(client, min_pixels=20, detection_range=50, silent=False):
#     camera_names = ["front"]  # Only using the front camera
#     detections = []
#     state = client.getMultirotorState()
#     altitude = abs(state.kinematics_estimated.position.z_val)

#     for cam in camera_names:
#         responses = client.simGetImages([
#             airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
#         ])
#         if not responses or responses[0].width == 0:
#             if not silent:
#                 print(f"No segmentation image from camera: {cam}")
#             continue

#         img_bytes = responses[0].image_data_uint8
#         img1d = np.frombuffer(img_bytes, dtype=np.uint8)
#         img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
#         img_rgb = np.flipud(img_rgb)

#         nonwhite_mask = ~((img_rgb[:, :, 0] == 255) & (img_rgb[:, :, 1] == 255) & (img_rgb[:, :, 2] == 255))
#         nonblack_mask = ~((img_rgb[:, :, 0] == 0) & (img_rgb[:, :, 1] == 0) & (img_rgb[:, :, 2] == 0))
#         target_mask = nonwhite_mask & nonblack_mask

#         if np.sum(target_mask) >= min_pixels:
#             mask_uint8 = target_mask.astype(np.uint8) * 255
#             num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_uint8)
#             if num_labels > 1:
#                 largest_component = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
#                 largest_mask = (labels == largest_component)
#                 if np.sum(largest_mask) >= min_pixels:
#                     ys, xs = np.where(largest_mask)
#                     center_x, center_y = int(np.mean(xs)), int(np.mean(ys))
#                     object_size = np.sum(largest_mask)
#                     distance, offset_x, offset_y = estimate_distance_to_target(
#                         center_x, center_y, object_size, cam, altitude
#                     )
#                     center_color = img_rgb[center_y, center_x]
                    
#                     # Always save detection images when target is found
#                     timestamp = int(time.time())
#                     filename = f"target_detected_{cam}_{timestamp}.png"
#                     mask_filename = f"detection_mask_{cam}_{timestamp}.png"
#                     cv2.imwrite(filename, cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
#                     mask_viz = np.zeros_like(img_rgb)
#                     mask_viz[target_mask] = [0, 255, 255]
#                     cv2.imwrite(mask_filename, cv2.cvtColor(mask_viz, cv2.COLOR_RGB2BGR))
                    
#                     detection_info = {
#                         'center_x': center_x,
#                         'center_y': center_y,
#                         'camera': cam,
#                         'object_size': object_size,
#                         'estimated_distance': distance,
#                         'offset_x': offset_x,
#                         'offset_y': offset_y,
#                         'center_color': center_color,
#                         'filename': filename,
#                         'mask_filename': mask_filename
#                     }
                    
#                     if not silent:
#                         print(f"📸 TARGET DETECTED! Non-white object from '{cam}' camera")
#                         print(f"   Position: pixel ({center_x}, {center_y})")
#                         print(f"   Estimated distance: {distance:.1f} meters")
#                         print(f"   Object size: {object_size} pixels")
#                         print(f"   Within range: {'YES' if distance <= detection_range else 'NO'}")
#                         print(f"   Images saved: {filename}, {mask_filename}")
                    
#                     detections.append(detection_info)

#     valid_detections = [d for d in detections if d['estimated_distance'] <= detection_range]
#     if valid_detections:
#         closest = min(valid_detections, key=lambda x: x['estimated_distance'])
#         if not silent:
#             print(f"🎯 TARGET IN RANGE! Closest detection: {closest['estimated_distance']:.1f}m from {closest['camera']} camera")
#         return closest
#     if detections:
#         closest = min(detections, key=lambda x: x['estimated_distance'])
#         if not silent:
#             print(f"👁️ Target detected but out of range: {closest['estimated_distance']:.1f}m (max: {detection_range}m)")
#         return None
#     return None

# def navigate_to_target(client, detection_info, speed=2.0):
#     print(f"🚁 Navigating to target detected by {detection_info['camera']} camera...")
#     current_state = client.getMultirotorState()
#     current_pos = current_state.kinematics_estimated.position
#     move_distance = min(detection_info['estimated_distance'] * 0.7, 20)
    
#     if detection_info['camera'] == "bottom":
#         move_scale = move_distance / 100
#         move_x = detection_info['offset_x'] * move_scale
#         move_y = -detection_info['offset_y'] * move_scale
#         new_x = current_pos.x_val + move_x
#         new_y = current_pos.y_val + move_y
#         new_z = current_pos.z_val
#     elif detection_info['camera'] == "front":
#         new_x = current_pos.x_val + move_distance
#         new_y = current_pos.y_val
#         new_z = current_pos.z_val
#     elif detection_info['camera'] == "back":
#         new_x = current_pos.x_val - move_distance
#         new_y = current_pos.y_val
#         new_z = current_pos.z_val
#     elif detection_info['camera'] == "left":
#         new_x = current_pos.x_val
#         new_y = current_pos.y_val - move_distance
#         new_z = current_pos.z_val
#     elif detection_info['camera'] == "right":
#         new_x = current_pos.x_val
#         new_y = current_pos.y_val + move_distance
#         new_z = current_pos.z_val
    
#     print(f"Moving {move_distance:.1f}m towards target...")
#     client.moveToPositionAsync(new_x, new_y, new_z, speed).join()
#     time.sleep(1)

# def get_ned_bounds_from_gps(waypoints, client):
#     gps_data = client.getGpsData()
#     home_lat = gps_data.gnss.geo_point.latitude
#     home_lon = gps_data.gnss.geo_point.longitude
#     home_pos = client.getMultirotorState().kinematics_estimated.position
#     ned_points = []
#     for lat, lon in waypoints:
#         north, east = gps_to_ned(lat, lon, home_lat, home_lon)
#         x = home_pos.x_val + north
#         y = home_pos.y_val + east
#         ned_points.append((x, y))
#     xs = [p[0] for p in ned_points]
#     ys = [p[1] for p in ned_points]
#     return min(xs), max(xs), min(ys), max(ys)

# def print_altitude_loop(client, target_altitude, mode="climb"):
#     while True:
#         state = client.getMultirotorState()
#         current_altitude = -state.kinematics_estimated.position.z_val
#         if mode == "climb":
#             print(f"Climbing... Altitude: {current_altitude:.2f} m")
#             if abs(current_altitude - abs(target_altitude)) < 0.5:
#                 break
#         elif mode == "descend":
#             print(f"Descending... Altitude: {current_altitude:.2f} m")
#             if state.landed_state == airsim.LandedState.Landed:
#                 print("Drone has landed.")
#                 break
#         time.sleep(0.5)
#     return current_altitude

# def precision_land_on_target(client, descend_distance=100, speed=1.0):
#     print("🛬 Initiating precision landing on target...")
#     state = client.getMultirotorState()
#     start_z = state.kinematics_estimated.position.z_val
#     current_pos = state.kinematics_estimated.position
#     descend_steps = 10
#     step_distance = descend_distance / descend_steps
    
#     for step in range(descend_steps):
#         target_z = start_z + (step + 1) * step_distance
#         client.moveToZAsync(target_z, speed).join()
#         result = detect_nonwhite_circle_all_cameras(client, detection_range=10)
#         if result:
#             center_x, center_y, cam = result['center_x'], result['center_y'], result['camera']
#             print(f"Step {step+1}/{descend_steps}: Still tracking target from {cam} camera")
#             img_center_x, img_center_y = 128, 72
#             offset_x = center_x - img_center_x
#             offset_y = center_y - img_center_y
#             if abs(offset_x) > 25 or abs(offset_y) > 25:
#                 current_pos = client.getMultirotorState().kinematics_estimated.position
#                 adjust_scale = 0.01
#                 new_x = current_pos.x_val + offset_x * adjust_scale
#                 new_y = current_pos.y_val - offset_y * adjust_scale
#                 client.moveToPositionAsync(new_x, new_y, target_z, speed/2).join()
#                 print(f"   Adjusted position by ({offset_x * adjust_scale:.2f}, {-offset_y * adjust_scale:.2f})")
#         else:
#             print(f"Step {step+1}/{descend_steps}: Target lost, continuing descent...")
#         time.sleep(0.5)
    
#     print("Final landing phase...")
#     client.landAsync().join()

# def search_lawnmower_with_continuous_detection(client, ned_bounds, speed, altitude, step, detection_range):
#     print(f"🔍 Starting continuous detection lawnmower search at altitude (NED): {altitude}")
#     print(f"Detection range: {detection_range} meters")
    
#     min_x, max_x, min_y, max_y = ned_bounds
#     direction = 1
#     y = min_y
#     search_points = 0
    
#     # Calculate waypoints for the entire search pattern
#     search_waypoints = []
#     current_y = min_y
#     while current_y <= max_y:
#         if direction == 1:
#             # Left to right
#             for x in np.linspace(min_x, max_x, max(2, int((max_x - min_x) / 5))):
#                 search_waypoints.append((x, current_y))
#         else:
#             # Right to left
#             for x in np.linspace(max_x, min_x, max(2, int((max_x - min_x) / 5))):
#                 search_waypoints.append((x, current_y))
#         current_y += step
#         direction *= -1
    
#     print(f"🗺️ Generated {len(search_waypoints)} search waypoints")
    
#     # Search with frequent detection checks
#     for i, (x, y) in enumerate(search_waypoints):
#         print(f"🔍 Waypoint {i+1}/{len(search_waypoints)}: Moving to ({x:.1f}, {y:.1f})")
        
#         # Move towards waypoint with small increments and frequent checks
#         current_pos = client.getMultirotorState().kinematics_estimated.position
        
#         # Break movement into smaller segments for continuous detection
#         segments = 5
#         for seg in range(segments):
#             # Calculate intermediate position
#             progress = (seg + 1) / segments
#             intermediate_x = current_pos.x_val + (x - current_pos.x_val) * progress
#             intermediate_y = current_pos.y_val + (y - current_pos.y_val) * progress
            
#             # Move to intermediate position
#             client.moveToPositionAsync(intermediate_x, intermediate_y, altitude, speed).join()
            
#             # Check for target immediately after each segment
#             result = detect_nonwhite_circle_all_cameras(client, detection_range=detection_range, silent=True)
#             if result:
#                 print("🛑 TARGET DETECTED DURING FLIGHT!")
#                 print(f"🎯 Target found at {result['estimated_distance']:.1f}m!")
#                 print(f"📸 Images saved: {result['filename']}, {result['mask_filename']}")
                
#                 # Stop and hover
#                 client.hoverAsync().join()
                
#                 # Navigate closer to target
#                 navigate_to_target(client, result, speed=speed)
                
#                 # Final verification
#                 final_result = detect_nonwhite_circle_all_cameras(client, detection_range=15)
#                 if final_result:
#                     print(f"✅ Target confirmed at {final_result['estimated_distance']:.1f}m. Landing...")
#                     precision_land_on_target(client, descend_distance=100, speed=1.0)
#                     return True
#                 else:
#                     print("❌ Target lost after approach. Continuing search...")
        
#         # Update current position for next waypoint
#         current_pos = client.getMultirotorState().kinematics_estimated.position
        
#         # Small delay between waypoints
#         time.sleep(0.2)
    
#     print("🔍 Search completed - Target not found within search boundary and detection range.")
#     return False

# def main():
#     parser = argparse.ArgumentParser(description="Drone Person Search with Continuous Detection and Landing")
#     parser.add_argument('--waypoints', type=str, nargs=4, required=True,
#                        help="4 GPS coordinates defining search boundary")
#     parser.add_argument('--altitude', type=float, default=5,
#                        help="Search altitude in meters (default: 5)")
#     parser.add_argument('--speed', type=float, default=5,
#                        help="Flight speed in m/s (default: 5)")
#     parser.add_argument('--step', type=float, default=15,
#                        help="Distance between search lines in meters (default: 15)")
#     parser.add_argument('--detection-range', type=float, default=50,
#                        help="Maximum detection range in meters (default: 50)")
    
#     args = parser.parse_args()
    
#     print("🚁 DRONE CONTINUOUS SEARCH AND LAND MISSION")
#     print("="*60)
    
#     try:
#         waypoints = [parse_coordinates(wp) for wp in args.waypoints]
#         validate_search_box(waypoints)
#         print("🗺️ 4-Point Search Boundary:")
#         for i, (lat, lon) in enumerate(waypoints, 1):
#             print(f"   Point {i}: {lat:.6f}, {lon:.6f}")
#         print(f"🎯 Detection Range: {args.detection_range} meters")
#     except ValueError as e:
#         print(f"❌ Error: {e}")
#         return 1

#     try:
#         print("\n🔌 Connecting to AirSim...")
#         client = airsim.MultirotorClient(ip="172.22.112.1")
#         client.confirmConnection()
#         print("✅ Connected to AirSim")
        
#         print("🎨 Setting up segmentation...")
#         client.simSetSegmentationObjectID(".*", 0, True)
#         client.simSetSegmentationObjectID("SkeletalMeshActor_1", 1, True)
        
#         scene_objects = client.simListSceneObjects()
#         print(f"Scene objects: {scene_objects}")
        
#         person_objects = ["SkeletalMeshActor_1"]
#         for obj_name in person_objects:
#             seg_id = client.simGetSegmentationObjectID(obj_name)
#             if seg_id != -1:
#                 print(f"   {obj_name}: Segmentation ID {seg_id}")
#             else:
#                 print(f"   {obj_name}: Not found")

#         print("\n🚁 Preparing drone...")
#         client.enableApiControl(True)
#         client.armDisarm(True)
#         print("🛫 Taking off...")
#         client.takeoffAsync().join()

#         if args.altitude <= 0:
#             print("⚠️ Altitude too low. Setting to minimum safe altitude of 5 meters.")
#             args.altitude = 5

#         target_z = -abs(args.altitude)
#         print(f"⬆️ Climbing to search altitude: {abs(args.altitude)}m")
#         client.moveToZAsync(target_z, args.speed).join()
#         final_alt = print_altitude_loop(client, abs(args.altitude), mode="climb")
#         print(f"✅ Reached target altitude: {final_alt:.2f} m")
#         time.sleep(1)

#         print("\n📍 Calculating search boundaries...")
#         gps_data = client.getGpsData()
#         home_lat = gps_data.gnss.geo_point.latitude
#         home_lon = gps_data.gnss.geo_point.longitude
#         print(f"🏠 Home position: {home_lat:.6f}, {home_lon:.6f}")
        
#         ned_bounds = get_ned_bounds_from_gps(waypoints, client)
#         print(f"📐 NED search bounds: X({ned_bounds[0]:.1f} to {ned_bounds[1]:.1f}), Y({ned_bounds[2]:.1f} to {ned_bounds[3]:.1f})")

#         print("\n🔍 Beginning continuous detection search mission...")
#         found = search_lawnmower_with_continuous_detection(
#             client, ned_bounds, speed=args.speed, altitude=target_z, 
#             step=args.step, detection_range=args.detection_range
#         )

#         if not found:
#             print("\n🛬 Target not found, landing at current position...")
#             client.landAsync().join()

#         print("\n🧹 Mission cleanup...")
#         client.armDisarm(False)
#         client.enableApiControl(False)
        
#         if found:
#             print("\n🎉 MISSION SUCCESS! Target found and landed on target!")
#         else:
#             print("\n✅ MISSION COMPLETED: Target not found within detection range")
        
#         return 0 if found else 1

#     except Exception as e:
#         print(f"\n❌ Mission failed: {e}")
#         try:
#             client.armDisarm(False)
#             client.enableApiControl(False)
#         except:
#             pass
#         return 1

# if __name__ == "__main__":
#     sys.exit(main())