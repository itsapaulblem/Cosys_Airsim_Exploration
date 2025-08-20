<<<<<<< HEAD
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
            raise ValueError("Invalid coordinate format. Use 'x,y' or 'x y'.")
        x = float(parts[0].strip())
        y = float(parts[1].strip())
        return x, y
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError(f"Search box requires exactly four coordinates, got {len(waypoints)}.")
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    if x_range == 0 or y_range == 0:
        raise ValueError("Search box must have non-zero area. Check your coordinates.")
    print(f"Search box dimensions: {x_range:.2f}m × {y_range:.2f}m")
    return True

def set_segmentation_ids(client):
    print("[INFO] Listing all mesh names in the scene...")
    try:
        mesh_list = client.simListInstanceSegmentationObjects()
        print(f"[INFO] Found {len(mesh_list)} meshes in scene:")
        for i, mesh_name in enumerate(mesh_list):
            print(f"  {i+1}: {mesh_name}")
    except Exception as e:
        print(f"[WARNING] Could not list meshes: {e}")
    
    print("\n[INFO] Assigning segmentation IDs...")
    client.simSetSegmentationObjectID(".*", 0, True)  # Everything = black
    
    # Try to set the target object - with and without regex
    success1 = client.simSetSegmentationObjectID("SkeletalMeshActor_0", 2, False)  # Exact match first
    success2 = client.simSetSegmentationObjectID("SkeletalMeshActor_0", 2, True)   # Regex match
    colorMap = client.simGetSegmentationColorMap()

    print(f"[INFO] Target object 'SkeletalMeshActor_0' - Exact match: {success1}, Regex match: {success2}")
    
    # Also try some common variations
    variations = ["SK_SciFiSoldier02", "Soldier", "Person", "Target", "SkeletalMeshActor_0", ".*[Pp]erson.*"]
    for variation in variations:
        success = client.simSetSegmentationObjectID(variation, 2, True)
        if success:
            print(f"[INFO] Successfully set segmentation ID for pattern: '{variation}'")
    
    print("[INFO] Segmentation IDs set: background=0 (black), target=2")

def verify_segmentation_works(client, camera_name="front_center"):
    print("[INFO] Verifying segmentation setup (target=ID 2, background=black)...")
    
    # Get the color map to see what colors are assigned
    try:
        color_map = client.simGetSegmentationColorMap()
        print(f"[INFO] Segmentation color map has {len(color_map)} entries")
        if len(color_map) > 2:
            print(f"[INFO] Color for ID 0 (background): {color_map[0]}")
            print(f"[INFO] Color for ID 2 (target): {color_map[2]}")
    except Exception as e:
        print(f"[WARNING] Could not get color map: {e}")
    
    # Use the same detection logic as detect_target_id2
    target_color = np.array([2, 0, 0])  # Same as in detect_target_id2
    tolerance = 2  # Same as in detect_target_id2

    response = client.simGetImages([
        airsim.ImageRequest(camera_name, airsim.ImageType.Segmentation, False, False)
    ])[0]

    if response.width == 0:
        print("[ERROR] Failed to capture segmentation image.")
        return False

    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)

    filename = f"verify_segmentation_{camera_name}.png"
    cv2.imwrite(filename, img_rgb)
    print(f"[INFO] Saved segmentation image: {filename}")
    
    # Debug: Show unique colors in the image
    unique_colors = np.unique(img_rgb.reshape(-1, 3), axis=0)
    print(f"[DEBUG] Unique colors in segmentation image: {len(unique_colors)} colors")
    for i, color in enumerate(unique_colors[:10]):  # Show first 10 colors
        pixel_count = np.sum(np.all(img_rgb == color, axis=2))
        print(f"  Color {color}: {pixel_count} pixels")
    
    total_pixels = img_rgb.shape[0] * img_rgb.shape[1]
    diff = np.abs(img_rgb.astype(np.int16) - target_color.astype(np.int16))
    target_mask = np.all(diff <= tolerance, axis=2)
    target_pixels = np.sum(target_mask)
    black_pixels = np.sum(np.all(img_rgb == [0, 0, 0], axis=2))

    if target_pixels > 10:
        print(f"[SUCCESS] Target detected with color {target_color} ({target_pixels} pixels).")
    else:
        print(f"[WARNING] Target NOT detected. Only {target_pixels} pixels with color {target_color}.")

    if black_pixels >= 0.95 * total_pixels:
        print(f"[INFO] Background is mostly black ({black_pixels}/{total_pixels} pixels).")
    else:
        print(f"[WARNING] Background is not fully black. Only {black_pixels}/{total_pixels} pixels are black.")

    return target_pixels > 10 and black_pixels >= 0.95 * total_pixels

def detect_target_id2(client, camera_names):
    target_color = np.array([2, 0, 0])  # Note: segmentation IDs stored as 8-bit RGB color multiples of ID
    tolerance = 2

    for cam in camera_names:
        response = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
        ])[0]
        if response.width == 0:
            continue

        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)

        diff = np.abs(img_rgb.astype(np.int16) - target_color.astype(np.int16))
        mask = np.all(diff <= tolerance, axis=2)
        pixel_count = np.sum(mask)

        if pixel_count > 10:
            print(f"Camera {cam}: Target detected with {pixel_count} pixels")
            return True, cam, pixel_count
    return False, None, 0

def move_and_check(client, x, y, z, speed, camera_names):
    print(f"Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    client.moveToPositionAsync(x, y, z, speed).join()
    time.sleep(0.5)

    found, cam, pixel_count = detect_target_id2(client, camera_names)
    if found:
        print(f"Target spotted by {cam} camera! ({pixel_count} pixels)")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"Target Position: X = {pos.x_val:.2f}, Y = {pos.y_val:.2f}, Z = {pos.z_val:.2f}")

        # Capture and save segmentation and RGB images from the camera that detected the target
        responses = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False),
            airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)
        ])

        seg_response = responses[0]
        rgb_response = responses[1]

        if seg_response.width > 0:
            seg_img_array = np.frombuffer(seg_response.image_data_uint8, dtype=np.uint8)
            seg_img = seg_img_array.reshape(seg_response.height, seg_response.width, 3)
            filename_seg = f"target_found_{cam}_segmentation.png"
            cv2.imwrite(filename_seg, seg_img)
            print(f"Saved segmentation image: {filename_seg}")

        if rgb_response.width > 0:
            rgb_img_array = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
            rgb_img = rgb_img_array.reshape(rgb_response.height, rgb_response.width, 3)
            filename_rgb = f"target_found_{cam}_rgb.png"
            cv2.imwrite(filename_rgb, rgb_img)
            print(f"Saved RGB image: {filename_rgb}")

        return True
    return False

def get_ned_bounds_from_coordinates(waypoints):
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    return min(xs), max(xs), min(ys), max(ys)

def search_and_land_spiral(client, ned_bounds, speed, altitude, step, max_attempts=50, x_resolution=10.0):
    min_x, max_x, min_y, max_y = ned_bounds
    home_pos = client.getMultirotorState().kinematics_estimated.position
    camera_names = ["front_center", "back_center", "front_left", "front_right", "bottom_center"]

    attempt_count = 0
    left = min_x
    right = max_x
    bottom = min_y
    top = max_y

    print(f"[INFO] Starting spiral search (max attempts = {max_attempts})")
    while left <= right and bottom <= top and attempt_count < max_attempts:
        for x in np.arange(left, right, x_resolution):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, x, bottom, altitude, speed, camera_names):
                return True
            attempt_count += 1
        bottom += step

        for y in np.arange(bottom, top, step):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, right, y, altitude, speed, camera_names):
                return True
            attempt_count += 1
        right -= x_resolution

        for x in np.arange(right, left, -x_resolution):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, x, top, altitude, speed, camera_names):
                return True
            attempt_count += 1
        top -= step

        for y in np.arange(top, bottom, -step):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, left, y, altitude, speed, camera_names):
                return True
            attempt_count += 1
        left += x_resolution

    print("[INFO] Maximum attempts reached. Target not found. Returning to home.")
    client.moveToPositionAsync(home_pos.x_val, home_pos.y_val, altitude, speed).join()
    return False

def main():
    parser = argparse.ArgumentParser(description="Spiral search for segmentation target")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four coordinates for the search boundary (x1,y1 x2,y2 x3,y3 x4,y4)")
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--max_attempts", type=int, default=50)
    parser.add_argument("--xres", type=float, default=10.0)
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1

    client = airsim.MultirotorClient(ip="172.22.112.1")
    client.confirmConnection()
    client.setSegmentationObjectID(".*", 0, True)  # Reset segmentation IDs to default
    
    # Set segmentation IDs for background and target
    set_segmentation_ids(client)
    time.sleep(1)  # let settings propagate

    # Verify segmentation image before tracking - but continue even if it fails
    segmentation_verified = verify_segmentation_works(client, camera_name="front_center")
    if not segmentation_verified:
        print("[WARNING] Segmentation verification failed, but continuing with search anyway.")
    else:
        print("[INFO] Segmentation verification passed. Proceeding with search.")

    print("Arming drone...")
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-abs(args.altitude), args.speed).join()
    time.sleep(1)

    ned_bounds = get_ned_bounds_from_coordinates(waypoints)
    found = search_and_land_spiral(
        client, ned_bounds, speed=args.speed, altitude=-abs(args.altitude),
        step=args.step, max_attempts=args.max_attempts, x_resolution=args.xres
    )

    print("Landing...")
    client.hoverAsync().join()
    time.sleep(1)
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    return 0

if __name__ == "__main__":
=======
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
            raise ValueError("Invalid coordinate format. Use 'x,y' or 'x y'.")
        x = float(parts[0].strip())
        y = float(parts[1].strip())
        return x, y
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError(f"Search box requires exactly four coordinates, got {len(waypoints)}.")
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    if x_range == 0 or y_range == 0:
        raise ValueError("Search box must have non-zero area. Check your coordinates.")
    print(f"Search box dimensions: {x_range:.2f}m × {y_range:.2f}m")
    return True

def set_segmentation_ids(client):
    print("[INFO] Listing all mesh names in the scene...")
    try:
        mesh_list = client.simListInstanceSegmentationObjects()
        print(f"[INFO] Found {len(mesh_list)} meshes in scene:")
        for i, mesh_name in enumerate(mesh_list):
            print(f"  {i+1}: {mesh_name}")
    except Exception as e:
        print(f"[WARNING] Could not list meshes: {e}")
    
    print("\n[INFO] Assigning segmentation IDs...")
    client.simSetSegmentationObjectID(".*", 0, True)  # Everything = black
    
    # Try to set the target object - with and without regex
    success1 = client.simSetSegmentationObjectID("SkeletalMeshActor_0", 2, False)  # Exact match first
    success2 = client.simSetSegmentationObjectID("SkeletalMeshActor_0", 2, True)   # Regex match
    colorMap = client.simGetSegmentationColorMap()

    print(f"[INFO] Target object 'SkeletalMeshActor_0' - Exact match: {success1}, Regex match: {success2}")
    
    # Also try some common variations
    variations = ["SK_SciFiSoldier02", "Soldier", "Person", "Target", "SkeletalMeshActor_0", ".*[Pp]erson.*"]
    for variation in variations:
        success = client.simSetSegmentationObjectID(variation, 2, True)
        if success:
            print(f"[INFO] Successfully set segmentation ID for pattern: '{variation}'")
    
    print("[INFO] Segmentation IDs set: background=0 (black), target=2")

def verify_segmentation_works(client, camera_name="front_center"):
    print("[INFO] Verifying segmentation setup (target=ID 2, background=black)...")
    
    # Get the color map to see what colors are assigned
    try:
        color_map = client.simGetSegmentationColorMap()
        print(f"[INFO] Segmentation color map has {len(color_map)} entries")
        if len(color_map) > 2:
            print(f"[INFO] Color for ID 0 (background): {color_map[0]}")
            print(f"[INFO] Color for ID 2 (target): {color_map[2]}")
    except Exception as e:
        print(f"[WARNING] Could not get color map: {e}")
    
    # Use the same detection logic as detect_target_id2
    target_color = np.array([2, 0, 0])  # Same as in detect_target_id2
    tolerance = 2  # Same as in detect_target_id2

    response = client.simGetImages([
        airsim.ImageRequest(camera_name, airsim.ImageType.Segmentation, False, False)
    ])[0]

    if response.width == 0:
        print("[ERROR] Failed to capture segmentation image.")
        return False

    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)

    filename = f"verify_segmentation_{camera_name}.png"
    cv2.imwrite(filename, img_rgb)
    print(f"[INFO] Saved segmentation image: {filename}")
    
    # Debug: Show unique colors in the image
    unique_colors = np.unique(img_rgb.reshape(-1, 3), axis=0)
    print(f"[DEBUG] Unique colors in segmentation image: {len(unique_colors)} colors")
    for i, color in enumerate(unique_colors[:10]):  # Show first 10 colors
        pixel_count = np.sum(np.all(img_rgb == color, axis=2))
        print(f"  Color {color}: {pixel_count} pixels")
    
    total_pixels = img_rgb.shape[0] * img_rgb.shape[1]
    diff = np.abs(img_rgb.astype(np.int16) - target_color.astype(np.int16))
    target_mask = np.all(diff <= tolerance, axis=2)
    target_pixels = np.sum(target_mask)
    black_pixels = np.sum(np.all(img_rgb == [0, 0, 0], axis=2))

    if target_pixels > 10:
        print(f"[SUCCESS] Target detected with color {target_color} ({target_pixels} pixels).")
    else:
        print(f"[WARNING] Target NOT detected. Only {target_pixels} pixels with color {target_color}.")

    if black_pixels >= 0.95 * total_pixels:
        print(f"[INFO] Background is mostly black ({black_pixels}/{total_pixels} pixels).")
    else:
        print(f"[WARNING] Background is not fully black. Only {black_pixels}/{total_pixels} pixels are black.")

    return target_pixels > 10 and black_pixels >= 0.95 * total_pixels

def detect_target_id2(client, camera_names):
    target_color = np.array([2, 0, 0])  # Note: segmentation IDs stored as 8-bit RGB color multiples of ID
    tolerance = 2

    for cam in camera_names:
        response = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
        ])[0]
        if response.width == 0:
            continue

        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)

        diff = np.abs(img_rgb.astype(np.int16) - target_color.astype(np.int16))
        mask = np.all(diff <= tolerance, axis=2)
        pixel_count = np.sum(mask)

        if pixel_count > 10:
            print(f"Camera {cam}: Target detected with {pixel_count} pixels")
            return True, cam, pixel_count
    return False, None, 0

def move_and_check(client, x, y, z, speed, camera_names):
    print(f"Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    client.moveToPositionAsync(x, y, z, speed).join()
    time.sleep(0.5)

    found, cam, pixel_count = detect_target_id2(client, camera_names)
    if found:
        print(f"Target spotted by {cam} camera! ({pixel_count} pixels)")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"Target Position: X = {pos.x_val:.2f}, Y = {pos.y_val:.2f}, Z = {pos.z_val:.2f}")

        # Capture and save segmentation and RGB images from the camera that detected the target
        responses = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False),
            airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)
        ])

        seg_response = responses[0]
        rgb_response = responses[1]

        if seg_response.width > 0:
            seg_img_array = np.frombuffer(seg_response.image_data_uint8, dtype=np.uint8)
            seg_img = seg_img_array.reshape(seg_response.height, seg_response.width, 3)
            filename_seg = f"target_found_{cam}_segmentation.png"
            cv2.imwrite(filename_seg, seg_img)
            print(f"Saved segmentation image: {filename_seg}")

        if rgb_response.width > 0:
            rgb_img_array = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
            rgb_img = rgb_img_array.reshape(rgb_response.height, rgb_response.width, 3)
            filename_rgb = f"target_found_{cam}_rgb.png"
            cv2.imwrite(filename_rgb, rgb_img)
            print(f"Saved RGB image: {filename_rgb}")

        return True
    return False

def get_ned_bounds_from_coordinates(waypoints):
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    return min(xs), max(xs), min(ys), max(ys)

def search_and_land_spiral(client, ned_bounds, speed, altitude, step, max_attempts=50, x_resolution=10.0):
    min_x, max_x, min_y, max_y = ned_bounds
    home_pos = client.getMultirotorState().kinematics_estimated.position
    camera_names = ["front_center", "back_center", "front_left", "front_right", "bottom_center"]

    attempt_count = 0
    left = min_x
    right = max_x
    bottom = min_y
    top = max_y

    print(f"[INFO] Starting spiral search (max attempts = {max_attempts})")
    while left <= right and bottom <= top and attempt_count < max_attempts:
        for x in np.arange(left, right, x_resolution):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, x, bottom, altitude, speed, camera_names):
                return True
            attempt_count += 1
        bottom += step

        for y in np.arange(bottom, top, step):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, right, y, altitude, speed, camera_names):
                return True
            attempt_count += 1
        right -= x_resolution

        for x in np.arange(right, left, -x_resolution):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, x, top, altitude, speed, camera_names):
                return True
            attempt_count += 1
        top -= step

        for y in np.arange(top, bottom, -step):
            if attempt_count >= max_attempts:
                break
            if move_and_check(client, left, y, altitude, speed, camera_names):
                return True
            attempt_count += 1
        left += x_resolution

    print("[INFO] Maximum attempts reached. Target not found. Returning to home.")
    client.moveToPositionAsync(home_pos.x_val, home_pos.y_val, altitude, speed).join()
    return False

def main():
    parser = argparse.ArgumentParser(description="Spiral search for segmentation target")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four coordinates for the search boundary (x1,y1 x2,y2 x3,y3 x4,y4)")
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--max_attempts", type=int, default=50)
    parser.add_argument("--xres", type=float, default=10.0)
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1

    client = airsim.MultirotorClient(ip="172.22.112.1")
    client.confirmConnection()
    client.setSegmentationObjectID(".*", 0, True)  # Reset segmentation IDs to default
    
    # Set segmentation IDs for background and target
    set_segmentation_ids(client)
    time.sleep(1)  # let settings propagate

    # Verify segmentation image before tracking - but continue even if it fails
    segmentation_verified = verify_segmentation_works(client, camera_name="front_center")
    if not segmentation_verified:
        print("[WARNING] Segmentation verification failed, but continuing with search anyway.")
    else:
        print("[INFO] Segmentation verification passed. Proceeding with search.")

    print("Arming drone...")
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-abs(args.altitude), args.speed).join()
    time.sleep(1)

    ned_bounds = get_ned_bounds_from_coordinates(waypoints)
    found = search_and_land_spiral(
        client, ned_bounds, speed=args.speed, altitude=-abs(args.altitude),
        step=args.step, max_attempts=args.max_attempts, x_resolution=args.xres
    )

    print("Landing...")
    client.hoverAsync().join()
    time.sleep(1)
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    return 0

if __name__ == "__main__":
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
    sys.exit(main())