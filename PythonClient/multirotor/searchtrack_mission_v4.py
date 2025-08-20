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
        lat = float(parts[0].strip())
        lon = float(parts[1].strip())
        if not (-90 <= lat <= 90):
            raise ValueError("Latitude must be between -90 and 90 degrees.")
        if not (-180 <= lon <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees.")
        return lat, lon
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError(f"Search box requires exactly four GPS coordinates, got {len(waypoints)}.")
    lats = [wp[0] for wp in waypoints]
    lons = [wp[1] for wp in waypoints]
    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    if lat_range == 0 or lon_range == 0:
        raise ValueError("Search box must have non-zero area. Check your coordinates.")
    print(f"Search box dimensions: {lat_range:.6f}° lat × {lon_range:.6f}° lon")
    return True

def gps_to_ned(lat, lon, home_lat, home_lon):
    R = 6378137
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    return north, east

def detect_any_camera_non_white(client, camera_names):
    for cam in camera_names:
        response = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
        ])[0]
        if response.width == 0:
            continue
        img_bytes = response.image_data_uint8
        img1d = np.frombuffer(img_bytes, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        non_white_mask = ~((img_rgb[:, :, 0] == 255) &
                           (img_rgb[:, :, 1] == 255) &
                           (img_rgb[:, :, 2] == 255))
        if np.any(non_white_mask):
            return True, cam
    return False, None

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

def move_and_check(client, x, y, z, speed, camera_names, home_pos, home_lat, home_lon):
    print(f"Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    client.moveToPositionAsync(x, y, z, speed).join()
    time.sleep(0.5)
    found, cam = detect_any_camera_non_white(client, camera_names)
    if found:
        print(f"Target spotted by {cam} camera!")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        dx = pos.x_val - home_pos.x_val
        dy = pos.y_val - home_pos.y_val
        dlat = dx / 6378137
        dlon = dy / (6378137 * math.cos(math.radians(home_lat)))
        target_lat = home_lat + math.degrees(dlat)
        target_lon = home_lon + math.degrees(dlon)
        print(f"Target GPS Coordinates: Latitude = {target_lat:.8f}, Longitude = {target_lon:.8f}")
        response = client.simGetImage(cam, airsim.ImageType.Segmentation)
        if response:
            img_array = np.frombuffer(response, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is not None:
                filename = f"target_found_{cam}_segmentation.png"
                cv2.imwrite(filename, img)
                print(f"Saved segmentation image from {cam} camera: {filename}")
        return True
    else:
        print("No target detected in any camera.")
        return False

def search_and_land_spiral(client, ned_bounds, speed, altitude, step, max_attempts=50, x_resolution=10.0):
    min_x, max_x, min_y, max_y = ned_bounds
    gps_data = client.getGpsData()
    home_lat = gps_data.gnss.geo_point.latitude
    home_lon = gps_data.gnss.geo_point.longitude
    home_pos = client.getMultirotorState().kinematics_estimated.position
    camera_names = ["front_center", "back_center", "front_left", "front_right", "bottom_center"]
    attempt_count = 0
    left = min_x
    right = max_x
    bottom = min_y
    top = max_y
    while left <= right and bottom <= top and attempt_count < max_attempts:
        for x in np.arange(left, right, x_resolution):
            if attempt_count >= max_attempts: break
            if move_and_check(client, x, bottom, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        bottom += step
        for y in np.arange(bottom, top, step):
            if attempt_count >= max_attempts: break
            if move_and_check(client, right, y, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        right -= x_resolution
        for x in np.arange(right, left, -x_resolution):
            if attempt_count >= max_attempts: break
            if move_and_check(client, x, top, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        top -= step
        for y in np.arange(top, bottom, -step):
            if attempt_count >= max_attempts: break
            if move_and_check(client, left, y, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        left += x_resolution
    print(f"Target not found after {max_attempts} attempts. Returning to home position.")
    client.moveToPositionAsync(home_pos.x_val, home_pos.y_val, altitude, speed).join()
    return False

def print_altitude_loop(client, target_altitude, mode="climb", timeout=30):
    start_time = time.time()
    velocity_epsilon = 0.05
    still_threshold = 10
    still_counter = 0
    check_interval = 0.5
    while True:
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        current_altitude = -pos.z_val
        if mode == "climb":
            print(f"Climbing... Altitude: {current_altitude:.2f} m")
            if abs(current_altitude - abs(target_altitude)) < 0.5:
                break
        elif mode == "descend":
            print(f"Descending... Altitude: {current_altitude:.2f} m")
            if state.landed_state == airsim.LandedState.Landed:
                print("Drone has landed.")
                break
        if time.time() - start_time > timeout:
            print("Landing timeout reached. Forcing exit.")
            target_z = pos.z_val + 100
            client.moveToZAsync(target_z, 5.0).join()
            time.sleep(1)
            while True:
                state = client.getMultirotorState()
                vel = state.kinematics_estimated.linear_velocity
                if abs(vel.x_val) < velocity_epsilon and abs(vel.y_val) < velocity_epsilon and abs(vel.z_val) < velocity_epsilon:
                    still_counter += 1
                else:
                    still_counter = 0
                if still_counter >= still_threshold:
                    print("Drone has stopped moving. Considering landed.")
                    break
                print(f"Waiting... velocity=({vel.x_val:.2f}, {vel.y_val:.2f}, {vel.z_val:.2f})")
                time.sleep(check_interval)
            client.armDisarm(False)
            print("Drone forcibly disarmed after drop and full stop.")
            break
        time.sleep(check_interval)
    return current_altitude

def main():
    parser = argparse.ArgumentParser(description="Spiral search mission using all cameras")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four GPS coordinates for the search boundary (lat1,lon1 lat2,lon2 lat3,lon3 lat4,lon4)")
    parser.add_argument("--altitude", type=float, default=2.0,
                        help="Search altitude in meters")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Flight speed in m/s")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Distance between spiral layers in meters")
    parser.add_argument("--max_attempts", type=int, default=50,
                        help="Maximum number of scan attempts")
    parser.add_argument("--xres", type=float, default=10.0,
                        help="Horizontal resolution in meters")
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1

    client = None
    try:
        client = airsim.MultirotorClient(ip="172.22.112.1")
        client.confirmConnection()

        print("Arming drone...")
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()

        target_z = -abs(args.altitude)
        client.moveToZAsync(target_z, args.speed).join()
        print_altitude_loop(client, abs(args.altitude), mode="climb")
        print(f"Reached target altitude: {abs(args.altitude):.2f} m")
        time.sleep(1)

        ned_bounds = get_ned_bounds_from_gps(waypoints, client)
        found = search_and_land_spiral(
            client, ned_bounds, speed=args.speed, altitude=target_z,
            step=args.step, max_attempts=args.max_attempts, x_resolution=args.xres
        )

        if found:
            print("Target found. Landing...")
        else:
            print("Landing after search completed.")

        client.hoverAsync().join()
        time.sleep(1)
        client.landAsync().join()
        print_altitude_loop(client, 0, mode="descend")

        client.armDisarm(False)
        client.enableApiControl(False)
        return 0

    except Exception as e:
        print(f"An error occurred: {e}")
        if client:
            try:
                client.armDisarm(False)
                client.enableApiControl(False)
            except:
                pass
        return 1

if __name__ == "__main__":
    sys.exit(main())
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
        lat = float(parts[0].strip())
        lon = float(parts[1].strip())
        if not (-90 <= lat <= 90):
            raise ValueError("Latitude must be between -90 and 90 degrees.")
        if not (-180 <= lon <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees.")
        return lat, lon
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError(f"Search box requires exactly four GPS coordinates, got {len(waypoints)}.")
    lats = [wp[0] for wp in waypoints]
    lons = [wp[1] for wp in waypoints]
    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    if lat_range == 0 or lon_range == 0:
        raise ValueError("Search box must have non-zero area. Check your coordinates.")
    print(f"Search box dimensions: {lat_range:.6f}° lat × {lon_range:.6f}° lon")
    return True

def gps_to_ned(lat, lon, home_lat, home_lon):
    R = 6378137
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    return north, east

def detect_any_camera_non_white(client, camera_names):
    for cam in camera_names:
        response = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)
        ])[0]
        if response.width == 0:
            continue
        img_bytes = response.image_data_uint8
        img1d = np.frombuffer(img_bytes, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        non_white_mask = ~((img_rgb[:, :, 0] == 255) &
                           (img_rgb[:, :, 1] == 255) &
                           (img_rgb[:, :, 2] == 255))
        if np.any(non_white_mask):
            return True, cam
    return False, None

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

def move_and_check(client, x, y, z, speed, camera_names, home_pos, home_lat, home_lon):
    print(f"Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    client.moveToPositionAsync(x, y, z, speed).join()
    time.sleep(0.5)
    found, cam = detect_any_camera_non_white(client, camera_names)
    if found:
        print(f"Target spotted by {cam} camera!")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        dx = pos.x_val - home_pos.x_val
        dy = pos.y_val - home_pos.y_val
        dlat = dx / 6378137
        dlon = dy / (6378137 * math.cos(math.radians(home_lat)))
        target_lat = home_lat + math.degrees(dlat)
        target_lon = home_lon + math.degrees(dlon)
        print(f"Target GPS Coordinates: Latitude = {target_lat:.8f}, Longitude = {target_lon:.8f}")
        response = client.simGetImage(cam, airsim.ImageType.Segmentation)
        if response:
            img_array = np.frombuffer(response, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is not None:
                filename = f"target_found_{cam}_segmentation.png"
                cv2.imwrite(filename, img)
                print(f"Saved segmentation image from {cam} camera: {filename}")
        return True
    else:
        print("No target detected in any camera.")
        return False

def search_and_land_spiral(client, ned_bounds, speed, altitude, step, max_attempts=50, x_resolution=10.0):
    min_x, max_x, min_y, max_y = ned_bounds
    gps_data = client.getGpsData()
    home_lat = gps_data.gnss.geo_point.latitude
    home_lon = gps_data.gnss.geo_point.longitude
    home_pos = client.getMultirotorState().kinematics_estimated.position
    camera_names = ["front_center", "back_center", "front_left", "front_right", "bottom_center"]
    attempt_count = 0
    left = min_x
    right = max_x
    bottom = min_y
    top = max_y
    while left <= right and bottom <= top and attempt_count < max_attempts:
        for x in np.arange(left, right, x_resolution):
            if attempt_count >= max_attempts: break
            if move_and_check(client, x, bottom, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        bottom += step
        for y in np.arange(bottom, top, step):
            if attempt_count >= max_attempts: break
            if move_and_check(client, right, y, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        right -= x_resolution
        for x in np.arange(right, left, -x_resolution):
            if attempt_count >= max_attempts: break
            if move_and_check(client, x, top, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        top -= step
        for y in np.arange(top, bottom, -step):
            if attempt_count >= max_attempts: break
            if move_and_check(client, left, y, altitude, speed, camera_names, home_pos, home_lat, home_lon): return True
            attempt_count += 1
        left += x_resolution
    print(f"Target not found after {max_attempts} attempts. Returning to home position.")
    client.moveToPositionAsync(home_pos.x_val, home_pos.y_val, altitude, speed).join()
    return False

def print_altitude_loop(client, target_altitude, mode="climb", timeout=30):
    start_time = time.time()
    velocity_epsilon = 0.05
    still_threshold = 10
    still_counter = 0
    check_interval = 0.5
    while True:
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        current_altitude = -pos.z_val
        if mode == "climb":
            print(f"Climbing... Altitude: {current_altitude:.2f} m")
            if abs(current_altitude - abs(target_altitude)) < 0.5:
                break
        elif mode == "descend":
            print(f"Descending... Altitude: {current_altitude:.2f} m")
            if state.landed_state == airsim.LandedState.Landed:
                print("Drone has landed.")
                break
        if time.time() - start_time > timeout:
            print("Landing timeout reached. Forcing exit.")
            target_z = pos.z_val + 100
            client.moveToZAsync(target_z, 5.0).join()
            time.sleep(1)
            while True:
                state = client.getMultirotorState()
                vel = state.kinematics_estimated.linear_velocity
                if abs(vel.x_val) < velocity_epsilon and abs(vel.y_val) < velocity_epsilon and abs(vel.z_val) < velocity_epsilon:
                    still_counter += 1
                else:
                    still_counter = 0
                if still_counter >= still_threshold:
                    print("Drone has stopped moving. Considering landed.")
                    break
                print(f"Waiting... velocity=({vel.x_val:.2f}, {vel.y_val:.2f}, {vel.z_val:.2f})")
                time.sleep(check_interval)
            client.armDisarm(False)
            print("Drone forcibly disarmed after drop and full stop.")
            break
        time.sleep(check_interval)
    return current_altitude

def main():
    parser = argparse.ArgumentParser(description="Spiral search mission using all cameras")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four GPS coordinates for the search boundary (lat1,lon1 lat2,lon2 lat3,lon3 lat4,lon4)")
    parser.add_argument("--altitude", type=float, default=2.0,
                        help="Search altitude in meters")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Flight speed in m/s")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Distance between spiral layers in meters")
    parser.add_argument("--max_attempts", type=int, default=50,
                        help="Maximum number of scan attempts")
    parser.add_argument("--xres", type=float, default=10.0,
                        help="Horizontal resolution in meters")
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1

    client = None
    try:
        client = airsim.MultirotorClient(ip="172.22.112.1")
        client.confirmConnection()

        print("Arming drone...")
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()

        target_z = -abs(args.altitude)
        client.moveToZAsync(target_z, args.speed).join()
        print_altitude_loop(client, abs(args.altitude), mode="climb")
        print(f"Reached target altitude: {abs(args.altitude):.2f} m")
        time.sleep(1)

        ned_bounds = get_ned_bounds_from_gps(waypoints, client)
        found = search_and_land_spiral(
            client, ned_bounds, speed=args.speed, altitude=target_z,
            step=args.step, max_attempts=args.max_attempts, x_resolution=args.xres
        )

        if found:
            print("Target found. Landing...")
        else:
            print("Landing after search completed.")

        client.hoverAsync().join()
        time.sleep(1)
        client.landAsync().join()
        print_altitude_loop(client, 0, mode="descend")

        client.armDisarm(False)
        client.enableApiControl(False)
        return 0

    except Exception as e:
        print(f"An error occurred: {e}")
        if client:
            try:
                client.armDisarm(False)
                client.enableApiControl(False)
            except:
                pass
        return 1

if __name__ == "__main__":
    sys.exit(main())
>>>>>>> 475892e9e9fca2f32e65cef95afb59f60bb0718a
