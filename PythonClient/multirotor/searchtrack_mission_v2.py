#!/usr/bin/env python3
import math
import time
import argparse
import numpy as np
import setup_path
import cosysairsim as airsim
import sys

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
    R = 6378137  # Earth radius in meters
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    difflat = lat2 - lat1
    difflon = lon2 - lon1
    north = difflat * R
    east = difflon * R * math.cos(lat1)
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

        non_white_mask = ~((img_rgb[:, :, 0] == 255) & 
                           (img_rgb[:, :, 1] == 255) & 
                           (img_rgb[:, :, 2] == 255))
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

def search_and_land(client, ned_bounds, speed, altitude, step):
    min_x, max_x, min_y, max_y = ned_bounds
    direction = 1
    y = min_y
    camera_names = ["bottom", "front", "back", "left", "right"]

    while y <= max_y:
        x_start, x_end = (min_x, max_x) if direction == 1 else (max_x, min_x)
        for x in np.linspace(x_start, x_end, num=2):
            print(f"Moving to ({x:.1f}, {y:.1f}, {altitude:.1f})")
            client.moveToPositionAsync(x, y, altitude, speed).join()
            time.sleep(1)

            if detect_any_non_white(client, camera_names):
                print("Target spotted! Landing on it now.")
                client.hoverAsync().join()
                time.sleep(1)
                precision_land_on_target(client)
                return True
            else:
                print("No target detected, continuing search...")

        y += step
        direction *= -1
        time.sleep(0.5)

    return False

def print_altitude_loop(client, target_altitude, mode="climb"):
    while True:
        state = client.getMultirotorState()
        current_altitude = -state.kinematics_estimated.position.z_val
        if mode == "climb":
            print(f"Climbing... Altitude: {current_altitude:.2f} m")
            if abs(current_altitude - abs(target_altitude)) < 0.5:
                break
        elif mode == "descend":
            print(f"Descending... Altitude: {current_altitude:.2f} m")
            if state.landed_state == airsim.LandedState.Landed:
                print("Drone has landed.")
                break
        time.sleep(0.5)
    return current_altitude

def precision_land_on_target(client, descend_distance=100, speed=1.0):
    print("Initiating precision landing on target...")
    state = client.getMultirotorState()
    start_z = state.kinematics_estimated.position.z_val
    current_pos = state.kinematics_estimated.position
    descend_steps = 10
    step_distance = descend_distance / descend_steps

    for step in range(descend_steps):
        target_z = start_z + (step + 1) * step_distance
        client.moveToZAsync(target_z, speed).join()
        print(f"Descending to: {-target_z:.2f} m")
        time.sleep(0.5)

    print("Final landing phase...")
    client.landAsync().join()
    print_altitude_loop(client, 0, mode="descend")

def main():
    parser = argparse.ArgumentParser(description="Search and track mission for drone search")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four GPS coordinates for the search boundary (lat1,lon1 lat2,lon2 lat3,lon3 lat4,lon4)")
    parser.add_argument("--altitude", type=float, default=5.0,
                        help="Altitude for the search mission in meters")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Speed for the search mission in m/s")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Distance between search lines in meters")
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
        found = search_and_land(
            client, ned_bounds, speed=args.speed, altitude=target_z, step=args.step
        )

        if found:
            print("Target found and landed successfully.")
        else:
            print("Target not found in the search area.")

        client.armDisarm(False)
        client.enableApiControl(False)
        return 0 if found else 1

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
