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
        parts = coord_string.replace(',', ' ').split()
        if len(parts) != 2:
            raise ValueError("Invalid coordinate format. Use 'x,y' or 'x y'.")
        return float(parts[0]), float(parts[1])
    except Exception as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def validate_search_box(waypoints):
    if len(waypoints) != 4:
        raise ValueError("Search box requires exactly four coordinates.")
    xs, ys = zip(*waypoints)
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    if x_range == 0 or y_range == 0:
        raise ValueError("Search box must have non-zero area.")
    print(f"Search box dimensions: {x_range:.2f}m × {y_range:.2f}m")
    return True

def detect_hot_target(client, camera_names, threshold=200):
    for cam in camera_names:
        response = client.simGetImages([airsim.ImageRequest(cam, 9, False, False)])[0]
        if response.width == 0:
            continue
        img = np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(response.height, response.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        cv2.imwrite(f"thermal_debug_{cam}.png", gray)
        print(f"Thermal stats [{cam}]: min={gray.min()}, max={gray.max()}, mean={gray.mean():.1f}")
        if np.any(gray > threshold):
            return True, cam
    return False, None

def move_and_check(client, x, y, z, speed, camera_names, threshold):
    print(f"Moving to ({x:.1f}, {y:.1f}, {z:.1f})")
    client.moveToPositionAsync(x, y, z, speed).join()
    time.sleep(0.5)
    found, cam = detect_hot_target(client, camera_names, threshold)
    if found:
        print(f"Hot target spotted by {cam} camera!")
        pos = client.getMultirotorState().kinematics_estimated.position
        print(f"Target Position: X = {pos.x_val:.2f}, Y = {pos.y_val:.2f}, Z = {pos.z_val:.2f}")

        responses = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Infrared, False, False),
            airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)
        ])
        thermal, rgb = responses
        if thermal.width:
            thermal_img = np.frombuffer(thermal.image_data_uint8, dtype=np.uint8).reshape(thermal.height, thermal.width, 3)
            cv2.imwrite(f"target_{cam}_thermal.png", thermal_img)
        if rgb.width:
            rgb_img = np.frombuffer(rgb.image_data_uint8, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
            cv2.imwrite(f"target_{cam}_rgb.png", rgb_img)
        return True
    return False

def get_ned_bounds_from_coordinates(waypoints):
    xs, ys = zip(*waypoints)
    return min(xs), max(xs), min(ys), max(ys)

def search_and_land_spiral(client, bounds, speed, alt, step, max_attempts, xres, threshold):
    min_x, max_x, min_y, max_y = bounds
    home = client.getMultirotorState().kinematics_estimated.position
    cams = ["thermal_front_center", "thermal_back_center", "thermal_front_left", "thermal_front_right", "thermal_bottom_center"]
    attempts = 0
    l, r, b, t = min_x, max_x, min_y, max_y
    print("[INFO] Starting spiral search...")

    while l <= r and b <= t and attempts < max_attempts:
        for x in np.arange(l, r, xres):
            if attempts >= max_attempts or move_and_check(client, x, b, alt, speed, cams, threshold): return True
            attempts += 1
        b += step
        for y in np.arange(b, t, step):
            if attempts >= max_attempts or move_and_check(client, r, y, alt, speed, cams, threshold): return True
            attempts += 1
        r -= xres
        for x in np.arange(r, l, -xres):
            if attempts >= max_attempts or move_and_check(client, x, t, alt, speed, cams, threshold): return True
            attempts += 1
        t -= step
        for y in np.arange(t, b, -step):
            if attempts >= max_attempts or move_and_check(client, l, y, alt, speed, cams, threshold): return True
            attempts += 1
        l += xres

    print("[INFO] Search complete. Returning to home.")
    client.moveToPositionAsync(home.x_val, home.y_val, alt, speed).join()
    return False

def print_altitude_loop(client, target_alt, mode="climb", timeout=30):
    start = time.time()
    interval = 0.5
    while True:
        state = client.getMultirotorState()
        alt = -state.kinematics_estimated.position.z_val
        if (mode == "climb" and abs(alt - abs(target_alt)) < 0.5) or \
           (mode == "descend" and state.landed_state == airsim.LandedState.Landed):
            print("Reached target altitude." if mode == "climb" else "Drone landed.")
            break
        if time.time() - start > timeout:
            print("[WARNING] Altitude operation timeout.")
            break
        time.sleep(interval)
    return alt

def main():
    parser = argparse.ArgumentParser(description="Thermal spiral search for hot targets")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True, help="Four coordinates: 'x1,y1 x2,y2 x3,y3 x4,y4'")
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--max_attempts", type=int, default=50)
    parser.add_argument("--xres", type=float, default=10.0)
    parser.add_argument("--threshold", type=int, default=200)
    args = parser.parse_args()

    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"[ERROR] {e}")
        return 1

    client = airsim.MultirotorClient(ip="172.22.112.1")
    client.confirmConnection()
    print("[INFO] Arming drone...")
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    target_z = -abs(args.altitude)
    client.moveToZAsync(target_z, args.speed).join()
    print_altitude_loop(client, args.altitude, "climb")
    time.sleep(1)

    bounds = get_ned_bounds_from_coordinates(waypoints)
    found = search_and_land_spiral(client, bounds, args.speed, target_z, args.step, args.max_attempts, args.xres, args.threshold)

    print("[INFO] Landing...")
    client.hoverAsync().join()
    time.sleep(1)
    client.landAsync().join()
    print_altitude_loop(client, 0, "descend")
    client.armDisarm(False)
    client.enableApiControl(False)
    return 0

if __name__ == "__main__":
    sys.exit(main())
