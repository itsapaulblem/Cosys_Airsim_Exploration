#!/usr/bin/env python3
# python3 collision_detection.py --waypoints "1.285305248758058, 103.85994577675734" "1.285305248758058, 103.85994577675734" --altitude 5 --speed 5 --hover_time 3
import math
import time
import argparse
import airsim


SAFE_DISTANCE = 1.0
ALTITUDE_STEP = 10.0
MAX_ALTITUDE = 200.0


def is_path_clear(client, safe_distance=SAFE_DISTANCE):
    lidar = client.getLidarData(lidar_name="LidarSensor1")
    if not lidar.point_cloud:
        return True

    points = lidar.point_cloud
    obstacle_count = 0

    for i in range(0, len(points), 3):
        x, y, z = points[i], points[i+1], points[i+2]
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        if z < -1.0:
            continue
        if x < 0:
            continue
        distance = math.sqrt(x**2 + y**2 + z**2)
        if distance < 0.1 or distance > 50.0:
            continue
        if distance < safe_distance:
            obstacle_count += 1
            print(f"Valid obstacle detected at distance {distance:.2f}m (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            if obstacle_count >= 3:
                return False
    return True


def move_to_waypoint_with_avoidance(client, target_x, target_y, target_z, speed):
    tolerance = 1.0
    max_attempts = 1000
    attempts = 0

    while attempts < max_attempts:
        attempts += 1
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        dx = target_x - pos.x_val
        dy = target_y - pos.y_val
        dz = target_z - pos.z_val
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        print(f"Attempt {attempts}: Distance to waypoint: {distance:.2f}m")
        if distance < tolerance:
            print("Reached waypoint")
            break
        if is_path_clear(client):
            print(f"Path clear, moving toward target...")
            if distance > 10.0:
                mag = math.sqrt(dx**2 + dy**2 + dz**2)
                if mag > 0:
                    vx = dx / mag * speed
                    vy = dy / mag * speed
                    vz = dz / mag * speed
                    step_time = min(2.0, distance / speed * 0.5)
                    client.moveByVelocityAsync(vx, vy, vz, step_time).join()
            else:
                print("Close to target, switching to position control...")
                client.moveToPositionAsync(target_x, target_y, target_z, speed * 0.5).join()
                time.sleep(1)
        else:
            current_alt = -pos.z_val
            if current_alt + ALTITUDE_STEP < MAX_ALTITUDE:
                print(f"🔼 Obstacle ahead. Climbing from {current_alt:.1f}m to {current_alt + ALTITUDE_STEP:.1f}m")
                client.moveToZAsync(pos.z_val - ALTITUDE_STEP, speed).join()
            else:
                print("🔀 Obstacle ahead. Sidestepping left to avoid.")
                client.moveByVelocityAsync(0, -speed, 0, 2.0).join()


        time.sleep(0.2)


    if attempts >= max_attempts:
        print(f"⚠️ Warning: Could not reach waypoint after {max_attempts} attempts")


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
        if not (-90 <= lat <= 90):
            raise ValueError(f"Invalid latitude: {lat}")
        if not (-180 <= lon <= 180):
            raise ValueError(f"Invalid longitude: {lon}")
        return lat, lon
    except (ValueError, IndexError) as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")


def gps_to_ned(target_lat, target_lon, home_lat, home_lon):
    R = 6378137.0
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(target_lat)
    lon2 = math.radians(target_lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    return north, east


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


def main():
    parser = argparse.ArgumentParser(description="LiDAR Collision Avoidance GPS Mission for AirSim (Cesium)")
    parser.add_argument('--waypoints', type=str, nargs='+', required=True,
                        help='GPS waypoints in format \"lat,lon\" or \"lat lon\"')
    parser.add_argument('--altitude', type=float, default=50,
                        help='Flight altitude in meters (default: 50)')
    parser.add_argument('--speed', type=float, default=5,
                        help='Flight speed in m/s (default: 5)')
    parser.add_argument('--hover_time', type=float, default=0,
                        help='Time to hover at each waypoint in seconds (default: 0 - no hovering)')
    args = parser.parse_args()


    waypoints = []
    for i, coord_str in enumerate(args.waypoints):
        lat, lon = parse_coordinates(coord_str)
        waypoints.append((lat, lon))
        print(f"Waypoint {i+1}: {lat:.6f}°, {lon:.6f}°")


    print("\n Connecting to AirSim...")
    client = airsim.MultirotorClient(ip="172.22.112.1")
    client.confirmConnection()
    print("Connected!")
    client.enableApiControl(True)
    client.armDisarm(True)


    print("Taking off...")
    client.takeoffAsync().join()
    time.sleep(1)


    target_z = -abs(args.altitude)
    print(f"Climbing to mission altitude: {abs(args.altitude)}m")
    client.moveToZAsync(target_z, args.speed)
    final_alt = print_altitude_loop(client, abs(args.altitude), mode="climb")
    print(f"Reached target altitude: {final_alt:.2f} m")


    gps_data = client.getGpsData()
    home_gps = (gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude)
    home_pos = client.getMultirotorState().kinematics_estimated.position
    print(f"Home GPS: {home_gps[0]:.6f}°, {home_gps[1]:.6f}°")


    for i, (lat, lon) in enumerate(waypoints):
        north, east = gps_to_ned(lat, lon, home_gps[0], home_gps[1])
        abs_x = home_pos.x_val + north
        abs_y = home_pos.y_val + east
        abs_z = -abs(args.altitude)


        print(f"\nNavigating to waypoint {i+1}/{len(waypoints)}: {lat:.6f}°, {lon:.6f}° (NED: {north:.1f}m N, {east:.1f}m E)")
        move_to_waypoint_with_avoidance(client, abs_x, abs_y, abs_z, args.speed)
       
        if args.hover_time > 0:
            print(f"Hovering for {args.hover_time} seconds...")
            time.sleep(args.hover_time)


    # print("\n🏁 All waypoints completed! Landing...")
    # client.moveToZAsync(1000, 1).join()
    # print_altitude_loop(client, 0, mode="descend")


    # client.armDisarm(False)
    # client.enableApiControl(False)
    # print("🏁 Mission complete!")
     # Record starting Z
    start_z = client.getMultirotorState().kinematics_estimated.position.z_val
    target_z = start_z + 100  # Move down 100 meters (NED: positive is down)
    client.moveToZAsync(target_z, args.speed)


    landed = False
    still_counter = 0
    still_threshold = 10  # Number of checks (0.5s each) to confirm stopped
    velocity_epsilon = 0.05  # m/s, threshold for "not moving"
    check_interval = 0.5


    while not landed:
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity


        # Check if moved down 100m
        if abs(pos.z_val - start_z) >= 100:
            print("✅ Drone has descended 1000 meters. Considering landed.")
            landed = True
            break


        # Check if drone is nearly stopped
        if abs(vel.x_val) < velocity_epsilon and abs(vel.y_val) < velocity_epsilon and abs(vel.z_val) < velocity_epsilon:
            still_counter += 1
        else:
            still_counter = 0


        if still_counter >= still_threshold:
            print("Drone has stopped moving. Considering landed.")
            landed = True
            break


        print(f"Descending... z={pos.z_val:.2f}, velocity=({vel.x_val:.2f}, {vel.y_val:.2f}, {vel.z_val:.2f})")
        time.sleep(check_interval)


    client.armDisarm(False)
    client.enableApiControl(False)
    print("Mission complete!")


if __name__ == "__main__":
    main()
