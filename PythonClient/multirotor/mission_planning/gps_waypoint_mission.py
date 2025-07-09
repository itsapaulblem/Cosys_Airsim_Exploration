#!/usr/bin/env python3
import math
import time
import argparse
import airsim

SAFE_DISTANCE = 10.0  # Minimum safe distance to avoid collisions
ALTITUDE_STEP = 10.0 
MAX_ALTITUDE = 200.0  # Maximum altitude limit

def is_path_clear(client, safe_distance):
    lidar = client.getLidarData(lidar_name="LidarSensor1")
    if not lidar.point_cloud:
        return True
    points = lidar.point_cloud
    for i in range(0, len(points), 3):
        x, y, z = points[i], points[i+1], points[i+2]
        distance = math.sqrt(x**2 + y**2 + z**2)
        if distance < safe_distance:
            print(f"⚠️ Obstacle detected at distance {distance:.2f}m, path is not clear!")
            return False
    return True

def parse_coordinates(coord_string):
    """Parse coordinate string in format 'lat,lon' or 'lat lon'."""
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
            raise ValueError(f"Invalid latitude: {lat} (must be between -90 and 90)")
        if not (-180 <= lon <= 180):
            raise ValueError(f"Invalid longitude: {lon} (must be between -180 and 180)")
        return lat, lon
    except (ValueError, IndexError) as e:
        raise ValueError(f"Invalid coordinate format '{coord_string}': {str(e)}")

def gps_to_ned(target_lat, target_lon, home_lat, home_lon):
    """Convert GPS coordinates to NED (North-East-Down) coordinates relative to home."""
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
    """Continuously print altitude until target is reached (for climb or descend)."""
    while True:
        state = client.getMultirotorState()
        current_altitude = -state.kinematics_estimated.position.z_val
        if mode == "climb":
            print(f"🛫 Climbing... Altitude: {current_altitude:.2f} m")
            if abs(current_altitude - abs(target_altitude)) < 0.5:
                break
        elif mode == "descend":
            print(f"🛬 Descending... Altitude: {current_altitude:.2f} m")
            if state.landed_state == airsim.LandedState.Landed:
                break
        time.sleep(0.5)
    return current_altitude

def main():
    parser = argparse.ArgumentParser(description="Simple GPS Waypoint Mission for AirSim")
    parser.add_argument('--waypoints', type=str, nargs='+', required=True,
                        help='GPS waypoints in format "lat,lon" or "lat lon"')
    parser.add_argument('--altitude', type=float, default=50,
                        help='Flight altitude in meters (default: 50)')
    parser.add_argument('--speed', type=float, default=5,
                        help='Flight speed in m/s (default: 5)')
    parser.add_argument('--hover_time', type=float, default=5,
                        help='Time to hover at each waypoint in seconds (default: 5)')
    args = parser.parse_args()

    # Parse waypoints
    waypoints = []
    for i, coord_str in enumerate(args.waypoints):
        lat, lon = parse_coordinates(coord_str)
        waypoints.append((lat, lon))
        print(f"✅ Waypoint {i+1}: {lat:.6f}°, {lon:.6f}°")

    # Connect to AirSim
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient(ip="172.22.112.1")
    client.confirmConnection()
    print("Connected!")
    client.enableApiControl(True)
    client.armDisarm(True)

    # Takeoff
    print("🚁 Taking off...")
    client.takeoffAsync().join()
    time.sleep(1)

    # Move to the specified altitude before starting the mission
    target_z = -abs(args.altitude)  # AirSim NED: z is negative up
    print(f"⬆️  Climbing to mission altitude: {abs(args.altitude)}m")
    client.moveToZAsync(target_z, args.speed)
    # Print altitude continuously during climb
    final_alt = print_altitude_loop(client, abs(args.altitude), mode="climb")
    print(f"🛫 Reached target altitude: {final_alt:.2f} m")

    # Get home GPS and position
    gps_data = client.getGpsData()
    home_gps = (gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude)
    home_pos = client.getMultirotorState().kinematics_estimated.position

    print(f"🏠 Home GPS: {home_gps[0]:.6f}°, {home_gps[1]:.6f}°")

    # Fly to each waypoint
    for i, (lat, lon) in enumerate(waypoints):
        north, east = gps_to_ned(lat, lon, home_gps[0], home_gps[1])
        abs_x = home_pos.x_val + north
        abs_y = home_pos.y_val + east
        current_altitude = abs(args.altitude)
        abs_z = -abs(current_altitude)  # AirSim NED: z is negative up

        print(f"\n🎯 Flying to waypoint {i+1}: {lat:.6f}°, {lon:.6f}° (NED: {north:.1f}m N, {east:.1f}m E) at {current_altitude}m altitude")
        client.moveToPositionAsync(abs_x, abs_y, abs_z, args.speed).join()
        print(f"  ⏱️  Hovering for {args.hover_time}s")
        time.sleep(args.hover_time)

        # Only check for obstacles after leaving home and before the last waypoint
        if i > 0 and i < len(waypoints) - 1:
            obstacle_check_count = 0
            max_obstacle_checks = 5  # Limit the number of altitude increases

            while not is_path_clear(client, SAFE_DISTANCE) and obstacle_check_count < max_obstacle_checks:
                if current_altitude >= MAX_ALTITUDE:
                    print(f"⚠️ Maximum altitude {MAX_ALTITUDE}m reached, proceeding anyway")
                    break
                print(f"⚠️ Obstacle detected! Increasing altitude to {current_altitude + ALTITUDE_STEP}m")
                current_altitude += ALTITUDE_STEP
                abs_z = -abs(current_altitude)
                client.moveToPositionAsync(
                    client.getMultirotorState().kinematics_estimated.position.x_val,
                    client.getMultirotorState().kinematics_estimated.position.y_val,
                    abs_z, args.speed
                ).join()
                time.sleep(2)
                obstacle_check_count += 1

    # Land at the last waypoint
    print("🎯 Reached last waypoint, landing...")
    client.landAsync()
    # Print altitude continuously during landing
    landed_altitude = print_altitude_loop(client, 0, mode="descend")
    print(f"🛬 Landed! Final altitude: {landed_altitude:.2f} m")

    client.armDisarm(False)
    client.enableApiControl(False)
    print("Landed! Mission complete✅")

if __name__ == "__main__":
    main()