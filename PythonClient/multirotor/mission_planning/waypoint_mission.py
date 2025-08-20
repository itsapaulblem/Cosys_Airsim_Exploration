#!/usr/bin/env python3
"""
GPS Waypoint Mission Script for AirSim PX4
Fly drone to specified latitude and longitude coordinates.
"""

import airsim
import argparse
import math
import time
import sys


class GPSWaypointMission:
    def __init__(self, altitude=20, speed=10, connection_timeout=60):
        """
        Initialize GPS waypoint mission.
        
        Args:
            altitude: Flight altitude in meters (AGL)
            speed: Flight speed in m/s
            connection_timeout: Connection timeout in seconds
        """
        self.altitude = altitude
        self.speed = speed
        self.connection_timeout = connection_timeout
        self.client = None
        self.origin_gps = None
        self.home_position = None
        
    def connect(self):
        """Connect to AirSim."""
        print("🔌 Connecting to AirSim...")
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            print("✅ Connected to AirSim successfully")
            
            # Get origin GPS coordinates from home position
            gps_data = self.client.getGpsData()
            self.origin_gps = {
                'latitude': gps_data.gnss.geo_point.latitude,
                'longitude': gps_data.gnss.geo_point.longitude,
                'altitude': gps_data.gnss.geo_point.altitude
            }
            
            print(f"📍 Origin GPS: {self.origin_gps['latitude']:.6f}, {self.origin_gps['longitude']:.6f}")
            
            # Get home position in NED coordinates
            self.home_position = self.client.getMultirotorState().kinematics_estimated.position
            print(f"🏠 Home NED: ({self.home_position.x_val:.2f}, {self.home_position.y_val:.2f}, {self.home_position.z_val:.2f})")
            
        except Exception as e:
            print(f"❌ Failed to connect to AirSim: {str(e)}")
            print("💡 Make sure AirSim is running and accessible")
            sys.exit(1)
    
    def gps_to_ned(self, target_lat, target_lon, target_alt=None):
        """
        Convert GPS coordinates to NED (North-East-Down) coordinates relative to origin.
        
        Args:
            target_lat: Target latitude in degrees
            target_lon: Target longitude in degrees  
            target_alt: Target altitude in meters (optional, uses flight altitude if None)
            
        Returns:
            tuple: (north, east, down) in meters
        """
        if target_alt is None:
            target_alt = self.origin_gps['altitude'] + self.altitude
        
        # Earth radius in meters
        R = 6378137.0
        
        # Convert degrees to radians
        lat1 = math.radians(self.origin_gps['latitude'])
        lon1 = math.radians(self.origin_gps['longitude'])
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)
        
        # Calculate differences
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Calculate North and East distances using equirectangular approximation
        # (Good for small distances, which is typical for drone missions)
        north = dlat * R
        east = dlon * R * math.cos((lat1 + lat2) / 2)
        
        # Calculate Down (negative of altitude difference)
        down = -(target_alt - self.origin_gps['altitude'])
        
        return north, east, down
    
    def ned_to_gps(self, north, east, down):
        """
        Convert NED coordinates to GPS coordinates.
        
        Args:
            north: North distance in meters
            east: East distance in meters
            down: Down distance in meters (negative for altitude)
            
        Returns:
            tuple: (latitude, longitude, altitude)
        """
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to GPS
        lat1 = math.radians(self.origin_gps['latitude'])
        lon1 = math.radians(self.origin_gps['longitude'])
        
        dlat = north / R
        dlon = east / (R * math.cos(lat1))
        
        target_lat = math.degrees(lat1 + dlat)
        target_lon = math.degrees(lon1 + dlon)
        target_alt = self.origin_gps['altitude'] - down
        
        return target_lat, target_lon, target_alt
    
    def print_mission_info(self, target_lat, target_lon, target_alt=None):
        """Print mission information."""
        print(f"\n📋 GPS WAYPOINT MISSION")
        print(f"Target GPS: {target_lat:.6f}, {target_lon:.6f}")
        print(f"Flight Altitude: {self.altitude}m AGL")
        print(f"Flight Speed: {self.speed}m/s")
        
        # Calculate distance and bearing
        north, east, down = self.gps_to_ned(target_lat, target_lon, target_alt)
        distance = math.sqrt(north**2 + east**2)
        bearing = math.degrees(math.atan2(east, north))
        if bearing < 0:
            bearing += 360
        
        print(f"Distance: {distance:.1f}m")
        print(f"Bearing: {bearing:.1f}° (from North)")
        print(f"NED Coordinates: ({north:.1f}, {east:.1f}, {down:.1f})")
        
        # Estimate flight time
        estimated_time = distance / self.speed
        print(f"Estimated Flight Time: {estimated_time:.1f}s ({estimated_time/60:.1f} minutes)")
    
    def arm_and_takeoff(self):
        """Arm the drone and take off to specified altitude."""
        print(f"\n🚁 Arming and taking off to {self.altitude}m...")
        
        try:
            # Enable API control
            self.client.enableApiControl(True)
            
            # Arm the drone
            print("🔐 Arming drone...")
            self.client.armDisarm(True)
            
            # Take off
            print(f"🚀 Taking off to {self.altitude}m...")
            self.client.takeoffAsync().join()
            
            # Move to target altitude
            target_z = -self.altitude  # Negative for AirSim NED coordinates
            self.client.moveToZAsync(target_z, self.speed).join()
            
            # Verify altitude
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            actual_altitude = -current_pos.z_val
            print(f"✅ Takeoff complete. Current altitude: {actual_altitude:.1f}m")
            
            # Brief pause for stabilization
            time.sleep(2)
            
        except Exception as e:
            print(f"❌ Takeoff failed: {str(e)}")
            raise
    
    def fly_to_gps(self, target_lat, target_lon, target_alt=None):
        """
        Fly to GPS coordinates.
        
        Args:
            target_lat: Target latitude in degrees
            target_lon: Target longitude in degrees
            target_alt: Target altitude in meters (optional)
        """
        print(f"\n🎯 Flying to GPS coordinates: {target_lat:.6f}, {target_lon:.6f}")
        
        try:
            # Convert GPS to NED coordinates
            north, east, down = self.gps_to_ned(target_lat, target_lon, target_alt)
            
            # Calculate absolute NED position (relative to AirSim origin)
            target_x = self.home_position.x_val + north
            target_y = self.home_position.y_val + east
            target_z = down
            
            print(f"🧭 Flying to NED position: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            
            # Fly to target position
            self.client.moveToPositionAsync(target_x, target_y, target_z, self.speed).join()
            
            # Verify arrival
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            distance_error = math.sqrt(
                (current_pos.x_val - target_x)**2 + 
                (current_pos.y_val - target_y)**2 + 
                (current_pos.z_val - target_z)**2
            )
            
            print(f"✅ Arrived at target GPS coordinates (error: {distance_error:.1f}m)")
            
            # Get current GPS position for verification
            current_gps = self.client.getGpsData()
            print(f"📍 Current GPS: {current_gps.gnss.geo_point.latitude:.6f}, {current_gps.gnss.geo_point.longitude:.6f}")
            
        except Exception as e:
            print(f"❌ Failed to fly to GPS coordinates: {str(e)}")
            raise
    
    def land_and_disarm(self):
        """Land the drone and disarm."""
        print(f"\n🛬 Landing and disarming...")
        
        try:
            # Land
            self.client.landAsync().join()
            print("✅ Landed successfully")
            
            # Disarm
            self.client.armDisarm(False)
            print("🔐 Disarmed successfully")
            
            # Disable API control
            self.client.enableApiControl(False)
            
        except Exception as e:
            print(f"❌ Landing/disarming failed: {str(e)}")
            raise
    
    def execute_mission(self, target_lat, target_lon, target_alt=None):
        """
        Execute complete GPS waypoint mission.
        
        Args:
            target_lat: Target latitude in degrees
            target_lon: Target longitude in degrees
            target_alt: Target altitude in meters (optional)
        """
        try:
            # Connect to AirSim
            self.connect(ip_address='172.30.160.1')
            
            # Print mission information
            self.print_mission_info(target_lat, target_lon, target_alt)
            
            # Arm and takeoff
            self.arm_and_takeoff()
            
            # Fly to GPS coordinates
            self.fly_to_gps(target_lat, target_lon, target_alt)
            
            # Optional: Hover at target for a few seconds
            print("⏸️  Hovering at target location for 5 seconds...")
            time.sleep(5)
            
            # Return to home (optional)
            print("🏠 Returning to home...")
            home_x = self.home_position.x_val
            home_y = self.home_position.y_val
            home_z = -self.altitude
            self.client.moveToPositionAsync(home_x, home_y, home_z, self.speed).join()
            print("✅ Returned to home position")
            
            # Land and disarm
            self.land_and_disarm()
            
            print("\n🎉 GPS waypoint mission completed successfully!")
            
        except KeyboardInterrupt:
            print("\n⚠️ Mission interrupted by user")
            self.emergency_land()
        except Exception as e:
            print(f"\n❌ Mission failed: {str(e)}")
            self.emergency_land()
            sys.exit(1)
    
    def emergency_land(self):
        """Emergency landing procedure."""
        print("🚨 Initiating emergency landing...")
        try:
            if self.client:
                self.client.landAsync().join()
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
                print("✅ Emergency landing completed")
        except Exception as e:
            print(f"❌ Emergency landing failed: {str(e)}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="GPS Waypoint Mission for AirSim PX4")
    
    # Required GPS coordinates
    parser.add_argument('latitude', type=float, help='Target latitude in degrees (e.g., 1.276870)')
    parser.add_argument('longitude', type=float, help='Target longitude in degrees (e.g., 103.858270)')
    
    # Optional parameters
    parser.add_argument('--altitude', type=float, default=20, help='Flight altitude in meters AGL (default: 20)')
    parser.add_argument('--speed', type=float, default=10, help='Flight speed in m/s (default: 10)')
    parser.add_argument('--target_altitude', type=float, help='Target altitude in meters MSL (optional)')
    parser.add_argument('--preview', action='store_true', help='Preview mission without executing')
    
    args = parser.parse_args()
    
    # Validate coordinates
    if not (-90 <= args.latitude <= 90):
        print("❌ Invalid latitude. Must be between -90 and 90 degrees.")
        sys.exit(1)
    
    if not (-180 <= args.longitude <= 180):
        print("❌ Invalid longitude. Must be between -180 and 180 degrees.")
        sys.exit(1)
    
    # Create mission
    mission = GPSWaypointMission(
        altitude=args.altitude,
        speed=args.speed
    )
    
    if args.preview:
        print("🔍 MISSION PREVIEW MODE")
        try:
            mission.connect()
            mission.print_mission_info(args.latitude, args.longitude, args.target_altitude)
            
            print(f"\n💡 To execute the mission, run:")
            print(f"python3 {sys.argv[0]} {args.latitude} {args.longitude} --altitude {args.altitude} --speed {args.speed}")
            
        except Exception as e:
            print(f"❌ Preview failed: {str(e)}")
            sys.exit(1)
    else:
        # Execute mission
        print("🚀 Starting GPS Waypoint Mission...")
        mission.execute_mission(args.latitude, args.longitude, args.target_altitude)


if __name__ == "__main__":
    main()