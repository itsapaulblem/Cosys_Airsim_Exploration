#!/usr/bin/env python3
"""
ROS2 Adaptation of searchtrack_mission_v4.py
Converts AirSim direct API calls to ROS2 mission actions and services

Original: Uses airsim.MultirotorClient() for direct vehicle control
ROS2 Version: Uses mission action servers for coordinated search operations

Ultra-Clean Architecture: /VehicleName/actions/search_area
"""

import math
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point32, Polygon
from mission_search_interfaces.action import SearchArea, NavigateToTarget
from mission_search_interfaces.srv import SetSearchPattern, GetVehicleCapabilities
from mission_search_interfaces.msg import MissionStatus, TargetDetection
import threading
from typing import List, Tuple, Optional

def parse_coordinates(coord_string: str) -> Tuple[float, float]:
    """Parse coordinate string into latitude, longitude tuple"""
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

def validate_search_box(waypoints: List[Tuple[float, float]]) -> bool:
    """Validate that search box has proper dimensions"""
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

def gps_to_ned(lat: float, lon: float, home_lat: float, home_lon: float) -> Tuple[float, float]:
    """Convert GPS coordinates to North-East-Down (NED) coordinates"""
    R = 6378137  # Earth radius in meters
    lat1 = math.radians(home_lat)
    lon1 = math.radians(home_lon)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * R
    east = dlon * R * math.cos(lat1)
    return north, east

def gps_waypoints_to_ned_polygon(waypoints: List[Tuple[float, float]], 
                                home_lat: float = 47.641468, 
                                home_lon: float = -122.140165) -> Polygon:
    """Convert GPS waypoints to NED polygon for ROS2 mission"""
    polygon = Polygon()
    
    for lat, lon in waypoints:
        north, east = gps_to_ned(lat, lon, home_lat, home_lon)
        point = Point32()
        point.x = north
        point.y = east
        point.z = 0.0
        polygon.points.append(point)
    
    return polygon

class SearchTrackMissionROS2(Node):
    """ROS2 Mission Client for Search and Track Operations"""
    
    def __init__(self, vehicle_name: str = "Droan1"):
        super().__init__('searchtrack_mission_ros2')
        
        self.vehicle_name = vehicle_name
        self.mission_complete = False
        self.target_found = False
        self.target_locations = []
        self.mission_status = None
        
        # Action clients following ultra-clean naming
        self.search_action_client = ActionClient(
            self, SearchArea, f'/{vehicle_name}/actions/search_area'
        )
        self.navigate_action_client = ActionClient(
            self, NavigateToTarget, f'/{vehicle_name}/actions/navigate_to_target'
        )
        
        # Service clients
        self.set_pattern_client = self.create_client(
            SetSearchPattern, f'/{vehicle_name}/services/set_search_pattern'
        )
        self.get_capabilities_client = self.create_client(
            GetVehicleCapabilities, f'/{vehicle_name}/services/get_capabilities'
        )
        
        # Status and detection subscribers
        self.status_subscription = self.create_subscription(
            MissionStatus, f'/{vehicle_name}/mission/status', 
            self.mission_status_callback, 10
        )
        self.detection_subscription = self.create_subscription(
            TargetDetection, f'/{vehicle_name}/detections/target',
            self.target_detection_callback, 10
        )
        
        self.get_logger().info(f"SearchTrack Mission ROS2 Client initialized for {vehicle_name}")
        self.get_logger().info("Ultra-Clean Architecture: Vehicle-Name-As-Node-Name")
        
    def mission_status_callback(self, msg: MissionStatus):
        """Handle mission status updates"""
        self.mission_status = msg
        status_names = {0: "IDLE", 1: "PLANNING", 2: "ACTIVE", 3: "PAUSED", 
                       4: "COMPLETED", 5: "FAILED", 6: "CANCELLED"}
        status_name = status_names.get(msg.status, "UNKNOWN")
        
        if msg.status in [4, 5, 6]:  # COMPLETED, FAILED, CANCELLED
            self.mission_complete = True
            
        self.get_logger().info(
            f"Mission Status: {status_name} | "
            f"Progress: {msg.progress_percent:.1f}% | "
            f"Targets: {msg.targets_detected} | "
            f"Elapsed: {msg.elapsed_time_seconds:.1f}s"
        )
        
    def target_detection_callback(self, msg: TargetDetection):
        """Handle target detection events"""
        self.target_found = True
        self.target_locations.append((msg.gps_location.latitude, msg.gps_location.longitude))
        
        self.get_logger().info("🎯 TARGET DETECTED!")
        self.get_logger().info(f"GPS: {msg.gps_location.latitude:.8f}, {msg.gps_location.longitude:.8f}")
        self.get_logger().info(f"Confidence: {msg.confidence:.2f}")
        self.get_logger().info(f"Classification: {msg.target_classification}")
        if msg.camera_name:
            self.get_logger().info(f"Detected by: {msg.camera_name}")
            
    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """Wait for vehicle services to become available"""
        self.get_logger().info("Waiting for vehicle services...")
        
        services_ready = True
        if not self.set_pattern_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"set_search_pattern service not available for {self.vehicle_name}")
            services_ready = False
            
        if not self.get_capabilities_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"get_capabilities service not available for {self.vehicle_name}")
            services_ready = False
            
        return services_ready
        
    def wait_for_action_servers(self, timeout: float = 10.0) -> bool:
        """Wait for vehicle action servers to become available"""
        self.get_logger().info("Waiting for action servers...")
        
        if not self.search_action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"search_area action server not available for {self.vehicle_name}")
            return False
            
        if not self.navigate_action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"navigate_to_target action server not available for {self.vehicle_name}")
            return False
            
        return True
        
    def get_vehicle_capabilities(self) -> Optional[dict]:
        """Query vehicle capabilities"""
        request = GetVehicleCapabilities.Request()
        
        try:
            future = self.get_capabilities_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                capabilities = {
                    'vehicle_name': response.capabilities.vehicle_name,
                    'vehicle_type': response.capabilities.vehicle_type,
                    'max_flight_time': response.capabilities.max_flight_time_minutes,
                    'max_speed': response.capabilities.max_speed_ms,
                    'max_altitude': response.capabilities.max_altitude_m,
                    'sensors': response.capabilities.available_sensors,
                    'battery': response.capabilities.current_battery_percent,
                    'status': response.capabilities.current_status
                }
                return capabilities
            else:
                self.get_logger().error("Failed to get vehicle capabilities")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting capabilities: {str(e)}")
            return None
            
    def configure_search_pattern(self, pattern_type: str = "spiral", 
                               spacing: float = 8.0, speed: float = 5.0, 
                               altitude: float = 25.0) -> bool:
        """Configure search pattern for the vehicle"""
        request = SetSearchPattern.Request()
        request.pattern_type = pattern_type
        request.pattern_spacing = spacing
        request.search_speed = speed
        request.altitude = altitude
        request.enable_detection = True
        request.confidence_threshold = 0.7
        
        try:
            future = self.set_pattern_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✅ Search pattern configured: {pattern_type}")
                    self.get_logger().info(f"Estimated coverage: {response.estimated_coverage_percent:.1f}%")
                    if response.warnings:
                        for warning in response.warnings:
                            self.get_logger().warn(f"⚠️  {warning}")
                    return True
                else:
                    self.get_logger().error(f"❌ Failed to configure pattern: {response.error_message}")
                    return False
            else:
                self.get_logger().error("Failed to configure search pattern")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error configuring pattern: {str(e)}")
            return False
            
    def execute_search_mission(self, search_polygon: Polygon, 
                             pattern: str = "spiral", altitude: float = 25.0, 
                             speed: float = 5.0, spacing: float = 8.0,
                             mission_id: str = "search_001") -> bool:
        """Execute search area mission using ROS2 action"""
        
        # Create search goal
        goal_msg = SearchArea.Goal()
        goal_msg.search_boundary = search_polygon
        goal_msg.search_pattern = pattern
        goal_msg.search_altitude = altitude
        goal_msg.search_speed = speed
        goal_msg.pattern_spacing = spacing
        goal_msg.enable_detection = True
        goal_msg.detection_confidence_threshold = 0.7
        goal_msg.mission_id = mission_id
        
        self.get_logger().info("🚁 Starting search mission...")
        self.get_logger().info(f"Pattern: {pattern} | Altitude: {altitude}m | Speed: {speed}m/s")
        
        # Send goal
        future = self.search_action_client.send_goal_async(
            goal_msg, feedback_callback=self.search_feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error("❌ Search goal was rejected or timed out")
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Search goal was rejected by action server")
            return False
            
        self.get_logger().info("✅ Search goal accepted - mission starting")
        
        # Wait for mission completion
        result_future = goal_handle.get_result_async()
        
        # Monitor mission progress
        while not self.mission_complete:
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # Check if target found during search
            if self.target_found:
                self.get_logger().info("🎯 Target detected during search!")
                # Could cancel search and proceed to investigation
                # For now, let search complete
                
        # Get final result
        if result_future.done():
            result = result_future.result()
            if result:
                self.get_logger().info("✅ Search mission completed successfully")
                self.get_logger().info(f"Targets found: {len(result.result.targets_detected)}")
                self.get_logger().info(f"Total mission time: {result.result.total_mission_time_seconds:.1f}s")
                self.get_logger().info(f"Coverage achieved: {result.result.coverage_percent:.1f}%")
                return True
            else:
                self.get_logger().error("❌ Search mission failed")
                return False
        else:
            self.get_logger().warn("⚠️  Search mission result not available")
            return False
            
    def search_feedback_callback(self, feedback_msg):
        """Handle search mission feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Search Progress: {feedback.progress_percent:.1f}% | "
            f"Current Pattern: {feedback.current_search_pattern} | "
            f"Waypoints: {feedback.waypoints_completed}/{feedback.total_waypoints}"
        )
        
    def investigate_target(self, target_lat: float, target_lon: float, 
                         mission_id: str = "investigate_001") -> bool:
        """Navigate to target location for closer investigation"""
        
        # Convert GPS to local coordinates (simplified - in real implementation, 
        # you'd get home position from vehicle status)
        home_lat, home_lon = 47.641468, -122.140165  # Default AirSim coordinates
        north, east = gps_to_ned(target_lat, target_lon, home_lat, home_lon)
        
        # Create navigation goal
        goal_msg = NavigateToTarget.Goal()
        goal_msg.target_position.x = north
        goal_msg.target_position.y = east
        goal_msg.target_position.z = 15.0  # Investigation altitude
        goal_msg.approach_altitude = 20.0
        goal_msg.approach_speed = 3.0
        goal_msg.hover_duration_seconds = 15.0  # Hover for investigation
        goal_msg.mission_id = mission_id
        
        self.get_logger().info(f"🔍 Investigating target at GPS: {target_lat:.6f}, {target_lon:.6f}")
        
        # Send navigation goal
        future = self.navigate_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None or not future.result().accepted:
            self.get_logger().error("❌ Investigation navigation goal rejected")
            return False
            
        self.get_logger().info("✅ Navigation goal accepted - proceeding to target")
        
        # Wait for navigation completion
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        if result_future.result():
            result = result_future.result().result
            self.get_logger().info("✅ Target investigation completed")
            self.get_logger().info(f"Final distance to target: {result.final_distance_to_target:.1f}m")
            return True
        else:
            self.get_logger().error("❌ Target investigation failed")
            return False
            
    def execute_full_mission(self, waypoints: List[Tuple[float, float]], 
                           altitude: float = 25.0, speed: float = 5.0, 
                           pattern: str = "spiral", spacing: float = 8.0) -> bool:
        """Execute complete search and track mission"""
        
        try:
            # 1. Wait for system readiness
            if not self.wait_for_services() or not self.wait_for_action_servers():
                return False
                
            # 2. Get vehicle capabilities
            capabilities = self.get_vehicle_capabilities()
            if capabilities:
                self.get_logger().info(f"Vehicle ready: {capabilities['vehicle_name']}")
                self.get_logger().info(f"Battery: {capabilities['battery']:.1f}%")
                self.get_logger().info(f"Max flight time: {capabilities['max_flight_time']} minutes")
            else:
                self.get_logger().warn("Could not retrieve vehicle capabilities")
                
            # 3. Configure search pattern
            if not self.configure_search_pattern(pattern, spacing, speed, altitude):
                return False
                
            # 4. Convert GPS waypoints to NED polygon
            search_polygon = gps_waypoints_to_ned_polygon(waypoints)
            
            # 5. Execute search mission
            mission_id = f"searchtrack_{int(time.time())}"
            search_success = self.execute_search_mission(
                search_polygon, pattern, altitude, speed, spacing, mission_id
            )
            
            if not search_success:
                self.get_logger().error("Search mission failed")
                return False
                
            # 6. Investigate any targets found
            if self.target_found and self.target_locations:
                self.get_logger().info(f"🎯 Investigating {len(self.target_locations)} detected targets")
                
                for i, (lat, lon) in enumerate(self.target_locations):
                    investigation_id = f"{mission_id}_investigate_{i+1}"
                    success = self.investigate_target(lat, lon, investigation_id)
                    if success:
                        self.get_logger().info(f"✅ Target {i+1} investigation completed")
                    else:
                        self.get_logger().warn(f"⚠️  Target {i+1} investigation failed")
                        
            else:
                self.get_logger().info("No targets detected during search mission")
                
            self.get_logger().info("🎉 Mission sequence completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Mission execution error: {str(e)}")
            return False

def main():
    """Main function - ROS2 adaptation of original searchtrack_mission_v4.py"""
    
    parser = argparse.ArgumentParser(description="ROS2 Search and Track Mission")
    parser.add_argument("--waypoints", type=str, nargs=4, required=True,
                        help="Four GPS coordinates for search boundary (lat1,lon1 lat2,lon2 lat3,lon3 lat4,lon4)")
    parser.add_argument("--vehicle", type=str, default="Droan1",
                        help="Vehicle name (node name in ultra-clean architecture)")
    parser.add_argument("--altitude", type=float, default=25.0,
                        help="Search altitude in meters")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Flight speed in m/s")
    parser.add_argument("--pattern", type=str, default="spiral", 
                        choices=["spiral", "grid", "lawnmower"],
                        help="Search pattern type")
    parser.add_argument("--spacing", type=float, default=8.0,
                        help="Pattern spacing in meters")
    
    args = parser.parse_args()
    
    # Parse and validate waypoints
    try:
        waypoints = [parse_coordinates(wp) for wp in args.waypoints]
        validate_search_box(waypoints)
    except ValueError as e:
        print(f"Error: {e}")
        return 1
        
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create mission client
        mission_client = SearchTrackMissionROS2(args.vehicle)
        
        print("=" * 60)
        print("🚁 ROS2 SEARCH AND TRACK MISSION")
        print("Ultra-Clean Architecture: Vehicle-Name-As-Node-Name")
        print(f"Vehicle: /{args.vehicle}")
        print(f"Pattern: {args.pattern} | Altitude: {args.altitude}m | Speed: {args.speed}m/s")
        print("=" * 60)
        
        # Execute mission
        success = mission_client.execute_full_mission(
            waypoints, args.altitude, args.speed, args.pattern, args.spacing
        )
        
        if success:
            print("🎉 Mission completed successfully!")
            return_code = 0
        else:
            print("❌ Mission failed!")
            return_code = 1
            
        # Keep node alive briefly for final status updates
        time.sleep(2.0)
        
        return return_code
        
    except KeyboardInterrupt:
        print("Mission interrupted by user")
        return 1
    except Exception as e:
        print(f"Mission error: {str(e)}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    import sys
    sys.exit(main())