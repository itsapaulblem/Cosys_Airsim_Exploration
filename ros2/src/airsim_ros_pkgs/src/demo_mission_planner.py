#!/usr/bin/env python3
"""
Demonstration Mission Planner for Ultra-Clean Architecture
Shows how to create mission plans using ROS2 interfaces
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32, Point
from mission_search_interfaces.msg import (
    MissionPlan, SearchZone, VehicleCapabilities, TargetDetection
)
from mission_search_interfaces.srv import PlanMission, AssignSearchZone
import random
import math

class DemoMissionPlanner(Node):
    def __init__(self):
        super().__init__('demo_mission_planner')
        
        # Ultra-clean architecture: Service clients follow /VehicleName/services/ pattern
        self.vehicle_names = ['Droan1', 'PX4_Drone2', 'SimpleFlight3']
        self.service_clients = {}
        
        # Create service clients for each vehicle
        for vehicle_name in self.vehicle_names:
            service_name = f'/{vehicle_name}/services/set_search_pattern'
            # Note: In full implementation, these would be proper service clients
            self.get_logger().info(f"Would connect to service: {service_name}")
        
        self.get_logger().info("Demo Mission Planner initialized - Ultra-Clean Architecture")
        
    def create_demo_search_zones(self) -> list:
        """Create demonstration search zones for multi-vehicle missions"""
        
        zones = []
        
        # Zone 1: Urban area (high priority)
        zone1 = SearchZone()
        zone1.zone_id = "urban_sector_1"
        zone1.priority = 1  # 1 = highest priority
        zone1.boundary_polygon.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=50.0, y=0.0, z=0.0),
            Point32(x=50.0, y=50.0, z=0.0),
            Point32(x=0.0, y=50.0, z=0.0)
        ]
        zone1.search_pattern = "grid"
        zone1.min_altitude = 25.0
        zone1.max_altitude = 35.0
        zone1.terrain_difficulty = 2
        zone1.area_sq_m = 2500.0
        zones.append(zone1)
        
        # Zone 2: Forest area (medium priority)
        zone2 = SearchZone()
        zone2.zone_id = "forest_sector_2"  
        zone2.priority = 2
        zone2.boundary_polygon.points = [
            Point32(x=60.0, y=0.0, z=0.0),
            Point32(x=110.0, y=0.0, z=0.0),
            Point32(x=110.0, y=40.0, z=0.0),
            Point32(x=60.0, y=40.0, z=0.0)
        ]
        zone2.search_pattern = "spiral"
        zone2.min_altitude = 35.0
        zone2.max_altitude = 45.0
        zone2.terrain_difficulty = 3
        zone2.area_sq_m = 2000.0
        zones.append(zone2)
        
        # Zone 3: Open field (lower priority)
        zone3 = SearchZone()
        zone3.zone_id = "field_sector_3"
        zone3.priority = 3
        zone3.boundary_polygon.points = [
            Point32(x=0.0, y=60.0, z=0.0),
            Point32(x=40.0, y=60.0, z=0.0),
            Point32(x=40.0, y=100.0, z=0.0),
            Point32(x=0.0, y=100.0, z=0.0)
        ]
        zone3.search_pattern = "lawnmower"
        zone3.min_altitude = 20.0
        zone3.max_altitude = 30.0
        zone3.terrain_difficulty = 1
        zone3.area_sq_m = 1600.0
        zones.append(zone3)
        
        return zones
    
    def create_demo_vehicle_capabilities(self) -> dict:
        """Create demonstration vehicle capabilities for different vehicle types"""
        
        capabilities = {}
        
        # Droan1 - Advanced multirotor with good sensors
        cap1 = VehicleCapabilities()
        cap1.vehicle_name = "Droan1"
        cap1.vehicle_type = "multirotor"
        cap1.max_flight_time_minutes = 25
        cap1.max_speed_ms = 15.0
        cap1.max_altitude_m = 120.0
        cap1.available_sensors = ["camera_rgb", "camera_depth", "lidar", "imu", "gps"]
        cap1.has_thermal_camera = False
        cap1.has_zoom_camera = True
        cap1.communication_range_m = 5000.0
        cap1.current_battery_percent = 95.0
        cap1.is_available_for_mission = True
        cap1.current_status = "ready"
        capabilities["Droan1"] = cap1
        
        # PX4_Drone2 - Professional drone with thermal imaging
        cap2 = VehicleCapabilities()
        cap2.vehicle_name = "PX4_Drone2"
        cap2.vehicle_type = "px4_multirotor"
        cap2.max_flight_time_minutes = 30
        cap2.max_speed_ms = 12.0
        cap2.max_altitude_m = 100.0
        cap2.available_sensors = ["camera_rgb", "thermal_camera", "imu", "gps", "magnetometer"]
        cap2.has_thermal_camera = True
        cap2.has_zoom_camera = False
        cap2.communication_range_m = 8000.0
        cap2.current_battery_percent = 88.0
        cap2.is_available_for_mission = True
        cap2.current_status = "ready"
        capabilities["PX4_Drone2"] = cap2
        
        # SimpleFlight3 - Basic drone for simple tasks
        cap3 = VehicleCapabilities()
        cap3.vehicle_name = "SimpleFlight3"
        cap3.vehicle_type = "simpleflight"
        cap3.max_flight_time_minutes = 20
        cap3.max_speed_ms = 8.0
        cap3.max_altitude_m = 80.0
        cap3.available_sensors = ["camera_rgb", "imu", "gps"]
        cap3.has_thermal_camera = False
        cap3.has_zoom_camera = False
        cap3.communication_range_m = 3000.0
        cap3.current_battery_percent = 92.0
        cap3.is_available_for_mission = True
        cap3.current_status = "ready"
        capabilities["SimpleFlight3"] = cap3
        
        return capabilities
    
    def optimize_zone_assignments(self, zones: list, capabilities: dict) -> dict:
        """Demonstrate intelligent zone assignment based on vehicle capabilities"""
        
        assignments = {}
        
        # Sort zones by priority (high to low)
        zones_sorted = sorted(zones, key=lambda z: z.priority, reverse=False)  # 1=highest
        
        # Sort vehicles by capability score
        def capability_score(vehicle_name):
            cap = capabilities[vehicle_name]
            # Score based on flight time, sensors, and battery
            score = cap.max_flight_time_minutes * 2
            score += len(cap.available_sensors) * 5
            score += cap.current_battery_percent * 0.5
            if cap.has_thermal_camera:
                score += 20
            return score
        
        vehicles_sorted = sorted(capabilities.keys(), key=capability_score, reverse=True)
        
        self.get_logger().info("🎯 Optimized Zone Assignments:")
        
        # Assign zones to vehicles
        for i, zone in enumerate(zones_sorted):
            if i < len(vehicles_sorted):
                vehicle = vehicles_sorted[i]
                assignments[zone.zone_id] = vehicle
                
                cap = capabilities[vehicle]
                estimated_time_minutes = zone.area_sq_m / 200  # Rough estimate
                estimated_battery_usage = (estimated_time_minutes / cap.max_flight_time_minutes) * 100
                
                self.get_logger().info(
                    f"  🚁 {vehicle} → {zone.zone_id} "
                    f"(Priority: {zone.priority}, Pattern: {zone.search_pattern}, "
                    f"Est. Battery: {estimated_battery_usage:.1f}%)"
                )
        
        return assignments
    
    def create_mission_plan(self) -> MissionPlan:
        """Create a comprehensive demonstration mission plan"""
        
        # Get demo data
        zones = self.create_demo_search_zones()
        capabilities = self.create_demo_vehicle_capabilities()
        assignments = self.optimize_zone_assignments(zones, capabilities)
        
        # Create mission plan
        mission = MissionPlan()
        mission.mission_id = f"search_rescue_demo_{int(rclpy.clock.Clock().now().nanoseconds / 1e9)}"
        mission.mission_name = "Urban Search & Rescue Demonstration"
        mission.mission_description = "Multi-vehicle coordinated search demonstrating ultra-clean architecture"
        mission.priority_level = 3
        mission.search_zones = zones
        
        # Add participating vehicles
        for vehicle_name in assignments.values():
            if vehicle_name not in mission.participating_vehicles:
                mission.participating_vehicles.append(vehicle_name)
        
        # Calculate total mission time
        total_time_minutes = max(zone.area_sq_m / 200 for zone in zones)  # Rough estimate
        mission.estimated_duration_minutes = total_time_minutes + 10  # Add buffer
        
        return mission
    
    def demonstrate_mission_planning(self):
        """Main demonstration function"""
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚁 MISSION PLANNING DEMONSTRATION")
        self.get_logger().info("Ultra-Clean Architecture: Vehicle-Name-As-Node-Name")
        self.get_logger().info("=" * 60)
        
        # Create and display mission plan
        mission = self.create_mission_plan()
        
        self.get_logger().info(f"📋 Mission: {mission.mission_name}")
        self.get_logger().info(f"🆔 ID: {mission.mission_id}")
        self.get_logger().info(f"⏱️  Duration: {mission.estimated_duration_minutes} minutes")
        self.get_logger().info(f"🚁 Vehicles: {', '.join(mission.participating_vehicles)}")
        self.get_logger().info(f"📍 Search Zones: {len(mission.search_zones)}")
        
        self.get_logger().info("\n🎯 Ultra-Clean Communication Patterns:")
        for vehicle in mission.participating_vehicles:
            self.get_logger().info(f"  • Node: /{vehicle}")
            self.get_logger().info(f"    - Actions: /{vehicle}/actions/search_area")
            self.get_logger().info(f"    - Services: /{vehicle}/services/set_search_pattern")
            self.get_logger().info(f"    - Topics: /{vehicle}/mission/status, /{vehicle}/detections/target")
        
        self.get_logger().info("\n✅ Mission planning demonstration complete!")
        self.get_logger().info("🔥 Ready for execution with ultra-clean ROS2 architecture!")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = DemoMissionPlanner()
        planner.demonstrate_mission_planning()
        
        # Keep node alive briefly to show logs
        rclpy.spin_once(planner, timeout_sec=2.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()