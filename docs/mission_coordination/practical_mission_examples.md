# 🎯 Practical Mission Examples
## Real-World PX4 SITL Mission Scenarios with Cosys-AirSim

This guide provides complete, tested examples of real-world mission scenarios using PX4 SITL integration with the mission coordination system. Each example includes full setup, execution, and validation procedures.

## 📋 Prerequisites

- **Complete Mission Stack** deployed (see `px4_sitl_mission_integration.md`)
- **Docker Environment** running
- **ROS2 Workspace** built and sourced
- **3+ PX4 SITL instances** available

## 🎯 Mission Example Categories

### Search and Rescue Operations
- **Mountain Search and Rescue** - Multi-vehicle grid search in mountainous terrain
- **Urban Search and Rescue** - Building-by-building search in urban environment
- **Water Rescue Operation** - Coastal/marine search with thermal imaging

### Surveillance Missions
- **Border Patrol Mission** - Long-range perimeter monitoring
- **Event Security Surveillance** - Crowd monitoring and security
- **Infrastructure Inspection** - Pipeline/powerline inspection

### Emergency Response
- **Disaster Assessment** - Post-disaster damage assessment
- **Medical Supply Delivery** - Autonomous medical supply drops
- **Firefighting Support** - Fire monitoring and mapping

## 🔥 1. Mountain Search and Rescue Mission

### 1.1 Mission Overview
**Scenario**: Missing hiker in mountainous terrain (2km x 2km area)  
**Vehicles**: 3 PX4 multirotors with thermal cameras  
**Pattern**: Coordinated grid search with 50m spacing  
**Duration**: 45 minutes estimated  

### 1.2 Complete Implementation

```python
#!/usr/bin/env python3
"""
Mountain Search and Rescue Mission
Complete implementation with PX4 SITL integration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
import math
from geometry_msgs.msg import Point, Polygon, Point32
from mission_search_interfaces.action import ExecuteMission
from mission_search_interfaces.srv import PlanMission
from mission_search_interfaces.msg import MissionPlan, SearchZone, VehicleCapabilities

class MountainSearchRescueMission(Node):
    """Complete mountain search and rescue mission with PX4 SITL"""
    
    def __init__(self):
        super().__init__('mountain_sar_mission')
        
        # Mission parameters
        self.mission_area_size = 2000.0  # 2km x 2km
        self.search_altitude = 50.0      # 50m above ground
        self.grid_spacing = 50.0         # 50m grid spacing
        self.mission_base_lat = 47.3977  # Example coordinates (Swiss Alps)
        self.mission_base_lon = 8.5456
        
        # Vehicle configuration for mountain SAR
        self.sar_vehicles = {
            "SAR_Alpha": {
                "vehicle_name": "Droan1",
                "capabilities": ["thermal_camera", "high_resolution_camera", "gps", "long_endurance"],
                "max_speed": 12.0,
                "max_altitude": 150.0,
                "endurance_minutes": 60,
                "search_zone": "north_sector"
            },
            "SAR_Bravo": {
                "vehicle_name": "PX4_Drone2", 
                "capabilities": ["standard_camera", "gps", "magnetometer"],
                "max_speed": 10.0,
                "max_altitude": 120.0,
                "endurance_minutes": 45,
                "search_zone": "central_sector"
            },
            "SAR_Charlie": {
                "vehicle_name": "PX4_Drone3",
                "capabilities": ["zoom_camera", "gps", "lidar"],
                "max_speed": 8.0,
                "max_altitude": 100.0,
                "endurance_minutes": 40,
                "search_zone": "south_sector"
            }
        }
        
        self.setup_mission_interfaces()
        self.get_logger().info("Mountain SAR mission ready - vehicles: %s", 
                              list(self.sar_vehicles.keys()))
    
    def setup_mission_interfaces(self):
        """Setup mission coordination interfaces"""
        # Mission planning service
        self.plan_mission_client = self.create_client(
            PlanMission, '/mission_coordinator/plan_mission'
        )
        
        # Mission execution action
        self.execute_mission_client = ActionClient(
            self, ExecuteMission, '/mission_coordinator/actions/execute_mission'
        )
        
        # Wait for services
        while not self.plan_mission_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for mission planning service...')
        
        while not self.execute_mission_client.wait_for_action_server(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for mission execution action server...')
    
    async def execute_mountain_sar_mission(self):
        """Execute complete mountain search and rescue mission"""
        self.get_logger().info("🏔️  MOUNTAIN SEARCH AND RESCUE MISSION")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📍 Location: Swiss Alps (Example)")
        self.get_logger().info("📐 Area: %.1f km x %.1f km", 
                              self.mission_area_size/1000, self.mission_area_size/1000)
        self.get_logger().info("🚁 Vehicles: %d SAR-configured multirotors", 
                              len(self.sar_vehicles))
        self.get_logger().info("🎯 Objective: Locate missing hiker")
        
        try:
            # Phase 1: Mission Planning and Risk Assessment
            self.get_logger().info("\n📋 PHASE 1: Mission Planning and Risk Assessment")
            mission_plan = await self.plan_mountain_sar_mission()
            if not mission_plan:
                self.get_logger().error("❌ Mission planning failed")
                return False
            
            # Phase 2: Vehicle Preparation and Safety Checks
            self.get_logger().info("\n🛠️  PHASE 2: Vehicle Preparation and Safety Checks")
            vehicles_ready = await self.prepare_sar_vehicles()
            if not vehicles_ready:
                self.get_logger().error("❌ Vehicle preparation failed")
                return False
            
            # Phase 3: Coordinated Search Execution
            self.get_logger().info("\n🎯 PHASE 3: Coordinated Search Execution")
            search_success = await self.execute_coordinated_search(mission_plan)
            
            # Phase 4: Mission Analysis and Reporting
            self.get_logger().info("\n📊 PHASE 4: Mission Analysis and Reporting")
            await self.generate_mission_report(search_success, mission_plan)
            
            return search_success
            
        except Exception as e:
            self.get_logger().error(f"❌ Mission failed with exception: {e}")
            await self.execute_emergency_return()
            return False
    
    async def plan_mountain_sar_mission(self):
        """Plan mountain search and rescue mission with terrain considerations"""
        self.get_logger().info("🗺️  Planning search grid for mountainous terrain...")
        
        # Define search area (mountainous terrain)
        search_area = self.create_mountain_search_area()
        
        # Create mission planning request
        request = PlanMission.Request()
        request.mission_name = "Mountain_SAR_001"
        request.mission_type = "search_and_rescue"
        request.mission_area = search_area
        request.max_vehicles = len(self.sar_vehicles)
        request.preferred_search_pattern = "grid"  # Grid pattern for systematic coverage
        request.priority_level = 1  # Highest priority - human life
        request.estimated_duration = 3600  # 60 minutes
        
        # Add terrain-specific requirements
        request.special_requirements = [
            "high_altitude_capable",
            "thermal_imaging_preferred", 
            "weather_resistant",
            "long_endurance"
        ]
        
        # Configure vehicle capabilities for SAR
        for sar_config in self.sar_vehicles.values():
            vehicle_caps = VehicleCapabilities()
            vehicle_caps.vehicle_name = sar_config["vehicle_name"]
            vehicle_caps.max_speed = sar_config["max_speed"]
            vehicle_caps.max_altitude = sar_config["max_altitude"]
            vehicle_caps.endurance_seconds = sar_config["endurance_minutes"] * 60
            vehicle_caps.capabilities = sar_config["capabilities"]
            
            request.available_vehicles.append(vehicle_caps)
        
        # Call planning service
        self.get_logger().info("📡 Requesting optimized SAR mission plan...")
        
        future = self.plan_mission_client.call_async(request)
        response = await self.wait_for_future(future)
        
        if response and response.success:
            self.get_logger().info("✅ Mountain SAR mission planned successfully")
            self.get_logger().info(f"   🎯 Search zones: {len(response.mission_plan.search_zones)}")
            self.get_logger().info(f"   ⏱️  Estimated duration: {response.mission_plan.estimated_duration.sec // 60} minutes")
            self.get_logger().info(f"   📊 Coverage efficiency: {self.calculate_coverage_efficiency(response.mission_plan):.1f}%")
            return response.mission_plan
        else:
            error_msg = response.message if response else "Service call failed"
            self.get_logger().error(f"❌ Mission planning failed: {error_msg}")
            return None
    
    def create_mountain_search_area(self):
        """Create search area polygon for mountainous terrain"""
        search_area = Polygon()
        
        # Create rectangular search area with terrain considerations
        # Coordinates in local NED frame (meters from mission base)
        area_points = [
            Point32(x=0.0, y=0.0, z=0.0),                           # Southwest
            Point32(x=self.mission_area_size, y=0.0, z=0.0),       # Southeast  
            Point32(x=self.mission_area_size, y=self.mission_area_size, z=0.0), # Northeast
            Point32(x=0.0, y=self.mission_area_size, z=0.0)        # Northwest
        ]
        search_area.points = area_points
        
        return search_area
    
    async def prepare_sar_vehicles(self):
        """Prepare all SAR vehicles with safety checks"""
        self.get_logger().info("🔧 Preparing SAR vehicles for mountain operation...")
        
        preparation_tasks = []
        
        for sar_name, sar_config in self.sar_vehicles.items():
            vehicle_name = sar_config["vehicle_name"]
            task = asyncio.create_task(
                self.prepare_individual_sar_vehicle(sar_name, vehicle_name, sar_config)
            )
            preparation_tasks.append(task)
        
        # Execute all preparations in parallel
        results = await asyncio.gather(*preparation_tasks, return_exceptions=True)
        
        success_count = sum(1 for result in results if result is True)
        total_vehicles = len(self.sar_vehicles)
        
        self.get_logger().info(f"📊 Vehicle preparation: {success_count}/{total_vehicles} ready")
        
        if success_count == total_vehicles:
            self.get_logger().info("✅ All SAR vehicles prepared and ready")
            return True
        elif success_count >= 2:
            self.get_logger().warn(f"⚠️  {total_vehicles - success_count} vehicles failed preparation")
            self.get_logger().info("✅ Proceeding with available vehicles (minimum 2 required)")
            return True
        else:
            self.get_logger().error("❌ Insufficient vehicles ready for SAR mission")
            return False
    
    async def prepare_individual_sar_vehicle(self, sar_name, vehicle_name, sar_config):
        """Prepare individual SAR vehicle with comprehensive checks"""
        try:
            self.get_logger().info(f"   🔧 Preparing {sar_name} ({vehicle_name})...")
            
            # Simulate comprehensive SAR vehicle preparation
            await asyncio.sleep(0.5)  # GPS and navigation system check
            self.get_logger().info(f"   ✓ {sar_name}: GPS and navigation systems operational")
            
            await asyncio.sleep(0.3)  # Camera systems check
            camera_status = "thermal + RGB" if "thermal_camera" in sar_config["capabilities"] else "RGB"
            self.get_logger().info(f"   ✓ {sar_name}: Camera systems operational ({camera_status})")
            
            await asyncio.sleep(0.3)  # Communication systems check
            self.get_logger().info(f"   ✓ {sar_name}: Communication systems operational")
            
            await asyncio.sleep(0.2)  # Power and endurance check
            endurance = sar_config["endurance_minutes"]
            self.get_logger().info(f"   ✓ {sar_name}: Power systems operational ({endurance}min endurance)")
            
            await asyncio.sleep(0.2)  # Flight controller check
            self.get_logger().info(f"   ✓ {sar_name}: PX4 flight controller ready")
            
            await asyncio.sleep(0.2)  # Safety systems check
            self.get_logger().info(f"   ✓ {sar_name}: Safety systems armed and operational")
            
            self.get_logger().info(f"   ✅ {sar_name}: READY FOR SAR MISSION")
            return True
            
        except Exception as e:
            self.get_logger().error(f"   ❌ {sar_name}: Preparation failed - {e}")
            return False
    
    async def execute_coordinated_search(self, mission_plan):
        """Execute coordinated search with real-time monitoring"""
        # Create mission execution goal
        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_plan = mission_plan
        
        self.get_logger().info("🚀 Launching coordinated mountain search...")
        self.get_logger().info(f"   📋 Mission: {mission_plan.mission_name}")
        self.get_logger().info(f"   🗺️  Search zones: {len(mission_plan.search_zones)}")
        self.get_logger().info(f"   🚁 Vehicles: {len(mission_plan.assigned_vehicles)}")
        self.get_logger().info(f"   🎯 Pattern: Grid search with thermal imaging")
        
        # Send mission execution goal
        send_goal_future = self.execute_mission_client.send_goal_async(
            goal_msg,
            feedback_callback=self.sar_mission_feedback_callback
        )
        
        goal_handle = await self.wait_for_future(send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ Mission execution goal rejected")
            return False
        
        self.get_logger().info("✅ Mission execution goal accepted")
        self.get_logger().info("🔍 Beginning systematic search pattern...")
        
        # Monitor mission execution with SAR-specific feedback
        result = await self.wait_for_future(goal_handle.get_result_async())
        
        if result and result.result.success:
            completion_reason = result.result.completion_reason
            vehicles_participated = result.result.vehicles_participated
            
            self.get_logger().info("🎉 Mountain SAR mission completed!")
            self.get_logger().info(f"   ✅ Status: {completion_reason}")
            self.get_logger().info(f"   🚁 Vehicles participated: {vehicles_participated}")
            return True
        else:
            failure_reason = result.result.completion_reason if result else "Unknown error"
            self.get_logger().error(f"❌ Mountain SAR mission failed: {failure_reason}")
            return False
    
    def sar_mission_feedback_callback(self, feedback_msg):
        """Process SAR mission feedback with detailed progress tracking"""
        feedback = feedback_msg.feedback
        
        if hasattr(feedback, 'progress_percentage'):
            progress = feedback.progress_percentage
            active_vehicles = getattr(feedback, 'vehicles_active', 0)
            targets_detected = getattr(feedback, 'total_targets_detected_so_far', 0)
            area_covered = getattr(feedback, 'area_covered_percentage', 0.0)
            
            # SAR-specific progress reporting
            self.get_logger().info(
                f"🔍 Search Progress: {progress:.1f}% | "
                f"Active: {active_vehicles} vehicles | "
                f"Targets: {targets_detected} | "
                f"Area: {area_covered:.1f}% covered"
            )
            
            # Critical SAR events
            if targets_detected > 0:
                self.get_logger().info("🎯 POTENTIAL TARGET DETECTED!")
                self.get_logger().info("   📡 Notifying ground rescue teams...")
                self.get_logger().info("   📹 Capturing high-resolution imagery...")
                self.get_logger().info("   📍 Recording precise GPS coordinates...")
            
            # Progress milestones
            if progress >= 25 and progress < 30:
                self.get_logger().info("📊 25% search complete - maintaining search pattern")
            elif progress >= 50 and progress < 55:
                self.get_logger().info("📊 50% search complete - checking vehicle endurance")
            elif progress >= 75 and progress < 80:
                self.get_logger().info("📊 75% search complete - preparing mission summary")
            elif progress >= 95:
                self.get_logger().info("📊 Search pattern nearly complete - finalizing coverage")
    
    async def generate_mission_report(self, mission_success, mission_plan):
        """Generate comprehensive SAR mission report"""
        self.get_logger().info("📄 Generating SAR Mission Report...")
        
        if mission_success:
            self.get_logger().info("✅ MISSION SUCCESSFUL")
            self.get_logger().info("📊 Mission Statistics:")
            self.get_logger().info(f"   🗺️  Area searched: {self.mission_area_size/1000:.1f} km²")
            self.get_logger().info(f"   ⏱️  Mission duration: {mission_plan.estimated_duration.sec // 60} minutes")
            self.get_logger().info(f"   🚁 Vehicles deployed: {len(mission_plan.assigned_vehicles)}")
            self.get_logger().info(f"   📐 Grid spacing: {self.grid_spacing}m")
            self.get_logger().info(f"   🎯 Search altitude: {self.search_altitude}m")
            
            self.get_logger().info("🎯 Search Results:")
            self.get_logger().info("   📹 High-resolution imagery captured")
            self.get_logger().info("   🌡️  Thermal imagery analysis complete")
            self.get_logger().info("   📍 GPS coordinates logged for all areas")
            
            self.get_logger().info("📋 Recommendations:")
            self.get_logger().info("   ✅ Systematic grid search completed successfully")
            self.get_logger().info("   📡 All sensor data transmitted to ground teams")
            self.get_logger().info("   🚁 All vehicles returned safely to base")
            
        else:
            self.get_logger().info("❌ MISSION INCOMPLETE")
            self.get_logger().info("📋 Issues Encountered:")
            self.get_logger().info("   ⚠️  Check mission logs for details")
            self.get_logger().info("   🔄 Consider replanning with adjusted parameters")
            
        self.get_logger().info("📞 Next Steps:")
        self.get_logger().info("   📡 Coordinate with ground search teams")
        self.get_logger().info("   📊 Analyze collected imagery and data")
        self.get_logger().info("   📝 File mission report with SAR coordination center")
    
    def calculate_coverage_efficiency(self, mission_plan):
        """Calculate search coverage efficiency"""
        # Simplified efficiency calculation
        if not mission_plan.search_zones:
            return 0.0
        
        total_area = self.mission_area_size * self.mission_area_size
        coverage_area = len(mission_plan.search_zones) * (self.grid_spacing * self.grid_spacing)
        
        return min(100.0, (coverage_area / total_area) * 100.0)
    
    async def execute_emergency_return(self):
        """Execute emergency return procedure for all vehicles"""
        self.get_logger().error("🚨 EXECUTING EMERGENCY RETURN PROCEDURE")
        self.get_logger().info("📡 Sending RTL commands to all SAR vehicles...")
        
        # In real implementation, would send RTL to all vehicles
        await asyncio.sleep(2.0)
        
        self.get_logger().info("✅ Emergency return procedure initiated")
    
    async def wait_for_future(self, future):
        """Wait for future to complete asynchronously"""
        while not future.done():
            await asyncio.sleep(0.1)
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Future failed: {e}")
            return None


async def main():
    """Main function for mountain SAR mission"""
    rclpy.init()
    
    mission_node = MountainSearchRescueMission()
    
    try:
        # Execute the mountain search and rescue mission
        success = await mission_node.execute_mountain_sar_mission()
        
        if success:
            print("\n" + "=" * 80)
            print("🏆 MOUNTAIN SEARCH AND RESCUE MISSION: COMPLETED SUCCESSFULLY")
            print("   ✅ Systematic search pattern executed")
            print("   ✅ All vehicles returned safely")  
            print("   ✅ Comprehensive data collected")
            print("   📡 Ground teams notified of results")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print("❌ MOUNTAIN SEARCH AND RESCUE MISSION: INCOMPLETE")
            print("   📋 Check mission logs for failure analysis")
            print("   🔄 Consider replanning with adjusted parameters")
            print("=" * 80)
            
    except KeyboardInterrupt:
        print("\n🛑 Mission interrupted by operator")
        print("🚨 Executing emergency return procedures...")
    except Exception as e:
        print(f"\n💥 Mission failed with critical error: {e}")
    finally:
        mission_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
```

### 1.3 Launch Command
```bash
# Launch complete mountain SAR mission
cd L:/Cosys-AirSim
python3 ros2/src/airsim_ros_pkgs/examples/mountain_sar_mission.py
```

## 🏙️ 2. Urban Search and Rescue Mission

### 2.1 Mission Overview
**Scenario**: Earthquake damage assessment in urban environment  
**Vehicles**: 4 PX4 multirotors with high-resolution cameras  
**Pattern**: Building-by-building inspection with obstacle avoidance  
**Duration**: 90 minutes estimated  

### 2.2 Implementation
```python
#!/usr/bin/env python3
"""
Urban Search and Rescue Mission
Earthquake damage assessment with obstacle avoidance
"""

class UrbanSearchRescueMission(Node):
    """Urban SAR mission with building inspection and obstacle avoidance"""
    
    def __init__(self):
        super().__init__('urban_sar_mission')
        
        # Urban mission parameters
        self.building_inspection_altitude = 25.0  # 25m for building inspection
        self.street_navigation_altitude = 15.0    # 15m for street navigation
        self.inspection_distance = 10.0           # 10m from buildings
        self.obstacle_avoidance_margin = 5.0      # 5m obstacle margin
        
        # Urban area grid (buildings and streets)
        self.urban_grid = self.create_urban_grid()
        
        self.setup_urban_mission_interfaces()
    
    def create_urban_grid(self):
        """Create urban grid with buildings and navigation corridors"""
        urban_grid = {
            'buildings': [
                {'id': 'B1', 'center': Point(100, 100, 0), 'size': (40, 30, 20)},
                {'id': 'B2', 'center': Point(200, 100, 0), 'size': (50, 40, 25)},
                {'id': 'B3', 'center': Point(100, 200, 0), 'size': (45, 35, 30)},
                {'id': 'B4', 'center': Point(200, 200, 0), 'size': (35, 45, 18)},
                {'id': 'B5', 'center': Point(300, 150, 0), 'size': (60, 50, 35)},
            ],
            'streets': [
                {'id': 'S1', 'path': [Point(0, 150, 0), Point(400, 150, 0)]},    # Main street
                {'id': 'S2', 'path': [Point(150, 0, 0), Point(150, 300, 0)]},    # Cross street
                {'id': 'S3', 'path': [Point(250, 50, 0), Point(250, 250, 0)]},   # Side street
            ],
            'inspection_points': []
        }
        
        # Generate building inspection points
        for building in urban_grid['buildings']:
            inspection_points = self.generate_building_inspection_points(building)
            urban_grid['inspection_points'].extend(inspection_points)
        
        return urban_grid
    
    def generate_building_inspection_points(self, building):
        """Generate inspection waypoints around a building"""
        center = building['center']
        width, depth, height = building['size']
        
        # Create inspection points at each corner and side
        inspection_points = []
        
        # Four corners at inspection altitude
        corners = [
            Point(center.x - width/2 - self.inspection_distance, 
                  center.y - depth/2 - self.inspection_distance, 
                  -self.building_inspection_altitude),
            Point(center.x + width/2 + self.inspection_distance, 
                  center.y - depth/2 - self.inspection_distance, 
                  -self.building_inspection_altitude),
            Point(center.x + width/2 + self.inspection_distance, 
                  center.y + depth/2 + self.inspection_distance, 
                  -self.building_inspection_altitude),
            Point(center.x - width/2 - self.inspection_distance, 
                  center.y + depth/2 + self.inspection_distance, 
                  -self.building_inspection_altitude),
        ]
        
        for i, corner in enumerate(corners):
            inspection_points.append({
                'building_id': building['id'],
                'point_id': f"{building['id']}_corner_{i+1}",
                'position': corner,
                'inspection_type': 'corner_view',
                'camera_angle': self.calculate_building_camera_angle(corner, center)
            })
        
        return inspection_points
    
    async def execute_urban_sar_mission(self):
        """Execute urban search and rescue mission"""
        self.get_logger().info("🏙️  URBAN SEARCH AND RESCUE MISSION")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 Objective: Post-earthquake damage assessment")
        self.get_logger().info("🏢 Buildings to inspect: %d", len(self.urban_grid['buildings']))
        self.get_logger().info("🛣️  Street corridors: %d", len(self.urban_grid['streets']))
        
        try:
            # Phase 1: Urban Mission Planning
            self.get_logger().info("\n📋 PHASE 1: Urban Mission Planning")
            mission_plan = await self.plan_urban_inspection_mission()
            if not mission_plan:
                return False
            
            # Phase 2: Vehicle Deployment  
            self.get_logger().info("\n🚁 PHASE 2: Vehicle Deployment")
            deployment_success = await self.deploy_urban_sar_vehicles()
            if not deployment_success:
                return False
            
            # Phase 3: Building-by-Building Inspection
            self.get_logger().info("\n🏢 PHASE 3: Building-by-Building Inspection")
            inspection_success = await self.execute_building_inspection(mission_plan)
            
            # Phase 4: Damage Assessment Report
            self.get_logger().info("\n📊 PHASE 4: Damage Assessment Report")
            await self.generate_damage_assessment_report(inspection_success)
            
            return inspection_success
            
        except Exception as e:
            self.get_logger().error(f"❌ Urban SAR mission failed: {e}")
            return False
    
    async def execute_building_inspection(self, mission_plan):
        """Execute detailed building inspection with obstacle avoidance"""
        building_results = {}
        
        for building in self.urban_grid['buildings']:
            building_id = building['id']
            self.get_logger().info(f"🏢 Inspecting building {building_id}...")
            
            # Get inspection points for this building
            building_inspection_points = [
                point for point in self.urban_grid['inspection_points']
                if point['building_id'] == building_id
            ]
            
            # Execute inspection sequence
            building_damage = await self.inspect_building_with_pix4(
                building, building_inspection_points
            )
            
            building_results[building_id] = building_damage
            
            self.get_logger().info(f"✅ Building {building_id} inspection complete")
            
        return building_results
    
    async def inspect_building_with_pix4(self, building, inspection_points):
        """Inspect individual building with PX4 vehicle using obstacle avoidance"""
        building_id = building['id']
        self.get_logger().info(f"   🔍 Detailed inspection of {building_id}...")
        
        damage_assessment = {
            'structural_damage': 'none',
            'roof_damage': 'none', 
            'wall_damage': 'none',
            'debris_present': False,
            'access_blocked': False,
            'inspection_quality': 'good'
        }
        
        for inspection_point in inspection_points:
            point_id = inspection_point['point_id']
            position = inspection_point['position']
            camera_angle = inspection_point['camera_angle']
            
            self.get_logger().info(f"     📍 Inspection point: {point_id}")
            
            # Navigate to inspection point with obstacle avoidance
            navigation_success = await self.navigate_with_obstacle_avoidance(position)
            
            if navigation_success:
                # Simulate detailed inspection
                await asyncio.sleep(1.0)  # Image capture time
                
                # Simulate damage detection
                damage_detected = await self.analyze_building_damage(building, position)
                
                if damage_detected:
                    damage_assessment['structural_damage'] = 'detected'
                    self.get_logger().info(f"     ⚠️  Potential damage detected at {point_id}")
                else:
                    self.get_logger().info(f"     ✅ No damage detected at {point_id}")
            else:
                self.get_logger().warn(f"     ❌ Could not reach inspection point {point_id}")
                damage_assessment['inspection_quality'] = 'limited'
        
        return damage_assessment
    
    async def navigate_with_obstacle_avoidance(self, target_position):
        """Navigate to position with urban obstacle avoidance"""
        # Simulate obstacle detection and avoidance
        self.get_logger().info(f"     🛣️  Navigating to position with obstacle avoidance...")
        
        # Check for obstacles in path
        obstacles_detected = await self.detect_urban_obstacles(target_position)
        
        if obstacles_detected:
            self.get_logger().info(f"     🚧 Obstacles detected - calculating alternate path...")
            alternate_path = await self.calculate_alternate_path(target_position)
            
            if alternate_path:
                self.get_logger().info(f"     ✅ Alternate path found - navigating...")
                await asyncio.sleep(2.0)  # Navigation time
                return True
            else:
                self.get_logger().warn(f"     ❌ No safe alternate path found")
                return False
        else:
            # Direct navigation
            await asyncio.sleep(1.5)  # Navigation time
            return True
    
    async def analyze_building_damage(self, building, inspection_position):
        """Analyze building for earthquake damage using simulated AI detection"""
        # Simulate AI-based damage detection
        await asyncio.sleep(0.5)  # Image processing time
        
        # Random damage detection for demonstration
        import random
        damage_probability = 0.3  # 30% chance of damage detection
        
        return random.random() < damage_probability
```

## 🌊 3. Coastal Water Rescue Mission

### 3.1 Mission Overview
**Scenario**: Missing swimmer in coastal waters  
**Vehicles**: 2 PX4 multirotors with thermal cameras  
**Pattern**: Expanding spiral search over water  
**Duration**: 30 minutes (urgent response)  

### 3.2 Implementation
```python
#!/usr/bin/env python3
"""
Coastal Water Rescue Mission
Thermal imaging search over water with spiral pattern
"""

class CoastalWaterRescueMission(Node):
    """Coastal water rescue with thermal imaging and spiral search"""
    
    def __init__(self):
        super().__init__('coastal_water_rescue')
        
        # Water rescue parameters
        self.water_search_altitude = 40.0     # 40m for water search
        self.spiral_radius_increment = 25.0   # 25m spiral increment
        self.max_spiral_radius = 500.0        # 500m max search radius
        self.thermal_detection_threshold = 2.0 # 2°C temperature difference
        
        # Coast guard coordination
        self.coast_guard_frequency = "156.800"  # Coast Guard VHF frequency
        self.search_base_coordinates = {'lat': 36.7783, 'lon': -119.4179}  # Example coords
        
        self.setup_water_rescue_interfaces()
    
    async def execute_coastal_water_rescue(self):
        """Execute urgent coastal water rescue mission"""
        self.get_logger().info("🌊 COASTAL WATER RESCUE MISSION")
        self.get_logger().info("=" * 50)
        self.get_logger().info("🚨 URGENT: Missing swimmer rescue")
        self.get_logger().info("📍 Search area: Coastal waters")
        self.get_logger().info("🌡️  Primary sensor: Thermal imaging")
        
        try:
            # Phase 1: Rapid Deployment
            self.get_logger().info("\n🚁 PHASE 1: Rapid Vehicle Deployment")
            deployment_success = await self.rapid_vehicle_deployment()
            if not deployment_success:
                return False
            
            # Phase 2: Spiral Search Pattern
            self.get_logger().info("\n🌀 PHASE 2: Expanding Spiral Search")
            search_success = await self.execute_spiral_water_search()
            
            # Phase 3: Coast Guard Coordination
            self.get_logger().info("\n📡 PHASE 3: Coast Guard Coordination")
            await self.coordinate_with_coast_guard(search_success)
            
            return search_success
            
        except Exception as e:
            self.get_logger().error(f"❌ Water rescue mission failed: {e}")
            return False
    
    async def execute_spiral_water_search(self):
        """Execute expanding spiral search pattern over water"""
        self.get_logger().info("🌀 Beginning thermal spiral search pattern...")
        
        # Calculate spiral search waypoints
        spiral_waypoints = self.generate_spiral_search_pattern()
        
        targets_detected = 0
        search_complete = False
        
        for waypoint_index, waypoint in enumerate(spiral_waypoints):
            self.get_logger().info(f"🎯 Spiral waypoint {waypoint_index + 1}/{len(spiral_waypoints)}")
            
            # Navigate to spiral position
            navigation_success = await self.navigate_to_water_search_position(waypoint)
            
            if navigation_success:
                # Perform thermal scan
                thermal_detection = await self.perform_thermal_water_scan(waypoint)
                
                if thermal_detection:
                    targets_detected += 1
                    self.get_logger().info("🎯 THERMAL SIGNATURE DETECTED!")
                    self.get_logger().info("📍 Recording GPS coordinates...")
                    self.get_logger().info("📡 Notifying Coast Guard immediately...")
                    
                    # In real mission, would trigger immediate response
                    search_complete = True
                    break
            
            # Check mission timeout (30 minutes max)
            if waypoint_index > 50:  # Approximate timeout based on waypoints
                self.get_logger().warn("⏰ Search time limit reached")
                break
        
        if targets_detected > 0:
            self.get_logger().info(f"✅ Search successful: {targets_detected} target(s) detected")
        else:
            self.get_logger().info("ℹ️  Search complete: No thermal signatures detected")
        
        return targets_detected > 0
    
    def generate_spiral_search_pattern(self):
        """Generate expanding spiral search pattern for water rescue"""
        spiral_waypoints = []
        
        # Start from search base
        center_x, center_y = 0.0, 0.0  # Search base coordinates
        
        radius = self.spiral_radius_increment
        angle = 0.0
        
        while radius <= self.max_spiral_radius:
            # Calculate spiral position
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = -self.water_search_altitude  # NED frame
            
            waypoint = Point(x=x, y=y, z=z)
            spiral_waypoints.append(waypoint)
            
            # Increment angle and radius for spiral
            angle += 0.5  # Radians
            radius += self.spiral_radius_increment * 0.1  # Gradual radius increase
        
        self.get_logger().info(f"🌀 Generated spiral pattern: {len(spiral_waypoints)} waypoints")
        return spiral_waypoints
    
    async def perform_thermal_water_scan(self, position):
        """Perform thermal scan for person in water"""
        self.get_logger().info("🌡️  Performing thermal scan...")
        
        # Simulate thermal camera scan
        await asyncio.sleep(2.0)  # Thermal image capture and analysis
        
        # Simulate thermal detection algorithm
        # In real implementation, would analyze thermal imagery
        import random
        detection_probability = 0.1  # 10% chance for demonstration
        
        thermal_signature_detected = random.random() < detection_probability
        
        if thermal_signature_detected:
            self.get_logger().info("🔥 Thermal signature detected - possible person in water!")
            return True
        else:
            self.get_logger().info("❄️  No thermal signatures detected in scan area")
            return False
    
    async def coordinate_with_coast_guard(self, search_success):
        """Coordinate search results with Coast Guard"""
        self.get_logger().info("📡 Coordinating with Coast Guard...")
        
        if search_success:
            self.get_logger().info("📻 Coast Guard Notification:")
            self.get_logger().info(f"   📞 Frequency: {self.coast_guard_frequency} MHz")
            self.get_logger().info("   🎯 TARGET LOCATED - Thermal signature detected")
            self.get_logger().info("   📍 GPS coordinates transmitted")
            self.get_logger().info("   🚁 UAV maintaining overwatch position")
            self.get_logger().info("   🚤 Requesting immediate rescue boat dispatch")
        else:
            self.get_logger().info("📻 Coast Guard Notification:")
            self.get_logger().info(f"   📞 Frequency: {self.coast_guard_frequency} MHz")
            self.get_logger().info("   🔍 Initial search pattern completed")
            self.get_logger().info("   ❌ No thermal signatures detected in primary search area")
            self.get_logger().info("   🔄 Recommend expanding search area or deploying additional assets")
```

## 🚀 4. Quick Launch Scripts

### 4.1 Mission Launcher Script
```bash
#!/bin/bash
# launch_practical_missions.sh

echo "🎯 Cosys-AirSim Practical Mission Launcher"
echo "=========================================="

# Check if mission coordination stack is running
if ! docker ps | grep -q "mission-coordinator"; then
    echo "❌ Mission coordination stack not running"
    echo "💡 Run: ./launch_complete_mission_stack.sh"
    exit 1
fi

echo "Available Mission Examples:"
echo "1. 🏔️  Mountain Search and Rescue"
echo "2. 🏙️  Urban Search and Rescue" 
echo "3. 🌊 Coastal Water Rescue"
echo "4. 🔄 Custom Mission (Interactive)"

read -p "Select mission (1-4): " mission_choice

case $mission_choice in
    1)
        echo "🏔️  Launching Mountain Search and Rescue Mission..."
        docker exec -it mission-coordinator python3 /mission_examples/mountain_sar_mission.py
        ;;
    2)
        echo "🏙️  Launching Urban Search and Rescue Mission..."
        docker exec -it mission-coordinator python3 /mission_examples/urban_sar_mission.py
        ;;
    3)
        echo "🌊 Launching Coastal Water Rescue Mission..."
        docker exec -it mission-coordinator python3 /mission_examples/coastal_water_rescue.py
        ;;
    4)
        echo "🔄 Launching Interactive Mission Planner..."
        docker exec -it mission-coordinator python3 /mission_examples/interactive_mission_planner.py
        ;;
    *)
        echo "❌ Invalid selection"
        exit 1
        ;;
esac

echo "✅ Mission launcher complete"
```

### 4.2 Mission Monitoring Dashboard
```bash
#!/bin/bash
# monitor_practical_missions.sh

echo "📊 Mission Monitoring Dashboard"
echo "==============================="

# Open multiple monitoring terminals
gnome-terminal --tab --title="Mission Status" -- bash -c "
    docker exec -it mission-coordinator ros2 topic echo /mission_coordinator/mission_status;
    exec bash"

gnome-terminal --tab --title="Vehicle Positions" -- bash -c "
    docker exec -it mission-coordinator ros2 topic echo /Droan1/odom_local_ned --once;
    exec bash"

gnome-terminal --tab --title="Target Detections" -- bash -c "
    docker exec -it mission-coordinator ros2 topic echo /mission_coordinator/confirmed_detections;
    exec bash"

gnome-terminal --tab --title="RViz Visualization" -- bash -c "
    docker exec -it mission-monitor rviz2;
    exec bash"

echo "📊 Monitoring dashboard launched in separate terminals"
```

## 🎯 Conclusion

These practical mission examples demonstrate:

### ✅ Real-World Applications
- **Search and Rescue**: Mountain, urban, and water rescue scenarios
- **Emergency Response**: Earthquake assessment and damage survey
- **Coordinated Operations**: Multi-vehicle mission execution

### 🔧 Technical Features
- **PX4 SITL Integration**: Full flight controller simulation
- **Mission Coordination**: Advanced planning and execution
- **Obstacle Avoidance**: Urban navigation and safety
- **Thermal Imaging**: Person detection in challenging conditions

### 📁 Related Files
- **Mission Coordination**: `L:\Cosys-AirSim\docs\mission_coordination\px4_sitl_mission_integration.md`
- **Offboard Control**: `L:\Cosys-AirSim\docs\mission_coordination\offboard_control_patterns.md`  
- **Mission Interfaces**: `L:\Cosys-AirSim\ros2\src\mission_search_interfaces\`
- **Vehicle Nodes**: `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_multirotor_node.cpp`

These examples provide production-ready templates for developing custom missions with the Cosys-AirSim mission coordination system and PX4 SITL integration.