# Mission Node Usage Guide

**Quick practical guide for using the existing mission coordination system in Cosys-AirSim.**

## What's Available

### Working Mission Nodes
- **Mission Coordination Node**: `/mission_coordinator` - Global mission orchestration
- **Mission Vehicle Nodes**: Auto-discovered vehicles from AirSim (e.g., `/Droan1`, `/PX4_Drone2`)
- **Mission Actions**: SearchArea, NavigateToTarget, TrackTarget
- **Mission Services**: PlanMission, AssignSearchZone, GetMissionStatus

### Launch Options

#### RPC Dynamic Launch (Recommended)
- **Auto-Discovery**: Automatically detects vehicles from running AirSim
- **Mission Mode Toggle**: Switch between standard and mission-capable nodes
- **Flexible Configuration**: Supports any vehicle names and types from settings.json

#### Mission Mode Options
- **Standard Mode** (`mission_mode:=false`): Basic sensors and flight control only
- **Mission Mode** (`mission_mode:=true`): Full mission capabilities with action servers

### Quick Start

**Option 1: RPC Dynamic Launch (Recommended)**
```bash
# Launch mission system with automatic vehicle discovery (in Docker container)
./airsim_ros2_docker.bat shell
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true
```

**Option 2: Dedicated Mission Demo**
```bash
# Launch specific mission coordination demo
ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py
```

### RPC Dynamic Launch Configuration

The `rpc_dynamic_vehicles.launch.py` now supports mission mode with full configuration options:

```bash
# Basic mission mode with defaults
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true

# Mission mode with custom parameters
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
  mission_mode:=true \
  search_altitude:=30.0 \
  search_speed:=6.0 \
  pattern_spacing:=20.0 \
  detection_threshold:=0.8 \
  mission_timeout:=2400.0

# Standard mode (sensors only, no mission capabilities)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=false

# With custom AirSim connection
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
  mission_mode:=true \
  host_ip:=192.168.1.100 \
  host_port:=41451
```

**Available Mission Parameters:**
- `mission_mode`: Enable mission capabilities (true/false)
- `search_altitude`: Default search altitude in meters (default: 25.0)
- `search_speed`: Default search speed in m/s (default: 5.0)
- `pattern_spacing`: Default pattern spacing in meters (default: 15.0)
- `detection_threshold`: Target detection confidence threshold (default: 0.7)
- `mission_timeout`: Mission timeout in seconds (default: 1800.0)

## Usage Examples

### Example 1: Simple Search Mission

**Start the mission nodes:**
```bash
# Terminal 1: Start AirSim with vehicles
# (Run AirSim with any vehicles configured in settings.json)

# Terminal 2: Launch mission system with auto-discovery
cd /airsim_ros2_ws
source install/setup.bash

# Recommended: RPC Dynamic Launch with Mission Mode
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true

# Alternative: Use dedicated mission demo
# ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py
```

**Send a search mission to a vehicle:**
```bash
# Terminal 3: Send search area command
ros2 action send_goal /Droan1/actions/search_area \
mission_search_interfaces/action/SearchArea \
"{
  search_boundary: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 100.0, y: 0.0, z: 0.0},
      {x: 100.0, y: 100.0, z: 0.0},
      {x: 0.0, y: 100.0, z: 0.0}
    ]
  },
  search_pattern: 'spiral',
  search_altitude: 25.0,
  search_speed: 5.0,
  pattern_spacing: 15.0,
  enable_detection: true,
  detection_types: ['person', 'vehicle'],
  detection_confidence_threshold: 0.7,
  max_search_time: {sec: 300, nanosec: 0}
}"
```

**Monitor progress:**
```bash
# Watch search progress
ros2 topic echo /Droan1/mission/status

# Watch for target detections
ros2 topic echo /Droan1/detections/target
```

### Example 2: Navigate to Specific Target

**Send navigation mission:**
```bash
ros2 action send_goal /PX4_Drone2/actions/navigate_to_target \
mission_search_interfaces/action/NavigateToTarget \
"{
  target_location: {x: 50.0, y: 50.0, z: 25.0},
  target_id: 'waypoint_001',
  approach_altitude: 30.0,
  standoff_distance: 2.0,
  navigation_speed: 5.0,
  approach_pattern: 'direct',
  investigation_time: {sec: 30, nanosec: 0},
  capture_detailed_images: true,
  minimum_safe_altitude: 5.0
}"
```

**Monitor navigation:**
```bash
# Check navigation feedback
ros2 action send_goal /PX4_Drone2/actions/navigate_to_target \
mission_search_interfaces/action/NavigateToTarget \
"{target_location: {x: 50.0, y: 50.0, z: 25.0}, target_id: 'waypoint_001'}" --feedback
```

### Example 3: Track a Moving Target

**Start target tracking:**
```bash
ros2 action send_goal /Droan1/actions/track_target \
mission_search_interfaces/action/TrackTarget \
"{
  target_id: 'person_001',
  initial_target_location: {x: 25.0, y: 25.0, z: 0.0},
  tracking_altitude: 20.0,
  tracking_distance: 10.0,
  tracking_mode: 'active',
  max_tracking_speed: 8.0,
  maintain_visual_contact: true,
  max_tracking_time: {sec: 600, nanosec: 0}
}"
```

**Update target position during tracking:**
```bash
# Publish new target detection
ros2 topic pub /Droan1/detections/target \
mission_search_interfaces/msg/TargetDetection \
"{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
  detection_id: 'det_001',
  vehicle_name: 'Droan1',
  camera_name: 'front_center',
  world_position: {x: 30.0, y: 30.0, z: 0.0},
  confidence_score: 0.95,
  target_type: 'person',
  target_description: 'Person walking',
  detection_altitude: 25.0,
  detection_distance: 15.2,
  detection_bearing: 45.0,
  verified: true
}"
```

### Example 4: Multi-Vehicle Coordinated Mission

**Plan a coordinated mission:**
```bash
ros2 service call /airsim_coordination_node/services/plan_mission \
mission_search_interfaces/srv/PlanMission \
"{
  mission_name: 'search_mission_001',
  mission_type: 'search_and_rescue',
  mission_area: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 200.0, y: 0.0, z: 0.0},
      {x: 200.0, y: 200.0, z: 0.0},
      {x: 0.0, y: 200.0, z: 0.0}
    ]
  },
  search_altitude: 25.0,
  preferred_search_pattern: 'grid',
  required_coverage_percent: 90.0,
  min_vehicles: 2,
  max_vehicles: 3,
  max_mission_time: {sec: 1800, nanosec: 0},
  target_types: ['person', 'vehicle'],
  detection_confidence_threshold: 0.7
}"
```

**Execute the planned mission:**
```bash
# First get the mission plan from the planning service response, then:
ros2 action send_goal /airsim_coordination_node/actions/execute_mission \
mission_search_interfaces/action/ExecuteMission \
"{
  mission_plan: {
    # Use the MissionPlan object returned from plan_mission service
    mission_id: 'search_mission_001',
    mission_name: 'search_mission_001',
    mission_type: 'search_and_rescue'
  },
  dry_run: false,
  allow_plan_modifications: true,
  require_all_vehicles_ready: true,
  max_mission_duration: {sec: 1800, nanosec: 0},
  inter_vehicle_coordination_interval: 5.0,
  enable_human_oversight: true
}" --feedback
```

## Monitoring Commands

### Check Available Mission Actions
```bash
# List all mission actions
ros2 action list | grep -E "(search_area|navigate_to_target|track_target|execute_mission)"

# Get action details
ros2 interface show mission_search_interfaces/action/SearchArea
ros2 interface show mission_search_interfaces/action/NavigateToTarget
ros2 interface show mission_search_interfaces/action/TrackTarget
```

### Monitor Mission Status
```bash
# Global mission status
ros2 topic echo /airsim_coordination_node/mission_status

# Individual vehicle mission status
ros2 topic echo /Droan1/mission/status
ros2 topic echo /PX4_Drone2/mission/status

# Target detections
ros2 topic echo /Droan1/detections/target
ros2 topic echo /PX4_Drone2/detections/target
```

### Check Mission Services
```bash
# Available mission services
ros2 service list | grep mission

# Get mission status
ros2 service call /airsim_coordination_node/services/get_mission_status \
mission_search_interfaces/srv/GetMissionStatus \
"{mission_id: '', include_details: true, include_vehicle_status: true}"

# Get vehicle capabilities
ros2 service call /Droan1/services/get_capabilities \
mission_search_interfaces/srv/GetVehicleCapabilities "{}"
```

## Troubleshooting

### Nodes Not Starting
```bash
# Check if mission nodes are running
ros2 node list

# Expected nodes:
# /airsim_coordination_node
# /Droan1  
# /PX4_Drone2

# Check node info
ros2 node info /airsim_coordination_node
ros2 node info /Droan1
```

### Actions Not Available
```bash
# Verify action servers are active
ros2 action info /Droan1/actions/search_area
ros2 action info /airsim_coordination_node/actions/execute_mission

# Should show: "Action clients: 0" and "Action servers: 1"
```

### Mission Interface Issues
```bash
# Rebuild mission interfaces if needed
cd /airsim_ros2_ws
colcon build --packages-select mission_search_interfaces
source install/setup.bash

# Check interface definitions
ros2 interface list | grep mission_search_interfaces
```

### Vehicle Connection Issues
```bash
# Check AirSim connection
ros2 topic echo /Droan1/odom_local_ned --once
ros2 topic echo /PX4_Drone2/odom_local_ned --once

# Should show position data if connected to AirSim
```

## Quick Reference

### Mission Node Names
- Mission Coordinator: `/airsim_coordination_node`
- Vehicle Nodes: `/Droan1`, `/PX4_Drone2`, `/SimpleFlight3`, etc.

### Mission Actions (per vehicle)
- `/VehicleName/actions/search_area` - Area search mission
- `/VehicleName/actions/navigate_to_target` - Navigate to specific point
- `/VehicleName/actions/track_target` - Track moving target

### Mission Services
- `/airsim_coordination_node/services/plan_mission` - Plan multi-vehicle mission
- `/airsim_coordination_node/services/assign_search_zone` - Assign search zones to vehicles
- `/airsim_coordination_node/services/get_mission_status` - Get mission status
- `/VehicleName/services/get_capabilities` - Get vehicle capabilities
- `/VehicleName/services/set_search_pattern` - Configure vehicle search patterns

### Mission Topics
- `/airsim_coordination_node/mission_status` - Global mission status
- `/VehicleName/mission/status` - Individual vehicle mission status  
- `/VehicleName/detections/target` - Target detection results

## File Locations

**Mission Interfaces:**
- `L:\Cosys-AirSim\ros2\src\mission_search_interfaces\`

**Mission Node Source:**
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_coordination_node.cpp`
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_multirotor_node.cpp`

**Launch Files:**
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\launch\rpc_dynamic_vehicles.launch.py` - Main launch with mission mode support
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\launch\mission_coordination_demo.launch.py` - Dedicated mission demo
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\launch\search_mission.launch.py` - Search & rescue operations
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\launch\survey_mission.launch.py` - Area survey missions
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\launch\multi_vehicle_mission.launch.py` - Multi-vehicle coordination

**Mission System Documentation:**
- `L:\Cosys-AirSim\ros2\MISSION_SYSTEM_USAGE.md` - Comprehensive mission system guide

**Python Mission Scripts:**
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\searchtrack_mission_ros2.py` - GPS-based search & track
- `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\spiral_search_ros2.py` - Spiral search patterns