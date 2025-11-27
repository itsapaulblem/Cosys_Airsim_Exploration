# Mission System Usage Guide

This guide provides comprehensive instructions for using the mission-based search and coordination system with the ultra-clean ROS2 architecture.

## System Architecture Overview

### Ultra-Clean Naming Convention
Our architecture uses **Vehicle-Name-As-Node-Name** for maximum simplicity:
```
Docker ROS2 Container:
├── /PX4_Drone1 (MissionMultirotorNode)
│   ├── Actions: /PX4_Drone1/actions/search_area, /PX4_Drone1/actions/navigate_to_target, /PX4_Drone1/actions/track_target
│   ├── Services: /PX4_Drone1/services/set_search_pattern, /PX4_Drone1/services/get_capabilities
│   └── Topics: /PX4_Drone1/mission/status, /PX4_Drone1/detections/target
├── /PX4_Drone2 (MissionMultirotorNode)
│   └── [Same mission capabilities with /PX4_Drone2/ prefix]
└── /mission_coordinator (MissionCoordinationNode)
    ├── Actions: /mission_coordinator/actions/execute_mission
    ├── Services: /mission_coordinator/services/plan_mission, /mission_coordinator/services/assign_zones
    └── Topics: /mission_coordinator/mission_status, /mission_coordinator/zone_assignments
```

### Key Components
- **Mission-Capable Vehicle Nodes**: Individual ROS2 nodes for each vehicle with mission action servers
- **Mission Coordination Node**: Global orchestration and multi-vehicle coordination
- **Cross-Platform Communication**: Docker ROS2 ↔ Windows AirSim via RPC
- **Mission Interfaces**: Complete ROS2 message/service/action definitions

## Quick Start Guide

### Prerequisites
1. **AirSim running on Windows** with multi-vehicle settings
2. **Docker ROS2 container** with mission packages built
3. **Network connectivity** between Windows AirSim and Docker ROS2

### 1. Start Mission System
```bash
# In Docker container
cd /airsim_ros2_ws
source install/setup.bash

# Start mission coordination demo
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true
```

### 2. Verify System Status
```bash
# Check nodes are running
ros2 node list
# Expected: /PX4_Drone1, /PX4_Drone2, /mission_coordinator

# Check action servers are available
ros2 action list
# Expected: /PX4_Drone1/actions/search_area, /PX4_Drone2/actions/search_area, etc.
```

### 3. Send Your First Mission
```bash
# Test mission client
ros2 run airsim_ros_pkgs test_mission_client.py

# Or send manual search area goal
ros2 action send_goal /PX4_Drone1/actions/search_area mission_search_interfaces/action/SearchArea "{
  search_boundary: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 50.0, y: 0.0, z: 0.0},
      {x: 50.0, y: 50.0, z: 0.0},
      {x: 0.0, y: 50.0, z: 0.0}
    ]
  },
  search_pattern: 'spiral',
  search_altitude: 25.0,
  search_speed: 5.0,
  pattern_spacing: 10.0,
  enable_detection: true,
  detection_confidence_threshold: 0.7
}"
```

## Detailed Usage Examples

### Search Area Operations

#### 1. Basic Search Mission
```bash
# Send search area goal to PX4_Drone1
ros2 action send_goal /PX4_Drone1/actions/search_area mission_search_interfaces/action/SearchArea "{
  search_boundary: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 100.0, y: 0.0, z: 0.0},
      {x: 100.0, y: 100.0, z: 0.0},
      {x: 0.0, y: 100.0, z: 0.0}
    ]
  },
  search_pattern: 'grid',
  search_altitude: 30.0,
  search_speed: 8.0,
  pattern_spacing: 15.0,
  enable_detection: true,
  detection_confidence_threshold: 0.8,
}" --feedback
```

#### 2. Monitor Search Progress
```bash
# Monitor mission status
ros2 topic echo /PX4_Drone1/mission/status

# Monitor target detections
ros2 topic echo /PX4_Drone1/detections/target
```

#### 3. Search Pattern Types
- **`spiral`**: Outward spiral from center (good for small areas)
- **`grid`**: Systematic grid pattern (comprehensive coverage)
- **`lawnmower`**: Back-and-forth pattern (efficient for rectangular areas)

### Navigation Operations

#### 1. Navigate to Specific Target
```bash
ros2 action send_goal /PX4_Drone2/actions/navigate_to_target mission_search_interfaces/action/NavigateToTarget "{
  target_location: {x: 150.0, y: 75.0, z: 25.0},
  approach_altitude: 20.0,
  approach_speed: 6.0,
  hover_duration_seconds: 10.0,
  mission_id: 'investigate_target_001'
}" --feedback
```

#### 2. Track Moving Target
```bash
ros2 action send_goal /PX4_Drone1/actions/track_target mission_search_interfaces/action/TrackTarget "{
  initial_target_location: {x: 200.0, y: 100.0, z: 0.0},
  tracking_altitude: 30.0,
  tracking_distance: 15.0,
  max_tracking_speed: 10.0,
  tracking_duration_seconds: 120.0,
  mission_id: 'track_suspect_001'
}" --feedback
```

### Service Operations

#### 1. Configure Search Patterns
```bash
# Set search pattern for vehicle
ros2 service call /PX4_Drone1/services/set_search_pattern mission_search_interfaces/srv/SetSearchPattern "{
  pattern_type: 'spiral',
  pattern_spacing: 12.0,
  search_speed: 5.0,
  altitude: 25.0,
  enable_detection: true,
  confidence_threshold: 0.75
}"
```

#### 2. Get Vehicle Capabilities
```bash
# Query vehicle capabilities
ros2 service call /PX4_Drone2/services/get_capabilities mission_search_interfaces/srv/GetVehicleCapabilities "{}"
```

### Multi-Vehicle Coordination

#### 1. Plan Multi-Vehicle Mission
```bash
# Plan coordinated mission
ros2 service call /mission_coordinator/services/plan_mission mission_search_interfaces/srv/PlanMission "{
  mission_name: 'Search and Rescue Alpha',
  search_areas: [
    {
      zone_id: 'sector_1',
      boundary_polygon: {
        points: [
          {x: 0.0, y: 0.0, z: 0.0},
          {x: 50.0, y: 0.0, z: 0.0},
          {x: 50.0, y: 50.0, z: 0.0},
          {x: 0.0, y: 50.0, z: 0.0}
        ]
      },
      search_pattern: 'grid',
      priority: 1
    }
  ],
  available_vehicles: ['PX4_Drone1', 'PX4_Drone2'],
  mission_timeout_minutes: 30
}"
```

#### 2. Execute Coordinated Mission
```bash
# Execute planned mission
ros2 action send_goal /mission_coordinator/actions/execute_mission mission_search_interfaces/action/ExecuteMission "{
  mission_plan: {
    mission_id: 'mission_001',
    mission_name: 'Search and Rescue Alpha',
    participating_vehicles: ['PX4_Drone1', 'PX4_Drone2']
  }
}" --feedback
```

#### 3. Monitor Coordination Status
```bash
# Monitor global mission status
ros2 topic echo /mission_coordinator/mission_status

# Monitor zone assignments
ros2 topic echo /mission_coordinator/zone_assignments
```

## Common Mission Scenarios

### Scenario 1: Single Vehicle Search & Rescue
```bash
# 1. Start mission system
ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py

# 2. Configure search pattern
ros2 service call /PX4_Drone1/services/set_search_pattern mission_search_interfaces/srv/SetSearchPattern "{
  pattern_type: 'spiral',
  pattern_spacing: 8.0,
  search_speed: 6.0,
  altitude: 20.0,
  enable_detection: true,
  confidence_threshold: 0.8
}"

# 3. Execute search mission
ros2 action send_goal /PX4_Drone1/actions/search_area mission_search_interfaces/action/SearchArea "{
  search_boundary: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 80.0, y: 0.0, z: 0.0},
      {x: 80.0, y: 80.0, z: 0.0},
      {x: 0.0, y: 80.0, z: 0.0}
    ]
  },
  search_pattern: 'spiral',
  search_altitude: 20.0,
  search_speed: 6.0,
  pattern_spacing: 8.0,
  enable_detection: true,
  detection_confidence_threshold: 0.8,
  mission_id: 'sar_001'
}"

# 4. Monitor in separate terminals
ros2 topic echo /PX4_Drone1/mission/status
ros2 topic echo /PX4_Drone1/detections/target
```

### Scenario 2: Multi-Vehicle Area Survey
```bash
# 1. Start mission system
ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py

# 2. Plan multi-vehicle mission
ros2 service call /mission_coordinator/services/plan_mission mission_search_interfaces/srv/PlanMission "{
  mission_name: 'Area Survey Delta',
  search_areas: [
    {
      zone_id: 'north_sector',
      boundary_polygon: {
        points: [
          {x: 0.0, y: 0.0, z: 0.0},
          {x: 100.0, y: 0.0, z: 0.0},
          {x: 100.0, y: 50.0, z: 0.0},
          {x: 0.0, y: 50.0, z: 0.0}
        ]
      },
      search_pattern: 'grid',
      priority: 1
    },
    {
      zone_id: 'south_sector',
      boundary_polygon: {
        points: [
          {x: 0.0, y: 60.0, z: 0.0},
          {x: 100.0, y: 60.0, z: 0.0},
          {x: 100.0, y: 110.0, z: 0.0},
          {x: 0.0, y: 110.0, z: 0.0}
        ]
      },
      search_pattern: 'lawnmower',
      priority: 2
    }
  ],
  available_vehicles: ['PX4_Drone1', 'PX4_Drone2'],
  mission_timeout_minutes: 45
}"

# 3. Execute coordinated mission
ros2 action send_goal /mission_coordinator/actions/execute_mission mission_search_interfaces/action/ExecuteMission "{
  mission_plan: {
    mission_id: 'survey_001',
    mission_name: 'Area Survey Delta',
    participating_vehicles: ['PX4_Drone1', 'PX4_Drone2']
  }
}"
```

### Scenario 3: Target Investigation
```bash
# 1. Navigate to suspected target location
ros2 action send_goal /PX4_Drone2/actions/navigate_to_target mission_search_interfaces/action/NavigateToTarget "{
  target_location: {x: 125.0, y: 85.0, z: 15.0},
  approach_altitude: 25.0,
  approach_speed: 4.0,
  hover_duration_seconds: 30.0,
  mission_id: 'investigate_001'
}"

# 2. If target confirmed, track it
ros2 action send_goal /PX4_Drone2/actions/track_target mission_search_interfaces/action/TrackTarget "{
  initial_target_location: {x: 125.0, y: 85.0, z: 0.0},
  tracking_altitude: 20.0,
  tracking_distance: 12.0,
  max_tracking_speed: 8.0,
  tracking_duration_seconds: 300.0,
  mission_id: 'track_001'
}"
```

## ROS2 Command Reference

### Available Actions
| Action | Node | Description | Interface |
|--------|------|-------------|-----------|
| `search_area` | Vehicle nodes | Execute search pattern in defined area | `mission_search_interfaces/action/SearchArea` |
| `navigate_to_target` | Vehicle nodes | Navigate to specific coordinates | `mission_search_interfaces/action/NavigateToTarget` |
| `track_target` | Vehicle nodes | Track moving target | `mission_search_interfaces/action/TrackTarget` |
| `execute_mission` | Mission coordinator | Execute multi-vehicle mission plan | `mission_search_interfaces/action/ExecuteMission` |

### Available Services
| Service | Node | Description | Interface |
|---------|------|-------------|-----------|
| `set_search_pattern` | Vehicle nodes | Configure search parameters | `mission_search_interfaces/srv/SetSearchPattern` |
| `get_capabilities` | Vehicle nodes | Query vehicle capabilities | `mission_search_interfaces/srv/GetVehicleCapabilities` |
| `plan_mission` | Mission coordinator | Plan multi-vehicle missions | `mission_search_interfaces/srv/PlanMission` |
| `assign_zones` | Mission coordinator | Assign search zones to vehicles | `mission_search_interfaces/srv/AssignSearchZone` |

### Available Topics

#### Mission & Movement Topics (Per Vehicle)
| Topic | Message Type | Update Rate | Description |
|-------|--------------|-------------|-------------|
| `/{VehicleName}/mission/status` | `mission_search_interfaces/msg/MissionStatus` | ~2 Hz | **Mission Progress & Metrics**<br/>• Progress percentage, current activity, mission ID<br/>• Performance metrics: targets detected, waypoints completed, area covered<br/>• Timing: start time, elapsed time, estimated remaining time<br/>• Vehicle state and battery information |
| `/{VehicleName}/mission/events` | `mission_search_interfaces/msg/MissionEvent` | Event-based | **Movement Event Detection**<br/>• Event types: TAKEOFF, LANDING, MOVEMENT, COMMAND_START<br/>• Position/velocity/altitude change analysis<br/>• Distance moved, speed changes, timing between events<br/>• Mission context and sequence tracking<br/>• Event source classification (ROS2_ACTION, DIRECT_CLIENT) |
| `/{VehicleName}/detections/target` | `mission_search_interfaces/msg/TargetDetection` | Event-based | **Target Detection Results**<br/>• Detection metadata: confidence, classification, world position<br/>• Image coordinates and bounding box information<br/>• Detection timestamp and vehicle context |

#### Base Vehicle Topics (Per Vehicle)  
| Topic | Message Type | Update Rate | Description |
|-------|--------------|-------------|-------------|
| `/{VehicleName}/odom_local_ned` | `nav_msgs/msg/Odometry` | ~20 Hz | **High-Frequency Movement Data**<br/>• Precise position and velocity in NED coordinates<br/>• Orientation quaternion and angular velocity<br/>• Covariance matrices for uncertainty estimation |
| `/{VehicleName}/imu` | `sensor_msgs/msg/Imu` | ~50 Hz | **Inertial Measurement Data**<br/>• Linear acceleration and angular velocity<br/>• Orientation estimation<br/>• Raw sensor data for movement analysis |
| `/{VehicleName}/gps_data` | `sensor_msgs/msg/NavSatFix` | ~10 Hz | **GPS Position Updates**<br/>• Global coordinates (latitude, longitude, altitude)<br/>• Position accuracy and GPS status information |

#### Coordination Topics (Mission Coordinator)
| Topic | Message Type | Update Rate | Description |
|-------|--------------|-------------|-------------|
| `/mission_coordinator/mission_status` | `mission_search_interfaces/msg/MissionStatus` | ~1 Hz | **Global Mission Orchestration**<br/>• Multi-vehicle mission coordination status<br/>• Overall mission progress and vehicle assignments |
| `/mission_coordinator/zone_assignments` | `mission_search_interfaces/msg/SearchZone[]` | Event-based | **Zone Allocation Updates**<br/>• Search area assignments to specific vehicles<br/>• Zone priority and coverage requirements |

### Useful ROS2 Commands
```bash
# List all nodes
ros2 node list

# List all action servers
ros2 action list

# List all services  
ros2 service list

# List all topics
ros2 topic list

# Get action interface details
ros2 interface show mission_search_interfaces/action/SearchArea

# Monitor action feedback
ros2 action send_goal /PX4_Drone1/actions/search_area mission_search_interfaces/action/SearchArea "{...}" --feedback

# Cancel running action
ros2 action send_goal /PX4_Drone1/actions/search_area mission_search_interfaces/action/SearchArea "{...}" --feedback

# Real-time topic monitoring
ros2 topic echo /PX4_Drone1/mission/status

# Service call with response
ros2 service call /PX4_Drone1/services/get_capabilities mission_search_interfaces/srv/GetVehicleCapabilities "{}"
```

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. Mission Nodes Not Starting
**Symptoms:** Nodes don't appear in `ros2 node list`
```bash
# Check container status
./airsim_ros2_docker.bat status

# Check container logs
./airsim_ros2_docker.bat logs --follow

# Rebuild if needed
cd /airsim_ros2_ws
colcon build --packages-select mission_search_interfaces airsim_ros_pkgs
source install/setup.bash
```

#### 2. Action Servers Not Available
**Symptoms:** `ros2 action list` doesn't show vehicle action servers
```bash
# Check if nodes are running
ros2 node list

# Check node-specific logs
ros2 run airsim_ros_pkgs mission_multirotor_node --ros-args --log-level DEBUG

# Verify AirSim connection
ros2 topic echo /PX4_Drone1/odom_local_ned --timeout 5
```

#### 3. AirSim Connection Errors
**Symptoms:** "Vehicle API not available" errors
```bash
# Check AirSim is running on Windows
# Check Windows Firewall allows Docker connections
# Test network connectivity
ping 172.28.240.1  # or your Windows IP

# Check AirSim settings.json has correct vehicle names
```

#### 4. Mission Goals Rejected
**Symptoms:** Action goals are rejected or fail immediately
```bash
# Check vehicle capabilities
ros2 service call /PX4_Drone1/services/get_capabilities mission_search_interfaces/srv/GetVehicleCapabilities "{}"

# Verify search area is reasonable size
# Check altitude constraints
# Ensure vehicle is armed and ready
# Check that 0.0 values are accepted (default handling now improved)
```

#### 5. No Target Detections
**Symptoms:** Search completes but no targets detected
```bash
# Verify detection is enabled
# Check confidence threshold (try lower values)
# Test with known segmentation objects in scene
# Monitor detection topic during search
ros2 topic echo /PX4_Drone1/detections/target
```

#### 6. Vehicle Preparation Failed (FIXED)
**Symptoms:** "vehicle_preparation_failed" error during mission start
**Status:** This issue has been resolved in recent updates
```bash
# The following improvements have been implemented:
# - Enhanced vehicle readiness checks
# - Better RPC connection validation
# - Improved error handling during preparation phase
# - Thread-safe state management

# If still experiencing issues, check:
ros2 topic echo /PX4_Drone1/mission/status
# Look for detailed error messages in the status updates
```

#### 7. Zero Value Parameter Issues (FIXED)
**Symptoms:** Commands with 0.0 values being rejected
**Status:** Default value handling has been improved
```bash
# These are now valid and accepted:
# - {x: 0.0, y: 0.0, z: 25.0} for takeoff at origin
# - hover_duration_seconds: 0.0 for no hover
# - pattern_spacing: 0.0 for default spacing

# No longer need to use small non-zero values as workarounds
```

#### 8. Field Name Errors (FIXED)
**Symptoms:** "Unknown field" errors in action calls
**Status:** All interfaces now use correct field names
```bash
# Use target_location (not target_position):
ros2 action send_goal /PX4_Drone1/actions/navigate_to_target mission_search_interfaces/action/NavigateToTarget "{
  target_location: {x: 100.0, y: 50.0, z: 25.0}
}"

# Use initial_target_location (not initial_target_position):
ros2 action send_goal /PX4_Drone1/actions/track_target mission_search_interfaces/action/TrackTarget "{
  initial_target_location: {x: 100.0, y: 50.0, z: 0.0}
}"
```

### Debug Commands
```bash
# Verbose node logging
ros2 run airsim_ros_pkgs mission_multirotor_node --ros-args --log-level DEBUG

# Check action server health
ros2 action info /PX4_Drone1/actions/search_area

# Service call testing
ros2 service call /PX4_Drone1/services/get_capabilities mission_search_interfaces/srv/GetVehicleCapabilities "{}"

# Network diagnostics
ros2 topic hz /PX4_Drone1/odom_local_ned  # Should show ~20 Hz
```

## Movement Statistics & Data Collection

The mission system provides comprehensive movement tracking through a sophisticated event detection system and high-frequency data streams. This section explains how to collect and analyze drone movement statistics.

### Understanding the Event Detection System

The mission nodes implement an intelligent event detection system with configurable thresholds:

```cpp
// Event Detection Thresholds (from mission_multirotor_node.cpp)
position_change_threshold_ = 0.5;  // meters
velocity_change_threshold_ = 1.0;  // m/s  
altitude_change_threshold_ = 0.3;  // meters
event_time_threshold_ = 1.0;       // seconds between similar events
```

#### Event Types Detected:
- **TAKEOFF**: Altitude transition from <1.0m to >3.0m
- **LANDING**: Altitude transition from >3.0m to <1.0m  
- **MOVEMENT**: Significant position changes (>0.5m horizontal or >0.3m vertical)
- **COMMAND_START**: Significant velocity changes (>1.0 m/s difference)

### Movement Data Sources

#### 1. Mission Events Topic (`/{VehicleName}/mission/events`)
**Best for**: Discrete movement analysis, flight phase detection
```bash
# Monitor movement events for PX4_Drone1
ros2 topic echo /PX4_Drone1/mission/events
```

**Key Fields for Statistics:**
```yaml
event_type: "MOVEMENT"  # TAKEOFF, LANDING, MOVEMENT, COMMAND_START
distance_moved: 15.3    # meters moved since last event  
altitude_change: 2.1    # meters altitude difference
speed_change: 3.5       # m/s speed difference
time_since_last_event:  # duration since previous event
previous_position: {x, y, z}
current_position: {x, y, z}
previous_velocity: {x, y, z}  
current_velocity: {x, y, z}
event_source: "ROS2_ACTION"  # ROS2_ACTION, DIRECT_CLIENT, UNKNOWN
```

#### 2. Mission Status Topic (`/{VehicleName}/mission/status`) 
**Best for**: Mission-level statistics, progress tracking
```bash
# Monitor mission statistics
ros2 topic echo /PX4_Drone1/mission/status
```

**Key Fields for Statistics:**
```yaml
progress_percentage: 67.5
waypoints_completed: 15
area_covered_sq_m: 1250.0
targets_detected: 3
mission_start_time: <timestamp>
estimated_remaining_time: <duration>
current_activity: "Navigating to waypoint 16"
```

#### 3. High-Frequency Odometry (`/{VehicleName}/odom_local_ned`)
**Best for**: Precise movement analysis, path tracking
```bash
# Monitor real-time position/velocity
ros2 topic echo /PX4_Drone1/odom_local_ned --field pose.pose.position
```

### Implementing a Movement Statistics Logger

#### Basic Python ROS2 Statistics Collector

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mission_search_interfaces.msg import MissionEvent, MissionStatus
from nav_msgs.msg import Odometry
import sqlite3
from datetime import datetime
import math

class DroneMovementStatsLogger(Node):
    def __init__(self):
        super().__init__('drone_stats_logger')
        
        # Database setup
        self.setup_database()
        
        # Vehicle tracking
        self.active_vehicles = set()
        self.vehicle_stats = {}
        
        # Discover and subscribe to all vehicle topics
        self.discover_vehicles()
        
        # Timer for periodic statistics updates
        self.stats_timer = self.create_timer(10.0, self.log_periodic_stats)
        
    def setup_database(self):
        """Initialize SQLite database for statistics storage."""
        self.conn = sqlite3.connect('drone_movement_stats.db')
        cursor = self.conn.cursor()
        
        # Movement events table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS movement_events (
                id INTEGER PRIMARY KEY,
                vehicle_name TEXT,
                timestamp REAL,
                event_type TEXT,
                distance_moved REAL,
                altitude_change REAL,
                speed_change REAL,
                position_x REAL,
                position_y REAL,
                position_z REAL,
                mission_id TEXT,
                event_source TEXT
            )
        ''')
        
        # Mission statistics table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS mission_stats (
                id INTEGER PRIMARY KEY,
                vehicle_name TEXT,
                timestamp REAL,
                mission_id TEXT,
                progress_percentage REAL,
                waypoints_completed INTEGER,
                area_covered REAL,
                targets_detected INTEGER,
                current_activity TEXT
            )
        ''')
        
        self.conn.commit()
    
    def discover_vehicles(self):
        """Dynamically discover active vehicles and subscribe to their topics."""
        # Get list of active nodes
        node_names = self.get_node_names()
        
        for node_name in node_names:
            # Look for vehicle nodes (PX4_Drone1, PX4_Drone2, etc.)
            if node_name.startswith('/') and 'Drone' in node_name:
                vehicle_name = node_name.strip('/')
                if vehicle_name not in self.active_vehicles:
                    self.subscribe_to_vehicle(vehicle_name)
                    self.active_vehicles.add(vehicle_name)
                    self.vehicle_stats[vehicle_name] = {
                        'total_distance': 0.0,
                        'flight_time': 0.0,
                        'takeoffs': 0,
                        'landings': 0,
                        'missions_completed': 0,
                        'last_position': None,
                        'last_timestamp': None
                    }
    
    def subscribe_to_vehicle(self, vehicle_name):
        """Subscribe to all relevant topics for a specific vehicle."""
        self.get_logger().info(f"Subscribing to topics for vehicle: {vehicle_name}")
        
        # Mission events subscription
        events_topic = f'/{vehicle_name}/mission/events'
        self.create_subscription(
            MissionEvent, 
            events_topic,
            lambda msg, vname=vehicle_name: self.mission_event_callback(msg, vname),
            10
        )
        
        # Mission status subscription  
        status_topic = f'/{vehicle_name}/mission/status'
        self.create_subscription(
            MissionStatus,
            status_topic, 
            lambda msg, vname=vehicle_name: self.mission_status_callback(msg, vname),
            10
        )
        
        # Odometry subscription for high-frequency tracking
        odom_topic = f'/{vehicle_name}/odom_local_ned'
        self.create_subscription(
            Odometry,
            odom_topic,
            lambda msg, vname=vehicle_name: self.odometry_callback(msg, vname),
            10
        )
    
    def mission_event_callback(self, msg, vehicle_name):
        """Process mission events for movement statistics."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Update vehicle-specific counters
        if msg.event_type == 'TAKEOFF':
            self.vehicle_stats[vehicle_name]['takeoffs'] += 1
        elif msg.event_type == 'LANDING':  
            self.vehicle_stats[vehicle_name]['landings'] += 1
        
        # Accumulate total distance
        if msg.distance_moved > 0:
            self.vehicle_stats[vehicle_name]['total_distance'] += msg.distance_moved
        
        # Store event in database
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO movement_events (
                vehicle_name, timestamp, event_type, distance_moved,
                altitude_change, speed_change, position_x, position_y, position_z,
                mission_id, event_source
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            vehicle_name, timestamp, msg.event_type, msg.distance_moved,
            msg.altitude_change, msg.speed_change, 
            msg.current_position.x, msg.current_position.y, msg.current_position.z,
            msg.active_mission_id, msg.event_source
        ))
        self.conn.commit()
        
        self.get_logger().info(
            f"Logged {msg.event_type} event for {vehicle_name}: "
            f"moved {msg.distance_moved:.1f}m"
        )
    
    def mission_status_callback(self, msg, vehicle_name):
        """Process mission status updates."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Store mission statistics
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO mission_stats (
                vehicle_name, timestamp, mission_id, progress_percentage,
                waypoints_completed, area_covered, targets_detected, current_activity
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            vehicle_name, timestamp, msg.mission_id, msg.progress_percentage,
            msg.waypoints_completed, msg.area_covered_sq_m, 
            msg.targets_detected, msg.current_activity
        ))
        self.conn.commit()
    
    def odometry_callback(self, msg, vehicle_name):
        """Process high-frequency odometry for precise distance tracking."""
        current_pos = msg.pose.pose.position
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        stats = self.vehicle_stats[vehicle_name]
        
        if stats['last_position'] is not None:
            # Calculate distance traveled since last update
            dx = current_pos.x - stats['last_position'].x
            dy = current_pos.y - stats['last_position'].y  
            dz = current_pos.z - stats['last_position'].z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Only count significant movements (filter noise)
            if distance > 0.1:  # 10cm threshold
                stats['total_distance'] += distance
                
            # Update flight time
            if stats['last_timestamp'] is not None:
                stats['flight_time'] += (current_time - stats['last_timestamp'])
        
        stats['last_position'] = current_pos
        stats['last_timestamp'] = current_time
    
    def log_periodic_stats(self):
        """Log periodic statistics summary."""
        for vehicle_name, stats in self.vehicle_stats.items():
            self.get_logger().info(
                f"Stats for {vehicle_name}: "
                f"Distance: {stats['total_distance']:.1f}m, "
                f"Flight time: {stats['flight_time']:.1f}s, "
                f"Takeoffs: {stats['takeoffs']}, "
                f"Landings: {stats['landings']}"
            )
    
    def generate_report(self, vehicle_name=None):
        """Generate movement statistics report."""
        cursor = self.conn.cursor()
        
        if vehicle_name:
            # Vehicle-specific report
            cursor.execute('''
                SELECT event_type, COUNT(*) as count, 
                       SUM(distance_moved) as total_distance,
                       AVG(distance_moved) as avg_distance
                FROM movement_events 
                WHERE vehicle_name = ?
                GROUP BY event_type
            ''', (vehicle_name,))
        else:
            # Fleet-wide report
            cursor.execute('''
                SELECT vehicle_name, event_type, COUNT(*) as count,
                       SUM(distance_moved) as total_distance
                FROM movement_events
                GROUP BY vehicle_name, event_type
            ''')
        
        return cursor.fetchall()

def main():
    rclpy.init()
    stats_logger = DroneMovementStatsLogger()
    
    try:
        rclpy.spin(stats_logger)
    except KeyboardInterrupt:
        # Generate final report on shutdown
        print("\nFinal Movement Statistics Report:")
        report = stats_logger.generate_report()
        for row in report:
            print(f"Vehicle: {row[0]}, Event: {row[1]}, Count: {row[2]}, Distance: {row[3]:.1f}m")
    finally:
        stats_logger.conn.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### CLI Statistics Analysis Tool

```python
#!/usr/bin/env python3
"""
CLI tool for analyzing drone movement statistics
Usage: python3 stats_analyzer.py --vehicle PX4_Drone1 --report daily
"""
import sqlite3
import argparse
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import pandas as pd

class DroneStatsAnalyzer:
    def __init__(self, db_path='drone_movement_stats.db'):
        self.conn = sqlite3.connect(db_path)
    
    def get_vehicle_summary(self, vehicle_name, days=7):
        """Get comprehensive vehicle statistics summary."""
        cursor = self.conn.cursor()
        
        # Time range
        end_time = datetime.now().timestamp()
        start_time = (datetime.now() - timedelta(days=days)).timestamp()
        
        # Movement events summary
        cursor.execute('''
            SELECT 
                event_type,
                COUNT(*) as count,
                SUM(distance_moved) as total_distance,
                AVG(distance_moved) as avg_distance,
                MAX(distance_moved) as max_distance
            FROM movement_events
            WHERE vehicle_name = ? AND timestamp BETWEEN ? AND ?
            GROUP BY event_type
        ''', (vehicle_name, start_time, end_time))
        
        events = cursor.fetchall()
        
        # Mission statistics
        cursor.execute('''
            SELECT 
                COUNT(DISTINCT mission_id) as missions,
                AVG(progress_percentage) as avg_progress,
                SUM(area_covered) as total_area,
                SUM(targets_detected) as total_targets
            FROM mission_stats
            WHERE vehicle_name = ? AND timestamp BETWEEN ? AND ?
        ''', (vehicle_name, start_time, end_time))
        
        mission_stats = cursor.fetchone()
        
        return {
            'events': events,
            'missions': mission_stats
        }
    
    def plot_movement_timeline(self, vehicle_name, days=1):
        """Generate movement timeline visualization."""
        cursor = self.conn.cursor()
        
        end_time = datetime.now().timestamp()
        start_time = (datetime.now() - timedelta(days=days)).timestamp()
        
        cursor.execute('''
            SELECT timestamp, event_type, distance_moved, altitude_change
            FROM movement_events
            WHERE vehicle_name = ? AND timestamp BETWEEN ? AND ?
            ORDER BY timestamp
        ''', (vehicle_name, start_time, end_time))
        
        data = cursor.fetchall()
        if not data:
            print(f"No data found for {vehicle_name} in the last {days} days")
            return
        
        # Convert to DataFrame for easier plotting
        df = pd.DataFrame(data, columns=['timestamp', 'event_type', 'distance', 'altitude_change'])
        df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')
        
        # Create subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Distance moved over time
        ax1.scatter(df['datetime'], df['distance'], c=df['distance'], cmap='viridis', alpha=0.7)
        ax1.set_ylabel('Distance Moved (m)')
        ax1.set_title(f'{vehicle_name} Movement Analysis - Last {days} Days')
        ax1.grid(True, alpha=0.3)
        
        # Altitude changes over time
        colors = {'TAKEOFF': 'green', 'LANDING': 'red', 'MOVEMENT': 'blue', 'COMMAND_START': 'orange'}
        for event_type in df['event_type'].unique():
            mask = df['event_type'] == event_type
            ax2.scatter(df[mask]['datetime'], df[mask]['altitude_change'], 
                       label=event_type, color=colors.get(event_type, 'gray'), alpha=0.7)
        
        ax2.set_ylabel('Altitude Change (m)')
        ax2.set_xlabel('Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{vehicle_name}_movement_timeline.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"Movement timeline saved as {vehicle_name}_movement_timeline.png")

def main():
    parser = argparse.ArgumentParser(description='Analyze drone movement statistics')
    parser.add_argument('--vehicle', required=True, help='Vehicle name (e.g., PX4_Drone1)')
    parser.add_argument('--report', choices=['summary', 'timeline'], default='summary',
                       help='Type of report to generate')
    parser.add_argument('--days', type=int, default=7, help='Number of days to analyze')
    
    args = parser.parse_args()
    
    analyzer = DroneStatsAnalyzer()
    
    if args.report == 'summary':
        print(f"\n=== Movement Statistics Summary for {args.vehicle} ===")
        print(f"Analysis period: Last {args.days} days\n")
        
        stats = analyzer.get_vehicle_summary(args.vehicle, args.days)
        
        print("Movement Events:")
        for event_type, count, total_dist, avg_dist, max_dist in stats['events']:
            print(f"  {event_type:12}: {count:3} events, "
                  f"total: {total_dist:6.1f}m, avg: {avg_dist:5.1f}m, max: {max_dist:5.1f}m")
        
        print(f"\nMission Statistics:")
        missions, avg_progress, total_area, total_targets = stats['missions']
        print(f"  Missions: {missions}")
        print(f"  Average Progress: {avg_progress:.1f}%")
        print(f"  Total Area Covered: {total_area:.1f} sq.m")
        print(f"  Total Targets Detected: {total_targets}")
    
    elif args.report == 'timeline':
        analyzer.plot_movement_timeline(args.vehicle, args.days)

if __name__ == '__main__':
    main()
```

### Usage Examples

#### 1. Start Movement Statistics Logger
```bash
# Run in background to collect all vehicle data
python3 drone_stats_logger.py
```

#### 2. Real-time Movement Monitoring  
```bash
# Monitor specific vehicle events
ros2 topic echo /PX4_Drone1/mission/events --field event_type --field distance_moved

# Monitor multiple vehicles simultaneously
ros2 topic echo /PX4_Drone1/mission/events & ros2 topic echo /PX4_Drone2/mission/events &
```

#### 3. Generate Movement Reports
```bash
# Weekly summary for specific vehicle
python3 stats_analyzer.py --vehicle PX4_Drone1 --report summary --days 7

# Generate movement timeline visualization
python3 stats_analyzer.py --vehicle PX4_Drone1 --report timeline --days 1
```

#### 4. Query Raw Statistics Data
```bash
# Monitor mission progress in real-time
ros2 topic echo /PX4_Drone1/mission/status --field progress_percentage --field current_activity

# Track total distance from events
ros2 topic echo /PX4_Drone1/mission/events --field distance_moved
```

### Advanced Analytics Ideas

#### Fleet Efficiency Analysis
- Compare mission completion times across vehicles
- Analyze search pattern efficiency (spiral vs grid vs lawnmower)
- Battery life correlation with mission parameters

#### Predictive Maintenance
- Flight hour tracking per vehicle
- Takeoff/landing cycle counting
- Unusual movement pattern detection

#### Mission Optimization
- Waypoint efficiency analysis
- Target detection rate vs search parameters
- Area coverage optimization

## 🔗 Integration Examples

### Using Mission System in Python Scripts
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mission_search_interfaces.action import SearchArea
from geometry_msgs.msg import Point32

class MyMissionClient(Node):
    def __init__(self):
        super().__init__('my_mission_client')
        self.action_client = ActionClient(self, SearchArea, '/PX4_Drone1/actions/search_area')
        
    def send_search_mission(self):
        goal_msg = SearchArea.Goal()
        goal_msg.search_boundary.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=50.0, y=0.0, z=0.0),
            Point32(x=50.0, y=50.0, z=0.0),
            Point32(x=0.0, y=50.0, z=0.0)
        ]
        goal_msg.search_pattern = "spiral"
        goal_msg.search_altitude = 25.0
        goal_msg.search_speed = 5.0
        goal_msg.pattern_spacing = 10.0
        goal_msg.enable_detection = True
        goal_msg.detection_confidence_threshold = 0.7
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        return future

def main():
    rclpy.init()
    client = MyMissionClient()
    future = client.send_search_mission()
    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Custom Launch Configuration
```python
# custom_mission.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_type', default_value='search'),
        DeclareLaunchArgument('vehicle_count', default_value='2'),
        
        Node(
            package='airsim_ros_pkgs',
            executable='mission_multirotor_node',
            name='PX4_Drone1',
            parameters=[{
                'vehicle_name': 'PX4_Drone1',
                'mission_type': LaunchConfiguration('mission_type')
            }]
        ),
        
        # Add more nodes based on parameters...
    ])
```

## 📚 Advanced Topics

### Custom Search Patterns
Extend the mission system by implementing custom search patterns in the C++ mission nodes.

### Multi-Environment Deployment
Configure the mission system to work across multiple AirSim environments or real hardware.

### Integration with External Systems
Connect mission status and detection data to external command and control systems.

### Performance Optimization
Tune search parameters and vehicle coordination for optimal coverage and efficiency.

---

**Last Updated:** 2025-08-21  
**Version:** 1.1  
**Architecture:** Ultra-Clean ROS2 Multi-Node  

For questions or issues, refer to the troubleshooting section or check the ROS2 logs for detailed error information.