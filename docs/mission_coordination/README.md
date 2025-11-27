# 🎯 Mission Coordination Documentation
## Complete Guide for Autonomous Mission Systems in Cosys-AirSim

This documentation covers the comprehensive mission coordination system in Cosys-AirSim, integrating PX4 SITL, MAVLink communication, ROS2 bridges, and autonomous mission planning for multi-vehicle operations.

## 📋 Documentation Overview

### Core Mission Coordination
- **[PX4 SITL Mission Integration](px4_sitl_mission_integration.md)** - Complete guide for PX4 SITL integration with mission coordination
- **[Mission Planning Guide](mission_planning_guide.md)** - Advanced mission planning and execution strategies
- **[Multi-Vehicle Coordination](multi_vehicle_coordination.md)** - Coordinating multiple vehicles in complex missions

### Technical Integration
- **[MAVLink Mission Protocol](mavlink_mission_protocol.md)** - MAVLink integration for mission waypoints and commands
- **[ROS2 Mission Architecture](ros2_mission_architecture.md)** - ROS2 action/service architecture for mission coordination
- **[Coordinate Transformations](coordinate_transformations.md)** - Precision NED ↔ ENU transformations for missions

### Deployment and Operations
- **[Docker Mission Stack](docker_mission_stack.md)** - Complete containerized deployment guide
- **[Mission Monitoring](mission_monitoring.md)** - Real-time monitoring and diagnostics
- **[Failsafe and Recovery](failsafe_recovery.md)** - Emergency procedures and fault tolerance

### AI and Perception Integration
- **[AI Perception Integration](../mission_coordination_ai_perception_integration.md)** - AI-based target detection and tracking

## 🎯 Mission Coordination System Architecture

### Core Components

```
Mission Coordination System
├── 🎛️  Mission Coordinator (/mission_coordinator)
│   ├── Planning Service (PlanMission)
│   ├── Execution Action (ExecuteMission) 
│   └── Status Monitoring
├── 🚁 Vehicle Nodes (Ultra-Clean Architecture)
│   ├── /Droan1 (PX4 SITL Integration)
│   ├── /PX4_Drone2 (Mission-Capable)
│   └── /VehicleName (Dynamic Discovery)
├── 🔗 Mission Actions (Per Vehicle)
│   ├── SearchArea (Autonomous search patterns)
│   ├── NavigateToTarget (Precision navigation)
│   └── TrackTarget (Dynamic target following)
└── 🌐 Integration Layers
    ├── MAVLink Protocol (PX4 Communication)
    ├── ROS2 Bridge (Ultra-Clean Messaging)
    └── Coordinate Transforms (NED ↔ ENU)
```

### Key Message Types

| Interface Type | Message/Service | Purpose |
|----------------|-----------------|---------|
| **Actions** | `SearchArea` | Execute autonomous search patterns |
| **Actions** | `NavigateToTarget` | Navigate to specific coordinates |
| **Actions** | `TrackTarget` | Follow moving targets |
| **Actions** | `ExecuteMission` | Complete mission execution |
| **Services** | `PlanMission` | Generate mission plans |
| **Messages** | `MissionStatus` | Real-time mission state |
| **Messages** | `TargetDetection` | Target detection results |

## 🚁 Supported Vehicle Types and Capabilities

### PX4 SITL Integration
- **Multi-Vehicle Support**: Up to 10+ simultaneous PX4 SITL instances
- **Flight Modes**: Offboard, Mission, Return-to-Launch
- **Sensors**: GPS, IMU, Magnetometer, Barometer
- **Cameras**: RGB, Thermal, Depth, Segmentation
- **Communication**: MAVLink UDP, ROS2 Bridge

### Vehicle Capabilities Framework
```yaml
# Example vehicle capabilities
Droan1:
  vehicle_type: "PX4Multirotor"
  capabilities:
    - "high_resolution_camera"
    - "thermal_camera" 
    - "gps"
    - "long_endurance"
  max_speed: 15.0  # m/s
  max_altitude: 120.0  # meters
  endurance: 45  # minutes
  
PX4_Drone2:
  vehicle_type: "PX4Multirotor"
  capabilities:
    - "standard_camera"
    - "gps"
    - "magnetometer"
  max_speed: 12.0
  max_altitude: 100.0
  endurance: 35
```

## 🎯 Mission Types and Patterns

### Search and Rescue Missions
- **Grid Search**: Systematic area coverage
- **Spiral Search**: Efficient coverage from center outward
- **Random Search**: Probabilistic coverage for unknown targets
- **Target Tracking**: Dynamic following of detected targets

### Surveillance Missions
- **Perimeter Patrol**: Boundary monitoring
- **Point Surveillance**: Fixed-point observation
- **Route Surveillance**: Path monitoring
- **Area Surveillance**: Continuous area monitoring

### Inspection Missions
- **Infrastructure Inspection**: Buildings, bridges, power lines
- **Agricultural Monitoring**: Crop health, livestock tracking
- **Environmental Monitoring**: Pollution, wildlife observation
- **Security Inspection**: Facility security, crowd monitoring

## 🛠️ Quick Start Guide

### 1. Prerequisites Setup
```bash
# Install required packages
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo apt install python3-pymavlink python3-mavproxy

# Clone and build workspace
git clone https://github.com/your-org/Cosys-AirSim.git
cd Cosys-AirSim/ros2
colcon build --packages-select mission_search_interfaces airsim_ros_pkgs
source install/setup.bash
```

### 2. Launch Mission System
```bash
# Option A: Complete Docker Stack (Recommended)
cd docker_clean/mission_stack
./launch_complete_mission_stack.sh

# Option B: Manual Launch
# Terminal 1: Start AirSim with mission settings
# Terminal 2: Start PX4 SITL instances
# Terminal 3: Launch mission coordination
ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py
```

### 3. Execute Test Mission
```bash
# Run search and rescue demo
ros2 run airsim_ros_pkgs simple_mission_coordination_demo

# Or use Python client
python3 src/airsim_ros_pkgs/src/demo_mission_planner.py
```

## 📊 Mission Monitoring and Visualization

### ROS2 Topic Monitoring
```bash
# Monitor mission status
ros2 topic echo /mission_coordinator/mission_status

# Monitor vehicle positions
ros2 topic echo /Droan1/odom_local_ned

# Monitor target detections
ros2 topic echo /mission_coordinator/confirmed_detections
```

### RViz Visualization
```bash
# Launch RViz with mission configuration
ros2 launch airsim_ros_pkgs rviz.launch.py

# View:
# - Vehicle positions and paths
# - Search area boundaries
# - Target detection markers
# - Mission progress visualization
```

### Ground Control Station
```bash
# Launch QGroundControl for PX4 monitoring
qgroundcontrol

# Connect to PX4 instances:
# - UDP: localhost:14550 (aggregated)
# - TCP: localhost:5760 (individual vehicles)
```

## 🔧 Configuration and Customization

### AirSim Settings
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Droan1": {
      "VehicleType": "PX4Multirotor",
      "UdpPort": 14560,
      "ControlPort": 14580
    }
  }
}
```

### Mission Parameters
```yaml
# Mission coordination parameters
mission_coordinator:
  max_vehicles: 10
  default_altitude: 30.0
  search_patterns: ["grid", "spiral", "random"]
  collision_avoidance: true
  min_separation_distance: 10.0
```

## 🚨 Troubleshooting

### Common Issues
1. **MAVLink Connection Failed**
   - Check PX4 SITL is running: `pxh> mavlink status`
   - Verify AirSim MAVLink ports: `netstat -an | grep 14560`

2. **Vehicle Not Responding**
   - Check ROS2 nodes: `ros2 node list | grep Droan`
   - Verify action servers: `ros2 action list`

3. **Coordinate Frame Issues**
   - Test transformations: See coordinate_transformations.md
   - Check frame IDs in TF tree: `ros2 run tf2_tools view_frames.py`

### Debug Commands
```bash
# Check mission system health
ros2 service call /mission_coordinator/health_check

# Monitor action server status
ros2 action list -t

# View active transforms
ros2 run tf2_ros tf2_echo map base_link
```

## 🔗 Related Documentation

### Core AirSim Integration
- **[Ultra-Clean ROS2 Architecture](../ros2/README_MULTIROTOR_ARCHITECTURE.md)** - Core ROS2 system
- **[PX4 Integration](../px4/README.md)** - PX4 setup and configuration
- **[Multi-Vehicle Setup](../core/multi_vehicle.md)** - General multi-vehicle configuration

### Advanced Features
- **[AI Perception Integration](../mission_coordination_ai_perception_integration.md)** - Computer vision for missions
- **[MAVLink Architecture](../px4/mavlink_architecture.md)** - Communication protocols
- **[Docker Integration](../installation/docker_ubuntu.md)** - Containerized deployment

## 📞 Support and Community

- **GitHub Issues**: Report bugs and request features
- **Documentation Wiki**: Community-contributed guides
- **Discord/Slack**: Real-time community support
- **Email Support**: professional-support@cosys-airsim.com

---

**Note**: This mission coordination system is under active development. Check the [CHANGELOG](../../CHANGELOG.md) for latest updates and features.