# airsim_ros_pkgs

A comprehensive ROS2 wrapper for Cosys-AirSim providing multi-vehicle simulation capabilities with advanced sensor integration.

## Current Implementation Status

### Architecture: Single-Node Centralized Design ✅

**Current State**: Production-ready single-node architecture with multi-vehicle support
**Core Component**: `AirsimROSWrapper` class serving as central orchestrator
**Vehicle Support**: MultiRotor, Car, and ComputerVision vehicles
**Multi-threading**: Parallel processing using specialized timers and callback groups

### Key Features

- **🚁 Multi-Vehicle Support**: Handle multiple vehicles simultaneously through unified interface
- **📡 Comprehensive Sensors**: Cameras, LiDAR, GPU LiDAR, IMU, GPS, Echo (radar/sonar), and more
- **⚡ High Performance**: Multi-threaded design with specialized RPC clients for optimal throughput
- **🔧 Configuration-Driven**: Automatic setup from AirSim settings.json
- **🛡️ Fault Tolerant**: Robust error handling and recovery mechanisms

### Build Architecture

```
airsim_ros_pkgs/
├── src/
│   ├── airsim_ros_wrapper.cpp          # Main wrapper implementation
│   ├── vehicles/
│   │   ├── VehicleNodeBase.cpp         # Base class for all vehicles
│   │   ├── MultiRotorNode.cpp          # Drone-specific implementation
│   │   ├── CarNode.cpp                 # Car-specific implementation
│   │   └── ComputerVisionNode.cpp      # Camera-only mode
│   ├── VehicleNodeFactory.cpp          # Factory pattern for vehicles
│   └── airsim_settings_parser.cpp      # Configuration parsing
├── include/
│   └── airsim_ros_pkgs/
│       └── airsim_ros_wrapper.h        # Main wrapper header
└── launch/
    ├── airsim_node.launch.py           # Primary launch file
    └── airsim_with_simple_PD_position_controller.launch.py
```

### Package Dependencies

- **Core**: `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
- **Interfaces**: `airsim_interfaces` (custom messages/services)
- **Vision**: `cv_bridge`, `image_transport`, `OpenCV`
- **Perception**: `pcl_conversions`, `PCL`
- **Transforms**: `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- **External**: `mavros_msgs`, `geographic_msgs`

### Interface Summary

#### Custom Messages
- `VelCmd`, `VelCmdGroup` - Velocity control commands
- `CarControls`, `CarState` - Car control and state
- `Environment` - Weather and environmental data
- `GimbalAngleEulerCmd`, `GimbalAngleQuatCmd` - Gimbal control

#### Custom Services
- `Takeoff`, `Land`, `TakeoffGroup`, `LandGroup` - Flight operations
- `SetAltitude`, `SetGPSPosition`, `SetLocalPosition` - Position control
- `CoordinatedHeightAndLand` - Multi-vehicle coordination
- `Reset` - Simulation reset

### Performance Characteristics

- **Scalability**: Linear scaling with vehicle count
- **Frequencies**: 
  - Vehicle state: ~50Hz
  - Camera data: ~30Hz
  - LiDAR data: ~10Hz
- **Multi-threading**: Parallel data processing with mutex protection
- **RPC Clients**: Specialized clients for different data types prevent blocking

### Evolution Path

**Current**: Single-node architecture (Production Ready) ✅
**Next**: Multi-node architecture implementation (In Progress) 🚧
- Phase 1-2: Vehicle node hierarchy ✅
- Phase 3-4: Dynamic launch system and testing 🔄

For detailed evolution plans, see `/ros2/MULTI_NODE_ROADMAP.md`

## Quick Start

```bash
# Build the workspace
cd ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Launch AirSim ROS2 wrapper
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

## Documentation

- **Full API Reference**: [docs/ros_cplusplus.md](../../docs/ros_cplusplus.md)
- **Usage Guide**: [ROS2_USAGE_GUIDE.md](../ROS2_USAGE_GUIDE.md)
- **Data Flow Explanation**: [AirSim_ROS2_DataFlow_Explanation.md](../AirSim_ROS2_DataFlow_Explanation.md)
- **Multi-Node Roadmap**: [MULTI_NODE_ROADMAP.md](../MULTI_NODE_ROADMAP.md)

## Testing

```bash
# Verify installation
ros2 node list
ros2 topic list
ros2 service list

# Monitor vehicle state
ros2 topic echo /airsim_node/drone_1/odom_local_ned

# Test basic flight
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'
```

## Contributing

The ROS2 wrapper is actively developed with focus on:
- Multi-node architecture implementation
- Performance optimization
- Enhanced sensor integration
- Improved fault tolerance

For contribution guidelines, see the main repository documentation.