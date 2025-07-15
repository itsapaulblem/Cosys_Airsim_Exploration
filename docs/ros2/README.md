# ROS2 Development Documentation

This section contains documentation for ROS2 integration with Cosys-AirSim, including both C++ and Python implementations.

## 🤖 ROS2 Wrapper Implementations

### C++ ROS2 Implementation (Recommended)
- **[ROS2 C++ Wrapper](ros_cplusplus.md)** - Complete C++ ROS2 wrapper documentation
  - Architecture overview and technical implementation
  - Build instructions and dependencies
  - API reference for topics, services, and parameters
  - Multi-vehicle support and configuration
  - Performance optimization and troubleshooting

### Python ROS2 Implementation
- **[ROS2 Python Wrapper](ros_python.md)** - Python-based ROS2 integration
  - Python client library usage
  - Custom ROS2 node development
  - Sensor data processing examples

## 🏗️ Implementation Architecture

### Current Status: Production Ready ✅
The ROS2 wrapper implements a **centralized, single-node architecture** with:
- **Multi-vehicle Support**: Handle multiple vehicles through unified interface
- **Comprehensive Sensors**: Cameras, LiDAR, GPU LiDAR, IMU, GPS, Echo sensors
- **High Performance**: Multi-threaded design with specialized RPC clients
- **Configuration-Driven**: Automatic setup from AirSim settings.json

### Key Components
- **AirsimROSWrapper**: Central orchestrator class
- **Vehicle Management**: Factory pattern for different vehicle types
- **Multi-threading**: Parallel processing with callback groups
- **RPC Architecture**: Specialized clients for optimal performance

## 📡 ROS2 Interface Summary

### Key Topics

#### Vehicle State (Published)
- `/airsim_node/VEHICLE-NAME/odom_local_ned` - Vehicle odometry
- `/airsim_node/VEHICLE-NAME/global_gps` - GPS coordinates
- `/airsim_node/VEHICLE-NAME/environment` - Environmental data

#### Vehicle Control (Subscribed)
- `/airsim_node/VEHICLE-NAME/vel_cmd_body_frame` - Body frame velocity commands
- `/airsim_node/VEHICLE-NAME/vel_cmd_world_frame` - World frame velocity commands
- `/airsim_node/all_robots/vel_cmd_body_frame` - Control all vehicles

### Key Services
- `/airsim_node/VEHICLE-NAME/takeoff` - Individual takeoff
- `/airsim_node/VEHICLE-NAME/land` - Individual landing
- `/airsim_node/takeoff_group` - Group takeoff operations
- `/airsim_node/coordinated_height_and_land` - Multi-vehicle coordination

## 🚀 Quick Start

### Build and Launch
```bash
# Build the ROS2 workspace
cd ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Launch AirSim ROS2 wrapper
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### Basic Testing
```bash
# Check node status
ros2 node list

# Monitor vehicle state
ros2 topic echo /airsim_node/drone_1/odom_local_ned

# Test basic flight
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'
```

## 📚 Additional ROS2 Documentation

### Comprehensive Guides (Located in `/ros2/` directory)
- **[ROS2 Usage Guide](../../ros2/ROS2_USAGE_GUIDE.md)** - Complete API usage examples
- **[Data Flow Explanation](../../ros2/AirSim_ROS2_DataFlow_Explanation.md)** - Technical implementation details
- **[Implementation Status](../../ros2/ROS2_IMPLEMENTATION_STATUS.md)** - Current development status
- **[Multi-Node Roadmap](../../ros2/MULTI_NODE_ROADMAP.md)** - Future architecture evolution

### Package Documentation
- **[Package README](../../ros2/src/airsim_ros_pkgs/README.md)** - Package-specific documentation
- **[Docker Guide](../../ros2/DOCKER_ROS2_QUICKSTART.md)** - Docker-based ROS2 development
- **[WSL2 Setup](../../ros2/WSL_ROS2_SETUP_GUIDE.md)** - Windows development with WSL2

## 🔄 Evolution: Multi-Node Architecture

### Current: Single-Node (Production Ready) ✅
- Centralized architecture with proven reliability
- Multi-vehicle support through single node
- High-performance multi-threaded design

### Future: Multi-Node Architecture (In Development) 🚧
- **Phase 1-2 Complete**: Vehicle node hierarchy and processing isolation
- **Phase 3-4 In Progress**: Dynamic launch system and comprehensive testing
- **Benefits**: True parallel processing, individual vehicle fault isolation

## 🔗 Integration Examples

### With PX4
```bash
# Launch with PX4 integration
ros2 launch airsim_ros_pkgs airsim_node.launch.py enable_api_control:=true

# Monitor PX4 vehicle state
ros2 topic echo /airsim_node/px4_drone/odom_local_ned
```

### With Mission Planning
```bash
# Launch with grid search mission planning
ros2 launch airsim_grid_search grid_search.launch.py

# Execute autonomous mission
ros2 action send_goal /airsim_grid_search/grid_search airsim_mission_interfaces/action/GridSearch '{...}'
```

## 🛠️ Development Tools

### Custom Messages & Services
- **airsim_interfaces**: Custom ROS2 message and service definitions
- **airsim_mission_interfaces**: Mission planning interfaces

### Launch Files
- `airsim_node.launch.py` - Primary launch file
- `airsim_with_simple_PD_position_controller.launch.py` - With position controller
- `rviz.launch.py` - RViz visualization

### Utilities
- Vehicle node factory for dynamic creation
- Settings parser for configuration management
- Position controller implementation

## 📊 Performance Characteristics

- **Scalability**: Linear scaling with vehicle count (tested 20+ vehicles)
- **Frequencies**: Vehicle state ~50Hz, Camera ~30Hz, LiDAR ~10Hz
- **Multi-threading**: Parallel data processing with mutex protection
- **RPC Clients**: Specialized clients prevent blocking between operations

For detailed technical documentation and advanced usage, refer to the comprehensive guides linked above.