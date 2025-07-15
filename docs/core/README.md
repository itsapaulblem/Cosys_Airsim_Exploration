# Core Features & API Documentation

This section contains documentation for the core functionality of Cosys-AirSim, including APIs, sensors, vehicle types, and fundamental features.

## 🔌 API Documentation

### Core APIs
- **[APIs Overview](apis.md)** - General API concepts and usage patterns
- **[C++ APIs](apis_cpp.md)** - C++ client library reference
- **[Adding New APIs](adding_new_apis.md)** - Guide for extending AirSim APIs

### Image & Camera APIs
- **[Image APIs](image_apis.md)** - Camera control and image capture
- **[Camera Views](camera_views.md)** - Camera positioning and view configuration
- **[Infrared Camera](InfraredCamera.md)** - Thermal imaging capabilities

## 🚁 Vehicle Types & Control

### Drone Vehicles
- **[Custom Drone](custom_drone.md)** - Creating and configuring custom drone models
- **[Multi-Vehicle Setup](multi_vehicle.md)** - Coordinating multiple vehicles in simulation

### Ground Vehicles
- **[Using Cars](using_car.md)** - Car simulation and control
- **[Skid Steer Vehicles](skid_steer_vehicle.md)** - Specialized ground vehicles (Husky, Pioneer)

## 📡 Sensor Systems

### LiDAR & Point Cloud Sensors
- **[LiDAR Sensor](lidar.md)** - Standard LiDAR configuration and usage
- **[GPU LiDAR](gpulidar.md)** - High-performance GPU-accelerated LiDAR
- **[Echo Sensors](echo.md)** - Radar and sonar simulation

### Standard Sensors
- **[Distance Sensors](distance_sensor.md)** - Ultrasonic and infrared distance measurement
- **[Sensors Overview](sensors.md)** - Complete sensor system documentation

## 🎮 Control & Input

### Manual Control
- **[Remote Control](remote_control.md)** - Manual vehicle control methods
- **[Xbox Controller](xbox_controller.md)** - Gamepad integration
- **[Steering Wheel](steering_wheel_installation.md)** - Force feedback steering wheel setup

## ⚙️ Configuration & Settings

### Core Configuration
- **[Settings](settings.md)** - Complete settings.json reference
- **[Settings Example](settings_example.json)** - Example configuration file

### World & Environment
- **[Dynamic Objects](dynamic_objects.md)** - Runtime object spawning and manipulation

## 🎯 Computer Vision & AI

### Object Detection & Segmentation
- **[Object Detection](object_detection.md)** - Object detection with neural networks
- **[Instance Segmentation](../image_segmentation/)** - Instance-level segmentation

### Data Collection & Processing
- **[Playback](playback.md)** - Recording and playback functionality
- **[Modify Recording Data](modify_recording_data.md)** - Post-processing recorded data
- **[PFM Files](pfm.md)** - Portable Float Map format handling

## 🛠️ Tools & Utilities

### Debugging & Analysis
- **[Log Viewer](log_viewer.md)** - Flight data analysis tool
- **[Event Simulation](event_sim.md)** - Event-driven simulation scenarios

### External Integrations
- **[MATLAB](matlab.md)** - MATLAB toolbox for AirSim
- **[Gazebo Drone](gazebo_drone.md)** - Gazebo simulation integration

## 🏗️ Architecture Overview

### Core System Design
Cosys-AirSim is built around several key architectural principles:

#### Vehicle Abstraction
- **VehicleApiBase**: Common interface for all vehicle types
- **Specialized Implementations**: MultiRotor, Car, ComputerVision modes
- **Plugin Architecture**: Extensible vehicle type system

#### Sensor Framework
- **SensorBase**: Common sensor interface
- **Modular Design**: Independent sensor implementations
- **Factory Pattern**: Dynamic sensor creation from configuration

#### API Architecture
- **RPC Communication**: msgpack-rpc for client-server communication
- **Multi-threaded**: Concurrent request handling
- **Extensible**: Easy addition of new API endpoints

### Key Features

#### Multi-Vehicle Support
- **Simultaneous Vehicles**: Multiple vehicles in same simulation
- **Independent Control**: Separate API access per vehicle
- **Coordinated Operations**: Group commands and synchronized actions

#### Advanced Sensors
- **GPU LiDAR**: Hardware-accelerated point cloud generation
- **Echo Simulation**: Radar and sonar with realistic physics
- **Camera Systems**: RGB, depth, segmentation, annotation
- **External Sensors**: World-positioned sensors independent of vehicles

#### Cosys-Lab Enhancements
- **Instance Segmentation**: Per-object identification
- **Dynamic Objects**: Runtime environment modification
- **Artificial Lighting**: Realistic lighting simulation
- **Enhanced Materials**: Advanced rendering features

## 📊 Performance & Optimization

### System Performance
- **Multi-threading**: Parallel sensor processing
- **Efficient Rendering**: Optimized graphics pipeline
- **Memory Management**: Smart resource allocation
- **Scalable Architecture**: Linear performance scaling

### Configuration Optimization
- **Sensor Frequencies**: Optimal update rates
- **Quality Settings**: Performance vs. fidelity trade-offs
- **Resource Allocation**: CPU and GPU utilization
- **Network Optimization**: RPC communication tuning

## 🔗 Integration Patterns

### Common Use Cases

#### Research Applications
- Autonomous navigation algorithm development
- Sensor fusion and SLAM research
- Multi-agent coordination studies
- Computer vision dataset generation

#### Industrial Applications
- Training autonomous vehicle systems
- Warehouse automation testing
- Inspection robot development
- Search and rescue simulation

#### Educational Use
- Robotics course material
- Drone programming tutorials
- Computer vision demonstrations
- Physics simulation examples

## 📚 Quick Reference

### Essential Configuration
```json
{
  "SeeDocsAt": "https://cosys-lab.github.io/Cosys-AirSim/settings/",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0
    }
  }
}
```

### Common API Patterns
```python
import airsim

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Basic flight operations
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Sensor data access
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene),
    airsim.ImageRequest("0", airsim.ImageType.DepthVis)
])
```

### Troubleshooting Quick Checks
1. ✅ Settings.json syntax is valid
2. ✅ Vehicle spawn positions are clear
3. ✅ API control is enabled
4. ✅ Network ports are available (41451)
5. ✅ Required sensors are configured

## 🎯 Next Steps

After understanding the core concepts:

1. **[Choose Installation Method](../installation/)** - Set up your development environment
2. **[Configure Your Vehicle](settings.md)** - Create your first vehicle configuration
3. **[Test API Connectivity](apis.md)** - Verify API communication
4. **[Explore Integrations](../ros2/)** - Connect with robotics frameworks

For detailed implementation guides and advanced usage, refer to the individual documentation files in this section and the specialized guides in [PX4](../px4/), [ROS2](../ros2/), and [Unreal](../unreal/) sections.