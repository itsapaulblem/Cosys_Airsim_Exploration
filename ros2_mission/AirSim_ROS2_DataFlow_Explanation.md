# AirSim ROS2 Wrapper - Data Flow Explanation

## Overview

The AirSim ROS2 wrapper creates a bridge between AirSim (running in Unreal Engine) and ROS2, enabling real-time communication for drone simulation, control, and sensor data processing.

## Architecture

```
┌─────────────────────┐    RPC/TCP    ┌─────────────────────┐    ROS2     ┌─────────────────┐
│   Unreal Engine     │◄─────────────►│   ROS2 Wrapper      │◄───────────►│  User Apps      │
│   + AirSim Plugin   │   Port 41451  │   (Docker)          │   Topics    │  + Tools        │
└─────────────────────┘               └─────────────────────┘             └─────────────────┘
```

## Communication Protocol

- **Transport**: TCP/IP connection on port `41451`
- **Protocol**: RPC (Remote Procedure Call) using `rpclib` with MessagePack
- **Direction**: Bidirectional - data flows both ways
- **Connection**: Multiple specialized RPC clients for different data types

## Data Flow: AirSim → ROS2 (Sensor Data)

### 1. Timer-Based Polling System

The ROS2 wrapper uses multiple timers to efficiently request data:

| Timer | Frequency | Purpose |
|-------|-----------|---------|
| State Timer | ~50Hz | Vehicle state, GPS, IMU, basic sensors |
| Image Timer | ~30Hz | Camera feeds, depth, segmentation |
| LiDAR Timer | ~10Hz | Point cloud data |

### 2. Data Retrieval Process

```cpp
// State Timer Callback (every 20ms)
void drone_state_timer_cb() {
    // 1. Request current vehicle state from AirSim
    MultirotorState state = rpc_client->getMultirotorState(vehicle_name);
    
    // 2. Convert AirSim data to ROS2 messages
    nav_msgs::Odometry odom = convert_to_ros_odom(state);
    sensor_msgs::Imu imu = convert_to_ros_imu(state);
    
    // 3. Publish to ROS2 topics
    odom_publisher->publish(odom);
    imu_publisher->publish(imu);
}
```

### 3. Published ROS2 Topics

#### Vehicle State
- `/airsim_node/Drone1/odom_local_ned` - Position, velocity, orientation
- `/airsim_node/Drone1/global_gps` - GPS coordinates
- `/airsim_node/Drone1/environment` - Weather, temperature, pressure

#### Sensors
- `/airsim_node/Drone1/imu/imu` - Acceleration, gyroscope
- `/airsim_node/Drone1/magnetometer/mag` - Magnetic field
- `/airsim_node/Drone1/barometer/alt` - Altitude
- `/airsim_node/Drone1/distance/distance` - Distance sensor

#### Cameras
- `/airsim_node/Drone1/front_center/Scene` - RGB camera
- `/airsim_node/Drone1/front_center/DepthPerspective` - Depth camera
- `/airsim_node/Drone1/front_center/Segmentation` - Segmentation mask

#### LiDAR
- `/airsim_node/Drone1/lidar/points` - Point cloud data

## Data Flow: ROS2 → AirSim (Control Commands)

### 1. Command Reception

```cpp
// Velocity Command Callback
void vel_cmd_body_frame_cb(const VelCmd::SharedPtr msg) {
    // 1. Convert ROS2 message to AirSim format
    VelCmd airsim_cmd = convert_ros_to_airsim(*msg);
    
    // 2. Queue command for execution
    vel_cmd_queue.push_back(airsim_cmd);
}
```

### 2. Command Execution

```cpp
// Command Update Timer (every 50ms)
void update_commands() {
    // 1. Process queued commands
    for (auto& cmd : vel_cmd_queue) {
        // 2. Send to AirSim via RPC
        rpc_client->moveByVelocityBodyFrameAsync(
            cmd.x, cmd.y, cmd.z, duration
        );
    }
    vel_cmd_queue.clear();
}
```

### 3. Subscribed ROS2 Topics

#### Vehicle Control
- `/airsim_node/Drone1/vel_cmd_body_frame` - Velocity commands (drone frame)
- `/airsim_node/Drone1/vel_cmd_world_frame` - Velocity commands (world frame)

#### Multi-Drone Control
- `/airsim_node/all_robots/vel_cmd_body_frame` - Control all drones
- `/airsim_node/group_of_robots/vel_cmd_body_frame` - Control drone group

#### Camera Control
- `/airsim_node/gimbal_angle_quat_cmd` - Gimbal orientation
- `/airsim_node/gimbal_angle_euler_cmd` - Gimbal Euler angles

#### Car Control (if in car mode)
- `/airsim_node/Car1/car_cmd` - Throttle, steering, brake

## Service-Based Commands

### Flight Control Services
```bash
# Takeoff
ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff

# Land
ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land

# Reset simulation
ros2 service call /airsim_node/reset airsim_interfaces/srv/Reset
```

## Multi-Vehicle Support

Each vehicle gets its own namespace and data streams:

```
/airsim_node/
├── Drone1/
│   ├── odom_local_ned
│   ├── vel_cmd_body_frame
│   ├── front_center/Scene
│   └── takeoff (service)
├── Drone2/
│   ├── odom_local_ned
│   ├── vel_cmd_body_frame
│   └── ...
└── all_robots/
    ├── vel_cmd_body_frame
    └── takeoff (service)
```

## Key Technical Details

### 1. Multiple RPC Clients
```cpp
// Specialized clients for different data types
MultirotorRpcLibClient airsim_client_;        // Main control
MultirotorRpcLibClient airsim_client_images_; // Camera data
MultirotorRpcLibClient airsim_client_lidar_;  // LiDAR data
```

### 2. Thread Safety
All AirSim API calls are mutex-protected:
```cpp
std::lock_guard<std::mutex> guard(control_mutex_);
rpc_client->moveByVelocityAsync(x, y, z);
```

### 3. Automatic Initialization
When `enable_api_control=true`:
```cpp
// Auto-enable API control and arm drones
airsim_client_->enableApiControl(true, vehicle_name);
airsim_client_->armDisarm(true, vehicle_name);
```

## Error Handling

```cpp
try {
    // AirSim RPC calls
    auto state = rpc_client->getMultirotorState(vehicle_name);
} catch (rpc::rpc_error& e) {
    RCLCPP_ERROR(logger, "AirSim API error: %s", e.what());
}
```

## Summary

The data flow is **bidirectional** and **real-time**:

1. **AirSim → ROS2**: Continuous polling of sensor data, vehicle state, and camera feeds
2. **ROS2 → AirSim**: Command queuing and execution for vehicle control
3. **Communication**: RPC over TCP/IP with multiple specialized clients
4. **Threading**: Timer-based polling with mutex protection
5. **Multi-vehicle**: Full support with per-vehicle namespaces

This architecture enables seamless integration between AirSim's high-fidelity simulation and ROS2's robotics ecosystem. 