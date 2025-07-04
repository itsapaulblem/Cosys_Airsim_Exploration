# AirSim Communication Architecture - Complete Guide

## Overview

AirSim uses a **multi-layered communication architecture** to enable real-time interaction between the simulation environment (Unreal Engine) and external clients (ROS2, Python, MATLAB, etc.). This document explains the complete data flow across network, transport, and data layers.

## Architecture Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────┐  │
│  │   ROS2      │  │   Python    │  │   MATLAB    │  │   C++   │  │
│  │  Wrapper    │  │   Client    │  │   Client    │  │  Client │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                          DATA LAYER                             │
│              Message Conversion & Data Structures               │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  ROS2 Messages ↔ AirSim Structs ↔ Python Dicts/Objects   │ │
│  │  sensor_msgs     MultirotorState    airsim.DroneClient    │ │
│  │  nav_msgs        ImageResponse      pose, velocity, etc.   │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                       TRANSPORT LAYER                           │
│                    RPC (Remote Procedure Call)                  │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              rpclib + MessagePack Serialization            │ │
│  │  • Function calls over network                             │ │
│  │  • Binary serialization (MessagePack)                     │ │
│  │  • Async/Sync call support                                │ │
│  │  • Error handling & timeouts                              │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                        NETWORK LAYER                            │
│                         TCP/IP Sockets                          │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              Host: localhost, Port: 41451                  │ │
│  │  • Reliable connection-based communication                 │ │
│  │  • Multiple concurrent client connections                  │ │
│  │  • Cross-platform compatibility                           │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                      AIRSIM CORE ENGINE                         │
│                   (Unreal Engine Plugin)                        │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  • Physics simulation                                      │ │
│  │  • Sensor simulation                                       │ │
│  │  • Vehicle dynamics                                        │ │
│  │  • Environment rendering                                   │ │
│  │  • RPC server endpoints                                    │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Network Layer

### **TCP/IP Foundation**

All AirSim communication runs over **TCP/IP sockets**:

- **Protocol**: TCP (Transmission Control Protocol)
- **Default Port**: `41451`
- **Host**: `localhost` (127.0.0.1) for local simulation
- **Connection Type**: Persistent, bidirectional

```cpp
// Network connection establishment (RpcLibClientBase.cpp)
struct RpcLibClientBase::impl {
    impl(const string& ip_address, uint16_t port, float timeout_sec)
        : client(ip_address, port)  // TCP socket connection
    {
        client.set_timeout(static_cast<int64_t>(timeout_sec * 1.0E3));
    }
    rpc::client client;
};
```

### **Multiple Client Support**

AirSim server can handle **multiple simultaneous connections**:

```
AirSim Server (Port 41451)
├─ Connection 1: ROS2 Wrapper
├─ Connection 2: Python Client  
├─ Connection 3: MATLAB Client
└─ Connection N: Custom C++ App
```

### **Network Configuration**

```json
// Docker/Remote setup
{
  "host_ip": "192.168.1.100",
  "host_port": 41451,
  "timeout_sec": 60
}
```

## Transport Layer - RPC (Remote Procedure Call)

### **What is RPC?**

RPC makes **remote function calls look like local function calls**:

```cpp
// This looks like a local function call:
MultirotorState state = client->getMultirotorState("Drone1");

// But actually sends this over the network:
// TCP Packet: {method: "getMultirotorState", params: ["Drone1"]}
// Response:   {result: {timestamp: 12345, position: {...}, ...}}
```

### **RPC Library: rpclib**

AirSim uses the **rpclib** library with **MessagePack** serialization:

```cpp
// Making an RPC call (MultirotorRpcLibClient.cpp)
MultirotorState MultirotorRpcLibClient::getMultirotorState(const std::string& vehicle_name) {
    return static_cast<rpc::client*>(getClient())->
        call("getMultirotorState", vehicle_name)
        .as<MultirotorRpcLibAdaptors::MultirotorState>().to();
}
```

**What happens internally:**
1. **Serialize**: Convert C++ parameters to MessagePack binary
2. **Send**: TCP packet with method name + serialized parameters  
3. **Execute**: AirSim server calls the actual function
4. **Return**: Serialize response and send back over TCP
5. **Deserialize**: Convert binary response back to C++ objects

### **MessagePack Serialization**

MessagePack is a **binary serialization format** (like JSON but faster/smaller):

```
// JSON (text): {"x": 1.5, "y": 2.3, "z": 0.8}  -> 32 bytes
// MessagePack:  [binary representation]          -> ~15 bytes
```

### **Synchronous vs Asynchronous RPC**

```cpp
// Synchronous: Blocks until response received
MultirotorState state = client->getMultirotorState("Drone1");

// Asynchronous: Returns immediately, execution continues
MultirotorRpcLibClient* future = client->moveByVelocityAsync(1.0, 0.0, 0.0, 2.0);
// ... do other work ...
future->waitOnLastTask(); // Wait for completion when needed
```

### **RPC Server Endpoints in AirSim**

```cpp
// Available RPC methods (implemented in AirSim)
"getMultirotorState"     -> Get drone position, velocity, orientation
"moveByVelocityAsync"    -> Send velocity commands
"takeoff"                -> Takeoff command
"land"                   -> Landing command  
"getImages"              -> Get camera images
"getLidarData"           -> Get LiDAR point clouds
"getImuData"             -> Get IMU sensor data
"simSetWeather"          -> Control weather
"reset"                  -> Reset simulation
// ... 100+ available methods
```

## Data Layer

### **Data Structure Mapping**

Each wrapper translates between its native format and AirSim's internal C++ structures:

#### **AirSim Internal (C++)**
```cpp
namespace msr::airlib {
    struct MultirotorState {
        uint64_t timestamp;
        
        struct Kinematics {
            Vector3r position;        // x, y, z in meters
            Quaternionr orientation;  // rotation quaternion
            Vector3r linear_velocity; // m/s
            Vector3r angular_velocity; // rad/s
        } kinematics;
        
        GeoPoint gps_location;       // lat, lon, alt
        uint collision_count;
        // ... more fields
    };
}
```

#### **ROS2 Translation (sensor_msgs)**
```cpp
sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) {
    sensor_msgs::msg::Imu imu_msg;
    
    // Direct field mapping
    imu_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();
    
    // Coordinate system conversion (NED → ENU)
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = -imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z();
    
    return imu_msg;
}
```

#### **Python Translation (airsim module)**
```python
# Python client receives same data, different representation
import airsim

client = airsim.MultirotorClient()
state = client.getMultirotorState()

# Access as Python objects/dictionaries
position = state.kinematics_estimated.position
print(f"Drone position: x={position.x_val}, y={position.y_val}, z={position.z_val}")

# Coordinate system is handled automatically
velocity = state.kinematics_estimated.linear_velocity
```

### **Coordinate System Transformations**

Different systems use different coordinate conventions:

| System | Coordinate Frame | Forward | Right | Up |
|--------|------------------|---------|-------|-----|
| **AirSim** | NED (North-East-Down) | +X | +Y | -Z |
| **ROS2** | ENU (East-North-Up) | +X | -Y | +Z |
| **Unreal** | Left-Handed | +X | +Y | +Z |

```cpp
// ROS2 wrapper handles coordinate conversion automatically
void convert_to_ros_coordinates(geometry_msgs::msg::Transform& tf_msg) {
    // Swap and negate axes to convert NED → ENU
    tf_msg.translation.z = -tf_msg.translation.z;  // Down → Up
    tf_msg.translation.y = -tf_msg.translation.y;  // East → North
    tf_msg.rotation.z = -tf_msg.rotation.z;        // Rotation adjustment
    tf_msg.rotation.y = -tf_msg.rotation.y;
}
```

## Complete Data Flow Examples

### **Example 1: Sensor Data Flow (AirSim → Client)**

#### **Step-by-Step Process:**

```cpp
// 1. Timer triggers in ROS2 wrapper (50Hz)
void drone_state_timer_cb() {
    
    // 2. Make RPC call over TCP
    auto rpc = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
    
    // 3. This becomes: TCP packet with method="getMultirotorState"
    MultirotorState airsim_state = rpc->getMultirotorState("Drone1");
    
    // 4. Convert AirSim struct → ROS2 message
    nav_msgs::msg::Odometry ros_odom = get_odom_msg_from_multirotor_state(airsim_state);
    
    // 5. Publish to ROS2 network
    odom_publisher_->publish(ros_odom);
}
```

#### **Network Traffic:**
```
Client → AirSim: [TCP] {"method": "getMultirotorState", "params": ["Drone1"]}
AirSim → Client: [TCP] {"result": {"timestamp": 12345, "position": {"x": 1.5, "y": 2.3, "z": -10.0}, ...}}
```

### **Example 2: Control Command Flow (Client → AirSim)**

#### **ROS2 Command Flow:**
```cpp
// 1. ROS2 subscriber receives message
void vel_cmd_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg) {
    
    // 2. Convert ROS2 message → AirSim command structure
    VelCmd airsim_cmd = get_airlib_body_vel_cmd(*msg, current_orientation);
    
    // 3. Store in command queue (thread-safe)
    drone->vel_cmd_ = airsim_cmd;
    drone->has_vel_cmd_ = true;
}

// 4. Control timer sends queued commands (20Hz)
void update_commands() {
    if (drone->has_vel_cmd_) {
        // 5. Make async RPC call
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())
            ->moveByVelocityAsync(drone->vel_cmd_.x, drone->vel_cmd_.y, 
                                drone->vel_cmd_.z, vel_cmd_duration_);
        drone->has_vel_cmd_ = false;
    }
}
```

#### **Python Command Flow:**
```python
import airsim

# 1. Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# 2. Direct RPC call (simpler than ROS2)
client.moveByVelocityAsync(1.0, 0.5, -0.2, 2.0)  # vx, vy, vz, duration
```

## Wrapper-Specific Implementations

### **1. ROS2 Wrapper**

**Characteristics:**
- **Multi-threaded**: Separate threads for different sensor types
- **Multiple RPC clients**: Prevents blocking between different data streams
- **Real-time**: Timer-based polling at different frequencies
- **Full integration**: Complete ROS2 ecosystem compatibility

```cpp
// Multiple specialized clients for efficiency
MultirotorRpcLibClient airsim_client_;         // Control & state
MultirotorRpcLibClient airsim_client_images_;  // Camera data
MultirotorRpcLibClient airsim_client_lidar_;   // LiDAR point clouds
MultirotorRpcLibClient airsim_client_gpulidar_;// GPU LiDAR
```

**Data Flow Frequencies:**
| Data Type | Frequency | RPC Client | Purpose |
|-----------|-----------|------------|---------|
| Vehicle State | 50Hz | `airsim_client_` | Position, velocity, orientation |
| Camera Images | 30Hz | `airsim_client_images_` | RGB, depth, segmentation |
| LiDAR | 10Hz | `airsim_client_lidar_` | Point clouds |
| Commands | 20Hz | `airsim_client_` | Velocity, takeoff, land |

### **2. Python Client**

**Characteristics:**
- **Single-threaded**: Simpler programming model
- **Synchronous**: Each call blocks until response
- **Direct mapping**: Python objects mirror C++ structures
- **Rapid prototyping**: Easy for scripting and experimentation

```python
import airsim
import time

# Single client handles all communication
client = airsim.MultirotorClient()
client.confirmConnection()

# Simple control loop
while True:
    # Get current state (blocks until response)
    state = client.getMultirotorState()
    
    # Process state and send commands
    if state.landed_state == airsim.LandedState.Landed:
        client.takeoffAsync().join()
    else:
        client.moveByVelocityAsync(1.0, 0.0, 0.0, 0.1)
    
    time.sleep(0.1)  # 10Hz update rate
```

### **3. MATLAB Client**

**Characteristics:**
- **Object-oriented**: MATLAB class wrapper around RPC
- **Matrix-friendly**: Data returned as MATLAB matrices
- **Visualization**: Easy integration with MATLAB plotting/analysis

```matlab
% MATLAB client example
client = AirSimClient('127.0.0.1', 41451);

% Get image as MATLAB matrix
image_request = ImageRequest('front_center', ImageType.Scene);
images = client.simGetImages({image_request});
img_matrix = reshape(images{1}.image_data, [images{1}.height, images{1}.width, 3]);

% Display using MATLAB functions
imshow(img_matrix);
```

## Performance Considerations

### **Network Optimization**

```cpp
// Specialized clients prevent blocking
void img_response_timer_cb() {
    // Large image data doesn't block control commands
    auto images = airsim_client_images_.simGetImages(image_requests);
    // Process images...
}

void drone_state_timer_cb() {
    // Control data processed separately at higher frequency
    auto state = airsim_client_.getMultirotorState("Drone1");
    // Send control commands...
}
```

### **Data Transfer Sizes**

| Data Type | Typical Size | Transfer Time (1 Gbps) |
|-----------|--------------|------------------------|
| Vehicle State | ~1 KB | <0.01ms |
| IMU Data | ~100 bytes | <0.001ms |
| Single Image (640x480 RGB) | ~900 KB | ~7ms |
| LiDAR Point Cloud (10k points) | ~400 KB | ~3ms |
| Depth Image (640x480) | ~1.2 MB | ~10ms |

### **Threading Model**

```cpp
// ROS2 wrapper uses multiple timers for different update rates
std::shared_ptr<rclcpp::TimerBase> airsim_control_update_timer_;  // 50Hz
std::shared_ptr<rclcpp::TimerBase> img_timer_;                    // 30Hz  
std::shared_ptr<rclcpp::TimerBase> lidar_timer_;                  // 10Hz

// All RPC calls are mutex-protected for thread safety
std::mutex control_mutex_;
```

## Error Handling & Reliability

### **Connection Management**

```cpp
void confirmConnection() {
    std::cout << "Waiting for connection - " << std::flush;
    while (getConnectionState() != RpcLibClientBase::ConnectionState::Connected) {
        std::cout << "X" << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << std::endl << "Connected!" << std::endl;
}
```

### **RPC Error Handling**

```cpp
try {
    MultirotorState state = rpc->getMultirotorState(vehicle_name);
    // Process state...
} catch (rpc::rpc_error& e) {
    RCLCPP_ERROR(logger_, "RPC call failed: %s", e.what());
    // Continue operation, retry on next timer cycle
}
```

### **Timeout Configuration**

```cpp
// Different timeouts for different operations
RpcLibClientBase::RpcLibClientBase(const string& ip_address, uint16_t port, float timeout_sec) {
    // Long timeout for image requests
    client.set_timeout(static_cast<int64_t>(timeout_sec * 1000));
}
```

## Summary

The AirSim communication architecture provides a **robust, multi-layered system** that enables real-time interaction between simulation and external applications:

1. **Network Layer**: TCP/IP sockets provide reliable, cross-platform communication
2. **Transport Layer**: RPC with MessagePack serialization enables efficient remote function calls
3. **Data Layer**: Automatic conversion between different data formats and coordinate systems
4. **Application Layer**: Multiple wrapper types (ROS2, Python, MATLAB) for different use cases

This architecture supports:
- **Real-time control** with sub-millisecond latencies
- **High-bandwidth data** transfer (images, point clouds)
- **Multiple simultaneous clients** 
- **Cross-platform compatibility**
- **Robust error handling** and recovery

The modular design allows developers to choose the appropriate wrapper for their needs while maintaining consistent access to AirSim's full simulation capabilities. 