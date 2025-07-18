# AirSim RPC Architecture: A Comprehensive Deep Dive

## Executive Summary

AirSim's RPC (Remote Procedure Call) architecture represents a sophisticated, multi-layered system that enables seamless communication between different programming languages, frameworks, and simulation components. This deep dive analyzes how AirSim's RPC system bridges Python clients, C++ simulation cores, ROS2 integrations, and Unreal Engine physics, creating a unified platform for autonomous vehicle research and development.

The system is built on established software engineering patterns and leverages high-performance libraries to achieve real-time communication capabilities essential for simulation and control applications.

## 1. Architecture Overview

### High-Level Design Philosophy

AirSim's RPC architecture follows a **client-server paradigm** with several key design principles:

- **Language Independence**: Python, C++, MATLAB, and ROS2 clients can all communicate with the same C++ simulation core
- **Performance Focus**: Binary serialization (msgpack) and efficient threading for real-time applications
- **Extensibility**: Plugin-based architecture allowing new vehicle types and sensors
- **Separation of Concerns**: Clear boundaries between transport, serialization, and business logic

### Component Relationships

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Python Client │    │   ROS2 Bridge   │    │   MATLAB Client │
│   (cosysairsim) │    │   (airsim_ros)  │    │   (AirSim.m)    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │     msgpack-rpc over TCP   │
                    │       (Port 41451)         │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │   C++ RPC Server (AirLib)  │
                    │   - RpcLibServerBase       │
                    │   - Vehicle-specific APIs  │
                    │   - Sensor Management      │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │   Unreal Engine Plugin     │
                    │   - Physics Integration    │
                    │   - Visual Rendering       │
                    │   - Sensor Simulation      │
                    └────────────────────────────┘
```

## 2. Core RPC Foundation

### Base Architecture Components

The foundation of AirSim's RPC system rests on three core classes:

#### RpcLibServerBase (`AirLib/include/api/RpcLibServerBase.hpp`)
- **Purpose**: Provides the base server infrastructure for all RPC operations
- **Key Responsibilities**:
  - Manages `rpc::server` instance from the rpclib library
  - Binds string identifiers to C++ methods (e.g., `"takeoff"` → `takeoffAsync()`)
  - Handles client connections and thread management
  - Provides access to world and vehicle APIs through `ApiProvider`

#### RpcLibClientBase (`AirLib/include/api/RpcLibClientBase.hpp`)
- **Purpose**: Base class for all client-side RPC communication
- **Key Responsibilities**:
  - Encapsulates `rpc::client` instance for network communication
  - Provides common methods (ping, connection management, version checking)
  - Handles error propagation and timeout management
  - Implements async operation tracking with `waitOnLastTask`

#### RpcLibAdaptorsBase (`AirLib/include/api/RpcLibAdaptorsBase.hpp`)
- **Purpose**: Handles serialization/deserialization between AirSim types and msgpack
- **Key Responsibilities**:
  - Defines adaptor structs for all data types (Vector3r, Pose, ImageResponse, etc.)
  - Uses `MSGPACK_DEFINE_MAP` macro for automatic serialization
  - Provides `to()` and `from()` methods for type conversion
  - Handles complex nested data structures

### Design Patterns Implementation

#### 1. Facade Pattern
The RPC base classes act as facades, providing simplified interfaces to complex underlying systems:

```cpp
// Simplified client interface
class RpcLibClientBase {
public:
    bool ping() { return client_.call("ping").as<bool>(); }
    void simPause(bool is_paused) { client_.call("simPause", is_paused); }
    // ... simplified methods hide rpclib complexity
};
```

#### 2. Adapter Pattern
The adaptor system converts between AirSim's internal types and msgpack-serializable types:

```cpp
struct Pose {
    Vector3r position;
    Quaternionr orientation;
    
    MSGPACK_DEFINE_MAP(position, orientation);
    
    Pose(const msr::airlib::Pose& p) : position(p.position), orientation(p.orientation) {}
    msr::airlib::Pose to() const { return msr::airlib::Pose(position.to(), orientation.to()); }
};
```

#### 3. PIMPL (Pointer to Implementation) Idiom
Both server and client classes use PIMPL to hide rpclib dependencies:

```cpp
class RpcLibServerBase {
private:
    struct impl;
    std::unique_ptr<impl> pimpl_;  // Hides rpclib headers
};
```

### Threading and Concurrency Model

#### Server-Side Threading
- **Async Mode**: `server.async_run(thread_count)` creates a thread pool
- **Blocking Mode**: `server.run()` runs on current thread
- **Thread Safety**: `suppress_exceptions(true)` prevents server crashes from individual request failures

#### Client-Side Threading  
- **Synchronous Calls**: `client.call()` blocks until response
- **Asynchronous Calls**: `client.call_async()` returns immediately with future
- **Background I/O**: msgpackrpc manages network I/O in background threads

## 3. Vehicle-Specific RPC Implementations

### Inheritance Hierarchy

Each vehicle type extends the base RPC architecture while maintaining the same fundamental patterns:

```
RpcLibServerBase
├── MultirotorRpcLibServer
├── CarRpcLibServer
└── ComputerVisionRpcLibServer

RpcLibClientBase
├── MultirotorRpcLibClient
├── CarRpcLibClient
└── ComputerVisionRpcLibClient
```

### Vehicle-Specific API Specializations

#### Multirotor API (`MultirotorRpcLibClient`)
- **Flight Control**: `takeoffAsync()`, `landAsync()`, `goHomeAsync()`
- **Movement Commands**: `moveToPositionAsync()`, `moveByVelocityAsync()`, `moveOnPathAsync()`
- **Low-Level Control**: `moveByRollPitchYawZAsync()`, `moveByMotorPWMsAsync()`
- **State Management**: `getMultirotorState()`, `getRotorStates()`

#### Car API (`CarRpcLibClient`)
- **Control**: `setCarControls()` (throttle, steering, brake, handbrake)
- **State**: `getCarState()` (speed, gear, rpm)
- **Simplified Interface**: Focus on ground vehicle dynamics

#### Computer Vision API (`ComputerVisionRpcLibClient`)
- **Minimal Interface**: Primarily uses base class methods
- **Camera-Centric**: Focused on sensor data collection without vehicle movement

### Adaptor Specializations

Each vehicle type defines its own adaptors for vehicle-specific data:

```cpp
// Multirotor-specific adaptors
struct MultirotorState {
    CollisionInfo collision;
    KinematicsState kinematics_estimated;
    GeoPoint gps_location;
    uint64_t timestamp;
    bool landed_state;
    
    MSGPACK_DEFINE_MAP(collision, kinematics_estimated, gps_location, timestamp, landed_state);
};

// Car-specific adaptors
struct CarControls {
    float throttle;
    float steering;
    float brake;
    bool handbrake;
    
    MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake);
};
```

## 4. Python Client Architecture

### Connection and Communication

The Python client uses the `msgpackrpc` library to communicate with the C++ server:

```python
class VehicleClient:
    def __init__(self, ip="", port=41451, timeout_value=3600):
        self.client = msgpackrpc.Client(
            msgpackrpc.Address(ip, port),
            timeout=timeout_value,
            pack_encoding='utf-8',
            unpack_encoding='utf-8'
        )
```

### Data Type Mapping

The Python client mirrors C++ types using the `MsgpackMixin` pattern:

```python
class MsgpackMixin:
    def to_msgpack(self):
        return self.__dict__
    
    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        obj.__dict__.update(encoded)
        return obj

class Vector3r(MsgpackMixin):
    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
```

### Async Operation Handling

Python clients use msgpackrpc's Future objects (not Python's async/await):

```python
def takeoffAsync(self, timeout_sec=20, vehicle_name=''):
    return self.client.call_async('takeoff', timeout_sec, vehicle_name)

# Usage
future = client.takeoffAsync()
# ... do other work ...
future.join()  # Wait for completion
```

### Performance Optimizations

- **Binary Serialization**: msgpack provides compact, fast serialization
- **Persistent Connections**: TCP connection reuse reduces overhead
- **Efficient Image Handling**: NumPy integration for image data processing
- **Compression Support**: Optional image compression for bandwidth optimization

## 5. C++ Server Implementation

### API Provider Architecture

The `ApiProvider` class manages vehicle discovery and API dispatch:

```cpp
class ApiProvider {
public:
    void insert_or_assign(const std::string& vehicle_name, 
                         VehicleApiBase* api, 
                         VehicleSimApiBase* sim_api);
    
    VehicleApiBase* getVehicleApi(const std::string& vehicle_name);
    VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name);
    WorldSimApiBase* getWorldSimApi();
    
private:
    UniqueValueMap<std::string, VehicleApiBase*> vehicle_apis_;
    UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    WorldSimApiBase* world_sim_api_;
};
```

### Multi-Vehicle Support

The server supports multiple vehicles through:
- **Vehicle Name Routing**: All RPC calls include `vehicle_name` parameter
- **Dynamic API Lookup**: `ApiProvider` routes calls to appropriate vehicle APIs
- **Concurrent Operations**: Thread pool handles multiple vehicle commands simultaneously

### Sensor Data Pipeline

The server provides unified sensor access across all vehicle types:

```cpp
// Examples of sensor data methods
ImageResponse simGetImage(const std::string& camera_name, 
                         ImageCaptureBase::ImageType image_type,
                         const std::string& vehicle_name);

LidarData getLidarData(const std::string& lidar_name,
                      const std::string& vehicle_name);
```

### Error Handling and Fault Tolerance

- **Exception Suppression**: `server.suppress_exceptions(true)` prevents crashes
- **ApiNotSupported**: Custom exception for unsupported operations
- **Graceful Degradation**: Missing APIs return appropriate error codes
- **Connection Management**: Automatic cleanup of disconnected clients

## 6. ROS2 Bridge Architecture

### Bridge Design Philosophy

The ROS2 bridge sits **on top** of the RPC layer, not alongside it:

```
ROS2 Clients → ROS2 Bridge → AirSim RPC Server → AirLib → Unreal Engine
```

### Dual Architecture Approach

#### Core ROS2 Implementation (`ros2/`)
- **Single-Node Architecture**: One monolithic node manages all vehicles
- **Specialized RPC Clients**: 5 separate RPC clients for performance optimization
  - General RPC Client (basic commands)
  - Image RPC Client (camera data)
  - Lidar RPC Client (traditional lidar)
  - GPU Lidar RPC Client (high-performance lidar)
  - Echo RPC Client (radar/sonar)

#### Mission Extension (`ros2_mission/`)
- **Multi-Node Architecture**: Vehicle-specific node classes
- **Advanced Capabilities**: Grid search, waypoint navigation, mission planning
- **Factory Pattern**: `VehicleNodeFactory` for creating appropriate node types

### Message Type Integration

The bridge defines comprehensive ROS2 message types:

```
airsim_interfaces/msg/
├── CarControls.msg
├── CarState.msg
├── MultirotorState.msg
├── VelCmd.msg
├── Environment.msg
└── InstanceSegmentationList.msg

airsim_interfaces/srv/
├── Takeoff.srv
├── Land.srv
├── SetGPSPosition.srv
└── RefreshInstanceSegmentation.srv
```

### Performance Optimization

- **Multiple RPC Connections**: Dedicated connections prevent sensor data bottlenecks
- **Timer-Based Updates**: Continuous sensor data streaming
- **Efficient Data Conversion**: Direct mapping between ROS2 and AirSim types

## 7. Complete Message Flow Analysis

### Example: Multirotor Takeoff Command

Let's trace a complete `takeoffAsync()` command from Python to Unreal Engine:

#### Step 1: Python Client Initiation
```python
client = airsim.MultirotorClient()
future = client.takeoffAsync(timeout_sec=10)
```

**What happens:**
- `MultirotorClient.takeoffAsync()` calls `self.client.call_async('takeoff', 10, '')`
- `msgpackrpc` serializes: `["takeoff", 10.0, ""]` → msgpack binary
- Background thread sends over TCP to port 41451
- Returns `Future` object immediately

#### Step 2: Network Transport
- **Protocol**: TCP socket connection
- **Serialization**: Binary msgpack format
- **Reliability**: TCP ensures ordered, error-free delivery

#### Step 3: C++ Server Reception
```cpp
server.bind("takeoff", [&](float timeout_sec, const std::string& vehicle_name) {
    return getVehicleApi(vehicle_name)->takeoffAsync(timeout_sec);
});
```

**What happens:**
- `rpclib` server receives binary data on worker thread
- Deserializes msgpack → `("takeoff", 10.0, "")`
- Looks up bound method and calls lambda
- Lambda calls `getVehicleApi("")->takeoffAsync(10.0)`

#### Step 4: AirLib Processing
```cpp
std::future<bool> MultirotorApiBase::takeoffAsync(float timeout_sec) {
    // Set takeoff goal
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();
    
    // Queue takeoff command for physics thread
    setGoal(TakeoffGoal{getCurrentPosition() + Vector3r(0,0,-3), timeout_sec});
    
    // Store promise for completion
    storePendingPromise(promise);
    
    return future;
}
```

**What happens:**
- Creates `std::promise<bool>` and `std::future<bool>` pair
- Sets takeoff goal in thread-safe manner
- Stores promise for later completion
- Returns future to RPC layer

#### Step 5: Unreal Engine Integration
**Every frame in Unreal's Tick:**
```cpp
void AirSimTickable::Tick(float DeltaTime) {
    // Update flight controller
    flight_controller_->update(DeltaTime);
    
    // Check goal completion
    if (takeoff_goal_active_ && vehicle_altitude_ >= target_altitude_) {
        completeTakeoffPromise(true);
    }
}
```

**What happens:**
- Flight controller reads takeoff goal
- Calculates rotor forces to achieve target altitude
- Applies forces to Unreal physics body: `UPrimitiveComponent::AddForce()`
- PhysX integrates forces → vehicle moves up
- Monitor checks completion condition

#### Step 6: Response Flow
```cpp
void completeTakeoffPromise(bool success) {
    if (takeoff_promise_) {
        takeoff_promise_->set_value(success);  // Fulfill C++ future
        takeoff_promise_.reset();
    }
}
```

**What happens:**
- Physics thread fulfills `std::promise<bool>` with `true`
- RPC worker thread unblocks from `future.get()`
- `rpclib` serializes `true` → msgpack binary
- Sends response over TCP with original message ID
- Python `msgpackrpc` receives response
- Python `Future` is fulfilled
- `future.join()` or `future.get()` returns `True`

### Thread Transitions Summary

1. **Python Main Thread** → **Python I/O Thread** (msgpackrpc)
2. **Network** → **C++ RPC Worker Thread** (rpclib)
3. **C++ Worker Thread** → **C++ Physics Thread** (goal setting)
4. **C++ Physics Thread** → **Unreal Game Thread** (force application)
5. **Unreal Game Thread** → **C++ Physics Thread** (completion monitoring)
6. **C++ Physics Thread** → **C++ RPC Worker Thread** (promise fulfillment)
7. **C++ Worker Thread** → **Python I/O Thread** (network response)
8. **Python I/O Thread** → **Python Main Thread** (future completion)

### Error Handling Throughout the Flow

#### Network Errors
- **Connection Loss**: Python msgpackrpc raises `ConnectionError`
- **Timeout**: Future throws timeout exception
- **Serialization Error**: msgpack exceptions in both Python and C++

#### API Errors
- **Invalid Vehicle**: `ApiNotSupported` exception thrown in C++
- **Invalid State**: Flight controller rejects takeoff (already flying)
- **Physics Error**: Unreal physics constraint violations

#### Timeout Handling
- **Python Timeout**: msgpackrpc timeout in client
- **C++ Timeout**: Flight controller timeout monitoring
- **Graceful Degradation**: Partial completion returns appropriate status

## 8. Integration Patterns

### Cross-Language Communication Strategy

AirSim's RPC system enables seamless integration across multiple programming environments:

#### Language-Agnostic Interface
```
Protocol Buffer Definition (Conceptual):
service AirSimAPI {
    rpc takeoff(TakeoffRequest) returns (TakeoffResponse);
    rpc getImage(ImageRequest) returns (ImageResponse);
    rpc setVehiclePose(PoseRequest) returns (PoseResponse);
}
```

#### Actual Implementation
```cpp
// C++ Server Bindings
server.bind("takeoff", [](float timeout, string vehicle_name) { ... });
server.bind("getImage", [](string camera_name, int image_type, string vehicle_name) { ... });
```

```python
# Python Client Calls
client.call_async('takeoff', timeout_sec, vehicle_name)
client.call('simGetImage', camera_name, image_type, vehicle_name)
```

### Performance Optimization Strategies

#### 1. Connection Pooling
- **ROS2 Bridge**: Uses 5 specialized RPC connections
- **Python Client**: Persistent TCP connections
- **Benefit**: Reduces connection overhead and prevents bottlenecks

#### 2. Efficient Serialization
- **msgpack**: Binary format, ~50% smaller than JSON
- **Direct Memory Access**: Minimizes copy operations
- **Streaming**: Large data (images) can be streamed

#### 3. Asynchronous Operations
- **Non-blocking**: Long operations don't block other requests
- **Parallel Processing**: Multiple commands execute concurrently
- **Future-based**: Clean async API without callback complexity

#### 4. Sensor Data Optimization
```cpp
// Image compression and format selection
ImageRequest request;
request.image_type = ImageCaptureBase::ImageType::Scene;
request.compress = true;  // Enable compression
request.pixels_as_float = false;  // Use bytes instead of floats
```

### Multi-Vehicle Architecture

#### Vehicle Management
```cpp
// ApiProvider manages multiple vehicles
api_provider_->insert_or_assign("Drone1", drone1_api, drone1_sim_api);
api_provider_->insert_or_assign("Drone2", drone2_api, drone2_sim_api);
api_provider_->insert_or_assign("Car1", car1_api, car1_sim_api);
```

#### Client-Side Multi-Vehicle
```python
# Create separate clients for each vehicle
drone1 = airsim.MultirotorClient()
drone2 = airsim.MultirotorClient() 
car1 = airsim.CarClient()

# Or use vehicle names with single client
client = airsim.MultirotorClient()
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2")
```

### ROS2 Integration Patterns

#### Message Bridging
```cpp
// Convert AirSim data to ROS2 message
void publishCarState(const CarState& airsim_state) {
    airsim_interfaces::msg::CarState ros_msg;
    ros_msg.speed = airsim_state.speed;
    ros_msg.gear = airsim_state.gear;
    ros_msg.rpm = airsim_state.rpm;
    
    car_state_pub_->publish(ros_msg);
}
```

#### Service Bridging
```cpp
// ROS2 service calls AirSim RPC
void takeoffCallback(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
                    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response) {
    
    bool success = airsim_client_->takeoffAsync(request->timeout_sec, request->vehicle_name).get();
    response->success = success;
}
```

## 9. Key Insights and Recommendations

### Architecture Strengths

#### 1. **Modular Design**
- **Separation of Concerns**: Transport, serialization, and business logic are cleanly separated
- **Extensibility**: New vehicle types can be added without modifying core RPC infrastructure
- **Language Independence**: New client languages can be added easily

#### 2. **Performance Focus**
- **Binary Serialization**: msgpack provides efficient data transfer
- **Async Operations**: Non-blocking commands enable responsive control
- **Thread Pool**: Concurrent request handling prevents bottlenecks

#### 3. **Robustness**
- **Exception Handling**: Comprehensive error handling prevents system crashes
- **Connection Management**: Automatic cleanup and recovery
- **Timeout Handling**: Prevents hanging operations

#### 4. **Standards Compliance**
- **msgpack-rpc**: Industry-standard RPC protocol
- **TCP**: Reliable transport layer
- **Thread Safety**: Proper synchronization patterns

### Identified Limitations

#### 1. **Single Point of Failure**
- **Central Server**: All communication goes through one RPC server
- **Recommendation**: Consider distributed architecture for very large simulations

#### 2. **Serialization Overhead**
- **Double Conversion**: AirSim types → Adaptors → msgpack → Network
- **Recommendation**: Direct msgpack serialization for performance-critical paths

#### 3. **Thread Context Switches**
- **Multiple Transitions**: Commands pass through many thread boundaries
- **Recommendation**: Minimize thread hops for latency-sensitive operations

#### 4. **Memory Allocation**
- **Frequent Allocation**: Adaptor objects created for each RPC call
- **Recommendation**: Object pooling for high-frequency operations

### Performance Optimization Opportunities

#### 1. **Zero-Copy Operations**
```cpp
// Current: Copy data through adaptors
ImageResponse response = ImageResponse(airsim_image);
return response;  // Another copy

// Proposed: Direct serialization
template<typename T>
auto serialize_direct(const T& obj) -> msgpack::object {
    // Direct msgpack serialization without intermediate objects
}
```

#### 2. **Batched Operations**
```cpp
// Current: Individual calls
client.call("getImage", camera1, ...);
client.call("getImage", camera2, ...);
client.call("getImage", camera3, ...);

// Proposed: Batch API
client.call("getImageBatch", {camera1, camera2, camera3}, ...);
```

#### 3. **Streaming APIs**
```cpp
// Current: Request-response for continuous data
while (true) {
    auto image = client.call("getImage", ...);
    process(image);
}

// Proposed: Server-side streaming
auto stream = client.call("getImageStream", ...);
for (auto image : stream) {
    process(image);
}
```

### Extensibility Considerations

#### 1. **Plugin Architecture**
- **Current**: Vehicle types are compiled into AirLib
- **Proposed**: Dynamic loading of vehicle plugins
- **Benefit**: Third-party vehicle types without recompilation

#### 2. **Protocol Versioning**
- **Current**: Version checking at connection time
- **Proposed**: Protocol version negotiation
- **Benefit**: Backward compatibility and gradual upgrades

#### 3. **Custom Serialization**
- **Current**: Fixed msgpack serialization
- **Proposed**: Pluggable serialization backends
- **Benefit**: Specialized serialization for specific use cases

### Future Evolution Recommendations

#### 1. **Microservices Architecture**
- **Current**: Monolithic RPC server
- **Proposed**: Separate services for different functionalities
  - Vehicle Control Service
  - Sensor Data Service  
  - Environment Service
  - Mission Planning Service

#### 2. **Event-Driven Architecture**
- **Current**: Request-response model
- **Proposed**: Event publishing/subscription
- **Benefit**: Reactive programming and loose coupling

#### 3. **WebSocket Support**
- **Current**: TCP-only transport
- **Proposed**: WebSocket transport layer
- **Benefit**: Web-based clients and real-time dashboards

#### 4. **GraphQL API**
- **Current**: Fixed RPC methods
- **Proposed**: GraphQL query layer
- **Benefit**: Flexible data fetching and reduced over-fetching

## Conclusion

AirSim's RPC architecture represents a well-engineered system that successfully bridges the gap between different programming languages, frameworks, and simulation components. The use of established patterns (Facade, Adapter, PIMPL) and high-performance libraries (msgpack, rpclib) creates a robust foundation for autonomous vehicle simulation and control.

The architecture's strength lies in its **modularity**, **performance focus**, and **extensibility**. The clear separation between transport, serialization, and business logic allows for easy maintenance and extension, while the async-first design ensures responsive operation even under heavy load.

The ROS2 integration demonstrates how higher-level frameworks can be built on top of the RPC foundation, providing domain-specific interfaces while leveraging the underlying performance and reliability of the core system.

While there are opportunities for optimization (zero-copy operations, batched calls, streaming APIs), the current architecture successfully serves AirSim's mission of providing a powerful, flexible platform for autonomous vehicle research and development.

The deep integration with Unreal Engine through careful thread management and the comprehensive support for multiple vehicle types and sensors make AirSim's RPC system a compelling example of how to build scalable, high-performance simulation infrastructure.