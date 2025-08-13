# MavLinkCom Architecture Documentation

## Overview

The MavLinkCom library is a comprehensive C++ implementation of the MAVLink protocol designed for robust communication between autopilots, ground control stations, and simulation environments like AirSim. This document provides a detailed architectural analysis of the library's design and implementation.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Core Components](#core-components)
3. [Connection Management](#connection-management)
4. [Message Handling](#message-handling)
5. [Vehicle State Management](#vehicle-state-management)
6. [AirSim Integration](#airsim-integration)
7. [Advanced Features](#advanced-features)
8. [Threading and Performance](#threading-and-performance)
9. [Usage Examples](#usage-examples)

## Architecture Overview

### Layered Architecture

The MavLinkCom library follows a **layered architecture** with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Application Layer                            │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   MavLinkNode   │  │ MavLinkVehicle  │  │    User Code    │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                    Protocol Layer                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │ Message Parsing │  │ Message Routing │  │ Command Handling│ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
├─────────────────────────────────────────────────────────────────┤
│                    Transport Layer                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   UDP Socket    │  │   TCP Socket    │  │   Serial Port   │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### Key Design Patterns

1. **PIMPL (Private Implementation)**: All major classes use the PIMPL idiom for binary compatibility
2. **Observer Pattern**: Message subscription system for loose coupling
3. **Factory Pattern**: Static factory methods for different connection types
4. **Bridge Pattern**: Connection joining for proxy and gateway scenarios
5. **Command Pattern**: Strongly-typed command objects with async execution

## Core Components

### Class Hierarchy

```cpp
// Core Communication Stack
MavLinkConnection
├── MavLinkConnectionImpl (PIMPL implementation)
├── Port (Transport abstraction)
│   ├── UdpClientPort
│   ├── TcpClientPort
│   └── SerialPort
└── MavLinkLog (Message logging)

// Node Abstraction
MavLinkNode
├── MavLinkNodeImpl (PIMPL implementation)
├── MavLinkConnection (Composition)
└── MavLinkVehicle (Specialization)
    ├── VehicleState (State management)
    └── AsyncResult<T> (Async operations)

// Message System
MavLinkMessageBase
├── Generated Message Classes
└── MavLinkCommand (Command abstraction)
```

### Key Classes

#### MavLinkConnection
**Purpose**: Core communication infrastructure
**Responsibilities**:
- Transport layer abstraction
- Message parsing and serialization
- Connection lifecycle management
- Message routing and distribution

```cpp
class MavLinkConnection {
public:
    // Factory methods for different connection types
    static std::shared_ptr<MavLinkConnection> connectSerial(
        const std::string& portName, int baudRate = 57600);
    static std::shared_ptr<MavLinkConnection> connectLocalUdp(
        const std::string& localAddr, int localPort);
    static std::shared_ptr<MavLinkConnection> connectRemoteUdp(
        const std::string& localAddr, int localPort, 
        const std::string& remoteAddr, int remotePort);
    static std::shared_ptr<MavLinkConnection> connectTcp(
        const std::string& localAddr, int localPort,
        const std::string& remoteAddr, int remotePort);
    
    // Connection management
    void startListening();
    void stopListening();
    void close();
    
    // Message handling
    void subscribe(MessageHandler handler);
    void unsubscribe(MessageHandler handler);
    void sendMessage(const MavLinkMessageBase& msg);
    
    // Connection bridging
    void join(std::shared_ptr<MavLinkConnection> other);
};
```

#### MavLinkNode
**Purpose**: Generic MAVLink entity representation
**Responsibilities**:
- Heartbeat management
- Parameter handling
- Generic MAVLink node behavior

```cpp
class MavLinkNode {
public:
    MavLinkNode(int systemId, int componentId);
    
    // Connection management
    void connect(std::shared_ptr<MavLinkConnection> connection);
    void disconnect();
    
    // Heartbeat management
    void startHeartbeat(uint8_t type, uint8_t autopilot);
    void stopHeartbeat();
    
    // Parameter handling
    AsyncResult<MavLinkParameter> getParameter(const std::string& name);
    AsyncResult<bool> setParameter(const std::string& name, float value);
    
    // Message handling
    void subscribe(std::function<void(const MavLinkMessage&)> handler);
};
```

#### MavLinkVehicle
**Purpose**: Vehicle-specific MAVLink interface
**Responsibilities**:
- Vehicle state management
- High-level control commands
- Mission and waypoint handling

```cpp
class MavLinkVehicle : public MavLinkNode {
public:
    MavLinkVehicle(int systemId = 1, int componentId = 1);
    
    // State management
    const VehicleState& getState() const;
    void waitForStableState(float timeout = 10.0f);
    
    // Control commands
    AsyncResult<bool> arm();
    AsyncResult<bool> disarm();
    AsyncResult<bool> takeoff(float altitude);
    AsyncResult<bool> land();
    AsyncResult<bool> goToLocation(double lat, double lon, float alt);
    
    // Flight modes
    AsyncResult<bool> setMode(const std::string& mode);
    AsyncResult<bool> setStabilizedFlightMode();
    AsyncResult<bool> setGuidedFlightMode();
};
```

## Connection Management

### Transport Abstraction

The library provides a unified interface for different transport types through the `Port` abstraction:

```cpp
class Port {
public:
    virtual ~Port() = default;
    virtual void connect() = 0;
    virtual void close() = 0;
    virtual int read(uint8_t* buffer, int bytesToRead) = 0;
    virtual int write(const uint8_t* buffer, int bytesToWrite) = 0;
    virtual bool isClosed() const = 0;
};
```

### Connection Types

#### UDP Connections
```cpp
// Local UDP listener (for incoming connections)
auto connection = MavLinkConnection::connectLocalUdp("127.0.0.1", 14550);

// Remote UDP client (for outgoing connections)
auto connection = MavLinkConnection::connectRemoteUdp(
    "127.0.0.1", 14550,    // Local address
    "192.168.1.100", 14560  // Remote address
);
```

#### TCP Connections
```cpp
// TCP client connection
auto connection = MavLinkConnection::connectTcp(
    "127.0.0.1", 0,         // Local (auto-assign port)
    "192.168.1.100", 5760   // Remote
);
```

#### Serial Connections
```cpp
// Serial port connection
auto connection = MavLinkConnection::connectSerial("/dev/ttyUSB0", 57600);

// Windows
auto connection = MavLinkConnection::connectSerial("COM3", 115200);
```

### Connection Bridging

The library supports connection bridging for proxy scenarios:

```cpp
// Bridge serial autopilot to UDP ground station
auto serialConn = MavLinkConnection::connectSerial("/dev/ttyUSB0", 57600);
auto udpConn = MavLinkConnection::connectLocalUdp("127.0.0.1", 14550);

// Join connections to create bridge
serialConn->join(udpConn);
```

## Message Handling

### Message Structure

MAVLink messages are represented as strongly-typed classes:

```cpp
class MavLinkMessageBase {
public:
    uint8_t msgid;
    uint8_t sysid;
    uint8_t compid;
    uint8_t seq;
    
    virtual void encode() = 0;
    virtual void decode(const mavlink_message_t& msg) = 0;
    virtual std::string toJSONString() const = 0;
};
```

### Message Subscription

```cpp
// Subscribe to specific message types
connection->subscribe([](const MavLinkMessage& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitude(msg);
            break;
        // ... other message types
    }
});
```

### Message Generation

The library includes a code generator that creates strongly-typed message classes from MAVLink XML definitions:

```cpp
// Generated message class example
class MavLinkAttitude : public MavLinkMessageBase {
public:
    float roll;         // Roll angle (rad)
    float pitch;        // Pitch angle (rad)
    float yaw;          // Yaw angle (rad)
    float rollspeed;    // Roll angular velocity (rad/s)
    float pitchspeed;   // Pitch angular velocity (rad/s)
    float yawspeed;     // Yaw angular velocity (rad/s)
    
    void encode() override;
    void decode(const mavlink_message_t& msg) override;
    std::string toJSONString() const override;
};
```

## Vehicle State Management

### VehicleState Structure

```cpp
struct VehicleState {
    // Attitude information
    struct AttitudeState {
        float roll, pitch, yaw;              // Euler angles (rad)
        float rollspeed, pitchspeed, yawspeed; // Angular velocities (rad/s)
        uint64_t time_usec;                   // Timestamp
    } attitude;
    
    // Global position
    struct GlobalState {
        double lat, lon;                      // GPS coordinates (deg)
        float alt;                           // Altitude above MSL (m)
        float relative_alt;                  // Altitude above ground (m)
        float vx, vy, vz;                    // Velocity (m/s)
        uint16_t hdg;                        // Heading (cdeg)
    } global_est;
    
    // Local position (NED frame)
    struct LocalState {
        float x, y, z;                       // Local position (m)
        float vx, vy, vz;                    // Local velocity (m/s)
    } local_est;
    
    // RC channels
    struct RCState {
        uint16_t rc[18];                     // RC channel values
        uint8_t rssi;                        // Signal strength
    } rc;
    
    // Control state
    struct ControlState {
        bool armed;                          // Armed/disarmed
        std::string mode;                    // Flight mode
        bool guided;                         // Guided mode active
        bool stabilized;                     // Stabilized mode active
    } controls;
    
    // Statistics
    struct Stats {
        uint64_t last_read_time_usec;        // Last update time
        uint32_t read_count;                 // Number of updates
        uint32_t write_count;                // Number of commands sent
    } stats;
};
```

### State Update Mechanism

```cpp
// State updates are triggered by MAVLink messages
void MavLinkVehicle::handleAttitude(const MavLinkAttitude& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    state_.attitude.roll = msg.roll;
    state_.attitude.pitch = msg.pitch;
    state_.attitude.yaw = msg.yaw;
    state_.attitude.rollspeed = msg.rollspeed;
    state_.attitude.pitchspeed = msg.pitchspeed;
    state_.attitude.yawspeed = msg.yawspeed;
    state_.attitude.time_usec = msg.time_usec;
    
    // Increment version for change detection
    state_version_++;
}
```

## AirSim Integration

### Integration Architecture

```
┌─────────────────┐    MAVLink     ┌─────────────────┐
│     AirSim      │◄──────────────►│   PX4/ArduPilot │
│   Simulation    │    Messages    │    Autopilot    │
└─────────────────┘                └─────────────────┘
         ▲                                   ▲
         │                                   │
         │            MavLinkCom            │
         │         ┌─────────────────┐      │
         └─────────│ MavLinkVehicle  │──────┘
                   │   - State Sync  │
                   │   - Commands    │
                   │   - Telemetry   │
                   └─────────────────┘
```

### Key Integration Points

1. **Vehicle Control**: High-level control interface for AirSim
2. **State Synchronization**: Vehicle state updates from simulation
3. **Command Processing**: External commands via MAVLink
4. **HIL Sensors**: Hardware-in-the-Loop sensor simulation

### Usage in AirSim

```cpp
// Create MAVLink connection for vehicle
auto connection = MavLinkConnection::connectLocalUdp("127.0.0.1", 14550);
auto vehicle = std::make_shared<MavLinkVehicle>(1, 1);
vehicle->connect(connection);

// Wait for vehicle to be ready
vehicle->waitForStableState(10.0f);

// Perform flight operations
vehicle->arm().wait();
vehicle->takeoff(10.0f).wait();
vehicle->goToLocation(47.641468, -122.140165, 50.0f).wait();
vehicle->land().wait();
```

## Advanced Features

### Asynchronous Operations

The library uses `AsyncResult<T>` for non-blocking operations:

```cpp
template<typename T>
class AsyncResult {
public:
    // Blocking wait
    T wait(float timeout = 10.0f);
    
    // Non-blocking check
    bool isComplete() const;
    
    // Callback on completion
    void then(std::function<void(const T&)> callback);
    
    // Cancel operation
    void cancel();
};
```

### Connection Monitoring

```cpp
// Connection health monitoring
connection->subscribe([](const MavLinkMessage& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        last_heartbeat_time_ = std::chrono::steady_clock::now();
    }
});

// Check connection health
bool isHealthy() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_heartbeat_time_).count();
    return elapsed < 5; // Healthy if heartbeat within 5 seconds
}
```

### Message Filtering

```cpp
// Filter messages by type or source
connection->subscribe([](const MavLinkMessage& msg) {
    // Only process messages from specific system
    if (msg.sysid == 1) {
        processMessage(msg);
    }
});
```

## Threading and Performance

### Threading Model

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Read Thread   │    │ Process Thread  │    │  Handler Thread │
│                 │    │                 │    │                 │
│ - Receive data  │───►│ - Parse messages│───►│ - Execute       │
│ - Buffer mgmt   │    │ - Validate      │    │   callbacks     │
│ - Error handling│    │ - Route msgs    │    │ - Update state  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Performance Optimizations

1. **Zero-Copy Parsing**: Minimal data copying in message pipeline
2. **Message Queuing**: Buffered message processing
3. **Selective Parsing**: Only parse messages with active subscribers
4. **Connection Pooling**: Reuse of connection objects

### Thread Safety

```cpp
// Thread-safe state access
const VehicleState& getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

// Atomic operations for statistics
std::atomic<uint32_t> message_count_{0};
std::atomic<uint64_t> last_message_time_{0};
```

## Usage Examples

### Basic Connection Setup

```cpp
#include "MavLinkConnection.hpp"
#include "MavLinkVehicle.hpp"

// Create connection
auto connection = MavLinkConnection::connectLocalUdp("127.0.0.1", 14550);

// Create vehicle interface
auto vehicle = std::make_shared<MavLinkVehicle>(1, 1);
vehicle->connect(connection);

// Start listening
connection->startListening();

// Wait for connection
vehicle->waitForStableState(10.0f);
```

### Command Execution

```cpp
// Arm vehicle
auto armResult = vehicle->arm();
if (armResult.wait(5.0f)) {
    std::cout << "Vehicle armed successfully" << std::endl;
} else {
    std::cout << "Failed to arm vehicle" << std::endl;
}

// Takeoff
auto takeoffResult = vehicle->takeoff(10.0f);
takeoffResult.then([](bool success) {
    if (success) {
        std::cout << "Takeoff completed" << std::endl;
    }
});
```

### State Monitoring

```cpp
// Monitor vehicle state
while (true) {
    const auto& state = vehicle->getState();
    
    std::cout << "Position: " << state.global_est.lat << ", " 
              << state.global_est.lon << ", " << state.global_est.alt << std::endl;
    std::cout << "Attitude: " << state.attitude.roll << ", " 
              << state.attitude.pitch << ", " << state.attitude.yaw << std::endl;
    std::cout << "Armed: " << (state.controls.armed ? "Yes" : "No") << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

### Message Logging

```cpp
// Enable message logging
auto logger = std::make_shared<MavLinkLog>("mavlink.log");
connection->subscribe([logger](const MavLinkMessage& msg) {
    logger->writeMessage(msg);
});
```

## Conclusion

The MavLinkCom library provides a robust, scalable foundation for MAVLink communication with:

- **Multi-protocol support** (UDP, TCP, Serial)
- **Strongly-typed message system** with code generation
- **Asynchronous command execution** with timeout handling
- **Thread-safe state management** with atomic operations
- **Connection bridging** for proxy scenarios
- **Comprehensive logging** and debugging support

This architecture enables seamless integration with AirSim's simulation environment while maintaining compatibility with standard MAVLink protocols and tools.