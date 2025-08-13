# AirSim RPC Documentation

This directory contains comprehensive documentation about AirSim's Remote Procedure Call (RPC) architecture and implementation.

## Documentation Overview

### [AirSim RPC Architecture: A Comprehensive Deep Dive](airsim_rpc_architecture_deep_dive.md)
A complete analysis of AirSim's RPC system covering:

- **Architecture Overview**: High-level design philosophy and component relationships
- **Core RPC Foundation**: Base classes, design patterns, and threading models
- **Vehicle-Specific Implementations**: Multirotor, car, and computer vision specializations
- **Python Client Architecture**: msgpackrpc integration and data type mapping
- **C++ Server Implementation**: API provider architecture and multi-vehicle support
- **ROS2 Bridge Architecture**: Bridge design and message type integration
- **Complete Message Flow Analysis**: End-to-end trace of command execution
- **MAVLink Integration**: Flight controller integration and dual-channel architecture
- **Integration Patterns**: Cross-language communication strategies
- **Performance Analysis**: Optimization opportunities and recommendations

## Key Concepts

### Dual-Channel Architecture
AirSim employs a sophisticated dual-channel communication system:

```
Client Applications (Python/ROS2/MATLAB)
           ↓
    msgpack-rpc over TCP (Port 41451)
           ↓
   C++ RPC Server (AirLib)
           ↓
   MavLinkMultirotorApi Bridge
           ↓
    MAVLink over UDP (Port 14550)
           ↓
   Flight Controller (PX4/ArduPilot)
           ↓
   Unreal Engine Plugin
```

### Communication Channels
- **RPC Channel**: High-level mission control (seconds to minutes)
- **MAVLink Channel**: Real-time flight control (milliseconds, 250Hz)

### Core Components
- **RpcLibServerBase**: Base server infrastructure
- **RpcLibClientBase**: Base client communication
- **RpcLibAdaptorsBase**: Serialization/deserialization
- **ApiProvider**: Vehicle discovery and API dispatch
- **MavLinkMultirotorApi**: Protocol translation bridge
- **Vehicle-Specific APIs**: Specialized implementations per vehicle type

### Design Patterns
- **Facade Pattern**: Simplified interfaces to complex systems
- **Adapter Pattern**: Type conversion and serialization
- **Strategy Pattern**: Pluggable vehicle API implementations (SimpleFlight vs MAVLink)
- **PIMPL Idiom**: Implementation hiding and dependency management

## Quick Reference

### Default Configuration
- **RPC Port**: 41451 (TCP)
- **MAVLink Port**: 14550 (UDP)
- **Serialization**: msgpack binary format
- **Threading**: Async thread pool for concurrent requests
- **Flight Controller**: PX4/ArduPilot integration via MAVLink

### Vehicle Types Supported
- **Multirotor**: Drones with flight control capabilities
- **Car**: Ground vehicles with driving controls
- **Computer Vision**: Camera-only mode for sensor data collection

### Client Libraries
- **Python**: `cosysairsim` package
- **ROS2**: `airsim_interfaces` and bridge implementations
- **MATLAB**: AirSim MATLAB toolbox
- **C++**: Direct AirLib integration

## Performance Considerations

### Optimizations
- Binary serialization with msgpack
- Persistent TCP connections
- Asynchronous operation support
- Multiple specialized RPC clients for ROS2

### Limitations
- Single point of failure (central server)
- Serialization overhead (double conversion)
- Thread context switches
- Memory allocation per RPC call

## Development Guidelines

### Adding New Vehicle Types
1. Extend `RpcLibServerBase` and `RpcLibClientBase`
2. Create vehicle-specific adaptors
3. Implement API bindings
4. Add client library support

### Adding New APIs
1. Define method in appropriate `*ApiBase` class
2. Create adaptor structs for new data types
3. Add RPC server bindings
4. Implement client-side methods

### Performance Optimization
- Use async operations for long-running commands
- Batch operations when possible
- Enable compression for large data transfers
- Consider specialized RPC clients for high-frequency operations

## Related Documentation
- [Core APIs](../core/apis.md)
- [ROS2 Integration](../ros2/README.md)
- [PX4 Integration](../px4/README.md)
- [MAVLink Architecture](../mavlinkcom_design/airsim_px4_network_architecture.md)
- [Multi-Vehicle Setup](../core/multi_vehicle.md)
- [Python Client Guide](../core/apis.md)