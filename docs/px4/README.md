# PX4 & MAVLink Development Documentation

This section contains documentation for PX4 integration, MAVLink communication, and flight controller development with Cosys-AirSim.

## 🚁 Flight Controller Integration

### PX4 Setup & Configuration
- **[PX4 Setup](px4_setup.md)** - Complete PX4 setup and configuration guide
- **[PX4 Build](px4_build.md)** - Building PX4 from source for AirSim integration
- **[PX4 SITL](px4_sitl.md)** - Software-in-the-Loop simulation setup
- **[PX4 SITL WSL2](px4_sitl_wsl2.md)** - Running PX4 SITL on Windows with WSL2
- **[PX4 Multi-Vehicle](px4_multi_vehicle.md)** - Multi-drone setup with PX4

### Flight Control Systems
- **[Flight Controller](flight_controller.md)** - Overview of flight controller integration
- **[Simple Flight](simple_flight.md)** - Built-in simple flight controller
- **[PX4 Lockstep](px4_lockstep.md)** - Deterministic simulation with PX4

### MAVLink Communication
- **[MAVLink Architecture](mavlink_architecture.md)** - MAVLink communication architecture
- **[MAVLink Connection Design](mavlink_connection_design.md)** - Connection design patterns
- **[MAVLink Networking Troubleshooting](mavlink_networking_troubleshooting.md)** - Network troubleshooting guide
- **[MAVLinkCom](mavlinkcom.md)** - MAVLink communication library
- **[MAVLinkCom MoCap](mavlinkcom_mocap.md)** - Motion capture integration

## 🛠️ Debugging & Troubleshooting

### Logging & Debugging
- **[PX4 Logging](px4_logging.md)** - PX4 logging and debugging techniques
- **[PX4 Console](PX4_CONSOLE.md)** - PX4 console commands and usage
- **[PX4 GPS Home Troubleshooting](px4_gps_home_troubleshooting.md)** - GPS home position issues

## 📊 Design Documentation

### Architecture Diagrams
- **[MAVLinkCom Design](../mavlinkcom_design/)** - Detailed design diagrams and documentation

## 🔗 Related Documentation

- **[Core APIs](../core/apis.md)** - Core AirSim API documentation
- **[Multi-Vehicle Setup](../core/multi_vehicle.md)** - General multi-vehicle configuration
- **[ROS2 Integration](../ros2/)** - ROS2 wrapper for PX4 integration

## 📋 Quick Reference

### Common PX4 Commands
```bash
# Start PX4 SITL
make px4_sitl none_iris

# Connect to PX4 console
pxh> help

# Check vehicle status
pxh> commander status
```

### MAVLink Testing
```bash
# Test MAVLink connection
mavlink status

# Monitor MAVLink messages
mavlink stream -s HIGHRES_IMU -r 50
```

### Troubleshooting Checklist
1. ✅ PX4 SITL running on correct port
2. ✅ AirSim listening on MAVLink port (14540)
3. ✅ Network connectivity between PX4 and AirSim
4. ✅ Vehicle armed and ready for API control
5. ✅ GPS home position set correctly

For additional support, see the [troubleshooting guides](px4_logging.md) and [network debugging](mavlink_networking_troubleshooting.md) documentation.