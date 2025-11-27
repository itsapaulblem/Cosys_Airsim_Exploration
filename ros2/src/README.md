# AirSim ROS2 Multi-Vehicle Modular Architecture


## Table of Contents 
1. Introduction
2. Startup Instructions
3. Architecture Overview
<<<<<<< HEAD
4. ROS Topics and Services
5. File by File Explanation
6. Detailed Comparisons: Old vs New Architecture
7. Design Decisions
8. Troubleshooting & FAQ
9. References 
=======
4. RPC Dynamic Discovery
5. ROS Topics and Services
6. File by File Explanation
7. Detailed Comparisons: Old vs New Architecture
8. Design Decisions
9. Troubleshooting & FAQ
10. References 
>>>>>>> main

--- 

## 1. Introduction 

This documentation describes the **Cosys-AirSim ROS 2 multi-vehicle modular architecture**, designed for robust, scalable, and maintainable multi-drone simulation and control. It replaces the legacy monolithic ROS node with a modern, component-based approach, supporting parallel sensor processing, fault isolation and dynamic vehicle management.

---

## 2. Startup Instructions (wsl 2.5.10.0 & Windows 10)

### Step 1: Run the Python generate settings.py file to determine the number of drones
``` bash
wsl
cd Cosys-AirSim/PythonClient/multirotor
python3 generate_settings.py 2
```
### Step 2: Launch Cosys-AirSim 
- Start Cosys-AirSim in Unreal Engine 5.5

### Step 3: Launch PX4 SITL (for multiple drones)
For each drone, run in separate terminals:
```bash
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i <instance_id>
```

Example for two drones:
```bash
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 1
```

Or use:
```bash
make px4_sitl_default none_iris
```

### Step 3: Build and Source ROS 2 Workspace

```bash
colcon build
source install/setup.bash 
```

### Step 4: Launch ROS2 Nodes

<<<<<<< HEAD
- **Single drone for testing (simple):** 
```bash
ros2 launch airsim_ros_pkgs simple_single_drone.launch.py
```

- **Single drone** 
```bash
ros2 launch airsim_ros_pkgs single_drone.launch.py
```

- **Multi-drone**
```bash
ros2 launch airsim_pos_pkgs multi_drone.launch.py
=======
**Ultra-Clean Two-Approach Strategy:**

- **Primary (RPC Auto-Discovery):** 
```bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
```
This creates nodes with perfect ultra-clean naming:
- Nodes: `/Droan1`, `/PX4_Drone2`, `/airsim_coordination_node`
- Topics: `/Droan1/odom_local_ned`, `/PX4_Drone2/imu`, etc.
- Services: `/Droan1/takeoff`, `/PX4_Drone2/land`, etc.

- **Legacy (Monolithic Single-Node):** 
```bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```
Traditional single-node approach for backward compatibility.

**Specialized Utilities:**
```bash
# Visualization
ros2 launch airsim_ros_pkgs rviz.launch.py

# Position controllers
ros2 launch airsim_ros_pkgs airsim_with_simple_PD_position_controller.launch.py
ros2 launch airsim_ros_pkgs position_controller_simple.launch.py

# Dynamic constraints
ros2 launch airsim_ros_pkgs dynamic_constraints.launch.py
>>>>>>> main
``` 

---

## 3. Architecture Overview

### Key Components

<<<<<<< HEAD
#### A. Vehicle Nodes

- **VehicleNodeBase**: Abstract base for all vehicle types. Handles parameter management, AirSim connections, publishers/services/timers and callback groups for parallel sensor processing. 
- **MultirotorNode**: Inherits from VehicleNodeBase. Implements drone-specific publishers (odom, GPS, IMU, environment, camera, lidar), services (takeoff, land), velocity command subscrivers, and sensor data processing. 
- **SimpleMultirotorNode**: Minimal node for single-drone testing/debugging. No inheritance, direct AirSim connection.

#### B. Coordination Node

- **CoordinationNode**: Manages global services (reset all, takeoff all, land all, pause simulation, health check), publishes system status and GPS origin, monitors all vehicles.

#### C. Settings Parser 
- **VehicleSettingsParser**: Parses AirSim `settings.json` to extract vehicle configurations and global parameters, enabling dynamic node creation. 

--- 

## 4. ROS Topics and Services
=======
#### A. Ultra-Clean Node Architecture

**Primary Approach (RPC Dynamic):**
- **Perfect Ultra-Clean Naming**: Vehicle names ARE node names
  - Individual vehicle nodes: `/Droan1`, `/PX4_Drone2`, `/VehicleName`
  - Topics: `/Droan1/odom_local_ned`, `/PX4_Drone2/imu`, `/VehicleName/global_gps`
  - Services: `/Droan1/takeoff`, `/PX4_Drone2/land`, `/VehicleName/reset`
- Global coordination node: `/airsim_coordination_node`
- Each vehicle node handles its own sensors, services, and state
- Fault isolation: one vehicle failure doesn't affect others
- **Cross-Platform Support**: Windows AirSim + Docker ROS2 via direct RPC
- Automatic discovery via RPC - zero manual configuration

**Legacy Approach (Monolithic):**
- Single node: `/airsim_node` handles ALL vehicles
- Backward compatibility for existing systems
- All vehicles in one process (single point of failure)

#### B. RPC Dynamic Discovery System
- **Automatic Vehicle Discovery**: Queries running AirSim server via RPC to discover active vehicles in real-time
- **Cross-Platform Architecture**: Windows AirSim + Docker ROS2 seamlessly supported
- **Direct RPC Communication**: Raw TCP/msgpack-rpc - no Python client dependencies required
- **Multiple Fallback Methods**: 
  1. Direct RPC (primary) - works without any Python clients
  2. cosysairsim Python client (if available)
  3. airsim Python client (if available)  
  4. Settings file fallback
  5. Default vehicle creation
- **Zero Configuration**: Discovers vehicles automatically - no manual setup needed

#### C. Vehicle Node Implementation
- **Ultra-Clean C++ Implementation**: Explicit topic prefixing creates perfect `/VehicleName/topic` structure
- **Individual RPC Connections**: Each vehicle node maintains its own isolated AirSim connection
- **Parallel Processing**: Independent timers and callback groups for maximum performance
- **Fault Isolation**: One vehicle failure doesn't affect others 

--- 

## 4. RPC Dynamic Discovery

### Overview

The RPC Dynamic Discovery system is the **recommended** way to launch AirSim ROS2 nodes. It automatically:

1. **Discovers Active Vehicles**: Queries the running AirSim server via RPC to find all active vehicles
2. **Creates Namespaced Nodes**: Automatically creates individual ROS2 nodes for each discovered vehicle
3. **Cross-Platform Compatible**: Works with Windows AirSim + Docker ROS2, or pure Linux setups
4. **No Manual Configuration**: No need to manually specify vehicle names or counts

### Launch Command

```bash
# Basic launch (auto-discovers vehicles and creates nodes)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# With custom AirSim server location
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py host_ip:=192.168.1.100 host_port:=41451

# With debug output
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py debug:=true
```

### Discovery Methods

The system uses multiple fallback methods:

1. **Direct RPC** (Primary): Raw TCP/msgpack-rpc calls - no dependencies
2. **cosysairsim Client**: Uses Cosys-AirSim Python client if available
3. **airsim Client**: Uses standard AirSim Python client if available
4. **Settings Fallback**: Reads settings.json file if RPC fails
5. **Default Fallback**: Creates default vehicle if all else fails

### Real-World Example Output

**Windows AirSim + Docker ROS2 Discovery:**
```
[INFO] [rpc_dynamic]: Attempting direct RPC to 172.28.240.1:41451...
[INFO] [rpc_dynamic]: Direct RPC discovered vehicles: ['Droan1', 'PX4_Drone2']
[INFO] [rpc_dynamic]: RPC discovery successful: 2 vehicles
[INFO] [rpc_dynamic]: Creating node for vehicle 'Droan1' of type 'multirotor'
[INFO] [rpc_dynamic]: Creating node for vehicle 'PX4_Drone2' of type 'multirotor'
```

**Resulting Ultra-Clean Structure:**
```bash
# Nodes discovered
/Droan1                        # Vehicle node
/PX4_Drone2                    # Vehicle node  
/airsim_coordination_node      # Global coordination

# Topics available  
/Droan1/odom_local_ned        # Odometry
/Droan1/global_gps            # GPS
/Droan1/imu                   # IMU
/PX4_Drone2/odom_local_ned    # Odometry
/PX4_Drone2/global_gps        # GPS
/PX4_Drone2/imu               # IMU

# Services available
/Droan1/takeoff               # Individual takeoff
/Droan1/land                  # Individual land
/PX4_Drone2/takeoff           # Individual takeoff
/PX4_Drone2/land              # Individual land
/airsim_coordination_node/takeoff_all  # Global takeoff
/airsim_coordination_node/land_all     # Global land
```

### Benefits

- ✅ **Perfect Ultra-Clean Naming**: Vehicle names ARE node names (`/Droan1`, `/PX4_Drone2`)
- ✅ **Cross-Platform Architecture**: Windows AirSim + Docker ROS2 seamlessly supported  
- ✅ **Zero Configuration**: Automatic discovery - no manual vehicle specification needed
- ✅ **Direct RPC Communication**: No Python client dependencies required
- ✅ **Multiple Fallback Methods**: Robust discovery with graceful fallbacks
- ✅ **Real-Time Discovery**: Only active vehicles discovered and launched
- ✅ **Fault Isolation**: Individual vehicle nodes with independent connections
- ✅ **Two-Strategy Approach**: Modern RPC dynamic + Legacy monolithic for compatibility

---

## 5. ROS Topics and Services
>>>>>>> main

### Topics Published Per Vehicle 

| Topic Name                | Message Type                       | Description                                      |
|---------------------------|------------------------------------|--------------------------------------------------|
<<<<<<< HEAD
| `/droneX/odom`            | `nav_msgs/msg/Odometry`            | Vehicle odometry (position, orientation, velocity)|
| `/droneX/gps`             | `sensor_msgs/msg/NavSatFix`        | GPS data (lat, lon, alt)                         |
| `/droneX/imu`             | `sensor_msgs/msg/Imu`              | IMU data (orientation, angular/linear accel)     |
| `/droneX/environment`     | `airsim_interfaces/msg/Environment`| AirSim environment state (pressure, temp, etc.)  |
| `/droneX/cameraY/image`   | `sensor_msgs/msg/Image`            | Camera image (Y = camera index/name)             |
| `/droneX/cameraY/camera_info` | `sensor_msgs/msg/CameraInfo`   | Camera calibration info                          |
| `/droneX/lidarZ/points`   | `sensor_msgs/msg/PointCloud2`      | Lidar point cloud (Z = lidar index/name)         |
| `/droneX/mag`             | `sensor_msgs/msg/MagneticField`    | Magnetometer data                                |
| `/droneX/baro`            | `sensor_msgs/msg/Range`            | Barometer/altimeter data                         |

### Topics Published by Coordination Node 

| Topic Name                | Message Type                       | Description                                      |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/origin_geo_point`       | `airsim_interfaces/msg/GPSYaw`     | Global GPS origin for all vehicles               |
| `/system_status`          | `airsim_interfaces/msg/StringArray`| Status of all vehicles (READY/ERROR)             |
| `/clock`                  | `rosgraph_msgs/msg/Clock`          | Simulation time                                  |
=======
| `/VehicleName/odom_local_ned` | `nav_msgs/msg/Odometry`        | Vehicle odometry (position, orientation, velocity)|
| `/VehicleName/global_gps` | `sensor_msgs/msg/NavSatFix`        | GPS data (lat, lon, alt)                         |
| `/VehicleName/imu`        | `sensor_msgs/msg/Imu`              | IMU data (orientation, angular/linear accel)     |
| `/VehicleName/environment`| `airsim_interfaces/msg/Environment`| AirSim environment state (pressure, temp, etc.)  |
| `/VehicleName/cameraY/image` | `sensor_msgs/msg/Image`         | Camera image (Y = camera index/name)             |
| `/VehicleName/cameraY/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration info                          |
| `/VehicleName/lidarZ/points` | `sensor_msgs/msg/PointCloud2`   | Lidar point cloud (Z = lidar index/name)         |
| `/VehicleName/mag`        | `sensor_msgs/msg/MagneticField`    | Magnetometer data                                |
| `/VehicleName/baro`       | `sensor_msgs/msg/Range`            | Barometer/altimeter data                         |

### Topics Published by Coordination Node 

| Topic Name                      | Message Type                       | Description                                      |
|---------------------------------|------------------------------------|--------------------------------------------------|
| `/airsim_coordination_node/origin_geo_point` | `airsim_interfaces/msg/GPSYaw` | Global GPS origin for all vehicles               |
| `/airsim_coordination_node/system_status` | `airsim_interfaces/msg/StringArray` | Status of all vehicles (READY/ERROR)             |
| `/airsim_coordination_node/clock` | `rosgraph_msgs/msg/Clock`       | Simulation time                                  |

### Node Structure (Ultra-Clean)

**Perfect Vehicle Name → Node Name Mapping:**

| Node Name                    | Type                  | Description                                      |
|------------------------------|----------------------|--------------------------------------------------|
| `/Droan1`                    | Vehicle Node         | Individual drone (vehicle name IS node name)     |
| `/PX4_Drone2`                | Vehicle Node         | Individual drone (vehicle name IS node name)     |
| `/VehicleName`               | Vehicle Node         | Any discovered vehicle (ultra-clean naming)      |
| `/airsim_coordination_node`  | Coordination Node    | Global services and system monitoring            |

**Key Architecture Features:**
- Vehicle names from AirSim settings become ROS2 node names directly
- No unnecessary namespace hierarchy or suffixes
- Topics prefixed in C++ code: `vehicle_name + "/" + topic_name`
- Services follow same pattern: `/VehicleName/service_name`
>>>>>>> main

### Services Per Vehicle

| Service Name              | Service Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
<<<<<<< HEAD
| `/droneX/takeoff`         | `airsim_interfaces/srv/Takeoff`    | Takeoff command for this vehicle                 |
| `/droneX/land`            | `airsim_interfaces/srv/Land`       | Land command for this vehicle                    |
| `/droneX/reset`           | `airsim_interfaces/srv/Reset`      | Reset this vehicle in AirSim                     |

### Global Services (Coordination Node)

| Service Name              | Service Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/reset_all`              | `airsim_interfaces/srv/Reset`      | Reset all vehicles                               |
| `/takeoff_all`            | `airsim_interfaces/srv/Takeoff`    | Takeoff all vehicles                             |
| `/land_all`               | `airsim_interfaces/srv/Land`       | Land all vehicles                                |
| `/pause_simulation`       | `std_srvs/srv/SetBool`             | Pause/unpause AirSim simulation                  |
| `/health_check`           | `airsim_interfaces/srv/ListSceneObjectTags` | Check health/status of all vehicles     |

### Command Topics (Subscribers)

| Topic Name                | Message Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/droneX/vel_cmd_body_frame` | `airsim_interfaces/msg/VelCmd`  | Velocity command in body frame                   |
| `/droneX/vel_cmd_world_frame`| `airsim_interfaces/msg/VelCmd`  | Velocity command in world frame                  |

---

## 5. File-by-File Explanation 
=======
| `/VehicleName/takeoff`    | `airsim_interfaces/srv/Takeoff`    | Takeoff command for this vehicle                 |
| `/VehicleName/land`       | `airsim_interfaces/srv/Land`       | Land command for this vehicle                    |
| `/VehicleName/reset`      | `airsim_interfaces/srv/Reset`      | Reset this vehicle in AirSim                     |

### Global Services (Coordination Node)

| Service Name                        | Service Type                       | Functionality                                    |
|-------------------------------------|------------------------------------|--------------------------------------------------|
| `/airsim_coordination_node/reset_all` | `airsim_interfaces/srv/Reset`   | Reset all vehicles                               |
| `/airsim_coordination_node/takeoff_all` | `airsim_interfaces/srv/Takeoff` | Takeoff all vehicles                             |
| `/airsim_coordination_node/land_all` | `airsim_interfaces/srv/Land`     | Land all vehicles                                |
| `/airsim_coordination_node/pause_simulation` | `std_srvs/srv/SetBool`     | Pause/unpause AirSim simulation                  |
| `/airsim_coordination_node/health_check` | `airsim_interfaces/srv/ListSceneObjectTags` | Check health/status of all vehicles     |

### Command Topics (Subscribers)

| Topic Name                      | Message Type                       | Functionality                                    |
|-------------------------------------|------------------------------------|--------------------------------------------------|
| `/VehicleName/vel_cmd_body_frame`  | `airsim_interfaces/msg/VelCmd`     | Velocity command in body frame                   |
| `/VehicleName/vel_cmd_world_frame` | `airsim_interfaces/msg/VelCmd`     | Velocity command in world frame                  |

---

## 6. File-by-File Explanation 
>>>>>>> main

### New Modular Architecture Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `vehicle_node_base.hpp/cpp`       | Abstract base for all vehicle nodes. Handles parameters, connections, publishers, timers, callback groups.|
| `multirotor_node.hpp/cpp`         | Implements drone-specific logic: sensor publishers, command subscribers, takeoff/land services.         |
| `simple_multirotor_node.cpp`      | Minimal node for single-drone testing/debugging. Direct AirSim connection, basic publishers/services.   |
| `coordination_node.hpp/cpp`       | Global node for system-wide services, status monitoring, GPS origin publishing, health checks.          |
| `vehicle_settings_parser.hpp/cpp` | Parses AirSim `settings.json` for dynamic vehicle configuration. Used by launch files for node creation.|
| `multirotor_main.cpp`             | Main entry for launching a multirotor node (per vehicle).                                              |
| `coordination_main.cpp`           | Main entry for launching the coordination node.                                                        |

<<<<<<< HEAD
### Launch Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `simple_single_drone.launch.py`   | Launches a single `simple_multirotor_node` in `/drone1` namespace. For quick testing/debugging.         |
| `single_drone.launch.py`          | Launches two `multirotor_node` instances and the coordination node.                                     |
| `multi_drone.launch.py`           | Dynamically launches nodes for all vehicles in `settings.json`, plus the coordination node.             |
| `airsim_node.launch.py`           | Legacy: launches the old monolithic node.                                                               |
=======
### Launch Files (Streamlined)

**Core Launch Files:**

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `rpc_dynamic_vehicles.launch.py`  | **PRIMARY**: Ultra-clean RPC auto-discovery. Creates perfect `/VehicleName` nodes automatically via direct RPC communication. Supports cross-platform Windows AirSim + Docker ROS2. |
| `airsim_node.launch.py`           | **LEGACY**: Monolithic single-node approach for backward compatibility with existing systems.           |

**Utility Launch Files:**

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `rviz.launch.py`                  | Visualization support with RViz for monitoring vehicle states and sensor data.                          |
| `airsim_with_simple_PD_position_controller.launch.py` | Position control integration for advanced flight control.                               |
| `dynamic_constraints.launch.py`   | Dynamic constraint handling for advanced mission planning.                                              |
| `position_controller_simple.launch.py` | Simple position controller for basic waypoint navigation.                                        |

**Streamlined Strategy:**
- **Two Primary Approaches**: RPC Dynamic (modern) + Monolithic (legacy compatibility)
- **Specialized Utilities**: Focused on specific control and visualization needs
- **Deprecated Files Removed**: Cleaned up from 12+ files to 6 focused files
>>>>>>> main

### Legacy (Old Architecture) Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `airsim_ros_wrapper.h/cpp`        | Monolithic node managing all vehicles in one process. Single point of failure, sequential processing.   |
| `airsim_node.cpp`                 | Main for launching the old monolithic node.                                                             |

---

<<<<<<< HEAD
## 6. Detailed Comparison: Old vs. New Architecture
=======
## 7. Detailed Comparison: Old vs. New Architecture
>>>>>>> main

| Aspect                | Old (Monolithic)                      | New (Modular, Multi-Node)                | Why New is Better                        |
|-----------------------|---------------------------------------|------------------------------------------|------------------------------------------|
| Node Structure        | Single node for all vehicles          | One node per vehicle, plus coordination  | Fault isolation, parallelism             |
| Extensibility         | Hard to add new vehicle types         | Easy to add new vehicle types/classes    | Clean inheritance, modular files         |
| Fault Isolation       | Failure in one vehicle affects all    | Each vehicle node is independent         | One crash doesn't affect others          |
| Performance           | Single-threaded, bottlenecked         | Multi-threaded, scalable                 | Parallel sensor processing               |
| Launch Flexibility    | Static, hardcoded                     | Dynamic, based on settings.json          | Add/remove vehicles without code change  |
| Coordination          | Ad-hoc, limited                       | Dedicated coordination node              | Centralized global services              |
| Testing/Debugging     | Hard to isolate issues                | Can launch/test nodes individually       | Per-vehicle logs, easier debugging       |
| Code Organization     | Large, monolithic classes             | Clean, separated by vehicle type         | Easier maintenance, less code coupling   |
| ROS2 Best Practices   | Not followed                          | Follows ROS2 node/component patterns     | Modern, maintainable, scalable           |
| Resource Management   | All processing on single core/thread  | Per-node threading, callback groups      | Better CPU utilization                   |
| RPC Connections       | Shared for all vehicles               | Independent per vehicle                  | No RPC contention, isolated failures     |
| Sensor Timers         | Shared, sequential                    | Per-vehicle, parallel                    | No sensor bottlenecks                    |

**Summary:**  
The new architecture is modular, robust, and scalable. Each vehicle runs in its own node and namespace, with isolated connections and timers. The coordination node manages global services and monitoring. This design enables parallel sensor processing, fault isolation, and dynamic vehicle management, making it ideal for large-scale multi-vehicle simulation.

---
## Overview
This documentation describes the modular, multi-node ROS2 architecture for Cosys-AirSim, supporting robust multi-drone simulation and control. It covers:

- The new architecture and its components
- How the system works
- How to launch and use it
- Error checking and troubleshooting
- Differences from the legacy (monolithic) architecture

---

<<<<<<< HEAD
## 7. Design Decisions
=======
## 8. Design Decisions
>>>>>>> main

- **Modularity:** Each vehicle type gets its own node class, making it easy to extend for new vehicle types (cars, drones, etc.).
- **Isolation:** Per-vehicle nodes mean a crash or RPC error in one vehicle does not affect others.
- **Parallelism:** Isolated callback groups and timers allow sensors to be processed in parallel, improving performance.
<<<<<<< HEAD
- **Dynamic Launch:** VehicleSettingsParser enables dynamic node creation based on `settings.json`, so you can add/remove vehicles without changing code.
- **Coordination Node:** Centralizes global services (reset, takeoff, land, pause, health check) and system status monitoring.
- **Legacy Compatibility:** Old files (`airsim_ros_wrapper.*`, `airsim_node.cpp`) are retained for reference and backward compatibility, but are not recommended for new deployments.

---

## 8. Troubleshooting & FAQ

- **bad_weak_ptr errors:** Ensure you have rebuilt your workspace and are not running old binaries.
- **Nodes not connecting:** Check that AirSim is running and vehicle names match those in `settings.json`.
- **Adding vehicles:** Update `settings.json` and use `multi_drone.launch.py`—nodes will be created automatically.
- **PX4 SITL:** Each drone instance needs its own PX4 SITL process.
- **Logs:** Each vehicle node logs independently; use `ros2 node list` and `ros2 topic list` to inspect running nodes and topics.

---

## 9. References
=======
- **Ultra-Clean Architecture:** Vehicle names ARE node names (`/Droan1`, `/PX4_Drone2`) - no unnecessary hierarchy.
- **Two-Strategy Approach:** Modern RPC auto-discovery as primary + Legacy monolithic for compatibility.
- **RPC Dynamic Discovery:** Real-time vehicle discovery via RPC eliminates manual configuration and supports cross-platform setups (Windows AirSim + Docker ROS2).
- **Streamlined Launch Files:** Reduced from 12+ files to 6 focused files (2 primary + 4 specialized utilities).
- **Coordination Node:** Centralizes global services at `/airsim_coordination_node` for system-wide operations.
- **Legacy Compatibility:** Monolithic `airsim_node.launch.py` provides full backward compatibility for existing deployments.

---

## 9. Troubleshooting & FAQ

### Common Issues

**RPC Discovery Issues:**
- **No vehicles discovered:** Ensure AirSim is running and accessible. For Docker: check if `172.28.240.1:41451` is reachable.
- **Connection timeout:** Increase `rpc_timeout` parameter: `ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py rpc_timeout:=20.0`
- **Cross-platform networking:** Verify Windows firewall allows Docker connections to port 41451.

**Node and Topic Issues:**
- **Topics not showing:** Check nodes are running with `ros2 node list`. Should show `/VehicleName` not `/namespace/VehicleName`.
- **Wrong topic structure:** Ensure using updated C++ implementation with explicit topic prefixing.
- **Services not available:** Verify vehicle nodes are healthy: `ros2 service list | grep VehicleName`

**Legacy Compatibility:**
- **Old namespace structure:** Use legacy launch: `ros2 launch airsim_ros_pkgs airsim_node.launch.py`
- **Missing deprecated launch files:** Use streamlined approach with 2 core + 4 utility files only.

**Vehicle Configuration:**
- **Vehicle not discovered:** Check AirSim `settings.json` has vehicle defined and AirSim is running.
- **Adding vehicles:** Just update AirSim `settings.json` and restart - RPC discovery handles automatically.
- **PX4 SITL:** Each drone instance needs separate PX4 SITL process with unique instance ID.

### Debug Commands

```bash
# Check discovered nodes (should show /VehicleName format)
ros2 node list

# Check topics (should show /VehicleName/topic_name format)  
ros2 topic list | grep -E "/(Droan|PX4_)"

# Test RPC connection manually
python3 -c "
import socket; 
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
print('✅ Connection OK' if sock.connect_ex(('172.28.240.1', 41451)) == 0 else '❌ Connection failed');
sock.close()
"

# Monitor vehicle data
ros2 topic echo /Droan1/odom_local_ned --once
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"

# Check coordination node  
ros2 service call /airsim_coordination_node/health_check airsim_interfaces/srv/ListSceneObjectTags
```

### Logs and Monitoring
- Each vehicle node logs independently with `/VehicleName` prefix
- Use `ros2 node info /VehicleName` to check individual node status
- Launch with debug: `ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py debug:=true`

---

## 10. References
>>>>>>> main

- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [ROS2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)

---
  
<<<<<<< HEAD
- **List nodes:**
  ```bash
  ros2 node list
  ```
- **List topics:**
  ```bash
  ros2 topic list
  ```
- **Echo odometry:**
  ```bash
  ros2 topic echo /drone1/odom
  ```
- **Takeoff:**
  ```bash
  ros2 service call /drone1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
  ```
- **Land:**
  ```bash
  ros2 service call /drone1/land airsim_interfaces/srv/Land "{wait_on_last_task: true}"
  ```
- **Global takeoff:**
  ```bash
  ros2 service call /takeoff_all airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
  ```
=======
## Quick Reference Commands

### Launch Commands
```bash
# Primary: Ultra-clean RPC auto-discovery
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Legacy: Monolithic single-node (backward compatibility)
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# With custom AirSim server (e.g., Windows AirSim from Docker)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py host_ip:=172.28.240.1 host_port:=41451
```

### Discovery and Monitoring
```bash
# Check discovered nodes (ultra-clean naming)
ros2 node list
# Expected output: /Droan1, /PX4_Drone2, /airsim_coordination_node

# Check ultra-clean topic structure  
ros2 topic list | head -10
# Expected: /Droan1/odom_local_ned, /Droan1/global_gps, /PX4_Drone2/imu, etc.

# Monitor real vehicle data
ros2 topic echo /Droan1/odom_local_ned --once
ros2 topic echo /PX4_Drone2/imu --once
```

### Vehicle Control
```bash
# Individual vehicle control (ultra-clean service names)
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
ros2 service call /PX4_Drone2/land airsim_interfaces/srv/Land "{wait_on_last_task: true}"

# Global coordination services
ros2 service call /airsim_coordination_node/takeoff_all airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
ros2 service call /airsim_coordination_node/land_all airsim_interfaces/srv/Land "{wait_on_last_task: true}"
ros2 service call /airsim_coordination_node/health_check airsim_interfaces/srv/ListSceneObjectTags

# Send velocity commands to specific vehicles
ros2 topic pub /Droan1/vel_cmd_body_frame geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}"
```

### System Status
```bash
# Check all services for a vehicle
ros2 service list | grep Droan1

# Monitor coordination status
ros2 topic echo /airsim_coordination_node/system_status --once

# Node health check
ros2 node info /Droan1
ros2 node info /airsim_coordination_node
```
>>>>>>> main

---
