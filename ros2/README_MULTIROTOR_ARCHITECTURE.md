# AirSim ROS2 Multi-Vehicle Modular Architecture

## Table of Contents 
1. Introduction
2. Startup Instructions
3. Architecture Overview
4. ROS Topics and Services
5. File by File Explanation
6. Detailed Comparisons: Old vs New Architecture
7. Design Decisions
8. Troubleshooting & FAQ
9. References 

--- 

## 1. Introduction 

This documentation describes the **Cosys-AirSim ROS 2 multi-vehicle modular architecture**, designed for robust, scalable, and maintainable multi-drone simulation and control. It replaces the legacy monolithic ROS node with a modern, component-based approach, supporting parallel sensor processing, fault isolation and dynamic vehicle management.

---

## 2. Startup Instructions (wsl 2.5.10.0 & Windows 10)

### Step 1: Run the Python generate settings.py file to determine the number of drones
```bash
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

### Step 4: Build and Source ROS 2 Workspace

```bash
colcon build
source install/setup.bash 
```

### Step 5: Launch ROS2 Nodes

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
``` 

---

## 3. Architecture Overview

### Key Components

#### A. Vehicle Nodes

- **VehicleNodeBase**: Abstract base for all vehicle types. Handles parameter management, AirSim connections, publishers/services/timers and callback groups for parallel sensor processing. 
- **MultirotorNode**: Inherits from VehicleNodeBase. Implements drone-specific publishers (odom, GPS, IMU, environment, camera, lidar), services (takeoff, land, gps_waypoint, track_object), velocity command subscribers, and sensor data processing. 
- **SimpleMultirotorNode**: Minimal node for single-drone testing/debugging. No inheritance, direct AirSim connection.

#### B. Coordination Node

- **CoordinationNode**: Manages global services (reset all, takeoff all, land all, pause simulation, health check), publishes system status and GPS origin, monitors all vehicles.

#### C. Settings Parser 
- **VehicleSettingsParser**: Parses AirSim `settings.json` to extract vehicle configurations and global parameters, enabling dynamic node creation. 

--- 

## 4. ROS Topics and Services

### Topics Published Per Vehicle 

| Topic Name                | Message Type                       | Description                                      |
|---------------------------|------------------------------------|--------------------------------------------------|
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

### Services Per Vehicle

| Service Name              | Service Type                       | Functionality                                    |
|---------------------------|------------------------------------|--------------------------------------------------|
| `/droneX/takeoff`         | `airsim_interfaces/srv/Takeoff`    | Takeoff command for this vehicle                 |
| `/droneX/land`            | `airsim_interfaces/srv/Land`       | Land command for this vehicle                    |
| `/droneX/reset`           | `airsim_interfaces/srv/Reset`      | Reset this vehicle in AirSim                     |
| `/droneX/gps_waypoint`    | `airsim_interfaces/srv/GpsWaypoint`| Move to GPS waypoint                             |
| `/droneX/track_object`    | `airsim_interfaces/srv/TrackObject`| Search and track a named object (NEW)            |

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

## Example: Track Object Service

The new `/droneX/track_object` service allows a drone to search for and track a named object in the simulation.

**Service definition (`airsim_interfaces/srv/TrackObject.srv`):**
```plaintext
string object_name
float64 search_radius
float64 track_duration
---
bool success
string message
```

**Example usage:**
```bash
ros2 service call /drone1/track_object airsim_interfaces/srv/TrackObject "{object_name: 'TargetCar', search_radius: 50.0, track_duration: 30.0}"
```

---

## 5. File-by-File Explanation 

### New Modular Architecture Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `vehicle_node_base.hpp/cpp`       | Abstract base for all vehicle nodes. Handles parameters, connections, publishers, timers, callback groups.|
| `multirotor_node.hpp/cpp`         | Implements drone-specific logic: sensor publishers, command subscribers, takeoff/land/gps_waypoint/track_object services.         |
| `simple_multirotor_node.cpp`      | Minimal node for single-drone testing/debugging. Direct AirSim connection, basic publishers/services.   |
| `coordination_node.hpp/cpp`       | Global node for system-wide services, status monitoring, GPS origin publishing, health checks.          |
| `vehicle_settings_parser.hpp/cpp` | Parses AirSim `settings.json` for dynamic vehicle configuration. Used by launch files for node creation.|
| `multirotor_main.cpp`             | Main entry for launching a multirotor node (per vehicle).                                              |
| `coordination_main.cpp`           | Main entry for launching the coordination node.                                                        |

### Launch Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `simple_single_drone.launch.py`   | Launches a single `simple_multirotor_node` in `/drone1` namespace. For quick testing/debugging.         |
| `single_drone.launch.py`          | Launches two `multirotor_node` instances and the coordination node.                                     |
| `multi_drone.launch.py`           | Dynamically launches nodes for all vehicles in `settings.json`, plus the coordination node.             |
| `airsim_node.launch.py`           | Legacy: launches the old monolithic node.                                                               |

### Legacy (Old Architecture) Files

| File Name                        | Purpose / Contribution                                                                                   |
|-----------------------------------|--------------------------------------------------------------------------------------------------------|
| `airsim_ros_wrapper.h/cpp`        | Monolithic node managing all vehicles in one process. Single point of failure, sequential processing.   |
| `airsim_node.cpp`                 | Main for launching the old monolithic node.                                                             |

---

## 6. Detailed Comparison: Old vs. New Architecture

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

## 7. Design Decisions

- **Modularity:** Each vehicle type gets its own node class, making it easy to extend for new vehicle types (cars, drones, etc.).
- **Isolation:** Per-vehicle nodes mean a crash or RPC error in one vehicle does not affect others.
- **Parallelism:** Isolated callback groups and timers allow sensors to be processed in parallel, improving performance.
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

- [AirSim Documentation](https://microsoft.github.io/AirSim/)
- [ROS2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)

---

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
- **Move to GPS waypoint:**
  ```bash
  ros2 service call /drone1/gps_waypoint airsim_interfaces/srv/GpsWaypoint "{latitude: 47.641468, longitude: -122.140165, altitude: 10.0, speed: 5.0, tolerance: 1.0, wait_on_last_task: true}"
  ```
- **Track object (TODO):**
  ```bash
  ros2 service call /drone1/track_object airsim_interfaces/srv/TrackObject "{object_name: 'TargetCar', search_radius: 50.0, track_duration: 30.0}"
  ```
Last update on 20 Aug 2025
---