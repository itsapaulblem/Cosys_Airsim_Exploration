# Architecture Comparison: AirStack vs Cosys-AirSim ROS2 Stack

## Executive Summary

This document provides an in-depth technical comparison between **CMU AirLab's AirStack** autonomy framework and your **Cosys-AirSim ROS2 Multi-Vehicle Architecture**. Both systems are production-grade, modular, and built on ROS2 Humble, but they serve different primary purposes:

- **AirStack**: Autonomy-first platform for real-world deployment
- **Cosys-AirSim**: Simulation-first platform for algorithm research and testing

---

## Table of Contents

1. [Cosys-AirSim Current Stack](#part-a-cosys-airsim-current-stack)
2. [AirStack Autonomy Stack](#part-b-airstack-autonomy-stack)
3. [Side-by-Side Comparison](#part-c-side-by-side-comparison)
4. [Integration Mapping](#part-d-integration-mapping)

---

# PART A: Cosys-AirSim Current Stack

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│                      COSYS-AIRSIM STACK                            │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 7: MISSION COORDINATION                                     │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Mission Coordinator Node (/mission_coordinator)             │ │
│  │  - ExecuteMission action server                              │ │
│  │  - PlanMission service                                        │ │
│  │  - Fleet-wide mission orchestration                          │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 6: AI PERCEPTION & TRACKING                                 │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Motion Detection Nodes (per vehicle)                        │ │
│  │  - YOLOv7 object detection                                   │ │
│  │  - DeepSORT multi-object tracking                            │ │
│  │  - BoxMOT extended tracking                                  │ │
│  │  - Multi-camera fusion (4 cameras per drone)                 │ │
│  │  - Target detection publishing                               │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 5: VEHICLE ACTIONS                                          │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Per-Vehicle Action Servers                                  │ │
│  │  - SearchArea (autonomous area coverage)                     │ │
│  │  - NavigateToTarget (precision waypoint nav)                 │ │
│  │  - TrackTarget (dynamic target following)                    │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 4: GLOBAL COORDINATION                                      │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Coordination Node (/airsim_coordination_node)               │ │
│  │  - Global services (/takeoff_all, /land_all, /reset_all)    │ │
│  │  - System health monitoring                                  │ │
│  │  - GPS origin publishing                                     │ │
│  │  - Simulation pause control                                  │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 3: LOCALIZATION & TRANSFORMS                                │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  REP 105 Compliant Transform Chain                           │ │
│  │  map → vehicle/odom → vehicle/base_link → sensors           │ │
│  │  - GPS-based spawn offset calculation                        │ │
│  │  - Per-vehicle frame separation                              │ │
│  │  - NED ↔ ENU coordinate transforms                           │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 2: VEHICLE NODES (ULTRA-CLEAN ARCHITECTURE)                │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Per-Vehicle Nodes (/Droan1, /PX4_Drone2, /VehicleName)     │ │
│  │                                                               │ │
│  │  Multirotor Node (inherits VehicleNodeBase)                  │ │
│  │  - Individual RPC connection to AirSim                       │ │
│  │  - Sensor data publishers:                                   │ │
│  │    * Odometry (/VehicleName/odom_local_ned)                  │ │
│  │    * GPS (/VehicleName/global_gps)                           │ │
│  │    * IMU (/VehicleName/imu)                                  │ │
│  │    * Cameras (/VehicleName/camera0/image, etc.)              │ │
│  │    * LiDAR (/VehicleName/lidar/points)                       │ │
│  │    * Magnetometer (/VehicleName/mag)                         │ │
│  │    * Barometer (/VehicleName/baro)                           │ │
│  │  - Vehicle services:                                          │ │
│  │    * /VehicleName/takeoff                                     │ │
│  │    * /VehicleName/land                                        │ │
│  │    * /VehicleName/reset                                       │ │
│  │  - Command subscribers:                                       │ │
│  │    * /VehicleName/vel_cmd_body_frame                         │ │
│  │    * /VehicleName/vel_cmd_world_frame                        │ │
│  │  - Parallel sensor processing (callback groups)              │ │
│  │  - Fault isolation (independent nodes)                       │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 1: RPC DYNAMIC DISCOVERY                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Vehicle Discovery System                                     │ │
│  │  - Direct RPC to AirSim server (TCP 41451)                   │ │
│  │  - Cross-platform (Windows AirSim + Docker ROS2)             │ │
│  │  - Multiple fallback methods:                                │ │
│  │    1. Direct msgpack-rpc                                     │ │
│  │    2. cosysairsim Python client                              │ │
│  │    3. airsim Python client                                   │ │
│  │    4. Settings.json parsing                                  │ │
│  │    5. Default vehicle creation                               │ │
│  │  - Zero manual configuration                                 │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 0: SIMULATION PLATFORM                                      │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Cosys-AirSim (Unreal Engine 5)                              │ │
│  │  - High-fidelity physics simulation                          │ │
│  │  - Sensor rendering (RGB, depth, segmentation, thermal)      │ │
│  │  - PX4 SITL integration (MAVLink UDP)                        │ │
│  │  - GPU LiDAR, echo sensors                                   │ │
│  │  - Dynamic object spawning                                   │ │
│  │  - Weather simulation                                        │ │
│  │  - 9+ drone ultra-swarm tested                               │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

## Component Breakdown

### 1. Vehicle Node Layer

**File**: `multirotor_node.cpp`, `vehicle_node_base.cpp`

**Responsibilities**:
- Per-vehicle ROS2 node lifecycle management
- AirSim RPC client connection (isolated per vehicle)
- Sensor data acquisition and publishing
- Command subscription and execution
- Service provisioning (takeoff, land, reset)

**Key Features**:
- **Ultra-Clean Naming**: Vehicle name IS node name (`/Droan1`, not `/namespace/Droan1`)
- **Explicit Topic Prefixing**: C++ code creates `/VehicleName/topic` structure
- **Fault Isolation**: Each vehicle crashes independently
- **Parallel Processing**: Independent callback groups per vehicle

**Topics Published** (per vehicle):
```yaml
/VehicleName/odom_local_ned:         nav_msgs/msg/Odometry
/VehicleName/global_gps:             sensor_msgs/msg/NavSatFix
/VehicleName/imu:                    sensor_msgs/msg/Imu
/VehicleName/mag:                    sensor_msgs/msg/MagneticField
/VehicleName/baro:                   sensor_msgs/msg/Range
/VehicleName/environment:            airsim_interfaces/msg/Environment
/VehicleName/camera0/image:          sensor_msgs/msg/Image
/VehicleName/camera0/camera_info:    sensor_msgs/msg/CameraInfo
/VehicleName/lidar/points:           sensor_msgs/msg/PointCloud2
```

**Services Provided** (per vehicle):
```yaml
/VehicleName/takeoff:     airsim_interfaces/srv/Takeoff
/VehicleName/land:        airsim_interfaces/srv/Land
/VehicleName/reset:       airsim_interfaces/srv/Reset
```

**Command Subscribers** (per vehicle):
```yaml
/VehicleName/vel_cmd_body_frame:   airsim_interfaces/msg/VelCmd
/VehicleName/vel_cmd_world_frame:  airsim_interfaces/msg/VelCmd
```

---

### 2. Coordination Node

**File**: `coordination_node.cpp`

**Responsibilities**:
- Fleet-wide command execution
- System health monitoring
- GPS origin broadcasting
- Simulation control

**Services Provided**:
```yaml
/airsim_coordination_node/takeoff_all:        airsim_interfaces/srv/Takeoff
/airsim_coordination_node/land_all:           airsim_interfaces/srv/Land
/airsim_coordination_node/reset_all:          airsim_interfaces/srv/Reset
/airsim_coordination_node/pause_simulation:   std_srvs/srv/SetBool
/airsim_coordination_node/health_check:       airsim_interfaces/srv/ListSceneObjectTags
```

**Topics Published**:
```yaml
/airsim_coordination_node/origin_geo_point:  airsim_interfaces/msg/GPSYaw
/airsim_coordination_node/system_status:     airsim_interfaces/msg/StringArray
/airsim_coordination_node/clock:             rosgraph_msgs/msg/Clock
```

---

### 3. AI Perception System

**File**: `motion_detection_node.py`, `generalised_object_tracking_node.py`

**Components**:
- **YOLOv7**: Real-time object detection
- **DeepSORT**: Multi-object tracking with re-identification
- **BoxMOT**: Extended tracking capabilities
- **Multi-Camera Fusion**: 4-camera (front, right, back, left) integration

**Capabilities**:
- Person detection and tracking
- Moving object classification
- Multi-target tracking
- Target selection logic
- PID control for target following

**Topics Published**:
```yaml
/target_detection:       mission_search_interfaces/msg/TargetDetection
/target_tracking:        mission_search_interfaces/msg/TargetTracking
```

**AI Models**:
- YOLOv7 weights: `yolov7.pt`
- DeepSORT checkpoint: `ckpt.t7`
- Fallback: OpenCV motion detection

---

### 4. Mission Coordination

**File**: `mission_coordination_node.cpp`, `mission_multirotor_node.cpp`

**Actions Provided** (per vehicle):
```yaml
/VehicleName/actions/search_area:       mission_search_interfaces/action/SearchArea
/VehicleName/actions/navigate_to_target: mission_search_interfaces/action/NavigateToTarget
/VehicleName/actions/track_target:      mission_search_interfaces/action/TrackTarget
```

**Services Provided**:
```yaml
/mission_coordinator/plan_mission:            mission_search_interfaces/srv/PlanMission
/mission_coordinator/get_mission_status:      mission_search_interfaces/srv/GetMissionStatus
/mission_coordinator/assign_search_zone:      mission_search_interfaces/srv/AssignSearchZone
```

**Mission Types**:
- Grid search patterns
- Spiral search patterns
- Waypoint navigation
- Target tracking
- Formation flight

---

### 5. Localization & Transforms

**File**: `localization_node.cpp`, `multirotor_node.cpp` (lines 440-491)

**REP 105 Transform Chain**:
```
map → Drone1/odom → Drone1/base_link → Drone1/camera0
                                       → Drone1/lidar
```

**Key Implementation Details**:
- **GPS-Based Spawn Offset**: Calculates vehicle separation from GPS coordinate differences
- **Per-Vehicle Odom Frames**: Each vehicle has independent odom frame
- **Frame Authority Separation**:
  - Odometry sources publish `odom → base_link`
  - Localization sources publish `map → odom`
- **Coordinate Transforms**: NED (AirSim) ↔ ENU (ROS2) handled automatically

**Code Reference** (`multirotor_node.cpp:440-491`):
```cpp
// GPS-based spawn offset calculation for proper multi-vehicle frame separation
Vector3r spawn_offset = gps_to_ned(vehicle_gps) - gps_to_ned(origin_gps);
base_link_pos = current_pos - spawn_offset;  // Relative to vehicle's odom frame
```

---

### 6. RPC Dynamic Discovery

**File**: `rpc_dynamic_vehicles.launch.py`, vehicle node constructors

**Discovery Methods** (in priority order):
1. **Direct RPC** (msgpack-rpc over TCP)
   - No Python client dependency
   - Raw socket communication
   - Works across Windows-Docker bridge

2. **cosysairsim Client**
   ```python
   import cosysairsim
   client = cosysairsim.MultirotorClient()
   vehicles = client.listVehicles()
   ```

3. **airsim Client**
   ```python
   import airsim
   client = airsim.MultirotorClient()
   # (less reliable for Cosys-AirSim features)
   ```

4. **Settings Fallback**
   - Parses `settings.json` if RPC fails
   - Manual vehicle configuration

5. **Default Vehicle**
   - Creates single default if all else fails

**Cross-Platform Support**:
```
Windows AirSim (172.28.240.1:41451)
         ↓ TCP
Docker ROS2 (airsim-ecosystem network)
         ↓ RPC Query
Response: ["Droan1", "PX4_Drone2", "Drone3"]
         ↓
Launch: /Droan1, /PX4_Drone2, /Drone3 nodes
```

---

## Deployment Architecture

### Docker Compose Structure

```yaml
# docker/airsim_ros2_wrapper/Linux/docker-compose.yml
services:
  ros2-x11-node:
    build: ./docker/airsim_ros2_wrapper/Linux/Dockerfile.x11
    container_name: ros2-x11-node
    networks:
      - airsim-ecosystem  # Cross-platform network
    environment:
      - AIRSIM_HOST_IP=172.28.240.1  # Windows AirSim IP
      - AIRSIM_HOST_PORT=41451
      - LAUNCH_MODE=multi  # Ultra-clean architecture
      - ENABLE_COORDINATION=true
    volumes:
      - ${PROJECT_ROOT}/ros2/src:/airsim_ros2_ws/src
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu, compute, utility, graphics]
```

### Network Architecture

```
┌─────────────────────────────────────────────┐
│  Windows Host                                │
│  ┌───────────────────────────────────────┐  │
│  │  AirSim (Unreal Engine 5)             │  │
│  │  Listens: 0.0.0.0:41451 (RPC)         │  │
│  │           14540-14550 (MAVLink)       │  │
│  └───────────────────────────────────────┘  │
└─────────────────┬───────────────────────────┘
                  │ 172.28.240.1 (WSL2 bridge)
┌─────────────────▼───────────────────────────┐
│  Docker (WSL2)                              │
│  ┌─────────────────────────────────────┐   │
│  │  ros2-x11-node                       │   │
│  │  - RPC client → 172.28.240.1:41451  │   │
│  │  - ROS2 publishers                   │   │
│  │  - MAVROS (optional)                 │   │
│  └─────────────────────────────────────┘   │
│                                              │
│  ┌─────────────────────────────────────┐   │
│  │  PX4 SITL Containers                 │   │
│  │  - px4-sitl-1 (14540/14557)          │   │
│  │  - px4-sitl-2 (14541/14558)          │   │
│  │  - ...                                │   │
│  └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

---

## Key Design Principles

### 1. Ultra-Clean Naming Convention
**Problem**: Legacy ROS often creates messy namespaces like `/namespace/vehicle/sensor/data`

**Solution**: Vehicle names ARE node names, topics prefixed in code:
```cpp
// In multirotor_node.cpp
std::string topic_name = vehicle_name_ + "/odom_local_ned";
odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
```

**Result**: Clean, intuitive structure
```
/Droan1                          # Node
/Droan1/odom_local_ned          # Topic
/Droan1/takeoff                 # Service
```

### 2. RPC Dynamic Discovery
**Problem**: Manual vehicle configuration is error-prone

**Solution**: Query running AirSim at launch time
```python
# rpc_dynamic_vehicles.launch.py
vehicles = discover_vehicles(host_ip, host_port)  # Returns ["Droan1", "PX4_Drone2"]
for vehicle in vehicles:
    launch_description.add_action(Node(
        package='airsim_ros_pkgs',
        executable='multirotor_node',
        name=vehicle,  # Vehicle name IS node name
        ...
    ))
```

### 3. Fault Isolation
**Problem**: Single-node architectures crash entirely if one vehicle fails

**Solution**: Independent nodes per vehicle
- Each vehicle has own RPC connection
- Separate callback groups
- Individual crash domains
- Independent logging

### 4. REP 105 Multi-Robot Compliance
**Problem**: AirSim spawns all vehicles at local [0, 0, 0] with different GPS

**Solution**: GPS-based spawn offset for proper frame separation
```cpp
// Each vehicle calculates its offset from origin
Vector3r spawn_offset = gps_to_ned(vehicle_gps) - gps_to_ned(origin_gps);

// base_link is relative to vehicle's own odom frame
base_link_pos = current_airsim_pos - spawn_offset;

// Publish map → odom transform (localization authority)
tf_map_to_odom.transform.translation = spawn_offset_to_vector3(spawn_offset);
```

**Result**: Proper multi-robot visualization in RViz with frame separation

---

## Strengths & Weaknesses

### ✅ Strengths

1. **High-Fidelity Simulation**: Unreal Engine 5 physics and rendering
2. **Ultra-Clean Architecture**: Intuitive naming, modern ROS2 patterns
3. **Cross-Platform**: Windows AirSim + Docker ROS2 seamless integration
4. **AI Vision**: State-of-the-art YOLOv7 + DeepSORT
5. **REP 105 Compliance**: Proper multi-robot coordinate frames
6. **Fault Isolation**: Per-vehicle nodes for robustness
7. **Dynamic Discovery**: Zero-configuration vehicle launching
8. **Scalability**: Tested with 9+ drones
9. **PX4 Integration**: Standard autopilot, well-documented

### ⚠️ Weaknesses

1. **No Advanced Planning**: Lacks trajectory optimization, obstacle avoidance algorithms
2. **No Behavior Trees**: Action-based missions less flexible than BT framework
3. **Limited SLAM**: No mapping or localization algorithms
4. **Manual Tuning**: PID controllers for target following require parameter tuning
5. **Simulation-Centric**: Not designed for real-world deployment
6. **No GCS**: No ground control station integration

---

# PART B: AirStack Autonomy Stack

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│                        AIRSTACK FRAMEWORK                          │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 5: BEHAVIOR                                                 │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Behavior Executive                                           │ │
│  │  - BehaviorTree.CPP framework                                │ │
│  │  - High-level mission planning                               │ │
│  │  - State machine management                                  │ │
│  │  - Mission sequencing                                        │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Behavior Trees                                               │ │
│  │  - Condition nodes                                           │ │
│  │  - Action nodes                                              │ │
│  │  - Control flow (sequence, fallback, parallel)               │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 4: GLOBAL PLANNING                                          │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Global Planner                                               │ │
│  │  - Long-range path planning                                  │ │
│  │  - Environment mapping                                       │ │
│  │  - Multi-waypoint optimization                               │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  World Model                                                  │ │
│  │  - Occupancy grids                                           │ │
│  │  - Costmaps                                                  │ │
│  │  - Static/dynamic obstacles                                 │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 3: LOCAL PLANNING                                           │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Trajectory Planner                                           │ │
│  │  - Real-time trajectory generation                           │ │
│  │  - Obstacle avoidance                                        │ │
│  │  - Dynamic constraints                                       │ │
│  │  - Smooth trajectory optimization                            │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Controls                                                     │ │
│  │  - PID/MPC controllers                                       │ │
│  │  - Trajectory tracking                                       │ │
│  │  - Setpoint generation                                       │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 2: PERCEPTION                                               │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  State Estimation                                             │ │
│  │  - Visual-Inertial Odometry (VIO)                            │ │
│  │  - SLAM (Simultaneous Localization and Mapping)              │ │
│  │  - Sensor fusion                                             │ │
│  │  - Pose estimation                                           │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Object Detection                                             │ │
│  │  - Vision-based detection                                    │ │
│  │  - Obstacle classification                                   │ │
│  │  - Tracking                                                  │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 1: SENSORS                                                  │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Sensor Interfaces                                            │ │
│  │  - Camera parameter server                                   │ │
│  │  - Gimbal stabilizer                                         │ │
│  │  - Sensor drivers                                            │ │
│  │  - Data preprocessing                                        │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  LAYER 0: INTERFACE                                                │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  MAVROS Interface                                             │ │
│  │  - MAVLink communication                                     │ │
│  │  - Command translation                                       │ │
│  │  - State monitoring                                          │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Robot Interface                                              │ │
│  │  - Vehicle abstraction                                       │ │
│  │  - Command interface                                         │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Drone Safety Monitor                                         │ │
│  │  - Failsafe conditions                                       │ │
│  │  - Emergency procedures                                      │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  GROUND CONTROL STATION                                            │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  - Multi-robot monitoring                                     │ │
│  │  - Mission planning UI                                        │ │
│  │  - TAK integration                                           │ │
│  │  - Real-time telemetry                                       │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────┐
│  SIMULATION (Primary: Isaac Sim)                                   │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  - Isaac Sim (NVIDIA)                                         │ │
│  │  - Gazebo                                                     │ │
│  │  - SimpleSim                                                 │ │
│  │  - Ascent SITL autopilot                                     │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

## Component Breakdown

### Layer 0: Interface

**Packages**:
- `drone_safety_monitor`
- `interface_bringup`
- `mavros_interface`
- `robot_interface`

**Responsibilities**:
- Abstract vehicle control via MAVROS
- Safety monitoring and failsafes
- Command translation (high-level → MAVLink)
- State aggregation

**MAVROS Topics** (expected):
```yaml
# Subscribers (commands TO vehicle)
/mavros/setpoint_position/local:     geometry_msgs/msg/PoseStamped
/mavros/setpoint_velocity/cmd_vel:   geometry_msgs/msg/Twist
/mavros/setpoint_attitude/attitude:  geometry_msgs/msg/PoseStamped

# Publishers (state FROM vehicle)
/mavros/local_position/pose:         geometry_msgs/msg/PoseStamped
/mavros/local_position/velocity:     geometry_msgs/msg/TwistStamped
/mavros/state:                       mavros_msgs/msg/State
/mavros/imu/data:                    sensor_msgs/msg/Imu
/mavros/global_position/global:      sensor_msgs/msg/NavSatFix
```

**Services**:
```yaml
/mavros/cmd/arming:       mavros_msgs/srv/CommandBool
/mavros/cmd/takeoff:      mavros_msgs/srv/CommandTOL
/mavros/cmd/land:         mavros_msgs/srv/CommandTOL
/mavros/set_mode:         mavros_msgs/srv/SetMode
```

---

### Layer 1: Sensors

**Packages**:
- `camera_param_server`
- `gimbal_stabilizer`
- `sensor_interfaces`
- `sensors_bringup`

**Capabilities**:
- Camera calibration management
- Gimbal control for stabilized imagery
- Sensor driver abstractions
- Data preprocessing and filtering

**Expected Topics**:
```yaml
/camera/image_raw:           sensor_msgs/msg/Image
/camera/camera_info:         sensor_msgs/msg/CameraInfo
/lidar/points:               sensor_msgs/msg/PointCloud2
/imu/data:                   sensor_msgs/msg/Imu
```

---

### Layer 2: Perception

**Components**:
- **State Estimation**: VIO, SLAM, sensor fusion
- **Object Detection**: Vision-based obstacle detection

**Algorithms** (inferred from CMU research):
- Visual-Inertial Odometry for pose estimation
- LiDAR-based SLAM
- Occupancy grid mapping
- Obstacle detection and tracking

**Topics** (typical):
```yaml
/perception/odometry:          nav_msgs/msg/Odometry
/perception/obstacles:         sensor_msgs/msg/PointCloud2
/perception/detected_objects:  vision_msgs/msg/Detection3DArray
```

---

### Layer 3: Local Planning

**Components**:
- **Trajectory Planner**: Real-time smooth trajectory generation
- **Obstacle Avoidance**: Dynamic replanning
- **Controls**: Trajectory tracking controllers

**Algorithms**:
- Model Predictive Control (MPC)
- RRT* variants for path planning
- Dynamic constraints handling

**Topics** (typical):
```yaml
/local_planner/trajectory:     nav_msgs/msg/Path
/local_planner/cmd_vel:        geometry_msgs/msg/Twist
```

---

### Layer 4: Global Planning

**Components**:
- **Global Planner**: Long-range path planning
- **World Model**: Global map representation

**Capabilities**:
- Multi-waypoint optimization
- Elevation-aware planning
- No-fly zone enforcement

**Topics** (typical):
```yaml
/global_planner/path:          nav_msgs/msg/Path
/global_planner/costmap:       nav_msgs/msg/OccupancyGrid
```

---

### Layer 5: Behavior

**Component**: Behavior Executive using BehaviorTree.CPP

**Behavior Tree Nodes**:
- **Condition Nodes**: Check robot state, mission state
- **Action Nodes**: Execute tasks (takeoff, navigate, search)
- **Control Nodes**: Sequence, Fallback, Parallel execution

**Example Behavior Tree**:
```xml
<BehaviorTree ID="SearchMission">
  <Sequence>
    <Action ID="Takeoff" height="10.0"/>
    <Action ID="NavigateToSearchArea" waypoint="area1"/>
    <Fallback>
      <Action ID="SearchPattern" pattern="spiral" duration="300"/>
      <Action ID="ReturnToHome"/>
    </Fallback>
    <Action ID="Land"/>
  </Sequence>
</BehaviorTree>
```

---

### Ground Control Station

**Features**:
- Multi-robot monitoring dashboard
- Mission planning interface
- TAK (Team Awareness Kit) integration
- Real-time telemetry visualization
- Operator-in-the-loop control

---

### Simulation Integration

**Primary**: NVIDIA Isaac Sim
- Photorealistic rendering
- GPU-accelerated physics
- ROS2 native bridges
- Ascent SITL integration

**Alternatives**:
- Gazebo (ROS standard)
- SimpleSim (lightweight)

**Autopilot**: Ascent SITL (proprietary CMU)
- Custom flight modes
- Research-oriented features
- Not open-source PX4

---

## Deployment Architecture

### Docker Structure

```yaml
# AirStack docker-compose.yml (simplified)
version: "3.8"

services:
  # Autonomy container
  autonomy:
    image: airlab-registry.andrew.cmu.edu/airstack:latest
    networks:
      - autonomy_net
    environment:
      - ROS_DOMAIN_ID=0
      - VEHICLE_NAMESPACE=/uav1
    volumes:
      - ./robot/ros_ws:/ros2_ws

  # Ground control station
  gcs:
    image: airlab-registry.andrew.cmu.edu/airstack-gcs:latest
    ports:
      - "8080:8080"  # Web interface

  # Simulation (Isaac Sim)
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:latest
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              capabilities: [gpu]
```

---

## Key Design Principles

### 1. Layered Autonomy
**Philosophy**: Each layer has clear inputs/outputs, enabling modularity

### 2. Production-Ready
**Focus**: Real-world deployment, not just simulation

### 3. MAVROS Standardization
**Benefit**: Works with any MAVROS-compatible autopilot

### 4. Behavior Trees
**Advantage**: Complex mission logic, reusable components

---

## Strengths & Weaknesses

### ✅ Strengths

1. **Production-Tested**: Used in real-world CMU deployments
2. **Advanced Algorithms**: SLAM, VIO, trajectory optimization
3. **Behavior Trees**: Flexible mission framework
4. **Ground Control Station**: Operator interface
5. **Multi-Robot**: Fleet coordination built-in
6. **MAVROS Standard**: Works with many autopilots
7. **TAK Integration**: Military/emergency response systems
8. **Modular**: Easy to swap components

### ⚠️ Weaknesses

1. **Simulator Dependency**: Designed for Isaac Sim (not AirSim)
2. **Ascent SITL**: Non-standard autopilot
3. **Access Restrictions**: ALPHA, internal CMU use
4. **No Open Perception Models**: No pre-trained YOLOv7/similar
5. **Learning Curve**: Behavior trees require training
6. **Complexity**: Full stack is heavyweight
7. **Documentation**: Limited public documentation

---

# PART C: Side-by-Side Comparison

## Architectural Comparison

| Layer/Component | Cosys-AirSim Stack | AirStack | Compatibility | Integration Effort |
|-----------------|-------------------|----------|--------------|-------------------|
| **Simulation** | Cosys-AirSim (UE5) | Isaac Sim | ❌ Incompatible | Bypass (treat as hardware) |
| **Autopilot** | PX4 SITL | Ascent SITL | ⚠️ Partial | MAVROS abstraction (medium) |
| **Vehicle Interface** | ROS2 direct | MAVROS | ✅ Compatible | Add MAVROS (low) |
| **Sensors** | ROS2 topics | ROS2 topics | ✅ Compatible | Topic remapping (low) |
| **Perception** | YOLOv7+DeepSORT | VIO+SLAM | ⚠️ Different | Replace or hybrid (medium) |
| **Local Planning** | PID control | Trajectory opt | ❌ Different | Replace (medium-high) |
| **Global Planning** | None | Path planner | ➕ Addition | Integrate (medium) |
| **Behavior** | ROS2 Actions | Behavior Trees | ⚠️ Different | Port actions (high) |
| **Coordination** | Coordination node | Fleet manager | ⚠️ Different | Choose primary (medium) |
| **Ground Control** | None | GCS+TAK | ➕ Addition | Integrate (low) |

**Legend**:
- ✅ **Compatible**: Direct compatibility
- ⚠️ **Partial**: Compatible with adaptation
- ❌ **Incompatible**: Major differences
- ➕ **Addition**: New capability

---

## Feature Comparison Matrix

| Feature | Cosys-AirSim | AirStack | Winner |
|---------|--------------|----------|--------|
| **Simulation Fidelity** | ⭐⭐⭐⭐⭐ UE5 | ⭐⭐⭐⭐ Isaac Sim | Cosys-AirSim |
| **Multi-Robot Scale** | ⭐⭐⭐⭐ (9+ drones) | ⭐⭐⭐⭐⭐ (Fleet tested) | AirStack |
| **AI Perception** | ⭐⭐⭐⭐⭐ YOLOv7 | ⭐⭐⭐ Unspecified | Cosys-AirSim |
| **Path Planning** | ⭐ PID only | ⭐⭐⭐⭐⭐ Advanced | AirStack |
| **Obstacle Avoidance** | ⚠️ Manual | ⭐⭐⭐⭐⭐ Dynamic | AirStack |
| **SLAM/Mapping** | ❌ None | ⭐⭐⭐⭐⭐ VIO+SLAM | AirStack |
| **Mission Framework** | ⭐⭐⭐ Actions | ⭐⭐⭐⭐⭐ Behavior Trees | AirStack |
| **Real-World Deploy** | ⭐ Sim-focused | ⭐⭐⭐⭐⭐ Production | AirStack |
| **Ease of Use** | ⭐⭐⭐⭐ Simple | ⭐⭐ Complex | Cosys-AirSim |
| **Documentation** | ⭐⭐⭐⭐ Good | ⭐⭐ Limited | Cosys-AirSim |
| **Cross-Platform** | ⭐⭐⭐⭐⭐ Windows+Docker | ⭐⭐⭐ Linux | Cosys-AirSim |
| **Open Source** | ✅ Full | ⚠️ Partial (ALPHA) | Cosys-AirSim |

---

## Technology Stack Comparison

| Technology | Cosys-AirSim | AirStack | Notes |
|------------|--------------|----------|-------|
| **ROS Version** | ROS2 Humble | ROS2 Humble | ✅ Same |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 | ✅ Same |
| **Programming** | C++17, Python 3 | C++, Python 3 | ✅ Compatible |
| **Middleware** | DDS (Fast-RTPS) | DDS | ✅ Same |
| **Build System** | CMake, Colcon | CMake, Colcon | ✅ Same |
| **Simulator** | AirSim (UE5) | Isaac Sim | ❌ Different |
| **Physics Engine** | PhysX | Isaac Physics | ❌ Different |
| **Autopilot** | PX4 | Ascent | ⚠️ Different |
| **Vehicle Control** | Direct RPC | MAVROS | ⚠️ Add MAVROS |
| **AI Framework** | PyTorch (YOLO) | ? | ⚠️ Unknown |
| **Planning Lib** | Custom | ? | ❌ Different |
| **Behavior Lib** | Custom Actions | BehaviorTree.CPP | ❌ Different |
| **Transform Lib** | tf2 | tf2 | ✅ Same |
| **Visualization** | RViz2 | RViz2 | ✅ Same |
| **Docker** | Docker Compose | Docker Compose | ✅ Same |
| **GPU Support** | NVIDIA CUDA | NVIDIA CUDA | ✅ Same |

---

# PART D: Integration Mapping

## Component Mapping

### 1. Vehicle Interface Layer

**Cosys-AirSim**: Direct RPC to AirSim
```python
import cosysairsim
client = cosysairsim.MultirotorClient()
client.takeoffAsync().join()
```

**AirStack**: MAVROS abstraction
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL
```

**Integration**:
```
AirStack MAVROS Commands
         ↓
    MAVROS Node (bridge)
         ↓
    PX4 SITL (MAVLink)
         ↓
    AirSim Physics
```

**Effort**: LOW (1-2 weeks)

---

### 2. Sensor Data Flow

**Cosys-AirSim**: Per-vehicle topics
```
/Droan1/camera/image → sensor_msgs/Image
/Droan1/lidar/points → sensor_msgs/PointCloud2
```

**AirStack**: Generic sensor topics
```
/camera/image_raw → sensor_msgs/Image
/lidar/points → sensor_msgs/PointCloud2
```

**Integration**: Topic remapping
```yaml
# topic_remap.yaml
remappings:
  - /camera/image_raw: /Droan1/camera/image
  - /lidar/points: /Droan1/lidar/points
```

**Effort**: LOW (topic remapping config)

---

### 3. Perception

**Cosys-AirSim**: YOLOv7 + DeepSORT
```python
detections = yolo_model(image)  # Real-time object detection
tracks = deepsort.update(detections)  # Multi-object tracking
```

**AirStack**: VIO + SLAM
```
Vision + IMU → VIO → Pose estimate
LiDAR + Pose → SLAM → Map
```

**Integration Options**:
- **Option A**: Keep YOLOv7, add SLAM from AirStack
- **Option B**: Replace with AirStack perception entirely
- **Option C**: Run both, publish to different topics

**Effort**: MEDIUM (2-4 weeks)

---

### 4. Planning

**Cosys-AirSim**: PID position control
```cpp
// Simple proportional control
vel_x = Kp * (target_x - current_x);
```

**AirStack**: Trajectory optimization
```
Current pose + Goal + Obstacles → Optimized trajectory
```

**Integration**:
```
AirStack Planner → Trajectory
         ↓
    MAVROS setpoints
         ↓
    Your system executes
```

**Effort**: MEDIUM (2-3 weeks)

---

### 5. Mission Logic

**Cosys-AirSim**: ROS2 Actions
```yaml
# SearchArea action
ros2 action send_goal /Droan1/actions/search_area mission_search_interfaces/action/SearchArea "{...}"
```

**AirStack**: Behavior Trees
```xml
<BehaviorTree>
  <Sequence>
    <Action ID="SearchArea"/>
    <Action ID="TrackTarget"/>
  </Sequence>
</BehaviorTree>
```

**Integration**: Port actions to BT nodes
```cpp
// Behavior tree action node
class SearchAreaAction : public BT::StatefulActionNode {
  BT::NodeStatus onStart() override {
    // Call your ROS2 action server
  }
};
```

**Effort**: HIGH (3-4 weeks)

---

## Integration Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                        INTEGRATED SYSTEM                          │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  AirStack Container (Autonomy Brain)                              │
│                                                                   │
│  Behavior Trees → Global Planner → Local Planner                 │
│       ↓               ↓                ↓                          │
│  MAVROS Commands:                                                │
│  - /mavros/setpoint_position/local                               │
│  - /mavros/setpoint_velocity/cmd_vel                             │
└────────────────────────┬─────────────────────────────────────────┘
                         │
                         │ ROS2 Topics (DDS)
                         │
┌────────────────────────▼─────────────────────────────────────────┐
│  Bridge Container (Topic Remapping + MAVROS)                     │
│                                                                   │
│  Topic Remapper:                                                 │
│  /camera/image_raw → /Droan1/camera/image                        │
│  /lidar/points → /Droan1/lidar/points                            │
│                                                                   │
│  MAVROS Node:                                                    │
│  MAVROS commands → MAVLink → PX4 SITL                            │
└────────────────────────┬─────────────────────────────────────────┘
                         │
                         │ MAVLink UDP + AirSim RPC
                         │
┌────────────────────────▼─────────────────────────────────────────┐
│  Your ROS2 Container (Sensor Layer)                              │
│                                                                   │
│  Per-Vehicle Nodes (/Droan1, /PX4_Drone2):                       │
│  - Sensor publishers                                             │
│  - REP 105 transforms                                            │
│  - AirSim RPC clients                                            │
│                                                                   │
│  Optional: Keep YOLOv7 perception                                │
└────────────────────────┬─────────────────────────────────────────┘
                         │
                         │ AirSim RPC (TCP 41451)
                         │ MAVLink (UDP 14540-14550)
                         │
┌────────────────────────▼─────────────────────────────────────────┐
│  AirSim (Unreal Engine 5) + PX4 SITL                             │
│  - Physics simulation                                             │
│  - Sensor rendering                                              │
└──────────────────────────────────────────────────────────────────┘
```

---

## Summary

### Best Integration Strategy

**Recommended Approach**: **Selective Integration**

1. **Keep from Your Stack**:
   - ✅ AirSim simulation (superior graphics)
   - ✅ Ultra-clean ROS2 architecture
   - ✅ YOLOv7 perception (proven performance)
   - ✅ REP 105 transforms
   - ✅ Cross-platform support

2. **Add from AirStack**:
   - ➕ MAVROS interface (standardization)
   - ➕ Trajectory planning (obstacle avoidance)
   - ➕ SLAM (mapping capabilities)
   - ➕ Behavior trees (mission flexibility)
   - ➕ Ground control station

3. **Bridge Layer** (custom development):
   - Topic remapping
   - MAVROS ↔ PX4 bridge
   - Coordinate frame conversions
   - Multi-robot namespace management

**Result**: Best of both worlds - AirSim's simulation quality with AirStack's autonomy algorithms.

---

**Document Version**: 1.0
**Last Updated**: 2025-10-16
**Status**: Draft for Review
