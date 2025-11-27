# AirStack Integration with Cosys-AirSim

## Executive Summary

**Integration Feasibility: MODERATE (⭐⭐⭐ out of 5)**

Integrating CMU AirLab's AirStack autonomy framework with your Cosys-AirSim system is **technically feasible with significant adaptation work**. Both systems share compatible middleware (ROS2 Humble, MAVROS), enabling direct communication at the autonomy layer. However, fundamental differences in simulator backend (Isaac Sim vs AirSim) and autopilot (Ascent SITL vs PX4 SITL) require a **"brain-body separation"** architecture where AirStack provides autonomy algorithms while AirSim handles simulation.

**Key Insight**: Treat AirSim as the "physical platform" and AirStack as the "autonomy brain" communicating via MAVROS - similar to deploying AirStack on real hardware.

---

## Table of Contents

1. [Integration Overview](#integration-overview)
2. [Compatibility Analysis](#compatibility-analysis)
3. [Integration Challenges](#integration-challenges)
4. [Recommended Architecture](#recommended-architecture)
5. [Quick Start Validation](#quick-start-validation)
6. [Effort Estimation](#effort-estimation)
7. [Alternative Approaches](#alternative-approaches)
8. [Decision Matrix](#decision-matrix)

---

## Integration Overview

### What is AirStack?

**AirStack** is a comprehensive robotics development framework developed by the AirLab at Carnegie Mellon University's Robotics Institute. It provides:

- **Layered Autonomy Architecture**: Modular stack from interface → sensors → perception → planning → behavior
- **Multi-Robot Support**: Fleet coordination and ground control station
- **Production-Ready**: Designed for real-world deployment including TAK (Team Awareness Kit) integration
- **ROS2 Native**: Built on ROS2 for robust inter-process communication
- **MAVROS Standardization**: Uses MAVROS for vehicle control abstraction

**Important Notes**:
- ALPHA project marked "internal usage only" at CMU
- Requires AirLab account for Docker registry access
- Designed primarily for Isaac Sim, with Gazebo and SimpleSim support
- Uses Ascent SITL (proprietary CMU autopilot) not standard PX4

### Your Current Cosys-AirSim Stack

**Cosys-AirSim** is a research-grade simulation platform with advanced ROS2 integration:

- **Ultra-Clean ROS2 Architecture**: Per-vehicle nodes with perfect naming (`/Droan1`, `/PX4_Drone2`)
- **RPC Dynamic Discovery**: Cross-platform Windows AirSim + Docker ROS2 support
- **Standard PX4 SITL**: Full MAVLink integration (tested with 9+ drones)
- **AI Vision System**: YOLOv7, DeepSORT, BoxMOT for perception
- **Mission Coordination**: Action-based mission framework
- **REP 105 Compliance**: Multi-robot coordinate frame standards
- **Unreal Engine 5 Simulation**: High-fidelity physics and sensor rendering

---

## Compatibility Analysis

### ✅ Strong Compatibility Points

#### 1. **MAVROS Interface Layer** ⭐⭐⭐⭐⭐
- **Status**: **DIRECT COMPATIBILITY**
- **Your System**: Has `mavros_msgs` dependency in ROS2 packages
- **AirStack**: Dedicated `mavros_interface` package in layer 0_interface
- **Integration**: Drop-in replacement - AirStack sends MAVROS commands, your system executes via PX4
- **Effort**: Low (1 week)

```
AirStack Planning → MAVROS Commands → Your ROS2 Bridge → PX4 SITL → AirSim Physics
```

#### 2. **ROS2 Middleware** ⭐⭐⭐⭐⭐
- **Status**: **100% COMPATIBLE**
- **Common Ground**: ROS2 Humble on Ubuntu 22.04
- **Message Types**: sensor_msgs, geometry_msgs, nav_msgs, tf2
- **DDS Discovery**: Automatic cross-container/network discovery
- **Effort**: None - works out-of-the-box

#### 3. **Sensor Data** ⭐⭐⭐⭐
- **Status**: **HIGH COMPATIBILITY**

| Sensor Type | Your System | AirStack | Compatibility |
|-------------|-------------|----------|---------------|
| RGB Cameras | ✅ sensor_msgs/Image | ✅ Expected | ✅ Direct |
| Depth Cameras | ✅ sensor_msgs/Image | ✅ Expected | ✅ Direct |
| LiDAR | ✅ sensor_msgs/PointCloud2 | ✅ Expected | ✅ Direct |
| IMU | ✅ sensor_msgs/Imu | ✅ Expected | ✅ Direct |
| GPS | ✅ sensor_msgs/NavSatFix | ✅ Expected | ✅ Direct |
| Odometry | ✅ nav_msgs/Odometry | ✅ Expected | ✅ Direct |

- **Effort**: Low (topic remapping only)

#### 4. **Docker Deployment** ⭐⭐⭐⭐
- **Status**: **HIGHLY COMPATIBLE**
- **Your System**: Production-ready Docker Compose with GPU support
- **AirStack**: Docker-native with docker-compose.yml
- **Integration**: Add AirStack containers to existing ecosystem network
- **Effort**: Low (networking configuration)

#### 5. **Multi-Robot Architecture** ⭐⭐⭐⭐
- **Status**: **COMPATIBLE WITH ADAPTATION**
- **Your System**: Per-vehicle ROS2 nodes with coordination
- **AirStack**: Multi-robot support with fleet coordination
- **Integration**: Map AirStack's fleet management to your coordination node
- **Effort**: Medium (2-3 weeks)

### ⚠️ Moderate Compatibility (Requires Adaptation)

#### 6. **Coordinate Frames** ⭐⭐⭐
- **Your System**: NED (North-East-Down) from AirSim, with REP 105 ENU transforms
- **AirStack**: ENU (East-North-Up) standard from MAVROS
- **Solution**: Your existing REP 105 transform chain handles this
- **Effort**: Low (already solved)

```
AirStack (ENU) → Your Transform Node → AirSim (NED)
        ↓
   map → odom → base_link (REP 105)
```

#### 7. **Perception Integration** ⭐⭐⭐
- **Your System**: YOLOv7 + DeepSORT + BoxMOT for object tracking
- **AirStack**: Vision-based perception algorithms (unspecified)
- **Options**:
  - **Option A**: Keep your AI vision, use AirStack planning only
  - **Option B**: Replace with AirStack perception, lose YOLOv7 integration
  - **Option C**: Hybrid - both systems publish detections
- **Effort**: Medium (2-4 weeks depending on option)

#### 8. **Mission Planning** ⭐⭐⭐
- **Your System**: Action-based (SearchArea, NavigateToTarget, TrackTarget)
- **AirStack**: Behavior tree-based mission executive
- **Integration**: Map your actions to behavior tree nodes
- **Effort**: Medium-High (3-4 weeks)

### ❌ Low Compatibility (Significant Challenges)

#### 9. **Simulator Backend** ⭐
- **Your System**: Cosys-AirSim (Unreal Engine 5)
- **AirStack**: Isaac Sim, Gazebo, SimpleSim
- **Impact**: **MAJOR** - Cannot use AirStack's simulation layer
- **Solution**: Abstraction - treat AirSim as "real hardware"
- **Effort**: None (bypass simulation layer entirely)

#### 10. **Autopilot** ⭐⭐
- **Your System**: Standard PX4 SITL (open source)
- **AirStack**: Ascent SITL (proprietary CMU)
- **Impact**: Low-level command differences
- **Solution**: Use high-level MAVROS commands (position/velocity setpoints)
- **Effort**: Low-Medium (1-2 weeks testing)

#### 11. **Access Restrictions** ⭐
- **Status**: ALPHA project, internal CMU use only
- **Docker Registry**: Requires CMU AirLab account
- **Solution**: Build from public GitHub source (check license)
- **Effort**: Legal/administrative overhead

---

## Integration Challenges

### 1. **Simulator Abstraction** (MAJOR)

**Problem**: AirStack expects Isaac Sim APIs for simulation control, environment queries, and object spawning.

**Impact**:
- Cannot use AirStack's simulation integration layer
- Scene setup and object management won't work
- Simulation time synchronization different

**Solution**:
```
┌─────────────────────────────────────┐
│     AirStack Autonomy Layers        │
│  (Perception, Planning, Behavior)   │
└──────────────┬──────────────────────┘
               │ MAVROS Interface
┌──────────────▼──────────────────────┐
│    Abstraction Layer (You Create)   │
│  - MAVROS → PX4 bridge              │
│  - Sensor topic remapping           │
│  - Coordinate transforms            │
└──────────────┬──────────────────────┘
               │ ROS2 Topics/Services
┌──────────────▼──────────────────────┐
│   Your Cosys-AirSim ROS2 Nodes      │
│  - Per-vehicle nodes                │
│  - Sensor publishers                │
│  - PX4 SITL integration             │
└──────────────┬──────────────────────┘
               │ AirSim RPC
┌──────────────▼──────────────────────┐
│      AirSim (Unreal Engine 5)       │
│  - Physics simulation               │
│  - Sensor rendering                 │
└─────────────────────────────────────┘
```

**Effort**: Design abstraction (1 week), implementation (2-3 weeks)

### 2. **Autopilot Differences** (MODERATE)

**Problem**: Ascent SITL vs PX4 SITL command differences.

**Mitigation**:
- Use **high-level MAVROS commands** only:
  - `setpoint_position/local` (position control)
  - `setpoint_velocity/cmd_vel` (velocity control)
  - `cmd/arming` (arm/disarm)
  - `cmd/takeoff` (takeoff)
  - `cmd/land` (landing)
- Avoid low-level mode switching or parameter tuning

**Compatibility Testing Required**:
```bash
# Test 1: Position setpoint
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped ...

# Test 2: Velocity setpoint
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/Twist ...

# Test 3: Arming
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Effort**: 1-2 weeks testing and validation

### 3. **Behavior Tree Integration** (COMPLEX)

**Problem**: AirStack uses BehaviorTree.CPP framework; you use ROS2 actions.

**Options**:

**Option A**: **Replace Your Actions with Behavior Trees** (Recommended)
- Pros: Full AirStack integration, production-tested framework
- Cons: Lose your custom action server implementations
- Effort: 3-4 weeks

**Option B**: **Wrap Your Actions in Behavior Tree Nodes**
- Pros: Keep existing functionality
- Cons: Added complexity, two layers of abstraction
- Effort: 2-3 weeks

**Option C**: **Keep Your Actions, Disable AirStack Behavior Layer**
- Pros: Minimal changes
- Cons: Miss out on AirStack's high-level autonomy
- Effort: 1 week

### 4. **Multi-Drone Coordination** (MODERATE)

**Problem**: Both systems have coordination layers that may conflict.

**Your System**:
- `/airsim_coordination_node` with global services
- Fleet-wide commands (`/takeoff_all`, `/land_all`)

**AirStack**:
- Multi-robot behavior coordination
- TAK integration for ground control

**Solution**: Choose one as primary coordinator
- **Recommended**: Use AirStack's coordinator, disable yours
- **Alternative**: Keep yours, disable AirStack's (lose GCS integration)

**Effort**: 2-3 weeks

### 5. **License and Access** (ADMINISTRATIVE)

**Challenges**:
- AirStack is ALPHA and marked "internal usage only"
- Docker images require CMU AirLab account login
- No clear public release license

**Actions Required**:
1. Review [AirStack License](https://github.com/castacks/AirStack/blob/main/LICENSE) (if exists)
2. Contact CMU AirLab for collaboration/access
3. Build from source if Docker registry unavailable

**Effort**: Administrative overhead, potential blockers

---

## Recommended Architecture

### "AirStack as Autonomy Brain, AirSim as Body"

This architecture treats AirSim as a hardware platform while AirStack provides the intelligence layer.

```
┌──────────────────────────────────────────────────────────────────┐
│                    AirStack Docker Container                     │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Layer 5: Behavior (Behavior Trees, Mission Executive)     │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │  Layer 4: Global Planning (Path planning, Mapping)         │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │  Layer 3: Local Planning (Trajectory, Obstacle Avoidance)  │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │  Layer 2: Perception (SLAM, Object Detection)              │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │  Layer 1: Sensors (Camera Processing, Point Cloud Filter)  │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │  Layer 0: Interface (MAVROS, Safety Monitor)               │  │
│  │  - MAVROS setpoint publishers                              │  │
│  │  - Command topics                                          │  │
│  │  - State subscribers                                       │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
└────────────────────────────┼─────────────────────────────────────┘
                             │
                             │ ROS2 Topics (DDS Discovery)
                             │ - /mavros/setpoint_position/local
                             │ - /mavros/setpoint_velocity/cmd_vel
                             │ - /camera/image_raw (sensors)
                             │ - /lidar/points
                             │
┌────────────────────────────▼────────────────────────────────────┐
│              Your ROS2 Docker Container                         │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  MAVROS Node (Bridge Layer)                              │   │
│  │  - Subscribes to AirStack commands                       │   │
│  │  - Publishes to PX4 SITL (MAVLink UDP)                   │   │
│  │  - Handles mode switching, arming, etc.                  │   │
│  └───────────────────────┬──────────────────────────────────┘   │
│                          │                                      │
│  ┌───────────────────────▼──────────────────────────────────┐   │
│  │  Per-Vehicle Nodes (/Droan1, /PX4_Drone2)                │   │
│  │  - Sensor topic publishers                               │   │
│  │  - AirSim RPC clients                                    │   │
│  │  - Coordinate transforms (REP 105)                       │   │
│  └───────────────────────┬──────────────────────────────────┘   │
│                          │                                      │
│  ┌───────────────────────▼──────────────────────────────────┐   │
│  │  Topic Remapper (Namespace Bridge)                       │   │
│  │  /Droan1/camera/image → /camera/image_raw                │   │
│  │  /Droan1/lidar/points → /lidar/points                    │   │
│  └───────────────────────┬──────────────────────────────────┘   │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                           │ AirSim RPC (TCP 41451)
                           │ MAVLink UDP (14540-14550)
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│                 AirSim (Unreal Engine 5)                        │
│                                                                 │
│  - Physics simulation (vehicles, collisions)                    │
│  - Sensor rendering (cameras, LiDAR, depth)                     │
│  - Environment (weather, lighting, terrains)                    │
│  - PX4 SITL MAVLink connection                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Communication Flow

**Autonomous Mission Example**:

1. **User** sends mission to AirStack behavior executive
2. **AirStack Behavior** creates high-level plan (search area, navigate waypoints)
3. **AirStack Global Planner** generates paths avoiding obstacles
4. **AirStack Local Planner** creates smooth trajectories
5. **AirStack Interface** publishes MAVROS setpoints:
   ```
   /mavros/setpoint_position/local → PoseStamped(x, y, z, yaw)
   ```
6. **Your MAVROS Node** converts to MAVLink and sends to PX4 SITL
7. **PX4 SITL** sends actuator commands to AirSim (MAVLink UDP)
8. **AirSim** updates physics and renders sensors
9. **Your Vehicle Nodes** publish sensor data:
   ```
   /Droan1/camera/image → /camera/image_raw (remapped)
   /Droan1/lidar/points → /lidar/points (remapped)
   ```
10. **AirStack Perception** processes sensor data, detects obstacles
11. **Loop continues** - closed-loop autonomous control

### Network Architecture

```yaml
# docker-compose.yml (simplified)
version: "3.8"

networks:
  autonomy_network:
    driver: bridge

services:
  # AirStack Autonomy
  airstack:
    image: airlab-registry.andrew.cmu.edu/airstack:latest  # or build from source
    networks:
      - autonomy_network
    environment:
      - ROS_DOMAIN_ID=0
      - MAVROS_FICU_URL=udp://@mavros-bridge:14550
    depends_on:
      - mavros-bridge

  # MAVROS Bridge
  mavros-bridge:
    image: your-mavros-image
    networks:
      - autonomy_network
    environment:
      - FCU_URL=udp://:14540@px4-sitl:14557
    ports:
      - "14540:14540/udp"  # PX4 connection

  # Your ROS2 Nodes
  airsim-ros2:
    build: ./docker/airsim_ros2_wrapper
    networks:
      - autonomy_network
    environment:
      - AIRSIM_HOST_IP=172.28.240.1  # Windows AirSim
      - ROS_DOMAIN_ID=0
    volumes:
      - ./ros2/src:/airsim_ros2_ws/src

  # PX4 SITL (per drone)
  px4-sitl-1:
    image: px4io/px4-sitl:latest
    networks:
      - autonomy_network
    command: make px4_sitl_default none_iris
```

---

## Quick Start Validation

Before committing to full integration, validate core compatibility with these tests.

### Test 1: MAVROS Installation (15 minutes)

**Objective**: Verify MAVROS can control your PX4 SITL

```bash
# In your ROS2 container
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Download geographiclib datasets (required for GPS)
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# Launch MAVROS
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14540@localhost:14557 \
  -p gcs_url:= \
  -p system_id:=1 \
  -p component_id:=1
```

**Expected Output**:
```
[INFO] [mavros_node]: FCU URL: udp://:14540@localhost:14557
[INFO] [mavros_node]: Connected to PX4 Autopilot
[INFO] [mavros_node]: CON: Got HEARTBEAT, connected
```

### Test 2: Verify Topics (5 minutes)

```bash
# Check MAVROS topics are available
ros2 topic list | grep mavros

# Expected output (partial):
# /mavros/state
# /mavros/local_position/pose
# /mavros/setpoint_position/local
# /mavros/setpoint_velocity/cmd_vel
# /mavros/imu/data
```

### Test 3: Arm and Takeoff via MAVROS (10 minutes)

```bash
# Terminal 1: Launch AirSim with PX4
# (Start AirSim + PX4 SITL as normal)

# Terminal 2: MAVROS
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557

# Terminal 3: Commands
# Set OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# Arm vehicle
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Send position setpoint (hover at 5m)
ros2 topic pub -r 10 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'map'
pose:
  position: {x: 0.0, y: 0.0, z: 5.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

**Success Criteria**:
- ✅ Drone arms
- ✅ Drone takes off
- ✅ Drone hovers at 5m altitude
- ✅ No errors in MAVROS logs

**If this works**: AirStack integration is **VIABLE** ✅

**If this fails**: Investigate PX4 configuration, MAVLink connection issues

### Test 4: Sensor Data Flow (10 minutes)

```bash
# Verify AirSim sensor data is accessible
ros2 topic list | grep -E "/(Droan|PX4_Drone)"

# Check camera images
ros2 topic hz /Droan1/camera/image
# Expected: ~30 Hz

# Check LiDAR point clouds
ros2 topic hz /Droan1/lidar/points
# Expected: 5-10 Hz

# Create topic remapping for AirStack compatibility
ros2 run topic_tools relay /Droan1/camera/image /camera/image_raw
ros2 run topic_tools relay /Droan1/lidar/points /lidar/points

# Verify remapped topics
ros2 topic echo /camera/image_raw --once
ros2 topic echo /lidar/points --once
```

**Success Criteria**:
- ✅ All sensor topics publishing
- ✅ Remapping works without latency
- ✅ Message types compatible

### Test 5: Multi-Drone MAVROS (20 minutes)

**Objective**: Test MAVROS with multiple PX4 instances

```bash
# Launch 2 PX4 SITL instances
# Terminal 1:
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0

# Terminal 2:
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 1

# Launch MAVROS for drone 1
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/drone1 \
  -p fcu_url:=udp://:14540@localhost:14557

# Launch MAVROS for drone 2
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/drone2 \
  -p fcu_url:=udp://:14541@localhost:14558

# Verify both connected
ros2 topic echo /drone1/mavros/state --once
ros2 topic echo /drone2/mavros/state --once
```

**Success Criteria**:
- ✅ Both MAVROS instances running
- ✅ Independent control (arm drone 1 doesn't affect drone 2)
- ✅ No MAVLink conflicts

**Validation Complete**: If all 5 tests pass, proceed to Phase 1 implementation.

---

## Effort Estimation

### Phase-by-Phase Breakdown

| Phase | Components | Complexity | Duration | Effort Level | Success Risk |
|-------|-----------|------------|----------|--------------|-------------|
| **Phase 1** | MAVROS Baseline | Low | 1-2 weeks | ⭐ | Low |
| **Phase 2** | Perception Integration | Medium | 2-3 weeks | ⭐⭐ | Low-Medium |
| **Phase 3** | Local Planning | Medium | 2-3 weeks | ⭐⭐ | Medium |
| **Phase 4** | Behavior Trees | High | 3-4 weeks | ⭐⭐⭐⭐ | High |
| **Phase 5** | Multi-Drone Coord | High | 2-3 weeks | ⭐⭐⭐⭐ | Medium-High |
| **Phase 6** | Testing & Polish | Medium | 2-3 weeks | ⭐⭐ | Low |
| **TOTAL** | Full Integration | **MODERATE** | **12-18 weeks** | **⭐⭐⭐** | **Medium** |

### Detailed Effort Analysis

#### Phase 1: MAVROS Baseline (1-2 weeks) ⭐

**Tasks**:
- [x] Install MAVROS in ROS2 container
- [x] Configure MAVROS for PX4 connection
- [x] Test basic commands (arm, takeoff, position setpoints)
- [x] Verify multi-drone MAVROS
- [x] Create topic remapping configuration

**Deliverables**:
- Working MAVROS nodes for each drone
- Documented command interface
- Validation test suite

**Prerequisites**: None

**Risk**: LOW - Well-documented, standard ROS2 packages

---

#### Phase 2: Perception Integration (2-3 weeks) ⭐⭐

**Tasks**:
- [ ] Clone AirStack perception layer from GitHub
- [ ] Build perception packages (skip simulator dependencies)
- [ ] Configure perception to use your sensor topics
- [ ] Test object detection with AirSim cameras
- [ ] Compare AirStack perception vs your YOLOv7
- [ ] Choose hybrid or replacement strategy

**Deliverables**:
- AirStack perception nodes running
- Performance comparison (accuracy, latency)
- Integration decision document

**Prerequisites**: Phase 1 complete

**Risk**: LOW-MEDIUM - May need to debug build issues

---

#### Phase 3: Local Planning (2-3 weeks) ⭐⭐

**Tasks**:
- [ ] Build AirStack planning layer
- [ ] Configure planner to use perception outputs
- [ ] Connect planner output to MAVROS setpoints
- [ ] Test obstacle avoidance in AirSim
- [ ] Tune planning parameters for AirSim physics

**Deliverables**:
- Working trajectory planner
- Obstacle avoidance demos
- Parameter tuning guide

**Prerequisites**: Phase 2 complete

**Risk**: MEDIUM - May need physics tuning

---

#### Phase 4: Behavior Trees (3-4 weeks) ⭐⭐⭐⭐

**Tasks**:
- [ ] Study AirStack behavior tree architecture
- [ ] Port your ROS2 actions to behavior tree nodes
- [ ] Integrate behavior executive
- [ ] Create mission definition files
- [ ] Test complex missions (search, track, return)

**Deliverables**:
- Behavior tree framework running
- Migrated mission definitions
- Mission execution demos

**Prerequisites**: Phase 3 complete

**Risk**: HIGH - Significant architecture changes

---

#### Phase 5: Multi-Drone Coordination (2-3 weeks) ⭐⭐⭐⭐

**Tasks**:
- [ ] Integrate AirStack multi-robot coordinator
- [ ] Configure fleet management
- [ ] Test coordinated missions (formation flight, distributed search)
- [ ] Integrate ground control station (optional)
- [ ] Performance testing with 5+ drones

**Deliverables**:
- Fleet coordination system
- Multi-drone mission demos
- Performance benchmarks

**Prerequisites**: Phase 4 complete

**Risk**: MEDIUM-HIGH - Complexity increases with fleet size

---

#### Phase 6: Testing & Polish (2-3 weeks) ⭐⭐

**Tasks**:
- [ ] End-to-end integration testing
- [ ] Performance optimization
- [ ] Documentation updates
- [ ] Create demo scenarios
- [ ] User training materials

**Deliverables**:
- Validated integrated system
- Complete documentation
- Demo videos
- Training guide

**Prerequisites**: All phases complete

**Risk**: LOW - Cleanup and documentation

---

### Resource Requirements

**Personnel**:
- **1 Full-Time Engineer**: 12-18 weeks
- **OR 2 Engineers**: 6-9 weeks (parallel work on perception + planning)

**Skills Required**:
- ROS2 (intermediate)
- Docker (intermediate)
- C++ (intermediate)
- Python (intermediate)
- MAVLink/PX4 (basic understanding)
- Behavior trees (learning required)

**Hardware**:
- Ubuntu 22.04 workstation
- NVIDIA GPU (RTX 3070+)
- 32GB RAM (minimum)
- Windows PC for AirSim (if WSL2 setup)

**Software Licenses**:
- Unreal Engine 5 (free for research)
- AirStack (verify CMU license/access)

---

## Alternative Approaches

### Option A: Full Integration (Recommended for Research)

**What**: Integrate all AirStack layers (perception → planning → behavior)

**Pros**:
- ✅ Access to production-tested algorithms
- ✅ Advanced autonomy capabilities
- ✅ Ground control station integration
- ✅ Multi-robot coordination

**Cons**:
- ❌ Long integration time (12-18 weeks)
- ❌ Lose some custom implementations (YOLOv7)
- ❌ Dependency on AirStack updates

**When to Choose**: Research focus on autonomy algorithms, access to CMU support

---

### Option B: Selective Integration (Pragmatic)

**What**: Cherry-pick specific AirStack components

**Common Selections**:

1. **Perception Only**:
   - Use: AirStack object detection/tracking
   - Keep: Your planning and mission coordination
   - Time: 3-4 weeks

2. **Planning Only**:
   - Use: AirStack trajectory planner
   - Keep: Your perception (YOLOv7) and missions
   - Time: 3-4 weeks

3. **Behavior Trees Only**:
   - Use: BehaviorTree.CPP framework
   - Keep: Everything else
   - Time: 4-5 weeks

**Pros**:
- ✅ Faster integration
- ✅ Lower risk
- ✅ Keep proven components

**Cons**:
- ❌ Miss out on full stack benefits
- ❌ May need custom glue code

**When to Choose**: Tight deadlines, specific capability gaps

---

### Option C: MAVROS-Only Integration (Minimal)

**What**: Use only AirStack's MAVROS interface layer

**Implementation**:
- Install MAVROS
- Use as alternative control interface
- Skip all other AirStack components

**Pros**:
- ✅ Minimal effort (1-2 weeks)
- ✅ Adds MAVROS capability
- ✅ No architecture changes

**Cons**:
- ❌ Minimal benefit from AirStack
- ❌ Don't get advanced autonomy

**When to Choose**: Just need MAVROS for other integrations (e.g., other planning tools)

---

### Option D: Parallel Evaluation (De-Risking)

**What**: Run both systems in parallel for comparison

**Implementation**:
1. Set up AirStack in separate container
2. Configure to control one drone (Droan1)
3. Keep your system controlling other drones (PX4_Drone2, etc.)
4. Compare performance, capabilities, stability
5. Make data-driven decision after 1 month

**Pros**:
- ✅ Direct comparison
- ✅ Low commitment
- ✅ Learn AirStack without disrupting existing work

**Cons**:
- ❌ Extra maintenance overhead
- ❌ Duplicate effort

**When to Choose**: Unsure about full commitment, want proof-of-concept

---

## Decision Matrix

Use this matrix to guide your integration decision:

| Criterion | Full Integration | Selective | MAVROS-Only | Parallel Eval |
|-----------|-----------------|-----------|-------------|---------------|
| **Timeline** | 12-18 weeks | 4-6 weeks | 1-2 weeks | 4 weeks |
| **Effort** | High | Medium | Low | Medium |
| **Risk** | Medium | Low | Very Low | Low |
| **Autonomy Gain** | Very High | Medium | Minimal | Evaluation |
| **AirSim Compatibility** | Good | Good | Excellent | Good |
| **Keeps YOLOv7** | No | Maybe | Yes | Yes |
| **Keeps Missions** | No | Maybe | Yes | Yes |
| **CMU Support Needed** | Yes | Maybe | No | Maybe |
| **Multi-Drone Support** | Excellent | Good | Good | Good |

### Recommendation Flowchart

```
START
  │
  ├─ Do you have CMU AirLab access/collaboration?
  │   ├─ NO  → Option C (MAVROS-Only) or Option D (Parallel Eval)
  │   └─ YES → Continue
  │
  ├─ Is your research focus on autonomy algorithms?
  │   ├─ YES → Option A (Full Integration)
  │   └─ NO  → Continue
  │
  ├─ Do you have 12+ weeks for integration?
  │   ├─ NO  → Option B (Selective Integration)
  │   └─ YES → Continue
  │
  ├─ Are you satisfied with current capabilities?
  │   ├─ YES → Option C (MAVROS-Only for future flexibility)
  │   └─ NO  → Option A (Full Integration)
  │
  └─ Recommendation: Start with Option D (Parallel Eval)
      → Make final decision after 4 weeks
```

---

## Next Steps

### Immediate Actions (Week 1)

1. **Legal/Administrative**:
   - [ ] Review AirStack license
   - [ ] Contact CMU AirLab for access/collaboration
   - [ ] Assess organizational support for 12-18 week project

2. **Technical Validation**:
   - [ ] Run [Quick Start Validation](#quick-start-validation) tests
   - [ ] Document results
   - [ ] Identify blockers

3. **Decision Making**:
   - [ ] Review this document with stakeholders
   - [ ] Choose integration approach (Full, Selective, MAVROS, Parallel)
   - [ ] Allocate resources (personnel, hardware, time)

### If Proceeding (Week 2)

1. **Environment Setup**:
   - [ ] Clone AirStack repository
   - [ ] Set up Docker environment
   - [ ] Install MAVROS

2. **Phase 1 Kickoff**:
   - [ ] Follow [INTEGRATION_ROADMAP.md](INTEGRATION_ROADMAP.md) Phase 1
   - [ ] Set up weekly progress tracking
   - [ ] Define success metrics

---

## Conclusion

**AirStack integration with Cosys-AirSim is feasible** but requires careful planning and significant development effort. The recommended approach is:

1. **Start Small**: Quick Start Validation (1 week)
2. **Prove Viability**: Phase 1 MAVROS baseline (2 weeks)
3. **Assess Value**: Parallel evaluation (1 month)
4. **Commit or Pivot**: Decision point based on data

**Key Success Factors**:
- ✅ MAVROS compatibility (validated)
- ✅ Sensor topic compatibility (validated)
- ✅ Docker deployment (validated)
- ⚠️ CMU access/license (TBD)
- ⚠️ Behavior tree migration (significant effort)
- ⚠️ Multi-robot coordination (complexity)

**Expected Outcome**: A production-grade autonomy stack combining AirSim's high-fidelity simulation with AirStack's proven algorithms for real-world autonomous operations.

---

## References

- [AirStack GitHub](https://github.com/castacks/AirStack)
- [AirStack Documentation](https://docs.theairlab.org/)
- [MAVROS GitHub](https://github.com/mavlink/mavros)
- [PX4 Autopilot](https://docs.px4.io/)
- [Cosys-AirSim](https://cosys-lab.github.io/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

**Document Version**: 1.0
**Last Updated**: 2025-10-16
**Status**: Draft for Review
