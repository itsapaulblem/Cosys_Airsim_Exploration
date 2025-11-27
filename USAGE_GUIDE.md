# AirSim Multi-Drone Architecture - Complete Usage Guide

> **Production-Ready Guide** for deploying AirSim multi-drone simulations with ROS2, PX4 SITL, YOLOv7+DeepSORT AI, and complete monitoring stack.

---

## Table of Contents

1. [Quick Start (5 Minutes)](#1-quick-start-5-minutes)
2. [Architecture Overview](#2-architecture-overview)
3. [Prerequisites](#3-prerequisites)
4. [Deployment Options](#4-deployment-options)
5. [YOLOv7+DeepSORT AI Integration](#5-yolov7deepsort-ai-integration)
6. [Multi-Drone Configuration](#6-multi-drone-configuration)
7. [Camera Configuration](#7-camera-configuration)
8. [ROS2 Launch Configurations](#8-ros2-launch-configurations)
9. [Monitoring & Debugging](#9-monitoring--debugging)
10. [Advanced Features](#10-advanced-features)
11. [Common Issues & Solutions](#11-common-issues--solutions)
12. [Command Reference](#12-command-reference)
13. [File Locations](#13-file-locations)

---

## 1. Quick Start (5 Minutes)

### Simplest Path to First Flight

```bash
# 1. Clone and navigate to project
cd /path/to/Cosys_Airsim_Exploration

# 2. Set up environment (Linux X11 only)
xhost +local:docker

# 3. Start complete stack (AirSim + PX4 + ROS2)
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up

# 4. Wait for all containers to be healthy (~30 seconds)
# Watch logs: docker compose -f docker/docker-compose-master.yml logs -f

# 5. Access ROS2 container and launch system
docker exec -it ros2-x11-node bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# 6. Verify system is running
ros2 node list  # Should show /airsim_coordination_node + vehicle nodes
ros2 topic list  # Should show /Drone1/*, /Drone2/*, etc.
```

**Success Indicators:**
- ✅ AirSim container shows "Press Play to start" in Unreal
- ✅ PX4 containers show "Ready for takeoff"
- ✅ ROS2 nodes publishing `/Drone1/odom_local_ned`, `/Drone1/imu`, etc.

---

## 2. Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────────────┐
│                         HOST SYSTEM (Windows/Linux)                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌────────────────────┐       ┌──────────────────────────────────┐ │
│  │  AirSim Container  │◄──────┤   ROS2 X11 Node Container       │ │
│  │  (Unreal Engine)   │  RPC  │  - rpc_dynamic_vehicles.launch   │ │
│  │                    │ 41451 │  - YOLOv7+DeepSORT AI            │ │
│  │  - Physics Sim     │       │  - Motion Detection Node         │ │
│  │  - Sensors         │       │  - 4-Camera Processing           │ │
│  │  - Cameras/LiDAR   │       │  - Localization (REP 105)        │ │
│  └────────┬───────────┘       └────────────┬─────────────────────┘ │
│           │                                 │                        │
│           │                                 │ MAVLink (UDP 14550)    │
│           │                                 │                        │
│  ┌────────▼───────────────────────────────▼──────────────────────┐ │
│  │         PX4 SITL Containers (Multi-Instance)                   │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐           │ │
│  │  │ px4-drone-1 │  │ px4-drone-2 │  │ px4-drone-3 │  ...      │ │
│  │  │  Instance 0 │  │  Instance 1 │  │  Instance 2 │           │ │
│  │  │  Port 14540 │  │  Port 14541 │  │  Port 14542 │           │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘           │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │           Monitoring Stack (Optional)                         │ │
│  │  ┌─────────────┐  ┌──────────────┐  ┌──────────────────────┐│ │
│  │  │ Prometheus  │  │   Grafana    │  │   Metric Exporters   ││ │
│  │  │  Port 9090  │  │  Port 3000   │  │  (ROS2/AirSim/PX4)   ││ │
│  │  └─────────────┘  └──────────────┘  └──────────────────────┘│ │
│  └──────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **AirSim → PX4**: MAVLink telemetry (GPS, IMU, attitude)
2. **PX4 → AirSim**: Motor commands, actuator control
3. **AirSim → ROS2**: Sensor data (cameras, LiDAR), odometry (RPC API)
4. **ROS2 → Users**: Topics, services, visualizations (RViz2)
5. **YOLOv7 → Motion Detection**: Object detection + tracking with persistent IDs

### Key Architecture Principles

- **REP 105 Compliant**: Proper `map → odom → base_link → sensors` frame hierarchy
- **Ultra-Clean Naming**: Vehicle names ARE node names (`/Drone1`, `/PX4_Drone2`)
- **Fault Isolation**: Independent vehicle nodes prevent cascading failures
- **Cross-Platform**: Windows AirSim + Linux Docker ROS2 communication
- **Dynamic Discovery**: Automatic vehicle detection from settings.json

---

## 3. Prerequisites

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **OS** | Linux (Ubuntu 22.04+) or Windows 11 with WSL2 | Linux native |
| **RAM** | 16 GB | 32 GB+ |
| **GPU** | NVIDIA GTX 1060 6GB | RTX 3060 12GB+ |
| **CPU** | 8 cores | 12+ cores |
| **Disk** | 50 GB free | 100 GB+ SSD |

### Software Requirements

**Linux:**
```bash
# Docker & Docker Compose V2
sudo apt-get update
sudo apt-get install docker.io docker-compose-v2

# X11 forwarding (for native GUI performance)
sudo apt-get install x11-xserver-utils

# NVIDIA Container Toolkit (for GPU acceleration)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

**Windows (WSL2):**
```powershell
# Install WSL2
wsl --install -d Ubuntu-22.04

# Install Docker Desktop with WSL2 backend
# Download from: https://www.docker.com/products/docker-desktop

# In WSL2 terminal:
sudo apt-get update
sudo apt-get install docker-compose-v2
```

### AirSim Binary (Required)

You need a running AirSim instance. Options:

1. **Pre-built Binary** (Easiest):
   - Download from: https://github.com/Cosys-Lab/Cosys-AirSim/releases
   - Or use containerized AirSim (see `docker/unreal_executable/`)

2. **Build from Source**:
   ```bash
   # See: UnrealEngine_Build_Documentation.md
   ./setup.sh
   ./build.sh
   ```

---

## 4. Deployment Options

### Profile-Based Deployment

The system uses Docker Compose profiles for different deployment scenarios:

| Profile | Use Case | Components | Performance |
|---------|----------|------------|-------------|
| **`linux-integrated`** | Production Linux | AirSim + PX4 + ROS2 X11 + QGC X11 | **Highest** (native X11, 60-70% lower CPU) |
| **`integrated`** | Cross-platform | AirSim + PX4 + ROS2 VNC | Good (VNC overhead) |
| **`development`** | Active development | All + Dev helpers | Medium |
| **`monitoring`** | With metrics | Add Prometheus + Grafana | Medium (extra overhead) |
| **`px4-only`** | PX4 swarm testing | PX4 fleet only | Highest (no ROS2) |
| **`ros2-only`** | ROS2 development | ROS2 nodes only | High |

### Deployment Examples

#### Linux Native (Recommended)

**Highest performance, lowest latency:**

```bash
# Enable X11 forwarding
xhost +local:docker

# Launch full stack
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up

# With monitoring
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated --profile monitoring up

# Access:
# - Grafana: http://localhost:3000 (admin/P@ssw0rd)
# - Prometheus: http://localhost:9090
# - QGroundControl: Native X11 window
# - RViz2: docker exec -it ros2-x11-node rviz2
```

#### Windows + WSL2

**Cross-platform deployment:**

```bash
# In WSL2 terminal
cd /mnt/c/path/to/Cosys_Airsim_Exploration

# Launch with VNC (no X11 needed)
docker compose -f docker/docker-compose-master.yml \
  --profile integrated up

# Access VNC:
# - ROS2 VNC: http://localhost:5901 (password: ubuntu)
# - QGroundControl VNC: http://localhost:5900 (password: airsim)
```

#### Single Drone Testing

```bash
# Minimal deployment (1 drone)
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up \
  airsim-container px4-bridge-drone-1 ros2-x11-node
```

#### Large Swarm (6 Drones)

```bash
# Scale to 6 drones
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up \
  px4-bridge-drone-{1..6} ros2-x11-node

# Note: Update settings.json for 6 drones first
```

---

## 5. YOLOv7+DeepSORT AI Integration

### Overview

The system includes state-of-the-art AI for:
- **YOLOv7** object detection (real-time person detection)
- **DeepSORT** multi-object tracking (persistent ID assignment)
- **4-Camera 360° Coverage** (Camera_0: front, Camera_1: right, Camera_2: back, Camera_3: left)
- **Automatic Target Following** (drone tracks detected persons)

### Quick Enable

**Inside ROS2 Container:**

```bash
# Start motion detection with YOLOv7+DeepSORT
cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts
python3 motion_detection_node.py
```

**Expected Output:**
```
[INFO] YOLOv7 model loaded successfully
[INFO] YOLOv7 + DeepSORT initialized successfully
[INFO] YOLOv7+DeepSORT: enabled
[INFO] Multi-Camera Motion Detection node initialized for Drone1
```

### Configuration

**Motion Detection Parameters** (`motion_detection_node.py` lines 170-180):

```python
# Detection thresholds
self.confidence_threshold = 0.08  # Lower = more sensitive
self.motion_threshold_pixels = 15.0  # Motion detection sensitivity

# DeepSORT tracking
MAX_DIST = 0.2  # Feature distance threshold
MIN_CONFIDENCE = 0.3  # Detection confidence (lowered for flexibility)
MAX_AGE = 70  # Frames to keep lost tracks
N_INIT = 3  # Consecutive detections to confirm track
```

### Topics Published

```bash
# Detection visualization (one per camera)
/detection_visualization_cam0  # Front camera with bounding boxes
/detection_visualization_cam1  # Right camera
/detection_visualization_cam2  # Back camera
/detection_visualization_cam3  # Left camera

# Target detection (person following)
/target_detection  # Custom TargetDetection message
```

### Performance

| Component | CPU Mode | Speed (FPS) |
|-----------|----------|-------------|
| **YOLOv7 Inference** | CPU (torch.device('cpu')) | 4-8 FPS per camera |
| **DeepSORT Tracking** | CPU | ~20ms per frame |
| **Total Pipeline** | 4 cameras sequential | ~1-2 FPS combined |

**Optimization Tips:**
- Use GPU-enabled PyTorch for 10x speedup
- Use `yolov7-tiny.pt` for faster inference (lower accuracy)
- Process cameras in parallel (multi-threading)

### DeepSORT Checkpoint

**Full tracking requires manual checkpoint download:**

```bash
# Run helper script
cd ros2/src/airsim_ros_pkgs/scripts
./download_deepsort_checkpoint.sh

# Manual download:
# 1. Visit: https://drive.google.com/drive/folders/1kna8eWGrSfzaR6DtNJ8_GchGgPMv3VC8
# 2. Download ckpt.t7 (44MB)
# 3. Place in: YOLOv7-DeepSORT-Object-Tracking/deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7
```

**Without checkpoint:** YOLOv7 detection works, but no persistent tracking IDs.

### Visualization

**In RViz2:**

```bash
docker exec -it ros2-x11-node rviz2

# Add Image display for:
# - /detection_visualization_cam0
# - /detection_visualization_cam1
# - /detection_visualization_cam2
# - /detection_visualization_cam3
```

**Foxglove Studio** (Web-based):
```bash
# Install Foxglove bridge (if not already)
docker compose -f docker/docker-compose-master.yml up foxglove-bridge

# Access: http://localhost:8765
# Add Image panels for detection topics
```

---

## 6. Multi-Drone Configuration

### Settings.json Configuration

**Location:** `~/Documents/AirSim/settings.json` (Linux) or `C:\Users\<YourName>\Documents\AirSim\settings.json` (Windows)

**Example 3-Drone Setup:**

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ClockSpeed": 1,
  "LocalHostIp": "0.0.0.0",
  "ApiServerPort": 41451,
  "RpcEnabled": true,

  "OriginGeopoint": {
    "Latitude": 47.641468,
    "Longitude": -122.140165,
    "Altitude": 122
  },

  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4Multirotor",
      "X": 0, "Y": 0, "Z": 0,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "Cameras": {
        "Camera_0": {
          "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90}],
          "X": 0.90, "Y": 0.00, "Z": -0.05,
          "Yaw": 0.0
        },
        "Camera_1": {
          "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90}],
          "X": 0.40, "Y": 0.70, "Z": -0.05,
          "Yaw": 90.0
        },
        "Camera_2": {
          "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90}],
          "X": -0.50, "Y": 0.00, "Z": -0.05,
          "Yaw": 180.0
        },
        "Camera_3": {
          "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 720, "FOV_Degrees": 90}],
          "X": 0.40, "Y": -0.70, "Z": -0.05,
          "Yaw": -90.0
        }
      },
      "Sensors": {
        "Lidar": {
          "SensorType": 6,
          "Enabled": true,
          "NumberOfChannels": 16,
          "X": 0, "Y": 0, "Z": -0.45
        }
      }
    },
    "Drone2": {
      "VehicleType": "PX4Multirotor",
      "X": 3, "Y": 0, "Z": 0,
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "Cameras": { /* Same as Drone1 */ }
    },
    "Drone3": {
      "VehicleType": "PX4Multirotor",
      "X": 0, "Y": 3, "Z": 0,
      "TcpPort": 4562,
      "ControlPortLocal": 14542,
      "ControlPortRemote": 14582,
      "Cameras": { /* Same as Drone1 */ }
    }
  }
}
```

### PX4 Instance Mapping

| Drone Name | PX4 Instance | TCP Port | MAVLink Local | MAVLink Remote |
|------------|--------------|----------|---------------|----------------|
| Drone1 | 0 | 4560 | 14540 | 14580 |
| Drone2 | 1 | 4561 | 14541 | 14581 |
| Drone3 | 2 | 4562 | 14542 | 14582 |
| Drone4 | 3 | 4563 | 14543 | 14583 |
| Drone5 | 4 | 4564 | 14544 | 14584 |
| Drone6 | 5 | 4565 | 14545 | 14585 |

### Docker Compose Scaling

**Launch specific drones:**

```bash
# 3-drone setup
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up \
  px4-bridge-drone-{1..3} ros2-x11-node

# 6-drone swarm
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up \
  px4-bridge-drone-{1..6} ros2-x11-node
```

### Swarm Coordination

**ROS2 Coordination Node** automatically manages:
- Vehicle discovery from settings.json
- Namespace isolation (`/Drone1/*`, `/Drone2/*`)
- Service coordination (`/Drone1/takeoff`, `/Drone2/land`)
- Transform broadcasting (REP 105 compliant)

---

## 7. Camera Configuration

### Critical Naming Convention

**MUST use `Camera_X` format (capital C, underscore, number):**

✅ **Correct:**
```json
"Cameras": {
  "Camera_0": { ... },  // Front
  "Camera_1": { ... },  // Right
  "Camera_2": { ... },  // Back
  "Camera_3": { ... }   // Left
}
```

❌ **Incorrect:**
```json
"Cameras": {
  "camera_0": { ... },  // Lowercase 'c' - will NOT work
  "camera0": { ... },   // No underscore - will NOT work
  "front": { ... }      // Custom names - will NOT work without code changes
}
```

### 4-Camera 360° Layout

**Standard Configuration:**

| Camera | Direction | Yaw | ROS2 Topic |
|--------|-----------|-----|------------|
| **Camera_0** | Front | 0° | `/Drone1/camera0/image` |
| **Camera_1** | Right | 90° | `/Drone1/camera1/image` |
| **Camera_2** | Back | 180° | `/Drone1/camera2/image` |
| **Camera_3** | Left | -90° | `/Drone1/camera3/image` |

**Positioning (relative to drone center):**

```python
# Front camera (Camera_0)
X: 0.90   # 90cm forward
Y: 0.00   # Centered
Z: -0.05  # 5cm below center

# Right camera (Camera_1)
X: 0.40   # 40cm forward
Y: 0.70   # 70cm right
Z: -0.05

# Back camera (Camera_2)
X: -0.50  # 50cm backward
Y: 0.00   # Centered
Z: -0.05

# Left camera (Camera_3)
X: 0.40   # 40cm forward
Y: -0.70  # 70cm left
Z: -0.05
```

### Camera Parameters

```json
{
  "ImageType": 0,           // 0=Scene, 1=DepthPlanar, 2=DepthPerspective, etc.
  "Width": 1280,            // Resolution width
  "Height": 720,            // Resolution height
  "FOV_Degrees": 90,        // Field of view
  "AutoExposureSpeed": 100, // Optional: exposure control
  "TargetGamma": 1.0        // Optional: gamma correction
}
```

### Verification

**Check cameras are detected:**

```bash
# Inside ROS2 container
ros2 topic list | grep camera

# Expected output:
/Drone1/camera0/image
/Drone1/camera1/image
/Drone1/camera2/image
/Drone1/camera3/image
/Drone1/camera0/camera_info
# ...
```

---

## 8. ROS2 Launch Configurations

### Primary Launch File

**`rpc_dynamic_vehicles.launch.py`** - Modern, ultra-clean architecture

```bash
# Inside ROS2 container
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# With localization (REP 105 compliant transforms)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py enable_localization:=true

# With RViz visualization
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py launch_rviz:=true
```

**Features:**
- ✅ Automatic vehicle discovery from settings.json
- ✅ Per-vehicle independent nodes (`/Drone1`, `/Drone2`)
- ✅ Ultra-clean naming (vehicle name IS node name)
- ✅ REP 105 coordinate frames
- ✅ Cross-platform RPC communication

### Legacy Launch File

**`airsim_node.launch.py`** - Single monolithic node (backward compatibility)

```bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

**Use only for:**
- Testing single-drone setups
- Backward compatibility with old code
- Quick debugging

### Frame Hierarchy (REP 105)

**Proper coordinate frame structure:**

```
map (world frame)
 └─ Drone1/odom (odometry frame, GPS-derived spawn offset)
     └─ Drone1/base_link (vehicle body frame)
         ├─ Drone1/camera0_link (front camera)
         ├─ Drone1/camera1_link (right camera)
         ├─ Drone1/camera2_link (back camera)
         ├─ Drone1/camera3_link (left camera)
         ├─ Drone1/imu_link (IMU sensor)
         └─ Drone1/lidar_link (LiDAR sensor)
```

**Frame Authority:**
- **Localization Node** publishes `map → Drone1/odom`
- **Vehicle Node** publishes `Drone1/odom → Drone1/base_link`
- **Static Transforms** publish sensor attachment frames

### Topics and Services

**Topics (per vehicle):**

```bash
# Odometry
/Drone1/odom_local_ned       # NED frame odometry
/Drone1/odom                 # ROS standard odometry

# Sensors
/Drone1/imu                  # IMU data
/Drone1/camera0/image        # Camera images
/Drone1/lidar/points         # LiDAR point cloud

# Control
/Drone1/cmd_vel              # Velocity commands
```

**Services (per vehicle):**

```bash
# Flight control
/Drone1/takeoff              # Takeoff to default altitude
/Drone1/land                 # Land at current position
/Drone1/go_home              # Return to home position

# Camera control
/Drone1/set_camera_pose      # Set camera orientation
```

**Global Services:**

```bash
/airsim/reset                # Reset simulation
/airsim/pause                # Pause simulation
/airsim/continue             # Resume simulation
```

---

## 9. Monitoring & Debugging

### Prometheus + Grafana Stack

**Enable monitoring:**

```bash
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated --profile monitoring up
```

**Access:**
- **Grafana**: http://localhost:3000 (admin / P@ssw0rd)
- **Prometheus**: http://localhost:9090

**Dashboards:**
1. **AirSim Metrics** (Port 9201)
   - Drone positions, velocities
   - Collision status
   - API response times

2. **ROS2 Metrics** (Port 9200)
   - Topic publication rates
   - Node CPU/memory usage
   - Message queue sizes

3. **PX4 Metrics** (Port 9202)
   - Flight mode status
   - Battery levels
   - GPS fix quality

4. **Container Metrics** (cAdvisor Port 8080)
   - CPU, memory, network per container
   - Disk I/O

### Container Health Checks

```bash
# Check all container status
docker compose -f docker/docker-compose-master.yml ps

# Expected output:
# airsim-container    Up (healthy)
# px4-drone-1         Up (healthy)
# px4-drone-2         Up (healthy)
# ros2-x11-node       Up

# View container logs
docker compose -f docker/docker-compose-master.yml logs -f ros2-x11-node
docker compose -f docker/docker-compose-master.yml logs -f px4-drone-1
```

### ROS2 Diagnostics

**Inside ROS2 container:**

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Monitor topic publication rate
ros2 topic hz /Drone1/odom_local_ned

# View topic data
ros2 topic echo /Drone1/imu

# Check transform tree
ros2 run tf2_tools view_frames.py

# Monitor transforms
ros2 run tf2_ros tf2_monitor map Drone1/base_link
```

### Log Locations

**Container logs:**
```bash
# View live logs
docker logs -f ros2-x11-node
docker logs -f px4-drone-1

# Export logs to file
docker logs ros2-x11-node > ros2_node.log 2>&1
```

**ROS2 logs (inside container):**
```bash
# Default location
/airsim_ros2_ws/log/

# View latest log
tail -f /airsim_ros2_ws/log/latest/airsim_ros_pkgs/
```

**Motion detection logs:**
```bash
# Inside container
tail -f /tmp/motion_yolov7_test.log
```

---

## 10. Advanced Features

### GPS-Based Multi-Vehicle Positioning

**How it works:**

1. Each vehicle spawns at local origin `[0, 0, ground_height]`
2. Different GPS coordinates assigned per vehicle (from settings.json)
3. GPS-to-NED conversion creates proper world-space separation
4. Each vehicle gets independent `odom` frame anchored at GPS-derived position

**Example:**

```json
"Drone1": {
  "X": 0, "Y": 0, "Z": 0,  // Spawn position in settings
  "GPS": [47.641468, -122.140165, 122]  // Actual GPS coordinates
},
"Drone2": {
  "X": 3, "Y": 0, "Z": 0,  // Settings say 3m offset
  "GPS": [47.641495, -122.140138, 122]  // GPS reflects real-world offset
}
```

**Result:** In RViz with `Fixed Frame: map`, drones appear at correct relative positions based on GPS differences.

### LiDAR Point Cloud Merging (OctoMap)

**Launch point cloud merger:**

```bash
# Inside ROS2 container
ros2 run airsim_ros_pkgs pointcloud_merger_node
```

**Features:**
- Merges 4-drone LiDAR streams
- Transforms all clouds to `map` frame
- Publishes merged cloud to `/merged_pointcloud`
- Compatible with OctoMap 3D mapping

**OctoMap Integration:**

```bash
# Launch OctoMap server
ros2 run octomap_server octomap_server_node \
  --ros-args -p resolution:=0.1 \
  -r cloud_in:=/merged_pointcloud
```

### Mission Planning

**Mission coordination node:**

```bash
# Inside ROS2 container
ros2 run airsim_ros_pkgs mission_coordination_node
```

**Features:**
- Action server for `SearchArea` goals
- Multi-drone task allocation
- GPS waypoint navigation
- Area coverage planning

**Example mission client:**

```bash
cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts
python3 mission_coordination_demo_client.py
```

### QGroundControl Setup

**Docker X11 (Linux):**

```bash
# Included in linux-integrated profile
docker compose -f docker/docker-compose-master.yml \
  --profile linux-integrated up qgroundcontrol-x11

# QGC window appears on host display
```

**Docker VNC (Cross-platform):**

```bash
# Launch QGC with VNC
docker compose -f docker/docker-compose-master.yml up qgroundcontrol

# Access: http://localhost:6080 (noVNC web interface)
# Or: VNC client to localhost:5900 (password: airsim)
```

**Native QGC:**

1. Download from: http://qgroundcontrol.com/downloads/
2. QGC automatically discovers PX4 drones broadcasting on UDP 14550
3. No additional configuration needed for Docker bridge network

---

## 11. Common Issues & Solutions

### Issue: YOLOv7 Not Loading

**Symptoms:**
```
ModuleNotFoundError: No module named 'models'
ValueError: numpy.dtype size changed
```

**Solutions:**

1. **NumPy/SciPy binary incompatibility:**
```bash
docker exec ros2-x11-node pip3 install --force-reinstall --no-deps 'scipy>=1.4.1'
```

2. **Missing easydict:**
```bash
docker exec ros2-x11-node pip3 install easydict
```

3. **Rebuild image with fixes:**
```bash
docker compose -f docker/docker-compose-master.yml build ros2-x11-node
```

**See:** `docker/airsim_ros2_wrapper/YOLOV7_DEEPSORT_INTEGRATION.md` for complete guide.

---

### Issue: Cameras Not Detected

**Symptoms:**
```
[WARN] No cameras found for vehicle Drone1
ros2 topic list | grep camera  # Returns nothing
```

**Solution:**

Check camera naming in `settings.json`:

❌ **Wrong:**
```json
"cameras": {          // Lowercase 'c'
  "camera0": { ... }  // No underscore
}
```

✅ **Correct:**
```json
"Cameras": {          // Capital 'C'
  "Camera_0": { ... } // Underscore + number
}
```

**Verification:**
```bash
# Inside ROS2 container
ros2 topic list | grep camera
# Should show: /Drone1/camera0/image, /Drone1/camera1/image, etc.
```

---

### Issue: PX4 Connection Failures

**Symptoms:**
```
[ERROR] Failed to connect to AirSim at 172.x.x.x:4560
px4-drone-1 | Connection refused
```

**Solutions:**

1. **Check AirSim is running:**
```bash
docker logs airsim-container | grep "Listening"
# Should show: "Listening on 0.0.0.0:41451"
```

2. **Check network connectivity:**
```bash
docker exec px4-drone-1 ping airsim-container -c 3
docker exec px4-drone-1 nc -zv airsim-container 4560
```

3. **Check firewall (Windows):**
```powershell
# As Administrator
.\docker\px4_airsim_docker_v2\windows-firewall-setup.ps1
```

4. **Check PX4_SIM_HOSTNAME:**
```bash
docker exec px4-drone-1 env | grep PX4_SIM_HOSTNAME
# Should be: PX4_SIM_HOSTNAME=airsim-container
```

**See:** `docs/mavlink_networking_troubleshooting.md` for complete guide.

---

### Issue: ROS2 Nodes Not Communicating

**Symptoms:**
```
ros2 node list  # Shows nodes
ros2 topic list  # Shows topics
ros2 topic echo /Drone1/odom  # But no data
```

**Solutions:**

1. **Check ROS_DOMAIN_ID:**
```bash
docker exec ros2-x11-node env | grep ROS_DOMAIN_ID
docker exec ros2-x11-node bash -c "source /opt/ros/humble/setup.bash && ros2 doctor --report"
```

2. **Check DDS discovery:**
```bash
docker exec ros2-x11-node ros2 daemon stop
docker exec ros2-x11-node ros2 daemon start
docker exec ros2-x11-node ros2 topic list
```

3. **Check container networking:**
```bash
docker network inspect ros2-multi-node-network
docker network inspect airsim-ecosystem
```

---

### Issue: Performance / High CPU Usage

**Symptoms:**
- FPS drops below 10
- High CPU usage (>90%)
- Container OOM kills

**Solutions:**

1. **Use Linux X11 native (not VNC):**
```bash
# 60-70% lower CPU usage
docker compose -f docker/docker-compose-master.yml --profile linux-integrated up
```

2. **Disable YOLOv7 if not needed:**
```bash
# Don't run motion_detection_node.py
# Or use OpenCV-only mode (automatic fallback if imports fail)
```

3. **Reduce camera resolution:**
```json
"CaptureSettings": [{
  "Width": 640,   // Lower from 1280
  "Height": 480   // Lower from 720
}]
```

4. **Increase Docker resources:**
```bash
# Edit: ~/.docker/daemon.json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-shm-size": "4G"  // Increase shared memory
}

sudo systemctl restart docker
```

5. **Use GPU acceleration:**
```dockerfile
# In Dockerfile, change PyTorch to CUDA version
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

---

### Issue: Git Repository Errors in YOLOv7

**Symptoms:**
```
fatal: not a git repository (or any parent up to mount point)
```

**Solution:**

This is expected with volume mounts. The fix is already applied in Dockerfile:
- Git repository initialized during Docker build
- Path fix removes `.lower()` bug from `google_utils.py`
- YOLOv7 weights still load correctly

**If issue persists:**

```bash
# Inside container
cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking
git init
git add .
git commit -m "YOLOv7 for AirSim"
git tag v1.0
```

---

## 12. Command Reference

### Docker Compose Commands

```bash
# Build specific service
docker compose -f docker/docker-compose-master.yml build ros2-x11-node

# Start services
docker compose -f docker/docker-compose-master.yml up          # Foreground
docker compose -f docker/docker-compose-master.yml up -d       # Background
docker compose -f docker/docker-compose-master.yml up --build  # Rebuild + start

# Stop services
docker compose -f docker/docker-compose-master.yml stop
docker compose -f docker/docker-compose-master.yml down        # Stop + remove containers
docker compose -f docker/docker-compose-master.yml down -v     # Stop + remove volumes

# View logs
docker compose -f docker/docker-compose-master.yml logs -f ros2-x11-node
docker compose -f docker/docker-compose-master.yml logs --tail=100

# Container status
docker compose -f docker/docker-compose-master.yml ps
docker compose -f docker/docker-compose-master.yml top

# Execute commands
docker compose -f docker/docker-compose-master.yml exec ros2-x11-node bash
docker compose -f docker/docker-compose-master.yml exec px4-drone-1 bash

# Restart specific service
docker compose -f docker/docker-compose-master.yml restart ros2-x11-node
```

### ROS2 Launch Commands

```bash
# Primary launch (modern)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# With localization
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py enable_localization:=true

# With RViz
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py launch_rviz:=true

# Mission-capable launch
ros2 launch airsim_ros_pkgs rpc_dynamic_mission.launch.py

# Legacy single-node (backward compatibility)
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### ROS2 Diagnostics

```bash
# Nodes
ros2 node list
ros2 node info /Drone1

# Topics
ros2 topic list
ros2 topic info /Drone1/odom_local_ned
ros2 topic hz /Drone1/camera0/image
ros2 topic echo /Drone1/imu --once

# Services
ros2 service list
ros2 service call /Drone1/takeoff std_srvs/srv/Trigger

# Parameters
ros2 param list /Drone1
ros2 param get /Drone1 use_sim_time

# Transforms
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo map Drone1/base_link
ros2 run tf2_ros tf2_monitor

# General health
ros2 doctor --report
ros2 wtf
```

### Container Management

```bash
# List all containers
docker ps -a

# Start/stop specific container
docker start ros2-x11-node
docker stop ros2-x11-node
docker restart ros2-x11-node

# Execute commands
docker exec -it ros2-x11-node bash
docker exec ros2-x11-node ros2 node list

# View logs
docker logs -f ros2-x11-node
docker logs --tail=100 px4-drone-1

# Inspect container
docker inspect ros2-x11-node
docker stats ros2-x11-node

# Clean up
docker system prune -a          # Remove all unused containers/images
docker volume prune             # Remove unused volumes
docker network prune            # Remove unused networks
```

---

## 13. File Locations

### Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| **AirSim Settings** | `~/Documents/AirSim/settings.json` | Vehicle configuration, cameras, sensors |
| **Docker Compose** | `docker/docker-compose-master.yml` | Service orchestration |
| **Docker Environment** | `docker/.env` | Environment variables (AIRSIM_HOST_IP, etc.) |
| **ROS2 Params** | `ros2/src/airsim_ros_pkgs/config/` | ROS2 node parameters |

### Launch Files

| File | Location | Description |
|------|----------|-------------|
| **RPC Dynamic** | `ros2/src/airsim_ros_pkgs/launch/rpc_dynamic_vehicles.launch.py` | Primary multi-vehicle launch |
| **RPC Mission** | `ros2/src/airsim_ros_pkgs/launch/rpc_dynamic_mission.launch.py` | Mission-capable launch |
| **Legacy Single** | `ros2/src/airsim_ros_pkgs/launch/airsim_node.launch.py` | Legacy monolithic node |
| **Motion Detection** | `ros2/src/airsim_ros_pkgs/launch/motion_detection_launch.py` | YOLOv7 detection launcher |

### AI Models

| File | Location | Size | Description |
|------|----------|------|-------------|
| **YOLOv7 Weights** | `ros2/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking/yolov7.pt` | 73MB | Object detection model |
| **DeepSORT Checkpoint** | `ros2/src/airsim_ros_pkgs/scripts/YOLOv7-DeepSORT-Object-Tracking/deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7` | 44MB | Tracking model (manual download) |

### Source Code

| Component | Location | Description |
|-----------|----------|-------------|
| **Vehicle Nodes** | `ros2/src/airsim_ros_pkgs/src/multirotor_node.cpp` | Vehicle ROS2 wrapper |
| **Coordination** | `ros2/src/airsim_ros_pkgs/src/coordination_node.cpp` | Fleet coordination |
| **Localization** | `ros2/src/airsim_ros_pkgs/src/localization_node.cpp` | REP 105 localization |
| **Motion Detection** | `ros2/src/airsim_ros_pkgs/scripts/motion_detection_node.py` | YOLOv7+DeepSORT integration |
| **Point Cloud Merger** | `ros2/src/airsim_ros_pkgs/src/pointcloud_merger_node.cpp` | Multi-LiDAR fusion |

### Documentation

| Document | Location | Topics |
|----------|----------|--------|
| **This Guide** | `USAGE_GUIDE.md` | Complete usage reference |
| **Architecture README** | `ros2/src/airsim_ros_pkgs/README_MULTIROTOR_ARCHITECTURE.md` | ROS2 architecture details |
| **YOLOv7 Integration** | `docker/airsim_ros2_wrapper/YOLOV7_DEEPSORT_INTEGRATION.md` | AI integration guide |
| **Docker Guide** | `docker/airsim_containerization_guide.md` | Docker deployment |
| **Network Troubleshooting** | `docs/mavlink_networking_troubleshooting.md` | MAVLink debugging |
| **Monitoring Guide** | `docker/MONITORING_INTEGRATION_COMPLETE.md` | Prometheus + Grafana |

### Log Locations

| Log Type | Location | Access |
|----------|----------|--------|
| **Docker Logs** | Docker daemon | `docker logs <container>` |
| **ROS2 Logs** | `/airsim_ros2_ws/log/` (in container) | `docker exec ros2-x11-node tail -f /airsim_ros2_ws/log/latest/...` |
| **Motion Detection** | `/tmp/motion_yolov7_test.log` (in container) | `docker exec ros2-x11-node cat /tmp/motion_yolov7_test.log` |
| **Build Logs** | `/tmp/docker_*_build.log` (host) | `cat /tmp/docker_yolov7_rebuild.log` |

---

## Additional Resources

### Official Documentation

- **Cosys-AirSim**: https://cosys-lab.github.io/
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **PX4 Autopilot**: https://docs.px4.io/
- **Docker Compose**: https://docs.docker.com/compose/

### Community

- **Cosys-AirSim Issues**: https://github.com/Cosys-Lab/Cosys-AirSim/issues
- **ROS2 Forums**: https://discourse.ros.org/
- **PX4 Forums**: https://discuss.px4.io/

### Related Projects

- **YOLOv7**: https://github.com/WongKinYiu/yolov7
- **DeepSORT**: https://github.com/ZQPei/deep_sort_pytorch
- **OctoMap**: https://octomap.github.io/
- **Foxglove Studio**: https://foxglove.dev/

---

## Quick Reference Card

### Essential Commands

```bash
# Start full stack (Linux)
xhost +local:docker
docker compose -f docker/docker-compose-master.yml --profile linux-integrated up

# Inside ROS2 container
docker exec -it ros2-x11-node bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Enable YOLOv7+DeepSORT
cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts
python3 motion_detection_node.py

# Monitor system
ros2 node list
ros2 topic list
ros2 topic hz /Drone1/odom_local_ned

# Access monitoring
# Grafana: http://localhost:3000 (admin/P@ssw0rd)
# Prometheus: http://localhost:9090

# Troubleshooting
docker logs -f ros2-x11-node
ros2 doctor --report
```

### Critical Settings

- **Camera Naming**: `Camera_0`, `Camera_1`, `Camera_2`, `Camera_3` (capital C, underscore)
- **PX4 Ports**: 14540 + instance_id (Drone1=14540, Drone2=14541)
- **AirSim RPC**: Port 41451
- **ROS_DOMAIN_ID**: 0 (default, ensure all containers match)

---

**Last Updated:** 2025-10-15
**Version:** 1.0.0
**Maintainer:** AirSim Multi-Drone Team
**License:** MIT

For questions or issues, please open a GitHub issue or consult the documentation links above.
