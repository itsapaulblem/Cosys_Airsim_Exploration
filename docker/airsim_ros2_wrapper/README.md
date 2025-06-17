# AirSim ROS2 Docker Wrapper - Complete Guide

This repository provides a comprehensive Docker-based solution for integrating AirSim with ROS2 Humble, enabling seamless drone simulation, control, and sensor data processing.

## 🚀 Quick Start

### Prerequisites
- **Windows**: Docker Desktop with WSL2 support
- **AirSim**: Unreal Engine environment with AirSim plugin
- **Hardware**: 8GB+ RAM, dedicated GPU recommended

### 🔑 Key Features
- **Automatic API Control**: API control is enabled by default - drones are automatically armed and ready for commands
- **Multi-Drone Support**: Control multiple drones simultaneously
- **Full Sensor Integration**: Camera, LiDAR, IMU, GPS, and more
- **RViz2 Visualization**: Optional 3D visualization support

### 1. Choose Your Version

We provide two optimized Docker configurations:

| Version | Use Case | Size | Features |
|---------|----------|------|----------|
| **Simple** | Development & Full ROS2 | ~3-4GB | Full ROS2 desktop, development tools |
| **Minimal** | Production & CI/CD | ~2-3GB | Essential packages only |

### 2. Quick Launch

```bash
# For development (recommended)
./run_simple.bat

# For production/minimal setup
./run_minimal.bat
```

## 📁 Directory Structure

```
docker/airsim_ros2_wrapper/
├── Dockerfile.simple          # Full ROS2 development environment
├── Dockerfile.minimal         # Lightweight production environment
├── docker-compose.yml         # Docker Compose configuration
├── run_simple.bat            # Windows launcher for Simple version
├── run_minimal.bat           # Windows launcher for Minimal version
├── test_build.bat            # Build test script
├── test_minimal_build.bat    # Minimal build test script
├── test_ros2_connection.sh   # ROS2 connection test
├── launch_airsim_ros2_simple.sh  # Launch script (copied to container)
└── .dockerignore             # Build optimization (excludes Unreal files)
```

## 🔧 Detailed Setup Instructions

### Step 1: Configure AirSim Settings

Create or modify `C:\Users\<YourUser>\Documents\AirSim\settings.json`:

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            "PawnBP": "class '/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"
        }
    },
    "CameraDefaults": {
        "CaptureSettings": [
            {
                "ImageType": 0,
                "Width": 640,
                "Height": 480,
                "FOV_Degrees": 90
            }
        ]
    }
}
```

### Step 2: Build Docker Image

Choose your preferred method:

#### Option A: Using Batch Scripts (Recommended)
```bash
# Build Simple version (full development)
./test_build.bat

# Build Minimal version (lightweight)
./test_minimal_build.bat
```

#### Option B: Manual Docker Build
```bash
# Simple version
docker build -f Dockerfile.simple -t airsim_ros2_simple:latest .

# Minimal version
docker build -f Dockerfile.minimal -t airsim_ros2_minimal:latest .
```

#### Option C: Docker Compose
```bash
docker-compose up --build
```

### Step 3: Launch AirSim

1. Start your AirSim environment (e.g., Blocks.exe)
2. Wait for AirSim to fully load
3. Ensure no firewall blocking on port 41451

### Step 4: Run ROS2 Container

```bash
# Simple version (recommended for development)
./run_simple.bat

# Minimal version (for production)
./run_minimal.bat
```

### Step 5: Verify Connection

Inside the container, test the connection:
```bash
./test_ros2_connection.sh
```

## 🎮 ROS2 Services and Topics Guide

### 📡 Available Topics

#### **Sensor Data Topics**
```bash
# Drone State & Navigation
/airsim_node/Drone1/odom_local_ned          # Odometry (position, velocity, orientation)
/airsim_node/Drone1/global_gps              # GPS coordinates
/airsim_node/Drone1/environment             # Environmental data

# Camera Topics
/airsim_node/Drone1/front_center/Scene                    # RGB camera feed
/airsim_node/Drone1/front_center/DepthPerspective        # Depth camera
/airsim_node/Drone1/front_center/Segmentation            # Segmentation mask
/airsim_node/Drone1/front_center/SurfaceNormals          # Surface normals
/airsim_node/Drone1/front_center/camera_info             # Camera calibration

# Sensor Topics
/airsim_node/Drone1/imu/imu                 # IMU data (acceleration, gyroscope)
/airsim_node/Drone1/magnetometer/mag       # Magnetometer
/airsim_node/Drone1/barometer/alt          # Barometer/altimeter
/airsim_node/Drone1/gps/gps                # GPS sensor
/airsim_node/Drone1/lidar/points           # LiDAR point cloud
/airsim_node/Drone1/distance/distance      # Distance sensor
```

#### **Control Topics**
```bash
# Single Drone Control
/airsim_node/Drone1/vel_cmd_body_frame      # Velocity commands (body frame)
/airsim_node/Drone1/vel_cmd_world_frame     # Velocity commands (world frame)

# Multi-Drone Control
/airsim_node/all_robots/vel_cmd_body_frame  # Control all drones (body frame)
/airsim_node/all_robots/vel_cmd_world_frame # Control all drones (world frame)
/airsim_node/group_of_robots/vel_cmd_body_frame  # Control drone group
```

### 🛠️ Available Services

#### **Flight Control Services**
```bash
# Individual Drone Control
/airsim_node/Drone1/takeoff                 # Takeoff service
/airsim_node/Drone1/land                    # Landing service

# Multi-Drone Control
/airsim_node/all_robots/takeoff             # Takeoff all drones
/airsim_node/all_robots/land                # Land all drones
/airsim_node/group_of_robots/takeoff        # Takeoff drone group
/airsim_node/group_of_robots/land           # Land drone group

# System Control
/airsim_node/reset                          # Reset simulation
```

#### **Position Control Services**
```bash
/airsim_node/local_position_goal            # Set local position target
/airsim_node/gps_goal                       # Set GPS position target
```

#### **Sensor Services**
```bash
/airsim_node/Drone1/instance_segmentation_refresh  # Refresh segmentation
/airsim_node/Drone1/object_transforms_refresh      # Refresh object transforms
```

## 💻 Usage Examples

### Basic Drone Control

#### 1. List Available Topics
```bash
ros2 topic list
```

#### 2. Monitor Drone State
```bash
# View drone odometry
ros2 topic echo /airsim_node/Drone1/odom_local_ned

# View GPS data
ros2 topic echo /airsim_node/Drone1/global_gps

# View camera feed info
ros2 topic echo /airsim_node/Drone1/front_center/camera_info
```

#### 3. Control Drone Movement

**Takeoff:**
```bash
ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
```

**Move Forward (Body Frame):**
```bash
ros2 topic pub /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd "
twist:
  linear:
    x: 2.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"
```

**Move in World Frame:**
```bash
ros2 topic pub /airsim_node/Drone1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd "
twist:
  linear:
    x: 1.0
    y: 1.0
    z: -0.5
  angular:
    x: 0.0
    y: 0.0
    z: 0.1
"
```

**Land:**
```bash
ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land "{waitOnLastTask: true, timeout_sec: 10.0}"
```

#### 4. Position Control
```bash
# Set local position target (x, y, z in meters)
ros2 service call /airsim_node/local_position_goal airsim_interfaces/srv/SetLocalPosition "
position:
  x: 10.0
  y: 5.0
  z: -20.0
yaw: 1.57
"
```

#### 5. Multi-Drone Control

**Takeoff All Drones:**
```bash
ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff "{waitOnLastTask: true, timeout_sec: 15.0}"
```

**Control Drone Group:**
```bash
ros2 topic pub /airsim_node/group_of_robots/vel_cmd_body_frame airsim_interfaces/msg/VelCmdGroup "
vehicle_names: ['Drone1', 'Drone2']
vel_cmd:
  twist:
    linear:
      x: 1.0
      y: 0.0
      z: 0.0
"
```

### Advanced Sensor Usage

#### 1. LiDAR Point Cloud
```bash
# View LiDAR data
ros2 topic echo /airsim_node/Drone1/lidar/points

# Save point cloud to file
ros2 bag record /airsim_node/Drone1/lidar/points
```

#### 2. Camera Data Processing
```bash
# View camera images using rqt
rqt_image_view

# Record camera data
ros2 bag record /airsim_node/Drone1/front_center/Scene
```

#### 3. IMU and GPS Integration
```bash
# Monitor IMU data
ros2 topic echo /airsim_node/Drone1/imu/imu

# Monitor GPS coordinates
ros2 topic echo /airsim_node/Drone1/gps/gps
```

## 🐛 Troubleshooting

### Common Issues

#### 1. ROS2 Command Not Found (`ros2: not found`)
If you get `ros2: not found` when using `docker exec`:

**Solution A - Use the helper script (Recommended):**
```bash
# For interactive shell
./ros2_exec.bat

# For single commands
./ros2_exec.bat "ros2 topic list"
./ros2_exec.bat "ros2 node list"
```

**Solution B - Use bash explicitly:**
```bash
docker exec -it airsim_ros2_container bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && ros2 topic list"
```

**Solution C - Enter interactive bash:**
```bash
docker exec -it airsim_ros2_container bash
# Then inside container: ros2 topic list
```

#### 2. Connection Problems
```bash
# Check if AirSim is running and accessible
nc -z host.docker.internal 41451

# Test ROS2 connection inside container
./test_ros2_connection.sh
```

#### 2. No Topics Appearing
```