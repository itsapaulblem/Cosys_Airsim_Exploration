# AirSim ROS2 Docker Wrapper - Complete Guide

A comprehensive Docker-based solution for integrating AirSim with ROS2 Humble, enabling seamless drone simulation, control, and sensor data processing.

## 🚀 Quick Start

### Prerequisites
- **Windows**: Docker Desktop with WSL2 support
- **AirSim**: Unreal Engine environment with AirSim plugin
- **Hardware**: 8GB+ RAM, dedicated GPU recommended

### Available Docker Configurations

| Version | Use Case | Size | Features | File |
|---------|----------|------|----------|------|
| **Simple** | Development & Full ROS2 | ~3-4GB | Full ROS2 desktop, development tools | `Dockerfile.simple` |
| **VNC** | GUI Applications (RViz2) | ~4-5GB | VNC desktop + software rendering | `Dockerfile.simple-with-vnc` |
| **Minimal** | Production & CI/CD | ~2-3GB | Essential packages only | `Dockerfile.minimal` |

### Quick Launch Commands

```bash
# For development (recommended)
./run_simple.bat

# For GUI applications (RViz2)
./run_vnc.bat

# For production/minimal setup
./run_headless.bat
```

## 📁 Docker Files and Scripts

### Core Docker Files
- **`Dockerfile.simple`** - Full ROS2 development environment
- **`Dockerfile.simple-with-vnc`** - GUI-enabled version with VNC desktop
- **`docker-compose.yml`** - Docker Compose configuration

### Windows Batch Scripts
- **`build_vnc.bat`** - Build VNC-enabled Docker image
- **`run_simple.bat`** - Run simple version container
- **`run_vnc.bat`** - Run VNC version with GUI support
- **`run_headless.bat`** - Run without GUI
- **`test_build.bat`** - Test Simple build
- **`ros2_exec.bat`** - Execute ROS2 commands in container

### Linux Scripts
- **`launch_airsim_ros2_simple.sh`** - Main launch script (copied to container)
- **`launch_airsim_ros2_with_rviz.sh`** - Launch with RViz2 support
- **`test_ros2_connection.sh`** - Test ROS2 connection

## 🔧 Installation & Setup

### Step 1: Configure AirSim Settings

Create or modify `C:\Users\<YourUser>\Documents\AirSim\settings.json`:

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "ApiServerEndpoint": "0.0.0.0:41451",
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

Choose your build method:

#### Option A: Using Batch Scripts (Recommended)
```bash
# Build Simple version (full development)
./test_build.bat

# Build VNC version (with GUI)
./build_vnc.bat
```

#### Option B: Manual Docker Build
```bash
# Change to project root first
cd /path/to/Cosys-AirSim

# Simple version
docker build -f docker/airsim_ros2_wrapper/Dockerfile.simple -t airsim_ros2_simple:latest .

# VNC version
docker build -f docker/airsim_ros2_wrapper/Dockerfile.simple-with-vnc -t airsim_ros2_vnc:latest .
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

# VNC version (for GUI applications like RViz2)
./run_vnc.bat

# Headless version (for production)
./run_headless.bat
```

## 🎮 Usage Examples

### Basic Commands

#### List Available Topics and Services
```bash
# Using helper script (recommended)
./ros2_exec.bat "ros2 topic list"
./ros2_exec.bat "ros2 service list"
./ros2_exec.bat "ros2 node list"

# Interactive shell
./ros2_exec.bat
# Then inside container: ros2 topic list
```

#### Monitor Drone Data
```bash
# Monitor position and orientation
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom_local_ned"

# Monitor GPS data
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/global_gps"

# Monitor IMU data
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/imu/imu"

# Monitor camera feed info
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/front_center/camera_info"
```

### Drone Control Commands

#### ✅ CORRECT Service Syntax (Use underscores, not camelCase)

```bash
# Individual Drone Control
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"

# Multi-Drone Control
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"
```

#### Velocity Control
```bash
# Move forward (body frame)
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}}}'"

# Move in world frame
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 1.0, y: 1.0, z: -0.5}}}'"
```

### Available Topics and Services

#### Sensor Data Topics
```bash
# Navigation
/airsim_node/Drone1/odom_local_ned          # Odometry (position, velocity, orientation)
/airsim_node/Drone1/global_gps              # GPS coordinates
/airsim_node/Drone1/environment             # Environmental data

# Camera Topics
/airsim_node/Drone1/front_center/Scene                    # RGB camera feed
/airsim_node/Drone1/front_center/DepthPerspective        # Depth camera
/airsim_node/Drone1/front_center/Segmentation            # Segmentation mask
/airsim_node/Drone1/front_center/camera_info             # Camera calibration

# Sensors
/airsim_node/Drone1/imu/imu                 # IMU data (acceleration, gyroscope)
/airsim_node/Drone1/magnetometer/mag       # Magnetometer
/airsim_node/Drone1/barometer/alt          # Barometer/altimeter
/airsim_node/Drone1/gps/gps                # GPS sensor
/airsim_node/Drone1/lidar/points           # LiDAR point cloud
/airsim_node/Drone1/distance/distance      # Distance sensor
```

#### Control Services
```bash
# Flight Control
/airsim_node/Drone1/takeoff                 # Takeoff service
/airsim_node/Drone1/land                    # Landing service
/airsim_node/all_robots/takeoff             # Takeoff all drones
/airsim_node/all_robots/land                # Land all drones

# Position Control
/airsim_node/local_position_goal            # Set local position target
/airsim_node/gps_goal                       # Set GPS position target

# System Control
/airsim_node/reset                          # Reset simulation
```

## 🖥️ GUI Applications (RViz2)

### Using VNC Version

1. **Build VNC Image**: `./build_vnc.bat`
2. **Run VNC Container**: `./run_vnc.bat`
3. **Access GUI**:
   - **Web Browser**: `http://localhost:6901/vnc.html`
   - **VNC Viewer**: `localhost:5901`
   - **Password**: `airsim123`

### RViz2 will automatically launch with:
- AirSim ROS2 node
- 3D visualization of drone
- Sensor data visualization
- Topic monitoring

## 🐛 Troubleshooting

### Common Issues

#### Container Not Running
```bash
# Check container status
docker ps

# View container logs
docker logs airsim_ros2_container
```

#### ROS2 Command Not Found
```bash
# Use helper script (automatically sets environment)
./ros2_exec.bat "ros2 topic list"

# Or use explicit environment setup
docker exec -it airsim_ros2_container bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && ros2 topic list"
```

#### Connection Issues
- Ensure AirSim is running before starting container
- Check if port 41451 is accessible
- Try setting custom host IP: `docker run -e AIRSIM_HOST_IP=192.168.1.100 ...`

#### Service Call Errors
- Use **underscores** not camelCase: `wait_on_last_task` not `waitOnLastTask`
- Check service structure: `./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/Takeoff"`

### Performance Optimization

#### Build Context Optimization
The `.dockerignore` file excludes large Unreal Engine files:
- **Before**: 22.19GB transfer time
- **After**: ~2GB transfer time
- **Improvement**: 90% reduction in build time

#### Environment Variables
```bash
# Set ROS domain ID
-e ROS_DOMAIN_ID=42

# Set AirSim host
-e AIRSIM_HOST_IP=192.168.1.100
-e AIRSIM_HOST_PORT=41451

# Enable RViz2
-e LAUNCH_RVIZ=true
```

## 📊 Available Docker Configurations

### 1. Simple Development (Dockerfile.simple)
```bash
./run_simple.bat
```
- Full ROS2 Humble desktop
- All AirSim packages
- Development tools
- ~3-4GB size

### 2. VNC GUI Support (Dockerfile.simple-with-vnc)
```bash
./run_vnc.bat
```
- Everything from Simple +
- VNC desktop environment
- RViz2 3D visualization
- Software OpenGL rendering
- ~4-5GB size

### 3. Minimal Production (run via docker-compose)
```bash
docker-compose up
```
- Essential packages only
- Optimized for production
- ~2-3GB size

## 🔗 Integration with External Systems

### Multi-Machine Setup
See `AirSim_Multi_Machine_Setup.md` for:
- Network configuration
- Distributed ROS2 setup
- Firewall settings

### PX4 Autopilot Integration
See `docker/px4_airsim_docker/` for:
- PX4 SITL setup
- MAVLink integration
- Hardware-in-the-loop (HITL)

### Custom Service Development
See `CUSTOM_SERVICE_DEVELOPMENT_GUIDE.md` for:
- Creating custom ROS2 services (FlyCircle, ChangeAltitude, etc.)
- Step-by-step development workflow
- Complete code examples and testing

---

**Key Features:**
- ✅ Automatic API control (drones auto-arm on startup)
- ✅ Multi-drone support
- ✅ Complete sensor integration
- ✅ GUI visualization with RViz2
- ✅ Production-ready deployment
- ✅ Cross-platform compatibility (Windows/Linux)