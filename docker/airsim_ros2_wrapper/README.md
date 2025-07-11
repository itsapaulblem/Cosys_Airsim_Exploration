# 🚁 AirSim ROS2 Wrapper with VNC

**Containerized ROS2 development environment for AirSim with VNC remote desktop access**

This Docker container provides a complete ROS2 development environment for AirSim with graphical access through VNC. Perfect for developing and running ROS2 nodes that interface with AirSim simulation.

## 🚀 Quick Start

### Prerequisites

- Docker Desktop with docker-compose
- AirSim running on host (Unreal Engine)
- VNC viewer (TigerVNC, RealVNC, or any VNC client)

### Launch Container

```bash
# Navigate to wrapper directory
cd /path/to/Cosys-AirSim/docker/airsim_ros2_wrapper

# Start the VNC container
docker-compose up -d

# Check container status
docker-compose ps
```

### Connect via VNC

1. **Open your VNC client**
2. **Connect to**: `localhost:5901`
3. **Password**: `airsim`
4. **Resolution**: 1920x1080

You'll see an XFCE desktop with development tools ready!

## 🖥️ VNC Desktop Environment

### Desktop Shortcuts

The VNC desktop includes these convenient shortcuts:

- **🚁 AirSim ROS2** - Launch AirSim ROS2 wrapper nodes
- **📊 RViz2** - Open RViz2 for visualization  
- **💻 Terminal** - Open terminal in ROS2 workspace
- **📝 VSCodium** - Code editor (if installed)

### Pre-configured Environment

- **ROS2 Humble** fully configured
- **Workspace**: `/airsim_ros2_ws` (your working directory)
- **Auto-sourcing**: ROS2 and workspace automatically sourced
- **Build aliases**: Convenient build commands available

## 📁 Workspace Structure

```
/airsim_ros2_ws/                 # ROS2 workspace root
├── src/                         # Source code (mounted from host)
│   ├── airsim_interfaces/       # AirSim ROS2 interfaces
│   └── airsim_ros_pkgs/         # AirSim ROS2 packages
├── build/                       # Build artifacts (persisted)
├── install/                     # Install space (persisted)
└── log/                         # Build logs (persisted)
```

## 🔧 Development Workflow

### 1. Building ROS2 Packages

Open terminal in VNC desktop:

```bash
# Build entire workspace
colcon build --symlink-install

# Build specific packages
colcon build --packages-select airsim_interfaces
colcon build --packages-select airsim_ros_pkgs

# Use convenient aliases
build                    # Full workspace build
build_interfaces         # Build interfaces only
build_pkgs              # Build packages only
source_ws               # Source workspace
clean_build             # Clean and rebuild
```

### 2. Running AirSim ROS2 Nodes

**Method 1: Desktop Shortcut**
- Double-click "AirSim ROS2" desktop icon

**Method 2: Terminal**
```bash
# Launch AirSim ROS2 wrapper
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# Or run individual nodes
ros2 run airsim_ros_pkgs airsim_node
```

### 3. Using RViz2

**Method 1: Desktop Shortcut**
- Double-click "RViz2" desktop icon

**Method 2: Terminal**
```bash
# Launch RViz2
rviz2

# Or with specific config
rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/airsim.rviz
```

### 4. Testing Connection to AirSim

```bash
# Check AirSim connection
/debug_airsim_connection.sh

# Check ROS2 topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /airsim_node/drone_1/odom
ros2 topic echo /airsim_node/drone_1/global_gps
```

## 🔗 AirSim Integration

### Connection Settings

The container connects to AirSim running on your host:

- **AirSim Host**: `host.docker.internal` (automatically resolves to host)
- **AirSim Port**: `41451` (default AirSim API port)
- **ROS Domain**: `0` (default ROS2 domain)

### Supported AirSim Features

- **Multi-drone support**: Connect to multiple vehicles
- **Sensor data**: IMU, GPS, cameras, lidar
- **Control commands**: Position, velocity, attitude control
- **Image streaming**: Camera feeds via ROS2 topics
- **Transform broadcasting**: TF2 transforms for each vehicle

## 📊 Available ROS2 Topics

When connected to AirSim, you'll see topics like:

```bash
# Vehicle states
/airsim_node/drone_1/odom                    # Odometry
/airsim_node/drone_1/global_gps              # GPS position
/airsim_node/drone_1/imu                     # IMU data

# Camera feeds
/airsim_node/drone_1/front_center/Scene     # RGB camera
/airsim_node/drone_1/front_center/DepthVis  # Depth camera

# Control topics
/airsim_node/drone_1/pose_cmd                # Position commands
/airsim_node/drone_1/vel_cmd_body_frame      # Velocity commands
```

## 🔧 Customization

### Environment Variables

Modify `docker-compose.yml` to customize:

```yaml
environment:
  - VNC_PORT=5901          # VNC port (default: 5901)
  - RESOLUTION=1920x1080   # VNC resolution
  - USER=airsim            # Container user (default: airsim)
  - PASSWD=airsim          # VNC password (default: airsim)
  - ROS_DOMAIN_ID=0        # ROS2 domain (default: 0)
  - AIRSIM_HOST_IP=host.docker.internal  # AirSim host
  - AIRSIM_HOST_PORT=41451 # AirSim API port
```

### Persistent Data

Data is automatically persisted in Docker volumes:

- **vnc_home**: User home directory and settings
- **airsim_ros2_build**: Build artifacts
- **airsim_ros2_install**: Installed packages
- **airsim_ros2_logs**: Build and runtime logs

## 🚨 Troubleshooting

### VNC Connection Issues

```bash
# Check if VNC is running
docker-compose logs airsim-ros2-vnc

# Check VNC process inside container
docker exec airsim-ros2-vnc pgrep -f vnc

# Restart VNC service
docker-compose restart airsim-ros2-vnc
```

### AirSim Connection Issues

```bash
# Test AirSim connection from container
docker exec -it airsim-ros2-vnc /debug_airsim_connection.sh

# Check if AirSim is running on host
# Make sure AirSim is listening on 0.0.0.0:41451, not just localhost
```

### ROS2 Build Issues

```bash
# Enter container for debugging
docker exec -it airsim-ros2-vnc bash

# Check ROS2 environment
env | grep ROS

# Rebuild from scratch
rm -rf build install log
colcon build --symlink-install
```

### Common Solutions

1. **VNC Black Screen**: Wait 30 seconds for XFCE to start
2. **AirSim Not Found**: Ensure AirSim API is accessible from Docker
3. **Build Errors**: Check that ROS2 source is mounted correctly
4. **Permission Issues**: Restart container with `docker-compose restart`

## 🔄 Container Management

### Start/Stop Container

```bash
# Start in background
docker-compose up -d

# Stop container
docker-compose stop

# Restart container
docker-compose restart

# View logs
docker-compose logs -f
```

### Enter Container

```bash
# Enter as default user
docker exec -it airsim-ros2-vnc bash

# Enter as root
docker exec -it -u root airsim-ros2-vnc bash
```

### Clean Up

```bash
# Remove container and networks
docker-compose down

# Remove container, networks, and volumes
docker-compose down -v

# Remove everything including images
docker-compose down -v --rmi all
```

## 📝 Development Tips

### Code Editing

1. **In VNC**: Use desktop applications (VSCodium, gedit)
2. **On Host**: Edit source files directly (they're mounted)
3. **CLI**: Use vim/nano in terminal

### Debugging

```bash
# Monitor all ROS2 topics
ros2 topic list

# Check node status
ros2 node list
ros2 node info /airsim_node

# View TF tree
ros2 run tf2_tools view_frames.py
```

### Performance

- VNC resolution affects performance
- Use lower resolution for better responsiveness: `1280x720`
- Close unused applications in VNC desktop

## 🔧 Adding New Services, Messages, and Actions

This container supports seamless development of new ROS2 interfaces. For complete instructions, see `/ros2/README_Adding_Services_Messages.md`.

### Quick Development Workflow

1. **Create Interface Definitions** (on host)
   ```bash
   # Add new .srv, .msg, or .action files to:
   # ros2/src/airsim_interfaces/srv/
   # ros2/src/airsim_interfaces/msg/
   # ros2/src/airsim_interfaces/action/
   
   # Update CMakeLists.txt to register new interfaces
   ```

2. **Build Interfaces** (in VNC container)
   ```bash
   # Use convenient aliases
   build_interfaces    # Build only interface package
   source_ws          # Source workspace
   ```

3. **Implement Service/Action Handlers** (on host)
   ```bash
   # Edit files in ros2/src/airsim_ros_pkgs/
   # Update headers in include/airsim_ros_wrapper.h
   # Add implementations in src/airsim_ros_wrapper.cpp
   ```

4. **Build and Test** (in VNC container)
   ```bash
   # Build main package
   build_pkgs
   source_ws
   
   # Launch AirSim ROS2 wrapper
   ./launch_airsim_ros2.sh
   
   # Test your new interfaces
   ros2 service list | grep your_service
   ros2 action list | grep your_action
   ros2 topic list | grep your_topic
   ```

### Development Aliases Available

- `build` - Build entire workspace
- `build_interfaces` - Build only airsim_interfaces package  
- `build_pkgs` - Build only airsim_ros_pkgs package
- `source_ws` - Source the workspace
- `clean_build` - Clean and rebuild everything

### Volume Mounting

The container automatically mounts your local source code:
- Host: `ros2/src/` → Container: `/airsim_ros2_ws/src/`
- Changes on host are immediately reflected in container
- No need to rebuild Docker image for code changes

## 🚨 Common Issues and Solutions

### GPS Home Location Error

If you encounter: `Vehicle does not have a valid GPS home location`

**Cause**: AirSim hasn't established a GPS home position before arming

**Solutions**:

1. **Ensure GPS is configured** in your `settings.json`:
   ```json
   "Sensors": {
     "Gps": {
       "SensorType": 3,
       "Enabled": true,
       "StartLatitude": 47.641468,
       "StartLongitude": -122.140165,
       "StartAltitude": 122.0
     }
   }
   ```

2. **Wait for GPS fix** before arming:
   ```bash
   # Check GPS status
   ros2 topic echo /airsim_node/drone_1/global_gps
   
   # Verify position data is being published
   ```

3. **Disable auto-arming** if needed:
   ```bash
   # Launch with API control disabled
   ros2 launch airsim_ros_pkgs airsim_node.launch.py enable_api_control:=false
   
   # Manually arm after GPS is ready
   ros2 service call /airsim_node/drone_1/arm_disarm airsim_interfaces/srv/SetArm "{arm: true}"
   ```

### Container Connection Issues

```bash
# Test AirSim connection from container
docker exec -it airsim-ros2-vnc /debug_airsim_connection.sh

# Check if AirSim is accessible on host
netstat -an | grep 41451

# Ensure AirSim binds to all interfaces (not just localhost)
# In AirSim settings.json:
"LocalHostIp": "0.0.0.0"
```

## 🌐 Network Configuration

The container runs on a custom Docker network:

- **Network**: `airsim-network`
- **Subnet**: `172.25.0.0/16`
- **Host Communication**: via `host.docker.internal`

This ensures proper isolation while maintaining connectivity to AirSim.

---

## 🎯 Quick Reference

| Component | Access | Default |
|-----------|--------|---------|
| VNC Desktop | `localhost:5901` | Password: `airsim` |
| Workspace | `/airsim_ros2_ws` | Auto-mounted source |
| ROS2 Domain | `ROS_DOMAIN_ID` | `0` |
| AirSim API | `host.docker.internal:41451` | Auto-configured |

**Perfect for**: ROS2 development, testing AirSim integration, running simulations with visual feedback!

---

*This container provides a complete, ready-to-use ROS2 development environment for AirSim with the convenience of remote desktop access.*