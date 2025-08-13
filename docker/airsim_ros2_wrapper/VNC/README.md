# 🖥️ VNC Files for AirSim ROS2 Wrapper

This directory contains the core VNC implementation files for the AirSim ROS2 wrapper.

## 📁 Files Overview

### Core Files
- **`Dockerfile.ros2_vnc`** - Main Dockerfile for VNC-enabled ROS2 environment
- **`entrypoint_lightweight.sh`** - Container entry point and VNC setup
- **`launch_airsim_ros2.sh`** - Script to launch AirSim ROS2 nodes
- **`debug_airsim_connection.sh`** - Connection testing utility

### Helper Scripts
- **`airsim_docker.bat`** - Windows batch launcher
- **`airsim-docker.sh`** - Linux shell launcher

## 🚀 Usage

These files are automatically used when running:

```bash
cd /path/to/Cosys-AirSim/docker/airsim_ros2_wrapper
docker-compose up -d
```

The VNC desktop will be available at `localhost:5901` with password `airsim`.

## 🔧 Key Features

- **XFCE Desktop Environment** - Full graphical desktop in container
- **ROS2 Humble** - Pre-configured ROS2 environment
- **Volume Mounting** - Source code mounted from host
- **Desktop Shortcuts** - Convenient launchers for AirSim ROS2 and RViz2
- **Auto-restart** - VNC server automatically restarts if it crashes

---

*These files provide the VNC implementation that enables graphical access to the ROS2 development environment.*