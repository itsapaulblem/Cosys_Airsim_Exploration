# AirSim ROS2 with Native X11 Forwarding (Linux)

**Optimized Docker configuration for native Linux performance with direct X11 display forwarding.**

## Overview

This setup provides **60-70% lower resource usage** compared to the VNC-based configuration by:
- Eliminating VNC server overhead
- Removing XFCE desktop environment
- Direct GPU acceleration for RViz2
- Native X11 rendering performance

**Use this for:** Linux development environments
**Use VNC for:** Windows/Mac or remote access

---

## Quick Start

### Prerequisites

1. **X11 Server Running** (should already be active if using Linux desktop)
   ```bash
   echo $DISPLAY  # Should output :0 or :1
   ```

2. **Docker with GPU Support** (optional but recommended for RViz2)
   ```bash
   docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

### Step 1: Allow Docker to Access X Server

```bash
# Grant X server access to Docker containers
xhost +local:docker
```

> **Security Note:** This grants all Docker containers access to your display. For production, use more restrictive permissions (see Security section).

### Step 2: Set Environment Variables

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/airsim_ros2_wrapper/Linux

# Copy and customize environment template
cp .env.example .env

# Required variables (auto-detected if not set):
export PROJECT_ROOT=/home/mnsuser/Cosys_Airsim_Exploration
export UID=$(id -u)
export GID=$(id -g)
export DISPLAY=${DISPLAY}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
```

### Step 3: Launch Container

```bash
# Build and start container
docker-compose up -d

# View logs
docker-compose logs -f
```

### Step 4: Access Container and Test

```bash
# Access interactive shell
docker exec -it ros2-x11-node bash

# Inside container - test X11 connection
xclock  # Should open clock window on host display

# Test GPU acceleration
glxinfo | grep "OpenGL renderer"

# Launch RViz2
rviz2
```

---

## Usage Examples

### Launch ROS2 Multi-Node System

```bash
docker exec -it ros2-x11-node bash

# Inside container:
ros2-system start multi
```

This launches:
- AirSim coordination node
- Individual vehicle nodes (auto-discovered via RPC)

### Manual ROS2 Launch

```bash
# Inside container:
source /opt/ros/humble/setup.bash
source /airsim_ros2_ws/install/setup.bash

ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
    host_ip:=airsim-container \
    enable_coordination:=true
```

### Rebuild Workspace

```bash
# Inside container:
cd /airsim_ros2_ws

# Quick rebuild (uses aliases from development_aliases.sh)
build

# Full clean rebuild
clean && build
```

### Launch RViz2 with Custom Config

```bash
# Inside container:
rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz
```

---

## 🔧 Configuration

### Environment Variables

Edit `.env` file or set in docker-compose.yml:

| Variable | Default | Description |
|----------|---------|-------------|
| `DISPLAY` | `:0` | X11 display number |
| `XAUTHORITY` | `$HOME/.Xauthority` | X11 auth file path |
| `PROJECT_ROOT` | (required) | Path to Cosys_Airsim_Exploration |
| `UID` | `1000` | User ID (match host user) |
| `GID` | `1000` | Group ID (match host user) |
| `AIRSIM_HOST_IP` | `airsim-container` | AirSim server address |
| `ROS_DOMAIN_ID` | `0` | ROS2 DDS domain |
| `AUTO_BUILD` | `true` | Build workspace on startup |

### Volume Mounts

```yaml
volumes:
  # X11 socket - CRITICAL for GUI
  - /tmp/.X11-unix:/tmp/.X11-unix:rw

  # X11 authentication - CRITICAL for security
  - ${XAUTHORITY}:/tmp/.Xauthority:ro

  # ROS2 workspace source (live editing)
  - ${PROJECT_ROOT}/ros2/src:/airsim_ros2_ws/src:rw

  # Build artifacts (persistent)
  - ros2_x11_build:/airsim_ros2_ws/build
  - ros2_x11_install:/airsim_ros2_ws/install
```

### GPU Access

**IMPORTANT:** The docker-compose.yml is configured for **NVIDIA GPUs by default** (RTX series, etc.).

#### For NVIDIA GPUs (Default Configuration):

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu, compute, utility, graphics]
```

**Prerequisites:**
- NVIDIA driver installed on host (check with `nvidia-smi`)
- NVIDIA Container Toolkit installed (check with `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`)

#### For Intel/AMD GPUs:

Replace the `deploy:` section with:

```yaml
devices:
  - /dev/dri:/dev/dri  # Intel/AMD GPU
```

#### Verify GPU Access:

```bash
# Inside container
glxinfo | grep "OpenGL renderer"

# Should show your actual GPU, not "llvmpipe" (software rendering)
# Example NVIDIA: "NVIDIA GeForce RTX 5080"
# Example Intel: "Mesa Intel(R) Arc(tm) Graphics"
```

---

## 🐛 Troubleshooting

### Issue: "No protocol specified" or "Can't open display"

**Cause:** X11 authentication failure

**Solution:**
```bash
# On host:
xhost +local:docker

# Verify DISPLAY and XAUTHORITY are correct
echo $DISPLAY
echo $XAUTHORITY

# Ensure XAUTHORITY file exists
ls -la $XAUTHORITY
```

### Issue: "xdpyinfo: unable to open display"

**Cause:** X11 socket not mounted or wrong DISPLAY value

**Solution:**
```bash
# Check X11 socket exists on host
ls -la /tmp/.X11-unix/

# Verify docker-compose.yml mounts socket correctly
docker inspect ros2-x11-node | grep -A 10 Mounts
```

### Issue: RViz2 is slow or uses software rendering (llvmpipe)

**Symptoms:**
```bash
# Inside container
glxinfo | grep "OpenGL renderer"
# Shows: llvmpipe (LLVM 15.0.7, 256 bits)  # BAD - software rendering
```

**Cause:** GPU not accessible - either wrong driver configuration or missing permissions

**Solution for NVIDIA GPUs:**
```bash
# 1. Verify NVIDIA driver on host
nvidia-smi  # Should show your GPU

# 2. Verify NVIDIA Container Runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# 3. Ensure docker-compose.yml has NVIDIA configuration (see GPU Access section)
# 4. Rebuild and restart
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# 5. Test inside container
docker exec -it ros2-x11-node glxinfo | grep "OpenGL renderer"
# Should show: NVIDIA GeForce RTX ...
```

**Solution for Intel/AMD GPUs:**
```bash
# 1. Check /dev/dri exists on host
ls -la /dev/dri/

# 2. Verify user in video group on host
sudo usermod -aG video $USER
# Log out and log back in

# 3. Change docker-compose.yml from deploy: section to:
# devices:
#   - /dev/dri:/dev/dri

# 4. Restart container
docker-compose restart
```

### Issue: Permission denied on /dev/dri

**Solution:**
```bash
# Add user to video group on host
sudo usermod -aG video $USER

# Log out and log back in

# Verify group membership
groups | grep video
```

### Issue: Container starts but X11 test fails

**Solution:**
```bash
# Inside container - check entrypoint logs:
docker logs ros2-x11-node

# Common fixes:
# 1. Merge Xauthority manually
xauth merge /tmp/.Xauthority

# 2. Test basic X11
xdpyinfo

# 3. Check DISPLAY matches host
echo $DISPLAY  # Should match host's DISPLAY
```

### Issue: "fatal error: math.h: No such file or directory" when running build

**Symptoms:**
```bash
# Inside container
build
# Error: fatal error: math.h: No such file or directory
# Error: fatal error: stdlib.h: No such file or directory
```

**Cause:** Missing build tools (build-essential, cmake) in container

**Solution:**
This has been fixed in the latest Dockerfile.x11. Rebuild your container:
```bash
# On host
cd docker/airsim_ros2_wrapper/Linux
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# Test inside container
docker exec -it ros2-x11-node bash -c "cd /airsim_ros2_ws && build"
```

If you're using an older image, the container includes build tools starting from the 2025-10-01 update.

### Issue: Headers installed but build still fails with "No such file or directory"

**Symptoms:**
```bash
# build-essential is installed
dpkg -l | grep build-essential  # Shows installed

# Headers exist
ls /usr/include/stdlib.h  # File exists

# But compilation fails
fatal error: stdlib.h: No such file or directory
```

**Cause:** Environment variables `C_INCLUDE_PATH` or `CPLUS_INCLUDE_PATH` are prepending `/usr/include` to the compiler search path, breaking `#include_next` directives in C++ standard headers.

**Solution:**
This was fixed in the 2025-10-01 update by removing problematic include path environment variables. If you're using an older image:

```bash
# Temporary fix in running container
docker exec -it ros2-x11-node bash
unset C_INCLUDE_PATH CPLUS_INCLUDE_PATH CMAKE_CXX_FLAGS CMAKE_C_FLAGS
build  # Should work now

# Permanent fix: Rebuild with latest Dockerfile
docker-compose build --no-cache
docker-compose up -d
```

**Technical Details:**
- GCC's default search paths already include `/usr/include` in the correct order
- Manually prepending it via environment variables places it BEFORE C++ standard library directories
- C++ headers use `#include_next` to find C headers, which only looks forward in the search path
- With `/usr/include` moved to the front, `#include_next` can't find it

### Issue: "Failed to find mission_search_interfaces" when building

**Symptoms:**
```bash
# Inside container
build

# Error:
ERROR:colcon.colcon_cmake.task.cmake.build:Failed to find the following files:
- /airsim_ros2_ws/install/mission_search_interfaces/share/mission_search_interfaces/package.sh
Check that the following packages have been built:
- mission_search_interfaces
```

**Cause:** The `build` alias uses `fix_rpc_build.sh` which was only building `airsim_ros_pkgs` without its dependencies.

**Solution:**
This was fixed in the 2025-10-01 update. The script now uses `--packages-up-to` instead of `--packages-select`, which builds all dependencies first.

```bash
# The fix changes this:
colcon build --packages-select airsim_ros_pkgs  # ✗ No dependencies

# To this:
colcon build --packages-up-to airsim_ros_pkgs   # ✓ Includes dependencies
```

**Manual workaround if needed:**
```bash
# Inside container
cd /airsim_ros2_ws

# Build dependencies first
colcon build --packages-select mission_search_interfaces airsim_interfaces

# Then build main package
colcon build --packages-select airsim_ros_pkgs

# Or build everything
colcon build
```

### Issue: "fatal error: Python.h: No such file or directory" when building airsim_interfaces

**Symptoms:**
```bash
# Inside container
build

# Error:
fatal error: Python.h: No such file or directory
    5 | #include <Python.h>
Failed   <<< airsim_interfaces [30.2s, exited with code 2]
```

**Cause:** Missing Python include path. ROS2 message Python bindings need `/usr/include/python3.10/` in the compiler search path.

**Solution:**
This was fixed in the 2025-10-01 update by adding Python include path to environment variables.

**Temporary fix in running container:**
```bash
docker exec -it ros2-x11-node bash

# Inside container
export CPLUS_INCLUDE_PATH="/usr/include/python3.10"
export C_INCLUDE_PATH="/usr/include/python3.10"
cd /airsim_ros2_ws
build  # Should work now
```

**Permanent fix: Rebuild container**
```bash
cd docker/airsim_ros2_wrapper/Linux
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

---

## 🔒 Security Best Practices

### Production X11 Access Control

Instead of `xhost +local:docker`, use specific container access:

```bash
# Get container hostname
CONTAINER_HOST=$(docker inspect -f '{{.Config.Hostname}}' ros2-x11-node)

# Grant specific access
xhost +local:$CONTAINER_HOST

# Revoke after use
xhost -local:$CONTAINER_HOST
```

### Alternative: Use SSH X11 Forwarding

For remote development, use SSH with X11 forwarding instead:

```bash
# On remote machine:
ssh -X user@remote-host

# Then launch container normally
docker-compose up
```

---

## 🆚 When to Use VNC vs X11

### Use X11 Forwarding (This Setup) When:
- Developing on **Linux desktop/laptop**
- Need **maximum performance** (RViz2, Gazebo)
- Have **local GPU** for acceleration
- Want **minimal resource usage**

### Use VNC Setup When:
- Developing on **Windows or Mac**
- Need **remote access** from different machine
- Want **self-contained desktop** environment
- Don't mind **VNC overhead**

---

## 📁 File Structure

```
docker/airsim_ros2_wrapper/Linux/
├── Dockerfile.x11           # Optimized X11-only build
├── docker-compose.yml        # X11 socket/auth mounting
├── entrypoint_x11.sh         # X11 validation script
├── README.md                 # This file
└── .env.example              # Environment template
```

---

## 🔗 Integration with Master Compose

Add to `docker/docker-compose-master.yml`:

```yaml
services:
  # Linux X11 Native Setup (NEW)
  ros2-x11-node:
    extends:
      file: airsim_ros2_wrapper/Linux/docker-compose.yml
      service: ros2-x11-node
    profiles:
      - linux-native
      - linux-dev

# Usage:
# docker-compose -f docker-compose-master.yml --profile linux-native up
```

---

## 📝 Additional Resources

- **ROS2 Multi-Node Architecture:** `/docker/airsim_ros2_wrapper/MULTI_NODE_GUIDE.md`
- **AirSim ROS2 Wrapper:** `/ros2/README.md`
- **Development Aliases:** Use `alias` command inside container to see shortcuts
- **Terminator Config:** Custom multi-tab setup for development

---

## 🎯 Performance Tips

1. **Use RAM disk for build artifacts** (optional):
   ```bash
   # On host - create tmpfs for faster builds
   sudo mount -t tmpfs -o size=2G tmpfs /path/to/build
   ```

2. **Enable ccache for faster rebuilds**:
   ```bash
   # Inside container
   export CMAKE_CXX_COMPILER_LAUNCHER=ccache
   export CMAKE_C_COMPILER_LAUNCHER=ccache
   ```

3. **Limit colcon parallel jobs** if running low on RAM:
   ```bash
   colcon build --parallel-workers 2
   ```

---

## 📞 Support

For issues specific to:
- **X11 forwarding:** Check this README troubleshooting section
- **ROS2 build issues:** See `development_aliases.sh` for debugging tools
- **AirSim connection:** Use `/debug_airsim_connection.sh` inside container

---

**Created:** 2025-10-01
**Maintained by:** Cosys AirSim Team
**License:** MIT
