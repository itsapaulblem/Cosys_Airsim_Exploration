# Docker ROS2 Quick Start Guide

This guide provides a quick reference for using the Cosys-AirSim ROS2 Docker environment.

## Directory Structure and Launch Locations

### Key Directories

| Location | Purpose | What to Run From Here |
|----------|---------|----------------------|
| **L:\Cosys-AirSim** | AirSim root directory | All Docker commands (`airsim_ros2_docker.bat`) |
| **/airsim_ros2_ws** | Container workspace | ROS2 commands, build scripts, launch scripts |
| **L:\Cosys-AirSim\ros2\src** | Local source files | Edit files in your IDE |
| **/airsim_ros2_ws/src** | Mounted source in container | Automatically synced with local files |

## Quick Start Commands

### 1. Building and Running Docker (From AirSim Root)

**Always run these from `L:\Cosys-AirSim`:**

```bash
# Build the Docker image (first time or after Dockerfile changes)
.\airsim_ros2_docker.bat build

# Run the container with VNC
.\airsim_ros2_docker.bat run

# Run with RViz auto-launch
.\airsim_ros2_docker.bat run --launch-rviz

# Access container shell
.\airsim_ros2_docker.bat shell

# View container logs
.\airsim_ros2_docker.bat logs

# Stop the container
.\airsim_ros2_docker.bat stop

# Clean up (remove container and optionally image)
.\airsim_ros2_docker.bat clean
```

### 2. Inside the Container (Default: /airsim_ros2_ws)

**These commands are run inside the Docker container:**

```bash
# Quick launch AirSim ROS2 node
./launch_airsim_ros2.sh

# Build commands (using aliases)
build_interfaces    # Build only interface packages
build_pkgs         # Build only main packages  
build              # Build entire workspace
clean_build        # Clean and rebuild everything
source_ws          # Source the workspace

# Manual build process
export AIRSIM_ROOT=/airsim_ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch ROS2 nodes
ros2 launch airsim_ros_pkgs airsim_node.launch.py
ros2 launch airsim_ros_pkgs rviz.launch.py
```

## Development Workflow

### Step 1: Initial Setup (One Time)

```bash
# From Windows PowerShell/Command Prompt
cd L:\Cosys-AirSim
.\airsim_ros2_docker.bat build
```

### Step 2: Daily Development

1. **Start the container** (from AirSim root):
   ```bash
   cd L:\Cosys-AirSim
   .\airsim_ros2_docker.bat run --launch-rviz
   ```

2. **Connect via VNC**:
   - VNC Client: Connect to `localhost:5901`
   - Password: `ubuntu`

3. **Edit files locally**:
   - Use your favorite IDE to edit files in `L:\Cosys-AirSim\ros2\src\`
   - Changes are automatically available in the container

4. **Build in container** (VNC terminal):
   ```bash
   # You'll already be in /airsim_ros2_ws
   build_interfaces    # After changing .msg/.srv/.action files
   build_pkgs         # After changing implementation
   source_ws          # Always source after building
   ```

5. **Test your changes**:
   ```bash
   ./launch_airsim_ros2.sh
   ```

### Step 3: Adding New Interfaces

1. **Create files locally**:
   ```
   L:\Cosys-AirSim\ros2\src\airsim_interfaces\msg\YourMessage.msg
   L:\Cosys-AirSim\ros2\src\airsim_interfaces\srv\YourService.srv
   L:\Cosys-AirSim\ros2\src\airsim_interfaces\action\YourAction.action
   ```

2. **Update CMakeLists.txt locally**:
   Edit `L:\Cosys-AirSim\ros2\src\airsim_interfaces\CMakeLists.txt`

3. **Build in container**:
   ```bash
   build_interfaces
   source_ws
   ```

## Common Issues and Solutions

### Issue: "Could not find AIRSIM_ROOT"
**Solution**: The container automatically sets this. Use `./launch_airsim_ros2.sh` or set manually:
```bash
export AIRSIM_ROOT=/airsim_ros2_ws
```

### Issue: "Permission denied: 'log/build_YYYY-MM-DD_HH-MM-SS'"
**Solution**: Fix ownership of build directories in Docker container:
```bash
cd /airsim_ros2_ws
sudo chown -R $USER:$USER build install log
```

### Issue: Changes not reflected in container
**Solution**: Ensure you're editing files in the correct location:
- Edit: `L:\Cosys-AirSim\ros2\src\*`
- NOT: `L:\Cosys-AirSim\docker\*` or other locations

### Issue: Build fails after interface changes
**Solution**: Clean build may be needed:
```bash
clean_build
# OR
rm -rf build install log
colcon build --symlink-install
```

### Issue: Can't connect to AirSim
**Solution**: Check AirSim settings.json:
```json
"ApiServerEndpoint": "0.0.0.0:41451"
```

## Container Management

### View Running Containers
```bash
docker ps
```

### Access Running Container (Alternative Methods)
```bash
# Method 1: Using the helper script
.\airsim_ros2_docker.bat shell

# Method 2: Direct docker command
docker exec -it airsim-vnc-ros2 /bin/bash
```

### Container Status
```bash
.\airsim_ros2_docker.bat status
```

## Troubleshooting

### Common Issues

1. **Container won't start**
   - Check Docker is running
   - Verify port 5901 is available
   - Try: `docker logs airsim-vnc-ros2`

2. **VNC connection failed**
   - Ensure container is running: `docker ps`
   - Check port mapping: `docker port airsim-vnc-ros2`
   - Try different VNC client

3. **Build errors**
   - Check volume mounts are correct
   - Verify AirSim is running and accessible
   - Try rebuilding container: `.\airsim_ros2_docker.bat build --no-cache`

4. **ROS2 commands not found**
   - Source ROS2: `source /opt/ros/humble/setup.bash`
   - Source workspace: `source /airsim_ros2_ws/install/setup.bash`
   - Check if workspace is built: `ls /airsim_ros2_ws/install/`

### Docker-Specific Issues

5. **"AirSim.sln not found" error**
   - This error should no longer occur with the new Docker workflow
   - If it does, ensure you're using the latest container image
   - Rebuild container: `.\airsim_ros2_docker.bat build --no-cache`

6. **RPC library linking errors**
   - This is a known issue with position-independent code
   - Workaround: Build interfaces only: `colcon build --packages-select airsim_interfaces`
   - The core ROS2 functionality will work with interfaces

7. **Volume mounting not working**
   - Verify you're running from the AirSim root directory
   - Check Docker volume syntax in batch script
   - Try: `docker exec -it airsim-vnc-ros2 ls -la /airsim_ros2_ws/src`

8. **Container clock skew warnings**
   - These are harmless warnings due to Docker time sync
   - Can be ignored if build completes successfully

### Debugging Commands

```bash
# Check container status
docker ps -a

# View container logs
docker logs airsim-vnc-ros2

# Check volume mounts
docker inspect airsim-vnc-ros2 | grep -A 20 "Mounts"

# Access container shell for debugging
docker exec -it airsim-vnc-ros2 /bin/bash

# Check ROS2 environment in container
docker exec -it airsim-vnc-ros2 printenv | grep ROS

# Test AirSim connection from container
docker exec -it airsim-vnc-ros2 timeout 5 bash -c "echo >/dev/tcp/host.docker.internal/41451"
```

## File Paths Reference

### On Windows Host
- AirSim Root: `L:\Cosys-AirSim\`
- ROS2 Source: `L:\Cosys-AirSim\ros2\src\`
- Docker Files: `L:\Cosys-AirSim\docker\airsim_ros2_wrapper\VNC\`
- Helper Script: `L:\Cosys-AirSim\airsim_ros2_docker.bat`

### Inside Container
- Workspace: `/airsim_ros2_ws/`
- Source Files: `/airsim_ros2_ws/src/` (mounted from host)
- Build Output: `/airsim_ros2_ws/build/`
- Install Output: `/airsim_ros2_ws/install/`
- Launch Script: `/airsim_ros2_ws/launch_airsim_ros2.sh`

## Tips for Efficient Development

1. **Use VNC for visual feedback**: Especially helpful when working with RViz
2. **Keep a terminal open**: For quick rebuilds and testing
3. **Use the aliases**: They save typing and include correct parameters
4. **Edit locally, build in container**: Best of both worlds
5. **Check logs for errors**: `.\airsim_ros2_docker.bat logs --follow`

## Next Steps

- See [ROS2_USAGE_GUIDE.md](ROS2_USAGE_GUIDE.md) for detailed ROS2 commands
- See [README_Adding_Services_Messages.md](README_Adding_Services_Messages.md) for adding new interfaces
- See [WSL2_ROS2_TROUBLESHOOTING.md](WSL2_ROS2_TROUBLESHOOTING.md) for troubleshooting

Remember: **Always run Docker commands from the AirSim root directory!**