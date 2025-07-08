# AirSim ROS2 Wrapper with VNC Support

This Docker container provides a complete AirSim ROS2 development environment with VNC access for remote visualization using RViz2.

## Features

- **ROS2 Humble Desktop**: Full ROS2 installation including RViz2, rqt, and other GUI tools
- **VNC Access**: Remote desktop access via VNC viewer or web browser
- **AirSim Integration**: Pre-built AirSim ROS2 wrapper for drone simulation
- **Ubuntu Mate Desktop**: Full desktop environment optimized for VNC
- **Graphics Support**: Software rendering optimized for VNC/remote access

## Quick Start

### 1. Build the Container

```bash
# From the AirSim root directory
docker build -f docker/airsim_ros2_wrapper/Dockerfile.simple-with-vnc -t airsim-ros2-vnc .
```

### 2. Run the Container

**Basic usage:**
```bash
docker run -it --rm \
  -p 5901:5901 \
  -p 6901:80 \
  -e AIRSIM_HOST_IP=host.docker.internal \
  airsim-ros2-vnc
```

**With custom settings:**
```bash
docker run -it --rm \
  -p 5901:5901 \
  -p 6901:80 \
  -e AIRSIM_HOST_IP=192.168.1.100 \
  -e AIRSIM_HOST_PORT=41451 \
  -e ROS_DOMAIN_ID=0 \
  -e USER=myuser \
  -e PASSWD=mypassword \
  airsim-ros2-vnc
```

### 3. Access the Desktop

**Via VNC Viewer:**
- Host: `localhost:5901`
- Password: `ubuntu` (or your custom password)

**Via Web Browser:**
- URL: http://localhost:6901/vnc.html
- Password: `ubuntu` (or your custom password)

## Available Scripts

Once connected to the desktop, you can use these scripts:

- `/launch_airsim_ros2.sh` - Launch only the AirSim ROS2 node
- `/launch_rviz_airsim.sh` - Launch only RViz2 with AirSim-optimized settings
- `/launch_airsim_with_rviz.sh` - Launch both AirSim ROS2 and RViz2 together

## Desktop Shortcuts

The desktop includes convenient shortcuts for:
- **AirSim ROS2 Node** - Launches the AirSim ROS2 wrapper
- **RViz2 for AirSim** - Launches RViz2 visualization
- **Terminal** - Access to command line
- **Firefox** - Web browser

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `AIRSIM_HOST_IP` | `host.docker.internal` | IP address of AirSim host |
| `AIRSIM_HOST_PORT` | `41451` | Port of AirSim API |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `USER` | `ubuntu` | VNC user name |
| `PASSWD` | `ubuntu` | VNC password |

## Usage Examples

### Example 1: Basic AirSim Connection

1. Start AirSim on your host machine
2. Run the container with default settings
3. Connect via VNC at `localhost:5901`
4. Double-click "AirSim ROS2 Node" on desktop
5. Double-click "RViz2 for AirSim" to visualize

### Example 2: Custom AirSim Host

```bash
# If AirSim is running on IP 192.168.1.50
docker run -it --rm \
  -p 5901:5901 \
  -p 6901:80 \
  -e AIRSIM_HOST_IP=192.168.1.50 \
  airsim-ros2-vnc
```

### Example 3: Web Access

1. Run container with port mapping
2. Open browser to http://localhost:6901/vnc.html
3. Enter password and connect
4. Use desktop normally through web interface

## Troubleshooting

### Graphics Issues
If RViz2 has rendering problems:
- The container uses software rendering optimized for VNC
- Graphics performance may be limited compared to native execution
- Ensure your VNC client supports the resolution and color depth

### Connection Issues
- Verify AirSim is running and accessible
- Check firewall settings for ports 5901 and 6901
- Confirm AIRSIM_HOST_IP points to correct machine

### Performance Tips
- Use VNC viewer instead of web browser for better performance
- Adjust VNC resolution if needed: modify the entrypoint script
- Close unnecessary applications in the desktop environment

## Development

To modify or extend this container:

1. Edit `Dockerfile.simple-with-vnc`
2. Rebuild: `docker build -f docker/airsim_ros2_wrapper/Dockerfile.simple-with-vnc -t airsim-ros2-vnc .`
3. Test your changes

The entrypoint script handles user creation, VNC setup, and desktop environment initialization based on the reference implementation. 