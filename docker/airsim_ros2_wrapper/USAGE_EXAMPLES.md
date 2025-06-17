# AirSim ROS2 Docker - Usage Examples

## Prerequisites
- AirSim running on host machine (Windows/Linux)
- Docker installed
- Container built and running

## Important Notes
- **API Control is enabled by default** - The ROS2 wrapper automatically requests API control and arms drones on startup
- Make sure AirSim is running before starting the container
- The container connects to AirSim via `host.docker.internal` by default

## Basic Usage

### 1. List all available topics
```bash
# Windows
./ros2_exec.bat "ros2 topic list"

# Linux
./ros2_exec.sh "ros2 topic list"
```

### 2. Monitor drone state
```bash
# Monitor position and orientation
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom"

# Monitor IMU data
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/imu/Imu"
```

### 3. Camera topics
```bash
# View available camera topics
./ros2_exec.bat "ros2 topic list | grep camera"

# Monitor front camera
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/front_center_custom/Scene"
```

## Service Calls

### 4. Takeoff (API control enabled automatically)
```bash
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{waitOnLastTask: true}'"
```

### 5. Land
```bash
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '{waitOnLastTask: true}'"
```

### 6. Move by velocity
```bash
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/move_by_velocity airsim_interfaces/srv/MoveByVelocity '{vx: 1.0, vy: 0.0, vz: 0.0, duration: 2.0, waitOnLastTask: true}'"
```

## Advanced Examples

### 7. Multi-drone operations
```bash
# List all drones
./ros2_exec.bat "ros2 topic list | grep Drone"

# Takeoff multiple drones
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{waitOnLastTask: true}'"
./ros2_exec.bat "ros2 service call /airsim_node/Drone2/takeoff airsim_interfaces/srv/Takeoff '{waitOnLastTask: true}'"
```

### 8. Image capture
```bash
# Capture image from front camera
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/capture_image airsim_interfaces/srv/CaptureImage '{camera_name: \"front_center_custom\", image_type: 0}'"
```

## Troubleshooting

### Connection Issues
- Ensure AirSim is running before starting the container
- Check if the host IP is correct (default: `host.docker.internal`)
- Try setting custom host IP: `docker run -e AIRSIM_HOST_IP=192.168.1.100 ...`

### API Control Issues
- API control is automatically enabled on startup
- If services fail, check AirSim logs for connection errors
- Restart the container if API control is lost

### Multi-drone Setup
- Ensure your AirSim settings.json includes multiple vehicles
- Each drone will have its own namespace (Drone1, Drone2, etc.)

## 🚀 Quick Commands Using ros2_exec.bat

The `ros2_exec.bat` script makes it easy to run ROS2 commands in the container without environment issues.

### Interactive Shell (Recommended for Development)
```bash
# Start interactive bash session in container
./ros2_exec.bat

# Now you can run any ROS2 commands:
ros2 topic list
ros2 node list
ros2 service list
```

### Single Commands
```bash
# List all topics
./ros2_exec.bat "ros2 topic list"

# List all nodes
./ros2_exec.bat "ros2 node list"

# Echo a topic
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom_local_ned"

# Call a service
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{waitOnLastTask: true}'"
```

## 🎮 Common ROS2 Commands

### Exploration Commands
```bash
# See what's available
./ros2_exec.bat "ros2 topic list"
./ros2_exec.bat "ros2 service list"
./ros2_exec.bat "ros2 node list"

# Get topic info
./ros2_exec.bat "ros2 topic info /airsim_node/Drone1/odom_local_ned"
./ros2_exec.bat "ros2 topic hz /airsim_node/Drone1/front_center/Scene"

# Get service info
./ros2_exec.bat "ros2 service type /airsim_node/Drone1/takeoff"
```

### Drone Control Commands
```bash
# Takeoff (CORRECT syntax - note underscore, not camelCase)
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Takeoff all drones
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Land
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"

# Land all drones  
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"

# Move forward (body frame)
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}}}'"
```

### Monitoring Commands
```bash
# Monitor drone position
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom_local_ned"

# Monitor GPS
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/global_gps"

# Monitor camera feed info
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/front_center/camera_info"
```

### Data Recording
```bash
# Record all topics
./ros2_exec.bat "ros2 bag record -a"

# Record specific topics
./ros2_exec.bat "ros2 bag record /airsim_node/Drone1/odom_local_ned /airsim_node/Drone1/front_center/Scene"
```

## 🔧 Alternative Methods (Without Helper Script)

### Method 1: Direct docker exec with bash
```bash
docker exec -it airsim_ros2_container bash -c "source /opt/ros/humble/setup.bash && source /airsim_ros2_ws/install/setup.bash && ros2 topic list"
```

### Method 2: Interactive bash session
```bash
docker exec -it airsim_ros2_container bash
# Then inside container:
ros2 topic list
```

## 🐛 Troubleshooting

### Container Not Running
```bash
# Check if container is running
docker ps

# Start container if needed
./run_simple.bat
```

### Permission Issues
```bash
# Run as administrator if needed on Windows
# Right-click Command Prompt -> "Run as administrator"
```

### ROS2 Environment Issues
The helper script automatically sources the ROS2 environment, but if you have issues:
```bash
# Check ROS2 environment inside container
./ros2_exec.bat "printenv | grep ROS"
```

## 🎨 RViz2 Visualization

### Option 1: Launch with RViz2 from Start
```bash
# Run container with RViz2 enabled
./run_simple_with_rviz.bat
```

### Option 2: Launch RViz2 Manually
```bash
# Start normal container
./run_simple.bat

# In another terminal, launch RViz2
./ros2_exec.bat "ros2 launch airsim_ros_pkgs rviz.launch.py"
```

### Option 3: Set Environment Variable
```bash
# Start container with RViz2 environment variable
docker run -it --rm \
    --name airsim-ros2-wrapper \
    -e LAUNCH_RVIZ=true \
    -e DISPLAY=host.docker.internal:0.0 \
    airsim_ros2_simple:latest
``` 