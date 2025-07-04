# AirSim ROS2 Docker - Usage Examples & Recipes

This document provides practical, tested examples for using the AirSim ROS2 Docker wrapper.

## 🚀 Quick Start Recipes

### Recipe 1: Development with RViz2
```bash
# 1. Build VNC-enabled image
./build_vnc.bat

# 2. Start AirSim (Blocks environment)
# Launch your AirSim.exe

# 3. Run container with GUI
./run_vnc.bat

# 4. Access web-based GUI
# Open: http://localhost:6901/vnc.html
# Password: airsim123

# RViz2 will automatically launch showing drone visualization
```

### Recipe 2: Headless Development
```bash
# 1. Build simple image
./test_build.bat

# 2. Start AirSim
# Launch your AirSim.exe

# 3. Run headless container
./run_simple.bat

# 4. Execute commands
./ros2_exec.bat "ros2 topic list"
```

### Recipe 3: Production Deployment
```bash
# Use docker-compose for production
docker-compose up --build
```

## 🎮 Control Examples

### Basic Flight Operations

#### ✅ Correct Service Call Syntax
```bash
# Takeoff (individual drone)
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Takeoff (all drones)
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Land (individual drone)
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"

# Land (all drones)
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: false}'"

# Change height (individual drone)
ros2 service call /airsim_node/Drone2/set_altitude airsim_interfaces/srv/SetAltitude '{z: -30.0, velocity: 5.0, vehicle_name: "Drone2", wait_on_last_task: true}'


# Reset simulation
./ros2_exec.bat "ros2 service call /airsim_node/reset std_srvs/srv/Empty"
```

### Movement Control

#### Velocity Commands (Body Frame)
```bash
# Move forward
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}}}'"

# Move right
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 0.0, y: 2.0, z: 0.0}}}'"

# Move up
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 0.0, y: 0.0, z: -2.0}}}'"

# Rotate (yaw)
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {angular: {z: 1.0}}}'"

# Stop (zero velocity)
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}'"
```

#### Velocity Commands (World Frame)
```bash
# Move north-east-up in world coordinates
./ros2_exec.bat "ros2 topic pub --once /airsim_node/Drone1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 1.0, y: 1.0, z: -0.5}}}'"
```

### Multi-Drone Operations

#### Control Multiple Drones
```bash
# Takeoff all drones
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Move all drones forward
./ros2_exec.bat "ros2 topic pub --once /airsim_node/all_robots/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}'"

# Land all drones
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"
```

## 📊 Sensor Data Examples

### Camera Operations
```bash
# List all camera topics
./ros2_exec.bat "ros2 topic list | grep Scene"

# Monitor front camera info
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/front_center/camera_info"

# Monitor depth camera
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/front_center/DepthPerspective --once"

# Check camera feed rate
./ros2_exec.bat "ros2 topic hz /airsim_node/Drone1/front_center/Scene"
```

### Navigation & Positioning
```bash
# Monitor drone position and orientation
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/odom_local_ned"

# Monitor GPS coordinates
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/global_gps"

# Monitor altitude
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/barometer/alt"
```

### IMU & Motion Sensors
```bash
# Monitor IMU data (acceleration, gyroscope)
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/imu/imu"

# Monitor magnetometer
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/magnetometer/mag"

# Monitor distance sensor
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/distance/distance"
```

### LiDAR Operations
```bash
# Monitor LiDAR point cloud
./ros2_exec.bat "ros2 topic echo /airsim_node/Drone1/lidar/points --once"

# Check LiDAR data rate
./ros2_exec.bat "ros2 topic hz /airsim_node/Drone1/lidar/points"

# Record LiDAR data
./ros2_exec.bat "ros2 bag record /airsim_node/Drone1/lidar/points"
```

## 🔧 Advanced Usage

### Data Recording
```bash
# Record all topics
./ros2_exec.bat "ros2 bag record -a"

# Record specific topics
./ros2_exec.bat "ros2 bag record /airsim_node/Drone1/odom_local_ned /airsim_node/Drone1/front_center/Scene /airsim_node/Drone1/imu/imu"

# Record for specific duration (30 seconds)
./ros2_exec.bat "ros2 bag record -a --duration 30"
```

### System Inspection
```bash
# List all nodes
./ros2_exec.bat "ros2 node list"

# List all topics
./ros2_exec.bat "ros2 topic list"

# List all services
./ros2_exec.bat "ros2 service list"

# Get topic information
./ros2_exec.bat "ros2 topic info /airsim_node/Drone1/odom_local_ned"

# Get service type
./ros2_exec.bat "ros2 service type /airsim_node/Drone1/takeoff"

# Show message structure
./ros2_exec.bat "ros2 interface show geometry_msgs/msg/Twist"
./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/Takeoff"
```

### Performance Monitoring
```bash
# Check topic frequencies
./ros2_exec.bat "ros2 topic hz /airsim_node/Drone1/odom_local_ned"
./ros2_exec.bat "ros2 topic hz /airsim_node/Drone1/front_center/Scene"

# Monitor system resources
./ros2_exec.bat "top"
./ros2_exec.bat "htop"

# Check ROS2 node performance
./ros2_exec.bat "ros2 run rqt_graph rqt_graph"  # Note: Only works with VNC version
```

## 🐛 Debugging & Troubleshooting

### Common Debugging Commands
```bash
# Check if AirSim is accessible
./ros2_exec.bat "nc -z host.docker.internal 41451"

# Test ROS2 connection
./test_ros2_connection.sh

# View container logs
docker logs airsim_ros2_container

# Enter interactive shell
./ros2_exec.bat
# Then run any command interactively
```

### Service Call Debugging
```bash
# Check service availability
./ros2_exec.bat "ros2 service list | grep takeoff"

# Test service call syntax
./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/Takeoff"

# Verify node is running
./ros2_exec.bat "ros2 node info /airsim_node"
```

## 🎯 Integration Examples

### Python Client Integration
```bash
# Run Python scripts from host
cd /path/to/Cosys-AirSim/PythonClient
python multirotor/hello_drone.py

# The ROS2 wrapper and Python client can run simultaneously
```

### Mission Planning
```bash
# Use mission planning scripts
cd /path/to/Cosys-AirSim/PythonClient/multirotor/mission_planning
python mission_simple.py
```

### Computer Vision
```bash
# Access computer vision examples
cd /path/to/Cosys-AirSim/PythonClient/computer_vision
python cv_mode.py
```

## 🔗 Multi-Machine Setup Examples

### Distributed ROS2 Setup
```bash
# On machine with AirSim
docker run -e ROS_DOMAIN_ID=42 -e AIRSIM_HOST_IP=0.0.0.0 ...

# On remote machine
export ROS_DOMAIN_ID=42
ros2 topic list  # Should see topics from AirSim machine
```

### Cross-Platform Testing
```bash
# Linux client connecting to Windows AirSim
docker run -e AIRSIM_HOST_IP=192.168.1.100 -e ROS_DOMAIN_ID=42 ...
```

---

## 📝 Notes

- **API Control**: Automatically enabled on startup - drones are ready to receive commands
- **Syntax**: Use `wait_on_last_task` not `waitOnLastTask` in service calls
- **Namespaces**: Each drone has its own namespace (`Drone1`, `Drone2`, etc.)
- **Multi-drone**: Use `all_robots` or `group_of_robots` for batch operations
- **Performance**: Use `.dockerignore` for faster builds (90% size reduction) 