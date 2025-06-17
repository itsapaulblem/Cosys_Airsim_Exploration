# AirSim ROS2 Service Syntax - Quick Fix Guide

## ❌ **INCORRECT Syntax** (causes errors)
```bash
# This will fail with "object has no attribute 'waitOnLastTask'"
ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff "{waitOnLastTask: true, timeout_sec: 15.0}"
```

## ✅ **CORRECT Syntax**
```bash
# Use underscores, not camelCase, and no timeout_sec field
ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
```

## 📋 **Correct Service Commands**

### Individual Drone Control
```bash
# Takeoff
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Land
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"
```

### Multi-Drone Control
```bash
# Takeoff all drones
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Land all drones
./ros2_exec.bat "ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'"
```

## 🔍 **How to Check Service Definitions**
```bash
# Check service structure
./ros2_exec.bat "ros2 service type /airsim_node/Drone1/takeoff"
./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/Takeoff"

# List all available services
./ros2_exec.bat "ros2 service list"
```

## 🎨 **RViz2 Visualization Options**

### Option 1: Launch with RViz2 from Start
```bash
./run_simple_with_rviz.bat
```

### Option 2: Launch RViz2 Manually
```bash
# Start container normally
./run_simple.bat

# Launch RViz2 in container
./ros2_exec.bat "ros2 launch airsim_ros_pkgs rviz.launch.py"
```

### Option 3: Environment Variable
```bash
# Set LAUNCH_RVIZ=true when running container
docker run -it --rm \
    --name airsim-ros2-wrapper \
    -e LAUNCH_RVIZ=true \
    -e DISPLAY=host.docker.internal:0.0 \
    airsim_ros2_simple:latest
``` 