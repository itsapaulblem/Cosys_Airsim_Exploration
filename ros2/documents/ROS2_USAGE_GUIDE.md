# Cosys-AirSim ROS2 Usage Guide

This guide provides comprehensive documentation for using ROS2 topics, services, and actions with Cosys-AirSim.

## Table of Contents

- [Quick Start](#quick-start)
- [Topics (Publishers/Subscribers)](#topics-publisherssubscribers)
- [Services](#services)
- [Actions](#actions)
- [Common Usage Examples](#common-usage-examples)
- [Multi-Vehicle Operations](#multi-vehicle-operations)
- [Mission Planning](#mission-planning)
- [Troubleshooting](#troubleshooting)

## Quick Start

### Prerequisites
1. Build the ROS2 workspace:
   ```bash
   cd ros2
   colcon build
   source install/setup.bash
   ```

2. Launch AirSim ROS2 wrapper:
   
   **Ultra-Clean Multi-Vehicle (Primary):**
   ```bash
   ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
   ```
   This automatically discovers vehicles and creates clean nodes: `/Droan1`, `/PX4_Drone2`, etc.
   
   **Legacy Single-Node (Compatibility):**
   ```bash
   ros2 launch airsim_ros_pkgs airsim_node.launch.py
   ```

### Basic Commands

```bash
# List all active topics
ros2 topic list

# List all available services
ros2 service list

# List all available actions
ros2 action list

# Get topic info
ros2 topic info <topic_name>

# Get service info
ros2 service type <service_name>

# Get action info
ros2 action info <action_name>

# Check for ultra-clean vehicle nodes (primary architecture)
ros2 node list  # Should show /Droan1, /PX4_Drone2, etc.
```

## Topics (Publishers/Subscribers)

### Vehicle State Topics

#### Drone State
```bash
# Vehicle odometry (position, velocity, orientation) - Ultra-Clean Naming
ros2 topic echo /Droan1/odom_local_ned        # Primary architecture
ros2 topic echo /PX4_Drone2/odom_local_ned     # Multi-vehicle example

# GPS position
ros2 topic echo /Droan1/global_gps

# Environment data (gravity, air pressure, temperature)
ros2 topic echo /Droan1/environment

# Legacy naming (for compatibility)
ros2 topic echo /airsim_node/drone_1/odom_local_ned
```

#### Car State
```bash
# Car state (pose, speed, gear, RPM)
ros2 topic echo /airsim_node/car_1/car_state

# Car odometry
ros2 topic echo /airsim_node/car_1/odom_local_ned
```

### Sensor Topics

#### Camera Data
```bash
# Camera images
ros2 topic echo /airsim_node/drone_1/front_center_custom/Scene
ros2 topic echo /airsim_node/drone_1/front_center_custom/DepthVis

# Camera info
ros2 topic echo /airsim_node/drone_1/front_center_custom/Scene/camera_info
```

#### LiDAR Data
```bash
# LiDAR point cloud
ros2 topic echo /airsim_node/drone_1/lidar_1/LidarCustom

# GPU LiDAR point cloud
ros2 topic echo /airsim_node/drone_1/gpu_lidar_1/GPULidarCustom
```

#### IMU and GPS
```bash
# IMU data
ros2 topic echo /airsim_node/drone_1/imu_1/Imu

# GPS data
ros2 topic echo /airsim_node/drone_1/gps_1/GpsCustom

# Magnetometer
ros2 topic echo /airsim_node/drone_1/magnetometer_1/MagnetometerCustom

# Barometer/Altimeter
ros2 topic echo /airsim_node/drone_1/barometer_1/BarometerCustom
```

### Control Topics

#### Velocity Commands
```bash
# Send velocity command to single drone (body frame) - Ultra-Clean Naming
ros2 topic pub /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 1.0, y: 0.0, z: 0.0},    # Forward 1 m/s in body frame
    angular: {x: 0.0, y: 0.0, z: 0.0}    # No rotation
  }
}'

# Send velocity command to single drone (world frame) - Ultra-Clean Naming
ros2 topic pub /Droan1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 1.0, y: 0.0, z: 0.0},    # Move 1 m/s in world X direction
    angular: {x: 0.0, y: 0.0, z: 0.5}    # Yaw rotation 0.5 rad/s
  }
}'

# Multi-vehicle velocity commands
ros2 topic pub /PX4_Drone2/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 2.0, y: 1.0, z: -0.5},   # Forward + right + up movement
    angular: {x: 0.0, y: 0.0, z: 0.2}    # Slow yaw rotation
  }
}'

# Send velocity command to multiple drones (legacy group commands)
ros2 topic pub /airsim_node/vel_cmd_group_body_frame airsim_interfaces/msg/VelCmdGroup '{
  vehicle_names: ["Droan1", "PX4_Drone2"],
  vel_cmds: [
    {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}},
    {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}
  ]
}'
```

**Improved Velocity Command Features:**
- **Separate Frame Handling**: Body and world frame commands processed independently
- **No Command Conflicts**: Multiple frame commands can be sent simultaneously
- **Correct AirSim APIs**: Uses `moveByVelocityBodyFrameAsync()` for body frame commands
- **Enhanced Duration**: Commands processed every 100ms for smoother control
- **Better Yaw Control**: Improved yaw rate handling in both coordinate frames

#### Car Controls
```bash
# Send car control commands
ros2 topic pub /airsim_node/car_1/car_cmd airsim_interfaces/msg/CarControls '{
  throttle: 0.5,
  brake: 0.0,
  steering: 0.0,
  handbrake: false,
  is_manual_gear: false,
  manual_gear: 0,
  gear_immediate: false
}'
```

#### Gimbal Control
```bash
# Control gimbal with Euler angles
ros2 topic pub /airsim_node/gimbal_angle_euler_cmd airsim_interfaces/msg/GimbalAngleEulerCmd '{
  vehicle_name: "drone_1",
  camera_name: "front_center_custom",
  roll: 0.0,
  pitch: -0.5,
  yaw: 0.0
}'

# Control gimbal with quaternion
ros2 topic pub /airsim_node/gimbal_angle_quat_cmd airsim_interfaces/msg/GimbalAngleQuatCmd '{
  vehicle_name: "drone_1",
  camera_name: "front_center_custom",
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}'
```

## Services

### Basic Flight Operations

#### Single Vehicle Operations
```bash
# Takeoff drone - Ultra-Clean Naming
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# Land drone - Ultra-Clean Naming
ros2 service call /Droan1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'

# Multi-vehicle individual control
ros2 service call /PX4_Drone2/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'
ros2 service call /PX4_Drone2/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'

# Set altitude (legacy service)
ros2 service call /airsim_node/drone_1/set_altitude airsim_interfaces/srv/SetAltitude '{
  altitude: 10.0,
  velocity: 2.0,
  wait_on_last_task: true
}'

# Set GPS position (legacy service)
ros2 service call /airsim_node/drone_1/set_gps_position airsim_interfaces/srv/SetGPSPosition '{
  latitude: 47.641468,
  longitude: -122.140165,
  altitude: 122.0,
  yaw: 0.0,
  wait_on_last_task: true
}'

# Set local position (legacy service)
ros2 service call /airsim_node/drone_1/set_local_position airsim_interfaces/srv/SetLocalPosition '{
  x: 10.0,
  y: 5.0,
  z: -20.0,
  yaw: 1.57,
  vehicle_name: "drone_1",
  wait_on_last_task: true
}'
```

**Enhanced Takeoff/Land Services:**
- **Improved Vehicle Control**: Proper arming sequence for PX4 compatibility
- **Better Error Handling**: Comprehensive state validation and logging
- **Vehicle State Cleanup**: Proper disarming and API control management after landing
- **Individual Vehicle Access**: Each vehicle has dedicated service endpoints

#### Multi-Vehicle Operations
```bash
# Takeoff multiple drones
ros2 service call /airsim_node/takeoff_group airsim_interfaces/srv/TakeoffGroup '{
  vehicle_names: ["drone_1", "drone_2", "drone_3"],
  wait_on_last_task: true
}'

# Land multiple drones
ros2 service call /airsim_node/land_group airsim_interfaces/srv/LandGroup '{
  vehicle_names: ["drone_1", "drone_2"],
  wait_on_last_task: true
}'

# Takeoff all drones
ros2 service call /airsim_node/takeoff_all airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# Land all drones
ros2 service call /airsim_node/land_all airsim_interfaces/srv/Land '{wait_on_last_task: true}'

# Coordinated height and land operation (all drones to specific height, then land)
ros2 service call /airsim_node/coordinated_height_and_land airsim_interfaces/srv/CoordinatedHeightAndLand '{
  vehicle_names: [],  # Empty = all vehicles
  target_height: -15.0,  # 15 meters altitude (negative in NED)
  ascent_speed: 3.0,
  hover_time: 5.0,  # Hover for 5 seconds before landing
  wait_on_last_task: true
}'

# Coordinated height and land for specific drones
ros2 service call /airsim_node/coordinated_height_and_land airsim_interfaces/srv/CoordinatedHeightAndLand '{
  vehicle_names: ["drone_1", "drone_2"],
  target_height: -20.0,  # 20 meters altitude
  ascent_speed: 2.5,
  hover_time: 3.0,
  wait_on_last_task: true
}'
```

### Simulation Control
```bash
# Reset simulation
ros2 service call /airsim_node/reset airsim_interfaces/srv/Reset '{wait_on_last_task: true}'

# Refresh instance segmentation
ros2 service call /airsim_node/drone_1/refresh_instance_segmentation airsim_interfaces/srv/RefreshInstanceSegmentation '{}'

# Refresh object transforms
ros2 service call /airsim_node/drone_1/refresh_object_transforms airsim_interfaces/srv/RefreshObjectTransforms '{}'

# List scene objects
ros2 service call /airsim_node/list_scene_object_tags airsim_interfaces/srv/ListSceneObjectTags '{regex_name: ".*"}'
```

### Mission Planning Services
```bash
# Generate grid search waypoints
ros2 service call /airsim_grid_search/generate_grid airsim_mission_interfaces/srv/GenerateGrid '{
  area: {
    coordinate_type: 0,
    min_x: 0.0,
    max_x: 100.0,
    min_y: 0.0,
    max_y: 100.0,
    min_altitude: 10.0,
    max_altitude: 50.0
  },
  grid_params: {
    pattern_type: 0,
    spacing: 10.0,
    overlap: 0.2,
    altitude: 20.0,
    speed: 5.0
  }
}'

# Get mission status
ros2 service call /airsim_grid_search/get_mission_status airsim_mission_interfaces/srv/GetMissionStatus '{mission_id: "mission_001"}'

# Update mission
ros2 service call /airsim_grid_search/update_mission airsim_mission_interfaces/srv/UpdateMission '{
  mission_id: "mission_001",
  operation: 0,
  parameters: {speed: 3.0}
}'
```

## Actions

### Grid Search Mission
```bash
# Start grid search mission
ros2 action send_goal /airsim_grid_search/grid_search airsim_mission_interfaces/action/GridSearch '{
  search_area: {
    coordinate_type: 0,
    min_x: 0.0,
    max_x: 100.0,
    min_y: 0.0,
    max_y: 100.0,
    min_altitude: 15.0,
    max_altitude: 25.0
  },
  grid_params: {
    pattern_type: 0,
    spacing: 15.0,
    overlap: 0.3,
    altitude: 20.0,
    speed: 4.0,
    enable_photo_capture: true,
    photo_interval: 2.0
  },
  safety_params: {
    min_battery_level: 20.0,
    max_wind_speed: 15.0,
    return_home_battery: 30.0,
    enable_geofencing: true,
    emergency_land_battery: 10.0
  },
  vehicle_name: "drone_1",
  mission_name: "survey_mission_001"
}'

# Monitor grid search progress
ros2 action send_goal /airsim_grid_search/grid_search airsim_mission_interfaces/action/GridSearch '{...}' --feedback
```

### Waypoint Navigation
```bash
# Navigate through waypoints
ros2 action send_goal /airsim_grid_search/waypoint_navigation airsim_mission_interfaces/action/WaypointNavigation '{
  waypoints: [
    {
      waypoint_type: 0,
      position: {x: 10.0, y: 10.0, z: -15.0},
      yaw: 0.0,
      speed: 5.0,
      hover_time: 2.0
    },
    {
      waypoint_type: 1,
      position: {x: 20.0, y: 20.0, z: -15.0},
      yaw: 1.57,
      speed: 5.0,
      hover_time: 3.0,
      enable_photo_capture: true
    }
  ],
  vehicle_name: "drone_1",
  loop_waypoints: false,
  speed_override: 4.0
}'
```

## Service Descriptions

### Core Flight Services
- **Takeoff/Land**: Basic flight operations for single or multiple vehicles
- **SetAltitude**: Move vehicle to specific altitude while maintaining current XY position
- **SetGPSPosition**: Move vehicle to GPS coordinates with specified altitude and yaw
- **SetLocalPosition**: Move vehicle to local NED coordinates with specified yaw
- **CoordinatedHeightAndLand**: Multi-vehicle coordination service for synchronized operations

### Multi-Vehicle Coordination
The **CoordinatedHeightAndLand** service provides sophisticated multi-vehicle coordination:
1. **Height Phase**: Moves all specified vehicles to target height simultaneously
2. **Hover Phase**: Maintains formation at target height for specified duration
3. **Landing Phase**: Coordinates synchronized landing of all vehicles
4. **Error Handling**: Provides detailed success/failure reporting per vehicle
5. **Flexible Targeting**: Can operate on all vehicles or specified subset

## Common Usage Examples

### 1. Basic Drone Control Sequence
```bash
# 1. Takeoff - Ultra-Clean Naming
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# 2. Move forward (body frame) - Improved velocity command handling
ros2 topic pub --once /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}'

# 3. Move diagonally (world frame) - Can be used simultaneously with body frame
ros2 topic pub --once /Droan1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}
}'

# 4. Hover for 5 seconds
sleep 5

# 5. Land - Enhanced landing with proper cleanup
ros2 service call /Droan1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'
```

**Key Improvements:**
- **Ultra-Clean Naming**: Direct vehicle names as node names (`/Droan1`)
- **Simultaneous Commands**: Body and world frame commands work independently
- **Enhanced Services**: Improved takeoff/land with proper vehicle state management

### 2. Coordinated Multi-Drone Height and Landing
```bash
# Command all drones to go to 25m altitude, hover for 3 seconds, then land
ros2 service call /airsim_node/coordinated_height_and_land airsim_interfaces/srv/CoordinatedHeightAndLand '{
  vehicle_names: [],  # Empty = all vehicles
  target_height: -25.0,  # 25 meters altitude (NED coordinates)
  ascent_speed: 2.5,
  hover_time: 3.0,
  wait_on_last_task: true
}'
```

### 3. Multi-Drone Formation Flight
```bash
# 1. Individual takeoff with ultra-clean naming (more reliable than group commands)
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}' &
ros2 service call /PX4_Drone2/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}' &
ros2 service call /MyDrone3/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}' &
wait

# 2. Formation movement - Individual vehicle commands (enhanced control)
ros2 topic pub --once /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}' &
ros2 topic pub --once /PX4_Drone2/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}' &
ros2 topic pub --once /MyDrone3/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}' &

# 3. Alternative: Legacy group command for backward compatibility
ros2 topic pub --once /airsim_node/vel_cmd_group_body_frame airsim_interfaces/msg/VelCmdGroup '{
  vehicle_names: ["Droan1", "PX4_Drone2", "MyDrone3"],
  vel_cmds: [
    {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}},
    {twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}},
    {twist: {linear: {x: 1.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}
  ]
}'

# 4. Individual landing (more precise control)
ros2 service call /Droan1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}' &
ros2 service call /PX4_Drone2/land airsim_interfaces/srv/Land '{wait_on_last_task: true}' &
ros2 service call /MyDrone3/land airsim_interfaces/srv/Land '{wait_on_last_task: true}' &
wait
```

**Formation Flight Advantages:**
- **Individual Control**: Each vehicle has dedicated command endpoints
- **Better Reliability**: Individual services more robust than group commands
- **Enhanced Precision**: Separate body/world frame commands per vehicle
- **Improved Error Handling**: Per-vehicle error reporting and state management

### 3. Sensor Data Collection
```bash
# 1. Start recording camera data
ros2 bag record /airsim_node/drone_1/front_center_custom/Scene

# 2. Start recording LiDAR data
ros2 bag record /airsim_node/drone_1/lidar_1/LidarCustom

# 3. Start recording IMU data
ros2 bag record /airsim_node/drone_1/imu_1/Imu

# 4. Record everything
ros2 bag record -a
```

### 4. Car Control Example
```bash
# 1. Drive forward
ros2 topic pub --once /airsim_node/car_1/car_cmd airsim_interfaces/msg/CarControls '{
  throttle: 0.6,
  brake: 0.0,
  steering: 0.0,
  handbrake: false
}'

# 2. Turn left while driving
ros2 topic pub --once /airsim_node/car_1/car_cmd airsim_interfaces/msg/CarControls '{
  throttle: 0.4,
  brake: 0.0,
  steering: -0.5,
  handbrake: false
}'

# 3. Stop
ros2 topic pub --once /airsim_node/car_1/car_cmd airsim_interfaces/msg/CarControls '{
  throttle: 0.0,
  brake: 1.0,
  steering: 0.0,
  handbrake: true
}'
```

## Multi-Vehicle Operations

### Configuration
Vehicles are configured in `settings.json`. Example for multiple drones:

```json
{
  "Vehicles": {
    "drone_1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0
    },
    "drone_2": {
      "VehicleType": "SimpleFlight", 
      "X": 5, "Y": 5, "Z": 0
    },
    "drone_3": {
      "VehicleType": "SimpleFlight",
      "X": -5, "Y": 5, "Z": 0
    }
  }
}
```

### Common Multi-Vehicle Commands
```bash
# Check all vehicle states
ros2 topic echo /airsim_node/drone_1/odom_local_ned &
ros2 topic echo /airsim_node/drone_2/odom_local_ned &
ros2 topic echo /airsim_node/drone_3/odom_local_ned &

# Coordinate takeoff
ros2 service call /airsim_node/takeoff_group airsim_interfaces/srv/TakeoffGroup '{
  vehicle_names: ["drone_1", "drone_2", "drone_3"],
  wait_on_last_task: true
}'

# Swarm movement
ros2 topic pub --once /airsim_node/vel_cmd_group_world_frame airsim_interfaces/msg/VelCmdGroup '{
  vehicle_names: ["drone_1", "drone_2", "drone_3"],
  vel_cmds: [
    {twist: {linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}},
    {twist: {linear: {x: 0.0, y: 5.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}},
    {twist: {linear: {x: -5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}
  ]
}'
```

## Mission Planning

### Grid Search Mission Planning
```bash
# 1. Generate waypoints
ros2 service call /airsim_grid_search/generate_grid airsim_mission_interfaces/srv/GenerateGrid '{
  area: {
    coordinate_type: 0,
    min_x: 0.0,
    max_x: 200.0,
    min_y: 0.0,
    max_y: 200.0,
    min_altitude: 20.0,
    max_altitude: 30.0
  },
  grid_params: {
    pattern_type: 0,
    spacing: 25.0,
    overlap: 0.3,
    altitude: 25.0,
    speed: 6.0,
    enable_photo_capture: true,
    photo_interval: 3.0
  }
}'

# 2. Execute mission
ros2 action send_goal /airsim_grid_search/grid_search airsim_mission_interfaces/action/GridSearch '{
  search_area: {...},
  grid_params: {...},
  safety_params: {
    min_battery_level: 25.0,
    max_wind_speed: 12.0,
    return_home_battery: 35.0,
    enable_geofencing: true,
    emergency_land_battery: 15.0
  },
  vehicle_name: "drone_1",
  mission_name: "mapping_mission_001"
}'

# 3. Monitor progress
ros2 action send_goal /airsim_grid_search/grid_search airsim_mission_interfaces/action/GridSearch '{...}' --feedback
```

### Pattern Types
- `0`: Boustrophedon (back-and-forth)
- `1`: Spiral (outward spiral)
- `2`: Adaptive (dynamic based on conditions)

### Mission States
- `0`: IDLE
- `1`: PLANNING
- `2`: EXECUTING
- `3`: PAUSED
- `4`: COMPLETED
- `5`: ABORTED
- `6`: EMERGENCY
- `7`: RETURNING_HOME

## Troubleshooting

### Common Issues

#### 1. Connection Problems
```bash
# Check if AirSim is running
ros2 service call /airsim_node/reset airsim_interfaces/srv/Reset '{wait_on_last_task: false}'

# Check ROS2 node status
ros2 node info /airsim_node
```

#### 2. Vehicle Not Responding
```bash
# Reset simulation
ros2 service call /airsim_node/reset airsim_interfaces/srv/Reset '{wait_on_last_task: true}'

# Check vehicle state
ros2 topic echo /airsim_node/drone_1/odom_local_ned
```

#### 3. Sensor Data Issues
```bash
# Check topic publishing rate
ros2 topic hz /airsim_node/drone_1/front_center_custom/Scene

# Check sensor configuration
ros2 topic info /airsim_node/drone_1/imu_1/Imu
```

#### 4. Mission Execution Problems
```bash
# Check mission status
ros2 service call /airsim_grid_search/get_mission_status airsim_mission_interfaces/srv/GetMissionStatus '{mission_id: "mission_001"}'

# Abort mission
ros2 service call /airsim_grid_search/update_mission airsim_mission_interfaces/srv/UpdateMission '{
  mission_id: "mission_001",
  operation: 2
}'
```

### Debug Commands
```bash
# List all topics with types
ros2 topic list -t

# Monitor all service calls
ros2 service call --verbose <service_name> <service_type> '<service_args>'

# Check message definitions
ros2 interface show <message_type>

# Monitor network traffic
ros2 doctor

# Check parameter server
ros2 param list /airsim_node
```

### Performance Tips
1. **Use appropriate QoS settings** for high-frequency topics
2. **Batch multi-vehicle operations** using group services
3. **Monitor system resources** during intensive operations
4. **Use selective recording** instead of `ros2 bag record -a`
5. **Implement proper error handling** in mission scripts

### Velocity Command Best Practices

#### Frame Selection Guidelines
```bash
# Use BODY FRAME for:
# - Natural robot control (forward/backward, left/right)
# - Joystick/gamepad input
# - Relative movement commands
ros2 topic pub /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}
}'

# Use WORLD FRAME for:
# - GPS waypoint navigation
# - Formation flight coordination
# - Absolute positioning tasks
ros2 topic pub /Droan1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.0, y: 1.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}'
```

#### Advanced Velocity Control
```bash
# Simultaneous body and world commands (both work independently)
ros2 topic pub /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}' &
ros2 topic pub /Droan1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.2}}
}' &

# Mixed control: body frame translation + world frame yaw
ros2 topic pub /Droan1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}'
ros2 topic pub /Droan1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}
}'
```

### Ultra-Clean Architecture Benefits

**Primary ROS2 Launch (Recommended):**
```bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
# Creates: /Droan1, /PX4_Drone2, /MyDrone3, etc.
# Each vehicle = independent ROS2 node with full service/topic set
```

**Key Advantages:**
- **Direct Vehicle Access**: `/VehicleName/topic` instead of `/airsim_node/VehicleName/topic`
- **Independent Error Handling**: Vehicle failures don't affect other vehicles
- **Scalable Architecture**: Add/remove vehicles without affecting others
- **Cleaner Debugging**: Individual vehicle logs and state monitoring
- **Better Performance**: Distributed processing across multiple nodes

This guide covers the essential ROS2 commands and interfaces for Cosys-AirSim. For advanced usage and custom implementations, refer to the source code and additional documentation in the project repository.