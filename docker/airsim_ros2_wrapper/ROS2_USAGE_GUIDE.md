# 🚁 ROS2 Usage Guide for AirSim

**Complete guide for running ROS2 commands in the AirSim VNC container**

This guide covers all ROS2 commands and workflows for controlling AirSim drones, monitoring data, and developing ROS2 applications.

## 🚀 Quick Start

### 1. Launch Container and Connect
```bash
# Start the container
./launch.sh    # Linux
launch.bat     # Windows

# Connect via VNC: localhost:5901 (password: airsim)
```

### 2. Basic ROS2 Environment Check
Open terminal in VNC desktop:
```bash
# Check ROS2 installation
ros2 --version

# List available ROS2 commands
ros2 -h

# Check environment
env | grep ROS
```

## 📊 Available ROS2 Topics and Services

### Vehicle Topics (per drone)

#### Navigation and State
```bash
# Odometry (position, velocity, orientation)
/airsim_node/drone_1/odom_local_ned

# GPS coordinates
/airsim_node/drone_1/global_gps

# IMU data (acceleration, gyroscope)
/airsim_node/drone_1/imu

# Environment data (pressure, temperature)
/airsim_node/drone_1/environment
```

#### Camera Topics
```bash
# RGB camera feed
/airsim_node/drone_1/front_center/Scene

# Depth camera
/airsim_node/drone_1/front_center/DepthVis
/airsim_node/drone_1/front_center/DepthPerspective

# Segmentation
/airsim_node/drone_1/front_center/Segmentation

# Camera info (calibration)
/airsim_node/drone_1/front_center/camera_info
```

#### Sensor Topics
```bash
# Magnetometer
/airsim_node/drone_1/magnetometer

# Barometer/altimeter
/airsim_node/drone_1/barometer

# LiDAR point cloud
/airsim_node/drone_1/lidar/points

# Distance sensor
/airsim_node/drone_1/distance
```

#### Control Topics
```bash
# Position commands
/airsim_node/drone_1/pose_cmd

# Velocity commands (body frame)
/airsim_node/drone_1/vel_cmd_body_frame

# Velocity commands (world frame)  
/airsim_node/drone_1/vel_cmd_world_frame
```

### Services

#### Flight Control
```bash
# Takeoff/Landing
/airsim_node/drone_1/takeoff
/airsim_node/drone_1/land

# Multi-drone control
/airsim_node/all_robots/takeoff
/airsim_node/all_robots/land

# Position control
/airsim_node/drone_1/local_position_goal
/airsim_node/drone_1/gps_goal

# System control
/airsim_node/reset
```

## 🎮 Essential ROS2 Commands

### 1. Discovery and Inspection

#### List Resources
```bash
# List all topics
ros2 topic list

# List all services
ros2 service list

# List all nodes
ros2 node list

# List all parameters
ros2 param list
```

#### Get Information
```bash
# Topic information
ros2 topic info /airsim_node/drone_1/odom_local_ned

# Service information  
ros2 service type /airsim_node/drone_1/takeoff

# Node information
ros2 node info /airsim_node

# Parameter value
ros2 param get /airsim_node some_parameter
```

#### Show Message/Service Structure
```bash
# Show topic message structure
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show nav_msgs/msg/Odometry

# Show service structure
ros2 interface show airsim_interfaces/srv/Takeoff
ros2 interface show airsim_interfaces/srv/Land
```

### 2. Monitoring Data

#### Echo Topics (Real-time Data)
```bash
# Monitor drone position
ros2 topic echo /airsim_node/drone_1/odom_local_ned

# Monitor GPS coordinates
ros2 topic echo /airsim_node/drone_1/global_gps

# Monitor IMU data
ros2 topic echo /airsim_node/drone_1/imu

# Monitor specific fields only
ros2 topic echo --field pose.pose.position /airsim_node/drone_1/odom_local_ned
```

#### Topic Statistics
```bash
# Check topic frequency
ros2 topic hz /airsim_node/drone_1/odom_local_ned

# Check topic bandwidth
ros2 topic bw /airsim_node/drone_1/front_center/Scene

# Get latest message once
ros2 topic echo --once /airsim_node/drone_1/global_gps
```

### 3. Drone Control Commands

#### Takeoff and Landing
```bash
# Single drone takeoff
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# Single drone landing
ros2 service call /airsim_node/drone_1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'

# All drones takeoff
ros2 service call /airsim_node/all_robots/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# All drones landing
ros2 service call /airsim_node/all_robots/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'
```

#### Position Control
```bash
# Move to local position (x, y, z in meters)
ros2 service call /airsim_node/drone_1/local_position_goal airsim_interfaces/srv/LocalPositionGoal '{
  position: {x: 10.0, y: 5.0, z: -20.0},
  yaw: 0.0,
  wait_on_last_task: true
}'

# Move to GPS coordinates
ros2 service call /airsim_node/drone_1/gps_goal airsim_interfaces/srv/GpsGoal '{
  latitude: 47.641468,
  longitude: -122.140165,
  altitude: 50.0,
  yaw: 0.0,
  wait_on_last_task: true
}'
```

#### Velocity Control
```bash
# Move forward in body frame (2 m/s forward)
ros2 topic pub --once /airsim_node/drone_1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 2.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}'

# Move in world frame (1 m/s north, 1 m/s east, 0.5 m/s up)
ros2 topic pub --once /airsim_node/drone_1/vel_cmd_world_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 1.0, y: 1.0, z: -0.5},
    angular: {x: 0.0, y: 0.0, z: 0.1}
  }
}'

# Hover (stop all movement)
ros2 topic pub --once /airsim_node/drone_1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{
  twist: {
    linear: {x: 0.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}'
```

#### System Control
```bash
# Reset simulation
ros2 service call /airsim_node/reset std_srvs/srv/Empty '{}'

# Reset drone to initial position
ros2 service call /airsim_node/drone_1/reset_drone std_srvs/srv/Empty '{}'
```

## 🛠️ Advanced Usage

### 1. Recording and Playback

#### Record Data
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /airsim_node/drone_1/odom_local_ned /airsim_node/drone_1/imu

# Record with custom name
ros2 bag record -o my_flight_data /airsim_node/drone_1/odom_local_ned
```

#### Playback Data
```bash
# List recordings
ros2 bag info my_flight_data

# Playback recording
ros2 bag play my_flight_data

# Playback at different speed
ros2 bag play --rate 2.0 my_flight_data
```

### 2. Custom Scripts and Launch Files

#### Create Custom Python Script
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from airsim_interfaces.srv import Takeoff

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/airsim_node/drone_1/odom_local_ned',
            self.odom_callback, 
            10
        )
        
        # Service client for takeoff
        self.takeoff_client = self.create_client(Takeoff, '/airsim_node/drone_1/takeoff')
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.get_logger().info(f'Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
    
    def takeoff(self):
        if self.takeoff_client.wait_for_service(timeout_sec=5.0):
            request = Takeoff.Request()
            request.wait_on_last_task = True
            future = self.takeoff_client.call_async(request)
            return future
        else:
            self.get_logger().error('Takeoff service not available')

def main():
    rclpy.init()
    controller = DroneController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Run Custom Script
```bash
# Save as drone_controller.py
python3 drone_controller.py
```

### 3. Launch Files
```xml
<!-- simple_flight.launch.xml -->
<launch>
  <!-- Launch AirSim node -->
  <node pkg="airsim_ros_pkgs" exec="airsim_node" name="airsim_node">
    <param name="update_airsim_control_every_n_sec" value="0.02"/>
    <param name="publish_clock" value="false"/>
  </node>
  
  <!-- Launch RViz2 -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" 
        args="-d $(find-pkg-share airsim_ros_pkgs)/rviz/default.rviz"/>
  
  <!-- Auto takeoff after 5 seconds -->
  <node pkg="airsim_ros_pkgs" exec="takeoff_node" name="auto_takeoff">
    <param name="delay_seconds" value="5.0"/>
    <param name="drone_name" value="drone_1"/>
  </node>
</launch>
```

```bash
# Run launch file
ros2 launch airsim_ros_pkgs simple_flight.launch.xml
```

## 📊 Monitoring and Debugging

### 1. RViz2 Visualization

#### Launch RViz2
```bash
# Basic RViz2
rviz2

# With AirSim config
rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/airsim.rviz
```

#### Useful RViz2 Displays
- **TF Tree** - Show coordinate frames
- **Odometry** - Show drone path
- **PointCloud2** - Show LiDAR data
- **Image** - Show camera feeds
- **Marker** - Show custom visualizations

### 2. Debugging Tools

#### Check TF Transforms
```bash
# View transform tree
ros2 run tf2_tools view_frames.py

# Check specific transform
ros2 run tf2_ros tf2_echo world drone_1

# List all frames
ros2 run tf2_ros tf2_monitor
```

#### Network Diagnostics
```bash
# Check node connections
ros2 node list
ros2 topic list
ros2 service list

# Test AirSim connection
/debug_airsim_connection.sh

# Monitor system resources
htop  # or top
```

### 3. Performance Tuning

#### Adjust Topic Rates
```bash
# Throttle high-frequency topics
ros2 run topic_tools throttle messages /airsim_node/drone_1/front_center/Scene 5.0

# Drop messages to reduce bandwidth
ros2 run topic_tools drop /airsim_node/drone_1/front_center/Scene 1 2
```

#### Parameter Tuning
```bash
# List AirSim node parameters
ros2 param list /airsim_node

# Get parameter value
ros2 param get /airsim_node update_airsim_control_every_n_sec

# Set parameter value
ros2 param set /airsim_node update_airsim_control_every_n_sec 0.01
```

## 🔧 Troubleshooting

### Common Issues and Solutions

#### 1. No Topics Available
```bash
# Check if AirSim node is running
ros2 node list

# Check AirSim connection
/debug_airsim_connection.sh

# Restart AirSim node
ros2 lifecycle set /airsim_node shutdown
ros2 run airsim_ros_pkgs airsim_node
```

#### 2. Service Calls Fail
```bash
# Check service availability
ros2 service list | grep takeoff

# Test with empty message first
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff

# Check service type
ros2 service type /airsim_node/drone_1/takeoff
```

#### 3. Control Commands Not Working
```bash
# Verify drone is in correct state
ros2 topic echo --once /airsim_node/drone_1/odom_local_ned

# Check if API control is enabled
ros2 param get /airsim_node is_vulkan

# Reset simulation
ros2 service call /airsim_node/reset std_srvs/srv/Empty
```

## 📝 Quick Reference Commands

### Essential Commands Cheatsheet

```bash
# === BASIC DISCOVERY ===
ros2 topic list                    # List all topics
ros2 service list                  # List all services
ros2 node list                     # List all nodes

# === MONITORING ===
ros2 topic echo /airsim_node/drone_1/odom_local_ned     # Position data
ros2 topic echo /airsim_node/drone_1/global_gps         # GPS data
ros2 topic hz /airsim_node/drone_1/odom_local_ned       # Topic frequency

# === FLIGHT CONTROL ===
# Takeoff
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'

# Land
ros2 service call /airsim_node/drone_1/land airsim_interfaces/srv/Land '{wait_on_last_task: true}'

# Move forward 2 m/s
ros2 topic pub --once /airsim_node/drone_1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}}}'

# Hover/Stop
ros2 topic pub --once /airsim_node/drone_1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}'

# === VISUALIZATION ===
rviz2                              # Launch RViz2
ros2 run tf2_tools view_frames.py  # View TF tree

# === RECORDING ===
ros2 bag record -a                 # Record all topics
ros2 bag play my_recording         # Playback recording

# === DEBUGGING ===
/debug_airsim_connection.sh        # Test AirSim connection
ros2 topic info <topic_name>       # Topic information
ros2 interface show <msg_type>     # Message structure
```

---

## 🎯 Next Steps

1. **Start with basic monitoring** - Use `ros2 topic list` and `ros2 topic echo`
2. **Try simple controls** - Takeoff, hover, and land
3. **Explore with RViz2** - Visualize drone movement and sensor data
4. **Create custom scripts** - Build your own ROS2 nodes
5. **Record and analyze** - Use `ros2 bag` for data collection

This guide provides everything you need to effectively use ROS2 with AirSim. Start with the basic commands and gradually work up to more complex operations!

---

*Happy flying with ROS2 and AirSim! 🚁*