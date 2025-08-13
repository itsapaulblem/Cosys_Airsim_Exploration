# airsim_ros_pkgs

A ROS2 wrapper over the Cosys-AirSim C++ client library providing comprehensive integration with the AirSim simulation environment. All coordinates and data are in the right-handed coordinate frame of the ROS standard and not in NED except for geo points.

## Architecture Overview

The ROS2 wrapper implements a **centralized, single-node architecture** with the following key components:

- **AirsimROSWrapper**: Core class that bridges AirSim and ROS2 ecosystems
- **Vehicle Support**: MultiRotor, Car, and ComputerVision vehicle types
- **Multi-threading**: Parallel processing using callback groups and specialized timers
- **RPC Architecture**: Multiple specialized RPC clients for optimal data throughput
- **Configuration-driven**: Dynamic setup based on AirSim settings.json

### Key Features

- **Multi-vehicle Support**: Handle multiple vehicles simultaneously through a single node
- **Sensor Integration**: Comprehensive sensor support including cameras, LiDAR, IMU, GPS, and custom sensors
- **Fault Tolerance**: Robust error handling and recovery mechanisms
- **Performance Optimized**: Multi-threaded design for high-frequency sensor data processing
- **Scalable**: Linear performance scaling with vehicle count

The following was tested on Ubuntu 22.04 with ROS2 Iron and Humble.

## Build

### Prerequisites

- Build Cosys-AirSim as per the instructions
- Ubuntu 22.04 with ROS2 Iron or Humble installed
- Required dependencies: OpenCV, PCL, yaml-cpp, and ROS2 standard libraries

### Environment Setup

Make sure that you have set up the environment variables for ROS. Add the `source` command to your `.bashrc` for convenience (replace `iron` with specific version name):
```shell
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Dependencies

Install dependencies with rosdep, if not already installed:

```shell
apt-get install python3-rosdep
sudo rosdep init
rosdep update
cd <path-to-cosys-airsim>/ros2
rosdep install --from-paths src -y --ignore-src --skip-keys pcl --skip-keys message_runtime --skip-keys message_generation
```

### Build Process

Build the ROS2 workspace:

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The build process creates:
- **airsim_ros_pkgs**: Main wrapper package with vehicle nodes and utilities
- **airsim_interfaces**: Message and service definitions
- **airsim_grid_search**: Mission planning and grid search capabilities

### Build Architecture

The CMake build system creates several key libraries:
- `airsim_ros`: Main wrapper library with AirsimROSWrapper class
- `vehicle_node_base`: Base class for all vehicle types
- `multirotor_node`, `car_node`, `computer_vision_node`: Vehicle-specific implementations
- `vehicle_node_factory`: Factory pattern for dynamic vehicle creation
- `airsim_settings_parser`: Configuration parsing utilities

## Running

### Basic Launch

```shell
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### Advanced Launch Options

```shell
# Custom AirSim host and port
ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=192.168.1.100 host_port:=41451

# Enable API control and object transforms
ros2 launch airsim_ros_pkgs airsim_node.launch.py enable_api_control:=true enable_object_transforms_list:=true

# Vulkan rendering mode
ros2 launch airsim_ros_pkgs airsim_node.launch.py is_vulkan:=true

# Enable simulation clock publishing
ros2 launch airsim_ros_pkgs airsim_node.launch.py publish_clock:=true
```

### Launch with Position Controller

```shell
ros2 launch airsim_ros_pkgs airsim_with_simple_PD_position_controller.launch.py
```

### Verification

Verify the wrapper is running correctly:

```shell
# Check node status
ros2 node list

# Check published topics
ros2 topic list

# Check available services
ros2 service list

# Monitor vehicle state
ros2 topic echo /airsim_node/drone_1/odom_local_ned
```

## Technical Implementation

### Architecture Deep Dive

The ROS2 wrapper implements a **centralized, single-node architecture** with the `AirsimROSWrapper` class serving as the primary bridge between AirSim and ROS2. This design provides several key advantages:

#### Core Components

1. **AirsimROSWrapper**: The central orchestrator that:
   - Manages connections to AirSim via RPC
   - Dynamically creates vehicle nodes based on settings.json configuration
   - Handles all ROS2 publishers, subscribers, and services
   - Coordinates multi-vehicle operations

2. **Vehicle Management**:
   - `VehicleROS` base class with specialized implementations:
     - `MultiRotorROS` for drone vehicles
     - `CarROS` for ground vehicles  
     - `ComputerVisionROS` for camera-only mode
   - Factory pattern for dynamic vehicle instantiation
   - Per-vehicle namespacing for topic and service separation

3. **Multi-threading Design**:
   - Specialized timer callbacks for different data types:
     - Main state timer (~50Hz): Vehicle odometry, GPS, IMU
     - Image timer (~30Hz): Camera feeds, depth, segmentation
     - LiDAR timer (~10Hz): Point cloud data
     - GPU LiDAR timer (~10Hz): High-density point clouds
     - Echo timer (~10Hz): Radar/sonar sensor data
   - Parallel processing using ROS2 callback groups
   - Mutex protection for thread-safe RPC communication

4. **RPC Architecture**:
   - Multiple specialized RPC clients for optimal performance:
     - `airsim_client_`: Main control and state
     - `airsim_client_images_`: Camera data
     - `airsim_client_lidar_`: LiDAR data
   - Prevents blocking between high-bandwidth and low-latency operations

#### Performance Features

- **Configuration-driven Setup**: Parses settings.json to automatically configure vehicles and sensors
- **Efficient Data Flow**: Timer-based polling optimized for different sensor frequencies
- **Command Queueing**: Decouples command reception from execution for smooth control
- **Fault Isolation**: Robust error handling prevents individual failures from affecting the entire system

#### Data Flow

1. **AirSim → ROS2** (Sensor Data):
   - Timer callbacks periodically fetch data from AirSim
   - Data conversion from AirSim formats to ROS2 standard messages
   - Publishing to appropriately namespaced topics

2. **ROS2 → AirSim** (Control Commands):
   - ROS2 subscribers receive control commands
   - Commands are queued and processed in dedicated timer callbacks
   - Batch execution to AirSim via RPC for optimal performance

### Multi-Node Evolution

The current implementation provides a foundation for the planned **multi-node architecture** evolution:

- **Phase 1-2 Complete**: Vehicle node hierarchy and processing isolation implemented
- **Phase 3-4 In Progress**: Dynamic launch system and comprehensive testing
- **Target Benefits**: True parallel processing, individual vehicle fault isolation, improved scalability

Refer to `/ros2/MULTI_NODE_ROADMAP.md` for detailed evolution plans.

## Using Cosys-Airsim ROS wrapper

The ROS wrapper is composed of two ROS nodes - the first is a wrapper over Cosys-AirSim's multirotor C++ client library, and the second is a simple PD position controller.
Let's look at the ROS API for both nodes:

### Cosys-Airsim ROS Wrapper Node

#### Publishers:
The publishers will be automatically created based on the settings in the `settings.json` file for all vehicles and the sensors.

- `/airsim_node/VEHICLE-NAME/car_state` [airsim_interfaces::CarState](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/CarState.msg)
  The state of the car if the vehicle is of this sim-mode type.

- `/airsim_node/VEHICLE-NAME/computervision_state` [airsim_interfaces::ComputerVisionState](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/ComputerVisionState.msg)
  The state of the computer vision actor if the vehicle is of this sim-mode type.

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  GPS coordinates corresponding to global frame. This is set in the airsim's [settings.json](https://cosys-lab.github.io/Cosys-AirSim/settings/) file under the `OriginGeopoint` key.

- `/airsim_node/VEHICLE-NAME/global_gps` [sensor_msgs::NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
  This the current GPS coordinates of the drone in airsim.

- `/airsim_node/VEHICLE-NAME/environment` [airsim_interfaces::Environment](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/Environment.msg)

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Odometry frame (default name: odom_local, launch name and frame type are configurable) wrt take-off point.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/camera_info` [sensor_msgs::CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  Optionally if the image type is annotation the annotation layer name is also included in the topic name.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/image` [sensor_msgs::Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
  RGB or float image depending on image type requested in settings.json. Optionally if the image type is annotation the annotation layer name is also included in the topic name.

- `/tf` [tf2_msgs::TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

- `/airsim_node/VEHICLE-NAME/altimeter/SENSOR_NAME` [airsim_interfaces::Altimeter](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/Altimeter.msg)
  This the current altimeter reading for altitude, pressure, and [QNH](https://en.wikipedia.org/wiki/QNH)

- `/airsim_node/VEHICLE-NAME/imu/SENSOR_NAME` [sensor_msgs::Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  IMU sensor data.

- `/airsim_node/VEHICLE-NAME/magnetometer/SENSOR_NAME` [sensor_msgs::MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
  Measurement of magnetic field vector/compass.

- `/airsim_node/VEHICLE-NAME/distance/SENSOR_NAME` [sensor_msgs::Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)
  Measurement of distance from an active ranger, such as infrared or IR

- `/airsim_node/VEHICLE-NAME/lidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  LIDAR pointcloud 

- `/airsim_node/VEHICLE-NAME/lidar/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud of the lidar sensor

- `/airsim_node/VEHICLE-NAME/gpulidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  GPU LIDAR pointcloud. The instance segmentation/annotation color data is stored in the rgb field of the pointcloud. The intensity data is stored as well in the intensity field

- `/airsim_node/VEHICLE-NAME/echo/active/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for active sensing

- `/airsim_node/VEHICLE-NAME/echo/passive/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for passive sensing

- `/airsim_node/VEHICLE-NAME/echo/active/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the active echo pointcloud

- `/airsim_node/VEHICLE-NAME/echo/passive/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the passive echo pointcloud

- `/airsim_node/instance_segmentation_labels` [airsim_interfaces::InstanceSegmentationList](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/InstanceSegmentationList.msg)
  Custom message type with an array of a custom messages that are the names, color and index of the instance segmentation system for each object in the world.
   
- `/airsim_node/object_transforms` [airsim_interfaces::ObjectTransformsList](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/ObjectTransformsList.msg)
  Custom message type with an array of [geometry_msgs::TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html) that are the transforms of all objects in the world, each child frame ID is the object name.
   
#### Subscribers:

- `/airsim_node/VEHICLE-NAME/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/VEHICLE-NAME/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/all_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Set velocity command for all drones.

- `/airsim_node/all_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)

- `/airsim_node/group_of_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmdGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.
- 
- `/airsim_node/group_of_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmdGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.

- `/gimbal_angle_euler_cmd` [airsim_interfaces::GimbalAngleEulerCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleEulerCmd.msg)
  Gimbal set point in euler angles.

- `/gimbal_angle_quat_cmd` [airsim_interfaces::GimbalAngleQuatCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleQuatCmd.msg)
  Gimbal set point in quaternion.

- `/airsim_node/VEHICLE-NAME/car_cmd` [airsim_interfaces::CarControls](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/CarControls.msg)
Throttle, brake, steering and gear selections for control. Both automatic and manual transmission control possible, see the [`car_joy.py`](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/scripts/car_joy) script for use.

#### Services:

- `/airsim_node/VEHICLE-NAME/land` [airsim_interfaces::Land](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Land.html)

- `/airsim_node/VEHICLE-NAME/takeoff` [airsim_interfaces::Takeoff](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.html)

- `/airsim_node/all_robots/land` [airsim_interfaces::Land](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Land.html)
 land all drones

- `/airsim_node/all_robots/takeoff` [airsim_interfaces::Takeoff](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.html)
 take-off all drones

- `/airsim_node/group_of_robots/land` [airsim_interfaces::LandGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/LandGroup.html)
 land a specific set of drones

- `/airsim_node/group_of_robots/takeoff` [airsim_interfaces::TakeoffGroup](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/TakeoffGroup.html)
 take-off a specific set of drones

- `/airsim_node/reset` [airsim_interfaces::Reset](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/Reset.html)
 Resets *all* vehicles

- `/airsim_node/instance_segmentation_refresh` [airsim_interfaces::RefreshInstanceSegmentation](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/RefreshInstanceSegmentation.html)
 Refresh the instance segmentation list

- `/airsim_node/object_transforms_refresh` [airsim_interfaces::RefreshObjectTransforms](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/srv/RefreshObjectTransforms.html)
 Refresh the object transforms list

  

#### Parameters:

- `/airsim_node/host_ip` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: localhost
  The IP of the machine running the airsim RPC API server.

- `/airsim_node/host_port` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 41451
  The port of the machine running the airsim RPC API server.

- `/airsim_node/enable_api_control` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: false
  Set the API control and arm the drones on startup. If not set to true no control is available. 

- `/airsim_node/enable_object_transforms_list` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: true
  Retrieve the object transforms list from the airsim API at the start or with the service to refresh. If disabled this is not available but can save time on startup.

- `/airsim_node/host_port` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 41451
  The port of the machine running the airsim RPC API server.

- `/airsim_node/is_vulkan` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: True
  If using Vulkan, the image encoding is switched from rgb8 to bgr8. 

- `/airsim_node/world_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: world

- `/airsim_node/odom_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: odom_local

- `/airsim_node/update_airsim_control_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for updating drone odom and state from airsim, and sending in control commands.
  The current RPClib interface to unreal engine maxes out at 50 Hz.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_airsim_img_response_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all cameras in airsim.
  The speed will depend on number of images requested and their resolution.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_lidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all Lidar data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.


- `/airsim_node/update_gpulidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all GPU-Lidar data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_echo_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all echo sensor data in airsim.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/publish_clock` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`
  Default: false
  Will publish the ros /clock topic if set to true.

### Simple PID Position Controller Node

#### Parameters:

- PD controller parameters:
  * `/pd_position_node/kp_x` [double],
    `/pd_position_node/kp_y` [double],
    `/pd_position_node/kp_z` [double],
    `/pd_position_node/kp_yaw` [double]
    Proportional gains

  * `/pd_position_node/kd_x` [double],
    `/pd_position_node/kd_y` [double],
    `/pd_position_node/kd_z` [double],
    `/pd_position_node/kd_yaw` [double]
    Derivative gains

  * `/pd_position_node/reached_thresh_xyz` [double]
    Threshold euler distance (meters) from current position to setpoint position

  * `/pd_position_node/reached_yaw_degrees` [double]
    Threshold yaw distance (degrees) from current position to setpoint position

- `/pd_position_node/update_control_every_n_sec` [double]
  Default: 0.01 seconds

#### Services:

- `/airsim_node/VEHICLE-NAME/gps_goal` [Request: [airsim_interfaces::SetGPSPosition](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/srv/SetGPSPosition.srv)]
  Target gps position + yaw.
  In **absolute** altitude.

- `/airsim_node/VEHICLE-NAME/local_position_goal` [Request: [airsim_interfaces::SetLocalPosition](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros/src/airsim_ros_pkgs/srv/SetLocalPosition.srv)]
  Target local position + yaw in global frame.

#### Subscribers:

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  Listens to home geo coordinates published by `airsim_node`.

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Listens to odometry published by `airsim_node`

#### Publishers:

- `/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

- `/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

#### Global params

- Dynamic constraints. These can be changed in `dynamic_constraints.launch`:
    * `/max_vel_horz_abs` [double]
  Maximum horizontal velocity of the drone (meters/second)

    * `/max_vel_vert_abs` [double]
  Maximum vertical velocity of the drone (meters/second)

    * `/max_yaw_rate_degree` [double]
  Maximum yaw rate (degrees/second)