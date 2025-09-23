# Multi-Camera Motion Detection Node for AirSim ROS2 Integration

## Overview

The `motion_detection_node.py` is a ROS 2 node that implements real-time person detection, tracking and autonomous following using a multi-camera drone system in AirSim. The node combines computer vision (YOLOv7 + DeepSORT or OpenCV fallback), drone control algorithms and ROS 2 communication to create an intelligent drone than can autonomously follow a person while maintaining safe distances and smooth flight behaviour. 

## Table of Contents
- [Architecture Overview](#architecture-overview)
- [Key Components](#key-components)
- [Detection Systems](#detection-systems)
- [Control Logic](#control-logic)
- [Multi-Camera System](#multi-camera-system)
- [Configuration parameters](#configuration-parameters)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Troubleshooting](#troubleshooting)

## Architercture Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    MultiCameraMotionDetectionNode               │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Camera 0      │  │   Camera 1      │  │   Camera 2/3    │ │
│  │   (Front)       │  │   (Right)       │  │  (Back/Left)    │ │
│  │  Primary Cam    │  │                 │  │                 │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│           │                     │                     │         │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              Detection & Tracking Engine               │ │
│  │  ┌─────────────────┐  ┌─────────────────┐              │ │
│  │  │  YOLOv7 +       │  │   OpenCV        │              │ │
│  │  │  DeepSORT       │  │   Fallback      │              │ │
│  │  └─────────────────┘  └─────────────────┘              │ │
│  └─────────────────────────────────────────────────────────────┘ │
│           │                                                     │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                Target Selection Logic                  │ │
│  │     • Multi-camera fusion                               │ │
│  │     • Target confidence scoring                         │ │
│  │     • Position prediction                               │ │
│  └─────────────────────────────────────────────────────────────┘ │
│           │                                                     │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              Flight Control System                     │ │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────┐  │ │
│  │  │ PID Control │ │ State       │ │ Safety &            │  │ │
│  │  │ (X,Y,Z,Yaw) │ │ Machine     │ │ Smoothing           │  │ │
│  │  └─────────────┘ └─────────────┘ └─────────────────────┘  │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Key Components

### 1. Multi-Camera System
- **4-Camera Configuration**: Front (0°), Right (90°), Back (180°), Left (-90°)
- **Primary Camera**: Front camera (ID: 0) used for main person following
- **360° Coverage**: Complete situational awareness around the drone
- **Coordinate Transformation**: Each camera's view is transformed to drone body frame

#### Camera Configuration Details
| Camera ID | Position | Orientation | Role | Processing Rate |
|-----------|----------|-------------|------|-----------------|
| 0 (Front) | Primary | 0° (Forward) | Main following control, takeoff trigger | 15 Hz |
| 1 (Right) | Secondary | 90° (Right side) | Target re-acquisition, side tracking | 10 Hz |
| 2 (Back) | Secondary | 180° (Backward) | Lost target recovery, 360° coverage | 10 Hz |
| 3 (Left) | Secondary | -90° (Left side) | Target handoff, side tracking | 10 Hz |

### 2. Detection Systems

#### YOLOv7 + DeepSORT (Primary)
- **Object Detection**: YOLOv7 neural network for real-time object detection
- **Object Tracking**: DeepSORT for maintaining consistent target IDs across frames.
- **Person Classification**: Focuses on COCO class 0 (person) detection

#### Target Re-acquisition Logic 
```python 
# When target is lost from front camera, system checks all cameras
for camera_id in [1, 2, 3]:
    if person_detected_in_camera(camera_id):
        switch_primary_control_to(camera_id)
        maintain_following_behaviour()
```

#### OpenCV Motion Detection (Fallback)
- **Background Substraction**: Mixture of Gaussian (MOG) 2 background substractor
- **Motion Analysis**: Contour-based motion detection
- **Backup System**: Used when YOLOv7 + DeepSORT is unavailable

### 3. Motion Prediction System
The `MotionPredictor`clalss implements sophisticated target position prediction
- **Position History**: Maintains last 10 target positons 
- **Velocity Calculation**: Computes target velocity from position changes 
- **Future Position Prediction**: Predicts where target will be in next frame 
- **Smoothing**: Uses weighted averages for stability 

### 4. State Machine
The drone operates through several states:

```
IDLE → TAKING_OFF → HOVERING → FOLLOWING → SEARCHING
  ↑                    ↓           ↑           ↓
  └────────────────────┴───────────┴───────────┘
```

- **IDLE**: Waiting for person detection to initiate takeoff
- **TAKING_OFF**: Ascending to follow height
- **HOVERING**: Maintaining position, slow rotation search
- **FOLLOWING**: Actively tracking and following person
- **SEARCHING**: Lost target, systematic search pattern

### 5. PID Control System
Advanced control with separate PID controllers for each axis:

```python
# Control Parameters (optimized for smooth following)
self.pid_x = {'kp': 0.6, 'ki': 0.008, 'kd': 0.03}  # Forward/Backward
self.pid_y = {'kp': 0.7, 'ki': 0.005, 'kd': 0.02}  # Left/Right
self.pid_z = {'kp': 0.35, 'ki': 0.003, 'kd': 0.01} # Up/Down
self.pid_yaw = {'kp': 0.8, 'ki': 0.008, 'kd': 0.04} # Rotation
```

**Features**:
- **Integral Windup Protection**: Prevents integral term from growing too large
- **Derivative Limiting**: Reduces noise from rapid changes
- **Deadband Zones**: Prevents micro-movements for stability

## Detection Systems

### Multi-Camera Detection Workflow

The system utilises all 4 cameras simultaneously for comprehensive person tracking:

1. **Simultaneous Processing**: All cameras process incoming frames in parallel 
2. **Detection Fusion**: Results from all cameras are merged into global coordinate system
3. **Target Selection**: Best person target is selected across all camera views using confidence scoring
4. **Primary vs Secondary Role**
    - **Front Camera (Primary)**: Controls drone movement, gets confidence boost
    - **Side/Back Cameras**: Provide backup detection and handle target handoffs
5. **Camera Switching**: Control automatically switches when target moves between camera views 

#### Multi-Camera Usage Pattern
- **Primary Control**: Front camera drives main following behaviour and drone movement
- **Target Handoff**: When person moves out of front view, control switches to appropriate side/back camera
- **Continuous Monitoring**: All cameras simultaneously process for person detection 
- **Confidence Boost**: Front camera detections get 1.2x confidence multiplier in target selection
- **Re-acquisition**: Lost targets are searched across all 4 camera views
- **Coordinate Fusion**: All camera detections merged into unified drone body frame coordinates

#### Multi-Camera Benefits 
The 4-camera system provides several key advantages: 

**360° Awareness**
- Person walks behind drone → **Back camera** takes over tracking
- Person moves to drone's side → **Left/Right cameras** maintain visibility
- No blind spots during following maneuvers

**Robust Target Handoff**
```python
def select_target_person_multi_camera(self, merged_targets):
    # If target lost in primary camera, check all others
    if self.target_locked and self.target_camera_id is not None:
        for person in persons:
            camera_id = person.get('camera_id')
            if camera_id != self.target_camera_id:  # Check other cameras
                distance = calculate_position_distance(person, self.target_person_position)
                if distance < self.target_search_radius:  # Found in different camera
                    self.target_camera_id = camera_id  # Switch control
                    return person, camera_id
```

**Enhanced Search Patterns**
- Lost target triggers systematic check of all 4 cameras
- Rotation search benfits from continuous monitoring of all views
- Faster re-acquisition when target reappears in any camera field off view 

## Control Logic Deep Dive

### 1. Image Processing Pipeline

```python
def image_callback(self, msg, camera_id):
    # 1. Convert ROS Image → OpenCV
    cv_image = self.bridge.imgmsg_to_cv2(msg)
    
    # 2. Resize to target resolution
    cv_image = cv2.resize(cv_image, (640, 480))
    
    # 3. Run detection system
    if self.use_yolo_deepsort:
        targets = self.detect_and_track_yolo_deepsort(cv_image, camera_id)
    else:
        targets = self.detect_motion_opencv(cv_image, camera_id)
    
    # 4. Store results for multi-camera fusion
    self.all_camera_targets[camera_id] = targets
    
    # 5. Update person following (primary camera)
    if camera_id == self.primary_camera:
        self.update_target_data_async(targets, camera_id, msg.header)
```

### 2. Multi-Camera Fusion
The system merges detections from all cameras into a unified world coordinate system:

```python
def merge_camera_detections(self):
    merged_targets = []
    
    for cam_id, targets in self.all_camera_targets.items():  # ALL 4 cameras
        cam_orientation = self.camera_orientations[cam_id]
        
        for target in targets:
            # Transform each camera's coordinates to drone body frame
            cam_yaw = math.radians(cam_orientation['yaw'])
            world_x = target['world_x'] * math.cos(cam_yaw) - target['world_y'] * math.sin(cam_yaw)
            world_y = target['world_x'] * math.sin(cam_yaw) + target['world_y'] * math.cos(cam_yaw)
            
            transformed_target = {
                'world_x': world_x,
                'world_y': world_y,
                'camera_id': cam_id,
                'confidence': target['confidence']
            }
            merged_targets.append(transformed_target)
    
    return merged_targets
```

#### Camera Coordinate Transformations
Each camera's detection coordinates are transformed based on its orientation:
- **Front Camera (0°)**: No transformation needed (reference frame)
- **Right Camera (90°)**: Rotate coordinates 90° clockwise
- **Back Camera (180°)**: Rotate coordinates 180° 
- **Left Camera (-90°)**: Rotate coordinates 90° counter-clockwise

### 3. Distance Estimation
Uses person bounding box height to estimate distance:

```python
def estimate_person_distance(self, bbox):
    bbox_height = bbox[3]  # Bounding box height in pixels
    
    # Assume average person height of 1.7m
    raw_distance = (1.7 * self.image_height) / (
        bbox_height * math.tan(math.radians(self.camera_fov_vertical/2)) * 2
    )
    
    # Apply smoothing filter to reduce noise
    self.distance_history.append(raw_distance)
    smoothed_distance = np.median(list(self.distance_history))
    
    return smoothed_distance
```

### 4. Control Loop (15Hz)
The main control loop runs at 15Hz for responsive following:

```python
def control_loop(self):
    # 1. Get current target data
    person = self.target_person_data
    
    # 2. Calculate errors
    yaw_error = pixel_to_angle_conversion(target_center_x)
    distance_error = estimated_distance - follow_distance
    height_error = pixel_to_angle_conversion(target_center_y) 
    
    # 3. PID control
    yaw_cmd = self.pid_control(self.pid_yaw, yaw_error, dt)
    forward_cmd = self.pid_control(self.pid_x, distance_error, dt)
    height_cmd = self.pid_control(self.pid_z, height_error, dt)
    
    # 4. Apply safety limits and smoothing
    yaw_cmd = self.smooth_velocity_command('yaw', yaw_cmd)
    forward_cmd = self.smooth_velocity_command('x', forward_cmd)
    
    # 5. Publish velocity command
    vel_cmd = VelCmd()
    vel_cmd.twist.linear.x = forward_cmd
    vel_cmd.twist.angular.z = yaw_cmd
    self.cmd_vel_pub.publish(vel_cmd)
```

### 5. Search Behavior
When target is lost, the drone implements intelligent search:

```python
def momentum_search_behavior(self):
    # Continue in last known direction with rotation
    vel_cmd = VelCmd()
    vel_cmd.twist.linear.x = 0.4  # Slow forward
    
    # Variable rotation speed based on search time
    if time_in_search < 3.0:
        vel_cmd.twist.angular.z = 1.2  # Fast initial search
    elif time_in_search < 7.0:  
        vel_cmd.twist.angular.z = -0.8  # Reverse direction
    else:
        vel_cmd.twist.angular.z = 0.5   # Slow systematic scan
```

## Configuration Parameters

### Detection Parameters
```python
'confidence_threshold': 0.2    # Minimum detection confidence
'iou_threshold': 0.45         # Non-max suppression threshold  
'motion_threshold': 15.0      # Motion detection sensitivity (pixels)
```

### Following Parameters  
```python
'follow_distance': 0.5        # Target following distance (meters)
'follow_height': 3.0         # Flight altitude (meters)
'max_follow_speed': 4.0      # Maximum following speed (m/s)
```

### Camera Parameters
```python
'image_width': 640           # Processed image width
'image_height': 480          # Processed image height  
'image_quality': 95          # JPEG quality for visualization
```

### Control Parameters
```python
# PID gains optimized for smooth following
# Deadband zones to prevent jitter
# Velocity smoothing factors
# Acceleration limits
```

## Usage

### 1. Enable PX4
```bash 
export PX4_SIM_HOSTNAME=172.22.112.1
make px4_sitl_default none_iris
```

### 2. Launch Multirotor Node
```bash
source install/setup.bash
ros2 launch airsim_ros_pkgs single_drone.launch.py host_ip:=172.22.112.1 enableApiControl:=true
```

### 3. Launch Motion Detection Node
```bash
# Launch the motion detection node
ros2 run airsim_ros_pkgs motion_detection_node.py vehicle_name:=Drone1

# With custom parameters
ros2 run airsim_ros_pkgs motion_detection_node.py --ros-args \
  -p vehicle_name:=Drone1 \
  -p enable_following:=true \
  -p follow_distance:=3.0
```

Ensure AirSim is running with proper camera configuration:
```json
{
  "SettingsVersion": 2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "RpcEnabled": true,
      "TcpPort": 4560,
      "ControlIp": "remote",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "172.22.112.1",
      "X": -5.0,
      "Y": 0.0,
      "Z": -1.0,
      "Yaw": 0.0,
      "Cameras": {
        "0": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1280,
              "Height": 720,
              "FOV_Degrees": 90,
              "AutoExposureSpeed": 100,
              "MotionBlurAmount": 0
            }
          ],
          "X": 0.90, "Y": 0.00, "Z": -0.05,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        },
        "1": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1280,
              "Height": 720,
              "FOV_Degrees": 90,
              "AutoExposureSpeed": 100,
              "MotionBlurAmount": 0
            }
          ],
          "X": 0.40, "Y": 0.70, "Z": -0.05,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 90.0
        },
        "2": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1280,
              "Height": 720,
              "FOV_Degrees": 90,
              "AutoExposureSpeed": 100,
              "MotionBlurAmount": 0
            }
          ],
          "X": -0.50, "Y": 0.00, "Z": -0.05,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 180.0
        },
        "3": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1280,
              "Height": 720,
              "FOV_Degrees": 90,
              "AutoExposureSpeed": 100,
              "MotionBlurAmount": 0
            }
          ],
          "X": 0.40, "Y": -0.70, "Z": -0.05,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": -90.0
        }
      },
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled": true
        },
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled": true
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled": true
        },
        "Lidar1": {
          "SensorType": 6,
          "Enabled": true,
          "NumberOfChannels": 16,
          "Range": 100,
          "PointsPerSecond": 10000,
          "DrawDebugPoints": false,
          "X": 0,
          "Y": 0,
          "Z": 0,
          "Roll": 0,
          "Pitch": 0,
          "Yaw": 0
        }
      }
    }
  },
  "PawnPaths": {
    "DefaultQuadrotor": {
      "PawnBP": "Class'/AirSim/Blueprints/BP_MyPawn.BP_MyPawn_C'"
    }
  }
}
```

### 4. Monitoring
```bash
export PATH=$PATH:/opt/ros/humble/bin:/opt/ros/humble/lib/rqt_image_view
rqt_image_view

# View detection output
ros2 topic echo /target_detection

# Monitor velocity commands  
ros2 topic echo /drone1/vel_cmd_body_frame

# Check node status
ros2 node info /multi_camera_motion_detection_node
```

## Dependencies

### Required Python Packages
```bash
pip install torch torchvision  # PyTorch for YOLOv7
pip install opencv-python      # Computer vision
pip install numpy scipy       # Numerical computing  
```

### Required ROS2 Packages
```bash
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
```

### Optional (YOLOv7 + DeepSORT)
```bash
# Clone YOLOv7-DeepSORT repository
git clone https://github.com/augmentedstartups/YOLOv7-DeepSORT-Object-Tracking.git
```

## Troubleshooting

### Common Issues

1. **YOLOv7 Import Errors**
   - Verify YOLOv7-DeepSORT path is correct
   - Check PyTorch installation
   - Falls back to OpenCV detection automatically

2. **Camera Topics Not Found**
   - Verify AirSim camera configuration
   - Check vehicle name parameter matches AirSim
   - Ensure proper ROS2-AirSim bridge is running

3. **Poor Following Performance**
   - Adjust PID parameters for your environment
   - Check camera calibration and FOV settings
   - Verify image resolution matches expectations

4. **Target Detection Issues**
   - Adjust confidence threshold
   - Improve lighting conditions  
   - Check person is visible in Unreal Engine

### Debug Features
- Real-time visualization shows detection boxes and tracking
- Console logging every 5 seconds shows system status
- Frame counters and performance metrics
- State machine status reporting

## Extension Points

The system is designed for extensibility:

### Adding New Detection Classes
Modify the target selection logic to detect other objects beyond moving persons.

### Custom Control Strategies  
Implement new PID parameters or control algorithms in the control loop.

### Additional Sensors
Integrate IMU, GPS, or depth sensors for enhanced navigation.

### Multi-Drone Coordination
Extend to coordinate multiple drones for collaborative tracking.

---

## License & Attribution

This motion detection system builds upon several open-source projects:
- YOLOv7: Object detection neural network
- DeepSORT: Multi-object tracking algorithm  
- OpenCV: Computer vision library
- ROS2: Robot operating system framework
