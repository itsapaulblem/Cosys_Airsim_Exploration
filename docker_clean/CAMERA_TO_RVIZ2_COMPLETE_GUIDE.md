# Complete Camera Configuration: AirSim → ROS2 → RViz2

## 🎯 **Overview**

This guide provides a complete pipeline for configuring AirSim cameras, streaming data through ROS2, and visualizing in RViz2. Based on comprehensive codebase analysis and existing ROS2 wrapper implementation.

## 📋 **Data Flow Architecture**

```
┌─────────────┐    RPC/TCP     ┌──────────────┐    ROS2 Topics    ┌─────────────┐
│   AirSim    │◄──────────────►│   ROS2       │◄─────────────────►│   RViz2     │
│  (Unreal)   │   41451        │   Wrapper    │   Image Topics    │ Visualization│
│             │                │              │                   │             │
│ • Cameras   │                │ • airsim_node│                   │ • Image     │
│ • Sub-Windows│               │ • Topic Pub  │                   │ • Camera    │
│ • Rendering │                │ • Timer Poll │                   │ • PointCloud│
└─────────────┘                └──────────────┘                   └─────────────┘
```

## 🔧 **Step 1: Configure AirSim Cameras**

### **Available Camera Types**

| ImageType | Description | ROS2 Topic Suffix | Use Case |
|-----------|-------------|------------------|----------|
| 0 | Scene (RGB) | `/Scene` | General vision, navigation |
| 1 | DepthPlanar | `/DepthPlanar` | Depth estimation, SLAM |
| 2 | DepthPerspective | `/DepthPerspective` | 3D reconstruction |
| 3 | DepthVis | `/DepthVis` | Visualization |
| 4 | DisparityNormalized | `/DisparityNormalized` | Stereo vision |
| 5 | Segmentation | `/Segmentation` | Object detection, ML |
| 6 | SurfaceNormals | `/SurfaceNormals` | Surface analysis |
| 7 | Infrared | `/Infrared` | Thermal imaging |

### **Generate Camera-Enabled Settings**

```bash
cd docker_clean/testing_scripts

# Basic setup: 2 drones with RGB and depth cameras
python3 generate_camera_settings.py --num-drones 2 --cameras front_rgb front_depth

# Advanced setup: Multiple camera types with RViz config
python3 generate_camera_settings.py --num-drones 1 \
    --cameras front_rgb front_depth segmentation_cam downward_cam \
    --create-rviz

# Stereo setup for depth estimation
python3 generate_camera_settings.py --num-drones 1 \
    --cameras stereo_left stereo_right front_depth \
    --create-rviz
```

### **Camera Presets Available**

| Preset | Type | Resolution | Position | Use Case |
|--------|------|------------|----------|----------|
| `front_rgb` | RGB | 1920x1080 | Forward-facing | General navigation |
| `front_depth` | Depth | 640x480 | Forward-facing | Obstacle avoidance |
| `downward_cam` | RGB | 640x480 | Downward (-90°) | Landing, ground tracking |
| `segmentation_cam` | Segmentation | 640x480 | Forward-facing | Object detection |
| `stereo_left` | RGB | 640x480 | Left offset | Stereo vision |
| `stereo_right` | RGB | 640x480 | Right offset | Stereo vision |
| `gimbal_cam` | RGB | 1920x1080 | Gimbal (-45°) | Inspection, photography |

### **Manual Camera Configuration**

```json
{
  "Vehicles": {
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "Cameras": {
        "front_rgb": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.50, "Y": 0.00, "Z": 0.10,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        },
        "front_depth": {
          "CaptureSettings": [
            {
              "ImageType": 1,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.50, "Y": 0.00, "Z": 0.10,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        }
      }
    }
  },
  "SubWindows": [
    {
      "WindowID": 0,
      "CameraName": "front_rgb",
      "ImageType": 0,
      "VehicleName": "PX4_Drone1",
      "Visible": true
    }
  ]
}
```

## 🐳 **Step 2: Start ROS2 Docker Wrapper**

### **Option A: VNC Desktop (Recommended for RViz2)**

```bash
# Windows
cd docker/airsim_ros2_wrapper
./run_vnc.bat

# Access GUI
# Web Browser: http://localhost:6901/vnc.html
# Password: airsim123
```

### **Option B: Headless (Terminal Only)**

```bash
# Windows
cd docker/airsim_ros2_wrapper
./run_simple.bat
```

### **Verify ROS2 Connection**

```bash
# Check if AirSim node is running
./ros2_exec.bat "ros2 node list | grep airsim"

# List available camera topics
./ros2_exec.bat "ros2 topic list | grep camera"

# Example topics you should see:
# /airsim_node/PX4_Drone1/front_rgb/Scene
# /airsim_node/PX4_Drone1/front_depth/DepthPlanar
# /airsim_node/PX4_Drone1/front_rgb/camera_info
```

## 📺 **Step 3: Configure RViz2 Visualization**

### **Auto-Generated RViz Configuration**

If you used `--create-rviz` option, load the configuration:

```bash
# In VNC desktop terminal
rviz2 -d /path/to/airsim_cameras.rviz
```

### **Manual RViz2 Setup**

1. **Start RViz2**: `rviz2`

2. **Set Fixed Frame**: Change from `map` to `world_ned`

3. **Add Image Displays**:
   - Click "Add" → "By topic"
   - Select `/airsim_node/PX4_Drone1/front_rgb/Scene`
   - Choose "Image" display type
   - Repeat for other cameras

4. **Add Camera Display** (for 3D projection):
   - Add "Camera" display
   - Set Image Topic: `/airsim_node/PX4_Drone1/front_rgb/Scene`
   - RViz will automatically find the corresponding `camera_info` topic

### **RViz2 Camera Display Configuration**

```yaml
# Example RViz2 config for multiple cameras
Displays:
  - Class: rviz_default_plugins/Image
    Enabled: true
    Name: "Drone1 RGB"
    Topic:
      Value: "/airsim_node/PX4_Drone1/front_rgb/Scene"
    Transport Hint: "raw"
    
  - Class: rviz_default_plugins/Image
    Enabled: true
    Name: "Drone1 Depth"
    Topic:
      Value: "/airsim_node/PX4_Drone1/front_depth/DepthPlanar"
    Transport Hint: "raw"
    
  - Class: rviz_default_plugins/Camera
    Enabled: true
    Name: "3D Camera View"
    Topic: "/airsim_node/PX4_Drone1/front_rgb/Scene"
```

## 🔧 **Step 4: Advanced Camera Operations**

### **Access Camera Data Programmatically**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        # Subscribe to RGB camera
        self.rgb_subscription = self.create_subscription(
            Image,
            '/airsim_node/PX4_Drone1/front_rgb/Scene',
            self.rgb_callback,
            10
        )
        
        # Subscribe to depth camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/airsim_node/PX4_Drone1/front_depth/DepthPlanar',
            self.depth_callback,
            10
        )
    
    def rgb_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Process image
        cv2.imshow('RGB Camera', cv_image)
        cv2.waitKey(1)
    
    def depth_callback(self, msg):
        # Convert depth image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        
        # Normalize for visualization
        normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow('Depth Camera', normalized)
        cv2.waitKey(1)

def main():
    rclpy.init()
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Monitor Camera Topics**

```bash
# Check topic info
./ros2_exec.bat "ros2 topic info /airsim_node/PX4_Drone1/front_rgb/Scene"

# Monitor publishing rate
./ros2_exec.bat "ros2 topic hz /airsim_node/PX4_Drone1/front_rgb/Scene"

# Echo camera info
./ros2_exec.bat "ros2 topic echo /airsim_node/PX4_Drone1/front_rgb/camera_info"
```

## 🎮 **Complete Workflow Example**

### **1. Generate Multi-Drone Camera Setup**

```bash
cd docker_clean/testing_scripts
python3 generate_camera_settings.py --num-drones 3 \
    --cameras front_rgb front_depth downward_cam \
    --create-rviz
```

### **2. Start AirSim**

```bash
# AirSim will automatically load ~/Documents/AirSim/settings.json
./YourUnrealProject.exe
```

### **3. Start ROS2 Wrapper**

```bash
cd docker/airsim_ros2_wrapper
./run_vnc.bat
```

### **4. Verify Camera Streams**

```bash
# In VNC terminal
ros2 topic list | grep Scene
# Expected output:
# /airsim_node/PX4_Drone1/front_rgb/Scene
# /airsim_node/PX4_Drone2/front_rgb/Scene  
# /airsim_node/PX4_Drone3/front_rgb/Scene
```

### **5. Launch RViz2**

```bash
# Load pre-configured camera displays
rviz2 -d /path/to/airsim_cameras.rviz
```

## 🐛 **Troubleshooting**

### **No Camera Topics Appearing**

```bash
# Check if AirSim is connected
./ros2_exec.bat "ros2 topic echo /airsim_node/PX4_Drone1/odom_local_ned --max-count 1"

# Check AirSim node status
./ros2_exec.bat "ros2 node info /airsim_node"

# Restart AirSim node
./ros2_exec.bat "ros2 lifecycle set /airsim_node deactivate"
./ros2_exec.bat "ros2 lifecycle set /airsim_node activate"
```

### **RViz2 Not Displaying Images**

- **Check transport**: Use "raw" transport hint in RViz2
- **Verify topics**: Ensure topic names match exactly
- **Check encoding**: RGB images use "bgr8", depth uses "32FC1"

### **Performance Issues**

```bash
# Reduce image resolution in settings.json
"Width": 320, "Height": 240

# Limit publishing rate
# In airsim_node launch parameters:
publish_clock: false
```

## 📊 **Camera Topic Reference**

### **Complete Topic List**

```bash
# RGB Camera
/airsim_node/{vehicle}/front_rgb/Scene
/airsim_node/{vehicle}/front_rgb/camera_info

# Depth Camera  
/airsim_node/{vehicle}/front_depth/DepthPlanar
/airsim_node/{vehicle}/front_depth/camera_info

# Segmentation
/airsim_node/{vehicle}/segmentation_cam/Segmentation
/airsim_node/{vehicle}/segmentation_cam/camera_info

# Downward Camera
/airsim_node/{vehicle}/downward_cam/Scene
/airsim_node/{vehicle}/downward_cam/camera_info
```

### **Message Types**

```bash
# Image topics
sensor_msgs/msg/Image

# Camera info topics  
sensor_msgs/msg/CameraInfo

# Compressed (if available)
sensor_msgs/msg/CompressedImage
```

## 🎯 **Quick Reference Commands**

```bash
# Generate camera settings
python3 generate_camera_settings.py --num-drones 2 --cameras front_rgb front_depth

# Start VNC ROS2 wrapper
docker/airsim_ros2_wrapper/run_vnc.bat

# Access VNC desktop
# http://localhost:6901/vnc.html (password: airsim123)

# List camera topics
ros2 topic list | grep Scene

# Monitor camera data
ros2 topic echo /airsim_node/PX4_Drone1/front_rgb/Scene --max-count 1

# Launch RViz2
rviz2
```

This comprehensive guide provides everything needed to configure cameras in AirSim, stream the data through ROS2, and visualize it in RViz2, with both automated generation tools and manual configuration options.