# RViz Configuration for AirSim LiDAR Visualization

This directory contains RViz configuration files optimized for AirSim multi-vehicle LiDAR visualization.

## Quick Start

Launch RViz with optimized LiDAR visualization:

```bash
# From Docker container or ROS2 workspace
rviz2 -d ~/ros2/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz

# If in VNC:
rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz
```

## Available Configurations

### `lidar_visualization.rviz`
**Purpose**: Optimized for AirSim multi-vehicle LiDAR point cloud visualization  
**Features**:
- **Fixed Frame**: `map` (REP 105 compliant)
- **Multi-vehicle support**: Drone_1 (white), D2 (red), MyDrone3 (green)
- **Timestamp synchronized**: Compatible with ROS2 system time
- **Optimized for sparse data**: Large point sizes for low-density clouds
- **TF tree visualization**: Shows complete transform hierarchy

**Vehicle Configuration**:
- **Drone_1**: Dense LiDAR (3000+ points), 8px points, white color
- **D2**: Sparse LiDAR (50+ points), 15px points, red color  
- **MyDrone3**: Medium LiDAR, 12px points, green color

### `default.rviz`
**Purpose**: General AirSim visualization (legacy configuration)

## Configuration Details

### Fixing "0 points from 0 messages" Issues

If you see "0 points from 0 messages", check these settings:

#### 1. Fixed Frame Configuration
```yaml
Global Options:
  Fixed Frame: map  # MUST match your TF tree root
```

**Available Frames**:
- `map` - Global reference frame (recommended)
- `{vehicle}/odom` - Vehicle-specific odometry frame
- `{vehicle}/base_link` - Vehicle body frame

#### 2. Point Cloud Topic Settings
```yaml
Topic:
  Value: /Drone_1/LidarSensor1/points  # Exact topic name
  Reliability Policy: Reliable         # Must match publisher
  Durability Policy: Volatile          # Must match publisher
```

**Available Topics** (check with `ros2 topic list`):
- `/Drone_1/LidarSensor1/points`
- `/D2/LidarSensor1/points`
- `/MyDrone3/LidarSensor1/points`

#### 3. Transform Frame Settings
```yaml
PointCloud2:
  Use Fixed Frame: true  # Essential for multi-robot visualization
```

### Optimizing Visualization for Different Data Densities

#### Dense LiDAR Data (3000+ points)
```yaml
Size (Pixels): 3-8      # Small points for dense clouds
Decay Time: 0           # No accumulation needed
Style: Points           # Standard point rendering
```

#### Sparse LiDAR Data (50-200 points)
```yaml
Size (Pixels): 12-20    # Large points for visibility
Decay Time: 2-5         # Accumulate over time
Style: Points           # Or try Spheres for maximum visibility
Color: High contrast    # Bright colors (red, green, white)
```

#### Ultra-Sparse LiDAR Data (<50 points)
```yaml
Size (Pixels): 20+      # Maximum point size
Size (m): 0.1-0.2      # Physical size for spheres
Decay Time: 5-10        # Long accumulation
Style: Spheres          # 3D spheres more visible than points
```

### Color Configuration Options

#### By Intensity
```yaml
Color Transformer: Intensity
Use rainbow: true
```

#### By Height (Z-coordinate)
```yaml
Color Transformer: AxisColor
Axis: Z
Use rainbow: true
```

#### Fixed Colors (Multi-vehicle)
```yaml
Color Transformer: FlatColor
Color: [255, 0, 0]  # RGB values
```

**Recommended Vehicle Colors**:
- **Drone_1**: White (255, 255, 255) - Primary vehicle
- **D2**: Red (255, 0, 0) - Secondary vehicle
- **MyDrone3**: Green (0, 255, 0) - Third vehicle

### Camera Positioning for AirSim Coordinates

#### Default View Settings
```yaml
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 25                    # Camera distance
    Focal Point:
      X: -5                        # Center of typical AirSim LiDAR data
      Y: 2                         # Adjust based on vehicle positions
      Z: -1                        # Ground level reference
    Pitch: 0.28                    # Downward viewing angle
    Yaw: 0.58                      # Side viewing angle
```

#### Coordinate Range Optimization
Based on typical AirSim LiDAR coordinates:

**Dense Data (Drone_1)**:
- X: -13 to -2m, Y: ~0m, Z: -1.2m
- Camera Focus: X: -8, Y: 0, Z: -1

**Sparse Data (D2)**:
- X: 28-32m, Y: -36 to -27m, Z: -19m  
- Camera Focus: X: 30, Y: -32, Z: -19

## Troubleshooting Guide

### Issue: "Invalid frame ID" Errors

**Cause**: TF frame mismatch between Fixed Frame and message frames

**Solutions**:
1. **Check available frames**: `ros2 run tf2_tools view_frames.py`
2. **Test frame connectivity**: `ros2 run tf2_ros tf2_echo map Drone_1/LidarSensor1_link`
3. **Update Fixed Frame**: Set to `map` or available root frame

### Issue: "Message Filter dropping message: timestamp earlier than transform cache"

**Cause**: Timestamp synchronization mismatch (SOLVED in latest code)

**Solutions**:
1. **Verify timestamp fix**: Check that LiDAR publishers use `this->get_clock()->now()`
2. **Rebuild packages**: `colcon build --packages-select airsim_ros_pkgs`
3. **Restart nodes**: Fresh launch with updated code

### Issue: Points Visible but Outside View

**Cause**: Camera positioned outside point cloud area

**Solutions**:
1. **Reset view**: Click "Views" → "Current View" → "Focal Point" 
2. **Auto-center**: Use mouse middle-click + drag to pan
3. **Adjust distance**: Mouse wheel to zoom in/out
4. **Use Focus tool**: Click "Focus Camera" tool and click on point cloud

### Issue: Points Too Small to See

**Cause**: Point size not optimized for data density

**Solutions**:
1. **Increase point size**: Set "Size (Pixels)" to 10-20
2. **Enable spheres**: Change "Style" from "Points" to "Spheres"
3. **Set physical size**: Adjust "Size (m)" for sphere rendering
4. **High contrast color**: Use bright colors on dark background

## Advanced Configuration

### Custom Launch Integration

Create a launch file to automatically load your RViz configuration:

```python
# launch/airsim_rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('airsim_ros_pkgs'),
        'rviz',
        'lidar_visualization.rviz'
    )
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
```

### TF Frame Reference

Understanding the AirSim TF tree structure:

```
map
├── Drone_1/odom
│   └── Drone_1/base_link
│       └── Drone_1/LidarSensor1_link
├── D2/odom  
│   └── D2/base_link
│       └── D2/LidarSensor1_link
└── MyDrone3/odom
    └── MyDrone3/base_link
        └── MyDrone3/LidarSensor1_link
```

### QoS Profile Matching

Ensure RViz subscription matches publisher settings:

```yaml
Topic:
  Reliability Policy: Reliable    # Must match airsim_ros_pkgs publisher
  Durability Policy: Volatile     # Must match airsim_ros_pkgs publisher
  History Policy: Keep Last       # Recommended for real-time data
  Depth: 5                        # Buffer size
```

## Performance Optimization

### For Real-time Visualization
```yaml
Global Options:
  Frame Rate: 30          # High refresh rate
  
PointCloud2:
  Decay Time: 0           # No point accumulation
  Filter size: 10         # Limit processing
```

### For Recording/Analysis
```yaml
Global Options:
  Frame Rate: 10          # Lower refresh rate
  
PointCloud2:
  Decay Time: 5           # Accumulate for better coverage
  Filter size: 100        # Process more points
```

## Validation Checklist

Before reporting visualization issues, verify:

- [ ] **Topics publishing**: `ros2 topic hz /Drone_1/LidarSensor1/points`
- [ ] **TF connectivity**: `ros2 run tf2_ros tf2_echo map Drone_1/LidarSensor1_link`  
- [ ] **Message content**: `ros2 topic echo --once /Drone_1/LidarSensor1/points`
- [ ] **Fixed Frame**: Matches available TF root frame
- [ ] **Point size**: Appropriate for data density
- [ ] **Camera position**: Within coordinate range of data
- [ ] **Timestamp sync**: No "dropping message" errors in logs

## Support

For additional help:
1. **Check logs**: RViz console output shows detailed error messages
2. **Enable debug**: `RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] [{severity}]: {message}"`
3. **TF debugging**: `ros2 run tf2_tools view_frames.py && evince frames.pdf`