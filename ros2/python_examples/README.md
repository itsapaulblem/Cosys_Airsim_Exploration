# AirSim ROS2 Python Examples

This directory contains Python examples for interacting with Cosys-AirSim through ROS2.

## Prerequisites

### System Requirements
- ROS2 (Foxy/Galactic/Humble)
- Python 3.8+
- AirSim simulation environment
- Built AirSim ROS2 workspace

### Installation

1. **Build the AirSim ROS2 workspace:**
   ```bash
   cd /path/to/cosys-airsim/ros2
   colcon build
   source install/setup.bash
   ```

2. **Install Python dependencies:**
   ```bash
   pip install rclpy numpy matplotlib
   ```

## Available Examples

### 1. AirSim ROS2 Subscriber (`airsim_ros2_subscriber.py`)

A comprehensive subscriber example that demonstrates:
- Subscribing to all major AirSim ROS2 topics
- Real-time vehicle state monitoring
- Sensor data processing
- Command publishing
- Service calling
- Data logging

#### Usage

**Basic usage:**
```bash
python3 airsim_ros2_subscriber.py
```

**With specific vehicle:**
```bash
python3 airsim_ros2_subscriber.py --vehicle drone_1
```

**With data logging:**
```bash
python3 airsim_ros2_subscriber.py --log-file airsim_data.log
```

**Run demonstration:**
```bash
python3 airsim_ros2_subscriber.py --demo
```

#### Command Line Options

- `--vehicle, -v`: Specify vehicle name (default: all vehicles)
- `--log-file, -l`: Enable data logging to file
- `--no-logging`: Disable console logging
- `--demo`: Run demonstration commands

#### Features

**Topic Subscriptions:**
- `/odom_local` - Vehicle odometry
- `/global_gps` - GPS position
- `/environment` - Environment data
- `/imu` - IMU sensor data
- `/magnetometer` - Magnetometer data
- `/instance_segmentation` - Object instance segmentation
- `/object_transforms` - Dynamic object transforms

**Command Publishing:**
- Velocity commands (world/body frame)
- Gimbal control commands

**Service Calls:**
- Takeoff/Land
- Set altitude
- Reset simulation
- Refresh segmentation

**Data Logging:**
- JSON format logging
- Real-time statistics
- Message frequency monitoring

## ROS2 Topic Structure

### Core Vehicle Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/odom_local` | `nav_msgs/Odometry` | Vehicle position and velocity |
| `/global_gps` | `sensor_msgs/NavSatFix` | GPS coordinates |
| `/environment` | `airsim_interfaces/Environment` | Environmental conditions |

### Sensor Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/imu` | `sensor_msgs/Imu` | IMU acceleration and angular velocity |
| `/magnetometer` | `sensor_msgs/MagneticField` | Magnetic field measurements |
| `/camera_0/image_raw` | `sensor_msgs/Image` | Camera images |
| `/lidar` | `sensor_msgs/PointCloud2` | LiDAR point clouds |

### AirSim-Specific Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/instance_segmentation` | `airsim_interfaces/InstanceSegmentationList` | Object instance masks |
| `/object_transforms` | `airsim_interfaces/ObjectTransformsList` | Dynamic object poses |

### Command Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/vel_cmd_world_frame` | `airsim_interfaces/VelCmd` | Velocity commands (world frame) |
| `/vel_cmd_body_frame` | `airsim_interfaces/VelCmd` | Velocity commands (body frame) |

## Usage Examples

### 1. Basic Subscription

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_local',
            self.odom_callback,
            10)
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        print(f"Position: {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}")

def main():
    rclpy.init()
    subscriber = SimpleSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()
```

### 2. Sending Velocity Commands

```python
from airsim_interfaces.msg import VelCmd

class VelCommandPublisher(Node):
    def __init__(self):
        super().__init__('vel_command_publisher')
        self.publisher = self.create_publisher(VelCmd, '/vel_cmd_world_frame', 10)
        
    def move_forward(self, speed=1.0):
        msg = VelCmd()
        msg.vx = speed
        msg.vy = 0.0
        msg.vz = 0.0
        msg.yaw_rate = 0.0
        self.publisher.publish(msg)
```

### 3. Calling Services

```python
from airsim_interfaces.srv import Takeoff
import rclpy

def call_takeoff():
    rclpy.init()
    node = rclpy.create_node('takeoff_client')
    
    client = node.create_client(Takeoff, '/takeoff')
    client.wait_for_service()
    
    request = Takeoff.Request()
    request.waitonfull_timeout = 10.0
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result().success:
        print("Takeoff successful!")
    else:
        print("Takeoff failed!")
    
    rclpy.shutdown()
```

## Multi-Vehicle Support

For multi-vehicle setups, topics are namespaced by vehicle name:

```python
# For vehicle named "drone_1"
vehicle_topics = {
    'odometry': '/drone_1/odom_local',
    'gps': '/drone_1/global_gps',
    'imu': '/drone_1/imu',
    'vel_cmd': '/drone_1/vel_cmd_world_frame'
}
```

## Running with AirSim

### 1. Start AirSim Simulation

```bash
# Start AirSim (Windows)
cd C:\path\to\Unreal\Environment
Environment.exe

# Start AirSim (Linux)
cd /path/to/Unreal/Environment
./Environment.sh
```

### 2. Launch AirSim ROS2 Node

```bash
# Terminal 1: Source workspace
cd /path/to/cosys-airsim/ros2
source install/setup.bash

# Launch AirSim node
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### 3. Run Python Subscriber

```bash
# Terminal 2: Run subscriber
cd /path/to/cosys-airsim/ros2/python_examples
python3 airsim_ros2_subscriber.py --demo
```

## Troubleshooting

### Common Issues

1. **"No module named 'airsim_interfaces'"**
   - Solution: Build and source the ROS2 workspace
   ```bash
   cd /path/to/cosys-airsim/ros2
   colcon build
   source install/setup.bash
   ```

2. **"AirSim connection failed"**
   - Solution: Ensure AirSim is running and accessible
   - Check that AirSim is listening on localhost:41451

3. **"No topics published"**
   - Solution: Ensure `airsim_node` is running
   - Check topic list: `ros2 topic list`

4. **"Service call failed"**
   - Solution: Check if service is available
   - List services: `ros2 service list`

### Debug Commands

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /odom_local

# Echo topic data
ros2 topic echo /odom_local

# List services
ros2 service list

# Show service type
ros2 service type /takeoff
```

## Performance Tips

1. **Use appropriate QoS settings:**
   - `BEST_EFFORT` for high-frequency sensor data
   - `RELIABLE` for important state information

2. **Use callback groups for parallel processing:**
   ```python
   from rclpy.callback_groups import ReentrantCallbackGroup
   callback_group = ReentrantCallbackGroup()
   ```

3. **Limit message frequency for logging:**
   ```python
   if self.message_count % 10 == 0:  # Log every 10th message
       self.get_logger().info(f'Message: {data}')
   ```

## Data Logging and Analysis

The subscriber supports JSON logging for data analysis:

```python
# Enable logging
python3 airsim_ros2_subscriber.py --log-file data.log

# Analyze logged data
import json
with open('data.log', 'r') as f:
    for line in f:
        if line.startswith('#'):
            continue
        timestamp, msg_type, data = line.strip().split(',', 2)
        parsed_data = json.loads(data)
        # Process data...
```

## Contributing

To add new examples:

1. Create a new Python file in this directory
2. Follow the existing code structure
3. Add documentation to this README
4. Test with different vehicle configurations

## License

This code is part of the Cosys-AirSim project and follows the same MIT license terms.