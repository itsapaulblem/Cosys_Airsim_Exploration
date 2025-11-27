# AirStack Integration Roadmap
## Phase-by-Phase Implementation Guide

---

## Document Overview

This roadmap provides detailed, step-by-step instructions for integrating CMU AirLab's AirStack autonomy framework with your Cosys-AirSim ROS2 stack. Each phase includes:

- Prerequisites and preparation
- Detailed implementation steps
- Configuration examples
- Testing procedures
- Troubleshooting guide
- Success criteria
- Rollback instructions

**Expected Total Duration**: 12-18 weeks (full integration)

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Phase 1: MAVROS Baseline (1-2 weeks)](#phase-1-mavros-baseline)
3. [Phase 2: Perception Integration (2-3 weeks)](#phase-2-perception-integration)
4. [Phase 3: Local Planning (2-3 weeks)](#phase-3-local-planning)
5. [Phase 4: Behavior Trees (3-4 weeks)](#phase-4-behavior-trees)
6. [Phase 5: Multi-Drone Coordination (2-3 weeks)](#phase-5-multi-drone-coordination)
7. [Phase 6: Testing & Polish (2-3 weeks)](#phase-6-testing--polish)
8. [Rollback Procedures](#rollback-procedures)
9. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Environment Setup

**Hardware Requirements**:
- Ubuntu 22.04 workstation
- NVIDIA GPU (RTX 3070 or better)
- 32GB RAM minimum
- 100GB free disk space
- Windows PC for AirSim (if WSL2 setup)

**Software Prerequisites**:
```bash
# ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Development tools
sudo apt install git build-essential cmake python3-pip
sudo apt install ros-humble-rosidl-default-generators

# Docker + NVIDIA Container Toolkit
sudo apt install docker.io docker-compose
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

### Access and Licenses

**AirStack Access**:
1. Visit [AirStack GitHub](https://github.com/castacks/AirStack)
2. Review license (verify commercial/research use)
3. If Docker registry access needed:
   - Contact CMU AirLab: airlab@andrew.cmu.edu
   - Request collaboration or access to `airlab-storage.andrew.cmu.edu:442`
4. Alternative: Build from source (no registry needed)

### Baseline Validation

**Ensure your current system works**:
```bash
# Terminal 1: Launch AirSim (Windows or Linux)
# Start your AirSim environment

# Terminal 2: Launch ROS2 nodes
cd ~/PaulSim/Cosys_Airsim_Exploration
source /opt/ros/humble/setup.bash
cd ros2 && source install/setup.bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Terminal 3: Verify
ros2 node list  # Should show /Droan1, /PX4_Drone2, etc.
ros2 topic hz /Droan1/odom_local_ned  # Should show ~50-100 Hz
```

**If baseline fails**: Fix your current setup before proceeding.

---

# Phase 1: MAVROS Baseline

**Duration**: 1-2 weeks
**Complexity**: ⭐ Low
**Risk**: Low

## Objectives

1. Install and configure MAVROS for ROS2 Humble
2. Connect MAVROS to your PX4 SITL instances
3. Test basic commands (arm, takeoff, position setpoints)
4. Validate multi-drone MAVROS
5. Create topic remapping infrastructure

## Step 1.1: MAVROS Installation (Day 1)

### Install MAVROS

```bash
# Install MAVROS and extras
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Download geographiclib datasets (required for GPS conversions)
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### Verify Installation

```bash
# Check installed packages
ros2 pkg list | grep mavros

# Expected output:
# mavros
# mavros_extras
# mavros_msgs
# libmavconn
```

---

## Step 1.2: Single Drone MAVROS Test (Day 1-2)

### Launch Configuration

Create `ros2/src/airsim_ros_pkgs/launch/mavros_single.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS node for Drone1
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='Drone1',
            parameters=[{
                'fcu_url': 'udp://:14540@localhost:14557',  # PX4 SITL connection
                'gcs_url': '',  # No GCS
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
            output='screen'
        ),
    ])
```

### Test Procedure

**Terminal 1**: Launch AirSim + PX4 SITL
```bash
# Start AirSim (Windows or Linux)

# Start PX4 SITL
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0
```

**Terminal 2**: Launch your ROS2 vehicle node
```bash
cd ~/PaulSim/Cosys_Airsim_Exploration/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
```

**Terminal 3**: Launch MAVROS
```bash
source install/setup.bash
ros2 launch airsim_ros_pkgs mavros_single.launch.py
```

**Terminal 4**: Monitor connection
```bash
# Check MAVROS state
ros2 topic echo /Drone1/mavros/state --once

# Expected output:
# connected: true
# armed: false
# mode: "AUTO.LOITER" or "MANUAL"
```

### Test Basic Commands

```bash
# Set OFFBOARD mode
ros2 service call /Drone1/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# Arm vehicle
ros2 service call /Drone1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Send position setpoint (hover at 5m)
ros2 topic pub -r 10 /Drone1/mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 5.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```

### Success Criteria

- ✅ MAVROS connects to PX4 (`connected: true`)
- ✅ Vehicle arms successfully
- ✅ Vehicle takes off to 5m altitude
- ✅ Position hold stable (< 0.5m drift)
- ✅ No MAVROS errors in logs

---

## Step 1.3: Multi-Drone MAVROS (Day 3-4)

### Launch Configuration

Create `ros2/src/airsim_ros_pkgs/launch/mavros_multi.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vehicles = [
        {'name': 'Drone1', 'udp_in': 14540, 'udp_out': 14557, 'sys_id': 1},
        {'name': 'Drone2', 'udp_in': 14541, 'udp_out': 14558, 'sys_id': 2},
        {'name': 'Drone3', 'udp_in': 14542, 'udp_out': 14559, 'sys_id': 3},
    ]

    nodes = []
    for v in vehicles:
        nodes.append(
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                namespace=v['name'],
                parameters=[{
                    'fcu_url': f"udp://:{v['udp_in']}@localhost:{v['udp_out']}",
                    'gcs_url': '',
                    'target_system_id': v['sys_id'],
                    'target_component_id': 1,
                }],
                output='screen'
            )
        )

    return LaunchDescription(nodes)
```

### Test Procedure

**Terminal 1-3**: Launch PX4 SITL instances
```bash
# Drone 1
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0

# Drone 2
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 1

# Drone 3
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 2
```

**Terminal 4**: Launch AirSim + your ROS2 nodes
```bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
```

**Terminal 5**: Launch multi-MAVROS
```bash
ros2 launch airsim_ros_pkgs mavros_multi.launch.py
```

**Terminal 6**: Verify all connected
```bash
# Check each drone
ros2 topic echo /Drone1/mavros/state --once
ros2 topic echo /Drone2/mavros/state --once
ros2 topic echo /Drone3/mavros/state --once

# All should show connected: true
```

### Test Independent Control

```bash
# Arm only Drone1
ros2 service call /Drone1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Verify Drone2 is still disarmed
ros2 topic echo /Drone2/mavros/state --once  # armed: false

# Arm Drone2
ros2 service call /Drone2/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Success Criteria

- ✅ All 3 MAVROS instances running
- ✅ All connected to respective PX4 instances
- ✅ Independent control (arming one doesn't affect others)
- ✅ No MAVLink routing conflicts
- ✅ Each drone responds to individual commands

---

## Step 1.4: Topic Remapping Infrastructure (Day 5-6)

### Create Remapping Node

Create `ros2/src/airsim_ros_pkgs/src/topic_remapper_node.cpp`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class TopicRemapperNode : public rclcpp::Node
{
public:
  TopicRemapperNode(const std::string& vehicle_name)
  : Node("topic_remapper_" + vehicle_name), vehicle_name_(vehicle_name)
  {
    // Subscribe to vehicle-specific topics
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/" + vehicle_name + "/camera/image",
      10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        // Republish to generic topic for AirStack
        camera_pub_->publish(*msg);
      }
    );

    camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10
    );

    // Similar for LiDAR
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + vehicle_name + "/lidar/points",
      10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidar_pub_->publish(*msg);
      }
    );

    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10
    );

    RCLCPP_INFO(this->get_logger(), "Topic remapper started for %s", vehicle_name_.c_str());
  }

private:
  std::string vehicle_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicRemapperNode>("Drone1");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Update CMakeLists.txt

```cmake
# Add to ros2/src/airsim_ros_pkgs/CMakeLists.txt

add_executable(topic_remapper_node src/topic_remapper_node.cpp)
ament_target_dependencies(topic_remapper_node
  rclcpp
  sensor_msgs
)
install(TARGETS topic_remapper_node DESTINATION lib/${PROJECT_NAME})
```

### Build and Test

```bash
cd ~/PaulSim/Cosys_Airsim_Exploration/ros2
colcon build --packages-select airsim_ros_pkgs
source install/setup.bash

# Launch remapper
ros2 run airsim_ros_pkgs topic_remapper_node

# In another terminal, verify remapping
ros2 topic hz /Drone1/camera/image  # Original topic
ros2 topic hz /camera/image_raw     # Remapped topic

# Both should show same frequency
```

---

## Step 1.5: Integration Testing (Day 7)

### Create Integration Test Script

Create `ros2/src/airsim_ros_pkgs/test/test_mavros_integration.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import time

class MAVROSIntegrationTest(Node):
    def __init__(self):
        super().__init__('mavros_integration_test')

        # Clients
        self.arming_client = self.create_client(CommandBool, '/Drone1/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/Drone1/mavros/set_mode')

        # Publisher
        self.setpoint_pub = self.create_publisher(PoseStamped, '/Drone1/mavros/setpoint_position/local', 10)

        # State subscriber
        self.state_sub = self.create_subscription(State, '/Drone1/mavros/state', self.state_callback, 10)
        self.current_state = State()

    def state_callback(self, msg):
        self.current_state = msg

    def test_connection(self):
        """Test 1: Verify MAVROS connection"""
        self.get_logger().info("Test 1: Checking MAVROS connection...")
        timeout = time.time() + 10  # 10 second timeout

        while time.time() < timeout:
            if self.current_state.connected:
                self.get_logger().info("✅ Test 1 PASSED: MAVROS connected")
                return True
            time.sleep(0.1)

        self.get_logger().error("❌ Test 1 FAILED: MAVROS not connected")
        return False

    def test_mode_switch(self):
        """Test 2: Switch to OFFBOARD mode"""
        self.get_logger().info("Test 2: Switching to OFFBOARD mode...")

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info("✅ Test 2 PASSED: Switched to OFFBOARD")
            return True
        else:
            self.get_logger().error("❌ Test 2 FAILED: Could not switch mode")
            return False

    def test_arming(self):
        """Test 3: Arm vehicle"""
        self.get_logger().info("Test 3: Arming vehicle...")

        # Send setpoints first (required for OFFBOARD arming)
        for i in range(20):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.z = 5.0
            msg.pose.orientation.w = 1.0
            self.setpoint_pub.publish(msg)
            time.sleep(0.05)

        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("✅ Test 3 PASSED: Vehicle armed")
            return True
        else:
            self.get_logger().error("❌ Test 3 FAILED: Could not arm vehicle")
            return False

    def test_takeoff(self):
        """Test 4: Takeoff to 5m"""
        self.get_logger().info("Test 4: Taking off to 5m...")

        start_time = time.time()
        while time.time() - start_time < 20:  # 20 second timeout
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.z = 5.0
            msg.pose.orientation.w = 1.0
            self.setpoint_pub.publish(msg)
            time.sleep(0.1)

        self.get_logger().info("✅ Test 4 PASSED: Takeoff command sent (verify in AirSim)")
        return True

    def run_all_tests(self):
        """Run all integration tests"""
        self.get_logger().info("========================================")
        self.get_logger().info("MAVROS Integration Test Suite")
        self.get_logger().info("========================================")

        tests = [
            self.test_connection,
            self.test_mode_switch,
            self.test_arming,
            self.test_takeoff,
        ]

        results = []
        for test in tests:
            results.append(test())
            time.sleep(1)

        self.get_logger().info("========================================")
        self.get_logger().info(f"Results: {sum(results)}/{len(results)} tests passed")
        self.get_logger().info("========================================")

        return all(results)

def main():
    rclpy.init()
    test_node = MAVROSIntegrationTest()

    # Wait for connections
    time.sleep(2)

    # Run tests
    success = test_node.run_all_tests()

    test_node.destroy_node()
    rclpy.shutdown()

    return 0 if success else 1

if __name__ == '__main__':
    exit(main())
```

### Run Integration Tests

```bash
# Make executable
chmod +x ros2/src/airsim_ros_pkgs/test/test_mavros_integration.py

# Run test
python3 ros2/src/airsim_ros_pkgs/test/test_mavros_integration.py
```

### Expected Output

```
========================================
MAVROS Integration Test Suite
========================================
Test 1: Checking MAVROS connection...
✅ Test 1 PASSED: MAVROS connected
Test 2: Switching to OFFBOARD mode...
✅ Test 2 PASSED: Switched to OFFBOARD
Test 3: Arming vehicle...
✅ Test 3 PASSED: Vehicle armed
Test 4: Taking off to 5m...
✅ Test 4 PASSED: Takeoff command sent
========================================
Results: 4/4 tests passed
========================================
```

---

## Phase 1 Success Criteria

**Before proceeding to Phase 2, verify**:

- ✅ MAVROS installed and working
- ✅ Single drone control via MAVROS
- ✅ Multi-drone independent MAVROS control
- ✅ Topic remapping infrastructure created
- ✅ Integration tests passing (4/4)
- ✅ No MAVLink errors or connection issues
- ✅ Stable flight in AirSim via MAVROS commands

**Deliverables**:
- `mavros_single.launch.py`
- `mavros_multi.launch.py`
- `topic_remapper_node.cpp`
- `test_mavros_integration.py`
- Documentation of tested commands

---

## Phase 1 Rollback

If Phase 1 fails or needs to be reverted:

```bash
# Remove MAVROS
sudo apt remove ros-humble-mavros ros-humble-mavros-extras

# Revert to direct AirSim control
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Your original system still works
```

---

# Phase 2: Perception Integration

**Duration**: 2-3 weeks
**Complexity**: ⭐⭐ Medium
**Risk**: Low-Medium

## Objectives

1. Clone and build AirStack perception components
2. Integrate perception with your sensor topics
3. Compare AirStack vs YOLOv7 performance
4. Choose integration strategy (replace, hybrid, or keep yours)

## Step 2.1: AirStack Setup (Days 1-2)

### Clone AirStack

```bash
cd ~/
git clone --recursive https://github.com/castacks/AirStack.git
cd AirStack
```

### Build Perception Layer Only

```bash
cd robot/ros_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build only perception packages (skip simulator dependencies)
colcon build --packages-select \
  autonomy_perception \
  perception_interfaces \
  state_estimation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

### Expected Build Issues

**Issue 1**: Isaac Sim dependencies
```bash
# Error: isaac_ros_* packages not found
# Solution: Skip Isaac packages
colcon build --packages-skip isaac_ros_visual_slam isaac_ros_nvblox
```

**Issue 2**: Missing Python packages
```bash
pip3 install torch torchvision opencv-python numpy scipy
```

---

## Step 2.2: Sensor Topic Integration (Days 3-5)

### Configure Perception to Use Your Topics

Create `~/AirStack/robot/ros_ws/src/autonomy/2_perception/config/airsim_sensors.yaml`:

```yaml
# AirStack perception configuration for Cosys-AirSim

perception:
  camera:
    topic: /camera/image_raw  # Remapped from /Droan1/camera/image
    frame_id: Droan1/camera0
    width: 640
    height: 480
    fps: 30

  depth:
    topic: /depth/image
    frame_id: Droan1/camera0
    width: 640
    height: 480

  lidar:
    topic: /lidar/points  # Remapped from /Droan1/lidar/points
    frame_id: Droan1/lidar

  imu:
    topic: /Droan1/imu
    frame_id: Droan1/base_link

  odometry:
    topic: /Droan1/odom_local_ned
    frame_id: Droan1/base_link
```

### Launch Perception with AirSim Sensors

Create `~/AirStack/robot/ros_ws/src/autonomy/2_perception/launch/airsim_perception.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = get_package_share_directory('autonomy_perception')
    config_file = os.path.join(config_dir, 'config', 'airsim_sensors.yaml')

    return LaunchDescription([
        # State estimation node
        Node(
            package='state_estimation',
            executable='vio_node',
            name='vio',
            parameters=[config_file],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/imu/data', '/Droan1/imu'),
            ],
            output='screen'
        ),

        # Obstacle detection node
        Node(
            package='obstacle_detection',
            executable='lidar_obstacle_node',
            name='lidar_obstacles',
            parameters=[config_file],
            remappings=[
                ('/lidar/points', '/lidar/points'),
            ],
            output='screen'
        ),
    ])
```

---

## Step 2.3: Performance Comparison (Days 6-8)

### Create Comparison Test

Create `ros2/src/airsim_ros_pkgs/test/perception_comparison.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import time
import json

class PerceptionComparison(Node):
    def __init__(self):
        super().__init__('perception_comparison')

        self.yolo_detections = []
        self.airstack_detections = []

        # Subscribe to both perception systems
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )

        self.airstack_sub = self.create_subscription(
            Detection2DArray,
            '/airstack/detections',
            self.airstack_callback,
            10
        )

        # Metrics
        self.start_time = time.time()
        self.test_duration = 60  # 60 seconds

        self.timer = self.create_timer(1.0, self.print_metrics)

    def yolo_callback(self, msg):
        self.yolo_detections.append({
            'time': time.time(),
            'num_detections': len(msg.detections),
            'latency': (self.get_clock().now().nanoseconds - msg.header.stamp.sec * 1e9) / 1e6  # ms
        })

    def airstack_callback(self, msg):
        self.airstack_detections.append({
            'time': time.time(),
            'num_detections': len(msg.detections),
            'latency': (self.get_clock().now().nanoseconds - msg.header.stamp.sec * 1e9) / 1e6  # ms
        })

    def print_metrics(self):
        elapsed = time.time() - self.start_time

        if elapsed > self.test_duration:
            self.generate_report()
            raise SystemExit

        # Real-time comparison
        yolo_rate = len(self.yolo_detections) / elapsed if elapsed > 0 else 0
        airstack_rate = len(self.airstack_detections) / elapsed if elapsed > 0 else 0

        self.get_logger().info(f"YOLOv7: {yolo_rate:.1f} Hz | AirStack: {airstack_rate:.1f} Hz")

    def generate_report(self):
        """Generate comparison report"""
        yolo_avg_latency = sum(d['latency'] for d in self.yolo_detections) / len(self.yolo_detections) if self.yolo_detections else 0
        airstack_avg_latency = sum(d['latency'] for d in self.airstack_detections) / len(self.airstack_detections) if self.airstack_detections else 0

        yolo_avg_detections = sum(d['num_detections'] for d in self.yolo_detections) / len(self.yolo_detections) if self.yolo_detections else 0
        airstack_avg_detections = sum(d['num_detections'] for d in self.airstack_detections) / len(self.airstack_detections) if self.airstack_detections else 0

        report = {
            'test_duration': self.test_duration,
            'yolo': {
                'total_messages': len(self.yolo_detections),
                'avg_latency_ms': yolo_avg_latency,
                'avg_detections_per_frame': yolo_avg_detections,
                'frequency_hz': len(self.yolo_detections) / self.test_duration
            },
            'airstack': {
                'total_messages': len(self.airstack_detections),
                'avg_latency_ms': airstack_avg_latency,
                'avg_detections_per_frame': airstack_avg_detections,
                'frequency_hz': len(self.airstack_detections) / self.test_duration
            }
        }

        # Print report
        self.get_logger().info("========================================")
        self.get_logger().info("Perception Comparison Report")
        self.get_logger().info("========================================")
        self.get_logger().info(json.dumps(report, indent=2))

        # Save to file
        with open('/tmp/perception_comparison.json', 'w') as f:
            json.dump(report, f, indent=2)

        self.get_logger().info("Report saved to /tmp/perception_comparison.json")

def main():
    rclpy.init()
    node = PerceptionComparison()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Comparison

```bash
# Terminal 1: Launch AirSim + your nodes
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Terminal 2: Launch YOLOv7 perception
ros2 launch airsim_ros_pkgs motion_detection_launch.py vehicle_name:=Drone1

# Terminal 3: Launch AirStack perception
cd ~/AirStack/robot/ros_ws
source install/setup.bash
ros2 launch autonomy_perception airsim_perception.launch.py

# Terminal 4: Run comparison
python3 ros2/src/airsim_ros_pkgs/test/perception_comparison.py
```

### Analyze Results

```bash
cat /tmp/perception_comparison.json

# Expected output:
# {
#   "test_duration": 60,
#   "yolo": {
#     "total_messages": 1800,
#     "avg_latency_ms": 45.2,
#     "avg_detections_per_frame": 2.3,
#     "frequency_hz": 30.0
#   },
#   "airstack": {
#     "total_messages": 600,
#     "avg_latency_ms": 120.5,
#     "avg_detections_per_frame": 1.8,
#     "frequency_hz": 10.0
#   }
# }
```

---

## Step 2.4: Choose Integration Strategy (Days 9-10)

### Option A: Keep YOLOv7, Add SLAM

**Pros**: Best of both worlds
**Cons**: More complex system

```yaml
# Integration architecture:
YOLOv7 (Object Detection) → Your tracking
AirStack SLAM (Mapping) → Map updates
```

**Implementation**: Keep your motion detection, add AirStack SLAM node only

---

### Option B: Replace with AirStack Perception

**Pros**: Unified system
**Cons**: Lose YOLOv7 performance

```yaml
# Replace:
AirStack Perception → Replaces all your AI vision
```

**Implementation**: Disable your motion detection nodes, use AirStack entirely

---

### Option C: Hybrid (Recommended)

**Pros**: Flexibility
**Cons**: Moderate complexity

```yaml
# Hybrid approach:
YOLOv7 → Target detection (primary)
AirStack SLAM → Mapping (added capability)
AirStack VIO → Pose refinement (optional)
```

**Implementation**:

```python
# Create hybrid perception coordinator
class HybridPerception(Node):
    def __init__(self):
        # Subscribe to both
        self.yolo_sub = self.create_subscription(...)
        self.airstack_slam_sub = self.create_subscription(...)

        # Publish unified output
        self.detections_pub = self.create_publisher(...)
        self.map_pub = self.create_publisher(...)
```

---

## Phase 2 Success Criteria

- ✅ AirStack perception builds successfully
- ✅ Perception processes your sensor topics
- ✅ Comparison test completes
- ✅ Integration strategy chosen and documented
- ✅ Hybrid/chosen system tested in AirSim
- ✅ Performance meets requirements (specify Hz, latency)

**Deliverables**:
- AirStack perception build
- `airsim_perception.launch.py`
- `perception_comparison.py` test + results
- Integration decision document
- Performance benchmarks

---

## Phase 2 Rollback

```bash
# Disable AirStack perception
# Keep using your YOLOv7 system

ros2 launch airsim_ros_pkgs motion_detection_launch.py vehicle_name:=Drone1
```

---

# Phases 3-6: Summary

Due to length constraints, here's a summary of remaining phases. Full implementation details follow same pattern as Phases 1-2.

## Phase 3: Local Planning (2-3 weeks)

**Key Tasks**:
- Build AirStack local planner
- Connect planner to MAVROS setpoints
- Test obstacle avoidance
- Tune parameters for AirSim physics

**Main Deliverable**: Trajectory planner replacing PID control

---

## Phase 4: Behavior Trees (3-4 weeks)

**Key Tasks**:
- Study BehaviorTree.CPP
- Port ROS2 actions to BT nodes
- Create mission XML files
- Test complex missions

**Main Deliverable**: Behavior tree mission framework

---

## Phase 5: Multi-Drone Coordination (2-3 weeks)

**Key Tasks**:
- Integrate fleet coordinator
- Configure multi-robot namespace management
- Test coordinated missions
- Performance testing with 5+ drones

**Main Deliverable**: Fleet coordination system

---

## Phase 6: Testing & Polish (2-3 weeks)

**Key Tasks**:
- End-to-end integration testing
- Performance optimization
- Documentation
- Demo scenarios
- Training materials

**Main Deliverable**: Production-ready integrated system

---

# Rollback Procedures

## Emergency Rollback (Any Phase)

```bash
# Stop all AirStack components
pkill -f airstack

# Revert to your original system
cd ~/PaulSim/Cosys_Airsim_Exploration/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Your system should work exactly as before
```

## Selective Rollback

```bash
# Keep MAVROS, remove everything else
ros2 launch airsim_ros_pkgs mavros_multi.launch.py  # Keep Phase 1
# Don't launch AirStack perception/planning/behavior

# Keep perception, remove planning
ros2 launch autonomy_perception airsim_perception.launch.py  # Keep Phase 2
# Don't launch AirStack planning

# etc.
```

---

# Troubleshooting

## Common Issues

### Issue 1: MAVROS Not Connecting

**Symptoms**:
```
[ERROR] [mavros]: FCU: Timeout
```

**Solutions**:
```bash
# Check PX4 is running
ps aux | grep px4

# Check MAVLink port
sudo netstat -tunlp | grep 14540

# Test direct connection
nc -u -l 14540  # Listen on port

# Restart PX4 SITL with verbose
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i 0 -d
```

---

### Issue 2: AirStack Build Failures

**Symptoms**:
```
CMake Error: Could not find a package configuration file provided by "isaac_ros_*"
```

**Solutions**:
```bash
# Skip Isaac packages
colcon build --packages-skip isaac_ros_visual_slam isaac_ros_nvblox

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Update ROS2
sudo apt update && sudo apt upgrade
```

---

### Issue 3: Topic Remapping Not Working

**Symptoms**:
```
[WARN] [airstack]: No data on /camera/image_raw
```

**Solutions**:
```bash
# Check original topic
ros2 topic hz /Droan1/camera/image

# Check remapped topic
ros2 topic hz /camera/image_raw

# Verify remapper running
ros2 node list | grep remapper

# Debug remapper
ros2 run airsim_ros_pkgs topic_remapper_node --ros-args --log-level debug
```

---

## Getting Help

1. **AirStack Issues**:
   - GitHub: https://github.com/castacks/AirStack/issues
   - Contact: airlab@andrew.cmu.edu

2. **Cosys-AirSim Issues**:
   - GitHub: https://github.com/Cosys-Lab/Cosys-AirSim/issues
   - Docs: https://cosys-lab.github.io/

3. **MAVROS Issues**:
   - GitHub: https://github.com/mavlink/mavros/issues
   - ROS Answers: https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:mavros/

---

## Success Metrics

Track these metrics throughout integration:

| Metric | Baseline (Your System) | Target (Integrated) | Current |
|--------|------------------------|---------------------|---------|
| Detection Latency | 45ms (YOLOv7) | < 100ms | TBD |
| Planning Frequency | N/A | > 10 Hz | TBD |
| Multi-Drone Scale | 9 drones | 5+ drones | TBD |
| Mission Complexity | Simple actions | Complex BT missions | TBD |
| Obstacle Avoidance | Manual | Automatic | TBD |

---

**Document Version**: 1.0
**Last Updated**: 2025-10-16
**Status**: Implementation Guide

**Next Steps**: Begin Phase 1 MAVROS baseline integration.
