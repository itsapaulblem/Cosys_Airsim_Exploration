# AirSim Multi-Machine Setup Guide

## Overview

This guide explains how to run AirSim simulation across multiple machines, enabling distributed development, testing, and deployment. You can separate the simulation engine, ROS2 wrapper, and user applications across different computers on the same network.

## Architecture

```
┌─────────────────────┐    TCP/IP      ┌─────────────────────┐    ROS2 DDS    ┌─────────────────────┐
│   Sim Machine       │   Port 41451   │   ROS2 Machine      │   Multi-cast   │   User Machine      │
│   192.168.1.100     │◄──────────────►│   192.168.1.101     │◄──────────────►│   192.168.1.102     │
│                     │                │                     │                │                     │
│ ┌─────────────────┐ │                │ ┌─────────────────┐ │                │ ┌─────────────────┐ │
│ │ Unreal Engine   │ │                │ │ ROS2 Wrapper    │ │                │ │ User Apps       │ │
│ │ + AirSim Plugin │ │                │ │ (Docker)        │ │                │ │ - RViz2         │ │
│ │                 │ │                │ │                 │ │                │ │ - Navigation    │ │
│ │ RPC Server      │ │                │ │ RPC Client ────────┤                │ │ - Custom Apps   │ │
│ │ :41451          │ │                │ │ ROS2 Publishers │ │                │ │ - Mission Plan  │ │
│ └─────────────────┘ │                │ └─────────────────┘ │                │ └─────────────────┘ │
│                     │                │                     │                │                     │
│ Windows/Linux       │                │ Ubuntu + Docker     │                │ Ubuntu/Windows      │
│ GPU Required        │                │ CPU Only            │                │ Lightweight         │
└─────────────────────┘                └─────────────────────┘                └─────────────────────┘
```

## Prerequisites

### **Network Requirements**
- All machines on the same network (LAN/WiFi)
- TCP port `41451` accessible between machines
- UDP multicast support for ROS2 (ports 7400-7500)
- Stable network connection (>100 Mbps recommended for real-time operation)

### **Machine Requirements**

| Machine Type | OS | Specs | Purpose |
|-------------|-----|-------|---------|
| **Sim Machine** | Windows/Linux | GPU, 16GB+ RAM | Runs Unreal Engine + AirSim |
| **ROS2 Machine** | Ubuntu 22.04 | 8GB+ RAM, Docker | Runs ROS2 wrapper |
| **User Machine** | Any | 4GB+ RAM | Development, visualization |

## Step 1: Configure Sim Machine

### **1.1 AirSim Network Configuration**

Edit the AirSim settings file to enable network access:

**Windows:**
```json
// C:\Users\<YourUser>\Documents\AirSim\settings.json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ApiServerEndpoint": "0.0.0.0:41451",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            "PawnBP": "class '/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"
        }
    },
    "CameraDefaults": {
        "CaptureSettings": [
            {
                "ImageType": 0,
                "Width": 640,
                "Height": 480,
                "FOV_Degrees": 90
            }
        ]
    }
}
```

**Linux:**
```json
// ~/Documents/AirSim/settings.json
// Same content as Windows
```

**Key Changes:**
- `"ApiServerEndpoint": "0.0.0.0:41451"` - Binds to all network interfaces instead of localhost only

### **1.2 Firewall Configuration**

**Windows:**
```powershell
# Run as Administrator
New-NetFirewallRule -DisplayName "AirSim RPC Server" -Direction Inbound -Port 41451 -Protocol TCP -Action Allow

# Or use GUI: Windows Defender Firewall → Advanced Settings → Inbound Rules → New Rule
```

**Linux:**
```bash
# UFW (Ubuntu/Debian)
sudo ufw allow 41451/tcp
sudo ufw reload

# iptables (Red Hat/CentOS)
sudo iptables -A INPUT -p tcp --dport 41451 -j ACCEPT
sudo iptables-save
```

### **1.3 Find Your IP Address**

**Windows:**
```cmd
ipconfig
# Look for: IPv4 Address . . . : 192.168.1.100
```

**Linux:**
```bash
hostname -I
# Or: ip addr show | grep 'inet '
```

### **1.4 Test Local Access**

```bash
# Start AirSim (Blocks environment example)
./Blocks.exe -windowed

# Test from another terminal on the same machine
telnet localhost 41451
# Should connect successfully
```

## Step 2: Configure ROS2 Machine

### **2.1 Docker Setup (Recommended)**

**Pull the AirSim ROS2 image:**
```bash
# If using pre-built image
docker pull cosys/airsim-ros2:latest

# Or build from source (see main README)
cd Cosys-AirSim/docker/airsim_ros2_wrapper
docker build -f Dockerfile.simple -t airsim_ros2_simple:latest .
```

**Create environment file:**
```bash
# Create .env file
cat << EOF > airsim_ros2.env
AIRSIM_HOST_IP=192.168.1.100
AIRSIM_HOST_PORT=41451
ROS_DOMAIN_ID=42
LAUNCH_RVIZ=false
EOF
```

**Run the container:**
```bash
docker run -it --rm \
    --name airsim_ros2_container \
    --network host \
    --env-file airsim_ros2.env \
    airsim_ros2_simple:latest
```

### **2.2 Native Installation Setup**

**If running ROS2 natively (without Docker):**

```bash
# Set environment variables
export AIRSIM_HOST_IP=192.168.1.100
export AIRSIM_HOST_PORT=41451
export ROS_DOMAIN_ID=42

# Add to bashrc for persistence
echo "export AIRSIM_HOST_IP=192.168.1.100" >> ~/.bashrc
echo "export AIRSIM_HOST_PORT=41451" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Source ROS2 and build workspace
source /opt/ros/humble/setup.bash
cd ~/airsim_ros2_ws
colcon build --symlink-install

# Launch the wrapper
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
    host_ip:=$AIRSIM_HOST_IP \
    host_port:=$AIRSIM_HOST_PORT \
    enable_api_control:=true \
    output:=screen
```

### **2.3 Custom Launch File**

**Create a remote launch configuration:**

```python
# ~/airsim_ros2_ws/src/airsim_ros_pkgs/launch/remote_airsim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.1.100',
            description='AirSim server IP address'
        ),
        DeclareLaunchArgument(
            'host_port',
            default_value='41451',
            description='AirSim server port'
        ),
        DeclareLaunchArgument(
            'enable_api_control',
            default_value='true',
            description='Enable API control (auto-arm drones)'
        ),
        DeclareLaunchArgument(
            'world_frame_id',
            default_value='world_ned',
            description='World frame ID'
        ),
        
        Node(
            package='airsim_ros_pkgs',
            executable='airsim_node',
            name='airsim_node',
            parameters=[{
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'enable_api_control': LaunchConfiguration('enable_api_control'),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'update_airsim_control_every_n_sec': 0.02,  # 50Hz
                'publish_clock': True,
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
```

**Use the custom launch file:**
```bash
ros2 launch airsim_ros_pkgs remote_airsim.launch.py host_ip:=192.168.1.100
```

### **2.4 Verify Connection**

```bash
# Check if ROS2 wrapper connected to AirSim
ros2 topic list | grep airsim

# Expected output:
# /airsim_node/Drone1/odom_local_ned
# /airsim_node/Drone1/global_gps
# /airsim_node/Drone1/front_center/Scene
# ... (more topics)

# Test data flow
ros2 topic echo /airsim_node/Drone1/odom_local_ned --once
```

## Step 3: Configure User Machines

### **3.1 ROS2 User Setup**

**Install ROS2 and configure domain:**
```bash
# Set same ROS domain ID
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Install ROS2 (if not already installed)
# Ubuntu 22.04:
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rviz2

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Test ROS2 connection:**
```bash
# List available topics (should see AirSim topics)
ros2 topic list

# Monitor drone state
ros2 topic echo /airsim_node/Drone1/odom_local_ned

# Send velocity commands
ros2 topic pub /airsim_node/Drone1/vel_cmd_body_frame airsim_interfaces/msg/VelCmd "
twist:
  linear: {x: 1.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
" --once
```

### **3.2 Visualization with RViz2**

```bash
# Launch RViz2
rviz2

# Or launch with pre-configured setup
ros2 launch airsim_ros_pkgs rviz.launch.py
```

**Configure RViz2 for AirSim:**
1. **Add Display** → **By Topic** → Select topics:
   - `/airsim_node/Drone1/odom_local_ned` (Odometry)
   - `/airsim_node/Drone1/front_center/Scene` (Image)
   - `/airsim_node/Drone1/lidar/points` (PointCloud2)

2. **Set Fixed Frame**: `world_ned` or `world`

3. **Save Configuration** for future use

### **3.3 Direct Python/C++ Connection**

**Python example (bypasses ROS2):**
```python
#!/usr/bin/env python3
import airsim
import time

# Connect directly to remote AirSim
client = airsim.MultirotorClient(ip="192.168.1.100", port=41451)

try:
    # Test connection
    client.confirmConnection()
    print("Connected to AirSim successfully!")
    
    # Enable API control
    client.enableApiControl(True)
    client.armDisarm(True)
    
    # Simple flight
    print("Taking off...")
    client.takeoffAsync().join()
    
    print("Flying forward...")
    client.moveByVelocityAsync(2.0, 0.0, 0.0, 3.0).join()
    
    print("Landing...")
    client.landAsync().join()
    
except Exception as e:
    print(f"Error: {e}")
    
finally:
    client.reset()
```

**C++ example:**
```cpp
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>

int main() {
    // Connect to remote AirSim
    msr::airlib::MultirotorRpcLibClient client("192.168.1.100", 41451);
    
    try {
        client.confirmConnection();
        std::cout << "Connected to AirSim!" << std::endl;
        
        client.enableApiControl(true);
        client.armDisarm(true);
        
        // Simple flight
        client.takeoffAsync()->waitOnLastTask();
        client.moveByVelocityAsync(2.0f, 0.0f, 0.0f, 3.0f)->waitOnLastTask();
        client.landAsync()->waitOnLastTask();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    return 0;
}
```

## Step 4: Advanced Network Configuration

### **4.1 ROS2 Multi-Machine Discovery**

**Configure ROS2 for complex networks:**

```bash
# Method 1: Simple domain configuration (most cases)
export ROS_DOMAIN_ID=42

# Method 2: Explicit interface selection
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI="<cyclonedx><domain><general><interfaces><networkInterface name=\"eth0\"/></interfaces></general></domain></cyclonedx>"

# Method 3: Disable multicast (use unicast only)
export CYCLONEDX_URI="<cyclonedx><domain><general><allowMulticast>false</allowMulticast></general></domain></cyclonedx>"
```

**For Docker containers:**
```bash
# Use host networking (simplest)
docker run --network host ...

# Or bridge with port mapping
docker run -p 7400-7500:7400-7500/udp ...
```

### **4.2 SSH Tunneling (Secure Remote Access)**

**For accessing AirSim over the internet:**
```bash
# Create SSH tunnel from user machine to sim machine
ssh -L 41451:localhost:41451 user@sim-machine-public-ip

# Now connect to localhost:41451 instead of remote IP
client = airsim.MultirotorClient(ip="localhost", port=41451)
```

### **4.3 VPN Setup**

**For distributed teams:**
```bash
# Example with WireGuard VPN
# All machines connect to VPN network: 10.0.0.0/24
# Sim Machine: 10.0.0.100
# ROS2 Machine: 10.0.0.101
# User Machine: 10.0.0.102

# Update configurations to use VPN IPs
export AIRSIM_HOST_IP=10.0.0.100
```

## Step 5: Testing and Validation

### **5.1 Network Connectivity Tests**

```bash
# Test 1: Basic TCP connectivity
nc -z 192.168.1.100 41451
echo $?  # Should return 0 (success)

# Test 2: RPC functionality
python3 -c "
import airsim
client = airsim.MultirotorClient('192.168.1.100', 41451)
client.confirmConnection()
print('AirSim RPC connection successful!')
"

# Test 3: ROS2 topic visibility
ros2 topic list | grep airsim | wc -l
# Should return > 0 (number of AirSim topics)

# Test 4: Data flow
timeout 5 ros2 topic echo /airsim_node/Drone1/odom_local_ned --once
```

### **5.2 Performance Benchmarks**

```bash
# Test ROS2 topic frequency
ros2 topic hz /airsim_node/Drone1/odom_local_ned
# Expected: ~50 Hz

# Test image data rate
ros2 topic hz /airsim_node/Drone1/front_center/Scene
# Expected: ~30 Hz

# Test network latency
ping 192.168.1.100
# Expected: <10ms on LAN
```

### **5.3 End-to-End Test**

**Complete system test script:**
```python
#!/usr/bin/env python3
import airsim
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class MultiMachineTest(Node):
    def __init__(self):
        super().__init__('multi_machine_test')
        
        # ROS2 connection
        self.odom_sub = self.create_subscription(
            Odometry, '/airsim_node/Drone1/odom_local_ned', 
            self.odom_callback, 10)
        
        # Direct RPC connection
        self.airsim_client = airsim.MultirotorClient(ip="192.168.1.100", port=41451)
        self.airsim_client.confirmConnection()
        
        self.odom_received = False
        
    def odom_callback(self, msg):
        self.get_logger().info(f"ROS2 Odometry: x={msg.pose.pose.position.x:.2f}")
        self.odom_received = True
    
    def test_direct_rpc(self):
        state = self.airsim_client.getMultirotorState()
        self.get_logger().info(f"Direct RPC: x={state.kinematics_estimated.position.x_val:.2f}")
    
    def run_test(self):
        self.get_logger().info("Starting multi-machine test...")
        
        for i in range(10):
            # Test ROS2 connection
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # Test direct RPC
            self.test_direct_rpc()
            
            time.sleep(1)
        
        if self.odom_received:
            self.get_logger().info("✅ Multi-machine setup working correctly!")
        else:
            self.get_logger().error("❌ ROS2 connection failed")

if __name__ == '__main__':
    rclpy.init()
    test_node = MultiMachineTest()
    test_node.run_test()
    rclpy.shutdown()
```

## Troubleshooting

### **Common Issues**

#### **1. "Connection refused" Error**
```bash
# Symptoms: Cannot connect to AirSim
telnet 192.168.1.100 41451
# Connection refused

# Solutions:
# 1. Check AirSim is running
# 2. Verify settings.json has "ApiServerEndpoint": "0.0.0.0:41451"
# 3. Check firewall rules
# 4. Verify IP address
```

#### **2. ROS2 Topics Not Visible**
```bash
# Symptoms: ros2 topic list shows no AirSim topics

# Debug steps:
# 1. Check ROS_DOMAIN_ID is same on all machines
env | grep ROS_DOMAIN_ID

# 2. Test ROS2 discovery
ros2 node list

# 3. Check network interfaces
ip route show

# 4. Restart ROS2 wrapper with verbose output
ros2 launch airsim_ros_pkgs airsim_node.launch.py --ros-args --log-level debug
```

#### **3. Slow Performance**
```bash
# Symptoms: High latency, dropped frames

# Optimizations:
# 1. Use wired connection instead of WiFi
# 2. Reduce image resolution in settings.json
# 3. Lower update frequencies
# 4. Use dedicated network interface
```

#### **4. Docker Network Issues**
```bash
# Symptoms: Docker container can't reach AirSim

# Solutions:
# 1. Use host networking
docker run --network host ...

# 2. Check container IP
docker exec -it container_name ip addr

# 3. Test from inside container
docker exec -it container_name nc -z 192.168.1.100 41451
```

### **Debug Commands**

```bash
# Network debugging
netstat -tuln | grep 41451  # Check if port is listening
ss -tuln | grep 41451       # Alternative to netstat
lsof -i :41451              # Check which process uses port

# ROS2 debugging
ros2 daemon stop && ros2 daemon start  # Restart ROS2 discovery
ros2 doctor                             # Check ROS2 setup
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # Try different RMW

# Docker debugging
docker logs container_name              # Check container logs
docker exec -it container_name bash    # Enter container
```

## Production Deployment

### **Scaling Considerations**

```yaml
# Docker Compose example for production
version: '3.8'
services:
  airsim-ros2:
    image: airsim_ros2_simple:latest
    environment:
      - AIRSIM_HOST_IP=192.168.1.100
      - ROS_DOMAIN_ID=42
    network_mode: host
    restart: unless-stopped
    
  monitoring:
    image: prom/prometheus
    ports:
      - "9090:9090"
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
```

### **Monitoring Setup**

```bash
# Install ROS2 monitoring tools
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# Monitor system performance
rqt_graph  # Visualize ROS2 graph
rqt_topic  # Monitor topic rates
htop       # System resources
```

## Summary

This multi-machine setup enables:

- **🎮 Simulation Machine**: High-performance graphics and physics
- **🤖 ROS2 Machine**: Real-time robotics middleware 
- **💻 User Machines**: Development, visualization, control
- **🌐 Network Distribution**: Flexible deployment options
- **🔒 Security**: VPN and SSH tunnel support

The key to success is proper network configuration and ensuring all machines can communicate on the required ports. Once set up, the system provides a powerful, distributed simulation environment for developing and testing autonomous systems.

### **Quick Start Checklist**

- [ ] Configure AirSim `settings.json` with `"ApiServerEndpoint": "0.0.0.0:41451"`
- [ ] Open firewall port 41451 on Sim Machine
- [ ] Set `AIRSIM_HOST_IP` environment variable on ROS2 Machine
- [ ] Set matching `ROS_DOMAIN_ID` on all ROS2 machines
- [ ] Test basic connectivity with `nc -z <sim_ip> 41451`
- [ ] Verify ROS2 topics are visible with `ros2 topic list`
- [ ] Run end-to-end test with sample application

Happy flying! 🚁 