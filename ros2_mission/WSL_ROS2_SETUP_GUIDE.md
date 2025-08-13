# Complete ROS2 Setup Guide for WSL2

## Overview

This guide provides step-by-step instructions for setting up ROS2 Humble on Windows Subsystem for Linux 2 (WSL2) for AirSim integration. This setup enables seamless development with AirSim simulation running on Windows and ROS2 nodes running in a Linux environment.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [WSL2 Installation and Setup](#wsl2-installation-and-setup)
3. [ROS2 Humble Installation](#ros2-humble-installation)
4. [AirSim ROS2 Package Setup](#airsim-ros2-package-setup)
5. [Network Configuration](#network-configuration)
6. [Testing the Setup](#testing-the-setup)
7. [RViz2 Configuration](#rviz2-configuration)
8. [Troubleshooting](#troubleshooting)
9. [Advanced Configuration](#advanced-configuration)

## Prerequisites

### System Requirements
- **OS**: Windows 10 version 2004+ or Windows 11
- **CPU**: x64 processor with virtualization support
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space for WSL2 and ROS2

### Required Software
- Windows Terminal (recommended)
- WSL2 enabled
- Hyper-V enabled (automatic with WSL2)

## WSL2 Installation and Setup

### Step 1: Enable WSL2

**Option A: Using Windows Features**
1. Open **Turn Windows features on or off**
2. Enable:
   - ✅ **Windows Subsystem for Linux**
   - ✅ **Virtual Machine Platform**
3. Restart your computer

**Option B: Using PowerShell (Administrator)**
```powershell
# Enable WSL and Virtual Machine Platform
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Restart computer
shutdown /r /t 0
```

### Step 2: Install WSL2 Kernel Update
1. Download from: https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi
2. Run installer as Administrator
3. Set WSL2 as default:
```powershell
wsl --set-default-version 2
```

### Step 3: Install Ubuntu 22.04 LTS

**Option A: Microsoft Store**
1. Open Microsoft Store
2. Search for **Ubuntu 22.04.3 LTS**
3. Click **Install**

**Option B: Command Line**
```powershell
wsl --install -d Ubuntu-22.04
```

### Step 4: Initial Ubuntu Setup
1. Launch Ubuntu from Start Menu
2. Create user account:
```bash
# Follow prompts to create username and password
# Example: username 'airsim', password of your choice
```

3. Update system:
```bash
sudo apt update && sudo apt upgrade -y
```

### Step 5: WSL2 Optimization

**Increase WSL2 Memory Limit**
Create `%USERPROFILE%\.wslconfig`:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
localhostForwarding=true
```

**Enable systemd (Ubuntu 22.04+)**
```bash
echo -e '[boot]\nsystemd=true' | sudo tee -a /etc/wsl.conf
```

**Restart WSL**:
```powershell
wsl --shutdown
wsl
```

## ROS2 Humble Installation

### Step 1: Setup ROS2 Repository
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update
```

### Step 2: Install ROS2 Humble Desktop
```bash
# Full desktop installation (recommended)
sudo apt install ros-humble-desktop-full -y

# Alternative: Base installation (minimal)
# sudo apt install ros-humble-ros-base -y
```

### Step 3: Install Development Tools
```bash
# Essential development tools
sudo apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-vcstool

# Additional useful packages
sudo apt install -y \
    git \
    wget \
    curl \
    vim \
    htop \
    tree \
    unzip
```

### Step 4: Setup ROS2 Environment
```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Source for current session
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

### Step 5: Install Additional ROS2 Packages
```bash
# Image transport and vision packages
sudo apt install -y \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-geometry

# RViz2 and visualization
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins

# Sensor and message packages
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-tools

# Build tools
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    python3-rosdep
```

### Step 6: Initialize rosdep
```bash
# Initialize rosdep (run once)
sudo rosdep init
rosdep update
```

## AirSim ROS2 Package Setup

### Step 1: Create ROS2 Workspace
```bash
# Create workspace
mkdir -p ~/airsim_ws/src
cd ~/airsim_ws
```

### Step 2: Clone AirSim ROS2 Packages
```bash
# Clone from your Cosys-AirSim repository
cd src
git clone /mnt/l/Cosys-AirSim/ros2/src/airsim_interfaces
git clone /mnt/l/Cosys-AirSim/ros2/src/airsim_ros_pkgs

# Alternative: Copy files directly
# cp -r /mnt/l/Cosys-AirSim/ros2/src/* ~/airsim_ws/src/
```

### Step 3: Install Dependencies
```bash
cd ~/airsim_ws

# Install dependencies for workspace packages
rosdep install --from-paths src --ignore-src -r -y

# Install Python packages for AirSim
pip3 install airsim msgpack-rpc-python
```

### Step 4: Build Workspace
```bash
cd ~/airsim_ws

# Build all packages
colcon build

# Alternative: Build specific packages
# colcon build --packages-select airsim_interfaces
# colcon build --packages-select airsim_ros_pkgs

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/airsim_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Verify Package Installation
```bash
# Check available packages
ros2 pkg list | grep airsim

# Check available services
ros2 interface list | grep airsim

# Check available messages
ros2 interface list | grep airsim_interfaces
```

## Network Configuration

### Step 1: Configure AirSim Network Access

**Find WSL2 IP Address**:
```bash
# Get WSL2 IP
ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1

# Get Windows host IP from WSL
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
```

**Test AirSim Connection**:
```bash
# Test connection to AirSim (running on Windows)
# Default AirSim port is 41451
curl -v telnet://$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):41451

# Alternative test with nc (netcat)
nc -zv $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}') 41451
```

### Step 2: Configure Firewall (Windows)

**Windows Defender Firewall**:
1. Open **Windows Security** → **Firewall & network protection**
2. Click **Allow an app through firewall**
3. Add **AirSim/Unreal Engine** executable
4. Allow for both **Private** and **Public** networks

**PowerShell (Administrator)**:
```powershell
# Allow AirSim port
New-NetFirewallRule -DisplayName "AirSim ROS2" -Direction Inbound -Protocol TCP -LocalPort 41451 -Action Allow

# Allow WSL2 subnet (adjust if needed)
New-NetFirewallRule -DisplayName "WSL2 Subnet" -Direction Inbound -RemoteAddress 172.16.0.0/12 -Action Allow
```

### Step 3: Test Network Connectivity
```bash
# Create test script
cat > ~/test_airsim_connection.py << 'EOF'
#!/usr/bin/env python3
import airsim
import sys

def test_connection():
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Test basic API
        state = client.getMultirotorState()
        print(f"✅ AirSim connection successful!")
        print(f"   Drone position: {state.kinematics_estimated.position}")
        return True
        
    except Exception as e:
        print(f"❌ AirSim connection failed: {e}")
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
EOF

chmod +x ~/test_airsim_connection.py
```

## Testing the Setup

### Step 1: Start AirSim (Windows)
1. Launch your Unreal Engine project with AirSim
2. Wait for **"Waiting for connection on port 41451"** message

### Step 2: Test ROS2 Packages (WSL2)
```bash
# Test 1: Basic AirSim connection
cd ~
python3 test_airsim_connection.py

# Test 2: Launch AirSim ROS2 node
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# Test 3: Check published topics (in new terminal)
ros2 topic list | grep airsim

# Test 4: View camera data
ros2 topic echo /airsim_node/drone_1/front_center_custom/Scene --once
```

### Step 3: Verify Camera Integration
```bash
# List camera topics
ros2 topic list | grep -E "(Scene|DepthPlanar|Segmentation)"

# View image info
ros2 topic info /airsim_node/drone_1/front_center_custom/Scene

# Save a test image
ros2 topic echo /airsim_node/drone_1/front_center_custom/Scene --once > test_image.yaml
```

## RViz2 Configuration

### Step 1: Install GUI Support for WSL2

**Option A: Using VcXsrv (Recommended)**
1. Download VcXsrv: https://sourceforge.net/projects/vcxsrv/
2. Install on Windows
3. Launch XLaunch with settings:
   - ✅ Multiple windows
   - ✅ Start no client
   - ✅ Disable access control

**Configure WSL2**:
```bash
# Add to ~/.bashrc
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc

# Source for current session
source ~/.bashrc
```

**Option B: Using WSLg (Windows 11)**
```bash
# WSLg is built-in for Windows 11
# Test with simple GUI app
sudo apt install x11-apps -y
xclock
```

### Step 2: Launch RViz2
```bash
# Test RViz2 launch
rviz2

# Launch with AirSim configuration
cd ~/airsim_ws
ros2 launch airsim_ros_pkgs rviz.launch.py

# Alternative: Use generated camera config
rviz2 -d /mnt/l/Cosys-AirSim/docker_clean/multi_drone/airsim_cameras.rviz
```

### Step 3: Configure Camera Displays
1. **Add Image Display**:
   - Click **Add** → **Image**
   - Set **Topic**: `/airsim_node/drone_1/front_center_custom/Scene`
   - Set **Transport Hint**: `raw`

2. **Add Multiple Cameras**:
   - Repeat for different image types:
     - `/airsim_node/drone_1/front_center_custom/DepthPlanar`
     - `/airsim_node/drone_1/front_center_custom/Segmentation`

3. **Save Configuration**:
   - **File** → **Save Config As** → `airsim_custom.rviz`

## Troubleshooting

### Common Issues and Solutions

#### 1. "ModuleNotFoundError: No module named 'airsim'"
```bash
# Solution: Install AirSim Python package
pip3 install airsim

# Alternative: Install from source
cd /mnt/l/Cosys-AirSim/PythonClient
pip3 install -e .
```

#### 2. "Connection refused" to AirSim
```bash
# Check AirSim is running on Windows
# Test network connectivity
ping $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')

# Check if port is accessible
telnet $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}') 41451

# Windows Firewall may be blocking - add exception
```

#### 3. RViz2 displays blank/no GUI
```bash
# Check DISPLAY variable
echo $DISPLAY

# Test X11 forwarding
xclock

# If using VcXsrv, ensure it's running and configured properly
# If using WSLg, try restarting WSL:
# (In Windows PowerShell): wsl --shutdown && wsl
```

#### 4. "No topics available" in ROS2
```bash
# Check if airsim_node is running
ros2 node list

# Check for error messages
ros2 launch airsim_ros_pkgs airsim_node.launch.py --debug

# Verify AirSim connection works outside ROS2
python3 test_airsim_connection.py
```

#### 5. Build errors in colcon
```bash
# Clean and rebuild
cd ~/airsim_ws
rm -rf build install log
colcon build --symlink-install

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check for missing system packages
sudo apt update && sudo apt upgrade -y
```

#### 6. Permission denied accessing /mnt/l/
```bash
# Mount with proper permissions
sudo umount /mnt/l 2>/dev/null || true
sudo mkdir -p /mnt/l
sudo mount -t drvfs L: /mnt/l -o uid=1000,gid=1000,metadata

# Make permanent by adding to /etc/fstab
echo "L: /mnt/l drvfs uid=1000,gid=1000,metadata 0 0" | sudo tee -a /etc/fstab
```

### Performance Optimization

#### WSL2 Memory and CPU
```bash
# Monitor resource usage
htop

# Windows .wslconfig optimization:
# [wsl2]
# memory=12GB
# processors=6
# swap=4GB
# kernelCommandLine=vsyscall=emulate
```

#### ROS2 Performance
```bash
# Use Fast-DDS instead of CycloneDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Optimize DDS discovery
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_profile.xml
```

#### Network Performance
```bash
# Increase network buffers
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## Advanced Configuration

### Custom Launch Files
Create `~/airsim_ws/src/custom_launch.py`:
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airsim_ros_pkgs',
            executable='airsim_node',
            name='airsim_node',
            parameters=[{
                'odom_frame_id': 'odom_ned',
                'coordinate_frame': 'NEU',
                'update_airsim_img_response_every_n_sec': 0.1,
                'publish_clock': False
            }],
            output='screen'
        )
    ])
```

### Environment Automation
Create `~/setup_airsim_ros2.sh`:
```bash
#!/bin/bash
# AirSim + ROS2 Environment Setup Script

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up AirSim + ROS2 Environment...${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/airsim_ws/install/setup.bash

# Set environment variables
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42

# Test AirSim connection
echo -e "${YELLOW}Testing AirSim connection...${NC}"
if python3 ~/test_airsim_connection.py; then
    echo -e "${GREEN}✅ AirSim connection successful${NC}"
    
    # Launch AirSim ROS2 node
    echo -e "${YELLOW}Launching AirSim ROS2 node...${NC}"
    ros2 launch airsim_ros_pkgs airsim_node.launch.py &
    
    # Wait for node to initialize
    sleep 5
    
    # Show available topics
    echo -e "${GREEN}Available topics:${NC}"
    ros2 topic list | grep airsim
    
    echo -e "${GREEN}Setup complete! Use 'fg' to bring airsim_node to foreground${NC}"
else
    echo -e "${RED}❌ AirSim connection failed. Make sure AirSim is running.${NC}"
    exit 1
fi
```

Make executable:
```bash
chmod +x ~/setup_airsim_ros2.sh
```

### Automated Startup (Optional)
Add to `~/.bashrc`:
```bash
# Auto-setup function
setup_airsim() {
    source /opt/ros/humble/setup.bash
    source ~/airsim_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=42
    export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
    echo "AirSim ROS2 environment configured!"
}

# Optional: Auto-run on shell start
# setup_airsim
```

## Quick Start Summary

After completing this guide, your typical workflow will be:

### Windows Side:
1. Start VcXsrv (if using)
2. Launch AirSim/Unreal Engine
3. Wait for "Waiting for connection" message

### WSL2 Side:
```bash
# Setup environment
setup_airsim

# Launch AirSim ROS2 integration
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# In another terminal: Launch RViz2
rviz2 -d ~/airsim_ws/airsim_cameras.rviz

# View camera topics
ros2 topic list | grep Scene
```

## Integration with Docker Clean Workflow

This WSL2 setup integrates with the Docker Clean workflow:

```bash
# Generate camera configuration
cd /mnt/l/Cosys-AirSim/docker_clean/config_generator/tools
python3 unified_generator.py multi --num-drones 2 --cameras front_rgb front_depth --create-rviz

# Use generated RViz config
rviz2 -d /mnt/l/Cosys-AirSim/docker_clean/multi_drone/airsim_cameras.rviz

# Start Docker containers (from Windows)
cd /mnt/l/Cosys-AirSim/docker_clean/multi_drone
./launch.bat

# Connect ROS2 (from WSL2)
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

This setup provides a complete development environment with AirSim simulation on Windows and ROS2 development on WSL2, enabling the best of both worlds for robotics development.