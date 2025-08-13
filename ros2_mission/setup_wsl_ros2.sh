#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_step() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check if running in WSL
if ! grep -qi microsoft /proc/version; then
    print_error "This script is designed for WSL2. Please run in Windows Subsystem for Linux."
    exit 1
fi

print_step "Setting up ROS2 Humble in WSL2 for AirSim"

# Update system
print_step "Updating system packages"
sudo apt update && sudo apt upgrade -y
print_success "System updated"

# Set locale
print_step "Setting up locale"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
print_success "Locale configured"

# Install required packages
print_step "Installing essential packages"
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-flake8-docstrings \
    python3-pytest-cov \
    python3-setuptools \
    python3-vcstool \
    ros-dev-tools \
    wget \
    vim \
    htop \
    tree \
    unzip
print_success "Essential packages installed"

# Add ROS2 repository
print_step "Adding ROS2 repository"
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
print_success "ROS2 repository added"

# Install ROS2 Humble Desktop
print_step "Installing ROS2 Humble Desktop (this may take a while...)"
sudo apt install -y ros-humble-desktop-full
print_success "ROS2 Humble Desktop installed"

# Install additional ROS2 packages for AirSim
print_step "Installing AirSim-specific ROS2 packages"
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv \
    ros-humble-image-geometry \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-geographic-msgs \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-tools \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    python3-rosdep
print_success "AirSim ROS2 packages installed"

# Initialize rosdep
print_step "Initializing rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
print_success "rosdep initialized"

# Setup ROS2 environment
print_step "Configuring ROS2 environment"
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Add useful environment variables
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

if ! grep -q "RMW_IMPLEMENTATION" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
fi

# Setup X11 forwarding for GUI apps
if ! grep -q "DISPLAY.*nameserver" ~/.bashrc; then
    echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '"'"'{print $2}'"'"'):0.0' >> ~/.bashrc
    echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
fi

source ~/.bashrc
print_success "ROS2 environment configured"

# Install Python packages for AirSim
print_step "Installing AirSim Python packages"
pip3 install airsim msgpack-rpc-python numpy opencv-python
print_success "AirSim Python packages installed"

# Create workspace for AirSim ROS2
print_step "Creating AirSim ROS2 workspace"
mkdir -p ~/airsim_ws/src
cd ~/airsim_ws

# Auto-copy AirSim ROS2 source if available
COSYS_PATH="/mnt/l/Cosys-AirSim/ros2/src"
if [ -d "$COSYS_PATH" ]; then
    print_step "Copying AirSim ROS2 source from Cosys-AirSim"
    cp -r "$COSYS_PATH"/* ~/airsim_ws/src/
    print_success "AirSim ROS2 source copied"
    
    # Build the workspace
    print_step "Building AirSim ROS2 workspace"
    cd ~/airsim_ws
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    
    if [ $? -eq 0 ]; then
        print_success "Workspace built successfully"
        
        # Add workspace to bashrc
        if ! grep -q "source ~/airsim_ws/install/setup.bash" ~/.bashrc; then
            echo "source ~/airsim_ws/install/setup.bash" >> ~/.bashrc
        fi
        
        # Source for current session
        source install/setup.bash
        
    else
        print_error "Workspace build failed"
    fi
else
    print_warning "Cosys-AirSim source not found at $COSYS_PATH"
    echo "You'll need to manually copy the ROS2 source code to ~/airsim_ws/src/"
fi

# Create test script
print_step "Creating AirSim connection test script"
cat > ~/test_airsim_connection.py << 'EOF'
#!/usr/bin/env python3
import airsim
import sys

def test_connection():
    try:
        # Connect to AirSim (adjust IP if needed)
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Test basic API
        state = client.getMultirotorState()
        print(f"✅ AirSim connection successful!")
        print(f"   Drone position: {state.kinematics_estimated.position}")
        print(f"   Connection IP: {client.client_ip}")
        return True
        
    except Exception as e:
        print(f"❌ AirSim connection failed: {e}")
        print("   Make sure AirSim is running on Windows and firewall allows connections")
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
EOF

chmod +x ~/test_airsim_connection.py
print_success "Test script created"

# Create environment setup script
print_step "Creating environment setup script"
cat > ~/setup_airsim_ros2.sh << 'EOF'
#!/bin/bash
# AirSim + ROS2 Environment Setup Script

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Setting up AirSim + ROS2 Environment...${NC}"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
if [ -f ~/airsim_ws/install/setup.bash ]; then
    source ~/airsim_ws/install/setup.bash
fi

# Set environment variables
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0

echo -e "${GREEN}Environment configured!${NC}"

# Test AirSim connection
echo -e "${YELLOW}Testing AirSim connection...${NC}"
if python3 ~/test_airsim_connection.py; then
    echo -e "${GREEN}Ready to launch AirSim ROS2 integration!${NC}"
    echo -e "${YELLOW}Commands:${NC}"
    echo "  ros2 launch airsim_ros_pkgs airsim_node.launch.py"
    echo "  rviz2"
    echo "  ros2 topic list | grep airsim"
else
    echo -e "${RED}AirSim connection failed. Make sure:${NC}"
    echo "  1. AirSim is running on Windows"
    echo "  2. Windows Firewall allows connections"
    echo "  3. No antivirus blocking the connection"
fi
EOF

chmod +x ~/setup_airsim_ros2.sh
print_success "Environment setup script created"

# Install GUI support packages
print_step "Installing GUI support for RViz2"
sudo apt install -y x11-apps mesa-utils
print_success "GUI support installed"

print_step "Installation Complete!"
echo ""
print_success "ROS2 Humble with AirSim integration is now installed!"
echo ""
echo -e "${BLUE}Quick Start:${NC}"
echo "1. ${YELLOW}Start VcXsrv on Windows${NC} (for GUI apps like RViz2)"
echo "2. ${YELLOW}Launch AirSim/Unreal on Windows${NC}"
echo "3. ${YELLOW}In WSL2:${NC} Run ~/setup_airsim_ros2.sh"
echo "4. ${YELLOW}Launch ROS2:${NC} ros2 launch airsim_ros_pkgs airsim_node.launch.py"
echo "5. ${YELLOW}Launch RViz2:${NC} rviz2"
echo ""
echo -e "${BLUE}Useful Files Created:${NC}"
echo "  ~/test_airsim_connection.py - Test AirSim connectivity"
echo "  ~/setup_airsim_ros2.sh - Quick environment setup"
echo "  ~/airsim_ws/ - ROS2 workspace with AirSim packages"
echo ""
echo -e "${BLUE}Integration with Docker Clean:${NC}"
echo "  Use camera configs: /mnt/l/Cosys-AirSim/docker_clean/*/airsim_cameras.rviz"
echo "  Generate configs: cd /mnt/l/Cosys-AirSim/docker_clean/config_generator/tools"
echo ""
print_warning "Restart your terminal or run 'source ~/.bashrc' to apply environment changes" 