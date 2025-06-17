#!/bin/bash

echo "=== Setting up ROS2 Humble in WSL2 for AirSim ==="

# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions

# Add ROS2 repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional ROS2 packages for AirSim
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-geographic-msgs \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create workspace for AirSim ROS2
mkdir -p ~/airsim_ros2_ws/src
cd ~/airsim_ros2_ws/src

# Clone AirSim ROS2 wrapper (assuming it's in your Cosys-AirSim repo)
echo "Please copy the ros2 folder from your Cosys-AirSim directory to ~/airsim_ros2_ws/src/"
echo "You can do this from Windows with:"
echo "cp -r /mnt/c/path/to/Cosys-AirSim/ros2/* ~/airsim_ros2_ws/src/"

echo "=== Setup complete! ==="
echo "Next steps:"
echo "1. Copy AirSim ROS2 source code to ~/airsim_ros2_ws/src/"
echo "2. Build with: cd ~/airsim_ros2_ws && colcon build"
echo "3. Source with: source ~/airsim_ros2_ws/install/setup.bash"
echo "4. Run with: ros2 launch airsim_ros_pkgs airsim_node.launch.py" 