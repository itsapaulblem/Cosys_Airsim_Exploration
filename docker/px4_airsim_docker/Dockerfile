# Use a base image with Ubuntu 22.04, suitable for ROS 2 Humble
FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    curl \
    lsb-release \
    python3-pip \
    python3-venv \
    sudo \
    unzip \
    cmake \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble base and development tools
# Based on https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    # Add ROS 2 repository to sources list
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    # Update apt-get and install ROS 2 packages
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS 2 setup script for subsequent commands in Dockerfile
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENV PATH="/opt/ros/humble/bin:${PATH}"
ENV CMAKE_PREFIX_PATH="/opt/ros/humble:${CMAKE_PREFIX_PATH}"

# Create and set up a ROS 2 workspace for building PX4 and micro-ROS related packages
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Clone micro-ROS agent source with pinned version for stability
RUN git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_agent.git

# Clone PX4-Autopilot repository and its submodules - pin to stable release
RUN git clone --depth 1 --recursive -b v1.14.3 https://github.com/PX4/PX4-Autopilot.git

# Clone PX4 ROS2 communication packages
RUN git clone --depth 1 -b release/1.14.x https://github.com/PX4/px4_msgs.git

RUN git clone --depth 1 -b release/1.14.x https://github.com/PX4/px4_ros_com.git

# Install PX4 dependencies.
# We use the PX4's own setup script but exclude unnecessary simulation tools
# (like Gazebo, as we are using AirSim) and toolchains for efficiency.
# This should be run from within the PX4-Autopilot directory.
WORKDIR /ros2_ws/src/PX4-Autopilot
RUN ./Tools/setup/ubuntu.sh --no-sim-tools --no-ccache --no-mavlink-gitsubmodules

# Build PX4 with ROS 2 support
# This step compiles the PX4 firmware for SITL (Software In The Loop)
# The 'none_iris' target is a generic multicopter model without a specific integrated simulator.
# It also builds the micro-XRCE-DDS client into the firmware for ROS 2 communication.
# RUN make px4_sitl_default none_iris

# Return to the ROS 2 workspace root to build all cloned packages
WORKDIR /ros2_ws

# Source ROS 2 and build the workspace (micro_ros_agent, px4_msgs, px4_ros_com)
# Ensure to source the installed ROS 2 before building the workspace
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the ROS 2 workspace overlay for runtime use
# This should be done for all subsequent commands that need access to the built packages
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO=humble 
# Explicitly set ROS_DISTRO

# Set PX4 environment variables
ENV PX4_HOME_LAT=47.641468
ENV PX4_HOME_LON=-122.140165
ENV PX4_HOME_ALT=0.0

# Create a simple startup script for the container
WORKDIR /
# Copy the script from host to container
COPY start_px4_airsim_gcs.sh /start_px4_airsim_gcs.sh
# Make the script executable
RUN chmod +x /start_px4_airsim_gcs.sh

# Expose ports required for communication:
# 4560 (TCP): AirSim connection to PX4 SITL (default AirSim-PX4 TCP port)
# 14550 (UDP): MAVLink for Ground Control Station (QGroundControl)
# 8888 (UDP): micro-ROS agent for ROS 2 communication (default micro-ROS agent port)
EXPOSE 4560/tcp 14550/udp 8888/udp

# Command to run when the container starts
# This will execute our custom startup script.
CMD ["/start_px4_airsim_gcs.sh"]