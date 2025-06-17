### PX4, ROS 2, and AirSim Simulation with Docker ###

This repository provides the necessary files and instructions to set up a robust drone simulation environment using PX4 Autopilot, ROS 2 Humble, and AirSim, all containerized with Docker. This setup allows you to run PX4 SITL (Software-In-The-Loop) in a Docker container, connect it to AirSim running on your Windows machine, and communicate with a Ground Control Station (QGroundControl) and other ROS 2 nodes.

Table of Contents
Introduction

Prerequisites

Project Structure

Setup Instructions

1. Configure AirSim settings.json (Windows)

2. Create Project Files (Linux/WSL 2)

3. Build the Docker Image

4. Run the Docker Container

5. Launch AirSim (Windows)

6. Connect QGroundControl (Windows)

7. Interact with ROS 2

Troubleshooting

1. Introduction
This project addresses the challenge of integrating different robotics software components (PX4, ROS 2, AirSim) by leveraging Docker containers. Key benefits of this approach include:

Dependency Management: All necessary libraries and tools for PX4 and ROS 2 are encapsulated within the Docker image, avoiding conflicts with your host system.

Reproducibility: Ensures a consistent development and simulation environment across different machines and team members.

Isolation: The containerized environment prevents interference with your host operating system.

Portability: Easily move your entire simulation setup between various Linux/WSL 2 environments.

The setup consists of:

PX4 Autopilot SITL: Running as a simulated flight controller inside a Docker container.

ROS 2 Humble: Installed within the same Docker container, including the micro_ros_agent for bridging PX4 uORB topics to ROS 2.

AirSim: Running natively on your Windows machine, connecting to the PX4 SITL via TCP.

QGroundControl (GCS): Running natively on your Windows machine, connecting to PX4 SITL via MAVLink UDP.

2. Prerequisites
Before you begin, ensure you have the following installed on your system:

On your Windows Machine:

Docker Desktop: Essential for running Linux Docker containers and for Docker Compose. This includes WSL 2, which Docker Desktop leverages.

AirSim:

An Unreal Engine environment with the AirSim plugin, or a pre-built AirSim binary (e.g., the Blocks environment).

Familiarity with AirSim's settings.json configuration.

QGroundControl: For monitoring and controlling the simulated drone.

(Optional) WSL 2 Terminal: While Docker Desktop handles the Linux VM, running commands directly in a WSL 2 terminal provides a more native Linux experience for Docker operations.

On your Linux machine / WSL 2 Instance (if not using Windows for Docker operations):

Docker Engine: If you are running on a native Linux machine.

Git: For cloning repositories.

3. Project Structure
Create a directory on your Linux machine or within your WSL 2 instance (e.g., ~/px4_airsim_docker) and place the following files inside:

px4_airsim_docker/
├── Dockerfile
└── start_px4_airsim_gcs.sh

4. Setup Instructions
Follow these steps to get your PX4, ROS 2, and AirSim simulation running.

1. Configure AirSim settings.json (Windows)
On your Windows machine, navigate to your AirSim settings directory (typically C:\Users\<YourUser>\Documents\AirSim\) and create or modify the settings.json file with the following content:

{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlPort": 14580,
            "UdpPort": 14540,
            "LockStep": true,
            "ExternalPhysicsEngine": false,
            "ConnectionIp": "127.0.0.1",
            "Params": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "MAV_BROADCAST": 1
            }
        }
    }
}

"ConnectionIp": "127.0.0.1": This is crucial. Since Docker will map the container's internal ports to your Windows host's localhost (127.0.0.1), AirSim can simply connect to this IP.

"TcpPort": 4560: AirSim will connect to PX4 SITL on this TCP port.

"LockStep": true: Recommended for deterministic simulation where AirSim waits for PX4.

2. Create Project Files (Linux/WSL 2)
Create the Dockerfile and start_px4_airsim_gcs.sh files in your px4_airsim_docker directory.

Dockerfile Content:
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
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
# Based on https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    # Add ROS 2 repository to sources list
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    # Update apt-get and install ROS 2 desktop full, plus PX4 and micro-ROS packages
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-px4-msgs \
    ros-humble-px4-ros-com \
    ros-humble-micro-ros-agent \
    ros-humble-rmw-microros \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script for subsequent commands in Dockerfile
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENV PATH="/opt/ros/humble/bin:${PATH}"
ENV CMAKE_PREFIX_PATH="/opt/ros/humble:${CMAKE_PREFIX_PATH}"

# Clone PX4-Autopilot repository
# Create a workspace directory
RUN mkdir -p /px4_ws/src
WORKDIR /px4_ws/src
# Clone PX4-Autopilot and its submodules
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git
WORKDIR /px4_ws/src/PX4-Autopilot

# Install PX4 dependencies.
# We use the PX4's own setup script but exclude unnecessary simulation tools
# (like Gazebo, as we are using AirSim) and toolchains for efficiency.
RUN ./Tools/setup/ubuntu.sh --no-sim-tools --no-ccache --no-mavlink-gitsubmodules --no-px4-toolchain --no-arm-toolchain

# Build PX4 with ROS 2 support
# This step compiles the PX4 firmware for SITL (Software In The Loop)
# The 'none_iris' target is a generic multicopter model without a specific integrated simulator.
# It also builds the micro-XRCE-DDS client into the firmware for ROS 2 communication.
RUN make px4_sitl_default none_iris

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

start_px4_airsim_gcs.sh Content:
#!/bin/bash

# Source ROS 2 environment for the current session
# This ensures that ROS 2 commands like 'ros2 run' are available.
source /opt/ros/humble/setup.bash

# Start the micro-ROS agent in the background.
# The micro-ROS agent acts as a bridge between the PX4 firmware (micro-ROS client)
# and the full ROS 2 environment. It listens for UDP connections on port 8888.
echo "Starting micro-ROS agent on UDP port 8888..."
micro_ros_agent udp4 --port 8888 &

# Give the micro-ROS agent a moment to initialize before PX4 tries to connect.
sleep 2

# Start PX4 SITL (Software In The Loop).
# -w /px4_ws/src/PX4-Autopilot: Sets the working directory for PX4.
# rcS: The main startup script for PX4.
# -s: Tells PX4 to expect an external simulator (AirSim).
# -m udp:127.0.0.1:14550: Configures PX4 to send MAVLink data to UDP port 14550 on localhost
#                          within the container, which will be mapped to the host.
# This command runs the compiled PX4 SITL application.
echo "Starting PX4 SITL with AirSim and GCS connections..."
/px4_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4 -w /px4_ws/src/PX4-Autopilot /px4_ws/src/PX4-Autopilot/build/px4_sitl_default/etc/init.d-root/rcS -s -m udp:127.0.0.1:14550

# Keep the script running indefinitely to prevent the container from exiting.
# This is a common practice in Docker containers running services.
tail -f /dev/null

3. Build the Docker Image
Open your terminal (on Linux or in your WSL 2 instance) in the px4_airsim_docker directory where you saved the Dockerfile and start_px4_airsim_gcs.sh.

Run the following command to build the Docker image:

docker build -t px4_airsim_ros2:humble .

This process might take a while as it downloads ROS 2 packages, clones the PX4-Autopilot repository, and compiles the PX4 firmware.

4. Run the Docker Container
Once the Docker image is built, run the container with the necessary port mappings:

docker run -it --rm \
    --name px4_sim_container \
    -p 4560:4560/tcp \
    -p 14550:14550/udp \
    -p 8888:8888/udp \
    px4_airsim_ros2:humble

--rm: Ensures the container is automatically removed when you stop it.

-p <host_port>:<container_port>/<protocol>: Maps the specified ports.

4560:4560/tcp: For AirSim to connect to PX4 SITL.

14550:14550/udp: For QGroundControl (GCS) to connect via MAVLink.

8888:8888/udp: For the micro_ros_agent (ROS 2 communication).

5. Launch AirSim (Windows)
On your Windows machine, start your AirSim environment (e.g., by running Blocks.exe or launching your Unreal project). AirSim should automatically connect to the PX4 SITL instance running in your Docker container via localhost:4560. You will see connection messages in the Docker terminal.

6. Connect QGroundControl (Windows)
Launch QGroundControl on your Windows machine. Due to the UDP port mapping (-p 14550:14550/udp), QGroundControl should automatically detect and connect to the simulated PX4 vehicle. You can then monitor telemetry, change flight modes, and send commands as you would with a physical drone.

7. Interact with ROS 2
The micro_ros_agent running inside the Docker container bridges PX4's uORB topics to ROS 2. You can now develop ROS 2 applications on your Windows host (e.g., within another WSL 2 instance or a native Linux ROS 2 environment) and communicate with the simulated PX4.

To verify ROS 2 communication:

Open a new terminal (e.g., another WSL 2 instance) where you have ROS 2 Humble installed and sourced.

Ensure your ROS 2 environment is sourced:

source /opt/ros/humble/setup.bash

List active ROS 2 topics:

ros2 topic list

You should see various PX4-related topics, such as /fmu/out/vehicle_odometry, /fmu/in/offboard_control_mode, etc. These indicate that the micro_ros_agent is successfully bridging PX4 data to the ROS 2 network on your host.

You can then create your own ROS 2 nodes to subscribe to telemetry data or publish commands to the PX4 SITL instance.

5. Troubleshooting
"Waiting for TCP connection on port 4560..." (PX4 SITL): This means PX4 is ready to connect but AirSim hasn't initiated the connection. Double-check your AirSim settings.json for correct TcpPort and ConnectionIp. Ensure AirSim is actually running and attempting to connect.

"Accepting TCP socket failed, is another instance running?" or "TcpClientPort socket bind failed with error:10048" (AirSim): This typically indicates that something else on your Windows machine is already using port 4560, or AirSim is having trouble binding to it. Ensure no other instances of AirSim or PX4 are running.

QGroundControl not connecting: Verify that your Windows firewall isn't blocking UDP traffic on port 14550.

ROS 2 topics not appearing:

Ensure the micro_ros_agent is running inside the Docker container (check the container's output).

Verify that the micro_ros_agent port 8888/udp is correctly mapped in the docker run command.

Confirm your external ROS 2 environment is properly sourced.

Container exits immediately: Check the start_px4_airsim_gcs.sh script for any errors. The tail -f /dev/null command is crucial to keep the container running indefinitely after its services start.

Performance Issues: Running AirSim and a Docker container can be resource-intensive. Ensure your machine has enough RAM and a dedicated GPU.

WSL 2 Network Issues: If you face persistent network issues with WSL 2, try restarting the WSL 2 subsystem (wsl --shutdown in PowerShell, then restart Docker Desktop).

By following these instructions, you should have a functional and well-integrated PX4, ROS 2, and AirSim simulation environment ready for your drone development tasks.