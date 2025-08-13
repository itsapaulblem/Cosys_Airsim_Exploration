# Unified PX4 Docker Image for AirSim Integration
# Supports both single drone and multi-drone configurations

FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
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
    ccache \
    net-tools \
    socat \
    && rm -rf /var/lib/apt/lists/*

# Create working directory for PX4
WORKDIR /px4_workspace

# Clone PX4-Autopilot repository - pin to stable release
# RUN git clone --depth 1 --recursive -b v1.14.3 https://github.com/PX4/PX4-Autopilot.git
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 dependencies
WORKDIR /px4_workspace/PX4-Autopilot
RUN ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Fix EmPy version issue - PX4 v1.14.3 requires older EmPy version
RUN pip3 install --force-reinstall empy==3.3.4

# Build PX4 with SITL target including none_iris model for AirSim
RUN DONT_RUN=1 make px4_sitl_default
# RUN DONT_RUN=1 make px4_sitl_default none_iris

# Set default PX4 environment variables
ENV PX4_HOME_LAT=47.641468
ENV PX4_HOME_LON=-122.140165
ENV PX4_HOME_ALT=0.0
ENV PX4_SYS_AUTOSTART=10016
ENV PX4_INSTANCE=1

# Install Fast-DDS 2.3.1
# RUN rm -rf /usr/local/include/fastrtps /usr/local/share/fastrtps /usr/local/lib/libfastrtps* \
# 	&& git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.3.1 /tmp/FastRTPS-2.3.1 \
# 	&& cd /tmp/FastRTPS-2.3.1 \
# 	&& mkdir build && cd build \
# 	&& cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
# 	&& cmake --build . --target install -- -j $(nproc) \
# 	&& rm -rf /tmp/*

#
# Setup XRCE-DDS Agent & Client
#
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/

# Create Scripts directory and copy PX4Scripts
RUN mkdir -p /Scripts
COPY scripts/run_airsim_sitl_final.sh /Scripts/
COPY scripts/run_airsim_sitl_fixed.sh /Scripts/
COPY scripts/sitl_kill.sh /Scripts/
COPY scripts/px4_console.sh /Scripts/
COPY scripts/px4_shell.sh /Scripts/
RUN chmod +x /Scripts/*.sh

# Create convenient aliases
RUN echo 'alias px4-console="/Scripts/px4_shell.sh"' >> /root/.bashrc
RUN echo 'alias px4-shell="/Scripts/px4_shell.sh"' >> /root/.bashrc

# Expose the port range for multi-drone configurations
EXPOSE 4561-4565/tcp 14541-14545/udp 14581-14585/udp

# Set the working directory
WORKDIR /px4_workspace/PX4-Autopilot

# Run the PX4 SITL script with instance number
CMD ["/bin/bash", "-c", "/Scripts/run_airsim_sitl_final.sh ${PX4_INSTANCE}"] 