# Ultra-Swarm PX4 Docker Image for AirSim Integration
# Enhanced version supporting up to 27 drones (9 per swarm x 3 swarms)
# Includes GPS home location fixes and proper parameter configuration

FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies for PX4
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

# Clone PX4-Autopilot repository
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 dependencies
WORKDIR /px4_workspace/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Build PX4 SITL
RUN DONT_RUN=1 make px4_sitl_default

# Install additional utilities for ultra-swarm support and GPS configuration
RUN apt-get update && apt-get install -y --no-install-recommends \
    netcat-openbsd \
    procps \
    curl \
    jq \
    && rm -rf /var/lib/apt/lists/*

# Environment variables for ultra-swarm support
ENV PX4_HOME_LAT=47.641468
ENV PX4_HOME_LON=-122.140165
ENV PX4_HOME_ALT=0.0
ENV PX4_SYS_AUTOSTART=10016

# Instance-specific variables (will be overridden by docker-compose or runtime)
ENV PX4_INSTANCE=1
ENV SWARM_ID=1
ENV SWARM_SIZE=1
ENV MAX_SWARMS=3
ENV MAX_DRONES_PER_SWARM=9

# Copy GPS configuration files
COPY config/px4_params.txt /Scripts/
COPY config/gps_locations.json /Scripts/

# Create the enhanced ultra-swarm startup script with GPS configuration
RUN echo '#!/bin/bash' > /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Ultra-swarm PX4 startup script with GPS home location fixes' >> /Scripts/run_ultra_swarm.sh && \
    echo 'instance_num=${PX4_INSTANCE:-1}' >> /Scripts/run_ultra_swarm.sh && \
    echo 'swarm_id=${SWARM_ID:-1}' >> /Scripts/run_ultra_swarm.sh && \
    echo 'swarm_size=${SWARM_SIZE:-1}' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Enhanced port calculation for 9 drones per swarm x 3 swarms' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Swarm 1: 4561-4569, Swarm 2: 4571-4579, Swarm 3: 4581-4589' >> /Scripts/run_ultra_swarm.sh && \
    echo 'airsim_tcp_port=$((4560 + (swarm_id - 1) * 10 + instance_num))' >> /Scripts/run_ultra_swarm.sh && \
    echo 'qgc_port=$((14549 + (swarm_id - 1) * 10 + instance_num))' >> /Scripts/run_ultra_swarm.sh && \
    echo 'mavlink_udp_port=$((18569 + (swarm_id - 1) * 10 + instance_num))' >> /Scripts/run_ultra_swarm.sh && \
    echo 'mavlink_tcp_port=$((4659 + (swarm_id - 1) * 10 + instance_num))' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# GPS home location based on swarm ID (slight offsets for realism)' >> /Scripts/run_ultra_swarm.sh && \
    echo 'home_lat=$(echo "${PX4_HOME_LAT} + (${swarm_id} - 1) * 0.001" | bc -l)' >> /Scripts/run_ultra_swarm.sh && \
    echo 'home_lon=$(echo "${PX4_HOME_LON} + (${swarm_id} - 1) * 0.001" | bc -l)' >> /Scripts/run_ultra_swarm.sh && \
    echo 'home_alt=${PX4_HOME_ALT}' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "🚁 Starting PX4 SITL - Ultra-Swarm Configuration"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "   Swarm ID: $swarm_id, Instance: $instance_num (of $swarm_size)"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "   AirSim TCP: $airsim_tcp_port, QGC UDP: $qgc_port, MAVLink UDP: $mavlink_udp_port"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "   GPS Home: $home_lat, $home_lon, $home_alt"' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# PX4 paths' >> /Scripts/run_ultra_swarm.sh && \
    echo 'PARENT_DIR="/px4_workspace/PX4-Autopilot"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'BUILD_DIR=$PARENT_DIR/ROMFS/px4fmu_common' >> /Scripts/run_ultra_swarm.sh && \
    echo 'instance_path=$PARENT_DIR/build/px4_sitl_default' >> /Scripts/run_ultra_swarm.sh && \
    echo 'BIN_DIR=$PARENT_DIR/build/px4_sitl_default/bin/px4' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Use none_iris model for external simulators' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export PX4_SIM_MODEL=none_iris' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export PX4_SIMULATOR=none' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# GPS and EKF2 configuration for reliable home position' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export PX4_HOME_LAT=$home_lat' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export PX4_HOME_LON=$home_lon' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export PX4_HOME_ALT=$home_alt' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Check if PX4 binary exists' >> /Scripts/run_ultra_swarm.sh && \
    echo 'if [ ! -f "$BIN_DIR" ]; then' >> /Scripts/run_ultra_swarm.sh && \
    echo '    echo "❌ PX4 binary not found at: $BIN_DIR"' >> /Scripts/run_ultra_swarm.sh && \
    echo '    exit 1' >> /Scripts/run_ultra_swarm.sh && \
    echo 'fi' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Create instance working directory' >> /Scripts/run_ultra_swarm.sh && \
    echo 'working_dir="$instance_path/ultra_swarm_${swarm_id}/drone_$instance_num"' >> /Scripts/run_ultra_swarm.sh && \
    echo '[ ! -d "$working_dir" ] && mkdir -p "$working_dir"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "Working directory: $working_dir"' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Configure MAVLink for this instance' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export MAV_0_UDP_PORT=$mavlink_udp_port' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export MAV_0_REMOTE_PORT=$qgc_port' >> /Scripts/run_ultra_swarm.sh && \
    echo 'export MAV_1_TCP_PORT=$mavlink_tcp_port' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Create parameter file with GPS configuration' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param_file="$working_dir/px4_params_${swarm_id}_${instance_num}.txt"' >> /Scripts/run_ultra_swarm.sh && \
    echo 'cat > "$param_file" << PARAMS_EOF' >> /Scripts/run_ultra_swarm.sh && \
    echo '# GPS and EKF2 Parameters for Ultra-Swarm Configuration' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set LPE_LAT $home_lat' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set LPE_LON $home_lon' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set COM_ARM_WO_GPS 1' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set NAV_RCL_ACT 0' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set NAV_DLL_ACT 0' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set COM_OBL_ACT 1' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set EKF2_GPS_CHECK 21' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set EKF2_GPS_CTRL 7' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set EKF2_HGT_MODE 0' >> /Scripts/run_ultra_swarm.sh && \
    echo 'param set SYS_HAS_GPS 1' >> /Scripts/run_ultra_swarm.sh && \
    echo 'PARAMS_EOF' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Change to working directory' >> /Scripts/run_ultra_swarm.sh && \
    echo 'cd "$working_dir"' >> /Scripts/run_ultra_swarm.sh && \
    echo '' >> /Scripts/run_ultra_swarm.sh && \
    echo '# Start PX4 with GPS configuration and instance-specific parameters' >> /Scripts/run_ultra_swarm.sh && \
    echo 'echo "🚀 Starting PX4 process with GPS configuration..."' >> /Scripts/run_ultra_swarm.sh && \
    echo 'exec $BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $PARENT_DIR/test_data -d "$param_file"' >> /Scripts/run_ultra_swarm.sh

# Create health check script for ultra-swarms
RUN echo '#!/bin/bash' > /Scripts/ultra_swarm_health_check.sh && \
    echo 'echo "🔍 Ultra-Swarm Health Check - Swarm $SWARM_ID, Drone $PX4_INSTANCE"' >> /Scripts/ultra_swarm_health_check.sh && \
    echo 'if pgrep -f "px4" > /dev/null; then' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    echo "✅ PX4 process is running"' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    # Check GPS status' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    if [ -f "/tmp/px4_gps_status" ]; then' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '        echo "📍 GPS Status: $(cat /tmp/px4_gps_status)"' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    fi' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    exit 0' >> /Scripts/ultra_swarm_health_check.sh && \
    echo 'else' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    echo "❌ PX4 process not found"' >> /Scripts/ultra_swarm_health_check.sh && \
    echo '    exit 1' >> /Scripts/ultra_swarm_health_check.sh && \
    echo 'fi' >> /Scripts/ultra_swarm_health_check.sh

# Install bc for floating point calculations
RUN apt-get update && apt-get install -y bc && rm -rf /var/lib/apt/lists/*

# Make all scripts executable
RUN chmod +x /Scripts/*.sh

# Create convenient aliases for ultra-swarm management
RUN echo 'alias ultra-status="/Scripts/ultra_swarm_health_check.sh"' >> /root/.bashrc && \
    echo 'alias px4-params="cat /Scripts/px4_params.txt"' >> /root/.bashrc

# Expose port ranges for ultra-swarm configurations
# Supports 3 swarms x 9 drones each = 27 total drones
EXPOSE 4561-4589/tcp 14550-14578/udp 18570-18598/udp 4660-4688/tcp

# Add enhanced healthcheck for ultra-swarm monitoring
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD /Scripts/ultra_swarm_health_check.sh || exit 1

# Set the working directory
WORKDIR /px4_workspace/PX4-Autopilot/Scripts

# Default command supports ultra-swarm mode with GPS configuration
CMD ["/bin/bash"]