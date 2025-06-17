#!/bin/bash

set -e  # Exit on any error

echo "=== Starting PX4 SITL for AirSim Integration ==="

# Source ROS 2 environment
echo "Sourcing ROS 2 Humble environment..."
source /opt/ros/humble/setup.bash

# Verify PX4 binary exists
PX4_BINARY="/px4_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [ ! -f "$PX4_BINARY" ]; then
    echo "ERROR: PX4 binary not found at $PX4_BINARY"
    echo "Checking if PX4 build directory exists..."
    ls -la /px4_ws/src/PX4-Autopilot/build/ || echo "Build directory not found"
    echo "Attempting to build PX4..."
    cd /px4_ws/src/PX4-Autopilot
    make px4_sitl_default
    if [ ! -f "$PX4_BINARY" ]; then
        echo "ERROR: PX4 build failed"
        exit 1
    fi
fi

echo "PX4 Home Position: LAT=$PX4_HOME_LAT, LON=$PX4_HOME_LON, ALT=$PX4_HOME_ALT"

# Set PX4 working directory
cd /px4_ws/src/PX4-Autopilot

# Use the standard PX4 startup script but modify simulator settings
# Copy the default rcS and modify it for AirSim
cp /px4_ws/src/PX4-Autopilot/build/px4_sitl_default/etc/init.d-root/rcS /tmp/rcS_airsim

# Add AirSim-specific configuration to the startup script
cat >> /tmp/rcS_airsim << 'EOF'

# AirSim-specific configuration
param set SIM_BAT_ENABLE 1
param set CBRK_SUPPLY_CHK 894281
param set SYS_USE_IO 0

# Start simulator for AirSim (TCP port 4560)
simulator start -c 4560

# Start MAVLink for ground control station
mavlink start -x -u 14550 -r 4000000
EOF

chmod +x /tmp/rcS_airsim

# Start PX4 SITL in daemon mode for AirSim
echo "Starting PX4 SITL for AirSim connection..."
echo "PX4 will listen on TCP port 4560 for AirSim"
echo "MAVLink will be available on UDP port 14550"

# Start PX4 with custom AirSim configuration
$PX4_BINARY -d \
    -s /tmp/rcS_airsim \
    -w /px4_ws/src/PX4-Autopilot \
    /px4_ws/src/PX4-Autopilot &

PX4_PID=$!

# Function to cleanup on exit
cleanup() {
    echo "Shutting down PX4..."
    kill $PX4_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

sleep 2

# Check if PX4 is still running
if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 process died immediately"
    exit 1
fi

echo "=== PX4 SITL started successfully ==="
echo "PX4 SITL PID: $PX4_PID"
echo "Waiting for AirSim connection on TCP port 4560..."
echo "MAVLink available on UDP port 14550"

# Show listening ports
echo "Checking if ports are open..."
netstat -ln | grep -E "(4560|14550)" || echo "Ports not yet open, waiting..."

echo "=== Ready for AirSim connection ==="

# Wait for PX4 to finish
wait $PX4_PID 