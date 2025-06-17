#!/bin/bash

set -e  # Exit on any error

echo "=== Starting PX4 SITL for AirSim ==="

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

# Start PX4 SITL with configuration for AirSim
echo "Starting PX4 SITL..."
$PX4_BINARY -w /px4_ws/src/PX4-Autopilot \
    /px4_ws/src/PX4-Autopilot/build/px4_sitl_default/etc/init.d-root/rcS \
    -s -m udp:0.0.0.0:14550 -t 127.0.0.1:4560 &

PX4_PID=$!

make px4_sitl_default none_iris

# Function to cleanup on exit
cleanup() {
    echo "Shutting down PX4..."
    kill $PX4_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

echo "=== PX4 SITL started successfully ==="
echo "PX4 SITL PID: $PX4_PID"
echo "Ports:"
echo "  - 4560/tcp: AirSim connection"
echo "  - 14550/udp: MAVLink GCS"
echo "=== Ready for AirSim connection ==="

# Wait for PX4 to finish
wait $PX4_PID 