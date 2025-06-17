#!/bin/bash

set -e  # Exit on any error

echo "=== Starting PX4 SITL for AirSim (Minimal Setup) ==="

# Source ROS 2 environment
echo "Sourcing ROS 2 Humble environment..."
source /opt/ros/humble/setup.bash

# Verify PX4 binary exists
PX4_BINARY="/px4_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [ ! -f "$PX4_BINARY" ]; then
    echo "ERROR: PX4 binary not found at $PX4_BINARY"
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

# Set environment variables for PX4
export PX4_SIM_MODEL=iris
export PX4_SIM_HOSTNAME=127.0.0.1
export PX4_SIM_PORT=4560

# Start PX4 SITL using the standard approach
echo "Starting PX4 SITL for external simulator (AirSim)..."
echo "PX4 will connect to simulator on TCP port 4560"
echo "MAVLink will be available on UDP port 14550"

# Use the standard PX4 startup with external simulator
$PX4_BINARY -d \
    -s build/px4_sitl_default/etc/init.d-root/rcS \
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

sleep 3

# Check if PX4 is still running
if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 process died immediately"
    exit 1
fi

echo "=== PX4 SITL started successfully ==="
echo "PX4 SITL PID: $PX4_PID"
echo "Waiting for AirSim connection on TCP port 4560..."
echo "MAVLink available on UDP port 14550"

# Show listening ports (if netstat is available)
if command -v netstat >/dev/null 2>&1; then
    echo "Checking if ports are open..."
    netstat -ln | grep -E "(4560|14550)" || echo "Ports not yet open, waiting..."
fi

echo "=== Ready for AirSim connection ==="

# Wait for PX4 to finish
wait $PX4_PID 