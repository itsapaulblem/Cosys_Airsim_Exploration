#!/bin/bash

set -e  # Exit on any error

echo "=== Starting PX4-AirSim-ROS2 Container ==="

# Source ROS 2 environment from the base installation
echo "Sourcing ROS 2 Humble environment..."
source /opt/ros/humble/setup.bash

# Source the custom ROS 2 workspace where micro_ros_agent and PX4 messages were built
echo "Sourcing custom ROS 2 workspace..."
source /ros2_ws/install/setup.bash

# Verify that micro_ros_agent is available
if ! command -v ros2 &> /dev/null || ! ros2 pkg list | grep -q micro_ros_agent; then
    echo "ERROR: micro_ros_agent not found in ROS 2 workspace"
    exit 1
fi

# Start the micro-ROS agent in the background.
echo "Starting micro-ROS agent on UDP port 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > /tmp/micro_ros_agent.log 2>&1 &
MICRO_ROS_PID=$!

# Give the micro-ROS agent a moment to initialize before PX4 tries to connect.
echo "Waiting for micro-ROS agent to initialize..."
sleep 3

# Check if micro-ROS agent is still running
if ! kill -0 $MICRO_ROS_PID 2>/dev/null; then
    echo "ERROR: micro-ROS agent failed to start. Check logs:"
    cat /tmp/micro_ros_agent.log
    exit 1
fi

echo "micro-ROS agent started successfully (PID: $MICRO_ROS_PID)"

# Verify PX4 binary exists
PX4_BINARY="/ros2_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [ ! -f "$PX4_BINARY" ]; then
    echo "ERROR: PX4 binary not found at $PX4_BINARY"
    exit 1
fi

# Start PX4 SITL (Software In The Loop).
echo "Starting PX4 SITL with AirSim and GCS connections..."
echo "PX4 Home Position: LAT=$PX4_HOME_LAT, LON=$PX4_HOME_LON, ALT=$PX4_HOME_ALT"
make px4_sitl_default none_iris

# Set PX4 working directory
cd /ros2_ws/src/PX4-Autopilot

# Start PX4 with proper configuration for AirSim
$PX4_BINARY -w /ros2_ws/src/PX4-Autopilot \
    /ros2_ws/src/PX4-Autopilot/build/px4_sitl_default/etc/init.d-root/rcS \
    -s -m udp:0.0.0.0:14550 &

PX4_PID=$!

# Function to cleanup on exit
cleanup() {
    echo "Shutting down processes..."
    kill $PX4_PID 2>/dev/null || true
    kill $MICRO_ROS_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

echo "=== All services started successfully ==="
echo "PX4 SITL PID: $PX4_PID"
echo "micro-ROS Agent PID: $MICRO_ROS_PID"
echo "Ports:"
echo "  - 4560/tcp: AirSim connection"
echo "  - 14550/udp: MAVLink GCS"
echo "  - 8888/udp: micro-ROS agent"
echo "=== Container ready ==="

# Wait for processes to finish
wait