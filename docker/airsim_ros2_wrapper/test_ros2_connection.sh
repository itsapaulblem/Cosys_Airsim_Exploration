#!/bin/bash

echo "=== Testing ROS2 Connection to AirSim ==="

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Get Windows host IP (where AirSim is running)
WINDOWS_HOST_IP=${AIRSIM_HOST_IP:-"host.docker.internal"}

echo "Testing connection to AirSim at: $WINDOWS_HOST_IP:41451"

# Test if we can reach AirSim API port
if command -v nc >/dev/null 2>&1; then
    echo "Testing TCP connection..."
    nc -zv $WINDOWS_HOST_IP 41451
    if [ $? -eq 0 ]; then
        echo "SUCCESS: Can reach AirSim API port"
    else
        echo "ERROR: Cannot reach AirSim API port"
        echo "Make sure AirSim is running on Windows with API enabled"
    fi
else
    echo "netcat not available, skipping connection test"
fi

# Test if ROS2 is working
echo "Testing ROS2 installation..."
ros2 --help > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "SUCCESS: ROS2 is working"
else
    echo "ERROR: ROS2 is not working properly"
    exit 1
fi

# List available ROS2 packages
echo "Available ROS2 packages:"
ros2 pkg list | grep -E "(airsim|cv_bridge|image_transport)" || echo "AirSim packages not found"

echo "=== Test complete ===" 