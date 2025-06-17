#!/bin/bash

# AirSim ROS2 Wrapper Launch Script
echo "Starting AirSim ROS2 Wrapper..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /airsim_ros2_ws/install/setup.bash

# Set default parameters if not provided
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export AIRSIM_HOST_IP=${AIRSIM_HOST_IP:-host.docker.internal}
export AIRSIM_HOST_PORT=${AIRSIM_HOST_PORT:-41451}

echo "Configuration:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  AirSim Host: $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"

# Wait for AirSim to be available
echo "Waiting for AirSim connection..."
timeout=30
counter=0
while ! nc -z $AIRSIM_HOST_IP $AIRSIM_HOST_PORT; do
    sleep 1
    counter=$((counter + 1))
    if [ $counter -ge $timeout ]; then
        echo "Warning: Could not connect to AirSim at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
        echo "Make sure AirSim is running and accessible"
        break
    fi
done

if nc -z $AIRSIM_HOST_IP $AIRSIM_HOST_PORT; then
    echo "AirSim connection established!"
else
    echo "Proceeding without confirmed AirSim connection..."
fi

# Launch AirSim ROS2 node
echo "Launching AirSim ROS2 node..."
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
    host_ip:=$AIRSIM_HOST_IP \
    host_port:=$AIRSIM_HOST_PORT \
    output:=screen 