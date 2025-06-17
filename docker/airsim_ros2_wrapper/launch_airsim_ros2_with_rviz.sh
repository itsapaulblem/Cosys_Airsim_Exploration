#!/bin/bash

echo "Starting AirSim ROS2 Wrapper..."
source /opt/ros/humble/setup.bash
source /airsim_ros2_ws/install/setup.bash

# Set default parameters
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export AIRSIM_HOST_IP=${AIRSIM_HOST_IP:-host.docker.internal}
export AIRSIM_HOST_PORT=${AIRSIM_HOST_PORT:-41451}
export LAUNCH_RVIZ=${LAUNCH_RVIZ:-false}

echo "Configuration:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  AirSim Host: $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
echo "  Launch RViz2: $LAUNCH_RVIZ"

# Function to handle cleanup
cleanup() {
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null
    wait
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "Launching AirSim ROS2 node..."
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
    host_ip:=$AIRSIM_HOST_IP \
    host_port:=$AIRSIM_HOST_PORT \
    output:=screen &

AIRSIM_PID=$!

# Launch RViz2 if requested
if [ "$LAUNCH_RVIZ" = "true" ]; then
    echo "Waiting 5 seconds before launching RViz2..."
    sleep 5
    echo "Launching RViz2..."
    ros2 launch airsim_ros_pkgs rviz.launch.py &
    RVIZ_PID=$!
fi

# Wait for processes
wait $AIRSIM_PID 