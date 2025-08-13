#!/bin/bash
set -e
echo "=== Starting AirSim ROS2 Wrapper ==="
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ ! -f "/airsim_ros2_ws/install/setup.bash" ]; then
    echo "⚠️  Workspace not built yet. Building now..."
    echo "This is normal for first-time setup or when using volume mounting."
    cd /airsim_ros2_ws
    
    # Set AIRSIM_ROOT for the build process
    export AIRSIM_ROOT=/airsim_ros2_ws
    
    # Build the workspace
    echo "Building airsim_interfaces..."
    colcon build --packages-select airsim_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    echo "Building airsim_ros_pkgs..."
    colcon build --packages-select airsim_ros_pkgs --cmake-args -DCMAKE_BUILD_TYPE=Release -DAIRSIM_ROOT=/airsim_ros2_ws
    
    echo "✓ Workspace built successfully!"
fi

source /airsim_ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export AIRSIM_HOST_IP=${AIRSIM_HOST_IP:-host.docker.internal}
export AIRSIM_HOST_PORT=${AIRSIM_HOST_PORT:-41451}
export LAUNCH_RVIZ=${LAUNCH_RVIZ:-false}

echo "Configuration: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Host=$AIRSIM_HOST_IP:$AIRSIM_HOST_PORT, RViz=$LAUNCH_RVIZ"

if timeout 5 bash -c "echo >/dev/tcp/$AIRSIM_HOST_IP/$AIRSIM_HOST_PORT" 2>/dev/null; then
    echo "✓ Connected to AirSim API"
else
    echo "✗ Cannot connect to AirSim at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
    echo "Ensure AirSim is running with ApiServerEndpoint: '0.0.0.0:41451'"
    exit 1
fi

if [ "$LAUNCH_RVIZ" = "true" ]; then
    echo "🚀 Launching AirSim node with RViz2..."
    echo "📊 Point cloud retention tips:"
    echo "   - In RViz2: Add -> By topic -> /airsim_node/Drone1/lidar/PointCloud2"
    echo "   - Set History Policy to 'Keep All' or increase History Size"
    echo "   - Set Fixed Frame to 'odom' or 'world'"
    echo "   - Adjust Decay Time to retain points longer"
    
    ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=$AIRSIM_HOST_IP host_port:=$AIRSIM_HOST_PORT enable_api_control:=true output:=screen &
    sleep 5 && ros2 launch airsim_ros_pkgs rviz.launch.py &
    wait
else
    echo "🚀 Launching AirSim node..."
    ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=$AIRSIM_HOST_IP host_port:=$AIRSIM_HOST_PORT enable_api_control:=true output:=screen
fi 