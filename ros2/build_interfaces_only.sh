#!/bin/bash

# Build script for ROS2 interfaces only (for Docker environment)
# This builds only the interface packages without the full AirSim library

echo "Building ROS2 interfaces only (Docker-friendly build)"

# Source ROS2
source /opt/ros/humble/setup.bash

# Build only the interface packages
colcon build --packages-select airsim_interfaces airsim_mission_interfaces airsim_grid_search

echo "Interface packages built successfully!"
echo ""
echo "To use the interfaces:"
echo "source install/setup.bash"
echo ""
echo "Note: This build excludes airsim_ros_pkgs which requires full AirSim source tree."
echo "For full functionality, use the complete Docker setup with pre-built AirSim libraries."