#!/bin/bash

# Grid Search Mission System Build Script
# Builds the complete grid search system for AirSim

set -e

echo "🚁 Building AirSim Grid Search Mission System"
echo "============================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 environment not sourced. Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo "📋 ROS2 Distribution: $ROS_DISTRO"
echo "📁 Working directory: $(pwd)"

# Navigate to workspace
cd "$(dirname "$0")"

# Clean previous builds (optional)
if [ "$1" = "--clean" ]; then
    echo "🧹 Cleaning previous builds..."
    rm -rf build install log
fi

# Build mission interfaces first (required by grid search package)
echo ""
echo "🔧 Building mission interfaces..."
colcon build --packages-select airsim_mission_interfaces \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --executor sequential

if [ $? -ne 0 ]; then
    echo "❌ Failed to build airsim_mission_interfaces"
    exit 1
fi

# Source the built interfaces
echo "📦 Sourcing built interfaces..."
source install/setup.bash

# Build grid search package
echo ""
echo "🔧 Building grid search system..."
colcon build --packages-select airsim_grid_search \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --executor sequential

if [ $? -ne 0 ]; then
    echo "❌ Failed to build airsim_grid_search"
    exit 1
fi

# Source the complete workspace
echo "📦 Sourcing complete workspace..."
source install/setup.bash

echo ""
echo "✅ Build completed successfully!"
echo ""
echo "🚀 To use the grid search system:"
echo "   1. Source the workspace: source install/setup.bash"
echo "   2. Launch AirSim ROS2: ros2 launch airsim_ros_pkgs airsim_node.launch.py"
echo "   3. Launch grid search: ros2 launch airsim_grid_search grid_search.launch.py"
echo "   4. Test with CLI: ros2 run airsim_grid_search grid_search_cli --help"
echo ""
echo "📚 See README.md for detailed usage instructions"