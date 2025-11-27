#!/bin/bash
# Build workspace script for AirSim ROS2 development
# Handles common build scenarios with proper error handling

set -e  # Exit on error

WORKSPACE_DIR="/airsim_ros2_ws"
cd "$WORKSPACE_DIR"

echo "========================================"
echo "AirSim ROS2 Workspace Builder"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "ROS Distribution: $ROS_DISTRO"
echo

# Check if we're in the right directory
if [ ! -f "$WORKSPACE_DIR/src" ]; then
    echo "❌ Error: ROS2 workspace source directory not found"
    echo "Expected: $WORKSPACE_DIR/src"
    exit 1
fi

# Source ROS2 environment
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Check build mode
BUILD_MODE=${1:-"full"}
case "$BUILD_MODE" in
    "full")
        echo "🔨 Building complete workspace..."
        colcon build --symlink-install
        ;;
    "interfaces")
        echo "🔧 Building interfaces only..."
        colcon build --symlink-install --packages-select airsim_interfaces
        ;;
    "packages"|"pkgs")
        echo "📦 Building AirSim packages only..."
        colcon build --symlink-install --packages-select airsim_ros_pkgs
        ;;
    "clean")
        echo "🧹 Clean build - removing old artifacts..."
        rm -rf build install log
        colcon build --symlink-install
        ;;
    *)
        echo "❌ Unknown build mode: $BUILD_MODE"
        echo "Usage: $0 [full|interfaces|packages|clean]"
        exit 1
        ;;
esac

# Check build success
if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully!"
    echo
    echo "📋 Next steps:"
    echo "  source install/setup.bash    # Source the workspace"
    echo "  launch_multi                 # Launch multi-node system"
    echo "  launch_rviz                  # Open visualization"
else
    echo "❌ Build failed!"
    echo "Check the output above for error details"
    exit 1
fi

# Show workspace status
if [ -d install ]; then
    echo
    echo "📊 Workspace Status:"
    echo "Built packages:"
    ls -1 install/ | grep -v "_setup_util" | head -10
    if [ $(ls -1 install/ | grep -v "_setup_util" | wc -l) -gt 10 ]; then
        echo "... and $(( $(ls -1 install/ | grep -v "_setup_util" | wc -l) - 10 )) more"
    fi
fi

echo "========================================"