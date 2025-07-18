#!/bin/bash
# Docker container maintenance script

echo "=== AirSim ROS2 Container Maintenance ==="

# Function to print colored messages
print_info() {
    echo -e "\e[32m[INFO]\e[0m $1"
}

print_warning() {
    echo -e "\e[33m[WARNING]\e[0m $1"
}

print_error() {
    echo -e "\e[31m[ERROR]\e[0m $1"
}

# Check if running inside container
if [ ! -f /.dockerenv ]; then
    print_error "This script must be run inside the Docker container"
    exit 1
fi

# Get the current user
CURRENT_USER=$(whoami)
print_info "Running as user: $CURRENT_USER"

# Fix workspace permissions
print_info "Fixing workspace permissions..."
sudo chown -R $CURRENT_USER:$CURRENT_USER /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install /airsim_ros2_ws/src 2>/dev/null || true
chmod -R 755 /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install 2>/dev/null || true

# Clean up build artifacts if requested
if [ "$1" = "clean" ]; then
    print_info "Cleaning build artifacts..."
    rm -rf /airsim_ros2_ws/build /airsim_ros2_ws/install /airsim_ros2_ws/log
    print_info "Build artifacts cleaned"
fi

# Check disk space
print_info "Disk space usage:"
df -h /airsim_ros2_ws

# Check ROS2 environment
print_info "ROS2 environment:"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_VERSION: $ROS_VERSION"
else
    print_error "ROS2 not found!"
fi

# Check if workspace is built
if [ -f /airsim_ros2_ws/install/setup.bash ]; then
    print_info "Workspace is built"
else
    print_warning "Workspace not built yet"
fi

print_info "Maintenance complete!"