#!/bin/bash
# Colcon wrapper script that ensures proper permissions before building

# Function to fix permissions
fix_permissions() {
    echo "Fixing workspace permissions..."
    sudo chown -R $USER:$USER /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install /airsim_ros2_ws/src 2>/dev/null || true
    chmod -R 755 /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install 2>/dev/null || true
    echo "Permissions fixed!"
}

# Always fix permissions before running colcon
fix_permissions

# Run colcon with all passed arguments
exec colcon "$@"