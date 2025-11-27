#!/bin/bash
# Development aliases and environment setup for AirSim ROS2 workspace
# This script is sourced during Docker build to set up development conveniences

# Set up .bashrc with ROS2 environment
cat >> /home/Aortz/.bashrc << 'EOF'

# === AirSim ROS2 Development Environment ===
source /opt/ros/humble/setup.bash
source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true
cd /airsim_ros2_ws

# Standard shell aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# ROS2 workspace aliases with automatic PIE fix integration
alias build='echo "Applying PIE fix..." && /airsim_ros2_ws/scripts/fix_rpc_build.sh'
alias build_interfaces='echo "Applying PIE fix..." && /airsim_ros2_ws/scripts/fix_rpc_build.sh --packages-select airsim_interfaces'
alias build_pkgs='echo "Applying PIE fix..." && /airsim_ros2_ws/scripts/fix_rpc_build.sh --packages-select airsim_ros_pkgs'
alias source_ws='source install/setup.bash'
alias clean_build='rm -rf build install log && echo "Applying PIE fix..." && /airsim_ros2_ws/scripts/fix_rpc_build.sh'
# Separate aliases for manual PIE fix control
alias build_only='colcon build'
alias build_no_fix='colcon build'

# Multi-node launch aliases
alias launch_multi='ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py enable_octomap:=true enable_octomap_rviz:=true'
alias launch_mission='ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true enable_octomap:=true enable_octomap_rviz:=true'
alias launch_legacy='ros2 launch airsim_ros_pkgs airsim_node.launch.py'
alias launch_rviz='rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz'

# Development utilities
alias test_airsim='python3 /debug_airsim_connection.sh'
alias workspace_info='echo "Workspace: $(pwd)" && echo "ROS_DISTRO: $ROS_DISTRO" && echo "Build status: $([ -d install ] && echo "Built" || echo "Not built")"'

# Auto-show workspace status on login
echo "=== AirSim ROS2 Development Environment ==="
echo "Workspace: /airsim_ros2_ws"
echo "ROS Distribution: $ROS_DISTRO"
if [ -d /airsim_ros2_ws/install ]; then
    echo "Status: Workspace built and ready"
    echo "Use: launch_multi (for multi-node) or launch_mission (for multi-node with mission capabilities)"
    echo "Use: launch_rviz for rviz2 visualization"
else
    echo "Status: Workspace needs building"
    echo "Use: build (to build workspace)"
fi
echo "============================================"

EOF

# Set ownership
chown Aortz:Aortz /home/Aortz/.bashrc

echo "Development aliases configured successfully"