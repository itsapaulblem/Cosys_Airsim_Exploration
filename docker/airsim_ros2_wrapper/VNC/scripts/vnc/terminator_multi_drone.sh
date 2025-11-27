#!/bin/bash
# Enhanced Terminator launcher for multi-drone ROS2 development with config override

ENHANCED_CONFIG="/etc/terminator-config-enhanced"
USER_CONFIG="/home/Aortz/.config/terminator/config"

echo "Starting Terminator with Enhanced Multi-Drone ROS2 Development Layout..."

# Check if Terminator is already running (single instance check)
if pgrep -f "terminator" >/dev/null; then
    echo "Terminator is already running - bringing to focus instead of launching duplicate"
    # Try to bring existing Terminator to focus
    wmctrl -a "terminator" 2>/dev/null || true
    exit 0
fi

# Apply enhanced configuration override
echo "Applying enhanced configuration..."
if [ -f "$ENHANCED_CONFIG" ]; then
    mkdir -p "$(dirname "$USER_CONFIG")"
    cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null || sudo cp "$ENHANCED_CONFIG" "$USER_CONFIG"
    chown Aortz:Aortz "$USER_CONFIG" 2>/dev/null || sudo chown Aortz:Aortz "$USER_CONFIG"
    chmod 644 "$USER_CONFIG"

    config_size=$(wc -c < "$USER_CONFIG" 2>/dev/null || echo "0")
    echo "Enhanced config applied (${config_size} bytes)"
else
    echo "Enhanced config not found, using existing configuration"
fi

cd /airsim_ros2_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true

# Launch Terminator with enhanced settings
export TERM=xterm-256color
export COLORTERM=truecolor

echo "[SUCCESS] Launching 2-tab multi-drone development environment..."
terminator --maximise