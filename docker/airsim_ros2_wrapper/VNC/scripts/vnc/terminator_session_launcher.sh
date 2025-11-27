#!/bin/bash
# Session-based Terminator launcher (secondary fallback)

LOGFILE="/home/Aortz/.terminator-autostart.log"

log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] SESSION: $1" >> "$LOGFILE"
}

# Wait a bit more for session to stabilize
sleep 3

# Check if Terminator is already running
if pgrep -f "terminator.*layout=default" >/dev/null; then
    log_message "Terminator already running, session launcher not needed"
    exit 0
fi

log_message "Session launcher: No Terminator detected, launching..."

# Set environment
export DISPLAY=:1
export TERM=xterm-256color
export COLORTERM=truecolor
cd /airsim_ros2_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true

# Final attempt to launch Terminator
nohup terminator --layout=default --title="Multi-Drone ROS2 Development Environment" --maximise >/dev/null 2>&1 &

sleep 2
if pgrep -f "terminator.*layout=default" >/dev/null; then
    log_message "Session launcher: Terminator started successfully"
else
    log_message "Session launcher: Failed to start Terminator"
fi