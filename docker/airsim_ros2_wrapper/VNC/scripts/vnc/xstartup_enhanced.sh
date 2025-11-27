#!/bin/bash
# Enhanced VNC xstartup with Terminator integration

# Clean environment
unset SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS
export DISPLAY=:1

# Start dbus
[ -x /usr/bin/dbus-launch ] && eval $(dbus-launch --sh-syntax --exit-with-session)

# Load user resources
xrdb $HOME/.Xresources 2>/dev/null || true
xsetroot -solid grey 2>/dev/null || true

# Start XFCE4 (autostart will handle primary Terminator launch)
startxfce4 &

# Enhanced fallback mechanism - only launches if autostart fails
cat > /tmp/terminator_fallback_checker.sh << 'FALLBACK_EOF'
#!/bin/bash
# Fallback Terminator launcher for VNC - only if autostart fails

LOGFILE="/home/Aortz/.terminator-autostart.log"
AUTOSTART_TIMEOUT=15  # Give autostart 15 seconds to work

log_fallback() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] FALLBACK: $1" >> "$LOGFILE"
}

log_fallback "VNC Fallback checker started - waiting ${AUTOSTART_TIMEOUT}s for autostart"

# Wait for autostart to have a chance
sleep $AUTOSTART_TIMEOUT

# Check if Terminator is already running
if pgrep -f "terminator" >/dev/null; then
    log_fallback "Terminator already running via autostart - fallback not needed"
    exit 0
fi

log_fallback "No Terminator detected after ${AUTOSTART_TIMEOUT}s - activating fallback launcher"

# Set up environment for fallback launch
export DISPLAY=:1
export TERM=xterm-256color
export COLORTERM=truecolor
cd /airsim_ros2_ws

# Apply enhanced config before launching
ENHANCED_CONFIG="/etc/terminator-config-enhanced"
USER_CONFIG="/home/Aortz/.config/terminator/config"

if [ -f "$ENHANCED_CONFIG" ]; then
    mkdir -p "$(dirname "$USER_CONFIG")"
    cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null
    chown Aortz:Aortz "$USER_CONFIG" 2>/dev/null
    chmod 644 "$USER_CONFIG"
    log_fallback "Enhanced config applied for fallback launch"
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true

# Launch Terminator using enhanced config (no layout override)
log_fallback "Launching Terminator via VNC fallback mechanism"
nohup terminator --maximise >/dev/null 2>&1 &

# Verify launch
sleep 3
if pgrep -f "terminator" >/dev/null; then
    log_fallback "✓ Fallback Terminator launch successful"
else
    log_fallback "✗ Fallback Terminator launch failed"
fi
FALLBACK_EOF

chmod +x /tmp/terminator_fallback_checker.sh

# Start fallback checker in background
/tmp/terminator_fallback_checker.sh &

wait