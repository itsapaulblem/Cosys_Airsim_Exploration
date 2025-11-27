#!/bin/bash
# Reliable Terminator autostart wrapper with volume persistence bypass and config override

LOGFILE="/home/Aortz/.terminator-autostart.log"
MAX_RETRIES=8
RETRY_DELAY=2
ENHANCED_CONFIG="/etc/terminator-config-enhanced"
USER_CONFIG="/home/Aortz/.config/terminator/config"

log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOGFILE"
}

override_terminator_config() {
    log_message "Applying enhanced Terminator configuration override..."

    # Check if enhanced config exists
    if [ -f "$ENHANCED_CONFIG" ]; then
        # Ensure config directory exists
        mkdir -p "$(dirname "$USER_CONFIG")"

        # Copy enhanced config, bypassing volume persistence
        cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null || {
            log_message "Failed to copy enhanced config, attempting with sudo..."
            sudo cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null || {
                log_message "Failed to override config even with sudo"
                return 1
            }
        }

        # Set proper ownership and permissions
        chown Aortz:Aortz "$USER_CONFIG" 2>/dev/null || sudo chown Aortz:Aortz "$USER_CONFIG"
        chmod 644 "$USER_CONFIG"

        # Verify config was applied
        local config_size=$(wc -c < "$USER_CONFIG" 2>/dev/null || echo "0")
        if [ "$config_size" -gt 1000 ]; then
            log_message "Enhanced config applied successfully (${config_size} bytes)"
            return 0
        else
            log_message "Config appears too small (${config_size} bytes), may not have applied correctly"
            return 1
        fi
    else
        log_message "Enhanced config file not found at $ENHANCED_CONFIG"
        return 1
    fi
}

check_environment() {
    # Check if DISPLAY is set
    if [ -z "$DISPLAY" ]; then
        log_message "DISPLAY not set, attempting to set to :1"
        export DISPLAY=:1
    fi

    # Detect if running from XFCE autostart (less strict checks)
    local is_autostart=false
    if [ -n "$XDG_CURRENT_DESKTOP" ] && [ "$XDG_CURRENT_DESKTOP" = "XFCE" ]; then
        is_autostart=true
        log_message "Detected XFCE autostart context - using relaxed environment checks"
    fi

    # Check if X11 is responsive (with retries for autostart)
    local x11_ready=false
    local x11_attempts=1
    if [ "$is_autostart" = "true" ]; then
        x11_attempts=3  # More attempts for autostart
    fi

    for i in $(seq 1 $x11_attempts); do
        if xdpyinfo >/dev/null 2>&1; then
            x11_ready=true
            break
        fi
        if [ $i -lt $x11_attempts ]; then
            log_message "X11 not ready, waiting... (attempt $i/$x11_attempts)"
            sleep 2
        fi
    done

    if [ "$x11_ready" = "false" ]; then
        log_message "X11 display not ready after $x11_attempts attempts"
        return 1
    fi

    # For autostart, don't require XFCE session to be fully ready
    if [ "$is_autostart" = "false" ]; then
        # Check if XFCE4 desktop is running (only for manual launch)
        if ! pgrep -x "xfce4-session" >/dev/null; then
            log_message "XFCE4 session not ready"
            return 1
        fi
    else
        log_message "Skipping XFCE session check for autostart context"
    fi

    log_message "Environment validation passed"
    return 0
}

check_single_instance() {
    # Check if Terminator is already running
    if pgrep -f "terminator" >/dev/null; then
        log_message "Terminator already running - single instance check passed"
        return 1
    fi

    # Check for lockfile to prevent race conditions
    local lockfile="/tmp/terminator-autostart.lock"
    if [ -f "$lockfile" ]; then
        local lock_age=$(($(date +%s) - $(stat -c %Y "$lockfile" 2>/dev/null || echo 0)))
        if [ $lock_age -lt 30 ]; then
            log_message "Recent launch lockfile detected (${lock_age}s old) - preventing duplicate launch"
            return 1
        else
            log_message "Stale lockfile detected (${lock_age}s old) - removing and proceeding"
            rm -f "$lockfile"
        fi
    fi

    # Create lockfile
    touch "$lockfile"
    log_message "Single instance check passed - proceeding with launch"
    return 0
}

launch_terminator() {
    log_message "Attempting to launch Terminator with enhanced configuration..."

    # Single instance check
    if ! check_single_instance; then
        log_message "Terminator launch skipped - instance already exists or recent launch detected"
        return 0
    fi

    # Set up environment
    export TERM=xterm-256color
    export COLORTERM=truecolor
    cd /airsim_ros2_ws

    # Source ROS2 environment
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true

    # Launch Terminator using enhanced config (no --layout override)
    nohup terminator --maximise >/dev/null 2>&1 &

    # Wait a moment and check if it started
    sleep 3
    if pgrep -f "terminator" >/dev/null; then
        log_message "Terminator launched successfully using enhanced 2-tab configuration"
        # Clean up lockfile after successful launch
        rm -f /tmp/terminator-autostart.lock
        return 0
    else
        log_message "Terminator failed to start"
        rm -f /tmp/terminator-autostart.lock
        return 1
    fi
}

# Main execution with config override and retry logic
log_message "Starting enhanced Terminator autostart process"
log_message "System info: Docker volume persistence bypass enabled"

# Apply enhanced configuration override first
if override_terminator_config; then
    log_message "Configuration override successful, proceeding with launch"
else
    log_message "Configuration override failed, continuing with fallback"
fi

for attempt in $(seq 1 $MAX_RETRIES); do
    log_message "Attempt $attempt of $MAX_RETRIES"

    if check_environment; then
        if launch_terminator; then
            log_message "Enhanced Terminator autostart completed successfully"
            log_message "[SUCCESS] 2-tab multi-drone development environment ready"
            exit 0
        fi
    fi

    if [ $attempt -lt $MAX_RETRIES ]; then
        log_message "[INFO] Retrying in ${RETRY_DELAY} seconds..."
        sleep $RETRY_DELAY
    fi
done

log_message "[ERROR] Terminator autostart failed after $MAX_RETRIES attempts"
log_message "[INFO] Manual launch available: run 'terminator-multi-drone' or 'term'"
log_message "[DEBUG] Check 'terminator-log' for details"
exit 1