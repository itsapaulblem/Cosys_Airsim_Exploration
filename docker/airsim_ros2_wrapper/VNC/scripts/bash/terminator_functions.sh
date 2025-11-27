#!/bin/bash
# Debug Utilities for Terminator Configuration

# Create multi-line functions for complex operations to avoid long aliases
terminator-status() {
    echo "[DEBUG] Enhanced Terminator Status Check:"
    echo ""
    echo "Process Status:"
    pgrep -f terminator || echo "No Terminator processes found"
    echo ""
    echo "[INFO] Configuration Status:"
    [ -f /etc/terminator-config-enhanced ] && echo "Enhanced config available" || echo "Enhanced config missing"
    local CONFIG_SIZE
    CONFIG_SIZE=$(wc -c < /home/Aortz/.config/terminator/config 2>/dev/null || echo "0")
    echo "[INFO] Current config size: ${CONFIG_SIZE} bytes"
    [ "$CONFIG_SIZE" -gt 1000 ] && echo "Enhanced config appears applied" || echo "Basic config detected"
    echo ""
    echo "[INFO] Recent Autostart Log:"
    tail -10 /home/Aortz/.terminator-autostart.log 2>/dev/null || echo "No log file found"
}

terminator-force-config() {
    echo "[INFO] Force applying enhanced 2-tab configuration..."

    local ENHANCED_CONFIG="/etc/terminator-config-enhanced"
    local USER_CONFIG="/home/Aortz/.config/terminator/config"

    # Detailed diagnostic information
    echo "[DEBUG] Configuration diagnostic:"
    echo "   Enhanced config file: $ENHANCED_CONFIG"
    echo "   User config file: $USER_CONFIG"

    if [ -f "$ENHANCED_CONFIG" ]; then
        local enhanced_size=$(wc -c < "$ENHANCED_CONFIG" 2>/dev/null || echo "0")
        local enhanced_lines=$(wc -l < "$ENHANCED_CONFIG" 2>/dev/null || echo "0")
        echo "   [INFO] Enhanced config: ${enhanced_size} bytes, ${enhanced_lines} lines"

        # Check for 2-tab indicators
        if grep -q "ROS2 Development" "$ENHANCED_CONFIG"; then
            echo "   [SUCCESS] Found 'ROS2 Development' tab label"
        else
            echo "   [ERROR] Missing 'ROS2 Development' tab label"
        fi

        if grep -q "ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true" "$ENHANCED_CONFIG"; then
            echo "   [SUCCESS] Found ROS2 launch command"
        else
            echo "   [ERROR] Missing ROS2 launch command"
        fi

        if grep -q "rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz" "$ENHANCED_CONFIG"; then
            echo "   [SUCCESS] Found RViz command"
        else
            echo "   [ERROR] Missing RViz command"
        fi

        # Apply configuration
        mkdir -p ~/.config/terminator
        echo "[INFO] Copying enhanced config..."

        if cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null; then
            echo "   [SUCCESS] Config copied successfully"
        else
            echo "   [WARNING] Copy failed, trying with sudo..."
            sudo cp "$ENHANCED_CONFIG" "$USER_CONFIG" 2>/dev/null || {
                echo "   [ERROR] Failed to copy config even with sudo"
                return 1
            }
        fi

        # Set permissions
        chown Aortz:Aortz "$USER_CONFIG" 2>/dev/null || sudo chown Aortz:Aortz "$USER_CONFIG"
        chmod 644 "$USER_CONFIG"

        # Verify application
        if [ -f "$USER_CONFIG" ]; then
            local user_size=$(wc -c < "$USER_CONFIG" 2>/dev/null || echo "0")
            local user_lines=$(wc -l < "$USER_CONFIG" 2>/dev/null || echo "0")
            echo "   [INFO] Applied config: ${user_size} bytes, ${user_lines} lines"

            if [ "$user_size" = "$enhanced_size" ]; then
                echo "   [SUCCESS] Config sizes match - enhanced config successfully applied"
                return 0
            else
                echo "   [WARNING] Size mismatch: expected ${enhanced_size}, got ${user_size}"
                echo "   [DEBUG] This may indicate volume persistence or permission issues"
                return 1
            fi
        else
            echo "   [ERROR] Config file not found after copy operation"
            return 1
        fi
    else
        echo "   [ERROR] Enhanced config file not found at $ENHANCED_CONFIG"
        echo "   [INFO] Container may need rebuilding"
        return 1
    fi
}

terminator-config-debug() {
    echo "[DEBUG] === Terminator Configuration Debug Report ==="
    echo ""

    local ENHANCED_CONFIG="/etc/terminator-config-enhanced"
    local USER_CONFIG="/home/Aortz/.config/terminator/config"

    # File existence and permissions
    echo "[INFO] File Status:"
    if [ -f "$ENHANCED_CONFIG" ]; then
        local enhanced_size=$(wc -c < "$ENHANCED_CONFIG" 2>/dev/null || echo "0")
        local enhanced_lines=$(wc -l < "$ENHANCED_CONFIG" 2>/dev/null || echo "0")
        echo "   [SUCCESS] Enhanced config exists: ${enhanced_size} bytes, ${enhanced_lines} lines"
        ls -la "$ENHANCED_CONFIG"
    else
        echo "   [ERROR] Enhanced config MISSING at $ENHANCED_CONFIG"
        echo "      [INFO] Container needs rebuilding with: docker-compose build ros2-multi-node"
    fi

    if [ -f "$USER_CONFIG" ]; then
        local user_size=$(wc -c < "$USER_CONFIG" 2>/dev/null || echo "0")
        local user_lines=$(wc -l < "$USER_CONFIG" 2>/dev/null || echo "0")
        echo "   [INFO] User config exists: ${user_size} bytes, ${user_lines} lines"
        ls -la "$USER_CONFIG"
    else
        echo "   [WARNING] User config not found at $USER_CONFIG"
    fi

    # Configuration content analysis
    echo ""
    echo "[DEBUG] Configuration Content Analysis:"

    if [ -f "$USER_CONFIG" ]; then
        echo "   Current user config analysis:"

        # Check for 2-tab vs 4-tab layout
        local tab_count=$(grep -c "labels.*=" "$USER_CONFIG" 2>/dev/null || echo "0")
        if grep -q "Drone Monitoring, ROS2 Development" "$USER_CONFIG"; then
            echo "   [SUCCESS] Found 2-tab layout (Drone Monitoring, ROS2 Development)"
        elif grep -q "Drone Monitoring, System Monitor, ROS2 Development, Git & Files" "$USER_CONFIG"; then
            echo "   [WARNING] Found OLD 4-tab layout - needs updating"
            echo "      [INFO] Run: terminator-force-config"
        else
            echo "   [UNKNOWN] Unknown tab layout detected"
        fi

        # Check for specific ROS2 commands
        if grep -q "ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true" "$USER_CONFIG"; then
            echo "   [SUCCESS] Found ROS2 launch command in config"
        else
            echo "   [ERROR] Missing ROS2 launch command"
        fi

        if grep -q "rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz" "$USER_CONFIG"; then
            echo "   [SUCCESS] Found RViz command in config"
        else
            echo "   [ERROR] Missing RViz command"
        fi

        # Check for VPaned (vertical split) in Tab 2
        if grep -A 10 -B 5 "ROS2 Development" "$USER_CONFIG" | grep -q "VPaned"; then
            echo "   [SUCCESS] Found vertical split (VPaned) for ROS2 Development tab"
        else
            echo "   [ERROR] Missing vertical split in ROS2 Development tab"
        fi
    fi

    # Process status
    echo ""
    echo "[INFO] Process Status:"
    if pgrep -f terminator >/dev/null; then
        echo "   [SUCCESS] Terminator is running"
        pgrep -f terminator | while read pid; do
            ps -p $pid -o pid,cmd --no-headers
        done
    else
        echo "   [ERROR] Terminator is not running"
    fi

    # Volume persistence check
    echo ""
    echo "[DEBUG] Volume Persistence Analysis:"
    echo "   User config directory: $(dirname "$USER_CONFIG")"
    echo "   Directory ownership: $(ls -ld "$(dirname "$USER_CONFIG")" 2>/dev/null || echo "Not found")"

    if [ -f "$USER_CONFIG" ]; then
        local config_age=$(stat -c %Y "$USER_CONFIG" 2>/dev/null || echo "unknown")
        echo "   Config last modified: $(date -d @$config_age 2>/dev/null || echo "unknown")"
    fi

    # Docker volume info
    echo "   [INFO] If size mismatch persists, volume reset may be needed:"
    echo "      docker-compose -f docker-compose-master.yml down"
    echo "      docker volume rm vnc_home"
    echo "      docker-compose -f docker-compose-master.yml --profile integrated up ros2-multi-node"

    echo ""
    echo "[INFO] Recommended Actions:"
    if [ ! -f "$ENHANCED_CONFIG" ]; then
        echo "   1. Rebuild container: docker-compose build ros2-multi-node"
    fi

    if [ -f "$USER_CONFIG" ]; then
        local user_size=$(wc -c < "$USER_CONFIG" 2>/dev/null || echo "0")
        if [ "$user_size" -lt 1000 ]; then
            echo "   2. Apply enhanced config: terminator-force-config"
        fi
    fi

    echo "   3. Restart Terminator: terminator-restart"
    echo "   4. If issues persist: fix-terminator"
}

fix-terminator() {
    echo "[INFO] Comprehensive Terminator fix..."
    echo "Step 1: Killing existing Terminator processes"
    pkill -f terminator
    sleep 2

    echo "Step 2: Applying enhanced configuration"
    terminator-force-config

    echo "Step 3: Launching Terminator with enhanced environment"
    sleep 1
    export DISPLAY=:1
    cd /airsim_ros2_ws
    terminator --layout=default --maximise &

    sleep 3
    echo "Step 4: Verification"
    if pgrep -f terminator >/dev/null; then
        echo "[SUCCESS] Terminator restarted successfully"
    else
        echo "[ERROR] Terminator restart failed"
        echo "[INFO] Try manual debugging: terminator-config-debug"
    fi
}

workspace-status() {
    echo "=== AirSim ROS2 Development Environment ==="
    echo "Workspace: /airsim_ros2_ws"
    echo "ROS Distribution: $ROS_DISTRO"
    if [ -d /airsim_ros2_ws/install ]; then
        echo "Status: Workspace built and ready"
        echo "Use: launch_multi (for multi-node) or launch_mission (for mission-mode)"
    else
        echo "Status: Workspace needs building"
        echo "Use: build (to build workspace)"
    fi
    echo "============================================"
}