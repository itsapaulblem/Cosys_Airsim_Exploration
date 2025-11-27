#!/bin/bash
set -e
<<<<<<< HEAD
echo "=== Starting AirSim ROS2 Wrapper ==="
=======
echo "=== AirSim ROS2 Multi-Node System Helper ==="
echo "Enhanced 2-tab Terminator auto-launches with embedded commands!"
echo ""
echo "How to use this system:"
echo "   1. Access VNC at localhost:5901"
echo "   2. Terminator should auto-launch with 2 tabs:"
echo "      • Tab 1: Drone Monitoring (3 drone terminals)"
echo "      • Tab 2: ROS2 Development (auto-runs launch + RViz)"
echo ""
echo "Commands auto-executed in Terminator Tab 2:"
echo "   • Top terminal: ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true"
echo "   • Bottom terminal: rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz"
echo ""
echo "NEW: Manual Terminator Launch Available!"
echo "   Run: LAUNCH_MODE=terminator $0 (from VNC environment)"
echo ""
echo "This script provides manual execution and enhanced Terminator control"
echo ""

>>>>>>> main
source /opt/ros/humble/setup.bash

# Check if workspace is built
if [ ! -f "/airsim_ros2_ws/install/setup.bash" ]; then
    echo "⚠️  Workspace not built yet. Building now..."
    echo "This is normal for first-time setup or when using volume mounting."
    cd /airsim_ros2_ws
    
    # Set AIRSIM_ROOT for the build process
    export AIRSIM_ROOT=/airsim_ros2_ws
    
    # Build the workspace
    echo "Building airsim_interfaces..."
    colcon build --packages-select airsim_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    echo "Building airsim_ros_pkgs..."
    colcon build --packages-select airsim_ros_pkgs --cmake-args -DCMAKE_BUILD_TYPE=Release -DAIRSIM_ROOT=/airsim_ros2_ws
    
    echo "✓ Workspace built successfully!"
fi

source /airsim_ros2_ws/install/setup.bash

<<<<<<< HEAD
=======
# Multi-node configuration
>>>>>>> main
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export AIRSIM_HOST_IP=${AIRSIM_HOST_IP:-host.docker.internal}
export AIRSIM_HOST_PORT=${AIRSIM_HOST_PORT:-41451}
export LAUNCH_RVIZ=${LAUNCH_RVIZ:-false}
<<<<<<< HEAD

echo "Configuration: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Host=$AIRSIM_HOST_IP:$AIRSIM_HOST_PORT, RViz=$LAUNCH_RVIZ"

if timeout 5 bash -c "echo >/dev/tcp/$AIRSIM_HOST_IP/$AIRSIM_HOST_PORT" 2>/dev/null; then
    echo "✓ Connected to AirSim API"
else
    echo "✗ Cannot connect to AirSim at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
    echo "Ensure AirSim is running with ApiServerEndpoint: '0.0.0.0:41451'"
    exit 1
fi

if [ "$LAUNCH_RVIZ" = "true" ]; then
    echo "🚀 Launching AirSim node with RViz2..."
    echo "📊 Point cloud retention tips:"
    echo "   - In RViz2: Add -> By topic -> /airsim_node/Drone1/lidar/PointCloud2"
    echo "   - Set History Policy to 'Keep All' or increase History Size"
    echo "   - Set Fixed Frame to 'odom' or 'world'"
    echo "   - Adjust Decay Time to retain points longer"
    
    ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=$AIRSIM_HOST_IP host_port:=$AIRSIM_HOST_PORT enable_api_control:=true output:=screen &
    sleep 5 && ros2 launch airsim_ros_pkgs rviz.launch.py &
    wait
else
    echo "🚀 Launching AirSim node..."
    ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=$AIRSIM_HOST_IP host_port:=$AIRSIM_HOST_PORT enable_api_control:=true output:=screen
fi 
=======
export LAUNCH_MODE=${LAUNCH_MODE:-multi}  # multi, legacy, or custom
export ENABLE_COORDINATION=${ENABLE_COORDINATION:-true}
export RPC_TIMEOUT=${RPC_TIMEOUT:-10.0}

echo "Ultra-Clean Multi-Node Configuration:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   AirSim Server: $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
echo "   Launch Mode: $LAUNCH_MODE"
echo "   Coordination Node: $ENABLE_COORDINATION"
echo "   RPC Timeout: ${RPC_TIMEOUT}s"
echo "   RViz2: $LAUNCH_RVIZ"

# Enhanced connectivity check with RPC validation
echo "Testing AirSim connectivity..."
if timeout 5 bash -c "echo >/dev/tcp/$AIRSIM_HOST_IP/$AIRSIM_HOST_PORT" 2>/dev/null; then
    echo "✓ TCP connection to AirSim API successful"
    
    # Test RPC communication (requires Python client)
    if command -v python3 >/dev/null && python3 -c "
import socket, time
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(3)
try:
    sock.connect(('$AIRSIM_HOST_IP', $AIRSIM_HOST_PORT))
    print('✓ RPC socket connection verified')
except Exception as e:
    print('⚠ TCP works but RPC may have issues:', str(e))
finally:
    sock.close()
" 2>/dev/null; then
        echo "✓ RPC communication validated"
    else
        echo "⚠ RPC validation failed - will attempt launch anyway"
    fi
else
    echo "✗ Cannot connect to AirSim at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
    echo "Troubleshooting tips:"
    echo "   - Ensure AirSim is running with ApiServerEndpoint: '0.0.0.0:41451'"
    echo "   - For Windows AirSim + Docker: Check firewall allows port 41451"
    echo "   - Verify host.docker.internal resolves correctly"
    exit 1
fi

# Enhanced Terminator configuration application function
apply_enhanced_terminator_config() {
    local enhanced_config="/etc/terminator-config-enhanced"
    local user_config="$HOME/.config/terminator/config"

    echo "Applying enhanced Terminator configuration..."

    # Check if enhanced config exists
    if [ -f "$enhanced_config" ]; then
        # Ensure config directory exists
        mkdir -p "$(dirname "$user_config")"

        # Copy enhanced config, handling permissions
        if cp "$enhanced_config" "$user_config" 2>/dev/null; then
            echo "✓ Enhanced config copied successfully"
        else
            echo "⚠ Failed to copy config, attempting with sudo..."
            if sudo cp "$enhanced_config" "$user_config" 2>/dev/null; then
                sudo chown $USER:$USER "$user_config" 2>/dev/null || true
                echo "✓ Enhanced config copied with elevated permissions"
            else
                echo "✗ Failed to copy enhanced config"
                return 1
            fi
        fi

        # Set proper permissions
        chmod 644 "$user_config" 2>/dev/null || true

        # Verify config was applied correctly
        local config_size=$(wc -c < "$user_config" 2>/dev/null || echo "0")
        if [ "$config_size" -gt 1000 ]; then
            echo "✓ Enhanced config applied successfully (${config_size} bytes)"

            # Validate critical components
            if grep -q "Drone Monitoring, ROS2 Development" "$user_config" 2>/dev/null; then
                echo "✓ 2-tab layout configuration verified"
                return 0
            else
                echo "⚠ 2-tab layout not found in applied config"
                return 1
            fi
        else
            echo "✗ Enhanced config appears too small (${config_size} bytes)"
            return 1
        fi
    else
        echo "✗ Enhanced config file not found at $enhanced_config"
        echo "   Using fallback basic configuration"
        return 1
    fi
}

# Smart Terminator launcher with single instance detection
launch_terminator_with_config() {
    echo "Starting enhanced Terminator with multi-drone development layout..."

    # Single instance check
    if pgrep -f "terminator" >/dev/null; then
        echo "✓ Terminator is already running"
        echo "   Attempting to bring existing instance to focus..."

        # Try to bring existing Terminator to focus
        if command -v wmctrl >/dev/null 2>&1; then
            wmctrl -a "terminator" 2>/dev/null || true
        fi

        echo "   If not visible, check VNC desktop or run 'pkill terminator' to restart"
        return 0
    fi

    # Apply enhanced configuration first
    if ! apply_enhanced_terminator_config; then
        echo "⚠ Enhanced config application failed, proceeding with fallback"
    fi

    # Set up optimal environment for Terminator
    export TERM=xterm-256color
    export COLORTERM=truecolor

    # Ensure DISPLAY is set for VNC environment
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:1
        echo "✓ Set DISPLAY to :1 for VNC environment"
    fi

    # Change to workspace directory
    cd /airsim_ros2_ws 2>/dev/null || cd /

    # Source ROS2 environment for Terminator session
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source /airsim_ros2_ws/install/setup.bash 2>/dev/null || true

    echo "✓ Environment configured for Terminator launch"
    echo "✓ Using enhanced 2-tab configuration (Drone Monitoring + ROS2 Development)"

    # Launch Terminator with enhanced settings (no layout override to use config)
    echo "Launching Terminator..."

    if [ -n "$DISPLAY" ] && xdpyinfo >/dev/null 2>&1; then
        # VNC/X11 environment - launch in background
        nohup terminator --maximise >/dev/null 2>&1 &
        terminator_pid=$!

        # Give Terminator time to start
        sleep 3

        # Verify launch success
        if pgrep -f "terminator" >/dev/null; then
            echo "✓ Terminator launched successfully with enhanced configuration"
            echo "✓ 2-tab layout: 'Drone Monitoring' + 'ROS2 Development'"
            echo "✓ Access via VNC at localhost:5901"
            return 0
        else
            echo "✗ Terminator failed to start in VNC environment"
            return 1
        fi
    else
        echo "✗ No X11 display available - Terminator requires VNC environment"
        echo "   Ensure you're accessing via VNC viewer at localhost:5901"
        return 1
    fi
}

# Manual launch options (since Terminator auto-executes the commands)
case "$LAUNCH_MODE" in
    "multi")
        echo "Manual Multi-Node Launch (for terminal use)"
        echo "Note: This runs automatically in Terminator Tab 2"
        echo "Launching Mission Mode..."

        cd /airsim_ros2_ws
        source install/setup.bash 2>/dev/null || true

        if [ "$LAUNCH_RVIZ" = "true" ]; then
            echo "Launching ROS2 + RViz..."
            ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
                host_ip:=$AIRSIM_HOST_IP \
                host_port:=$AIRSIM_HOST_PORT \
                enable_coordination:=$ENABLE_COORDINATION \
                rpc_timeout:=$RPC_TIMEOUT \
                mission_mode:=true &
            sleep 8
            rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz &
            wait
        else
            ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
                host_ip:=$AIRSIM_HOST_IP \
                host_port:=$AIRSIM_HOST_PORT \
                enable_coordination:=$ENABLE_COORDINATION \
                rpc_timeout:=$RPC_TIMEOUT \
                mission_mode:=true
        fi
        ;;

    "legacy")
        echo "Manual Legacy Launch (for terminal use)"
        echo "Note: Multi-node with mission mode runs automatically in Terminator"

        cd /airsim_ros2_ws
        source install/setup.bash 2>/dev/null || true

        if [ "$LAUNCH_RVIZ" = "true" ]; then
            ros2 launch airsim_ros_pkgs airsim_node.launch.py \
                host_ip:=$AIRSIM_HOST_IP \
                host_port:=$AIRSIM_HOST_PORT \
                enable_api_control:=true \
                output:=screen &
            sleep 5
            rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz &
            wait
        else
            ros2 launch airsim_ros_pkgs airsim_node.launch.py \
                host_ip:=$AIRSIM_HOST_IP \
                host_port:=$AIRSIM_HOST_PORT \
                enable_api_control:=true \
                output:=screen
        fi
        ;;

    "terminator")
        echo "Enhanced Terminator Launch Mode"
        echo "Launching Terminator with 2-tab multi-drone development layout..."
        echo ""

        # Check if we're in a VNC environment
        if [ -z "$DISPLAY" ]; then
            echo "⚠ No DISPLAY set - this mode requires VNC access"
            echo "   Access the container via VNC at localhost:5901"
            echo "   Then run: LAUNCH_MODE=terminator $0"
            exit 1
        fi

        # Launch Terminator with enhanced configuration
        if launch_terminator_with_config; then
            echo ""
            echo "✓ Terminator launch completed successfully"
            echo "✓ Enhanced 2-tab layout ready for multi-drone development"
            echo ""
            echo "Terminator Layout Overview:"
            echo "   • Tab 1: Drone Monitoring - 3 horizontal terminals for drone management"
            echo "   • Tab 2: ROS2 Development - Split terminal for launch + RViz"
            echo ""
            echo "To auto-execute commands in Terminator tabs, use the embedded terminal commands"
            echo "Or run manual launches with LAUNCH_MODE=multi or LAUNCH_MODE=legacy"
        else
            echo ""
            echo "✗ Terminator launch failed"
            echo "   Check VNC connection and enhanced config availability"
            echo "   Try: pkill terminator && LAUNCH_MODE=terminator $0"
            exit 1
        fi
        ;;

    "info"|"custom"|*)
        echo "System Information:"
        echo "   Enhanced 2-tab Terminator auto-launches on container start"
        echo "   Access via VNC: localhost:5901"
        echo "   Commands run automatically in Terminator Tab 2:"
        echo "      • ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true"
        echo "      • rviz2 -d /airsim_ros2_ws/src/airsim_ros_pkgs/rviz/lidar_visualization.rviz"
        echo ""
        echo "Available manual aliases:"
        echo "   launch-mission  - Manual ROS2 launch with mission mode"
        echo "   launch-rviz     - Manual RViz2 launch"
        echo "   monitor-drones  - Monitor drone topics"
        echo "   terminator-status - Check Terminator configuration"
        echo ""
        echo "Available launch modes:"
        echo "   LAUNCH_MODE=multi $0      - Manual multi-node ROS2 launch"
        echo "   LAUNCH_MODE=legacy $0     - Manual legacy single-node launch"
        echo "   LAUNCH_MODE=terminator $0 - Launch enhanced Terminator with 2-tab layout"
        echo ""
        echo "Enhanced Terminator Features:"
        echo "   • Automatic configuration with 2-tab multi-drone layout"
        echo "   • Single instance detection (prevents duplicates)"
        echo "   • VNC-optimized environment setup"
        echo "   • Integration with ROS2 workspace and development aliases"
        exit 0
        ;;
esac 
>>>>>>> main
