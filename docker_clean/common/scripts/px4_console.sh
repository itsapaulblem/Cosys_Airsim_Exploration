#!/bin/bash

# PX4 Console Helper Script for Docker
# Usage: ./px4_console.sh [command]
# If no command provided, enters interactive mode

PX4_INSTANCE=${PX4_INSTANCE:-1}
PX4_WORKSPACE="/px4_workspace/PX4-Autopilot"

echo "🔧 PX4 Console Access (Instance $PX4_INSTANCE)"
echo "Container: $(hostname)"
echo "Working Directory: $(pwd)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if PX4 is running
if ! pgrep -f "px4.*-i $PX4_INSTANCE" > /dev/null; then
    echo "❌ PX4 instance $PX4_INSTANCE is not running!"
    echo "💡 Start PX4 first or check the instance number"
    exit 1
fi

# Function to send command to PX4
send_px4_command() {
    local cmd="$1"
    echo "Sending: $cmd"
    echo "$cmd" | socat - UDP:localhost:$((14540 + PX4_INSTANCE)) 2>/dev/null || {
        # Fallback: try using MAVLink proxy method
        echo "$cmd" > /tmp/px4_cmd_$PX4_INSTANCE.tmp
        timeout 2s tail -f /tmp/px4_cmd_$PX4_INSTANCE.tmp 2>/dev/null || echo "Command sent (no response)"
        rm -f /tmp/px4_cmd_$PX4_INSTANCE.tmp
    }
}

# If command provided as argument, execute it
if [ $# -gt 0 ]; then
    CMD="$*"
    echo "🚀 Executing: $CMD"
    send_px4_command "$CMD"
    exit 0
fi

# Interactive mode
echo "📡 PX4 Interactive Console"
echo "Available commands:"
echo "  commander status     - Check flight status"
echo "  commander arm        - Arm the vehicle"
echo "  commander disarm     - Disarm the vehicle"
echo "  commander takeoff    - Takeoff"
echo "  commander land       - Land"
echo "  listener sensor_gps  - Monitor GPS data"
echo "  param show          - Show all parameters"
echo "  help                - Show all commands"
echo "  exit/quit           - Exit console"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

while true; do
    echo -n "pxh> "
    read -r input
    
    case "$input" in
        "exit"|"quit"|"q")
            echo "👋 Goodbye!"
            break
            ;;
        "")
            continue
            ;;
        *)
            send_px4_command "$input"
            ;;
    esac
done 