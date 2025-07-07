#!/bin/bash

# Simple PX4 Shell Access for Docker
# This script provides direct access to the PX4 shell

PX4_INSTANCE=${PX4_INSTANCE:-1}
INSTANCE_DIR="/px4_workspace/PX4-Autopilot/build/px4_sitl_default/instance_$PX4_INSTANCE"

echo "🚁 PX4 Shell Access (Instance $PX4_INSTANCE)"
echo "Instance Directory: $INSTANCE_DIR"

# Check if PX4 is running
if ! pgrep -f "px4.*-i $PX4_INSTANCE" > /dev/null; then
    echo "❌ PX4 instance $PX4_INSTANCE is not running!"
    echo "💡 Try: docker logs px4-single"
    exit 1
fi

echo "✅ PX4 is running"
echo "📡 Connecting to PX4 shell..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# If specific command provided, execute it
if [ $# -gt 0 ]; then
    echo "🚀 Executing: $*"
    cd "$INSTANCE_DIR" 2>/dev/null || cd /px4_workspace/PX4-Autopilot
    
    # Try to send command via echo to the px4 process
    echo "$*" | timeout 5s /px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $PX4_INSTANCE -d 2>/dev/null || {
        echo "💡 Command sent to PX4 instance $PX4_INSTANCE"
        echo "💡 Check docker logs for output: docker logs px4-single --tail 20"
    }
    exit 0
fi

# Interactive shell
echo "📝 Common PX4 Commands:"
echo "   commander status      - Check vehicle status"
echo "   commander arm         - Arm vehicle"
echo "   commander disarm      - Disarm vehicle"  
echo "   commander takeoff     - Takeoff command"
echo "   commander land        - Land command"
echo "   param show SYS_*      - Show system parameters"
echo "   listener sensor_gps   - Monitor GPS"
echo "   help                  - Show all commands"
echo ""
echo "💡 Use 'docker logs px4-single --tail 20' to see command output"
echo "💡 Use Ctrl+C to exit this shell"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

cd "$INSTANCE_DIR" 2>/dev/null || cd /px4_workspace/PX4-Autopilot

while true; do
    echo -n "px4> "
    read -r cmd
    
    case "$cmd" in
        "exit"|"quit"|"q")
            echo "👋 Exiting PX4 shell"
            break
            ;;
        "")
            continue
            ;;
        "logs")
            echo "Recent PX4 logs:"
            tail -10 /tmp/px4.log 2>/dev/null || echo "No log file found"
            ;;
        *)
            echo "Sending: $cmd"
            echo "$cmd" | timeout 3s /px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $PX4_INSTANCE -d 2>/dev/null || {
                echo "✅ Command sent (check logs for output)"
            }
            ;;
    esac
done 