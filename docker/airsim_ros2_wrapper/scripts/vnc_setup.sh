#!/bin/bash
# VNC setup and configuration script
# Handles VNC server initialization and desktop environment setup

set -e

echo "========================================"
echo "VNC Server Configuration"
echo "========================================"

# Configuration variables
VNC_USER=${USER:-"Aortz"}
VNC_PORT=${VNC_PORT:-"5901"}
VNC_DISPLAY=${DISPLAY:-":1"}
VNC_RESOLUTION=${RESOLUTION:-"1920x1080"}
VNC_PASSWORD=${PASSWD:-"ubuntu"}

echo "User: $VNC_USER"
echo "Display: $VNC_DISPLAY"
echo "Port: $VNC_PORT"
echo "Resolution: $VNC_RESOLUTION"
echo

# Function to check if VNC is running
check_vnc_status() {
    if pgrep -f "X.*vnc.*$VNC_DISPLAY" > /dev/null; then
        echo "✅ VNC server is running on display $VNC_DISPLAY"
        return 0
    else
        echo "❌ VNC server is not running"
        return 1
    fi
}

# Function to start VNC server
start_vnc() {
    echo "🚀 Starting VNC server..."
    
    # Ensure VNC directory exists
    mkdir -p /home/$VNC_USER/.vnc
    
    # Kill any existing VNC sessions
    vncserver -kill $VNC_DISPLAY 2>/dev/null || true
    
    # Wait a moment for cleanup
    sleep 2
    
    # Start VNC server
    vncserver $VNC_DISPLAY \
        -geometry $VNC_RESOLUTION \
        -localhost no \
        -alwaysshared \
        -SecurityTypes VncAuth
    
    # Check if started successfully
    sleep 3
    if check_vnc_status; then
        echo "✅ VNC server started successfully"
        echo "🌐 Connect to: localhost:$VNC_PORT"
        echo "🔑 Password: $VNC_PASSWORD"
    else
        echo "❌ Failed to start VNC server"
        return 1
    fi
}

# Function to stop VNC server
stop_vnc() {
    echo "🛑 Stopping VNC server..."
    vncserver -kill $VNC_DISPLAY 2>/dev/null || echo "No VNC server running"
}

# Function to restart VNC server
restart_vnc() {
    echo "🔄 Restarting VNC server..."
    stop_vnc
    sleep 2
    start_vnc
}

# Function to show VNC logs
show_logs() {
    echo "📋 VNC Server Logs:"
    echo "===================="
    if [ -f "/home/$VNC_USER/.vnc/*$VNC_DISPLAY.log" ]; then
        tail -20 /home/$VNC_USER/.vnc/*$VNC_DISPLAY.log
    else
        echo "No log files found"
    fi
}

# Main script logic
case "${1:-start}" in
    "start")
        start_vnc
        ;;
    "stop")
        stop_vnc
        ;;
    "restart")
        restart_vnc
        ;;
    "status")
        check_vnc_status
        ;;
    "logs")
        show_logs
        ;;
    "help"|"-h"|"--help")
        echo "Usage: $0 {start|stop|restart|status|logs|help}"
        echo
        echo "Commands:"
        echo "  start   - Start VNC server (default)"
        echo "  stop    - Stop VNC server"
        echo "  restart - Restart VNC server"
        echo "  status  - Check VNC server status"
        echo "  logs    - Show VNC server logs"
        echo "  help    - Show this help message"
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac

echo "========================================"