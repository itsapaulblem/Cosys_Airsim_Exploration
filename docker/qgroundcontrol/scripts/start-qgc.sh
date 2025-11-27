#!/bin/bash
# QGroundControl Startup Script with VNC/noVNC Support
# Launches X server, VNC, noVNC, and QGroundControl for remote access

set -e

echo "=========================================="
echo "QGroundControl Container Startup"
echo "=========================================="

# Set display resolution (can be overridden via environment variable)
RESOLUTION=${VNC_RESOLUTION:-1920x1080}
VNC_DISPLAY=${DISPLAY:-:1}
VNC_PORT=5900
NOVNC_PORT=6080

# Extract display number from DISPLAY variable
DISPLAY_NUM=$(echo $VNC_DISPLAY | sed 's/^://')

echo "Configuration:"
echo "  Display: $VNC_DISPLAY"
echo "  Resolution: $RESOLUTION"
echo "  VNC Port: $VNC_PORT"
echo "  noVNC Port: $NOVNC_PORT"
echo ""

# Function to cleanup background processes on exit
cleanup() {
    echo "Shutting down services..."
    pkill -f QGroundControl || true
    pkill -f fluxbox || true
    pkill -f x11vnc || true
    pkill -f websockify || true
    pkill -f Xvfb || true
    exit 0
}

trap cleanup SIGTERM SIGINT

# Start Xvfb (X Virtual Frame Buffer)
echo "Starting Xvfb..."
Xvfb $VNC_DISPLAY -screen 0 ${RESOLUTION}x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Wait for X server to be ready
sleep 2

# Verify Xvfb is running
if ! ps -p $XVFB_PID > /dev/null; then
    echo "ERROR: Xvfb failed to start"
    exit 1
fi

echo "Xvfb started successfully (PID: $XVFB_PID)"

# Start Fluxbox window manager
echo "Starting Fluxbox window manager..."
fluxbox -display $VNC_DISPLAY &
FLUXBOX_PID=$!
sleep 2

echo "Fluxbox started (PID: $FLUXBOX_PID)"

# Start x11vnc server
echo "Starting x11vnc server..."
x11vnc -display $VNC_DISPLAY \
    -forever \
    -shared \
    -rfbport $VNC_PORT \
    -rfbauth /home/qgc/.vnc/passwd \
    -nopw \
    -xkb \
    -noxrecord \
    -noxfixes \
    -noxdamage \
    -wait 5 \
    -deferupdate 1 &
X11VNC_PID=$!

# Wait for x11vnc to start
sleep 2

if ! ps -p $X11VNC_PID > /dev/null; then
    echo "ERROR: x11vnc failed to start"
    cleanup
fi

echo "x11vnc started successfully (PID: $X11VNC_PID)"
echo "  VNC accessible at: vnc://localhost:$VNC_PORT"

# Start noVNC (websockify) for web browser access
echo "Starting noVNC (websockify)..."
websockify --web /usr/share/novnc $NOVNC_PORT localhost:$VNC_PORT &
NOVNC_PID=$!

# Wait for noVNC to start
sleep 2

if ! ps -p $NOVNC_PID > /dev/null; then
    echo "WARNING: noVNC failed to start (optional service)"
else
    echo "noVNC started successfully (PID: $NOVNC_PID)"
    echo "  Web interface: http://localhost:$NOVNC_PORT/vnc.html"
fi

echo ""
echo "=========================================="
echo "VNC Services Ready!"
echo "=========================================="
echo ""

# Display MAVLink connection information
echo "MAVLink Connection Info:"
echo "  QGroundControl will listen on UDP port 14550 (default)"
echo "  PX4 instances broadcast to this port automatically"
echo "  Multi-drone support: UDP 14550-14578"
echo ""

# Wait a bit for window manager to stabilize
sleep 2

# Launch QGroundControl
echo "Launching QGroundControl..."
echo ""

# Extract AppImage and run (more reliable than FUSE in containers)
cd /opt/qgroundcontrol

# Method 1: Try to run with FUSE (faster, but may not work in all containers)
if $QGC_APPIMAGE --appimage-help &>/dev/null; then
    echo "Running QGroundControl via FUSE AppImage..."
    DISPLAY=$VNC_DISPLAY $QGC_APPIMAGE &
    QGC_PID=$!
else
    # Method 2: Extract and run (slower first time, but more compatible)
    echo "FUSE unavailable, extracting AppImage..."
    if [ ! -d "/opt/qgroundcontrol/squashfs-root" ]; then
        $QGC_APPIMAGE --appimage-extract >/dev/null 2>&1
    fi
    echo "Running extracted QGroundControl..."
    DISPLAY=$VNC_DISPLAY /opt/qgroundcontrol/squashfs-root/AppRun &
    QGC_PID=$!
fi

sleep 3

if ! ps -p $QGC_PID > /dev/null; then
    echo "ERROR: QGroundControl failed to start"
    cleanup
fi

echo "QGroundControl started successfully (PID: $QGC_PID)"
echo ""
echo "=========================================="
echo "QGroundControl Container Running!"
echo "=========================================="
echo ""
echo "Access Options:"
echo "  1. VNC Client: vnc://localhost:5900 (password: airsim)"
echo "  2. Web Browser: http://localhost:6080/vnc.html"
echo ""
echo "MAVLink Communication:"
echo "  Listening on UDP port 14550 for PX4 connections"
echo "  Ensure PX4 containers are on same Docker network (px4_network)"
echo ""
echo "Logs are available via 'docker logs <container_name>'"
echo ""

# Monitor QGroundControl process - keep container running
echo "Monitoring QGroundControl process..."
echo "Press Ctrl+C to stop the container"
echo ""

# Wait for QGroundControl to exit (or container to receive SIGTERM)
wait $QGC_PID

echo "QGroundControl exited. Shutting down..."
cleanup
