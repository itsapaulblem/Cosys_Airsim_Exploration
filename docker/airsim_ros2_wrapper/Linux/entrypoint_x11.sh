#!/bin/bash

# ============================================
# X11 Forwarding Entrypoint (Lightweight)
# No VNC server - uses host X11 display
# ============================================

set -e

# Color codes for enhanced output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_header() {
    echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║       AirSim ROS2 X11-Forwarding Container Starting         ║${NC}"
    echo -e "${BLUE}║              Native Linux Performance Mode                   ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_header

# Runtime line ending conversion for mounted files (fix Windows CRLF issues)
print_info "Converting line endings for mounted files..."
dos2unix /launch_ros2_system.sh /debug_airsim_connection.sh 2>/dev/null || true
dos2unix /airsim_ros2_ws/scripts/*.sh 2>/dev/null || true

# ============================================
# X11 Connection Validation
# ============================================

print_info "Validating X11 forwarding configuration..."

# Check DISPLAY environment variable
if [ -z "$DISPLAY" ]; then
    print_error "DISPLAY environment variable not set!"
    print_error "Please ensure DISPLAY is exported in docker-compose.yml"
    print_error "Example: DISPLAY=\${DISPLAY:-:0}"
    exit 1
fi

print_info "DISPLAY set to: $DISPLAY"

# Check X11 socket exists
X11_SOCKET="/tmp/.X11-unix/X${DISPLAY##*:}"
if [ ! -S "$X11_SOCKET" ]; then
    print_warning "X11 socket not found at: $X11_SOCKET"
    print_info "Listing available X11 sockets:"
    ls -la /tmp/.X11-unix/ || echo "No X11 sockets found"
fi

# Check XAUTHORITY file
if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
    print_info "XAUTHORITY file found: $XAUTHORITY"
else
    print_warning "XAUTHORITY file not found or not set"
    print_info "Attempting to merge .Xauthority if available..."
    if [ -f "/tmp/.Xauthority" ]; then
        xauth merge /tmp/.Xauthority 2>/dev/null || true
        print_info "Merged /tmp/.Xauthority"
    fi
fi

# Test X11 connection
print_info "Testing X11 connection..."
if xdpyinfo > /dev/null 2>&1; then
    print_success "✅ X11 connection validated successfully!"

    # Display connection details
    X11_INFO=$(xdpyinfo | head -5)
    echo -e "${BLUE}X11 Connection Details:${NC}"
    echo "$X11_INFO" | sed 's/^/  /'
    echo ""
else
    print_error "❌ X11 connection test failed!"
    echo ""
    print_info "Troubleshooting steps:"
    echo "  1. Ensure xhost +local:docker is run on host"
    echo "  2. Verify DISPLAY is correct: echo \$DISPLAY (on host)"
    echo "  3. Check X11 socket is mounted: ls -la /tmp/.X11-unix/"
    echo "  4. Verify XAUTHORITY is mounted correctly"
    echo ""
    print_warning "Container will continue, but GUI applications may not work"
fi

# ============================================
# GPU Acceleration Check
# ============================================

print_info "Checking GPU acceleration for RViz2..."

if command -v glxinfo &> /dev/null; then
    GLX_INFO=$(glxinfo 2>&1 | grep -i "OpenGL renderer" || echo "GPU info unavailable")
    if echo "$GLX_INFO" | grep -qi "llvmpipe\|software"; then
        print_warning "Software rendering detected (no GPU acceleration)"
        print_info "For better RViz2 performance, ensure:"
        echo "  - GPU device is mounted: devices: /dev/dri:/dev/dri"
        echo "  - User has permission to access /dev/dri"
    else
        print_success "✅ GPU acceleration available"
        echo -e "${BLUE}  ${GLX_INFO}${NC}"
    fi
else
    print_warning "glxinfo not available - cannot check GPU status"
fi

echo ""

# ============================================
# Dynamic User Creation & Permission Fixing
# ============================================

print_info "Setting up user and permissions..."

# Get current user info from docker-compose user: directive
CURRENT_UID=$(id -u)
CURRENT_GID=$(id -g)
CURRENT_USER=$(whoami)

print_info "Running as UID=$CURRENT_UID, GID=$CURRENT_GID, USER=$CURRENT_USER"

# If running as root, we need to fix things
if [ "$CURRENT_UID" = "0" ]; then
    print_warning "Running as root - this is unexpected"
    print_info "Attempting to switch to UID 1000..."

    # Create user if doesn't exist
    if ! id 1000 &>/dev/null; then
        useradd -m -s /bin/bash -u 1000 containeruser
        usermod -aG sudo,video containeruser
        echo 'containeruser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
    fi

    # Fix workspace ownership
    chown -R 1000:1000 /airsim_ros2_ws

    # Re-exec as user 1000
    exec su - $(id -un 1000) -c "cd /airsim_ros2_ws && bash $0"
fi

# Ensure user has video group for GPU access
if ! groups | grep -q video; then
    print_warning "User not in video group - GPU acceleration may not work"
    print_info "Attempting to add user to video group..."
    sudo usermod -aG video $(whoami) 2>/dev/null || true
fi

# ============================================
# ROS2 Workspace Setup
# ============================================

print_info "Setting up ROS2 workspace..."
cd /airsim_ros2_ws

# Fix workspace directory permissions
print_info "Fixing workspace permissions..."
if [ -w /airsim_ros2_ws ]; then
    mkdir -p log build install 2>/dev/null || true
else
    # Need sudo to create directories
    sudo mkdir -p /airsim_ros2_ws/{log,build,install} 2>/dev/null || true
    sudo chown -R $CURRENT_UID:$CURRENT_GID /airsim_ros2_ws/{log,build,install,src} 2>/dev/null || true
    sudo chmod -R u+w /airsim_ros2_ws/{log,build,install} 2>/dev/null || true
fi

# Copy development aliases from dockeruser to current user if different
if [ -f /home/dockeruser/.bashrc ] && [ "$CURRENT_USER" != "dockeruser" ]; then
    print_info "Copying development environment to $CURRENT_USER..."

    # Create user home if it doesn't exist
    if [ ! -d "$HOME" ]; then
        sudo mkdir -p "$HOME"
        sudo chown $CURRENT_UID:$CURRENT_GID "$HOME"
    fi

    # Copy bashrc if user doesn't have one
    if [ ! -f "$HOME/.bashrc" ]; then
        cp /home/dockeruser/.bashrc "$HOME/.bashrc" 2>/dev/null || \
        sudo cp /home/dockeruser/.bashrc "$HOME/.bashrc"
        sudo chown $CURRENT_UID:$CURRENT_GID "$HOME/.bashrc" 2>/dev/null || true
    fi
fi

print_success "✅ User and permissions configured"

# Source ROS2 environment
print_info "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# ============================================
# Workspace Build (if AUTO_BUILD enabled)
# ============================================

if [ "${AUTO_BUILD:-false}" = "true" ]; then
    print_info "AUTO_BUILD enabled - building workspace..."
    echo ""

    # Ensure we have write permissions before building
    if [ ! -w /airsim_ros2_ws/log ]; then
        print_warning "Log directory not writable - fixing permissions..."
        sudo chown -R $(id -u):$(id -g) /airsim_ros2_ws/{log,build,install} 2>/dev/null || true
    fi

    # Build with error handling
    print_info "Building airsim_interfaces..."
    if colcon build --packages-select airsim_interfaces \
        --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | \
        grep -E "(Finished|Starting|Summary|packages)"; then
        print_success "✅ airsim_interfaces built successfully"
    else
        print_error "Failed to build airsim_interfaces"
    fi

    print_info "Building airsim_ros_pkgs..."
    if colcon build --packages-select airsim_ros_pkgs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | \
        grep -E "(Finished|Starting|Summary|packages)"; then
        print_success "✅ airsim_ros_pkgs built successfully"
    else
        print_error "Failed to build airsim_ros_pkgs"
    fi

    # Source if successful
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        print_success "✅ Workspace built and sourced successfully!"
    else
        print_error "Build failed - install/setup.bash not found"
        print_warning "You can rebuild manually with: colcon build"
    fi
elif [ -f install/setup.bash ]; then
    source install/setup.bash
    print_success "✅ Existing workspace sourced"
else
    print_warning "Workspace not built - run 'build' alias or colcon build"
fi

echo ""

# ============================================
# Container Ready
# ============================================

print_success "╔══════════════════════════════════════════════════════════════╗"
print_success "║              Container Ready - X11 Forwarding Active         ║"
print_success "╚══════════════════════════════════════════════════════════════╝"
echo ""

print_info "Available Commands:"
echo -e "  ${BLUE}ros2-system start multi${NC}   - Launch ROS2 multi-node architecture"
echo -e "  ${BLUE}rviz2${NC}                     - Launch RViz2 visualization"
echo -e "  ${BLUE}ros2 node list${NC}            - List active ROS2 nodes"
echo -e "  ${BLUE}ros2 topic list${NC}           - List available topics"
echo -e "  ${BLUE}build${NC}                     - Rebuild workspace"
echo -e "  ${BLUE}test_x11${NC}                  - Test X11 connection (xclock)"
echo ""

print_info "Quick Start:"
echo -e "  ${YELLOW}1.${NC} Launch ROS2 system:  ${BLUE}ros2-system start multi${NC}"
echo -e "  ${YELLOW}2.${NC} Open RViz2:          ${BLUE}rviz2${NC}"
echo -e "  ${YELLOW}3.${NC} Monitor nodes:       ${BLUE}ros2 node list${NC}"
echo ""

# Add convenience test function to user's environment
if [ "$(id -u)" != "0" ]; then
    # Running as user - add to current session
    alias test_x11='xclock -digital -update 1'
    alias test_gpu='glxinfo | grep -i "opengl renderer"'
    alias test_ros2='ros2 node list'
fi

# ============================================
# Keep Container Running
# ============================================

# Execute command if provided, otherwise start interactive shell
if [ $# -eq 0 ]; then
    print_info "Starting interactive shell..."
    exec /bin/bash
else
    print_info "Executing command: $@"
    exec "$@"
fi
