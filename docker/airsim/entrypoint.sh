#!/bin/bash
# Cosys-AirSim Container Entrypoint Script

set -e

# Function to print colored output
print_status() {
    echo -e "\033[1;32m[Cosys-AirSim Container]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[Cosys-AirSim Container ERROR]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[Cosys-AirSim Container WARNING]\033[0m $1"
}

# Function to setup display
setup_display() {
    if [ "$DISPLAY_MODE" = "vnc" ]; then
        print_status "Setting up VNC display..."
        export DISPLAY=:1
        Xvfb :1 -screen 0 ${VNC_RESOLUTION:-1920x1080}x24 &
        sleep 2
        fluxbox &
        x11vnc -display :1 -nopw -listen 0.0.0.0 -xkb -forever -shared &
        print_status "VNC server started on port 5900"
    elif [ "$DISPLAY_MODE" = "headless" ]; then
        print_status "Running in headless mode (no display)"
        export DISPLAY=:99
        Xvfb :99 -screen 0 1024x768x24 &
    else
        print_status "Using host display: $DISPLAY"
    fi
}

# Function to wait for network connectivity
wait_for_network() {
    print_status "Waiting for network connectivity..."
    for i in {1..30}; do
        if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
            print_status "Network connectivity established"
            return 0
        fi
        sleep 1
    done
    print_warning "Network connectivity check failed, continuing anyway..."
}

# Function to start AirSim
start_airsim() {
    print_status "Starting Cosys-AirSim..."
    
    # Wait a bit for display to be ready
    sleep 3
    
    # Set environment variables
    export SDL_VIDEODRIVER=x11
    export UE4_ROOT=/home/airsim_user
    
    # Choose binary based on environment
    if [ -x "/home/airsim_user/Blocks/LinuxNoEditor/Blocks.sh" ]; then
        AIRSIM_BINARY="/home/airsim_user/Blocks/LinuxNoEditor/Blocks.sh"
        AIRSIM_DIR="/home/airsim_user/Blocks/LinuxNoEditor"
    else
        print_error "Cosys-AirSim binary not found!"
        exit 1
    fi
    
    # Start AirSim with appropriate parameters
    cd "$AIRSIM_DIR"
    if [ "$DISPLAY_MODE" = "headless" ]; then
        print_status "Starting Cosys-AirSim in headless mode..."
        "$AIRSIM_BINARY" -nullrhi
    else
        print_status "Starting Cosys-AirSim with display..."
        "$AIRSIM_BINARY" -windowed -ResX=${SCREEN_WIDTH:-1280} -ResY=${SCREEN_HEIGHT:-720}
    fi
}

# Function to run test mode
run_tests() {
    print_status "Running Cosys-AirSim connectivity tests..."
    
    # Start AirSim in background
    start_airsim &
    AIRSIM_PID=$!
    
    # Wait for AirSim to start
    sleep 10
    
    # Test Python client connection
    python3 -c "
import cosysairsim as airsim
import time
print('Testing Cosys-AirSim connection...')
try:
    client = airsim.MultirotorClient('localhost')
    client.confirmConnection()
    print('✅ Cosys-AirSim connection successful')
    
    # Test basic API calls
    client.enableApiControl(True)
    state = client.getMultirotorState()
    print(f'✅ Vehicle state retrieved: {state.kinematics_estimated.position}')
    
    print('✅ All tests passed!')
except Exception as e:
    print(f'❌ Test failed: {e}')
    exit(1)
"
    
    # Keep AirSim running if tests pass
    wait $AIRSIM_PID
}

# Function to show help
show_help() {
    echo "Cosys-AirSim Container Entrypoint"
    echo "Usage: $0 [MODE]"
    echo ""
    echo "Modes:"
    echo "  airsim     Start Cosys-AirSim normally (default)"
    echo "  vnc        Start with VNC server"
    echo "  headless   Start without display"
    echo "  test       Run connectivity tests"
    echo "  bash       Open bash shell"
    echo "  help       Show this help"
    echo ""
    echo "Environment Variables:"
    echo "  DISPLAY_MODE        vnc|headless|host (default: host)"
    echo "  VNC_RESOLUTION      VNC resolution (default: 1920x1080)"
    echo "  SCREEN_WIDTH        AirSim window width (default: 1280)"
    echo "  SCREEN_HEIGHT       AirSim window height (default: 720)"
}

# Main execution
print_status "Cosys-AirSim Container Starting..."
print_status "Mode: ${1:-airsim}"
print_status "Display: ${DISPLAY:-not set}"
print_status "Display Mode: ${DISPLAY_MODE:-host}"

# Wait for network
wait_for_network

# Handle different modes
case "${1:-airsim}" in
    "airsim")
        setup_display
        start_airsim
        ;;
    "vnc")
        export DISPLAY_MODE=vnc
        setup_display
        start_airsim
        ;;
    "headless")
        export DISPLAY_MODE=headless
        setup_display
        start_airsim
        ;;
    "test")
        setup_display
        run_tests
        ;;
    "bash")
        print_status "Opening bash shell..."
        exec /bin/bash
        ;;
    "help")
        show_help
        ;;
    *)
        print_error "Unknown mode: $1"
        show_help
        exit 1
        ;;
esac 