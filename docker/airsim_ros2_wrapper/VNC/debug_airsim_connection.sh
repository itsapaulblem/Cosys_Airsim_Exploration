#!/bin/bash

echo "=== AirSim ROS2 Container Debug Script ==="
echo "This script will help diagnose connection and ROS2 wrapper issues"
echo

# Function to print colored messages
print_info() {
    echo -e "\e[32m[INFO]\e[0m $1"
}

print_warning() {
    echo -e "\e[33m[WARNING]\e[0m $1"
}

print_error() {
    echo -e "\e[31m[ERROR]\e[0m $1"
}

print_section() {
    echo
    echo -e "\e[34m=== $1 ===\e[0m"
}

# 1. Check environment variables
print_section "Environment Variables"
echo "ROS_DISTRO: ${ROS_DISTRO:-NOT SET}"
echo "ROS_VERSION: ${ROS_VERSION:-NOT SET}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "AIRSIM_HOST_IP: ${AIRSIM_HOST_IP:-NOT SET}"
echo "AIRSIM_HOST_PORT: ${AIRSIM_HOST_PORT:-NOT SET}"
echo "PYTHONPATH: ${PYTHONPATH:-NOT SET}"

# 2. Test network connectivity to AirSim
print_section "Network Connectivity Test"
AIRSIM_HOST=${AIRSIM_HOST_IP:-host.docker.internal}
AIRSIM_PORT=${AIRSIM_HOST_PORT:-41451}

print_info "Testing connection to AirSim at $AIRSIM_HOST:$AIRSIM_PORT"

# Test if host is reachable
if ping -c 1 "$AIRSIM_HOST" >/dev/null 2>&1; then
    print_info "✓ Host $AIRSIM_HOST is reachable"
else
    print_error "✗ Host $AIRSIM_HOST is NOT reachable"
    print_warning "Try using your actual host IP instead of host.docker.internal"
    print_warning "On Windows, run: ipconfig and use your Ethernet/WiFi IP"
fi

# Test if port is open
if timeout 5 bash -c "echo >/dev/tcp/$AIRSIM_HOST/$AIRSIM_PORT" 2>/dev/null; then
    print_info "✓ Port $AIRSIM_PORT is open on $AIRSIM_HOST"
else
    print_error "✗ Port $AIRSIM_PORT is NOT accessible on $AIRSIM_HOST"
    print_warning "Make sure AirSim is running with ApiServerEndpoint: '0.0.0.0:41451' in settings.json"
fi

# 3. Check ROS2 setup
print_section "ROS2 Environment Check"

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS2 not sourced, attempting to source..."
    source /opt/ros/humble/setup.bash 2>/dev/null || print_error "Failed to source ROS2"
fi

if [ -z "$ROS_VERSION" ]; then
    print_warning "AirSim workspace not sourced, attempting to source..."
    source /airsim_ros2_ws/install/setup.bash 2>/dev/null || print_error "Failed to source AirSim workspace"
fi

# Test ROS2 commands
if command -v ros2 >/dev/null 2>&1; then
    print_info "✓ ros2 command available"
    
    # Test ROS2 daemon
    if ros2 daemon status >/dev/null 2>&1; then
        print_info "✓ ROS2 daemon is running"
    else
        print_warning "ROS2 daemon not running, starting..."
        ros2 daemon start
    fi
    
    # List available packages
    print_info "Available AirSim packages:"
    ros2 pkg list | grep airsim || print_error "No AirSim packages found!"
    
else
    print_error "✗ ros2 command not found"
fi

# 4. Check AirSim interfaces
print_section "AirSim Interfaces Check"

if ros2 interface list | grep -q airsim_interfaces; then
    print_info "✓ AirSim interfaces are available"
    echo "Available AirSim message types:"
    ros2 interface list | grep airsim_interfaces
else
    print_error "✗ AirSim interfaces not found"
    print_warning "AirSim packages may not be built correctly"
fi

# 5. Test launching AirSim node
print_section "AirSim Node Launch Test"

print_info "Testing AirSim node launch (this will timeout after 10 seconds)..."

# Set environment for launch
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export AIRSIM_HOST_IP=${AIRSIM_HOST_IP:-host.docker.internal}
export AIRSIM_HOST_PORT=${AIRSIM_HOST_PORT:-41451}

# Try to launch AirSim node with timeout
timeout 10s ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=$AIRSIM_HOST_IP host_port:=$AIRSIM_HOST_PORT enable_api_control:=true output:=screen &
LAUNCH_PID=$!

sleep 5

# Check if any AirSim nodes are running
if ros2 node list 2>/dev/null | grep -q airsim; then
    print_info "✓ AirSim node is running!"
    print_info "Available nodes:"
    ros2 node list | grep airsim
    
    # Check topics
    print_info "Available AirSim topics:"
    ros2 topic list | grep airsim | head -10
    
    # Check services
    print_info "Available AirSim services:"
    ros2 service list | grep airsim | head -10
    
else
    print_error "✗ No AirSim nodes found"
    print_warning "Check the launch output above for error messages"
fi

# Clean up
kill $LAUNCH_PID 2>/dev/null || true

# 6. Provide troubleshooting recommendations
print_section "Troubleshooting Recommendations"

echo "Based on the test results:"
echo
echo "1. API Connection Issues:"
echo "   - Ensure AirSim is running before starting the container"
echo "   - Check settings.json has: \"ApiServerEndpoint\": \"0.0.0.0:41451\""
echo "   - Try using your actual host IP instead of host.docker.internal"
echo "   - Disable Windows Firewall or add exception for port 41451"
echo
echo "2. ROS2 Wrapper Issues:"
echo "   - Rebuild the container if AirSim packages are missing"
echo "   - Check if all dependencies are installed correctly"
echo "   - Verify the AirSim workspace was built successfully"
echo
echo "3. Network Issues:"
echo "   - On Windows: Run 'docker run --add-host=host.docker.internal:host-gateway ...'"
echo "   - Or use your actual IP: 'docker run -e AIRSIM_HOST_IP=192.168.1.XXX ...'"
echo
echo "4. Quick Fix Commands:"
echo "   # Get your Windows IP:"
echo "   ipconfig | findstr IPv4"
echo "   "
echo "   # Rebuild container:"
echo "   airsim_docker.bat clean"
echo "   airsim_docker.bat build --no-cache"
echo

print_info "Debug script completed!" 