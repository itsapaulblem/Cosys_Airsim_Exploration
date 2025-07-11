#!/bin/bash

instance_num=0
[ -n "$1" ] && instance_num="$1"

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris

# CRITICAL: Prevent auto-start of internal simulator
export PX4_SIMULATOR=none

# Docker-specific paths - PX4 is installed in /px4_workspace/PX4-Autopilot
PARENT_DIR="/px4_workspace/PX4-Autopilot"

# Modern PX4-Autopilot structure (no Firmware subdirectory)
BUILD_DIR=$PARENT_DIR/ROMFS/px4fmu_common
instance_path=$PARENT_DIR/build/px4_sitl_default
BIN_DIR=$PARENT_DIR/build/px4_sitl_default/bin/px4
TEST_DATA=$PARENT_DIR/test_data

# Configure different ports for each instance
# Instance 1: QGC port 14550, TCP port 4561, etc.
qgc_port=$((14549 + instance_num))
mavlink_udp_port=$((18570 + instance_num))
airsim_tcp_port=$((4560 + instance_num))

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible)"
echo "Model: $PX4_SIM_MODEL"
echo "QGroundControl target port: $qgc_port"
echo "MAVLink UDP port: $mavlink_udp_port"
echo "AirSim TCP port: $airsim_tcp_port"
echo "PX4_SIM_HOSTNAME: ${PX4_SIM_HOSTNAME:-localhost}"
echo "PARENT_DIR: $PARENT_DIR"

# Network validation
echo ""
echo "🔍 Network Validation"
echo "=============================="

# Test if we're in Docker
if [ -f /.dockerenv ]; then
    echo "📦 Running in Docker container"
    
    # Test AirSim connectivity
    AIRSIM_HOST=${PX4_SIM_HOSTNAME:-localhost}
    echo "Testing AirSim connectivity to $AIRSIM_HOST..."
    
    if timeout 3 nc -zv $AIRSIM_HOST 41451 2>/dev/null; then
        echo "✅ AirSim API reachable at $AIRSIM_HOST:41451"
    else
        echo "⚠️ AirSim API not reachable at $AIRSIM_HOST:41451"
        echo "💡 Ensure AirSim is running and accessible"
        echo "💡 Check Docker network mode (should be 'host' for direct access)"
    fi
    
    # Test if TCP port is available for PX4
    if timeout 1 nc -zv $AIRSIM_HOST $airsim_tcp_port 2>/dev/null; then
        echo "⚠️ TCP port $airsim_tcp_port already in use"
    else
        echo "✅ TCP port $airsim_tcp_port available"
    fi
else
    echo "🖥️ Running on host system"
fi

echo ""

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"

# Create instance-specific parameter file
param_file="$working_dir/parameters_instance_$instance_num.bson"
if [ ! -f "$param_file" ]; then
    echo "📝 Creating instance-specific parameters..."
    cat > "$working_dir/instance_params.txt" << EOF
MAV_0_CONFIG 101
MAV_0_MODE 0
MAV_0_RATE 240000
MAV_0_FORWARD 1
MAV_1_CONFIG 102
MAV_1_MODE 2
MAV_1_RATE 80000
SYS_COMPANION 921600
EOF
fi

echo "🚀 Starting PX4 with custom MAVLink configuration..."
echo ""

# Set environment variables for this instance
export MAV_0_UDP_PORT=$mavlink_udp_port
export MAV_0_REMOTE_PORT=$qgc_port

pushd "$working_dir" > /dev/null

echo "Starting PX4 in background..."
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA &
PX4_PID=$!

# Wait for PX4 to initialize
sleep 8

echo ""
echo "🔗 PX4 Status Check:"
if kill -0 $PX4_PID 2>/dev/null; then
    echo "✅ PX4 process running (PID: $PX4_PID)"
    echo "⏳ PX4 is now waiting for external simulator (AirSim)"
    echo "🔌 When AirSim connects to TCP port $airsim_tcp_port, you'll see 'Simulator connected'"
    echo ""
    echo "📋 Current Status:"
    echo "   - PX4 Instance: $instance_num"
    echo "   - AirSim TCP Port: $airsim_tcp_port"
    echo "   - QGC UDP Port: $qgc_port"
    echo "   - Internal Simulator: DISABLED"
    echo ""
    echo "💡 If you see 'Simulator connected' before starting AirSim,"
    echo "   that means the auto-simulator is still running."
    echo "   Stop the container and rebuild with the fix."
else
    echo "❌ PX4 failed to start"
fi

# Keep the container running and wait for PX4
wait $PX4_PID

popd > /dev/null 