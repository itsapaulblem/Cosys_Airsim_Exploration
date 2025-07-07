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

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible) - PROPER VERSION"
echo "Model: $PX4_SIM_MODEL"
echo "QGroundControl target port: $qgc_port"
echo "MAVLink UDP port: $mavlink_udp_port"
echo "AirSim TCP port: $airsim_tcp_port"
echo "PARENT_DIR: $PARENT_DIR"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"

echo "🚀 Starting PX4 with standard rcS but preventing auto-simulator..."
echo ""

# Set environment variables for this instance  
export MAV_0_UDP_PORT=$mavlink_udp_port
export MAV_0_REMOTE_PORT=$qgc_port

# Key insight: Set PX4_SIM_HOSTNAME to a non-listening host to prevent auto-connection
export PX4_SIM_HOSTNAME=127.0.0.1

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
    
    # Now manually start simulator_mavlink in proper mode
    echo "🔧 Manually configuring simulator_mavlink for AirSim..."
    
    # Connect to PX4 shell and start simulator properly
    echo "simulator_mavlink start -h host.docker.internal -p $airsim_tcp_port" | docker exec -i px4-drone-$instance_num /px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_num $BUILD_DIR -s /dev/stdin -t $TEST_DATA &
    
    echo "⏳ PX4 is now PROPERLY waiting for external simulator (AirSim)"
    echo "🔌 Connect AirSim to TCP port $airsim_tcp_port"
    echo ""
    echo "📋 Current Status:"
    echo "   - PX4 Instance: $instance_num"
    echo "   - AirSim TCP Port: $airsim_tcp_port"
    echo "   - QGC UDP Port: $qgc_port"  
    echo "   - Internal Simulator: PROPERLY DISABLED"
    echo ""
    echo "✅ Ready for AirSim connection on host.docker.internal:$airsim_tcp_port"
else
    echo "❌ PX4 failed to start"
fi

# Keep the container running and wait for PX4
wait $PX4_PID

popd > /dev/null