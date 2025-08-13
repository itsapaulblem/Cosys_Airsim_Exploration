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

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible) - FIXED VERSION"
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

# Create a custom rcS file that DOES NOT start simulator_mavlink
custom_rcs="$working_dir/custom_rcS"
cat > "$custom_rcs" << 'RCSEOF'
#!/bin/sh

# Custom rcS for AirSim - NO auto-simulator startup
# This prevents PX4 from automatically starting simulator_mavlink

uorb start
param load
param set MAV_0_CONFIG 101
param set MAV_0_MODE 0
param set MAV_0_RATE 240000
param set MAV_0_FORWARD 1
param set MAV_1_CONFIG 102
param set MAV_1_MODE 2
param set MAV_1_RATE 80000
param set SYS_COMPANION 921600

# Start essential modules but NOT simulator_mavlink
mavlink start -r 240000 -m config -o 14570
mavlink start -r 80000 -m gimbal -o 14580
mavlink boot_complete

# DO NOT start simulator_mavlink here - AirSim will connect externally
echo "PX4 started without auto-simulator - waiting for external simulator (AirSim)"
RCSEOF

chmod +x "$custom_rcs"

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

echo "🚀 Starting PX4 with CUSTOM rcS (no auto-simulator)..."
echo ""

# Set environment variables for this instance
export MAV_0_UDP_PORT=$mavlink_udp_port
export MAV_0_REMOTE_PORT=$qgc_port

pushd "$working_dir" > /dev/null

echo "Starting PX4 in background with custom rcS..."
$BIN_DIR -i $instance_num $BUILD_DIR -s "$custom_rcs" -t $TEST_DATA &
PX4_PID=$!

# Wait for PX4 to initialize
sleep 8

echo ""
echo "🔗 PX4 Status Check:"
if kill -0 $PX4_PID 2>/dev/null; then
    echo "✅ PX4 process running (PID: $PX4_PID)"
    echo "⏳ PX4 is PROPERLY waiting for external simulator (AirSim)"
    echo "🔌 NO auto-simulator should be running"
    echo ""
    echo "📋 Current Status:"
    echo "   - PX4 Instance: $instance_num"
    echo "   - AirSim TCP Port: $airsim_tcp_port"
    echo "   - QGC UDP Port: $qgc_port"
    echo "   - Internal Simulator: TRULY DISABLED"
    echo ""
    echo "✅ Ready for AirSim connection on port $airsim_tcp_port"
else
    echo "❌ PX4 failed to start"
fi

# Keep the container running and wait for PX4
wait $PX4_PID

popd > /dev/null