#!/bin/bash

instance_num=0
[ -n "$1" ] && instance_num="$1"

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris

# CRITICAL: Set PX4_SIMULATOR to bypass auto-startup of internal simulator
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

echo "🚁 Starting PX4 SITL instance $instance_num (External Simulator Mode)"
echo "Model: $PX4_SIM_MODEL"
echo "Simulator Mode: $PX4_SIMULATOR (prevents auto-start)"
echo "QGroundControl target port: $qgc_port"
echo "MAVLink UDP port: $mavlink_udp_port"
echo "AirSim TCP port: $airsim_tcp_port"
echo "PARENT_DIR: $PARENT_DIR"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"

# Create instance-specific parameter file for external simulator
param_file="$working_dir/parameters_instance_$instance_num.bson"
if [ ! -f "$param_file" ]; then
    echo "📝 Creating instance-specific parameters for external simulator..."
    cat > "$working_dir/instance_params.txt" << EOF
# External simulator configuration
MAV_0_CONFIG 101
MAV_0_MODE 0
MAV_0_RATE 240000
MAV_0_FORWARD 1
MAV_1_CONFIG 102
MAV_1_MODE 2
MAV_1_RATE 80000
SYS_COMPANION 921600

# Disable auto-start of internal simulators
SIM_GZ_EN 0
EOF
fi

echo "🚀 Starting PX4 in external simulator mode..."
echo "⏳ PX4 will wait for external simulator (AirSim) to connect..."
echo ""

# Set environment variables for this instance
export MAV_0_UDP_PORT=$mavlink_udp_port
export MAV_0_REMOTE_PORT=$qgc_port

# Ensure external simulator mode
export PX4_SIM_HOSTNAME=localhost
export PX4_SIM_HOST_ADDR=127.0.0.1

pushd "$working_dir" > /dev/null

# Start PX4 and manually start simulator_mavlink in the correct mode
$BIN_DIR -i $instance_num $BUILD_DIR -s etc/init.d-posix/rcS -t $TEST_DATA -d << EOF &
# Wait for PX4 to initialize
sleep 5
# Start simulator_mavlink in server mode waiting for external connection
simulator_mavlink start -s -p $airsim_tcp_port
echo "✅ PX4 ready for external simulator connection on port $airsim_tcp_port"
EOF

PX4_PID=$!

# Wait for PX4 to start and then start the MAVLink interface
sleep 10

echo ""
echo "🔗 PX4 Configuration:"
echo "   - Instance: $instance_num"
echo "   - Waiting for AirSim on TCP port: $airsim_tcp_port"
echo "   - QGroundControl can connect to UDP port: $qgc_port"
echo "   - Internal simulator: DISABLED"
echo ""
echo "📋 Next Steps:"
echo "   1. Launch AirSim (Unreal Engine)"
echo "   2. AirSim should connect to TCP port $airsim_tcp_port"
echo "   3. Look for 'Simulator connected' message AFTER AirSim starts"
echo ""

# Wait for the PX4 process
wait $PX4_PID

popd > /dev/null