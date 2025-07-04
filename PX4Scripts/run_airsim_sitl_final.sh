#!/bin/bash

instance_num=0
[ -n "$1" ] && instance_num="$1"

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# Modern PX4-Autopilot structure (no Firmware subdirectory)
BUILD_DIR=$PARENT_DIR/ROMFS/px4fmu_common
instance_path=$PARENT_DIR/build/px4_sitl_default
BIN_DIR=$PARENT_DIR/build/px4_sitl_default/bin/px4
TEST_DATA=$PARENT_DIR/test_data

# Configure different ports for each instance
# Instance 0: QGC port 14550, Instance 1: QGC port 14551, etc.
qgc_port=$((14550 + instance_num))
mavlink_udp_port=$((18570 + instance_num))

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible)"
echo "Model: $PX4_SIM_MODEL"
echo "QGroundControl target port: $qgc_port"
echo "MAVLink UDP port: $mavlink_udp_port"
echo "AirSim port: $((4560 + instance_num))"
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
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA
popd > /dev/null 