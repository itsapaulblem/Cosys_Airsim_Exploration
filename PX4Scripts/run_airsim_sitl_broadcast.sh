#!/bin/bash

# Modified AirSim SITL script with MAVLink broadcast enabled for container use

instance_num=0
[ -n "$1" ] && instance_num="$1"

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris

# Container-specific paths
PARENT_DIR="/px4_workspace/PX4-Autopilot"
BUILD_DIR=$PARENT_DIR/ROMFS/px4fmu_common
instance_path=$PARENT_DIR/build/px4_sitl_default
BIN_DIR=$PARENT_DIR/build/px4_sitl_default/bin/px4
TEST_DATA=$PARENT_DIR/test_data

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible with MAVLink broadcast)"
echo "Model: $PX4_SIM_MODEL"
echo "PARENT_DIR: $PARENT_DIR"
echo "BIN_DIR: $BIN_DIR"
echo "BUILD_DIR: $BUILD_DIR"
echo "MAVLink Broadcast: ENABLED"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"

# Create parameter file with MAVLink broadcast enabled
param_file="$working_dir/broadcast_params.txt"
cat > "$param_file" << EOF
# Enable MAVLink broadcast for network access
param set MAV_0_BROADCAST 1
param set MAV_1_BROADCAST 1
EOF

echo "🔧 MAVLink broadcast parameters created at: $param_file"
echo "🚀 Starting PX4 with MAVLink broadcast enabled..."
echo ""

# Set MAVLink broadcast environment variables
export MAV_0_BROADCAST=1
export MAV_1_BROADCAST=1

# Change to working directory
cd "$working_dir"

# Start PX4 with parameter file to enable broadcast
exec $BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA -d "$param_file"