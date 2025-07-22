#!/bin/bash

# Modified AirSim SITL script with MAVLink broadcast enabled
instance_num=0
[ -n "$1" ] && instance_num="$1"

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris

# PX4 paths for container environment
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

# Create parameter file that PX4 loads at startup  
param_file="$working_dir/px4_params"

# Use environment variable for remote port (set by docker-compose)
REMOTE_PORT=${MAV_0_REMOTE_PORT:-$((14549 + $instance_num))}

# Create PX4 parameter file (loaded before MAVLink starts)
cat > "$param_file" << EOF
# MAVLink Configuration for Docker Bridge Gateway
MAV_0_BROADCAST 1
MAV_0_MODE 2  
MAV_0_CONFIG 0
MAV_0_REMOTE_PORT $REMOTE_PORT
MAV_0_REMOTE_IP 2886991873
MAV_0_FORWARD 1
MAV_SYS_ID $instance_num

# Additional MAVLink instances  
MAV_1_BROADCAST 1
MAV_1_MODE 2
MAV_1_CONFIG 0
EOF

echo "🔧 MAVLink broadcast parameters:"
cat "$param_file"
echo ""

# Set MAVLink broadcast environment variables (these are read by PX4 startup)
export MAV_0_BROADCAST=1
export MAV_1_BROADCAST=1
export MAV_0_CONFIG=0
export MAV_1_CONFIG=0
export MAV_0_MODE=2
export MAV_1_MODE=2
export MAV_0_REMOTE_PORT=$REMOTE_PORT
export MAV_0_REMOTE_IP=2886991873
export MAV_0_FORWARD=1

# Also set as PX4 parameters via environment (alternative method)
export PX4_MAV_0_BROADCAST=1
export PX4_MAV_1_BROADCAST=1

# Set specific MAVLink system configuration
export PX4_SYS_ID=$instance_num
export PX4_COMP_ID=1

# Use PX4 environment variable format for MAVLink configuration
export PX4_PARAM_MAV_0_BROADCAST=1
export PX4_PARAM_MAV_0_MODE=2
export PX4_PARAM_MAV_0_CONFIG=0
export PX4_PARAM_MAV_0_REMOTE_PORT=$REMOTE_PORT
export PX4_PARAM_MAV_0_REMOTE_IP=2886991873
export PX4_PARAM_MAV_0_FORWARD=1
export PX4_PARAM_MAV_SYS_ID=$instance_num

echo "🚀 Starting PX4 with MAVLink broadcast enabled..."
echo "   Instance: $instance_num"
echo "   MAV_0_BROADCAST=$MAV_0_BROADCAST"
echo "   MAV_1_BROADCAST=$MAV_1_BROADCAST"
echo "   Remote Port: $REMOTE_PORT"
echo "   Remote IP: 172.20.0.1 (Docker bridge gateway)"
echo "   System ID: $PX4_SYS_ID"
echo ""

cd "$working_dir"

# Start PX4 with parameter file loaded at startup
exec $BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA -f "$param_file"