#!/bin/bash

set -e  # Exit on any error

echo "================================================"
echo "   Starting PX4 SITL for AirSim Integration"
echo "================================================"

# Get instance number from environment variable (default to 1)
INSTANCE_NUM=${PX4_INSTANCE:-1}

echo "Starting PX4 SITL Instance ${INSTANCE_NUM}"
echo "PX4_SYS_AUTOSTART: ${PX4_SYS_AUTOSTART}"
echo "Home position: LAT=${PX4_HOME_LAT}, LON=${PX4_HOME_LON}, ALT=${PX4_HOME_ALT}"
echo "Simulator hostname: ${PX4_SIM_HOSTNAME}"

# Calculate ports based on instance number (1-based)
TCP_PORT=$((4560 + INSTANCE_NUM))
QGC_PORT=$((14540 + INSTANCE_NUM))
MAVLINK_UDP_PORT=$((18570 + INSTANCE_NUM))

echo "Port configuration:"
echo "  TCP (AirSim):     ${TCP_PORT}"
echo "  QGC Port:         ${QGC_PORT}"
echo "  MAVLink UDP:      ${MAVLINK_UDP_PORT}"

# Change to PX4-Autopilot directory
cd /px4_workspace/PX4-Autopilot

# Use none_iris model which is designed for external simulators like AirSim
export PX4_SIM_MODEL=none_iris

# Setup directories based on your working script
BUILD_DIR=/px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common
INSTANCE_PATH=/px4_workspace/PX4-Autopilot/build/px4_sitl_default
BIN_DIR=/px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4
TEST_DATA=/px4_workspace/PX4-Autopilot/test_data

echo "🚁 Starting PX4 SITL instance $INSTANCE_NUM (AirSim-compatible)"
echo "Model: $PX4_SIM_MODEL"
echo "QGroundControl target port: $QGC_PORT"
echo "MAVLink UDP port: $MAVLINK_UDP_PORT"
echo "AirSim port: $TCP_PORT"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

WORKING_DIR="$INSTANCE_PATH/instance_$INSTANCE_NUM"
[ ! -d "$WORKING_DIR" ] && mkdir -p "$WORKING_DIR"

echo "Working directory: $WORKING_DIR"

# Create instance-specific parameter file
PARAM_FILE="$WORKING_DIR/parameters_instance_$INSTANCE_NUM.bson"
if [ ! -f "$PARAM_FILE" ]; then
    echo "📝 Creating instance-specific parameters..."
    cat > "$WORKING_DIR/instance_params.txt" << EOF
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
export MAV_0_UDP_PORT=$MAVLINK_UDP_PORT
export MAV_0_REMOTE_PORT=$QGC_PORT

pushd "$WORKING_DIR" > /dev/null
exec $BIN_DIR -i $INSTANCE_NUM $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA
popd > /dev/null 