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

echo "🚁 Starting PX4 SITL instance $instance_num (AirSim-compatible)"
echo "Model: $PX4_SIM_MODEL"
echo "PARENT_DIR: $PARENT_DIR"
echo "BIN_DIR: $BIN_DIR"
echo "BUILD_DIR: $BUILD_DIR"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "❌ PX4 binary not found at: $BIN_DIR"
    echo "💡 Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"
echo "🚀 Starting PX4..."
echo ""

pushd "$working_dir" > /dev/null
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA
popd > /dev/null