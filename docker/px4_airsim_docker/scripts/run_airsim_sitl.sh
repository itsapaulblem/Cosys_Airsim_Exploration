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

echo "=========================================="
echo "PX4 SITL Startup - Instance $instance_num"
echo "=========================================="
echo "$(date '+%Y-%m-%d %H:%M:%S') - Initializing..."
echo "Model: $PX4_SIM_MODEL"
echo "PARENT_DIR: $PARENT_DIR"
echo "BIN_DIR: $BIN_DIR"
echo "BUILD_DIR: $BUILD_DIR"

# Check if PX4 binary exists
if [ ! -f "$BIN_DIR" ]; then
    echo "ERROR: PX4 binary not found at: $BIN_DIR"
    echo "Make sure you've built PX4 with: make px4_sitl_default none_iris"
    exit 1
fi

working_dir="$instance_path/instance_$instance_num"
[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

echo "Working directory: $working_dir"

# Apply AirSim-recommended parameters
# Note: Config directory is mounted at /px4_workspace/PX4-Autopilot/Scripts/config via docker-compose
AIRSIM_PARAMS="$SCRIPT_DIR/config/airsim_params.txt"
if [ -f "$AIRSIM_PARAMS" ]; then
    echo "Applying AirSim-recommended parameters..."

    # Create extras.txt with param set commands
    # PX4 automatically sources extras.txt if it exists in the working directory
    > "$working_dir/extras.txt"  # Create empty file

    # Convert airsim_params.txt to param set commands
    while IFS= read -r line || [ -n "$line" ]; do
        # Skip comments and empty lines
        [[ "$line" =~ ^#.*$ ]] && continue
        [[ -z "${line// }" ]] && continue

        # Parse parameter name and value
        param_name=$(echo "$line" | awk '{print $1}')
        param_value=$(echo "$line" | awk '{print $2}')

        if [ -n "$param_name" ] && [ -n "$param_value" ]; then
            echo "param set $param_name $param_value" >> "$working_dir/extras.txt"
        fi
    done < "$AIRSIM_PARAMS"

    echo "✅ AirSim parameters configured ($(grep -c 'param set' "$working_dir/extras.txt") parameters)"
else
    echo "⚠️  AirSim parameters file not found at: $AIRSIM_PARAMS"
    echo "Continuing with default PX4 parameters..."
fi

# ================================
# AIRSIM CONNECTIVITY VALIDATION
# ================================
AIRSIM_HOST="${PX4_SIM_HOSTNAME:-localhost}"
AIRSIM_PORT="41451"  # Default AirSim API port
MAX_RETRIES=3
RETRY_DELAY=5

echo ""
echo "Validating AirSim connectivity..."
echo "Target: $AIRSIM_HOST:$AIRSIM_PORT"

attempt=1
airsim_reachable=false

while [ $attempt -le $MAX_RETRIES ]; do
    echo "$(date '+%H:%M:%S') - Attempt $attempt/$MAX_RETRIES: Testing connection to $AIRSIM_HOST..."

    # Test 1: Ping (may fail in some Docker networks, not critical)
    if ping -c 1 -W 2 $AIRSIM_HOST > /dev/null 2>&1; then
        echo "  ✅ Ping successful"
    else
        echo "  ⚠️  Ping failed (may be expected in Docker)"
    fi

    # Test 2: TCP port check (critical)
    if timeout 3 bash -c "echo > /dev/tcp/$AIRSIM_HOST/$AIRSIM_PORT" 2>/dev/null; then
        echo "  ✅ TCP port $AIRSIM_PORT is open"
        airsim_reachable=true
        break
    else
        echo "  ❌ TCP port $AIRSIM_PORT is not reachable"

        if [ $attempt -lt $MAX_RETRIES ]; then
            echo "  ⏳ Waiting ${RETRY_DELAY}s before retry..."
            sleep $RETRY_DELAY
        fi
    fi

    attempt=$((attempt + 1))
done

if [ "$airsim_reachable" = false ]; then
    echo ""
    echo "⚠️  WARNING: Could not connect to AirSim at $AIRSIM_HOST:$AIRSIM_PORT"
    echo "PX4 will start anyway, but MAVLink connection may fail."
    echo "Please verify:"
    echo "  1. AirSim is running"
    echo "  2. Hostname/IP is correct: $AIRSIM_HOST"
    echo "  3. Port $AIRSIM_PORT is accessible"
    echo ""
else
    echo ""
    echo "✅ AirSim connectivity validated"
    echo ""
fi

# ================================
# START PX4 SITL
# ================================
echo "$(date '+%H:%M:%S') - Starting PX4 SITL..."
echo ""

pushd "$working_dir" > /dev/null
$BIN_DIR -i $instance_num $BUILD_DIR -s "etc/init.d-posix/rcS" -t $TEST_DATA
exit_code=$?
popd > /dev/null

echo ""
echo "$(date '+%H:%M:%S') - PX4 SITL exited with code: $exit_code"
exit $exit_code