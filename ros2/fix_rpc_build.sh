#!/bin/bash
# Fix RPC library build issue in ROS2 Docker container

echo "Fixing RPC library linking issue..."

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# Step 1: Rebuild RPC library with -fPIC
echo -e "${GREEN}Step 1: Rebuilding RPC library with -fPIC${NC}"
cd /airsim_ros2_ws/external/rpclib/rpclib-2.3.1

# Clean previous build
rm -rf build
mkdir build && cd build

# Configure with position-independent code
cmake .. \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_CXX_FLAGS="-fPIC -O2" \
    -DCMAKE_C_FLAGS="-fPIC -O2" \
    -DRPCLIB_BUILD_STATIC=ON

# Build
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo -e "${RED}RPC library build failed${NC}"
    exit 1
fi

echo -e "${GREEN}RPC library rebuilt successfully${NC}"

# Step 2: Clean ROS2 build artifacts
echo -e "${GREEN}Step 2: Cleaning ROS2 build artifacts${NC}"
cd /airsim_ros2_ws
rm -rf build/airsim_ros_pkgs install/airsim_ros_pkgs

# Step 3: Rebuild ROS2 package with correct flags
echo -e "${GREEN}Step 3: Rebuilding ROS2 package${NC}"
colcon build --packages-select airsim_ros_pkgs \
    --cmake-args \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_CXX_FLAGS="-fPIC -O2" \
    -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Source the workspace: source /airsim_ros2_ws/install/setup.bash"
    echo "2. Launch nodes: ros2 launch airsim_ros_pkgs airsim_node.launch.py"
    echo "3. Check transform: ros2 run tf2_ros tf2_echo Drone_1_base_link Drone_1/LidarSensor1"
else
    echo -e "${RED}✗ Build failed${NC}"
    echo ""
    echo "Alternative: Use the monolithic node instead:"
    echo "ros2 launch airsim_ros_pkgs airsim_node.launch.py"
fi