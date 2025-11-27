#!/bin/bash
set -e

echo "Building Xfs Project in UE5.5 Container"
echo "============================================"

# Configuration
PROJECT_PATH="/workspace/airsim/docker/unreal_executable/Xfs/Xfs.uproject"
OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged/Xfs"
PLATFORM="Linux"
CONFIG="Development"

# Validate project exists
if [ ! -f "${PROJECT_PATH}" ]; then
    echo "ERROR: Xfs.uproject not found at ${PROJECT_PATH}"
    exit 1
fi

echo "Build Configuration:"
echo "  Project: ${PROJECT_PATH}"
echo "  Platform: ${PLATFORM}"
echo "  Configuration: ${CONFIG}"
echo "  Output: ${OUTPUT_DIR}"
echo ""

# Step 0: Build AirLib if needed
echo "Step 0: Ensuring AirLib is built..."
if [ ! -f "/workspace/airsim/AirLib/lib/librpc.a" ]; then
    echo "  AirLib not found - building with container toolchain..."
    /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/build-airlib.sh
else
    echo "  ✓ AirLib already built"
    # Verify it's not a host build with incompatible symbols
    if nm /workspace/airsim/AirLib/lib/librpc.a 2>/dev/null | grep -q "__isoc23"; then
        echo "  WARNING: AirLib was built on host (glibc 2.38+ symbols detected)"
        echo "  This will cause linker errors. Rebuilding with container toolchain..."
        /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/build-airlib.sh
    else
        echo "  ✓ AirLib compatible with container environment"
    fi
fi
echo ""

# Step 0.5: Copy AirSim plugin (always copy to ensure fresh libraries)
echo "Step 0.5: Copying AirSim plugin with latest libraries..."
PLUGIN_SRC="/workspace/airsim/Unreal/Plugins/AirSim"
PLUGIN_DST="/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim"

if [ ! -d "${PLUGIN_SRC}" ]; then
    echo "  ERROR: AirSim plugin source not found at ${PLUGIN_SRC}"
    echo "  Please build AirLib first: ./build.sh in repository root"
    exit 1
fi

# Always remove and recopy to ensure libraries are up-to-date
if [ -d "${PLUGIN_DST}" ]; then
    echo "  → Removing old plugin copy..."
    rm -rf "${PLUGIN_DST}"
fi

echo "  → Copying fresh AirSim plugin to Xfs project..."
mkdir -p /workspace/airsim/docker/unreal_executable/Xfs/Plugins
cp -r "${PLUGIN_SRC}" "${PLUGIN_DST}"
echo "  ✓ AirSim plugin copied with libraries from:"
echo "    - AirLib/lib/*.a ($(date -r /workspace/airsim/AirLib/lib/librpc.a '+%Y-%m-%d %H:%M:%S'))"

# Verify no glibc 2.38+ symbols in copied plugin libraries
if nm "${PLUGIN_DST}/Source/AirLib/deps/rpclib/lib/librpc.a" 2>/dev/null | grep -q "__isoc23"; then
    echo "  WARNING: Plugin librpc.a has glibc 2.38+ symbols!"
    echo "  This should not happen. Check AirLib build process."
    exit 1
else
    echo "  ✓ Plugin libraries verified compatible with container"
fi
echo ""

# Step 1: Clean previous build (optional)
echo ""
echo "🧹 Step 1: Cleaning previous build artifacts..."
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Binaries
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Intermediate
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Saved/Cooked
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Saved/StagedBuilds
echo "  ✓ Clean complete"

# Step 2: Build, Cook, and Package
echo ""
echo "🔨 Step 2: Building and packaging Xfs project..."
echo "  This may take 30-60 minutes depending on system performance..."
echo ""

/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
    -project="${PROJECT_PATH}" \
    -platform="${PLATFORM}" \
    -clientconfig="${CONFIG}" \
    -serverconfig="${CONFIG}" \
    -targetPlatform=Linux \
    -targetedRHIs=Vulkan \
    -CookFlavor=VULKAN_SM6 \
    -noP4 \
    -nodebuginfo \
    -allmaps \
    -build \
    -cook \
    -stage \
    -pak \
    -archive \
    -archivedirectory="${OUTPUT_DIR}" \
    -utf8output

BUILD_STATUS=$?

echo ""
echo "============================================"
if [ $BUILD_STATUS -eq 0 ]; then
    echo "Build Complete!"
    echo ""
    echo "Packaged output directory:"
    echo "   ${OUTPUT_DIR}"
    echo ""
    echo "Executable location:"
    echo "   ${OUTPUT_DIR}/LinuxNoEditor/Xfs/Binaries/Linux/Xfs"
    echo ""
    echo "Package size:"
    du -sh "${OUTPUT_DIR}" 2>/dev/null || echo "   (size calculation unavailable)"
    echo ""
    echo "To run the packaged executable:"
    echo "   cd ${OUTPUT_DIR}/LinuxNoEditor"
    echo "   ./Xfs/Binaries/Linux/Xfs"
else
    echo "Build Failed with exit code: ${BUILD_STATUS}"
    echo ""
    echo "Check logs above for error details."
    exit $BUILD_STATUS
fi
