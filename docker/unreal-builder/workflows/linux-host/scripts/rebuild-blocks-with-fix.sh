#!/bin/bash
set -e

echo "🔧 Complete Rebuild: AirLib + Blocks Executable"
echo "==============================================="
echo ""
echo "This script performs a complete rebuild to incorporate code changes:"
echo "  1. Rebuilds AirLib with latest source code changes"
echo "  2. Rebuilds Blocks Unreal project to incorporate new AirLib"
echo "  3. Packages Blocks executable for Docker deployment"
echo ""
echo "⏱️  Estimated time: 40-70 minutes (depending on system)"
echo ""

# Configuration
PROJECT_PATH="/workspace/airsim/Unreal/Environments/Blocks/Blocks.uproject"
OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged"
PLATFORM="Linux"
CONFIG="Development"

# Validate project exists
if [ ! -f "${PROJECT_PATH}" ]; then
    echo "❌ ERROR: Blocks.uproject not found at ${PROJECT_PATH}"
    exit 1
fi

echo "📋 Build Configuration:"
echo "  Project: ${PROJECT_PATH}"
echo "  Platform: ${PLATFORM}"
echo "  Configuration: ${CONFIG}"
echo "  Output: ${OUTPUT_DIR}"
echo ""

# Step 0: Prepare Output Directory
echo "═══════════════════════════════════════════════════════"
echo "Step 0: Preparing Output Directory"
echo "═══════════════════════════════════════════════════════"
echo ""

if [ -d "${OUTPUT_DIR}" ]; then
    echo "  Output directory exists: ${OUTPUT_DIR}"
    echo "  Checking permissions..."

    # Try to create a test file to check write permissions
    if ! touch "${OUTPUT_DIR}/.permission_test" 2>/dev/null; then
        echo "  ❌ ERROR: Cannot write to ${OUTPUT_DIR}"
        echo ""
        echo "  This is typically caused by Docker volume mount permission conflicts."
        echo "  Previous builds may have created files with different ownership."
        echo ""
        echo "  🔧 FIX: Run these commands on your HOST machine (outside container):"
        echo ""
        echo "  cd /home/mnsuser/Cosys_Airsim_Exploration"
        echo "  sudo rm -rf docker/unreal_executable/packaged/Linux"
        echo "  sudo rm -rf docker/unreal_executable/packaged/Blocks"
        echo "  sudo chown -R \$USER:\$USER docker/unreal_executable/packaged"
        echo ""
        echo "  Then re-run this build script."
        echo ""
        exit 1
    else
        rm -f "${OUTPUT_DIR}/.permission_test"
        echo "  ✓ Write permissions OK"
    fi

    # Clean previous Blocks build artifacts from output directory
    echo "  → Cleaning previous Blocks package artifacts..."
    rm -rf "${OUTPUT_DIR}/Linux" 2>/dev/null || true
    rm -rf "${OUTPUT_DIR}/Blocks" 2>/dev/null || true
    rm -rf "${OUTPUT_DIR}/LinuxNoEditor" 2>/dev/null || true
    echo "  ✓ Output directory cleaned"
else
    echo "  Creating output directory: ${OUTPUT_DIR}"
    mkdir -p "${OUTPUT_DIR}"

    # Verify we can write to it
    if ! touch "${OUTPUT_DIR}/.permission_test" 2>/dev/null; then
        echo "  ❌ ERROR: Cannot create or write to ${OUTPUT_DIR}"
        echo ""
        echo "  Check that the parent directory has correct permissions."
        echo ""
        exit 1
    else
        rm -f "${OUTPUT_DIR}/.permission_test"
        echo "  ✓ Output directory created successfully"
    fi
fi

echo ""
echo "✅ Output Directory Ready"
echo ""

# Step 1: Build AirLib
echo "═══════════════════════════════════════════════════════"
echo "Step 1: Building AirLib with Latest Code Changes"
echo "═══════════════════════════════════════════════════════"
echo ""

if [ ! -f "/workspace/airsim/AirLib/lib/librpc.a" ]; then
    echo "  ⚠️  AirLib not found - building with container toolchain..."
    /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
else
    # Check if AirLib needs rebuild
    echo "  Checking AirLib compatibility..."
    if nm /workspace/airsim/AirLib/lib/librpc.a 2>/dev/null | grep -q "__isoc23"; then
        echo "  ⚠️  WARNING: AirLib has glibc 2.38+ symbols (host build detected)"
        echo "  Rebuilding with container toolchain..."
        /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
    else
        echo "  ✓ AirLib found and compatible with container"
        echo "  Re-running build to ensure latest changes are incorporated..."
        /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
    fi
fi

echo ""
echo "✅ AirLib Build Complete"
echo ""

# Step 2: Clean previous Blocks build
echo "═══════════════════════════════════════════════════════"
echo "Step 2: Cleaning Previous Blocks Build Artifacts"
echo "═══════════════════════════════════════════════════════"
echo ""

echo "  Removing Binaries, Intermediate, and Saved directories..."
rm -rf /workspace/airsim/Unreal/Environments/Blocks/Binaries
rm -rf /workspace/airsim/Unreal/Environments/Blocks/Intermediate
rm -rf /workspace/airsim/Unreal/Environments/Blocks/Saved/Cooked
rm -rf /workspace/airsim/Unreal/Environments/Blocks/Saved/StagedBuilds

echo "  ✓ Clean complete"
echo ""

# Step 3: Build, Cook, and Package Blocks
echo "═══════════════════════════════════════════════════════"
echo "Step 3: Building and Packaging Blocks Project"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "  This may take 30-60 minutes depending on system performance..."
echo "  Building with Vulkan SM6 for optimal compatibility..."
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
echo "═══════════════════════════════════════════════════════"
if [ $BUILD_STATUS -eq 0 ]; then
    echo "✅ Complete Rebuild Successful!"
    echo ""
    echo "📁 Packaged output directory:"
    echo "   ${OUTPUT_DIR}"
    echo ""
    echo "🎮 Executable location:"
    echo "   ${OUTPUT_DIR}/LinuxNoEditor/Blocks/Binaries/Linux/Blocks"
    echo ""
    echo "📊 Package size:"
    du -sh "${OUTPUT_DIR}" 2>/dev/null || echo "   (size calculation unavailable)"
    echo ""
    echo "🔄 Next Steps:"
    echo "   1. Exit this container"
    echo "   2. Restart docker-compose services:"
    echo "      cd /home/mnsuser/Cosys_Airsim_Exploration/docker"
    echo "      docker-compose -f docker-compose-master.yml down"
    echo "      docker-compose -f docker-compose-master.yml up --profile linux-integrated"
    echo ""
    echo "   3. Test takeoff with ROS2:"
    echo "      ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py"
    echo "      ros2 service call /Drone1/takeoff airsim_interfaces/srv/Takeoff \"{}\""
    echo ""
    echo "   4. Monitor PX4 logs for OFFBOARD mode (proves fix works):"
    echo "      docker logs -f px4-drone-1 2>&1 | grep -i offboard"
    echo ""
    echo "   Expected: \"INFO [commander] Offboard control started\""
    echo ""
else
    echo "❌ Build Failed with exit code: ${BUILD_STATUS}"
    echo ""
    echo "Check logs above for error details."
    echo "Common issues:"
    echo "  - Insufficient memory (requires ~16GB RAM for UE5 builds)"
    echo "  - Disk space (requires ~50GB free space)"
    echo "  - AirLib compatibility (check glibc symbols)"
    exit $BUILD_STATUS
fi
