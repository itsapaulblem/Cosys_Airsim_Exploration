#!/bin/bash
set -e

echo "🔧 Complete Rebuild: AirLib + Xfs Custom Environment"
echo "===================================================="
echo ""
echo "This script performs a complete rebuild to incorporate code changes:"
echo "  1. Rebuilds AirLib with latest source code changes (SingleTaskCall fix)"
echo "  2. Copies fresh AirLib libraries to Xfs plugin"
echo "  3. Rebuilds Xfs Unreal project to incorporate new AirLib"
echo "  4. Packages Xfs executable for Docker deployment"
echo ""
echo "⏱️  Estimated time: 40-70 minutes (depending on system)"
echo ""

# Configuration
PROJECT_PATH="/workspace/airsim/docker/unreal_executable/Xfs/Xfs.uproject"
OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged"
PLATFORM="Linux"
CONFIG="Development"

# Validate project exists
if [ ! -f "${PROJECT_PATH}" ]; then
    echo "❌ ERROR: Xfs.uproject not found at ${PROJECT_PATH}"
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
        echo "  sudo rm -rf docker/unreal_executable/packaged/Xfs"
        echo "  sudo chown -R \$USER:\$USER docker/unreal_executable/packaged"
        echo ""
        echo "  Then re-run this build script."
        echo ""
        exit 1
    else
        rm -f "${OUTPUT_DIR}/.permission_test"
        echo "  ✓ Write permissions OK"
    fi

    # Clean previous Xfs build artifacts from output directory
    echo "  → Cleaning previous Xfs package artifacts..."
    rm -rf "${OUTPUT_DIR}/Linux" 2>/dev/null || true
    rm -rf "${OUTPUT_DIR}/Xfs" 2>/dev/null || true
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
echo "Step 1: Building AirLib with SingleTaskCall Fix"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "  This incorporates the fix: SingleCall → SingleTaskCall in MavLinkMultirotorApi.hpp:515"
echo "  The fix enables automatic OFFBOARD mode activation before takeoff commands"
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
        echo "  Forcing rebuild to ensure SingleTaskCall fix is included..."

        # Force clean rebuild to incorporate header changes
        echo "  Cleaning previous AirLib build..."
        rm -rf /workspace/airsim/AirLib/lib
        rm -rf /workspace/airsim/external/rpclib/build
        rm -rf /workspace/airsim/build_release

        /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
    fi
fi

echo ""
echo "✅ AirLib Build Complete with SingleTaskCall Fix"
echo ""

# Step 2: Copy Fresh AirLib Plugin to Xfs Project
echo "═══════════════════════════════════════════════════════"
echo "Step 2: Updating Xfs Plugin with New AirLib Libraries"
echo "═══════════════════════════════════════════════════════"
echo ""

PLUGIN_SRC="/workspace/airsim/Unreal/Plugins/AirSim"
PLUGIN_DST="/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim"

if [ ! -d "${PLUGIN_SRC}" ]; then
    echo "  ❌ ERROR: AirSim plugin source not found at ${PLUGIN_SRC}"
    echo "  AirLib build should have synced plugin, but sync failed"
    exit 1
fi

# Show current plugin library timestamps
if [ -f "${PLUGIN_DST}/Source/AirLib/lib/librpc.a" ]; then
    echo "  Current plugin librpc.a:"
    ls -lh "${PLUGIN_DST}/Source/AirLib/lib/librpc.a"
else
    echo "  No existing plugin found (first-time setup)"
fi

# Always remove and recopy to ensure libraries are up-to-date
echo ""
echo "  → Removing old plugin copy..."
rm -rf "${PLUGIN_DST}"

echo "  → Copying fresh AirSim plugin with new libraries..."
mkdir -p /workspace/airsim/docker/unreal_executable/Xfs/Plugins
cp -r "${PLUGIN_SRC}" "${PLUGIN_DST}"

echo "  ✓ AirSim plugin copied with libraries from:"
echo "    - Built: $(date -r /workspace/airsim/AirLib/lib/librpc.a '+%Y-%m-%d %H:%M:%S')"

# Verify no glibc 2.38+ symbols in copied plugin libraries
if nm "${PLUGIN_DST}/Source/AirLib/lib/librpc.a" 2>/dev/null | grep -q "__isoc23"; then
    echo "  ❌ ERROR: Plugin librpc.a has glibc 2.38+ symbols!"
    echo "  This should not happen. Check AirLib build process."
    exit 1
else
    echo "  ✓ Plugin libraries verified compatible with container"
fi

echo ""
echo "✅ Xfs Plugin Updated with SingleTaskCall Fix"
echo ""

# Step 3: Clean previous Xfs build
echo "═══════════════════════════════════════════════════════"
echo "Step 3: Cleaning Previous Xfs Build Artifacts"
echo "═══════════════════════════════════════════════════════"
echo ""

echo "  Removing Binaries, Intermediate, and Saved directories..."
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Binaries
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Intermediate
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Saved/Cooked
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Saved/StagedBuilds

echo "  ✓ Clean complete"
echo ""

# Step 4: Build, Cook, and Package Xfs
echo "═══════════════════════════════════════════════════════"
echo "Step 4: Building and Packaging Xfs Project"
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
    echo "✅ Complete Xfs Rebuild Successful!"
    echo ""
    echo "📁 Packaged output directory:"
    echo "   ${OUTPUT_DIR}/Xfs/"
    echo ""
    echo "🎮 Executable location:"
    XFS_EXECUTABLE="${OUTPUT_DIR}/LinuxNoEditor/Xfs/Binaries/Linux/Xfs"
    if [ -f "${XFS_EXECUTABLE}" ]; then
        echo "   ${XFS_EXECUTABLE}"
        ls -lh "${XFS_EXECUTABLE}"
    else
        # Try alternative location
        XFS_EXECUTABLE="${OUTPUT_DIR}/Xfs/LinuxNoEditor/Xfs/Binaries/Linux/Xfs"
        if [ -f "${XFS_EXECUTABLE}" ]; then
            echo "   ${XFS_EXECUTABLE}"
            ls -lh "${XFS_EXECUTABLE}"
        else
            echo "   ⚠️  Executable location verification skipped (will be accessible in packaged directory)"
        fi
    fi
    echo ""
    echo "📊 Package size:"
    du -sh "${OUTPUT_DIR}/Xfs" 2>/dev/null || du -sh "${OUTPUT_DIR}" 2>/dev/null || echo "   (size calculation unavailable)"
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "🔄 Next Steps:"
    echo "═══════════════════════════════════════════════════════"
    echo ""
    echo "1. Exit this container (type 'exit' or Ctrl+D)"
    echo ""
    echo "2. Verify your .env file points to the correct executable:"
    echo "   cd /home/mnsuser/Cosys_Airsim_Exploration/docker"
    echo "   cat .env | grep UNREAL_BINARY"
    echo ""
    echo "   Should show:"
    echo "   UNREAL_BINARY_PATH=/home/mnsuser/.../docker/unreal_executable/packaged"
    echo "   UNREAL_BINARY_SHELL_ABSPATH=/home/mnsuser/.../docker/unreal_executable/packaged/Xfs.sh"
    echo ""
    echo "3. Restart docker-compose services:"
    echo "   docker-compose -f docker-compose-master.yml down"
    echo "   docker-compose -f docker-compose-master.yml up --profile linux-integrated"
    echo ""
    echo "4. Launch ROS2 and test takeoff:"
    echo "   # In ROS2 container"
    echo "   ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py"
    echo ""
    echo "   # Test takeoff"
    echo "   ros2 service call /Drone1/takeoff airsim_interfaces/srv/Takeoff \"{}\""
    echo ""
    echo "5. Monitor PX4 logs to verify SingleTaskCall fix is active:"
    echo "   docker logs -f px4-drone-1 2>&1 | grep -i offboard"
    echo ""
    echo "   ✅ SUCCESS: You should see:"
    echo "   INFO [commander] Offboard control started"
    echo "   INFO [commander] Main state changed to MAIN_STATE_OFFBOARD"
    echo ""
    echo "   This proves the SingleTaskCall fix is working!"
    echo ""
    echo "6. Expected drone behavior:"
    echo "   - 'Ground already stable' messages (normal, benign)"
    echo "   - 'Take off to 137.2' message (normal GPS altitude)"
    echo "   - ✅ Drone actually takes off and climbs to ~3m altitude"
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo ""
else
    echo "❌ Build Failed with exit code: ${BUILD_STATUS}"
    echo ""
    echo "Check logs above for error details."
    echo "Common issues:"
    echo "  - Insufficient memory (requires ~16GB RAM for UE5 builds)"
    echo "  - Disk space (requires ~50GB free space)"
    echo "  - AirLib compatibility (check glibc symbols)"
    echo "  - Plugin not found (verify AirLib build completed)"
    exit $BUILD_STATUS
fi
