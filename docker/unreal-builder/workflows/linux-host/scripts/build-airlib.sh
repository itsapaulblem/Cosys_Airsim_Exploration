#!/bin/bash
set -e

echo "🔧 Building AirLib Inside Container with Epic's Toolchain"
echo "=========================================================="

cd /workspace/airsim

# Check if already built
if [ -f "AirLib/lib/librpc.a" ]; then
    echo "AirLib libraries already exist"
    echo "   To rebuild, delete: AirLib/lib/ and external/rpclib/build/"
    echo ""
    read -p "Delete and rebuild? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "✓ Using existing AirLib build"
        exit 0
    fi
    echo "Cleaning previous builds..."
    rm -rf AirLib/lib
    rm -rf external/rpclib/build
fi

echo ""
echo "Running setup.sh..."
./setup.sh

echo ""
echo "   Building AirLib with Epic's Clang 18.1.0 toolchain..."
echo "   Using CMake directly (bypassing build.sh to avoid compiler override)"
echo "   CMake will auto-discover Epic's bundled Clang"
echo "   This ensures glibc 2.28 compatibility with container environment"
echo ""

# Build AirLib by calling CMake directly
# This avoids build.sh setting CC/CXX to system compilers
# which triggers Epic's toolchain enforcement wrappers at /usr/local/bin/clang
mkdir -p build_release
pushd build_release

# Don't set CC/CXX environment variables
# CMake will auto-discover Epic's bundled Clang at:
# /home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/.../clang
cmake ../cmake -DCMAKE_BUILD_TYPE=Release

# Build with all available CPU cores
make -j$(nproc)

# Copy build artifacts to expected locations
# This replicates what build.sh does after building
echo ""
echo "Copying build artifacts to AirLib directories..."

# Create lib directories if they don't exist
mkdir -p ../AirLib/lib
mkdir -p ../AirLib/deps/rpclib/lib
mkdir -p ../AirLib/deps/MavLinkCom/lib

# Copy all static libraries from build output to AirLib/lib and deps
if [ -d "output/lib" ]; then
    # Copy to main lib directory
    cp -v output/lib/*.a ../AirLib/lib/
    echo "Copied libraries to AirLib/lib/"

    # Copy to deps directories (plugin sources from these)
    cp -v output/lib/librpc.a ../AirLib/deps/rpclib/lib/
    echo "Copied librpc.a to AirLib/deps/rpclib/lib/"

    cp -v output/lib/libMavLinkCom.a ../AirLib/deps/MavLinkCom/lib/
    echo "Copied libMavLinkCom.a to AirLib/deps/MavLinkCom/lib/"

elif [ -d "lib" ]; then
    # Copy to main lib directory
    cp -v lib/*.a ../AirLib/lib/
    echo "Copied libraries to AirLib/lib/"

    # Copy to deps directories
    cp -v lib/librpc.a ../AirLib/deps/rpclib/lib/
    echo "Copied librpc.a to AirLib/deps/rpclib/lib/"

    cp -v lib/libMavLinkCom.a ../AirLib/deps/MavLinkCom/lib/
    echo "Copied libMavLinkCom.a to AirLib/deps/MavLinkCom/lib/"
else
    echo "   ERROR: Could not find built libraries in expected locations"
    echo "   Checked: build_release/output/lib/ and build_release/lib/"
    echo "   Available directories in build_release/:"
    ls -la
    popd
    exit 1
fi

popd

# Sync libraries to Unreal plugin source directory (Layer 2)
# This is what build.sh normally does - we need to replicate it
echo ""
echo "Syncing libraries to Unreal plugin source directory..."
echo "   This ensures the plugin source has the latest container-built libraries"

# Create plugin lib directories if they don't exist
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/lib
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/lib

# Sync from AirLib build output to plugin source
cp -v AirLib/lib/*.a Unreal/Plugins/AirSim/Source/AirLib/lib/
echo "Synced libraries to Unreal/Plugins/AirSim/Source/AirLib/lib/"

cp -v AirLib/deps/rpclib/lib/librpc.a Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/
echo "Synced librpc.a to plugin deps/rpclib/lib/"

cp -v AirLib/deps/MavLinkCom/lib/libMavLinkCom.a Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/lib/
echo "Synced libMavLinkCom.a to plugin deps/MavLinkCom/lib/"

# Sync dependency headers to plugin source (Layer 2)
# These headers are required for UE5 to compile the plugin
echo ""
echo "Syncing dependency headers to plugin source..."

# Sync Eigen headers
if [ -d "AirLib/deps/eigen3/Eigen" ]; then
    mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3
    cp -r AirLib/deps/eigen3/Eigen Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3/
    echo "Synced eigen3/Eigen/ headers to plugin deps/"
else
    echo "WARNING: AirLib/deps/eigen3/Eigen/ not found"
fi

# Sync MavLinkCom headers
if [ -d "AirLib/deps/MavLinkCom/include" ]; then
    mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom
    cp -r AirLib/deps/MavLinkCom/include Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/
    echo "Synced MavLinkCom/include/ headers to plugin deps/"
else
    echo "WARNING: AirLib/deps/MavLinkCom/include/ not found"
fi

# Sync rpclib headers
if [ -d "AirLib/deps/rpclib/include" ]; then
    mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib
    cp -r AirLib/deps/rpclib/include Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/
    echo "Synced rpclib/include/ headers to plugin deps/"
else
    echo "WARNING: AirLib/deps/rpclib/include/ not found"
fi

# Verify build succeeded
if [ -f "AirLib/lib/librpc.a" ]; then
    echo ""
    echo "AirLib Built Successfully!"
    echo ""
    echo "  Build artifacts:"
    echo "  AirLib/lib/:"
    ls -lh AirLib/lib/
    echo ""
    echo "   AirLib/deps/rpclib/lib/:"
    ls -lh AirLib/deps/rpclib/lib/
    echo ""
    echo "   AirLib/deps/MavLinkCom/lib/:"
    ls -lh AirLib/deps/MavLinkCom/lib/
    echo ""
    echo "  Verifying glibc symbols in all librpc.a locations..."

    # Check main lib
    if nm AirLib/lib/librpc.a 2>/dev/null | grep -q "__isoc23"; then
        echo "   WARNING: Found glibc 2.38+ symbols in AirLib/lib/librpc.a"
        echo "   This may cause linker errors. Rebuild recommended."
    else
        echo "AirLib/lib/librpc.a - compatible with container"
    fi

    # Check deps lib (Layer 1)
    if nm AirLib/deps/rpclib/lib/librpc.a 2>/dev/null | grep -q "__isoc23"; then
        echo "   WARNING: Found glibc 2.38+ symbols in AirLib/deps/rpclib/lib/librpc.a"
        echo "   This may cause linker errors. Rebuild recommended."
    else
        echo "AirLib/deps/rpclib/lib/librpc.a - compatible with container"
    fi

    # Check plugin source lib (Layer 2 - critical!)
    if nm Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a 2>/dev/null | grep -q "__isoc23"; then
        echo "   WARNING: Found glibc 2.38+ symbols in plugin source librpc.a"
        echo "   This may cause linker errors. Rebuild recommended."
    else
        echo "Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a - compatible with container"
    fi
else
    echo ""
    echo "   Build Failed - librpc.a not found"
    exit 1
fi

echo ""
echo "🎯 AirLib is ready for UE5 plugin integration"
