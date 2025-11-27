#!/usr/bin/env bash

# Containerized AirSim build script using Epic's UE5 bundled toolchain
# Uses Epic's clang-18.1.0 + bundled libc++ for perfect ABI compatibility with UE5.5.4

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Go to AirSim root directory (two levels up from docker/unreal-airsim)
AIRSIM_ROOT="$( cd "$SCRIPT_DIR/../../" && pwd )"

# Fix Docker volume permission issues by copying to container-local directory
CONTAINER_BUILD_ROOT="/tmp/airsim_build"
echo "🐳 Fixing Docker volume permissions by copying to container-local directory..."
echo "   Source: $AIRSIM_ROOT"
echo "   Container workspace: $CONTAINER_BUILD_ROOT"

# Remove any existing container build directory
rm -rf "$CONTAINER_BUILD_ROOT"
# Copy source tree excluding large unnecessary directories for faster builds
echo "   Copying source (excluding large build/cache directories)..."
rsync -a --exclude='build_*/' --exclude='Binaries/' --exclude='Intermediate/' --exclude='.git/' --exclude='external/*/build/' --exclude='logs/' --exclude='*.log' --exclude='Docker/' --exclude='__pycache__/' "$AIRSIM_ROOT/" "$CONTAINER_BUILD_ROOT/"
# Change to container workspace
pushd "$CONTAINER_BUILD_ROOT" >/dev/null

set -e
set -x

debug=false
# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"
    case $key in
    --debug)
        debug=true
        shift # past argument
        ;;
    esac
done

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# check for rpclib
RPC_VERSION_FOLDER="rpclib-2.3.1"
if [ ! -d "./external/rpclib/$RPC_VERSION_FOLDER" ]; then
    echo "ERROR: new version of AirSim requires newer rpclib."
    echo "please run setup.sh first and then run build.sh again."
    exit 1
fi

# Configure Epic's UE5 toolchain for perfect ABI compatibility
setup_epic_toolchain() {
    echo "🔍 Configuring Epic's UE5 bundled toolchain for perfect ABI compatibility..."

    # Epic's UE5 toolchain paths
    EPIC_CLANG_PATH="/home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v23_clang-18.1.0-rockylinux8/x86_64-unknown-linux-gnu/bin"
    EPIC_LIBCPP_INCLUDE="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1"
    EPIC_LIBCPP_LIB="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/lib/Unix/x86_64-unknown-linux-gnu"

    # Verify Epic's clang exists
    if [[ ! -f "$EPIC_CLANG_PATH/clang" ]] || [[ ! -f "$EPIC_CLANG_PATH/clang++" ]]; then
        echo "❌ ERROR: Epic's bundled clang not found at $EPIC_CLANG_PATH"
        echo "   Make sure you're running this in Epic's UE5.5.4 container"
        exit 1
    fi

    # Verify Epic's libc++ headers exist
    if [[ ! -d "$EPIC_LIBCPP_INCLUDE" ]]; then
        echo "❌ ERROR: Epic's libc++ headers not found at $EPIC_LIBCPP_INCLUDE"
        echo "   Make sure you're running this in Epic's UE5.5.4 container"
        exit 1
    fi

    # Verify Epic's libc++ libraries exist
    if [[ ! -d "$EPIC_LIBCPP_LIB" ]]; then
        echo "❌ ERROR: Epic's libc++ libraries not found at $EPIC_LIBCPP_LIB"
        echo "   Make sure you're running this in Epic's UE5.5.4 container"
        exit 1
    fi

    echo "   ✅ Epic's clang-18.1.0: $EPIC_CLANG_PATH"
    echo "   ✅ Epic's libc++ headers: $EPIC_LIBCPP_INCLUDE"
    echo "   ✅ Epic's libc++ libraries: $EPIC_LIBCPP_LIB"

    # Verify Epic's clang version
    echo "🔍 Verifying Epic's clang version..."
    local version_output=$($EPIC_CLANG_PATH/clang --version 2>&1 | head -n1)
    echo "   Version: $version_output"

    # Test Epic's libc++ compilation
    echo "🔍 Testing Epic's libc++ compilation..."
    local test_flags="-stdlib=libc++ -nostdinc++ -I$EPIC_LIBCPP_INCLUDE -L$EPIC_LIBCPP_LIB"
    if echo '#include <memory>
int main(){auto p = std::make_unique<int>(42); return 0;}' | $EPIC_CLANG_PATH/clang++ $test_flags -x c++ - -o /tmp/test_epic_libcpp 2>/dev/null; then
        echo "   ✅ Epic's libc++ compilation test passed"
        rm -f /tmp/test_epic_libcpp
    else
        echo "❌ ERROR: Epic's libc++ compilation test failed"
        exit 1
    fi

    # Export Epic's toolchain
    export CC="$EPIC_CLANG_PATH/clang"
    export CXX="$EPIC_CLANG_PATH/clang++"
    export CMAKE_C_COMPILER="$EPIC_CLANG_PATH/clang"
    export CMAKE_CXX_COMPILER="$EPIC_CLANG_PATH/clang++"
}

# Setup Epic's UE5 toolchain
setup_epic_toolchain

# Configure Epic's libc++ with proper include/library paths and preprocessor definitions
EPIC_LIBCPP_INCLUDE="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1"
EPIC_LIBCPP_LIB="/home/ue4/UnrealEngine/Engine/Source/ThirdParty/Unix/LibCxx/lib/Unix/x86_64-unknown-linux-gnu"

# Epic's libc++ requires specific preprocessor definitions and paths
EPIC_CXX_FLAGS="-stdlib=libc++ -nostdinc++ -I$EPIC_LIBCPP_INCLUDE -D_LIBCPP_ABI_UNSTABLE -D_LIBCPP_DISABLE_AVAILABILITY"
EPIC_LINKER_FLAGS="-L$EPIC_LIBCPP_LIB -lc++ -lc++abi"

export CXXFLAGS="$EPIC_CXX_FLAGS"
export CMAKE_CXX_FLAGS="$EPIC_CXX_FLAGS"
export LDFLAGS="$EPIC_LINKER_FLAGS"
export CMAKE_EXE_LINKER_FLAGS="$EPIC_LINKER_FLAGS"
export CMAKE_SHARED_LINKER_FLAGS="$EPIC_LINKER_FLAGS"

echo "🔧 Epic's UE5 toolchain configuration:"
echo "   CC=$CC"
echo "   CXX=$CXX"
echo "   CXXFLAGS=$CXXFLAGS"
echo "   LDFLAGS=$LDFLAGS"
echo "   CMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS"
echo "   CMAKE_EXE_LINKER_FLAGS=$CMAKE_EXE_LINKER_FLAGS"
echo "   CMAKE_C_COMPILER=$CMAKE_C_COMPILER"
echo "   CMAKE_CXX_COMPILER=$CMAKE_CXX_COMPILER"
echo "   Epic libc++ include: $EPIC_LIBCPP_INCLUDE"
echo "   Epic libc++ library: $EPIC_LIBCPP_LIB"

# Use container's cmake - check multiple locations
if command -v cmake >/dev/null 2>&1; then
    CMAKE=$(which cmake)
elif [ -f /usr/bin/cmake ]; then
    CMAKE=/usr/bin/cmake
else
    echo "❌ ERROR: cmake not found in container"
    exit 1
fi

echo "📦 Using cmake: $CMAKE"

# variable for build output
if $debug; then
    build_dir=build_debug_libcpp
    CMAKE_BUILD_TYPE=Debug
else
    build_dir=build_release_libcpp
    CMAKE_BUILD_TYPE=Release
fi

#install EIGEN library
if [[ ! -d "./AirLib/deps/eigen3/Eigen" ]]; then
    echo "### Eigen is not installed. Please run setup.sh first."
    exit 1
fi

echo "putting build in $build_dir folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi

if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
fi

pushd $build_dir >/dev/null

# Configure CMAKE with Epic's UE5 toolchain and LLVM paths
EPIC_LLVM_DIR="/home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v23_clang-18.1.0-rockylinux8/x86_64-unknown-linux-gnu"
"$CMAKE" ../cmake \
    -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
    -DCMAKE_C_COMPILER="$CC" \
    -DCMAKE_CXX_COMPILER="$CXX" \
    -DCMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS" \
    -DCMAKE_EXE_LINKER_FLAGS="$CMAKE_EXE_LINKER_FLAGS" \
    -DCMAKE_SHARED_LINKER_FLAGS="$CMAKE_SHARED_LINKER_FLAGS" \
    -DLLVM_DIR="$EPIC_LLVM_DIR/lib/cmake/llvm" \
    -DCMAKE_PREFIX_PATH="$EPIC_LLVM_DIR" \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    || (popd && rm -r $build_dir && exit 1)

popd >/dev/null

pushd $build_dir >/dev/null
# Build with all available cores
make -j"$(nproc)"
popd >/dev/null

# Create output directories
if $debug; then
    folder_name="Debug"
else
    folder_name="Release"
fi

mkdir -p AirLib/lib/x64/$folder_name
mkdir -p AirLib/deps/rpclib/lib
mkdir -p AirLib/deps/MavLinkCom/lib

# Copy built libraries
cp $build_dir/output/lib/libAirLib.a AirLib/lib
cp $build_dir/output/lib/libMavLinkCom.a AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/librpc.a AirLib/deps/rpclib/lib/librpc.a

# Update AirLib/lib, AirLib/deps, Plugins folders with new binaries
rsync -a --delete $build_dir/output/lib/ AirLib/lib/x64/$folder_name
rsync -a --delete external/rpclib/$RPC_VERSION_FOLDER/include AirLib/deps/rpclib
rsync -a --delete MavLinkCom/include AirLib/deps/MavLinkCom
rsync -a --delete AirLib Unreal/Plugins/AirSim/Source
rm -rf Unreal/Plugins/AirSim/Source/AirLib/src

set +x

echo ""
echo " 🔄 Copying build results back to mounted volume..."
# Copy build results back to original mounted volume
popd >/dev/null  # Exit container workspace

echo "   Container build: $CONTAINER_BUILD_ROOT"
echo "   Mounted volume: $AIRSIM_ROOT"

# Copy critical build artifacts back to mounted volume
rsync -av --delete "$CONTAINER_BUILD_ROOT/AirLib/lib/" "$AIRSIM_ROOT/AirLib/lib/"
rsync -av --delete "$CONTAINER_BUILD_ROOT/AirLib/deps/" "$AIRSIM_ROOT/AirLib/deps/"
rsync -av --delete "$CONTAINER_BUILD_ROOT/Unreal/Plugins/AirSim/Source/" "$AIRSIM_ROOT/Unreal/Plugins/AirSim/Source/"

# Copy build directories for BuildCookRun testing
rsync -av "$CONTAINER_BUILD_ROOT/$build_dir/" "$AIRSIM_ROOT/$build_dir/"

echo ""
echo "================================================"
echo " AirSim built with Epic's UE5 bundled toolchain!"
echo " 🔗 Perfect ABI compatibility with UE5.5.4"
echo " 🔧 Using Epic's clang-18.1.0 + bundled libc++"
echo " 📁 Build artifacts copied to mounted volume"
echo "================================================"
echo ""