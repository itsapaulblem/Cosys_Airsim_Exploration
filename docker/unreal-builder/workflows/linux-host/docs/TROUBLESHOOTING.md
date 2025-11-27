# UE5 Project Builder - Troubleshooting Guide

## Common Build Errors

### 1. CRLF Line Ending Issues

**Error:**
```
/bin/bash^M: bad interpreter: No such file or directory
```

**Cause:** Script files have Windows-style CRLF line endings instead of Unix LF endings.

**Solution:**
```bash
# Fix a specific script
sed -i 's/\r$//' /path/to/script.sh

# Fix all scripts in a directory
find /path/to/scripts -type f -name "*.sh" -exec sed -i 's/\r$//' {} \;
```

**Prevention:** Configure Git to handle line endings:
```bash
git config --global core.autocrlf input  # On Linux/WSL
git config --global core.autocrlf true   # On Windows
```

---

### 2. glibc Version Mismatch

**Error:**
```
undefined symbol: __isoc23_strtol
undefined symbol: __isoc23_strtoul
```

**Cause:** AirLib was compiled on host with glibc 2.38+, but container has glibc 2.28.

**Solution:** Build AirLib inside the container using Epic's toolchain:
```bash
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/build-airlib.sh
```

**Verification:**
```bash
# Check for incompatible symbols
nm /workspace/airsim/AirLib/lib/librpc.a | grep __isoc23

# If the grep returns nothing, the build is compatible
# If it returns matches, rebuild AirLib in the container
```

**Prevention:** Always build AirLib in the same environment where you'll build UE projects.

---

### 3. Epic Toolchain Requirement

**Error:**
```
ERROR: Use Epic's bundled toolchain, not system tools

CMake Error: The C compiler "/usr/local/bin/clang" is not able to compile a simple test program.
```

**Cause:** Epic's UE5 container has wrapper scripts at `/usr/local/bin/gcc` and `/usr/local/bin/clang` that actively block system compiler usage. AirSim's `build.sh` always sets `CC` and `CXX` environment variables (either to `gcc`/`g++` with `--gcc` flag, or to `clang`/`clang++` without flags), which triggers these enforcement wrappers.

**Solution:** Build AirLib using CMake directly (bypassing `build.sh`) to allow CMake to auto-discover Epic's bundled toolchain:

```bash
# WRONG - build.sh sets CC/CXX to system compilers (triggers wrappers)
./build.sh          # Sets CC=clang → /usr/local/bin/clang → ERROR
./build.sh --gcc    # Sets CC=gcc → /usr/local/bin/gcc → ERROR

# CORRECT - Call CMake directly, let it discover Epic's Clang
mkdir -p build_release && cd build_release
cmake ../cmake -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

**If you're using build-airlib.sh:** Already fixed - it bypasses `build.sh` and calls CMake directly.

**Epic's Toolchain Discovery:**
- CMake searches standard paths
- Finds Epic's Clang: `/home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/.../clang`
- Uses Epic's libc++ and glibc 2.28 (container compatible)

**Why This Matters:** Epic's toolchain ensures ABI compatibility with UE5.5's custom standard library and the container's glibc 2.28.

---

### 4. UE5 Build System API Errors

**Error:**
```
error CS0103: The name 'PublicLibraryPaths' does not exist in the current context
error CS0103: The name 'PublicCompileEnvironment' does not exist in the current context
error CS0103: The name 'PublicLinkEnvironment' does not exist in the current context
```

**Cause:** AirSim plugin using deprecated UE4 Build.cs API.

**Solution:** Update `Unreal/Plugins/AirSim/Source/AirSim.Build.cs`:

```csharp
// UE4 (Deprecated)           → UE5.5 (Working)
PublicLibraryPaths            → PublicSystemLibraryPaths
PublicCompileEnvironment      → (Remove - use global environment)
PublicLinkEnvironment         → (Remove - use global environment)
```

**Specific Changes:**
- Line 173: `PublicLibraryPaths.Add(...)` → `PublicSystemLibraryPaths.Add(...)`
- Remove lines using `PublicCompileEnvironment` and `PublicLinkEnvironment`

---

### 5. Stale Plugin Copy / Library Mismatch

**Error:**
```
ld.lld: error: undefined symbol: __isoc23_strtol
>>> referenced by server.cc in archive librpc.a
```
or other linker errors despite successful AirLib build and verification.

**Cause:** Build script only copied plugin once. After rebuilding AirLib with Epic's toolchain, the plugin directory still contains OLD libraries from previous host build. UE5 links against the stale libraries in the plugin, not the newly built ones.

**Root Cause Detail:**
- Main AirLib: `/workspace/airsim/AirLib/lib/librpc.a` (newly built, clean)
- Plugin copy: `/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a` (old, contaminated)
- UE5 Build.cs links against the plugin copy, not main AirLib

**Solution (Automatic):**
The build script now automatically removes and recopies the plugin with fresh libraries every build (Step 0.5 in `build-xfs-example.sh`).

**Solution (Manual - if needed):**
```bash
# Delete old plugin copy to force fresh copy with new libraries
rm -rf /workspace/airsim/docker/unreal_executable/YOUR_PROJECT/Plugins/AirSim/

# Next build will copy the latest version
```

**Prevention:**
- Build script now includes Step 0.5: Always removes and recopies plugin to sync libraries
- Includes verification check for glibc 2.38+ symbols in copied plugin libraries
- Shows timestamp of source libraries being copied

**Verification:**
```bash
# Check timestamps - plugin should match main AirLib
stat -c "%y %n" /workspace/airsim/AirLib/lib/librpc.a
stat -c "%y %n" /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a

# Verify no glibc 2.38+ symbols in BOTH locations
nm /workspace/airsim/AirLib/lib/librpc.a | grep __isoc23
nm /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a | grep __isoc23
# Both should return nothing (exit code 1)
```

---

### 6. Missing Dependency Headers

**Error:**
```
fatal error: 'Eigen/Dense' file not found
   14 | #include "Eigen/Dense"
      |          ^~~~~~~~~~~~~~
```
or
```
Warning: Referenced directory '/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/eigen3' does not exist
Warning: Referenced directory '/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/include' does not exist
Warning: Referenced directory '/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/rpclib/include' does not exist
```

**Cause:** Build scripts were only syncing library files (.a), not header files (.h, .hpp). UE5 needs headers to **compile** the plugin source code, but they were missing from the plugin directories.

**Root Cause Detail:**
- AirLib build creates libraries in `AirLib/lib/` and `AirLib/deps/*/lib/`
- Headers exist in `AirLib/deps/eigen3/Eigen/`, `AirLib/deps/*/include/`
- Previous scripts only copied `.a` files, not header directories
- UE5 compilation failed because `#include` directives couldn't find headers

**Solution (Automatic):**
The build-airlib.sh script now syncs headers alongside libraries:
```bash
# After line 114 in build-airlib.sh:
cp -r AirLib/deps/eigen3/Eigen Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3/
cp -r AirLib/deps/MavLinkCom/include Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/
cp -r AirLib/deps/rpclib/include Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/
```

**Solution (Manual - if needed):**
```bash
# Sync headers from AirLib to plugin source
cd /workspace/airsim

# Eigen headers
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3
cp -r AirLib/deps/eigen3/Eigen Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3/

# MavLinkCom headers
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom
cp -r AirLib/deps/MavLinkCom/include Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/

# rpclib headers
mkdir -p Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib
cp -r AirLib/deps/rpclib/include Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/

# Remove stale project plugin copy to force fresh sync
rm -rf docker/unreal_executable/Xfs/Plugins/AirSim/
```

**Verification:**
```bash
# Check headers exist in all required locations
ls -la /workspace/airsim/AirLib/deps/eigen3/Eigen/
ls -la /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3/Eigen/
ls -la /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/eigen3/Eigen/

ls -la /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/include/
ls -la /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/include/
```

**Prevention:**
- Always run updated build-airlib.sh that syncs both libraries AND headers
- Remove and recopy project plugin after rebuilding AirLib
- build-xfs-example.sh Step 0.5 ensures fresh plugin with all dependencies

---

### 7. Missing AirLib Libraries

**Error:**
```
❌ ERROR: AirSim plugin source not found
Please build AirLib first: ./build.sh in repository root
```

**Cause:** AirLib not built yet.

**Solution:**
```bash
# Build AirLib inside container (recommended)
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/build-airlib.sh

# Or build on host (may cause glibc issues)
cd /workspace/airsim
./setup.sh
./build.sh --gcc
```

---

### 8. Docker Permission Issues

**Error:**
```
Permission denied: /workspace/airsim/...
```

**Cause:** Files created inside container with wrong ownership.

**Solution:**
```bash
# Fix ownership of files (run on host)
sudo chown -R $USER:$USER /path/to/workspace/

# Or run container with host user ID
docker compose run --rm --user $(id -u):$(id -g) xfs-builder ...
```

---

### 9. Out of Memory During Build

**Error:**
```
c++: fatal error: Killed signal terminated program cc1plus
```

**Cause:** UE5 compilation exhausted system memory.

**Solution:**
```bash
# Increase Docker memory limit (Docker Desktop Settings → Resources)
# Recommended: 16GB+ for UE5 builds

# Or limit parallel compilation
# Edit your .uproject file, add:
# "MaxParallelActions": 2
```

---

### 10. Vulkan Driver Issues

**Error:**
```
LogVulkanRHI: Error: Failed to create Vulkan instance
```

**Cause:** Missing or incompatible Vulkan drivers in container.

**Solution:**
```bash
# Install Vulkan dependencies (already in Dockerfile)
apt-get install -y libvulkan1 vulkan-utils

# For headless rendering, use Vulkan software rendering
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
```

---

## Diagnostic Commands

### Check glibc Version
```bash
# Inside container
ldd --version

# Check for incompatible symbols in library
nm /path/to/library.a | grep __isoc23
```

### Verify AirLib Build
```bash
ls -lh /workspace/airsim/AirLib/lib/
file /workspace/airsim/AirLib/lib/librpc.a
```

### Check Docker Container Status
```bash
docker compose ps
docker compose logs xfs-builder
```

### Monitor Build Progress
```bash
# Follow build logs in real-time
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/examples/build-xfs-example.sh \
  2>&1 | tee build.log
```

### Check UE5 Installation
```bash
# Inside container
ls -la /home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh
/home/ue4/UnrealEngine/Engine/Binaries/Linux/UnrealEditor --version
```

---

## Clean Build Process

If all else fails, perform a complete clean rebuild:

```bash
# Step 1: Clean AirLib
rm -rf /workspace/airsim/AirLib/lib
rm -rf /workspace/airsim/external/rpclib/build

# Step 2: Clean project builds
rm -rf /workspace/airsim/docker/unreal_executable/YOUR_PROJECT/Binaries
rm -rf /workspace/airsim/docker/unreal_executable/YOUR_PROJECT/Intermediate
rm -rf /workspace/airsim/docker/unreal_executable/YOUR_PROJECT/Saved/Cooked
rm -rf /workspace/airsim/docker/unreal_executable/YOUR_PROJECT/Plugins/AirSim

# Step 3: Clean packaged output
rm -rf /workspace/airsim/docker/unreal_executable/packaged/*

# Step 4: Rebuild from scratch
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/build-airlib.sh

docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/examples/build-xfs-example.sh
```

---

## Getting Help

If you encounter an error not covered here:

1. **Check Build Logs:** Save complete output with `2>&1 | tee build.log`
2. **Search Error Message:** Look for exact error text in AirSim GitHub issues
3. **Verify Prerequisites:** Ensure all steps in PROJECT_BUILD_GUIDE.md completed
4. **Docker Environment:** Confirm container has sufficient resources (CPU/RAM)
5. **File an Issue:** Include build logs, system info, and exact reproduction steps
