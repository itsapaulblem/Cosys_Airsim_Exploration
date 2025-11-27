# AirSim UE5 Docker Build System - Complete Guide

This guide documents the complete build pipeline for compiling AirSim UE5 projects into standalone executables using Epic Games' official UE5 Docker containers.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Critical Issues Solved](#critical-issues-solved)
4. [Quick Start](#quick-start)
5. [Detailed Build Process](#detailed-build-process)
6. [Verification](#verification)
7. [Troubleshooting](#troubleshooting)

---

## Overview

### What This System Does

Automates the complete pipeline from AirSim source code to a standalone UE5 Linux executable:

```
AirLib Source → Build with Epic's Toolchain → Sync Dependencies →
  Copy Plugin → Build UE Project → Package Executable
```

###Key Features

- ✅ **Full Automation**: Single command builds from source to executable
- ✅ **Epic Toolchain Integration**: Uses UE5's bundled Clang for glibc 2.28 compatibility
- ✅ **3-Layer Dependency Management**: Ensures all libraries and headers are synchronized
- ✅ **Generic & Reusable**: Works with ANY AirSim UE5 project
- ✅ **Container Isolation**: Reproducible builds in controlled environment

---

## Architecture

### 3-Layer Dependency Distribution

The build system manages dependencies across three critical layers:

```
┌─────────────────────────────────────────────────────────────┐
│ LAYER 1: AirLib Build Output                                │
│ /workspace/airsim/AirLib/                                   │
│   ├── lib/                                                   │
│   │   ├── libAirLib.a                                        │
│   │   ├── librpc.a                                           │
│   │   └── libMavLinkCom.a                                    │
│   └── deps/                                                  │
│       ├── eigen3/Eigen/          [HEADERS]                   │
│       ├── rpclib/                                            │
│       │   ├── lib/librpc.a                                   │
│       │   └── include/           [HEADERS]                   │
│       └── MavLinkCom/                                        │
│           ├── lib/libMavLinkCom.a                            │
│           └── include/           [HEADERS]                   │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼ (build-airlib.sh syncs libs + headers)
┌─────────────────────────────────────────────────────────────┐
│ LAYER 2: Plugin Source (Master Copy)                        │
│ /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/      │
│   ├── lib/                       [SYNCED from Layer 1]      │
│   │   ├── libAirLib.a                                        │
│   │   ├── librpc.a                                           │
│   │   └── libMavLinkCom.a                                    │
│   └── deps/                      [SYNCED from Layer 1]      │
│       ├── eigen3/Eigen/          [HEADERS]                   │
│       ├── rpclib/                                            │
│       │   ├── lib/librpc.a                                   │
│       │   └── include/           [HEADERS]                   │
│       └── MavLinkCom/                                        │
│           ├── lib/libMavLinkCom.a                            │
│           └── include/           [HEADERS]                   │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼ (build-xfs-example.sh copies complete plugin)
┌─────────────────────────────────────────────────────────────┐
│ LAYER 3: Project Plugin Copy                                │
│ /workspace/airsim/docker/unreal_executable/Xfs/             │
│   Plugins/AirSim/Source/AirLib/                             │
│   ├── lib/                       [COPIED from Layer 2]      │
│   │   ├── libAirLib.a            ← UE5 links these          │
│   │   ├── librpc.a                                           │
│   │   └── libMavLinkCom.a                                    │
│   └── deps/                      [COPIED from Layer 2]      │
│       ├── eigen3/Eigen/          ← UE5 compiles with these  │
│       ├── rpclib/include/        ← Headers required!        │
│       └── MavLinkCom/include/                                │
└─────────────────────────────────────────────────────────────┘
```

### Why 3 Layers?

1. **Layer 1 (Build Output)**: Fresh compilation artifacts from Epic's toolchain
2. **Layer 2 (Plugin Source)**: Master copy that AirSim's build system expects
3. **Layer 3 (Project Copy)**: What UE5 actually compiles and links against

**Critical**: All three layers must have **both libraries AND headers** synchronized, or the build will fail.

---

## Critical Issues Solved

### 1. Epic Toolchain Wrapper Enforcement

**Problem**: Epic's container has wrapper scripts at `/usr/local/bin/clang` and `/usr/local/bin/gcc` that block system compiler usage. AirSim's `build.sh` sets `CC`/`CXX` environment variables, triggering these wrappers.

**Solution**: Bypass `build.sh` entirely and call CMake directly:

```bash
# WRONG - triggers wrappers
./build.sh          # Sets CC=clang → ERROR
./build.sh --gcc    # Sets CC=gcc → ERROR

# CORRECT - CMake auto-discovers Epic's Clang
mkdir -p build_release && cd build_release
cmake ../cmake -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

CMake finds: `/home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/.../clang`

### 2. glibc Version Mismatch

**Problem**: Host system may have glibc 2.38+, but Epic's container uses glibc 2.28. Libraries compiled on host contain incompatible `__isoc23_*` symbols.

**Error**:
```
ld.lld: error: undefined symbol: __isoc23_strtol
>>> referenced by server.cc in archive librpc.a
```

**Solution**: Build AirLib **inside the container** with Epic's toolchain, which uses glibc 2.28.

**Verification**:
```bash
# Check for incompatible symbols (should return nothing)
nm AirLib/lib/librpc.a | grep __isoc23
```

### 3. Missing Dependency Headers

**Problem**: UE5 needs header files to **compile** the plugin source code, but previous build scripts only copied libraries (.a files).

**Error**:
```
fatal error: 'Eigen/Dense' file not found
Referenced directory '.../AirLib/deps/eigen3' does not exist
Referenced directory '.../AirLib/deps/MavLinkCom/include' does not exist
Referenced directory '.../AirLib/deps/rpclib/include' does not exist
```

**Solution**: Sync header directories alongside libraries:

```bash
# build-airlib.sh now syncs:
cp -r AirLib/deps/eigen3/Eigen Unreal/Plugins/AirSim/Source/AirLib/deps/eigen3/
cp -r AirLib/deps/MavLinkCom/include Unreal/Plugins/AirSim/Source/AirLib/deps/MavLinkCom/
cp -r AirLib/deps/rpclib/include Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/
```

### 4. Stale Plugin Library Copies

**Problem**: Project's plugin copy (Layer 3) becomes stale after rebuilding AirLib. UE5 links against old, incompatible libraries.

**Solution**: Always remove and recopy the complete plugin with fresh dependencies:

```bash
# build-xfs-example.sh Step 0.5
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/
cp -r /workspace/airsim/Unreal/Plugins/AirSim /workspace/airsim/docker/unreal_executable/Xfs/Plugins/
```

---

## Quick Start

### Prerequisites

1. **Docker** installed and running
2. **AirSim repository** cloned
3. **Docker image** built:
   ```bash
   cd docker/unreal-airsim
   docker compose build airsim-unreal-builder
   ```

### One-Command Build

```bash
cd docker/unreal-airsim

# Build the example Xfs project
docker compose run --rm xfs-builder ./build-xfs.sh
```

This single command:
1. ✅ Checks/builds AirLib with Epic's Clang (glibc 2.28 compatible)
2. ✅ Syncs all dependencies (libraries + headers) to plugin source
3. ✅ Copies fresh plugin to project
4. ✅ Builds, cooks, stages, and packages the UE project
5. ✅ Outputs executable to `docker/unreal_executable/packaged/Xfs/`

**Estimated Time**: 30-60 minutes (depending on system performance)

---

## Detailed Build Process

### Step 1: Build AirLib Inside Container

```bash
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/build-airlib.sh
```

**What this does**:
1. Runs `./setup.sh` (installs dependencies)
2. Calls CMake directly (bypasses build.sh to avoid wrapper issues)
3. Builds with Epic's Clang (`/home/ue4/UnrealEngine/Engine/Extras/...`)
4. Copies libraries to Layer 1 (`AirLib/lib/`, `AirLib/deps/*/lib/`)
5. Syncs libraries to Layer 2 (`Unreal/Plugins/AirSim/Source/AirLib/`)
6. **NEW**: Syncs header directories to Layer 2
7. Verifies no `__isoc23` symbols in all locations

**Output**:
```
✅ AirLib Built Successfully!
✅ AirLib/lib/librpc.a - compatible with container
✅ Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a - compatible with container
✅ Synced eigen3/Eigen/ headers to plugin deps/
✅ Synced MavLinkCom/include/ headers to plugin deps/
✅ Synced rpclib/include/ headers to plugin deps/
```

### Step 2: Build UE5 Project

```bash
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/examples/build-xfs-example.sh
```

**What this does**:

**Step 0**: Verify AirLib exists and is compatible
- Checks for `AirLib/lib/librpc.a`
- Scans for `__isoc23` symbols
- Rebuilds if incompatible

**Step 0.5**: Copy Fresh Plugin (with all dependencies)
- Removes stale plugin copy: `rm -rf Xfs/Plugins/AirSim/`
- Copies complete plugin from Layer 2 → Layer 3
- Includes libraries AND headers
- Verifies library compatibility

**Step 1**: Clean Previous Build
- Removes `Binaries/`, `Intermediate/`, `Saved/Cooked/`, `Saved/StagedBuilds/`

**Step 2**: Build, Cook, and Package
- Runs `/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun`
- Compiles plugin source using headers from Layer 3
- Links against libraries from Layer 3
- Cooks content, stages assets, creates .pak files
- Archives to output directory

**Output**:
```
✅ Build Complete!
📁 Packaged output: /workspace/airsim/docker/unreal_executable/packaged/Xfs/
🎮 Executable: .../LinuxNoEditor/Xfs/Binaries/Linux/Xfs
```

### Step 3: Run the Executable

```bash
cd docker/unreal_executable/packaged/Xfs/LinuxNoEditor
./Xfs/Binaries/Linux/Xfs -windowed -ResX=1920 -ResY=1080
```

---

## Verification

### Verify AirLib Build

```bash
# Check libraries exist
ls -lh /workspace/airsim/AirLib/lib/
# Should show: libAirLib.a, librpc.a, libMavLinkCom.a

# Check no incompatible symbols
nm /workspace/airsim/AirLib/lib/librpc.a | grep __isoc23
# Should return nothing (exit code 1)

# Check headers exist
ls -la /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps/
# Should show: eigen3/, rpclib/include/, MavLinkCom/include/
```

### Verify Plugin Sync

```bash
# Check timestamps match (plugin should be fresh)
stat -c "%y %n" /workspace/airsim/AirLib/lib/librpc.a
stat -c "%y %n" /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/lib/librpc.a
# Timestamps should be identical

# Verify headers copied to project
ls -la /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/eigen3/
# Should show Eigen/ directory
```

### Verify UE5 Build

```bash
# Check executable created
ls -lh /workspace/airsim/docker/unreal_executable/packaged/Xfs/LinuxNoEditor/Xfs/Binaries/Linux/Xfs
# Should show executable with +x permissions

# Check packaged size
du -sh /workspace/airsim/docker/unreal_executable/packaged/Xfs/
# Should be ~3-5 GB for Development builds
```

---

## Troubleshooting

See [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) for complete troubleshooting guide.

### Quick Diagnostics

```bash
# Check glibc version in container
docker compose run --rm xfs-builder ldd --version

# Check for incompatible symbols in all layers
nm /workspace/airsim/AirLib/lib/librpc.a | grep __isoc23
nm /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a | grep __isoc23
nm /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a | grep __isoc23

# Verify Epic's toolchain being used
docker compose run --rm xfs-builder which clang
# Should show: /home/ue4/UnrealEngine/Engine/Extras/ThirdPartyNotUE/SDKs/.../clang
```

### Clean Rebuild Process

If encountering persistent issues:

```bash
# Step 1: Clean all build artifacts
rm -rf /workspace/airsim/AirLib/lib
rm -rf /workspace/airsim/AirLib/build_release
rm -rf /workspace/airsim/external/rpclib/build
rm -rf /workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/deps
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Binaries
rm -rf /workspace/airsim/docker/unreal_executable/Xfs/Intermediate
rm -rf /workspace/airsim/docker/unreal_executable/packaged/*

# Step 2: Rebuild from scratch
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/scripts/build-airlib.sh

docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/build-xfs.sh
```

---

## Advanced Topics

### Building Other Projects

To build a different AirSim UE project (not Xfs):

1. Copy the example build script:
   ```bash
   cp scripts/examples/build-xfs-example.sh scripts/examples/build-myproject.sh
   ```

2. Update project paths in the script:
   ```bash
   PROJECT_PATH="/workspace/airsim/docker/unreal_executable/MyProject/MyProject.uproject"
   OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged/MyProject"
   ```

3. Run the build:
   ```bash
   docker compose run --rm xfs-builder \
     /workspace/airsim/docker/unreal-airsim/scripts/examples/build-myproject.sh
   ```

### Custom Build Configurations

Edit build scripts to change:

- **Platform**: `PLATFORM="Linux"` → Could be `Win64`, `Mac`
- **Configuration**: `CONFIG="Development"` → Could be `Shipping`, `Debug`
- **Cook Options**: Add `-allmaps`, `-iterate`, `-compressed`, etc.

Example Shipping build:
```bash
/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
  -project="${PROJECT_PATH}" \
  -platform=Linux \
  -clientconfig=Shipping \
  -serverconfig=Shipping \
  -build -cook -stage -pak -archive \
  -compressed \
  -archivedirectory="${OUTPUT_DIR}"
```

---

## References

- [Epic Games UE5 Containers](https://github.com/EpicGames/UnrealContainers)
- [PROJECT_BUILD_GUIDE.md](./PROJECT_BUILD_GUIDE.md) - Original Xfs project guide
- [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) - Complete troubleshooting reference
- [AirSim Build Documentation](https://microsoft.github.io/AirSim/build_linux/)

---

**Last Updated**: 2025-10-03
**Tested With**: UE 5.5.4, Docker 24.0.7, Ubuntu 22.04, AirSim (Cosys-Lab fork)
