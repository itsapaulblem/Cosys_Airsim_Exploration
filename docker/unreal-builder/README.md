# AirSim UE5 Docker Build System

Automated Docker-based build system for compiling AirSim UE5 projects into standalone Linux executables using Epic Games' official UE5 containers.

## Quick Start - Choose Your Workflow

### 🐧 **Linux / WSL2 Host** (Recommended)

**Perfect for:** Clean builds with native Linux or WSL2 environments

```bash
cd workflows/linux-host
./build-xfs.sh
```

**What you get:**
- ✅ Full automation: AirLib → UE5 Plugin → Packaged Executable
- ✅ Epic toolchain integration (glibc 2.28 compatible)
- ✅ 3-layer dependency synchronization (libraries + headers)
- ✅ Container-isolated reproducible builds

📖 **[Complete Linux Workflow Guide](workflows/linux-host/docs/DOCKER_UE5_BUILD_GUIDE.md)**

---

### 🪟 **Windows Host with Docker**

**Perfect for:** Windows development machines with Docker Desktop + WSL2 backend

```bash
cd workflows/windows-host/scripts
./build-containerized.sh
```

**Special considerations:**
- ⚠️ Windows→Linux volume mount permission boundaries
- 🔧 User ID mapping for build artifacts
- 📁 Post-build ownership fixes may be required

📖 **[Windows Workflow Guide](workflows/windows-host/docs/WINDOWS_PERMISSIONS.md)** *(coming soon)*

---

## What This System Does

Automates the complete pipeline from AirSim source code to a standalone UE5 Linux executable:

```
AirLib Source → Build with Epic's Toolchain → Sync Dependencies →
  Copy Plugin → Build UE Project → Package Executable
```

### Key Features

- **Full Automation**: Single command builds from source to executable
- **Epic Toolchain Integration**: Uses UE5's bundled Clang for glibc 2.28 compatibility
- **3-Layer Dependency Management**: Ensures all libraries and headers are synchronized
- **Generic & Reusable**: Works with ANY AirSim UE5 project
- **Container Isolation**: Reproducible builds in controlled environment

### Docker Services

The `docker-compose.yml` defines specialized services for different tasks:

| Service | Purpose | Usage |
|---------|---------|-------|
| `xfs-builder` | Build Xfs project | `docker compose run --rm xfs-builder <script>` |
| `airsim-unreal-builder` | General AirLib/UE builds | `docker compose run --rm airsim-unreal-builder <script>` |
| `buildcookrun` | UE5 BuildCookRun operations | For custom build configurations |
| `airsim-headless` | Headless AirSim execution | For running packaged binaries |

---

## Directory Structure

```
docker/unreal-airsim/
├── workflows/
│   ├── linux-host/          ← Clean workflow for Linux/WSL2
│   │   ├── build-xfs.sh     (Entry point)
│   │   ├── scripts/
│   │   │   ├── build-airlib.sh
│   │   │   └── examples/build-xfs-example.sh
│   │   └── docs/
│   │       ├── DOCKER_UE5_BUILD_GUIDE.md
│   │       └── TROUBLESHOOTING.md
│   │
│   └── windows-host/        ← Permission-aware workflow for Windows
│       ├── scripts/
│       │   ├── build-and-package.sh
│       │   └── build-containerized.sh
│       └── docs/
│           └── WINDOWS_PERMISSIONS.md (coming soon)
│
├── utilities/               ← Shared tools
│   ├── test-headless.sh
│   └── launch-headless.sh
│
├── docker-compose.yml       ← Service definitions
├── Dockerfile               ← UE5 container setup
└── README.md                ← This file
```

---

## Prerequisites

### System Requirements

- **Docker** 24.0+ with Docker Compose
- **Memory**: 16GB+ RAM (32GB recommended for UE5 builds)
- **Disk**: 100GB+ free space for UE5 container and builds
- **OS**: Linux, WSL2, or Windows with Docker Desktop

### First-Time Setup

1. **Clone AirSim repository:**
   ```bash
   git clone <airsim-repo>
   cd Cosys_Airsim_Exploration/docker/unreal-airsim
   ```

2. **Build Docker image:**
   ```bash
   docker compose build airsim-unreal-builder
   ```

3. **Choose your workflow** based on host OS (see Quick Start above)

---

## Building Custom Projects

To build a different AirSim UE5 project (not Xfs):

### Linux Workflow

1. Copy the example build script:
   ```bash
   cd workflows/linux-host/scripts/examples
   cp build-xfs-example.sh build-myproject.sh
   ```

2. Update project paths in `build-myproject.sh`:
   ```bash
   PROJECT_PATH="/workspace/airsim/docker/unreal_executable/MyProject/MyProject.uproject"
   OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged/MyProject"
   ```

3. Run the build:
   ```bash
   docker compose run --rm xfs-builder \
     /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/examples/build-myproject.sh
   ```

---

## Troubleshooting

### Common Issues

| Issue | Quick Fix | Full Guide |
|-------|-----------|------------|
| **glibc symbol errors** | Rebuild AirLib in container | [TROUBLESHOOTING.md](workflows/linux-host/docs/TROUBLESHOOTING.md#2-glibc-version-mismatch) |
| **Missing Eigen/Dense** | Run updated `build-airlib.sh` | [TROUBLESHOOTING.md](workflows/linux-host/docs/TROUBLESHOOTING.md#6-missing-dependency-headers) |
| **Stale plugin libraries** | Script auto-recopies plugin | [TROUBLESHOOTING.md](workflows/linux-host/docs/TROUBLESHOOTING.md#5-stale-plugin-copy--library-mismatch) |
| **Permission errors** | Use Windows workflow | [Windows Guide](workflows/windows-host/docs/WINDOWS_PERMISSIONS.md) |

### Getting Help

1. **Check Build Logs**: Save output with `2>&1 | tee build.log`
2. **Search Issues**: Look for error text in [AirSim GitHub issues](https://github.com/microsoft/AirSim/issues)
3. **Review Prerequisites**: Ensure Docker has sufficient resources
4. **Clean Rebuild**: See [Clean Build Process](workflows/linux-host/docs/TROUBLESHOOTING.md#clean-build-process)

---

## Architecture Deep Dive

### 3-Layer Dependency Distribution

The build system synchronizes dependencies across three critical layers:

**Layer 1: AirLib Build Output** (`/workspace/airsim/AirLib/`)
- Fresh compilation artifacts from Epic's toolchain
- Libraries: `lib/*.a`
- Headers: `deps/*/include/`, `deps/eigen3/Eigen/`

**Layer 2: Plugin Source** (`/workspace/airsim/Unreal/Plugins/AirSim/Source/AirLib/`)
- Master copy that AirSim's build system expects
- Synced from Layer 1 after AirLib build

**Layer 3: Project Plugin** (`/workspace/airsim/docker/unreal_executable/Xfs/Plugins/AirSim/`)
- What UE5 actually compiles and links against
- Copied from Layer 2 before UE5 build

**Critical**: All three layers must have **both libraries AND headers** synchronized.

📖 **[Full Architecture Documentation](workflows/linux-host/docs/DOCKER_UE5_BUILD_GUIDE.md#architecture)**

---

## Advanced Topics

### Custom Build Configurations

Edit build scripts to change:
- **Platform**: `PLATFORM="Linux"` (could be `Win64`, `Mac`)
- **Configuration**: `CONFIG="Development"` (could be `Shipping`, `Debug`)
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

### Vulkan Shader Model 6 (SM6) Builds

**Overview**: Configure UE5 projects for modern Vulkan 1.2+ with Shader Model 6 support, optimized for headless containerized execution.

#### Why Vulkan SM6?
- **Headless-Compatible**: No Ray Tracing hardware requirements
- **Modern SPIR-V**: Shader Model 6 features via Vulkan's SPIR-V compiler
- **37% Fewer Shaders**: SM6-only (no SM5 fallback) reduces compile time
- **Container-Optimized**: Works with software Vulkan drivers (llvmpipe, lavapipe)

#### Configuration Steps

**1. Configure DefaultEngine.ini**

File: `docker/unreal_executable/Xfs/Config/DefaultEngine.ini`

```ini
[/Script/Engine.RendererSettings]
; Disable Ray Tracing completely (requires GPU hardware)
r.RayTracing=False
r.RayTracing.AllowInGame=False
r.RayTracing.Shadows=False
r.Lumen.HardwareRayTracing=False

[/Script/LinuxTargetPlatform.LinuxTargetSettings]
; Vulkan SM6 ONLY - No SM5, No Ray Tracing
; Explicit shader format specification to prevent engine defaults
TargetedRHIs=SF_VULKAN_SM6
+TargetedShaderFormats=SF_VULKAN_SM6
-TargetedShaderFormats=SF_VULKAN_SM5
```

**2. Update Build Script**

File: `workflows/linux-host/scripts/examples/build-xfs-example.sh`

Add `-CookFlavor=VULKAN_SM6` flag to BuildCookRun command:

```bash
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
```

**3. Clean Previous Builds**

Before building with new configuration, remove all cached data:

```bash
rm -rf docker/unreal_executable/Xfs/Saved/*
rm -rf docker/unreal_executable/Xfs/Intermediate/*
rm -rf docker/unreal_executable/Xfs/Binaries/*
rm -rf docker/unreal_executable/Xfs/DerivedDataCache
```

**4. Build with Vulkan SM6**

```bash
cd docker/unreal-airsim
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/examples/build-xfs-example.sh \
  2>&1 | tee /tmp/vulkan_sm6_build.log
```

#### Verification

```bash
# Check shader library targets (should only see SF_VULKAN_SM6)
grep "Shader Library.*Stats" /tmp/vulkan_sm6_build.log

# Confirm no SM5 compilation (should be 0)
grep -c "SF_VULKAN_SM5" /tmp/vulkan_sm6_build.log

# Confirm no Ray Tracing (should be 0)
grep -c "RayTracing" /tmp/vulkan_sm6_build.log

# Verify packaged executable
ls -lh docker/unreal_executable/packaged-sm6/Xfs/Linux/Xfs/Binaries/Linux/Xfs
```

#### Output Structure

```
docker/unreal_executable/packaged-sm6/Xfs/
└── Linux/
    └── Xfs/
        ├── Binaries/
        │   └── Linux/
        │       └── Xfs              ← Standalone executable (282MB)
        ├── Content/
        │   └── Paks/
        │       └── Xfs-Linux.pak    ← Cooked content
        └── Engine/
            └── Binaries/
                └── ThirdParty/      ← Shared libraries
```

#### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| SM5 shaders still compiling | Cached DDC | Clean `Saved/`, `Intermediate/`, `DerivedDataCache/` |
| Ray Tracing shader errors | Engine defaults override | Add all 4 RT disable flags to DefaultEngine.ini |
| ShaderCompileWorker crashes | GPU hardware required | Ensure `r.RayTracing=False` |

#### Performance Comparison

| Configuration | Shader Count | Compile Time | Container Compatible |
|---------------|--------------|--------------|---------------------|
| **SM6 + SM5 + RT** | ~14,300 | ~45 min | ❌ (RT requires GPU) |
| **SM6 + SM5** | ~11,000 | ~30 min | ✅ (no RT) |
| **SM6 only** | ~9,000 | ~20 min | ✅ (optimal) |

### Headless Execution

Run packaged executables in headless mode:

```bash
docker compose up -d airsim-headless

# Or manually with Vulkan software rendering
docker compose run --rm airsim-headless bash
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
./docker/unreal_executable/packaged-sm6/Xfs/Linux/Xfs/Binaries/Linux/Xfs -nullrhi
```

---

## References

- [Epic Games UE5 Containers](https://github.com/EpicGames/UnrealContainers)
- [Linux Workflow Guide](workflows/linux-host/docs/DOCKER_UE5_BUILD_GUIDE.md)
- [Troubleshooting Reference](workflows/linux-host/docs/TROUBLESHOOTING.md)
- [AirSim Build Documentation](https://microsoft.github.io/AirSim/build_linux/)

---

**Last Updated**: 2025-10-07
**Tested With**: UE 5.5.4 (Vulkan SM6), Docker 24.0.7, Ubuntu 22.04, AirSim (Cosys-Lab fork)
