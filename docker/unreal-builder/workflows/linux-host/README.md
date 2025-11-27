# Linux/WSL2 Workflow for AirSim UE5 Builds

This directory contains the clean, recommended workflow for building AirSim UE5 projects on Linux or WSL2 environments.

## Quick Start

```bash
# From this directory
./build-xfs.sh

# Or from docker/unreal-airsim directory
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/workflows/linux-host/build-xfs.sh
```

## What Happens

1. **AirLib Build** - Compiles AirLib with Epic's toolchain (glibc 2.28 compatible)
2. **Dependency Sync** - Syncs libraries AND headers across all 3 layers
3. **Plugin Copy** - Copies fresh plugin with all dependencies to project
4. **UE5 Build** - Compiles, cooks, and packages the UE5 project
5. **Output** - Standalone Linux executable ready to run

## Files

```
workflows/linux-host/
├── build-xfs.sh                    # Main entry point (wrapper)
├── scripts/
│   ├── build-airlib.sh             # Builds AirLib with Epic's toolchain
│   └── examples/
│       └── build-xfs-example.sh    # Builds and packages Xfs project
└── docs/
    ├── DOCKER_UE5_BUILD_GUIDE.md   # Complete architecture and process guide
    └── TROUBLESHOOTING.md          # Common issues and solutions
```

## Documentation

📖 **[Complete Build Guide](docs/DOCKER_UE5_BUILD_GUIDE.md)** - Full architecture, process, and advanced topics

📖 **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Solutions to common build issues

## Building Custom Projects

1. Copy the example script:
   ```bash
   cp scripts/examples/build-xfs-example.sh scripts/examples/build-myproject.sh
   ```

2. Update paths in your script:
   ```bash
   PROJECT_PATH="/workspace/airsim/docker/unreal_executable/MyProject/MyProject.uproject"
   OUTPUT_DIR="/workspace/airsim/docker/unreal_executable/packaged/MyProject"
   ```

3. Run the build:
   ```bash
   docker compose run --rm xfs-builder \
     /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/examples/build-myproject.sh
   ```

## Key Benefits of This Workflow

✅ **Clean & Simple** - No permission workarounds needed
✅ **Fully Automated** - Single command from source to executable
✅ **Epic Toolchain** - Uses UE5's bundled Clang (glibc 2.28 compatible)
✅ **3-Layer Sync** - Ensures libraries + headers are synchronized
✅ **Reproducible** - Container-isolated builds

---

**See [main README](../../README.md) for system overview and prerequisites**
