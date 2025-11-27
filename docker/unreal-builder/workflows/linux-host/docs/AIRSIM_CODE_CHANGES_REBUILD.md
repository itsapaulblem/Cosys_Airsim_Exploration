# AirSim Code Changes: Complete Rebuild Workflow

## Overview

When making changes to AirSim source code (especially AirLib header files), you must **rebuild both AirLib AND the Unreal executable** to see changes take effect.

## Why Both Rebuilds Are Required

### Build Architecture

```
Source Code (AirLib/*.hpp)
         ↓
    [build-airlib.sh]
         ↓
  AirLib Libraries (*.a)
         ↓
    [Copy to Plugin]
         ↓
Unreal Plugin (compiled)
         ↓
   [BuildCookRun UAT]
         ↓
Packaged Executable (Blocks)
         ↓
  [Docker Container Runs]
```

**Critical Insight**: The packaged Unreal executable is a **monolithic binary** with AirLib compiled into it. Just rebuilding AirLib libraries updates the `.a` files, but the **executable still contains old code** until you rebuild it.

## Complete Rebuild Workflow

**RECOMMENDED: Use the Nuclear Rebuild Master Script**

For the most reliable rebuild (especially after permission issues or first-time setup):

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration

# Complete automated rebuild with verification
sudo ./docker/unreal-builder/workflows/linux-host/scripts/nuclear-rebuild-complete.sh
```

**Time:** 50-80 minutes
**What it does:**
- Stops all services
- Nuclear clean of packaged directory (removes ALL permission issues)
- Rebuilds PX4 containers with fixed MAVLink config
- Rebuilds Xfs executable with SingleTaskCall fix
- Starts services and verifies fixes are active
- Provides next steps for testing

---

**ALTERNATIVE: Manual Rebuild by Environment**

**IMPORTANT**: Choose the correct rebuild script for your environment:

### For Xfs Custom Environment (Most Users)

If your airsim-container runs the **Xfs custom environment**:

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/unreal-builder

docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-xfs-with-fix.sh
```

**Time:** 40-70 minutes

**Project Details:**
- Project: `docker/unreal_executable/Xfs/Xfs.uproject`
- Plugin: `docker/unreal_executable/Xfs/Plugins/AirSim/`
- Output: `docker/unreal_executable/packaged/Xfs/`

### For Blocks Demo Environment

If your airsim-container runs the **Blocks demo environment**:

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/unreal-builder

docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-blocks-with-fix.sh
```

**Time:** 40-70 minutes

**Project Details:**
- Project: `Unreal/Environments/Blocks/Blocks.uproject`
- Plugin: Requires copying to project (automated in script)
- Output: `docker/unreal_executable/packaged/Blocks/`

### How to Check Which Environment You're Using

```bash
# Method 1: Check .env configuration
cat /home/mnsuser/Cosys_Airsim_Exploration/docker/.env | grep UNREAL_BINARY

# Method 2: Check running container
docker exec airsim-container ps aux | grep -E "Blocks|Xfs"

# Method 3: Check packaged directory contents
ls -la /home/mnsuser/Cosys_Airsim_Exploration/docker/unreal_executable/packaged/
```

### Manual Step-by-Step

If you need more control or troubleshooting:

#### Step 1: Rebuild AirLib (5 minutes)

```bash
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
```

**What this does:**
- Compiles AirLib source code (including your header changes)
- Builds static libraries (`*.a` files)
- Copies to `AirLib/lib/` and `Unreal/Plugins/AirSim/Source/AirLib/lib/`

**Verification:**
```bash
# Check build timestamp
ls -lh /home/mnsuser/Cosys_Airsim_Exploration/AirLib/lib/librpc.a

# Verify no glibc 2.38+ symbols (container compatibility)
nm /home/mnsuser/Cosys_Airsim_Exploration/AirLib/lib/librpc.a | grep "__isoc23" && echo "❌ Host build detected!" || echo "✅ Container-compatible"
```

#### Step 2: Rebuild Blocks Executable (30-60 minutes)

```bash
docker compose run --rm xfs-builder /bin/bash

# Inside container:
/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
    -project="/workspace/airsim/Unreal/Environments/Blocks/Blocks.uproject" \
    -platform=Linux \
    -clientconfig=Development \
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
    -archivedirectory="/workspace/airsim/docker/unreal_executable/packaged" \
    -utf8output
```

**What this does:**
- Rebuilds Unreal Engine plugin with new AirLib libraries
- Compiles Blocks project
- Cooks assets
- Packages into standalone executable
- Places result in `docker/unreal_executable/packaged/`

#### Step 3: Restart Docker Services (2 minutes)

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Stop all services
docker-compose -f docker-compose-master.yml down

# Rebuild PX4 containers (if you made PX4 config changes)
docker-compose -f docker-compose-master.yml build px4-drone-1 px4-drone-2 px4-drone-3 px4-drone-4

# Start services with new executable
docker-compose -f docker-compose-master.yml up --profile linux-integrated
```

**Note**: No need to rebuild airsim-container - it **mounts** the executable from the host filesystem, so restarting picks up the new build automatically.

#### Step 4: Verify Fix

```bash
# Launch ROS2
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Monitor PX4 for OFFBOARD mode activation
docker logs -f px4-drone-1 2>&1 | grep -i "offboard\|commander"

# Test takeoff
ros2 service call /Drone1/takeoff airsim_interfaces/srv/Takeoff "{}"
```

**Success indicators:**
```
# PX4 logs (proves fix is active):
INFO [commander] Offboard control started       ← NEW (proves SingleTaskCall fix works)
INFO [commander] Main state changed to MAIN_STATE_OFFBOARD

# AirSim logs (normal):
Ground already stable (variance: 0.000 <= 0.100)
Take off to 137.199997

# Physical result:
Drone takes off and climbs to altitude ✅
```

## Common Scenarios

### Scenario 1: Header File Changes (e.g., SingleTaskCall Fix)

**Files modified:**
- `AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`

**Required rebuild:**
- ✅ AirLib rebuild (incorporates header changes)
- ✅ Unreal project rebuild (compiles new AirLib into executable)

**Command for Xfs:**
```bash
docker compose run --rm xfs-builder /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-xfs-with-fix.sh
```

**Command for Blocks:**
```bash
docker compose run --rm xfs-builder /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-blocks-with-fix.sh
```

### Scenario 2: Source File Changes (e.g., .cpp files)

**Files modified:**
- `AirLib/src/**/*.cpp`

**Required rebuild:**
- ✅ AirLib rebuild
- ✅ Unreal project rebuild

**Command:**
Same as Scenario 1 (choose Xfs or Blocks script based on your environment)

### Scenario 3: PX4 Config Changes (e.g., MAVLink streams)

**Files modified:**
- `docker/px4_airsim_docker/config/px4-rc.mavlink.network`

**Required rebuild:**
- ❌ AirLib rebuild (not needed)
- ❌ Blocks rebuild (not needed)
- ✅ PX4 container rebuild only

**Command:**
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker
docker-compose -f docker-compose-master.yml build px4-drone-1 px4-drone-2 px4-drone-3 px4-drone-4
docker-compose -f docker-compose-master.yml up --profile linux-integrated
```

### Scenario 4: ROS2 Code Changes

**Files modified:**
- `ros2/src/airsim_ros_pkgs/**/*.cpp`

**Required rebuild:**
- ❌ AirLib rebuild (not needed)
- ❌ Blocks rebuild (not needed)
- ✅ ROS2 workspace rebuild inside ros2-x11-node container

**Command:**
```bash
# Inside ros2-x11-node container
colcon build --packages-select airsim_ros_pkgs
source install/setup.bash
```

## Troubleshooting

### Issue: "Drone still doesn't take off after rebuild"

**Diagnosis:**
```bash
# Check if OFFBOARD mode is activated
docker logs px4-drone-1 2>&1 | grep -i offboard

# If you DON'T see "Offboard control started", the fix didn't apply
```

**Solutions:**
1. Verify AirLib was rebuilt with your changes:
   ```bash
   ls -lh /home/mnsuser/Cosys_Airsim_Exploration/AirLib/lib/librpc.a
   # Should show recent timestamp
   ```

2. Verify Blocks was rebuilt AFTER AirLib:
   ```bash
   ls -lh /home/mnsuser/Cosys_Airsim_Exploration/docker/unreal_executable/packaged/LinuxNoEditor/Blocks/Binaries/Linux/Blocks
   # Timestamp should be AFTER librpc.a timestamp
   ```

3. Verify container is using new executable:
   ```bash
   docker-compose -f docker-compose-master.yml down
   docker-compose -f docker-compose-master.yml up --profile linux-integrated
   ```

### Issue: "glibc symbol errors during Unreal build"

**Error message:**
```
undefined reference to `__isoc23_sscanf@@GLIBC_2.38'
```

**Cause:** AirLib was built on host with glibc 2.38+, but container has glibc 2.28

**Solution:** Rebuild AirLib inside container:
```bash
docker compose run --rm xfs-builder /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/build-airlib.sh
```

### Issue: "Out of memory during Unreal build"

**Error message:**
```
FATAL: command failed with exit code: 137 (killed by OS)
```

**Cause:** Unreal Engine 5 requires ~16GB RAM for builds

**Solutions:**
1. Increase Docker memory limit: Docker Desktop → Resources → Memory → 20GB
2. Close other applications during build
3. Build on machine with more RAM
4. Use swap file (slower but works)

### Issue: "Permission denied during archive/packaging"

**Error message:**
```
SafeCopyFile Exception was Exception in System.Private.CoreLib: Access to the path '/workspace/airsim/docker/unreal_executable/packaged/Linux/Manifest_NonUFSFiles_Linux.txt' is denied.
InnerException in : Permission denied
Failed to copy ... to /workspace/airsim/docker/unreal_executable/packaged/Linux/...
```

**Cause:** Docker volume mount permission conflict - previous builds created files with different ownership (often root)

**Solution (Run on HOST machine):**
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration

# Clean problematic directories
sudo rm -rf docker/unreal_executable/packaged/Linux
sudo rm -rf docker/unreal_executable/packaged/Xfs
sudo rm -rf docker/unreal_executable/packaged/Blocks

# Fix ownership
sudo chown -R $USER:$USER docker/unreal_executable/packaged

# Then re-run the rebuild script
cd docker/unreal-builder
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-xfs-with-fix.sh
```

**Prevention:**
The rebuild scripts now include automatic permission checking (Step 0). If permissions are incorrect, the script will:
1. Detect the issue before starting the long build
2. Display the exact fix commands for your system
3. Exit cleanly without wasting build time

**Alternative fix (inside container as root):**
```bash
# If host fix doesn't work
docker compose run --rm --user root xfs-builder bash

# Inside container:
chown -R ue4:ue4 /workspace/airsim/docker/unreal_executable/packaged
chmod -R 755 /workspace/airsim/docker/unreal_executable/packaged
exit

# Then run normal build
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-builder/workflows/linux-host/scripts/rebuild-xfs-with-fix.sh
```

### Issue: "Executable not found after build"

**Check output directory:**
```bash
ls -lh /home/mnsuser/Cosys_Airsim_Exploration/docker/unreal_executable/packaged/LinuxNoEditor/Blocks/Binaries/Linux/
```

**If empty:**
- Check build logs for errors
- Verify `-archivedirectory` path in BuildCookRun command
- Ensure BuildCookRun completed successfully (exit code 0)

## Quick Reference

### File Locations

| Component | Source | Built Output | Final Location |
|-----------|--------|--------------|----------------|
| AirLib headers | `AirLib/include/**/*.hpp` | `AirLib/lib/*.a` | Compiled into executable |
| AirLib source | `AirLib/src/**/*.cpp` | `AirLib/lib/*.a` | Compiled into executable |
| Unreal plugin | `Unreal/Plugins/AirSim/` | Project-level plugin | Compiled into executable |
| Blocks project | `Unreal/Environments/Blocks/` | Packaged executable | `docker/unreal_executable/packaged/` |
| PX4 config | `docker/px4_airsim_docker/config/` | Docker image | PX4 container |

### Build Times (Approximate)

| Task | Time | CPU Usage | Memory Usage |
|------|------|-----------|--------------|
| AirLib rebuild | 5 min | 100% (all cores) | ~4GB |
| Blocks rebuild (full) | 45 min | 100% (all cores) | ~16GB |
| Blocks rebuild (incremental) | 15 min | 100% (all cores) | ~8GB |
| PX4 container rebuild | 2 min | 50% | ~2GB |
| Container restart | 2 min | 10% | ~1GB |

### Verification Commands

```bash
# Check AirLib build
ls -lh AirLib/lib/librpc.a
nm AirLib/lib/librpc.a | grep -c "__isoc23"  # Should be 0

# Check Blocks build
ls -lh docker/unreal_executable/packaged/LinuxNoEditor/Blocks/Binaries/Linux/Blocks

# Check containers running
docker ps | grep -E "airsim-container|px4-drone"

# Check AirSim API responding
timeout 3 bash -c "echo > /dev/tcp/localhost/41451" && echo "✅ AirSim API up" || echo "❌ AirSim API down"

# Check PX4 OFFBOARD mode
docker logs px4-drone-1 2>&1 | grep "Offboard control started"
```

## Additional Resources

- **Unreal Build System**: [Epic Games Documentation](https://docs.unrealengine.com/5.0/en-US/BuildingUnrealEngineFromSource/)
- **AirSim Build Guide**: `/home/mnsuser/Cosys_Airsim_Exploration/CLAUDE.md`
- **Docker Workflow**: `docker/unreal-builder/workflows/linux-host/README.md`
- **MAVLink Integration**: `ros2/README_MAVROS_INTEGRATION.md`
