# TEVV AirSim Compose Guide

## Overview

The `docker-compose-master.yml` provides **clean orchestration** of the AirSim ecosystem by referencing specialized compose files rather than duplicating services. This approach maintains separation of concerns while enabling integrated deployments.

**Recommended Approach**: Use the **MNSTEVV CLI** for simplified container management. The CLI provides intuitive commands for common operations and automatically handles service selection based on drone count. Fall back to direct docker-compose commands for advanced use cases.

## Architecture

```
docker-compose-master.yml (Orchestrator)
├── airsim_ros2_wrapper/docker-compose.yml (ROS2 Multi-Node)
├── px4_airsim_docker/docker-compose-slim.yml (PX4 Swarm)
├── Shared Networks (Cross-ecosystem communication)
├── Infrastructure Services (Monitoring, Development tools)
└── Environment Templates (Configurable parameters)
```

## UE5 Build & Execution Pipeline

### Overview

The AirSim Docker ecosystem includes a complete UE5 build-to-execution pipeline:

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. BUILD PHASE (docker/unreal-airsim/)                          │
│                                                                  │
│    AirLib Source → Epic Toolchain → UE5 Project →               │
│    Cook Assets → Package → Standalone Executable                 │
│                                                                  │
│    Output: docker/unreal_executable/packaged-sm6/Xfs/           │
└─────────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ 2. EXECUTION PHASE (docker/unreal_executable/)                  │
│                                                                  │
│    Packaged Executable → Mount in Container →                    │
│    Vulkan Software Rendering → Headless Execution               │
│                                                                  │
│    Services: airsim-headless, ros2-integration                  │
└─────────────────────────────────────────────────────────────────┘
```

### Build System (unreal-airsim/)

**Purpose**: Build AirSim UE5 projects into standalone Linux executables

**Key Components**:
- Epic Games UE5.5 official containers (`ghcr.io/epicgames/unreal-engine:dev-5.5.4`)
- Automated AirLib compilation with Epic's toolchain
- 3-layer dependency synchronization (AirLib → Plugin Source → Project Plugin)
- Vulkan Shader Model 6 support for headless execution
- BuildCookRun automation for packaging

**Usage**:
```bash
cd docker/unreal-airsim
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/examples/build-xfs-example.sh
```

**Output**: `docker/unreal_executable/packaged-sm6/Xfs/Linux/Xfs/Binaries/Linux/Xfs`

**Documentation**: See [docker/unreal-airsim/README.md](unreal-airsim/README.md)

### Execution System (unreal_executable/)

**Purpose**: Run packaged AirSim executables in Docker containers

**Key Components**:
- Vulkan software rendering (llvmpipe, lavapipe)
- X11 display forwarding or headless mode (`-nullrhi`)
- Volume mounts for packaged executables
- Integration with PX4 SITL and ROS2 ecosystems

**Usage**:
```bash
# Headless execution
docker compose up -d airsim-headless

# Manual execution with Vulkan software rendering
docker compose run --rm airsim-headless bash
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
./docker/unreal_executable/packaged-sm6/Xfs/Linux/Xfs/Binaries/Linux/Xfs -nullrhi
```

**Documentation**: See [docker/unreal_executable/readme.md](unreal_executable/readme.md)

### Integration Workflow

**Complete Pipeline Example**:

```bash
# 1. Build AirSim executable (one-time, ~20-30 minutes)
cd docker/unreal-airsim
docker compose run --rm xfs-builder \
  /workspace/airsim/docker/unreal-airsim/workflows/linux-host/scripts/examples/build-xfs-example.sh

# 2. Verify packaged output
ls -lh ../unreal_executable/packaged-sm6/Xfs/Linux/Xfs/Binaries/Linux/Xfs

# 3. Configure execution environment
cd ../unreal_executable
cp settings_example.json packaged-sm6/Xfs/Linux/settings.json

# 4. Launch integrated ecosystem (AirSim + PX4 + ROS2)
cd ../
docker compose -f docker-compose-master.yml --profile integrated up

# 5. Monitor and interact
docker exec -it ros2-multi-node bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
```

**Benefits**:
- **Reproducible Builds**: Epic's official toolchain ensures consistency
- **Headless Execution**: Vulkan SM6 works without GPU hardware
- **Containerized**: Isolated environments for build and execution
- **Integrated**: Seamless connection with PX4 SITL and ROS2 stacks

## MNSTEVV CLI - Simplified Container Management

**MNSTEVV** (Multi-Node System Test Environment Vehicle Validator) provides an intuitive CLI interface for managing the AirSim Docker ecosystem.

### Installation

```bash
# Navigate to the CLI directory
cd docker/cli

# Windows 10/11 - Recommended method (handles everything automatically)
.\mnstevv-auto.bat --num_drones 3
# This script: installs package + handles PATH + detects WSL2 IP + launches

# Alternative: Manual installation (requires PATH setup)
./install.bat
# Note: install.bat only installs the Python package - does NOT add to PATH
# After install.bat, use one of these approaches:

# Option 1: Direct Python execution (always works)
python mnstevv.py --version

# Option 2: Add to PATH manually (see diagnostic guide below)
mnstevv --version  # Only works after adding Scripts directory to PATH
```

### MNSTEVV Installation & PATH Diagnostic Guide

Having trouble with **"mnstevv.exe is not on Path"** or **"mnstevv command not found"**? This comprehensive guide handles all scenarios.

#### Quick Solutions (Choose One)

**Option A: Auto-Launcher (Easiest - No Installation Required)**
```cmd
# Navigate to CLI directory and use the auto-launcher
cd docker\cli
.\mnstevv-auto.bat --num_drones 3

# This automatically handles:
# 1. Installation if mnstevv package not found
# 2. PATH issues by using 'python mnstevv.py' when needed
# 3. WSL2 IP detection and configuration
# Note: install.bat does NOT add mnstevv to PATH - auto-launcher handles this
```

**Option B: Direct Python Execution (Always Works)**
```cmd
# Navigate to the CLI directory first
cd docker\cli

# Use Python directly instead of the mnstevv command
python mnstevv.py --version
python mnstevv.py up --num_drones 3
python mnstevv.py status --detailed
```

**Option C: Permanent PATH Solution**
Follow the complete diagnostic flow below for a permanent fix.

---

#### Complete Diagnostic Flow
**STEP 1: Install MNSTEVV (if Test 1 failed)**
```cmd
# Navigate to the CLI directory
cd docker\cli

# Option A: Manual installation
pip install -e .

# Option B: Use install.bat (Windows convenience script)
install.bat

# IMPORTANT: install.bat only installs the Python package.
# It does NOT add mnstevv.exe to your Windows PATH.
# For automatic PATH handling, use mnstevv-auto.bat instead.

# Verify installation
python -c "import mnstevv; print('Installation successful')"
```

**STEP 2: Locate Script Directory**
```cmd
# Find where Python installs console scripts
python -c "import sysconfig; scripts_dir = sysconfig.get_path('scripts'); print('Scripts directory:', scripts_dir)"

# Check if mnstevv.exe exists in that directory  
python -c "import sysconfig, os; scripts_dir = sysconfig.get_path('scripts'); mnstevv_path = os.path.join(scripts_dir, 'mnstevv.exe'); print('mnstevv.exe path:', mnstevv_path); print('File exists:', os.path.exists(mnstevv_path))"

# Manual search for mnstevv.exe in common locations
dir "C:\Users\%USERNAME%\AppData\Local\Programs\Python\Python313\Scripts\mnstevv.exe" 
dir "C:\Users\%USERNAME%\AppData\Roaming\Python\Python313\Scripts\mnstevv.exe" /s  
dir "C:\Users\%USERNAME%\AppData\Local\Microsoft\WindowsApps\PythonSoftwareFoundation.Python313\Scripts\mnstevv.exe" /s

# Alternative: Search the entire system for mnstevv.exe
where /r C:\ mnstevv.exe 2>nul

Example Output: C:\Users\%USERNAME%\AppData\Local\Programs\Python\Python313\Scripts\mnstevv.exe

Scripts Directory: 
dir "C:\Users\%USERNAME%\AppData\Local\Programs\Python\Python313\Scripts\"
dir "C:\Users\%USERNAME%\AppData\Roaming\Python\Python313\Scripts\"
dir "C:\Users\%USERNAME%\AppData\Local\Microsoft\WindowsApps\PythonSoftwareFoundation.Python313\Scripts\"
```

**STEP 4: Configure Windows PATH**
```cmd
# Method A: Windows Settings GUI (Recommended)
echo.
echo 1. Copy this path: %SCRIPTS_DIR%
echo 2. Open Windows Settings ^> System ^> About ^> Advanced system settings
echo 3. Click "Environment Variables"
echo 4. Under "User variables", select "Path" and click "Edit"
echo 5. Click "New" and paste the scripts directory path
echo 6. Click "OK" to save all dialogs
echo 7. Restart your command prompt

# Method B: Command Line (Advanced)
setx PATH "%PATH%;%SCRIPTS_DIR%"
echo PATH updated. Restart your command prompt to apply changes.
```

**STEP 5: Final Verification**
```cmd
# Restart your command prompt, then test:
mnstevv --version

# If successful, you should see: mnstevv, version 1.0.x

# Test full functionality:
mnstevv up --num_drones 1 --profile integrated
```
---

#### Advanced Troubleshooting

**Multiple Python Installations Detected**
```cmd
# Check which Python is being used
python -c "import sys; print('Current Python:', sys.executable)"
where python

# Check which pip is being used
pip --version
where pip

# If different Python installations, ensure consistency:
# Use: py -m pip install -e . (Windows Python Launcher)
# Or specify exact Python: C:\Path\To\Specific\python.exe -m pip install -e .
```

**Script Exists But Not Found**
```cmd
# Manual search for mnstevv.exe in common locations
dir "C:\Users\%USERNAME%\AppData\Local\Programs\Python\Python*\Scripts\mnstevv.exe" /s
dir "C:\Users\%USERNAME%\AppData\Roaming\Python\Python*\Scripts\mnstevv.exe" /s  
dir "C:\Users\%USERNAME%\AppData\Local\Microsoft\WindowsApps\PythonSoftwareFoundation.Python*\Scripts\mnstevv.exe" /s

# Check current PATH contents
echo Current PATH:
echo %PATH%

# Test if scripts directory is in PATH
python -c "import sysconfig, os; scripts_dir = sysconfig.get_path('scripts'); path = os.environ.get('PATH', ''); print('Scripts dir in PATH:', scripts_dir.lower() in path.lower())"
```

**Permission or Installation Issues**
```cmd
# Try reinstalling with user flag
pip install -e . --user

# Check pip installation location
pip show -f mnstevv

# Clear pip cache and reinstall
pip cache purge
pip install -e . --force-reinstall --no-cache-dir
```

### Basic Usage

```bash
# Start services (default: integrated profile, 3 drones)
mnstevv up

# Start with specific number of drones
mnstevv up --num_drones 2

# Start with specific profile
mnstevv up --num_drones 4 --profile px4-only

# Stop services gracefully
mnstevv stop

# Stop all services and cleanup
mnstevv down

# Restart services
mnstevv restart --num_drones 3

# View running containers
mnstevv ps

# Check detailed service status
mnstevv status

# View logs
mnstevv logs --follow

# Clean up all resources
mnstevv clean
```

### Key Commands

| Command | Description | Key Options |
|---------|-------------|-------------|
| `mnstevv up` | Start services | `--num_drones`, `--profile`, `--detach` |
| `mnstevv down` | Stop and remove containers | `--cleanup-volumes` |
| `mnstevv stop` | Stop containers gracefully | `--services`, `--timeout` |
| `mnstevv restart` | Restart services | `--num_drones`, `--timeout` |
| `mnstevv status` | Show service health | `--detailed`, `--json` |
| `mnstevv ps` | List containers | `--all`, `--json`, `--filter` |
| `mnstevv logs` | View service logs | `--follow`, `--services`, `--tail` |
| `mnstevv config` | Validate configuration | `--show-computed` |
| `mnstevv clean` | Clean up resources | `--all`, `--volumes` |

### Deployment Profiles

| Profile | Description | Services Included |
|---------|-------------|-------------------|
| `integrated` | Full ecosystem (default) | ROS2 + PX4 (3 drones) + monitoring |
| `px4-only` | PX4 swarm only | PX4 drones + infrastructure |
| `ros2-only` | ROS2 development | ROS2 multi-node + VNC |
| `development` | Full ecosystem + debug tools | All services + enhanced debugging |
| `linux-integrated` | **Linux X11 native** (NEW) | ROS2 X11 + PX4 (3 drones) + monitoring |
| `linux-dev` | **Linux development** (NEW) | All services + X11 debugging |

### Linux X11 Native Forwarding (NEW)

**High-performance option for Linux users - 60-70% lower resource usage than VNC**

#### Platform Requirements:
- **Linux native only** (Ubuntu 20.04+, Debian 11+, Fedora 35+)
- X11 server running (default on Linux desktops)
- NVIDIA GPU recommended for RViz2 hardware acceleration

#### Prerequisites:
```bash
# Allow Docker containers to access X11 display (once per boot)
xhost +local:docker

# Verify X11 is working
echo $DISPLAY  # Should output :0 or :1
```

#### Quick Start:
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Launch Linux X11 system (3 drones + ROS2)
docker compose -f docker-compose-master.yml --profile linux-integrated up

# Access container shell
docker exec -it ros2-x11-node bash

# Inside container - test X11 forwarding
xclock  # Should open window on host display

# Launch ROS2 system (packages pre-built in image - instant startup!)
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# Optional: Install RViz2 for visualization (not included in base image)
sudo apt-get update && sudo apt-get install -y ros-humble-rviz2

# Test GPU acceleration
glxinfo | grep "OpenGL renderer"
# Expected: NVIDIA GeForce RTX 5080 (not llvmpipe)
```

**Note**: RViz2 is not pre-installed in the X11 image to reduce build time and network dependencies. Install it manually using the command above if you need 3D visualization.

#### Performance Comparison:

| Metric | VNC Setup (`integrated`) | X11 Setup (`linux-integrated`) | Improvement |
|--------|--------------------------|--------------------------------|-------------|
| Memory Usage | ~800MB | ~300MB | **62% reduction** |
| Startup Time | 15 seconds | 2 seconds | **87% faster** |
| Docker Image Size | 2.5GB | 1.8GB | **28% smaller** |
| RViz2 Frame Rate | 30-40 FPS (software) | 60+ FPS (hardware) | **50%+ faster** |
| GPU Rendering | llvmpipe (CPU) | NVIDIA direct | **Hardware accelerated** |
| Pre-built Packages | ✓ | ✓ | **Instant startup** |

#### When to Use X11 vs VNC:

**Use X11 Forwarding (`linux-integrated`) when:**
- ✓ Running on Linux desktop/laptop
- ✓ Need maximum RViz2/Gazebo performance
- ✓ Have local NVIDIA/Intel/AMD GPU
- ✓ Want minimal resource usage
- ✓ Prefer instant container startup

**Use VNC (`integrated`) when:**
- ✓ Running on Windows or macOS
- ✓ Need remote access from different machine
- ✓ Want self-contained desktop environment
- ✓ Don't mind VNC overhead (~500MB RAM)

### Common Workflows

#### Quick Testing (Single Drone)
```bash
mnstevv up --num_drones 1 --profile px4-only
mnstevv status --detailed
mnstevv logs --follow --services px4-drone-1
```

#### Development Workflow
```bash
# Start full development environment
mnstevv up --profile development --detach

# Monitor status
mnstevv status

# Access VNC for ROS2 development: localhost:5901 (password: ubuntu)

# View aggregated logs
mnstevv logs --follow
```

#### Large Swarm Testing
```bash
# Start maximum drone configuration
mnstevv up --num_drones 6 --profile development --detach

# Monitor specific services
mnstevv ps --filter px4

# Check detailed status
mnstevv status --detailed --json
```

### Troubleshooting with CLI

```bash
# Check overall ecosystem health
mnstevv status --detailed

# View service-specific logs
mnstevv logs --services px4-drone-1,ros2-multi-node

# Restart problematic services
mnstevv restart --services px4-drone-1 --timeout 30

# Clean restart
mnstevv down --cleanup-volumes
mnstevv up --num_drones 3 --profile integrated
```

## Windows + WSL2 Users: Critical Setup

### Important: Getting the Correct vEthernet IP

When running Docker containers in WSL2 that need to connect to AirSim running on Windows, you **MUST** use the stable Windows vEthernet (WSL) adapter IP address, not the default `host.docker.internal`.

#### Option 1: Automatic Detection (Recommended)
```cmd
# From Windows Command Prompt or PowerShell (in docker/cli directory)
.\mnstevv-auto.bat --num_drones 3
```
This script automatically detects the correct vEthernet IP and launches MNSTEVV with proper configuration.

#### Option 2: Manual IP Detection
```powershell
# In Windows PowerShell, get the vEthernet (WSL) adapter IP
(Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" -AddressFamily IPv4).IPAddress
# Example output: 172.24.144.1

# Or use the helper script to save to environment file
cd docker\cli
.\get-wsl2-ip.ps1 -WriteToEnv
```

Then use the detected IP with MNSTEVV:
```bash
mnstevv up --airsim-host 172.24.144.1 --px4-sim-host 172.24.144.1
```

#### Option 3: Environment File Configuration
```bash
# Generate .env.wsl2 with correct IPs
cd docker/cli
powershell.exe -File get-wsl2-ip.ps1 -WriteToEnv

# Use the generated environment file
cd ..
docker-compose --env-file .env.wsl2 -f docker-compose-master.yml up
```
## Quick Start

### 1. CLI Setup (Recommended)
```bash
# Navigate to docker directory
cd docker/

# Copy and customize environment file
cp .env.template .env
# Edit .env with your preferred settings

# Set up CLI
cd cli

# Windows: Run ./install.bat

# Test CLI installation
mnstevv --version

# Ensure AirSim is running on host
# Windows: Start Blocks.exe or your Unreal environment  
# Linux: ./LinuxNoEditor/Blocks.sh
```

### 2. Deployment Options (CLI - Recommended)

**Full Integrated Ecosystem** (Recommended)

**Linux Native (60% lower resources, instant startup):**
```bash
# Prerequisites: xhost +local:docker (once per boot)
xhost +local:docker

# Using CLI
mnstevv up  # Auto-suggests X11 on Linux

# OR direct docker-compose command
docker compose -f docker-compose-master.yml --profile linux-integrated up
```

**Windows/Mac/Remote (VNC cross-platform):**
```bash
# Using CLI (simplified)
mnstevv up

# Equivalent docker-compose command
docker compose -f docker-compose-master.yml --profile integrated up
```

**Features:**
- **Linux:** ROS2 X11 Native with GPU acceleration + instant startup
- **Windows/Mac:** ROS2 Multi-Node with VNC desktop
- PX4 Swarm (3 drones)
- Cross-ecosystem networking
- **Pre-built ROS2 packages** (instant startup, no build wait)
- VNC Access (VNC profile only): `localhost:5901` (password: `ubuntu`)

**Custom Drone Count**
```bash
# 2 drones using CLI
mnstevv up --num_drones 2

# 6 drones using CLI  
mnstevv up --num_drones 6 --profile development
```

**ROS2 Only** (Development/Testing)
```bash
# Using CLI
mnstevv up --profile ros2-only

# Equivalent docker-compose command
docker-compose -f docker-compose-master.yml --profile ros2-only up
```
- ROS2 Multi-Node system
- VNC desktop environment
- Perfect for ROS2 development without PX4

**PX4 Only** (Swarm Testing)
```bash
# Using CLI
mnstevv up --profile px4-only --num_drones 4

# Equivalent docker-compose command  
docker-compose -f docker-compose-master.yml --profile px4-only up
```
- PX4 swarm (configurable drone count)
- MAVLink endpoints
- Perfect for PX4/MAVLink development

**Development Environment**
```bash
# Using CLI (full ecosystem)
mnstevv up --profile development

# Equivalent docker-compose command
docker-compose -f docker-compose-master.yml --profile development up
```
- Full ecosystem including ALL 6 drones (1-6)
- Development helper tools and enhanced debugging
- Network monitoring and health checks
- Access dev tools: `docker exec -it airsim-dev-helper sh`
- **Note**: Development profile includes drones 4-6 that are excluded from `integrated`

### 3. Service Management (CLI - Recommended)

**View running services**
```bash
# Using CLI (formatted output)
mnstevv ps

# Equivalent docker-compose command
docker-compose -f docker-compose-master.yml ps
```

**Check service status**
```bash
# Using CLI (health checks)
mnstevv status --detailed

# JSON output for scripting
mnstevv status --json
```

**View logs**
```bash  
# Using CLI - All services
mnstevv logs --follow

# Using CLI - Specific services
mnstevv logs --services px4-drone-1,ros2-multi-node --tail 50

# Equivalent docker-compose commands
docker-compose -f docker-compose-master.yml logs -f
docker-compose -f docker-compose-master.yml logs -f px4-bridge-drone-1
```

**Stop services**
```bash
# Using CLI (graceful stop)
mnstevv stop

# Using CLI (stop and cleanup)
mnstevv down --cleanup-volumes

# Equivalent docker-compose commands
docker-compose -f docker-compose-master.yml down
docker-compose -f docker-compose-master.yml down -v
```

**Restart services**
```bash
# Using CLI 
mnstevv restart --num_drones 3

# Restart specific services
mnstevv restart --services px4-drone-1 --timeout 30
```

## Network Architecture

### Network Segments

| Network | Subnet | Purpose |
|---------|--------|---------|
| `airsim-ecosystem` | 172.30.0.0/16 | Cross-ecosystem bridge |
| `px4_network` | 172.20.0.0/16 | PX4 swarm communication |
| `ros2-multi-node-network` | 172.26.0.0/16 | ROS2 node communication |

### Service Endpoints

**ROS2 Services**
- VNC Desktop: `localhost:5901`
- ROS2 Discovery: `localhost:7400/udp`
- Container: `ros2-node`

**PX4 Services**  
- Drone 1 MAVLink: `localhost:4560/tcp`, `localhost:14540/udp`, `localhost:14550/udp`, `localhost:14580/udp`
- Drone 2 MAVLink: `localhost:4561/tcp`, `localhost:14541/udp`, `localhost:14581/udp`  
- Drone 3 MAVLink: `localhost:4562/tcp`, `localhost:14542/udp`, `localhost:14582/udp`
- Containers: `px4-drone-{1,2,3}`

**Infrastructure Services**
- Network Monitor: `airsim-monitor`
- Development Helper: `airsim-dev-helper`
- WSL2 IP Detector: `airsim-wsl2-detector`

## Configurable Drone Spawning

### Drone Service Overview

The master compose supports **1-6 configurable drones** with proper PX4 SITL integration:

| Service | Instance ID | AirSim Port | QGC Ports | Profile Support | Architecture |
|---------|-------------|-------------|-----------|-----------------|-------------|
| `px4-bridge-drone-1` | 0 | 4560 | 14540, 14550, 14580 | integrated, px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-2` | 1 | 4561 | 14541, 14581 | integrated, px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-3` | 2 | 4562 | 14542, 14582 | integrated, px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-4` | 3 | 4563 | 14543, 14583 | px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-5` | 4 | 4564 | 14544, 14584 | px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-6` | 5 | 4565 | 14545, 14585 | px4-only, development | **Extended from slim compose** |
| `px4-bridge-drone-7` | 6 | 4566 | 14546, 14586 | px4-only, development | **Extended from slim compose** |

### Quick Start Examples

#### 1. **Default 3-Drone Setup** (Recommended)
```bash
cd docker/
docker-compose -f docker-compose-master.yml --profile integrated up -d
```
- Spawns: px4-bridge-drone-1, px4-bridge-drone-2, px4-bridge-drone-3
- Includes: ROS2 Multi-Node + VNC + WSL2 IP detection + Network monitoring
- Access VNC: `localhost:5901` (password: `ubuntu`)
- **Note**: Only drones 1-3 are included in the `integrated` profile by default

#### 2. **Single Drone Testing**
```bash
# Using drone 4 (not in integrated profile, so specify explicitly)
docker-compose -f docker-compose-master.yml --profile px4-only up px4-bridge-drone-4 ros2-multi-node 

# Or use integrated profile with explicit service selection
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 ros2-multi-node 
```
- Perfect for testing individual drone behavior
- Drone 4-6 require `px4-only` or `development` profiles (not `integrated`)

#### 3. **Custom Multi-Drone Configuration**

**Windows (Command Prompt/PowerShell):**
```cmd
# Example: Two drones (1 and 2)
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 ros2-multi-node 

# Example: Drones 1, 4, and 6 (mixed architecture)  
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-4 px4-bridge-drone-6 ros2-multi-node 

# Example: Large swarm (all 6 drones)
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3 px4-bridge-drone-4 px4-bridge-drone-5 px4-bridge-drone-6 ros2-multi-node 

# Example: Independent drones only (4-6)
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-4 px4-bridge-drone-5 px4-bridge-drone-6 ros2-multi-node 
```

**Linux/WSL2 (Bash with brace expansion):**
```bash
# Example: Large swarm (all 6 drones) - bash shorthand
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..6} ros2-multi-node 

# Example: Independent drones only (4-6) - bash shorthand  
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{4..6} ros2-multi-node 
```

#### 4. **Development Environment** (Full ecosystem + tools)
```bash
docker-compose -f docker-compose-master.yml --profile development up -d
```
- All services + development helper tools
- Network monitoring and debugging capabilities
- Enhanced logging and health checks

## Usage Examples

### Example 1: Full Development Workflow
```bash
# 1. Start full environment  
docker-compose -f docker-compose-master.yml --profile development up -d

# 2. Connect to VNC for ROS2 development
# Open VNC viewer to localhost:5901 (password: ubuntu)

# 3. Inside VNC terminal, launch ROS2 system
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py

# 4. In separate terminal, test PX4 connection  
docker exec -it px4-drone-1 /bin/bash
# Inside container: Check PX4 SITL status, test MAVLink, etc.

# 5. Monitor ecosystem health
docker logs -f airsim-monitor
```

### Example 2: Single Drone Development
```bash
# Start single drone for focused development
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-5 ros2-multi-node 

# Monitor drone 5 specifically
docker logs -f px4-drone-5

# Access ROS2 environment via VNC
# VNC: localhost:5901 (password: ubuntu)
# Inside VNC: ros2 topic list, ros2 service list
```

### Example 3: Large Swarm Testing

**Windows Commands:**
```cmd
# Start all 6 drones for swarm testing
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3 px4-bridge-drone-4 px4-bridge-drone-5 px4-bridge-drone-6 ros2-multi-node  -d

# Connect QGroundControl to multiple drones:
# UDP endpoints: 14540/14550/14580 (drone 1), 14541/14581 (drone 2), 14542/14582 (drone 3), 
#                14543/14583 (drone 4), 14544/14584 (drone 5), 14545/14585 (drone 6)

# Test individual drone connections (Windows)
for %p in (4560 4561 4562 4563 4564 4565) do (
    echo Testing port %p...
    telnet localhost %p
)
```

**Linux/WSL2 Commands:**
```bash
# Start all 6 drones for swarm testing (bash shorthand)
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..6} ros2-multi-node  -d

# Test individual drone connections (Linux/WSL2)
for port in 4560 4561 4562 4563 4564 4565; do
    echo "Testing port $port..."
    nc -zv localhost $port
done
```

### Example 4: ROS2 Development Only
```bash
# Start ROS2 system only (no PX4 drones)
docker-compose -f docker-compose-master.yml --profile ros2-only up

# Access ROS2 container directly  
docker exec -it ros2-node /bin/bash
cd /airsim_ros2_ws
source install/setup.bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
```

## Environment Configuration

##**Platform Compatibility**

| Platform | Command Syntax | Brace Expansion | Recommended Shell |
|----------|----------------|-----------------|-------------------|
| **Windows 10/11** | `docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2` | Not supported | Command Prompt, PowerShell |
| **Linux** | `docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..2}` | Supported | Bash, Zsh |
| **WSL2** | `docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..2}` | Supported | Bash (default) |
| **macOS** | `docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..2}` | Supported | Bash, Zsh |

> **Important**: Windows users must specify each drone service individually. Linux/WSL2/macOS users can use bash brace expansion for convenience.

### Platform Selection Guide

**Choosing the Right Profile for Your Platform:**

```
Are you running on Linux desktop/laptop?
  ├── YES → Use --profile linux-integrated
  │          ✓ 60-70% lower resource usage
  │          ✓ Hardware GPU acceleration
  │          ✓ Instant startup (2s vs 15s)
  │          ✓ Pre-built packages
  │          Prerequisites: xhost +local:docker
  │
  └── NO (Windows/Mac/Remote) → Use --profile integrated
             ✓ Cross-platform compatibility
             ✓ Self-contained VNC desktop
             ✓ Remote access capability
             ✓ Pre-built packages
             Access: VNC on localhost:5901
```

**Environment Variable Matrix:**

| Variable | VNC Setup (`integrated`) | X11 Setup (`linux-integrated`) | Purpose |
|----------|--------------------------|-------------------------------|---------|
| `DISPLAY` | `:1` (internal VNC) | `${DISPLAY}` (host) | X11 display target |
| `VNC_PORT` | `5901` | N/A | VNC server port |
| `XAUTHORITY` | N/A | `${XAUTHORITY}` | X11 authentication |
| `UID` | `1000` | `${UID}` | User ID mapping |
| `GID` | `1000` | `${GID}` | Group ID mapping |
| `AUTO_BUILD` | `false` | `false` | **Packages pre-built in image** |

### Key Variables (.env file)

| Variable | Default | Description |
|----------|---------|-------------|
| `AIRSIM_HOST_IP` | `host.docker.internal` | AirSim API endpoint |
| `LAUNCH_MODE` | `multi` | ROS2 architecture mode |
| `SWARM_SIZE` | `3` | Number of PX4 drones |
| `LAUNCH_RVIZ` | `false` | Auto-launch RViz |
| `DEBUG` | `false` | Enable debug logging |


### Drone Count Configuration

Add to your `.env` file:
```env
# Maximum drones to spawn (1-6 supported)
MAX_DRONES=6

# Note: You still need to specify drone services explicitly:
# docker-compose up px4-bridge-drone-{1..6} ros2-multi-node 
```

## Troubleshooting

### Common Issues

#### **1. Port Conflicts** 
```bash
# Problem: "Address already in use" or "port already allocated"
# Solution: Stop all containers and restart cleanly
docker-compose -f docker-compose-master.yml down
docker system prune --volumes -f  # Optional: clean up completely
docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-4 ros2-multi-node 
```

#### **2. WSL2 IP Detection Issues**
```bash
# NEW: Use the automatic workflow script (RECOMMENDED)
cd cli
# Windows Command Prompt or PowerShell:
.\\mnstevv-auto.bat --num_drones 3

# Or manually get the correct IP:
# Windows PowerShell:
(Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" -AddressFamily IPv4).IPAddress

# Legacy: Check WSL2 IP detection logs (service runs once and exits)
docker logs airsim-wsl2-detector

# Verify detected IP configuration (persists in shared volume)
docker run --rm -v docker_wsl2_shared_config:/shared alpine:latest cat /shared/wsl2_config.env

# If detector exited early, check if other services successfully loaded config:
docker exec px4-drone-1 env | grep PX4_SIM_HOSTNAME
```

#### **3. PX4 SITL Connection Issues**
```bash
# Check individual drone logs
docker logs px4-drone-4 | grep -E "(SITL|simulator_mavlink|connected)"

# Expected: "Simulator connected on TCP port 4563"
# If not connected: Ensure AirSim is running on Windows/host system

# Test port connectivity
nc -zv localhost 4563  # Should show: Connection to localhost port 4563 [tcp/*] succeeded!
```

#### **4. Network Connectivity**
```bash
# Check ecosystem monitor logs
docker logs airsim-monitor

# Test AirSim connection manually
docker exec -it airsim-dev-helper nc -zv 172.18.144.1 41451

# Test individual drone MAVLink ports
for port in 4560 4561 4562 4563 4564 4565; do
    echo "Testing drone port $port..."
    nc -zv localhost $port
done
```

#### **5. Volume Permissions**
```bash  
# Fix ROS2 workspace permissions
docker exec -it ros2-node sudo chown -R $USER:$USER /airsim_ros2_ws

# Fix build/install directory permissions
docker exec -it ros2-node /bin/bash
# Inside container: fix_perms && build
```

#### **6. Service Dependencies**
```bash
# Restart with dependency resolution
docker-compose -f docker-compose-master.yml down
docker-compose -f docker-compose-master.yml --profile integrated up --force-recreate
```

### Advanced Debugging

#### **Check All Service Status**
```bash
# View all running services
docker-compose -f docker-compose-master.yml ps

# Check specific service health
docker-compose -f docker-compose-master.yml ps px4-bridge-drone-4

# View comprehensive logs
docker-compose -f docker-compose-master.yml logs -f px4-bridge-drone-4
```

#### **Verify Drone Instance Configuration**
```bash
# Check that each drone has correct instance ID
docker exec px4-drone-1 env | grep PX4_INSTANCE  # Should be 0
docker exec px4-drone-4 env | grep PX4_INSTANCE  # Should be 3
docker exec px4-drone-6 env | grep PX4_INSTANCE  # Should be 5

# Check WSL2 IP is loaded correctly
docker exec px4-drone-4 env | grep PX4_SIM_HOSTNAME  # Should be 172.18.144.1
```

#### **Performance Monitoring**
```bash
# Monitor resource usage
docker stats px4-drone-1 px4-drone-4 px4-drone-6 ros2-node

# Check network activity
docker exec -it airsim-monitor netstat -tuln | grep -E "(4560|4563|4565)"
```

### Health Checks

**Ecosystem Status**
```bash
# View comprehensive status
docker logs -f airsim-monitor

# Manual health check
docker exec -it airsim-dev-helper sh -c "
  echo 'Testing AirSim API...' && nc -zv host.docker.internal 41451 &&
  echo 'Testing ROS2 services...' && nc -zvu ros2 7400 &&  
  echo 'Testing PX4 drones...' && nc -zv px4-drone-1 4560
"
```

## Integration with Specialized Compose Files

This master compose **extends** rather than replaces the specialized compose files:

- **Direct use**: `docker-compose -f airsim_ros2_wrapper/docker-compose.yml up` (Specialized)
- **Orchestrated use**: `docker-compose -f docker-compose-master.yml --profile ros2-only up` (Master)

Both approaches work independently and can coexist.

## Migration from Legacy Setup

### From Old Master Compose
1. **Backup**: `cp docker-compose-master.yml docker-compose-master.yml.backup`
2. **Update**: Use new orchestrated version (already implemented)
3. **Environment**: `cp .env.template .env` and customize
4. **Test**: `docker-compose -f docker-compose-master.yml --profile integrated up --dry-run`

### Benefits of New Approach
- **No service duplication** - Single source of truth
- **Modular deployment** - Use specialized compose files independently  
- **Environment templating** - Easy configuration management
- **Profile-based deployment** - Scenario-specific launches
- **Enhanced monitoring** - Cross-ecosystem health checks
- **Development workflow** - Integrated debugging tools

## **Key Features Summary**

### **Pre-Built ROS2 Interfaces (NEW - BREAKING CHANGE)**
- **Zero build time on startup** - All packages built during Docker image creation
- **PIE-compliant RPC library** - rpclib compiled with `-fPIC` flags during image build
- **Complete dependency chain** - mission_search_interfaces → airsim_interfaces → airsim_ros_pkgs
- **Optimized for production** - Release build with `-O2` optimization
- **Instant launch capability** - `ros2 launch` works immediately after container starts (2s vs 5-10 minutes)
- **No AUTO_BUILD needed** - Packages pre-built, workspace ready to use

**Built Packages:**
1. `airsim_interfaces` - Custom message/service definitions for AirSim
2. `mission_search_interfaces` - Mission planning interfaces
3. `airsim_ros_pkgs` - Main ROS2 integration package (multi-node architecture)

**Build Configuration:**
- CMAKE_POSITION_INDEPENDENT_CODE: ON
- CMAKE_CXX_FLAGS: "-fPIC -O2"
- CMAKE_BUILD_TYPE: Release
- Colcon: `--packages-up-to airsim_ros_pkgs`

**Migration Notes:**
- `AUTO_BUILD` environment variable now defaults to `false`
- Runtime build scripts (fix_rpc_build.sh) no longer mounted or executed
- Image build time increases by ~5-8 minutes (one-time cost)
- Container startup reduces from 5-10 minutes to <5 seconds

### **Configurable Drone Spawning (NEW)**
- **1-6 drones supported** with any combination
- **Proper PX4 SITL integration** with correct instance IDs
- **Independent service definitions** (drones 4-6) avoid port conflicts
- **Mixed architecture support** (extended + independent services)

### **Automatic WSL2 Integration (NEW)**
- **Dynamic IP detection** - No manual configuration required
- **Self-cleaning detector** - Automatically exits after 90-second grace period
- **Cross-platform compatibility** - Works with Windows + Docker Desktop
- **Automatic configuration override** - Detects 172.18.144.1 bridge IP
- **Seamless connectivity** - AirSim on Windows ↔ Docker containers
- **No restart policy** - Detector cleans up after job completion

### **Enhanced Architecture**
- **Clean orchestration** using specialized compose file references  
- **Profile-based deployment** for different use cases
- **Zero port conflicts** with independent service definitions
- **Comprehensive monitoring** and debugging capabilities

### **Production-Ready Drone Services**
Each drone runs **proper PX4 SITL** with:
- Correct instance ID mapping (drone-1=instance 0, drone-4=instance 3, etc.)
- Unique port assignments (4560, 4563, 4564, 4565, etc.)
- AirSim connectivity via WSL2 bridge IP
- Independent service isolation
- QGroundControl UDP endpoint support

## CLI vs Docker Compose Command Reference

### Common Operations Comparison

| Operation | CLI Command | Docker Compose Equivalent |
|-----------|-------------|---------------------------|
| **Start 3 drones** | `mnstevv up` | `docker compose -f docker-compose-master.yml --profile integrated up` |
| **Start 2 drones** | `mnstevv up --num_drones 2` | `docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 ros2-multi-node` |
| **PX4 only** | `mnstevv up --profile px4-only --num_drones 4` | `docker-compose -f docker-compose-master.yml --profile px4-only up` |
| **Check status** | `mnstevv status --detailed` | `docker-compose -f docker-compose-master.yml ps` |
| **View logs** | `mnstevv logs --follow` | `docker-compose -f docker-compose-master.yml logs -f` |
| **Stop services** | `mnstevv stop` | `docker-compose -f docker-compose-master.yml stop` |
| **Clean shutdown** | `mnstevv down` | `docker-compose -f docker-compose-master.yml down` |
| **Restart** | `mnstevv restart --num_drones 3` | `docker-compose -f docker-compose-master.yml restart` |

### Migration Benefits

## Platform-Specific Quick Start

### **Windows (CLI Recommended)**
```cmd
# Navigate to docker directory
cd docker

# Set up CLI (if not already installed)
cd cli
install.bat
cd ..

# Test CLI
mnstevv --version

# 1. Quick start (default 3 drones) - CLI
mnstevv up

# 2. Test single drone - CLI
mnstevv up --num_drones 1 --profile px4-only

# 3. Two drones - CLI  
mnstevv up --num_drones 2

# 4. Large swarm (all 6 drones) - CLI
mnstevv up --num_drones 6 --profile development --detach

# Equivalent docker-compose commands (for reference):
# docker-compose -f docker-compose-master.yml --profile integrated up
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-4 ros2-multi-node 
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 ros2-multi-node 
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3 px4-bridge-drone-4 px4-bridge-drone-5 px4-bridge-drone-6 ros2-multi-node  -d
```

### **Linux/WSL2 (CLI Recommended)**
```bash
# Navigate to docker directory
cd docker/

# Set up CLI (if not already installed)
cd cli
pip install -e .
cd ..

# Test CLI
mnstevv --version

# 1. Quick start (default 3 drones) - CLI
mnstevv up

# 2. Test single drone - CLI
mnstevv up --num_drones 1 --profile px4-only

# 3. Large swarm - CLI
mnstevv up --num_drones 6 --profile development --detach

# 4. Custom drone configurations - CLI
mnstevv up --num_drones 4 --profile px4-only

# Equivalent docker-compose commands (bash brace expansion):
# docker-compose -f docker-compose-master.yml --profile integrated up
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-4 ros2-multi-node 
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{1..6} ros2-multi-node -d
# docker-compose -f docker-compose-master.yml --profile integrated up px4-bridge-drone-{4..6} ros2-multi-node 
```

### **Next Steps**
1. **Set up CLI** using platform-specific instructions above
2. **Choose your deployment** using CLI commands (`mnstevv up --num_drones X`)
3. **Customize** `.env` file for your specific requirements  
4. **Monitor** ecosystem health using CLI (`mnstevv status --detailed`)
5. **Scale** up or down using CLI (`mnstevv restart --num_drones X`)

### **CLI Quick Reference Card**
```bash
# Essential CLI commands for daily use
mnstevv up                          # Start default ecosystem (3 drones)
mnstevv up --num_drones 4           # Start with 4 drones
mnstevv status --detailed           # Check detailed service health
mnstevv ps --all                    # List all containers
mnstevv logs --follow              # View live logs
mnstevv stop                        # Stop services gracefully
mnstevv restart --num_drones 2      # Restart with 2 drones
mnstevv down --cleanup-volumes      # Full cleanup
mnstevv clean --all                 # Clean all resources
```

---
