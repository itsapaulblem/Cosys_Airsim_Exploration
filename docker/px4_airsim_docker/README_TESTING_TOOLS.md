# PX4-AirSim Docker Setup & Testing Tools Documentation

This directory contains setup scripts and testing tools for the PX4-AirSim Docker integration. This README covers the complete setup workflow including the main startup script, MAVLink network configuration, Docker Compose deployment, and comprehensive testing tools.

## 🚀 Complete Setup Overview

The PX4-AirSim Docker setup consists of four main components:

1. **Startup Script**: `../scripts/run_airsim_sitl.sh` - PX4 SITL launcher
2. **MAVLink Configuration**: `../config/px4-rc.mavlink.network` - Network MAVLink setup
3. **Docker Configuration**: `../docker-compose-slim.yml` - Multi-drone container orchestration
4. **Testing Tools**: This directory - Validation and diagnostic tools

## 🔧 Complete Setup Workflow

### Prerequisites
- Docker and Docker Compose installed
- AirSim running (Windows/Linux) with settings.json configured for multi-drone
- Sufficient system resources (8GB+ RAM recommended for 3+ drones)

### 1. Build Docker Image
```bash
cd /path/to/cosys-airsim/docker/px4_airsim_docker
docker build -f Dockerfile.slim -t px4-airsim:slim .

# Alternative: Use pre-built image if available
# docker pull your-registry/px4-airsim:slim
```

### 2. Configure AirSim Integration
```bash
# Set AirSim host IP (Docker Desktop)
export PX4_SIM_HOSTNAME=172.28.240.1

# Or for WSL2 (auto-detect Windows IP)
export PX4_SIM_HOSTNAME=$(grep nameserver /etc/resolv.conf | awk '{print $2}')

# Verify AirSim is reachable
ping $PX4_SIM_HOSTNAME
```

### 3. Launch Docker Containers
```bash
# Start specific number of drones
docker-compose -f docker-compose-slim.yml up px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3

# Or start in background
docker-compose -f docker-compose-slim.yml up -d px4-bridge-drone-{1..5}

# Check container health
docker-compose -f docker-compose-slim.yml ps
```

### 4. Verify PX4 SITL Operation
```bash
# Check PX4 processes in container
docker exec px4-bridge-drone-1 ps aux | grep px4

# View PX4 console output
docker logs -f px4-bridge-drone-1

# Interactive PX4 shell
docker exec -it px4-bridge-drone-1 /px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4-shell
```

### 5. Test MAVLink Connectivity
```bash
cd tools/

# Comprehensive connectivity test
python3 unified_mavlink_tester.py --drone-range "1-3"

# Test QGroundControl ports
python3 test_correct_qgc_ports.py

# Advanced diagnostic with auto-fix
python3 mavlink_diagnostic.py

# Verify specific drone connectivity
python3 test_specific_drone.py
```

### 6. Connect QGroundControl
```bash
# Find available QGC ports
python3 test_correct_qgc_ports.py

# QGroundControl configuration:
# 1. Add Comm Link: UDP, Port 14540-14542, Auto Connect
# 2. Or use discovered working ports from test output
```

### 7. API Client Connection
```python
# Python example using cosysairsim
import cosysairsim

# Connect to different drone instances
client1 = cosysairsim.MultirotorClient(port=14540)  # Drone 1 API port
client2 = cosysairsim.MultirotorClient(port=14541)  # Drone 2 API port
client3 = cosysairsim.MultirotorClient(port=14542)  # Drone 3 API port

# Verify connections
print(client1.getMultirotorState())
```

### Troubleshooting Setup Issues

#### Container Won't Start
```bash
# Check Docker daemon and compose file
docker --version && docker-compose --version
docker-compose -f docker-compose-slim.yml config

# Check resource usage
docker stats
```

#### No MAVLink Connectivity
```bash
# Verify port mappings
docker port px4-bridge-drone-1

# Check firewall (Windows + WSL2)
# Run as Administrator in PowerShell:
# New-NetFirewallRule -DisplayName 'QGC MAVLink' -Direction Inbound -Protocol UDP -LocalPort 14540-14549 -Action Allow

# Test internal container connectivity
docker exec px4-bridge-drone-1 netstat -tulpn | grep 14540
```

#### AirSim Connection Issues
```bash
# Verify AirSim accessibility from container
docker exec px4-bridge-drone-1 ping $PX4_SIM_HOSTNAME

# Check AirSim settings.json for correct vehicle configuration
# Ensure TCP HIL ports 4560+ are configured in AirSim settings
```

## Port Configuration Overview

Based on `docker-compose-slim.yml`, each drone exposes three types of ports:

| Drone | TCP HIL (AirSim) | UDP Control | QGC Port (localhost) | Container IP |
|-------|------------------|-------------|---------------------|--------------|
| 1     | 4561             | 14581       | 14541               | 172.20.0.11  |
| 2     | 4562             | 14582       | 14542               | 172.20.0.12  |
| 3     | 4563             | 14583       | 14543               | 172.20.0.13  |
| 4     | 4564             | 14584       | 14544               | 172.20.0.14  |
| 5     | 4565             | 14585       | 14545               | 172.20.0.15  |

## 📡 MAVLink Network Configuration (`../config/px4-rc.mavlink.network`)

The MAVLink configuration script handles external network connectivity for QGroundControl and API clients:

### Key Features
- **Port Mapping**: Automatically calculates ports based on PX4_INSTANCE
- **Multi-Link Support**: Configures GCS, API, payload, and gimbal links
- **AirSim Integration**: Special configuration when `PX4_SIMULATOR=airsim`
- **Target Host**: Configurable via `MAVLINK_TARGET` environment variable

### Port Calculations
```bash
# QGroundControl (GCS) Link
QGC_PORT = 14550 + PX4_INSTANCE    # 14550, 14551, 14552...

# API/MAVSDK Link  
API_PORT = 14540 + PX4_INSTANCE    # 14540, 14541, 14542...

# Internal container ports
GCS_LOCAL = 18570 + PX4_INSTANCE   # 18570, 18571, 18572...
API_LOCAL = 14580 + PX4_INSTANCE   # 14580, 14581, 14582...
```

### Environment Variables
- `MAVLINK_TARGET`: Target host IP (default: 192.168.65.254)
- `PX4_INSTANCE`: Instance number (0-based, set automatically)
- `PX4_SIMULATOR`: Set to "airsim" for AirSim-specific streams
- `AIRSIM_ENABLED`: Alternative flag for AirSim configuration

### MAVLink Stream Configuration
The script configures optimized message streams for different use cases:

#### GCS Streams (QGroundControl)
- High-rate: Position, attitude, servo outputs (50 Hz)
- Medium-rate: VFR_HUD, RC channels (20 Hz)  
- Low-rate: GPS, battery, system state (1-5 Hz)

#### API Streams (MAVSDK/DroneKit)
- Uses onboard mode for optimal API performance
- Additional high-rate IMU data (100 Hz) for AirSim

#### AirSim-Specific Streams
- ALTITUDE, SYSTEM_TIME (10-5 Hz)
- HIGHRES_IMU (100 Hz)
- ACTUATOR_OUTPUTS (50 Hz)

### Usage in Container
```bash
# Automatic execution via PX4 startup
# Called from: /px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS

# Manual execution for debugging
docker exec px4-bridge-drone-1 /px4_workspace/PX4-Autopilot/config/px4-rc.mavlink.network

# Check MAVLink status
docker exec px4-bridge-drone-1 /px4_workspace/PX4-Autopilot/build/px4_sitl_default/bin/px4-shell mavlink status
```

## 🐳 Docker Compose Configuration (`../docker-compose-slim.yml`)

The Docker Compose file orchestrates up to 9 PX4 drone containers with standardized networking and port mapping:

### Architecture Features
- **Bridge Network**: Uses `px4_network` (172.20.0.0/16) for container communication
- **Static IPs**: Each drone gets a predictable IP (172.20.0.11-172.20.0.19)
- **Port Mapping**: Exposes TCP (AirSim HIL) and UDP (MAVLink) ports to host
- **Shared Storage**: Common volume `px4-shared-data` for inter-drone communication
- **Health Checks**: Automated health monitoring with recovery

### Container Configuration Template
Each drone container follows this pattern:
```yaml
services:
  px4-bridge-drone-X:
    image: px4-airsim:slim
    container_name: px4-bridge-drone-X
    environment:
      PX4_INSTANCE: Y                    # 0-based instance (X-1)  
      PX4_SIM_HOSTNAME: ${PX4_SIM_HOSTNAME:-172.28.240.1}  # AirSim host
      PX4_SIM_MODEL: none_iris           # AirSim-compatible model
      PX4_HOME_LAT: 43.066...            # Boise, Idaho coordinates
      PX4_HOME_LON: -115.854...
      SWARM_ID: 1                        # Swarm identifier
      SWARM_SIZE: 9                      # Total swarm size
    ports:
      - "456X:456X/tcp"                  # AirSim HIL port
      - "1458X:1458X/udp"                # Control/API port  
      - "1454X:1454X/udp"                # QGroundControl port
    networks:
      px4_network:
        ipv4_address: 172.20.0.1X        # Static container IP
```

### Environment Variables
- `PX4_SIM_HOSTNAME`: AirSim host IP (Docker Desktop: 172.28.240.1, WSL2: detected IP)
- `PX4_INSTANCE`: 0-based drone instance for port calculations
- `SWARM_ID`/`SWARM_SIZE`: Swarm coordination parameters
- `MAV_*_BROADCAST`: Enable MAVLink broadcasting for external connectivity

### Port Mapping Strategy
```bash
# Drone 1 (PX4_INSTANCE=0):
TCP: 4560 → AirSim HIL connection
UDP: 14580 → API/Control port
UDP: 14540 → QGroundControl port

# Drone 2 (PX4_INSTANCE=1):  
TCP: 4561 → AirSim HIL connection
UDP: 14581 → API/Control port
UDP: 14541 → QGroundControl port
```

### Health Check Configuration
```yaml
healthcheck:
  test: ["CMD", "/px4_workspace/PX4-Autopilot/Scripts/ultra_swarm_health_check.sh"]
  interval: 30s      # Check every 30 seconds
  timeout: 10s       # 10 second timeout
  retries: 3         # 3 failed attempts before unhealthy
  start_period: 60s  # Grace period for startup
```

### Common Commands
```bash
# Start specific drones
docker-compose -f docker-compose-slim.yml up px4-bridge-drone-1 px4-bridge-drone-2

# Start full swarm (9 drones)
docker-compose -f docker-compose-slim.yml up

# Background mode
docker-compose -f docker-compose-slim.yml up -d px4-bridge-drone-{1..3}

# Check status
docker-compose -f docker-compose-slim.yml ps

# View logs
docker-compose -f docker-compose-slim.yml logs px4-bridge-drone-1

# Scale specific services
docker-compose -f docker-compose-slim.yml up --scale px4-bridge-drone-1=1

# Clean shutdown
docker-compose -f docker-compose-slim.yml down

# Cleanup with volumes
docker-compose -f docker-compose-slim.yml down -v
```

### Networking Details
- **Bridge Driver**: Standard Docker bridge with custom subnet
- **Container-to-Container**: Direct communication via static IPs
- **Host Access**: Port mapping enables external client connections
- **AirSim Integration**: Uses `PX4_SIM_HOSTNAME` for cross-platform compatibility

## 🛠 PX4 SITL Startup Script (`../scripts/run_airsim_sitl.sh`)

The startup script handles PX4 SITL initialization within containers:

### Key Features
- **User-Friendly Numbering**: Takes drone numbers 1-9, converts to 0-based instances
- **AirSim Compatibility**: Uses `none_iris` model for external simulator integration
- **Port Calculation**: Automatically calculates MAVLink ports based on drone number
- **Path Management**: Handles PX4 build directory structure and working directories

### Usage
```bash
# Inside container or standalone
./run_airsim_sitl.sh 1    # Start drone 1 (PX4_INSTANCE=0, ports 14550/14540)
./run_airsim_sitl.sh 2    # Start drone 2 (PX4_INSTANCE=1, ports 14551/14541)
./run_airsim_sitl.sh 5    # Start drone 5 (PX4_INSTANCE=4, ports 14554/14544)
```

### Port Assignment Logic
```bash
drone_number=${1:-1}                    # User input (1-9)
instance_num=$((drone_number - 1))      # Convert to 0-based (0-8)
qgc_port=$((14550 + instance_num))      # QGC: 14550, 14551, 14552...  
api_port=$((14540 + instance_num))      # API: 14540, 14541, 14542...
```

### Directory Structure
```bash
PARENT_DIR=/px4_workspace/PX4-Autopilot
BUILD_DIR=$PARENT_DIR/ROMFS/px4fmu_common
BIN_DIR=$PARENT_DIR/build/px4_sitl_default/bin/px4
working_dir=$PARENT_DIR/build/px4_sitl_default/instance_$instance_num
```

### Environment Export
- `PX4_INSTANCE`: 0-based instance for MAVLink config
- `PX4_SIM_MODEL`: Set to `none_iris` for AirSim compatibility

## 📦 Docker Image Configuration (`../Dockerfile.slim`)

The Dockerfile creates a multi-stage Ubuntu-based image optimized for PX4 SITL:

### Build Strategy
- **Builder Stage**: Ubuntu 22.04 with full development tools
- **Runtime Stage**: Slim Ubuntu with only necessary runtime dependencies
- **Multi-Stage Benefits**: Smaller final image, faster deployment, better security

### Key Components

#### Builder Stage Features
```dockerfile
FROM ubuntu:22.04 AS builder
# Install build dependencies: git, cmake, python3, build tools
# Clone PX4-Autopilot v1.14.0 with submodules
# Install PX4 dependencies via ./Tools/setup/ubuntu.sh
# Fix EmPy version compatibility (empy==3.3.4)
# Build PX4 SITL with none_iris model
```

#### PX4 Build Configuration
```bash
# Build targets included:
make px4_sitl_default none_iris    # AirSim-compatible model
# Additional models: iris, typhoon_h480, etc.
# Build output: /px4_workspace/PX4-Autopilot/build/px4_sitl_default/
```

#### Runtime Stage Features
```dockerfile 
FROM ubuntu:22.04 AS runtime
# Copy built PX4 from builder stage
# Install runtime dependencies only
# Configure ultra-swarm scripts and MAVLink configuration
# Set up health check scripts
# Configure user permissions and working directory
```

### Image Structure
```bash
/px4_workspace/
├── PX4-Autopilot/                    # Main PX4 directory
│   ├── build/px4_sitl_default/       # Built binaries and configs
│   ├── config/                       # MAVLink configuration scripts
│   ├── Scripts/                      # Health check and utility scripts
│   └── ROMFS/                        # Runtime file system
├── scripts/                          # Startup scripts
└── logs/                             # Container log directory
```

### Build Process
```bash
# Build the slim image
docker build -f Dockerfile.slim -t px4-airsim:slim .

# Build with custom PX4 version
docker build -f Dockerfile.slim --build-arg PX4_VERSION=v1.14.1 -t px4-airsim:slim .

# Check image size
docker images px4-airsim:slim

# Inspect image layers
docker history px4-airsim:slim
```

### Runtime Optimizations
- **Dependency Management**: Only essential runtime libraries included
- **Layer Caching**: Optimized layer structure for rebuild efficiency  
- **Security**: Non-root user configuration for container security
- **Logging**: Structured logging to `/px4_workspace/logs/`
- **Health Checks**: Built-in monitoring via `ultra_swarm_health_check.sh`

### Environment Configuration
The image supports these environment variables:
- `PX4_INSTANCE`: Drone instance number (0-8)
- `PX4_SIM_HOSTNAME`: AirSim host IP address  
- `PX4_SIM_MODEL`: Vehicle model (default: none_iris)
- `MAVLINK_TARGET`: Target for MAVLink connections
- `SWARM_ID`/`SWARM_SIZE`: Swarm coordination

### Troubleshooting Build Issues
```bash
# Check build logs for common issues:
docker build -f Dockerfile.slim -t px4-airsim:slim . --no-cache

# Common fixes:
# 1. EmPy version conflicts → Fixed by pinning empy==3.3.4
# 2. Submodule errors → Ensure git access and recursive clone
# 3. Build failures → Check PX4 version compatibility
# 4. Runtime errors → Verify dependency installation
```

## Testing Tools Overview

### 🎯 Primary Testing Tools

#### 1. `unified_mavlink_tester.py` (NEW)
**Purpose**: Master orchestrator that runs all tests in sequence
```bash
# Test specific drone range
python3 unified_mavlink_tester.py --drone-range "1-3"

# Save results to JSON
python3 unified_mavlink_tester.py --output results.json

# Quick test mode
python3 unified_mavlink_tester.py --quick

# JSON output only
python3 unified_mavlink_tester.py --json-only
```

**What it tests**:
- Environment detection (WSL2, Docker networking)
- Container health status
- Docker port mappings validation
- MAVLink connectivity using all other tools
- QGroundControl compatibility

**Output**: Comprehensive report with recommendations

#### 2. `mavlink_diagnostic.py`
**Purpose**: Comprehensive MAVLink connection diagnostic with auto-fix
```bash
python3 mavlink_diagnostic.py
```

**What it does**:
- Tests UDP listeners on MAVLink ports
- Checks Docker port mappings
- Analyzes container MAVLink output
- Tests network routing (WSL2 detection)
- Attempts automatic fixes for common issues
- Provides specific troubleshooting steps

#### 3. `test_correct_qgc_ports.py` (NEW)
**Purpose**: Tests actual QGC ports from docker-compose-slim.yml (14541-14549)
```bash
python3 test_correct_qgc_ports.py
```

**What it tests**:
- Sends MAVLink heartbeat to each QGC port
- Validates MAVLink responses
- Provides QGroundControl configuration recommendations

### 🔧 Specialized Testing Tools

#### 4. `test_mavlink_connection.py`
**Purpose**: MAVLink heartbeat testing for QGroundControl
```bash
python3 test_mavlink_connection.py
```
- Tests ports 14550-14554 (legacy QGC ports)
- Sends MAVLink HEARTBEAT messages
- Validates protocol responses

#### 5. `test_specific_drone.py`
**Purpose**: Test individual drone connections
```bash
python3 test_specific_drone.py
```
- Tests direct container connections
- Tests port mapping connections
- Focused on drone 1 (172.20.0.11)

#### 6. `test_udp_control_ports.py` (NEW)
**Purpose**: Test UDP control ports 14581-14589 and QGC ports 14541-14549
```bash
# Test control ports only
python3 test_udp_control_ports.py --control-only

# Test QGC ports only  
python3 test_udp_control_ports.py --qgc-only

# Test all ports
python3 test_udp_control_ports.py
```

#### 7. `simple_qgc_test.py`
**Purpose**: Basic QGroundControl connectivity test
```bash
python3 simple_qgc_test.py
```

#### 8. `qgc_fix.py`
**Purpose**: QGroundControl connection diagnostic and fix utility
```bash
python3 qgc_fix.py
```

### 🌐 Network Testing Tools

#### 9. `test_bridge_network.sh`
**Purpose**: Comprehensive bridge network test suite
```bash
./test_bridge_network.sh
```

#### 10. `test_container_connection.py`
**Purpose**: Direct container MAVLink testing
```bash
python3 test_container_connection.py
```

#### 11. `test-connectivity.sh`
**Purpose**: AirSim connectivity verification
```bash
./test-connectivity.sh
```

### 📊 Advanced Network Tools

#### 12. `network_tester.py` (in docker_clean/)
**Purpose**: Advanced network connectivity with environment detection
```bash
# From docker_clean/config_generator/tools/
python3 network_tester.py --hosts localhost 127.0.0.1
python3 network_tester.py --json  # JSON output
```

#### 13. `mavlink_debug.py`
**Purpose**: Advanced MAVLink packet monitor with message decoding
```bash
python3 mavlink_debug.py --port 14581
```

### 🔥 Utility Tools

#### 14. `fix_mavlink_broadcast.sh`
**Purpose**: Fix MAVLink broadcast configuration
```bash
./fix_mavlink_broadcast.sh
```

#### 15. `test_udp_ports.ps1` (Windows)
**Purpose**: Windows PowerShell UDP port testing
```powershell
.\test_udp_ports.ps1
```

## Testing Workflows

### 🚀 Quick Health Check
```bash
# 1. Quick comprehensive test
python3 unified_mavlink_tester.py --quick --drone-range "1-3"

# 2. Check specific QGC ports
python3 test_correct_qgc_ports.py
```

### 🔍 Deep Diagnostic
```bash
# 1. Full diagnostic with auto-fix
python3 mavlink_diagnostic.py

# 2. Network analysis
python3 ../../docker_clean/config_generator/tools/network_tester.py

# 3. Container-specific testing
python3 test_specific_drone.py
```

### 🎮 QGroundControl Setup Validation
```bash
# 1. Test correct QGC ports
python3 test_correct_qgc_ports.py

# 2. QGC-specific diagnostics
python3 qgc_fix.py

# 3. Legacy QGC port testing
python3 test_mavlink_connection.py
```

### 🐳 Container Debugging
```bash
# 1. Check container health
docker ps --filter "name=px4-bridge"

# 2. Test container connections
python3 test_container_connection.py

# 3. Check MAVLink configuration
docker exec px4-bridge-drone-1 cat /px4_workspace/PX4-Autopilot/build/px4_sitl_default/instance_1/broadcast_params.txt
```

## Expected Test Results

### ✅ Healthy System
- **Containers**: 3-9 running containers with "healthy" status
- **TCP HIL Ports**: 4561-456X listening and mapped
- **UDP Control Ports**: 14581-1458X in use by PX4 (good sign)
- **QGC Ports**: 14541-1454X listening on localhost

### ⚠️ Common Issues & Solutions

#### Issue: No MAVLink packets on QGC ports
**Symptoms**: QGC ports listening but no MAVLink responses
**Cause**: PX4 not configured to broadcast to QGC ports
**Solution**: 
```bash
# Check if SWARM_ID is set to enable full MAVLink broadcasting
docker exec px4-bridge-drone-1 env | grep SWARM
```

#### Issue: Port mapping errors
**Symptoms**: Docker port commands fail
**Cause**: Containers not running or port conflicts
**Solution**:
```bash
docker-compose -f docker-compose-slim.yml restart
```

#### Issue: WSL2 connectivity problems
**Symptoms**: Tests work in container but not from host
**Cause**: Windows firewall or WSL2 networking
**Solution**:
```powershell
# Windows (as Administrator)
New-NetFirewallRule -DisplayName 'QGC UDP' -Direction Inbound -Protocol UDP -LocalPort 14541-14549 -Action Allow
```

## Tool Dependencies

### Python Requirements
Most tools use only standard library modules:
- `socket` - Network communication
- `struct` - Binary data handling  
- `subprocess` - Running system commands
- `json` - JSON output formatting
- `time` - Timeouts and timing
- `threading` - Concurrent operations

### System Requirements
- Docker with containers running
- Python 3.6+
- Network access to containers
- For Windows: PowerShell for Windows-specific tools

## Tool Comparison Matrix

| Tool | Container Health | Port Mapping | MAVLink Packets | Environment Detection | Auto-Fix | JSON Output |
|------|------------------|--------------|-----------------|----------------------|----------|-------------|
| unified_mavlink_tester.py | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| mavlink_diagnostic.py | ❌ | ✅ | ✅ | ✅ | ✅ | ❌ |
| test_correct_qgc_ports.py | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ |
| test_mavlink_connection.py | ❌ | ❌ | ✅ | ❌ | ❌ | ❌ |
| network_tester.py | ❌ | ❌ | ❌ | ✅ | ❌ | ✅ |

## Integration with CI/CD

### GitHub Actions Example
```yaml
- name: Test MAVLink Connectivity
  run: |
    cd docker/px4_airsim_docker
    python3 tools/unified_mavlink_tester.py --json-only --output test_results.json
    # Parse results and fail if issues found
```

### Automated Monitoring
```bash
# Cron job for continuous monitoring
*/5 * * * * cd /path/to/project && python3 docker/px4_airsim_docker/tools/unified_mavlink_tester.py --quick --json-only >> /var/log/mavlink_health.log
```

## Troubleshooting Guide

### No containers running
```bash
cd /path/to/docker/px4_airsim_docker
docker-compose -f docker-compose-slim.yml up -d
```

### Containers healthy but no MAVLink
1. Check if AirSim is running and reachable
2. Verify `PX4_SIM_HOSTNAME` environment variable
3. Check firewall settings (especially Windows + WSL2)

### QGroundControl can't connect
1. Use `test_correct_qgc_ports.py` to find working ports
2. Configure QGC for UDP on localhost with detected port
3. Ensure QGC is not using TCP mode

### Performance issues
1. Use `--quick` flag for faster testing
2. Test specific drone ranges with `--drone-range "1-3"`
3. Consider using container networking mode for better performance

## Contributing

When adding new testing tools:

1. **Follow naming convention**: `test_specific_purpose.py`
2. **Add help/usage**: Include `--help` argument parsing
3. **Provide clear output**: Use emoji indicators (✅❌⚠️) for status
4. **Handle errors gracefully**: Don't crash on network timeouts
5. **Document in this README**: Add tool to appropriate section
6. **Update unified_mavlink_tester.py**: Integrate into master orchestrator if relevant

## License

These testing tools are part of the Cosys-AirSim project and follow the same MIT license.