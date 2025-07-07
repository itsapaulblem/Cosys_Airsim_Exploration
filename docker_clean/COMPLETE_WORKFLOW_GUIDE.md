# Complete AirSim + Docker PX4 Workflow Guide

## Overview ✅

This guide provides the complete step-by-step workflow for running AirSim with the fixed Docker PX4 setup. The key architectural principle is:

- **AirSim = Server** (runs on host, listens for connections)
- **PX4 Containers = Clients** (connect outbound to AirSim)

## 🚀 Step-by-Step Workflow

### Phase 1: Prerequisites

#### 1.1 Verify Docker Setup
```bash
cd /mnt/l/Cosys-AirSim/docker_clean/multi_drone
ls -la docker-compose.yml  # Ensure file exists
docker-compose config      # Validate configuration
```

#### 1.2 Check Available Profiles
```bash
# List available drone profiles
docker-compose config --profiles

# Available profiles:
# - drone-1, drone-2, drone-3, drone-4, drone-5
# - all (starts all drones)
```

### Phase 2: Configure AirSim

#### 2.1 Generate Configuration (Recommended) 🔥
Use the **unified configuration generator** with built-in BP_SpiritPawn support:

```bash
cd /mnt/l/Cosys-AirSim/docker_clean/config_generator/tools

# Generate complete configuration for 3 drones (settings.json + docker-compose.yml)
python3 unified_generator.py multi --num-drones 3

# Generate with cameras for advanced vision workflows
python3 unified_generator.py multi --num-drones 2 \
    --cameras front_rgb front_depth --create-rviz

# Generate single drone with complete camera setup
python3 unified_generator.py single \
    --cameras front_rgb downward_cam segmentation_cam --create-rviz

# Generate only settings.json with 5 drones
python3 unified_generator.py multi --num-drones 5 --settings-only

# Generate with different layouts
python3 unified_generator.py multi --num-drones 4 --layout circle
python3 unified_generator.py multi --num-drones 6 --layout line

# Custom output directory
python3 unified_generator.py multi --num-drones 3 --output-dir ./my_configs
```

**Generated configuration includes:**
- ✅ **BP_SpiritPawn**: `"PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"` (default)
- ✅ **Camera integration**: 7 presets (RGB, depth, segmentation, stereo, downward, gimbal)
- ✅ **SubWindows**: Live camera feeds in Unreal simulation window
- ✅ **RViz2 configuration**: Pre-configured camera visualization for ROS2
- ✅ **Fixed Docker networking**: No TCP port mappings (HIL communication fix applied)
- ✅ **Proper ControlPorts**: Auto-calculated for each drone
- ✅ **LockStep enabled**: For deterministic simulation  
- ✅ **Launcher scripts**: Ready-to-use startup scripts

#### 2.2 Manual Settings (Alternative)
If you prefer manual configuration, create `~/Documents/AirSim/settings.json`:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "PawnPaths": {
    "DefaultQuadrotor": {
      "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
    }
  },
  "Vehicles": {
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "X": 0, "Y": 0, "Z": 0,
      "UseSerial": false,
      "LockStep": true,
      "ControlPort": 4561
    },
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor", 
      "X": 5, "Y": 0, "Z": 0,
      "UseSerial": false,
      "LockStep": true,
      "ControlPort": 4562
    },
    "PX4_Drone3": {
      "VehicleType": "PX4Multirotor",
      "X": -5, "Y": 0, "Z": 0,
      "UseSerial": false,
      "LockStep": true,
      "ControlPort": 4563
    },
    "PX4_Drone4": {
      "VehicleType": "PX4Multirotor",
      "X": 0, "Y": 5, "Z": 0,
      "UseSerial": false,
      "LockStep": true,
      "ControlPort": 4564
    },
    "PX4_Drone5": {
      "VehicleType": "PX4Multirotor",
      "X": 0, "Y": -5, "Z": 0,
      "UseSerial": false,
      "LockStep": true,
      "ControlPort": 4565
    }
  }
}
```

**Key Configuration Notes:**
- `PawnPaths`: Specifies BP_SpiritPawn as default quadrotor pawn
- `ControlPort`: Must match PX4_INSTANCE + 4560 (drone 1 = 4561, drone 2 = 4562, etc.)
- `LockStep: true`: Essential for deterministic simulation
- `UseSerial: false`: We use TCP connections, not serial

### Phase 3: Startup Sequence (CRITICAL ORDER!)

#### 3.1 Start AirSim FIRST 🔥
```bash
# Launch your Unreal Engine project with AirSim
# This MUST be done before starting PX4 containers!

# From your Unreal project directory:
./YourProject.exe  # Windows
# OR
./YourProject      # Linux
```

**Wait for AirSim to show:**
```
[AirSim] Waiting for connection on port 4561
[AirSim] Waiting for connection on port 4562
[AirSim] Waiting for connection on port 4563
[AirSim] Waiting for connection on port 4564
[AirSim] Waiting for connection on port 4565
```

#### 3.2 Start PX4 Containers SECOND
```bash
cd /mnt/l/Cosys-AirSim/docker_clean/multi_drone

# Option A: Start specific number of drones
docker-compose --profile drone-1 up -d                    # 1 drone
docker-compose --profile drone-1 --profile drone-2 up -d  # 2 drones
docker-compose --profile all up -d                        # All 5 drones

# Option B: Start for testing (with logs)
docker-compose --profile drone-1 up  # See real-time logs
```

#### 3.3 Verify Connections
**Expected AirSim Logs:**
```
[AirSim] Client connected to port 4561
[AirSim] HIL connection established for PX4_Drone1
[AirSim] Client connected to port 4562  
[AirSim] HIL connection established for PX4_Drone2
```

**Expected PX4 Logs:**
```bash
# Check container logs
docker logs px4-drone-1

# Should show:
# INFO [simulator] Simulator connected on TCP port 4561
# INFO [ekf2] EKF GPS checks passed
# INFO [commander] Ready for takeoff
```

### Phase 4: Connect Additional Tools

#### 4.1 Start QGroundControl (Optional)
```bash
# QGroundControl will auto-detect drones on:
# - localhost:14550 (drone 1)
# - localhost:14551 (drone 2) 
# - localhost:14552 (drone 3)
# - localhost:14553 (drone 4)
# - localhost:14554 (drone 5)
```

#### 4.2 Test Python Client Connection
```bash
cd /mnt/l/Cosys-AirSim/docker_clean/testing_scripts
python3 test_hil_gps_fix.py
```

### Phase 5: Run Missions

#### 5.1 Test Basic Mission
```bash
cd /mnt/l/Cosys-AirSim/PythonClient/multirotor/multi_drone

# Preview mission (no execution)
python3 multi_drone_orbit_mission.py --preview

# Run actual mission
python3 multi_drone_orbit_mission.py --num_drones 2 --altitude 20
```

#### 5.2 Expected Results
- ✅ GPS home location established automatically
- ✅ `armDisarm()` succeeds without errors
- ✅ All drones takeoff and execute mission
- ✅ No HIL communication failures

## 🔧 Troubleshooting Guide

### Issue 1: "Connection Refused"
**Symptoms:** PX4 containers show "Connection refused on port 456X"

**Solutions:**
```bash
# 1. Verify AirSim is running and listening
netstat -an | grep 456  # Should show LISTENING ports

# 2. Check Docker host connectivity  
docker exec -it px4-drone-1 ping host.docker.internal

# 3. For Linux, add host mapping to docker-compose.yml:
extra_hosts:
  - "host.docker.internal:host-gateway"
```

### Issue 2: "GPS Home Location" Error
**Symptoms:** `armDisarm()` fails with GPS error

**Root Cause:** HIL connection not established properly

**Solutions:**
```bash
# 1. Verify startup order (AirSim first!)
# 2. Check AirSim logs for "Client connected"
# 3. Check PX4 logs for "Simulator connected" 
# 4. Ensure settings.json has correct ControlPort values
```

### Issue 3: Container Startup Failures
**Symptoms:** Containers exit immediately or stay in "Created" state

**Solutions:**
```bash
# 1. Check container logs
docker logs px4-drone-1

# 2. Verify no port conflicts
docker ps --format "table {{.Names}}\t{{.Ports}}"

# 3. Clean restart
docker-compose down -v
docker-compose --profile drone-1 up -d
```

### Issue 4: Firewall Blocking Connections
**Symptoms:** Network timeouts, connection refused

**Solutions:**
```bash
# Windows: Allow ports 4561-4565 in Windows Defender Firewall
# Linux: Allow Docker subnet access
sudo ufw allow from 172.0.0.0/8 to any port 4561:4565
```

## 🧪 Testing Commands

### Configuration Generation
```bash
cd docker_clean/config_generator/tools

# Generate complete multi-drone setup with Spirit Pawn
python3 unified_generator.py multi --num-drones 3

# Generate with cameras and RViz2 configuration
python3 unified_generator.py multi --num-drones 2 \
    --cameras front_rgb front_depth --create-rviz

# Generate single drone configuration  
python3 unified_generator.py single

# Generate single drone with complete camera suite
python3 unified_generator.py single \
    --cameras front_rgb downward_cam segmentation_cam --create-rviz

# Generate with different layouts
python3 unified_generator.py multi --num-drones 4 --layout circle

# Generate only settings.json
python3 unified_generator.py multi --num-drones 5 --settings-only
```

### Settings Validation (Alternative Tool)
```bash
cd docker_clean/testing_scripts

# Validate existing settings with secondary tool
python3 generate_airsim_settings.py --validate_only

# Generate backup settings with alternative generator
python3 generate_airsim_settings.py --num_drones 3 --preview
```

### Comprehensive Test Suite
```bash
# 1. Generate and validate settings
cd docker_clean/testing_scripts
python3 generate_airsim_settings.py --num_drones 3
python3 generate_airsim_settings.py --validate_only

# 2. Test Docker containers
python3 test_connections.py

# 3. Test HIL and GPS fix
python3 test_hil_gps_fix.py

# 4. Test AirSim connection
python3 test_airsim_connection.py 4561

# 5. Test multi-drone mission
cd ../../PythonClient/multirotor/multi_drone
python3 multi_drone_orbit_mission.py --preview --num_drones 2
```

### Individual Component Tests
```bash
# Test single drone
docker-compose --profile drone-1 up -d
python3 test_hil_gps_fix.py

# Test multi-drone
docker-compose --profile drone-1 --profile drone-2 up -d
python3 test_connections.py
```

## 📋 Quick Reference

### Port Mappings
| Service | Container Port | Host Port | Protocol | Purpose |
|---------|----------------|-----------|----------|---------|
| HIL | 4561-4565 | N/A | TCP | PX4 → AirSim (outbound only) |
| QGC | 14550 | 14550-14554 | UDP | QGroundControl |
| MAVLink | 14541-14545 | 14541-14545 | UDP | Python API |
| MAVLink Remote | 14581-14585 | 14581-14585 | UDP | Remote control |

### Environment Variables
```bash
PX4_INSTANCE=1                    # Drone number (1-5)
PX4_SIM_HOSTNAME=host.docker.internal  # Host connection
PX4_SIMULATOR=none               # External simulator mode
PX4_SIM_MODEL=none_iris         # Vehicle model
```

### Container Management
```bash
# Start
docker-compose --profile drone-1 up -d

# Stop
docker-compose --profile drone-1 down

# Logs
docker logs px4-drone-1 -f

# Clean restart
docker-compose down -v && docker-compose --profile drone-1 up -d
```

## ✅ Success Indicators

**When everything is working correctly, you should see:**

1. **AirSim:** "Client connected" messages for each drone
2. **PX4 Logs:** "Simulator connected" and "EKF GPS checks passed"
3. **Python Client:** Successful `armDisarm()` without GPS errors
4. **Mission Execution:** Drones takeoff and follow planned routes
5. **QGroundControl:** All drones visible and controllable

The workflow ensures proper HIL communication while maintaining all debugging and control capabilities.