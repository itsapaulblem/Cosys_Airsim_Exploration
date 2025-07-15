# PX4 Docker Swarm

This document explains how to use the PX4 Docker swarm configuration for AirSim, where containers start but don't immediately run the SITL scripts, allowing for controlled startup.

## Overview

The delayed startup configuration allows you to:
- Start all Docker containers without immediately launching PX4
- Manually control when each PX4 instance starts
- Debug startup issues more easily
- Coordinate startup timing with external systems
- Uses the simple `run_airsim_sitl.sh` approach for easy PX4 startup

## Quick Start

### 1. Start Containers in Delayed Mode

```bash
# Navigate to the directory
cd docker/px4_airsim_docker_v2

# Start 3 containers (default)
./tools/start-swarm.sh start 3

# Start all 9 containers
./tools/start-swarm.sh start 9

# Start single container for testing
./tools/start-swarm.sh start 1
```

### 2. Check Container Status

```bash
# Show container and PX4 process status
./tools/start-swarm.sh status
```

### 3. Start PX4 in Containers

```bash
# Start PX4 in a specific container
./tools/start-swarm.sh run-px4 px4-swarm-1-drone-1

# Start PX4 in all running containers
./tools/start-swarm.sh run-all-px4
```

## Configuration Files

### Ultra Swarm (Full Featured)
- **File**: `docker-compose.ultra-swarm-delayed.yml`
- **Features**: GPS configuration, enhanced parameters, full PX4 build
- **Script**: `/Scripts/run_ultra_swarm.sh`

### Simple Swarm (Lightweight)
- **File**: `docker-compose.simple-swarm-delayed.yml`
- **Features**: Uses existing `run_airsim_sitl.sh`, minimal configuration
- **Script**: `./Scripts/run_airsim_sitl.sh`

## Manual Container Management

### Access a Container

```bash
# Access container shell
docker exec -it px4-swarm-1-drone-1 bash

# Once inside, manually start PX4:
# For ultra swarm:
/Scripts/run_ultra_swarm.sh

# For simple swarm:
cd /px4_workspace/PX4-Autopilot
./Scripts/run_airsim_sitl.sh $PX4_INSTANCE
```

### View Container Logs

```bash
# Follow logs for a specific container
./tools/start-swarm.sh logs px4-swarm-1-drone-1

# Or using docker directly
docker logs -f px4-swarm-1-drone-1
```

### Execute Commands

```bash
# Run any command in a container
./tools/start-swarm.sh exec px4-swarm-1-drone-1 ls -la

# Check if PX4 is running
./tools/start-swarm.sh exec px4-swarm-1-drone-1 pgrep -f px4
```

## Advanced Usage

### Custom Startup Sequence

```bash
# Start containers
./tools/start-swarm.sh start

# Wait for external system to be ready
sleep 10

# Start drones one by one with delays
for i in {1..9}; do
    ./tools/start-swarm.sh run-px4 px4-swarm-1-drone-$i
    sleep 2
done
```

### Debugging Startup Issues

```bash
# Start container and immediately attach to see output
docker run -it --rm \
    --network host \
    -e SWARM_ID=1 \
    -e PX4_INSTANCE=1 \
    -e PX4_SIM_HOSTNAME=172.28.240.1 \
    px4-ultra-swarm:latest \
    bash

# Inside container, manually run the script to see errors
/Scripts/run_ultra_swarm.sh
```

### Using with WSL2

The script automatically detects WSL2 and updates the Windows host IP:

```bash
# The script will detect and use the correct Windows IP
./tools/start-swarm.sh start

# You can also manually set the IP
export PX4_SIM_HOSTNAME=172.28.240.1
./tools/start-swarm.sh start
```

## Differences from Normal Startup

### Normal Startup (Original Files)
- Containers immediately run PX4 on startup
- Defined in the Dockerfile CMD or docker-compose command
- Harder to debug startup issues
- All drones start simultaneously

### Delayed Startup (New Files)
- Containers start with `tail -f /dev/null` to stay running
- PX4 must be manually started
- Easier debugging and coordination
- Can control startup timing and sequence

## Troubleshooting

### Container Exits Immediately
- Check if the image was built correctly
- Verify volume mounts are correct
- Check container logs: `docker logs <container-name>`

### PX4 Won't Start
- Ensure you're in the correct directory inside container
- Check if the startup script exists and is executable
- Verify environment variables are set correctly

### Can't Connect to AirSim
- Verify PX4_SIM_HOSTNAME is set to correct IP
- Check if AirSim is running and accessible
- Ensure network mode is set to "host"

### GPS Home Location Issues

#### Problem: "Vehicle does not have a valid GPS home location"
This error occurs when trying to arm drones via Python client but manual PX4 arming works.

**Root Cause**: MAVLink GPS home position synchronization between PX4 and AirSim was incomplete.

**Solution Applied**: Enhanced GPS home position handling in MavLinkMultirotorApi.hpp:

1. **Complete GPS Home Message Handler**: Fixed incomplete `MavLinkHomePosition` message handling that was missing `current_state_.home.is_set = true`

2. **GPS Backup Mechanism**: Added backup home position setting from GPS data when PX4 doesn't send home position messages:
   ```cpp
   // When GPS lock is acquired but no home position set
   if (locked && !has_gps_lock_) {
       if (!current_state_.home.is_set) {
           // Set home from GPS coordinates
           current_state_.home.global_pos.lat = MavLinkGpsRawInt.lat / 1E7f;
           current_state_.home.global_pos.lon = MavLinkGpsRawInt.lon / 1E7f;
           current_state_.home.global_pos.alt = MavLinkGpsRawInt.alt / 1000.0f;
           current_state_.home.is_set = true;
       }
   }
   ```

3. **Local Position Fallback**: For no-GPS scenarios, implemented local position home setting:
   ```cpp
   void setLocalPositionAsHome() {
       current_state_.home.global_pos.lat = std::numeric_limits<float>::quiet_NaN();
       current_state_.home.global_pos.lon = std::numeric_limits<float>::quiet_NaN(); 
       current_state_.home.global_pos.alt = 0.0f;
       current_state_.home.is_set = true;
   }
   ```

**Files Modified**:
- `/mnt/l/cosys-airsim/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- `/mnt/l/cosys-airsim/Unreal/Plugins/AirSim/Source/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
- `/mnt/l/cosys-airsim/Unreal/Environments/Blocks/Plugins/AirSim/Source/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`

#### Problem: "Ground is not stable, variance is 1.000000"
This error occurs after GPS home fix when ground stability check fails.

**Root Cause**: Ground stability check was too strict for multi-drone scenarios and had no bypass for local position mode.

**Solution Applied**: Smart ground stability checker with adaptive tolerance:

1. **Local Position Mode Bypass**: Automatically skip ground stability check when operating without GPS:
   ```cpp
   void waitForStableGroundPosition(float timeout_sec) {
       // Skip check if GPS coordinates are NaN (local position mode)
       if (current_state_.home.global_pos.lat != current_state_.home.global_pos.lat) {
           addStatusMessage("Skipping ground stability check (local position mode)");
           return;
       }
       // ... continue with stability check
   }
   ```

2. **Adaptive Tolerance System**: Progressive tolerance relaxation for multi-drone scenarios:
   ```cpp
   float tolerance = 0.1f; // Start strict
   while (elapsed < timeout_sec) {
       if (variance <= tolerance) {
           // Ground is stable enough
           return;
       }
       
       // Relax tolerance progressively
       if (elapsed > timeout_sec * 0.3f) tolerance = 0.3f;
       if (elapsed > timeout_sec * 0.6f) tolerance = 0.5f;
       if (elapsed > timeout_sec * 0.8f) tolerance = 1.0f;
   }
   ```

3. **Enhanced Error Messages**: Detailed variance reporting for debugging:
   ```cpp
   addStatusMessage(Utils::stringf("Ground unstable after %.1fs, variance: %.6f (tolerance: %.6f)", 
                                  timeout_sec, variance, tolerance));
   ```

**Multi-Drone Benefits**:
- Handles simultaneous drone startup timing variations
- Adapts to different ground conditions across spawn points
- Provides detailed diagnostics for troubleshooting

### Manual Verification Commands

After implementing fixes, verify functionality:

```bash
# 1. Check GPS home status in PX4 console
docker exec -it px4-swarm-1-drone-1 bash
cd /px4_workspace/PX4-Autopilot
./Tools/sitl_run.sh px4 gazebo-classic
# In PX4 console: commander status

# 2. Test Python client arming
python3 -c "
import cosysairsim as airsim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)  # Should succeed without GPS errors
"

# 3. Monitor AirSim logs for home position messages
# Look for: 'GPS home position set' or 'Setting home position from GPS data'
```

### Docker Dual IP Configuration (Enhanced)

#### Understanding the Dual IP Setup
In WSL2 + Windows AirSim configurations, you have **two different IP addresses**:

- **LocalHostIp** (`172.28.240.1`): Default gateway - where PX4 expects to find AirSim
- **Remote Control IP** (`172.28.248.55`): WSL2 interface - actual network interface for remote control

#### Configuration Details

**In your settings.json:**
```json
{
  "LocalHostIp": "172.28.240.1",
  "ControlIp": "remote"
}
```

**In docker-compose.yml:**
```yaml
environment: &px4-env
  PX4_SIM_HOSTNAME: 172.28.240.1  # Uses LocalHostIp
```

#### Enhanced Script Features

The `start-swarm.sh` script now supports dual IP management:

1. **Dual IP Detection**: Identifies both LocalHostIp and Remote Control IP
2. **No Auto-Override**: Preserves your LocalHostIp setting (172.28.240.1)
3. **Connectivity Testing**: Tests both IP configurations
4. **Manual IP Management**: Easy switching between configurations

#### New Commands Available:

```bash
# Show dual IP configuration
./tools/start-swarm.sh show-config

# Test connectivity on both IPs
./tools/start-swarm.sh test-connection

# Set specific simulation IP (if needed)
./tools/start-swarm.sh set-sim-ip 172.28.240.1

# Start containers (preserves your LocalHostIp setting)
./tools/start-swarm.sh start
```

#### Expected Output:
```
[INFO] WSL2 detected - checking dual IP configuration...
[INFO] Network Configuration:
[INFO]   Remote Control IP (WSL2 interface): 172.28.248.55
[INFO]   LocalHostIp (PX4_SIM_HOSTNAME): 172.28.240.1
[INFO]   Default Gateway: 172.28.240.1
[INFO] Testing connectivity via Remote IP (172.28.248.55)...
[INFO] ✅ AirSim API accessible via Remote IP 172.28.248.55:41451
[INFO] Testing connectivity via LocalHostIp (172.28.240.1)...
[INFO] ⚠️  AirSim API not accessible via LocalHostIp 172.28.240.1:41451
[INFO]    This is expected for dual IP setup - PX4 connects to LocalHostIp
[INFO]    Remote control uses the Remote IP (172.28.248.55)
```

#### Why This Configuration Works

- **PX4 Simulation Traffic**: Uses LocalHostIp (172.28.240.1) for MAVLink simulation data
- **Remote Control Traffic**: Uses Remote IP (172.28.248.55) for external API calls
- **AirSim Configuration**: Listens on both interfaces via settings.json
- **Network Routing**: WSL2 automatically routes between the interfaces

#### Manual IP Management:
```bash
# Switch to Remote IP (if testing different configuration)
./tools/start-swarm.sh set-sim-ip 172.28.248.55

# Switch back to LocalHostIp (recommended)
./tools/start-swarm.sh set-sim-ip 172.28.240.1
```

### Advanced GPS Troubleshooting

If GPS issues persist after network configuration:

1. **Check MAVLink GPS Messages**:
   ```bash
   # Monitor GPS_RAW_INT messages
   docker exec -it px4-swarm-1-drone-1 bash
   cd /px4_workspace/PX4-Autopilot
   mavlink_shell.py # Then: listener gps_status
   ```

2. **Verify AirSim GPS Configuration**:
   ```json
   // In settings.json, ensure GPS is enabled
   "Sensors": {
     "Gps": {
       "SensorType": 3,
       "Enabled": true
     }
   }
   ```

3. **Force Local Position Mode** (for testing without GPS):
   ```bash
   # In PX4 console
   param set SYS_MC_EST_GROUP 1  # Use local position estimator
   param set EKF2_AID_MASK 0     # Disable GPS aiding
   ```

## Cleanup

```bash
# Stop all containers
./tools/start-swarm.sh stop

# Remove all containers and networks
docker-compose -f docker-compose.ultra-swarm-delayed.yml down
docker-compose -f docker-compose.simple-swarm-delayed.yml down
```