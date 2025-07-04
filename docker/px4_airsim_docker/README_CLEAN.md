# PX4 AirSim Docker Setup - Cleaned Version

This is a simplified and consolidated version of the PX4 AirSim Docker setup. All redundant files have been removed and functionality has been consolidated into just a few easy-to-use files.

## Quick Start

### Single Drone Setup
```bash
# Launch single drone
launch_single.bat

# Stop all drones
stop.bat
```

### Multi-Drone Setup (2-5 drones)
```bash
# Launch 2 drones
launch_multi.bat 2

# Launch 3 drones  
launch_multi.bat 3

# Launch up to 5 drones
launch_multi.bat 5

# Stop all drones
stop.bat
```

## Files Overview

### Core Files (Keep These)
- **`docker-compose-clean.yml`** - Single consolidated Docker Compose configuration
- **`Dockerfile-clean`** - Single unified Dockerfile for all configurations
- **`start_px4.sh`** - Unified startup script for containers
- **`launch_single.bat`** - Launch single drone
- **`launch_multi.bat`** - Launch multiple drones (2-5)
- **`stop.bat`** - Stop all containers
- **`README_CLEAN.md`** - This documentation

### Files to Remove (Old/Redundant)
- `docker-compose.yml` (old version)
- `docker-compose.simple.yml`
- `docker-compose.host.yml`
- `docker-compose.airsim.yml`
- `Dockerfile.px4-instance`
- `Dockerfile.simple`
- `Dockerfile` (old version)
- `launch_px4_simple.bat`
- `launch_px4_swarm.bat`
- `launch_4_drone_swarm.bat`
- `stop_px4_swarm.bat`
- All shell script variants (.sh files)
- Various utility bat files

## Port Configuration

### Single Drone
- **TCP (AirSim):** 4561
- **UDP (MAVLink):** 14541/14581
- **micro-ROS:** 8889

### Multi-Drone (1-based numbering)
- **Drone 1:** TCP 4561, UDP 14541/14581, micro-ROS 8889
- **Drone 2:** TCP 4562, UDP 14542/14582, micro-ROS 8890
- **Drone 3:** TCP 4563, UDP 14543/14583, micro-ROS 8891
- **Drone 4:** TCP 4564, UDP 14544/14584, micro-ROS 8892
- **Drone 5:** TCP 4565, UDP 14545/14585, micro-ROS 8893

## AirSim Configuration

The launcher scripts automatically display the correct AirSim `settings.json` configuration for your setup. Simply copy and paste the displayed JSON into your AirSim settings file.

### Example Single Drone settings.json:
```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "LockStep": true,
      "X": 0, "Y": 0, "Z": -2
    }
  }
}
```

## Usage Instructions

1. **Start AirSim/Unreal Engine** first
2. **Launch PX4 containers** using the appropriate script
3. **Connect QGroundControl** to the MAVLink ports
4. **Fly your drones!**

## Troubleshooting

### View Container Logs
```bash
# View logs for all containers
docker-compose -f docker-compose-clean.yml logs -f

# View logs for specific container
docker logs px4-single
docker logs px4-drone-1
```

### Check Container Status
```bash
docker ps
```

### Manual Container Management
```bash
# Stop specific profile
docker-compose -f docker-compose-clean.yml --profile single down
docker-compose -f docker-compose-clean.yml --profile drone-3 down

# Build images manually
docker-compose -f docker-compose-clean.yml build
```

## Network Configuration

All containers use the `airsim-network` (172.25.0.0/16) with static IP addresses:
- Single drone: 172.25.0.10
- Drone 1: 172.25.0.10
- Drone 2: 172.25.0.11
- Drone 3: 172.25.0.12
- Drone 4: 172.25.0.13
- Drone 5: 172.25.0.14

## Migration from Old Setup

1. **Test the new setup** with your existing AirSim configuration
2. **Update your settings.json** using the output from the launcher scripts
3. **Remove old files** once you've confirmed everything works
4. **Update any documentation or scripts** that reference the old files

The new system is much simpler and more maintainable while providing the same functionality as the old complex setup. 