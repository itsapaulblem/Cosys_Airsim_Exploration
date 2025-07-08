# 🚁 Ultra-Swarm PX4 + AirSim Configuration

**Enhanced Docker Setup Supporting Up to 27 Drones (9 per swarm × 3 swarms)**

This directory contains the ultra-swarm enhancement of the PX4-AirSim Docker integration, designed to support massive multi-drone simulations with proper GPS configuration and port management.

## 🌟 Key Features

- **Massive Scale**: Support for up to 27 drones across 3 swarms
- **GPS Home Location Fixes**: Resolves "Vehicle does not have a valid GPS home location" errors
- **Intelligent Port Allocation**: Automatic port assignment preventing conflicts
- **Incremental Testing**: Start with 1 drone and scale up to full swarms
- **Profile-Based Activation**: Selective swarm startup using Docker profiles
- **Dynamic Generation**: Python script for custom configurations
- **Comprehensive Management**: Helper scripts for easy operation

## 📁 Directory Structure

```
px4_airsim_docker_v2/
├── Dockerfile.ultra-swarm           # Enhanced PX4 image with GPS fixes
├── docker-compose.ultra-swarm.yml   # 27-drone configuration
├── start-ultra-swarms.sh            # Comprehensive management script
├── generate_compose.py              # Dynamic configuration generator
├── config/
│   ├── px4_params.txt              # PX4 parameters for GPS configuration
│   └── gps_locations.json          # GPS coordinates for all drones
└── logs/                           # Container logs directory
```

## 🔧 Quick Start

### 1. Build the Ultra-Swarm Image

```bash
cd /mnt/l/cosys-airsim/docker/px4_airsim_docker_v2
docker build -f Dockerfile.ultra-swarm -t px4-ultra-swarm:latest .
```

### 2. Start with a Single Drone (Testing)

```bash
./start-ultra-swarms.sh single
```

### 3. Scale Up to 3 Drones (Basic Swarm)

```bash
./start-ultra-swarms.sh test-3
```

### 4. Launch Full Swarm (9 drones)

```bash
./start-ultra-swarms.sh swarm1-full
```

### 5. Ultimate Configuration (All 27 drones)

```bash
./start-ultra-swarms.sh ultra-max
```

## 🔗 Port Allocation

The system uses intelligent port allocation to prevent conflicts:

| Swarm | Team | AirSim TCP | QGC UDP | MAVLink UDP | GPS Location |
|-------|------|------------|---------|-------------|--------------|
| 1 | 🔵 Blue | 4561-4569 | 14550-14558 | 18570-18578 | Seattle area |
| 2 | 🔴 Red | 4571-4579 | 14560-14568 | 18580-18588 | Bellevue area |
| 3 | 🟢 Green | 4581-4589 | 14570-14578 | 18590-18598 | Redmond area |

## 📋 Management Commands

### Basic Operations

```bash
# Start configurations
./start-ultra-swarms.sh single          # 1 drone testing
./start-ultra-swarms.sh test-3           # 3 drone testing
./start-ultra-swarms.sh swarm1-full      # Full Blue Team (9 drones)
./start-ultra-swarms.sh dual-swarms      # Blue + Red Teams (18 drones)
./start-ultra-swarms.sh ultra-max        # All 27 drones

# Monitoring
./start-ultra-swarms.sh status           # Container status
./start-ultra-swarms.sh health           # Health checks
./start-ultra-swarms.sh ports            # Active port allocations
./start-ultra-swarms.sh gps-info         # GPS configuration

# Logs and debugging
./start-ultra-swarms.sh logs px4-swarm-1-drone-1  # View specific drone logs

# Shutdown
./start-ultra-swarms.sh stop             # Stop all containers
./start-ultra-swarms.sh clean            # Remove all containers/networks
```

### Selective Swarm Control

```bash
# Individual swarm operations
./start-ultra-swarms.sh stop-swarm1      # Stop only Blue Team
./start-ultra-swarms.sh stop-swarm2      # Stop only Red Team  
./start-ultra-swarms.sh stop-swarm3      # Stop only Green Team
```

## 🐍 Dynamic Configuration Generator

Use the Python generator for custom configurations:

```bash
# Generate custom configurations
python3 generate_compose.py --swarms 2 --drones 5  # 2 swarms, 5 drones each
python3 generate_compose.py --preset ultra-max     # Full 27-drone setup
python3 generate_compose.py --preset test --dry-run # Preview configuration

# Available presets
--preset single       # 1 drone
--preset test         # 3 drones (1 swarm)
--preset full-swarm   # 9 drones (1 swarm)
--preset dual-swarms  # 18 drones (2 swarms)
--preset ultra-max    # 27 drones (3 swarms)
```

## 📍 GPS Configuration

Each swarm is positioned in realistic locations around Seattle:

- **Swarm 1 (Blue)**: Seattle area (47.641468, -122.140165)
- **Swarm 2 (Red)**: Bellevue area (47.642468, -122.139165)  
- **Swarm 3 (Green)**: Redmond area (47.643468, -122.138165)

Drones within each swarm are arranged in a 3×3 grid with ~11m spacing.

### GPS Parameters Configured

The system automatically configures these PX4 parameters to prevent GPS errors:

```
COM_ARM_WO_GPS=1      # Allow arming without GPS fix
EKF2_GPS_CHECK=21     # Reduced GPS validation (from default 245)
SYS_HAS_GPS=1         # Enable GPS hardware
LPE_LAT/LPE_LON       # Set home position per swarm
```

## 🔧 Technical Architecture

### Container Design

- **Base Image**: Built from existing `px4_airsim_docker-px4-single:latest`
- **Enhanced Scripts**: GPS-aware startup scripts with parameter injection
- **Health Monitoring**: Built-in health checks for all containers
- **Resource Limits**: 0.8 CPU, 512M memory per drone for stability

### Network Configuration

- **Custom Bridge Network**: `ultra-swarm-network` (172.27.0.0/16)
- **Service Discovery**: Containers can communicate by name
- **Port Exposure**: Only necessary ports exposed to host
- **Profile Isolation**: Swarms 2 & 3 use Docker profiles for selective startup

### Volume Management

- **Persistent Data**: `/px4_data` volume for drone state
- **Log Aggregation**: Centralized logging in `./logs/` directory
- **Configuration Sharing**: GPS and parameter files shared across containers

## 🚨 Troubleshooting

### Common Issues

**1. "Vehicle does not have a valid GPS home location"**
- ✅ **Fixed in ultra-swarm**: GPS parameters automatically configured
- Check: `./start-ultra-swarms.sh gps-info`

**2. Port conflicts between drones**
- ✅ **Fixed in ultra-swarm**: Intelligent port allocation
- Check: `./start-ultra-swarms.sh ports`

**3. Container startup failures**
- Run health checks: `./start-ultra-swarms.sh health`
- Check logs: `./start-ultra-swarms.sh logs [container-name]`
- Verify resources: `docker system df`

**4. High resource usage**
- Start incrementally: `single` → `test-3` → `swarm1-full`
- Monitor: `docker stats`
- Clean unused: `docker system prune`

### Debug Container

Access a debug container for troubleshooting:

```bash
# Start debug container
docker-compose -f docker-compose.ultra-swarm.yml --profile debug up px4-debug -d

# Access debug shell
docker exec -it px4-debug-ultra-swarm bash

# Check PX4 configuration
ultra-status
px4-params
```

## 📊 Performance Recommendations

### System Requirements

| Configuration | RAM | CPU | Recommended Use |
|---------------|-----|-----|-----------------|
| Single drone | 1GB | 2 cores | Development/testing |
| 3 drones | 2GB | 4 cores | Algorithm development |
| 9 drones (1 swarm) | 6GB | 8 cores | Swarm behavior testing |
| 18 drones (2 swarms) | 12GB | 12 cores | Multi-swarm scenarios |
| 27 drones (3 swarms) | 18GB | 16+ cores | Research/competition |

### Performance Tuning

```bash
# Increase Docker memory limit
# Edit Docker Desktop settings or /etc/docker/daemon.json

# Monitor resource usage
docker stats $(docker ps --format "{{.Names}}" | grep px4-swarm)

# Optimize for specific use cases
python3 generate_compose.py --swarms 1 --drones 5  # Custom scaling
```

## 🔄 Integration with AirSim

### AirSim Settings Configuration

Configure your AirSim `settings.json` to connect to the ultra-swarm:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "AutoCreate": true,
      "PawnBP": "DefaultQuadrotor",
      "ApiServerPort": 4561
    },
    "Drone2": {
      "VehicleType": "SimpleFlight", 
      "DefaultVehicleState": "Armed",
      "AutoCreate": true,
      "PawnBP": "DefaultQuadrotor",
      "ApiServerPort": 4562
    }
  }
}
```

### Python Client Example

```python
import cosysairsim as airsim

# Connect to multiple drones
clients = []
for port in range(4561, 4564):  # Drones 1-3
    client = airsim.MultirotorClient(port=port)
    client.confirmConnection()
    clients.append(client)

# Coordinate swarm operations
for i, client in enumerate(clients):
    client.armDisarm(True)
    client.takeoffAsync()
```

## 🆘 Support

### Getting Help

1. **Check logs**: `./start-ultra-swarms.sh logs [drone-name]`
2. **Health status**: `./start-ultra-swarms.sh health`
3. **System resources**: `docker system df && docker stats`
4. **GPS configuration**: `./start-ultra-swarms.sh gps-info`

### Common Commands Reference

```bash
# Quick status check
./start-ultra-swarms.sh status

# Full diagnostic
./start-ultra-swarms.sh health && ./start-ultra-swarms.sh ports

# Emergency stop
./start-ultra-swarms.sh stop

# Complete cleanup
./start-ultra-swarms.sh clean
```

---

## 📝 Version History

- **v2.0**: Ultra-swarm enhancement with 27-drone support
- **v1.0**: Original multi-swarm setup (3 drones per swarm)

## 🤝 Contributing

When modifying configurations:

1. Test incrementally: `single` → `test-3` → `swarm1-full`
2. Update GPS locations in `config/gps_locations.json`
3. Regenerate compose files: `python3 generate_compose.py`
4. Update documentation

---

**🚁 Ready to launch your ultra-swarm? Start with `./start-ultra-swarms.sh single` and scale up!**