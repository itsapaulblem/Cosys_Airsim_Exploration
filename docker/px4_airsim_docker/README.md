# PX4 AirSim Docker Setup

This folder contains the Docker setup for running PX4 SITL with AirSim integration, supporting both single-drone and multi-swarm configurations.

## File Overview

### 🚁 **Multi-Swarm Setup (Recommended)**
- `Dockerfile.multi-swarm.fixed` - Docker image for multi-swarm PX4 with proper port allocation
- `docker-compose.multi-swarm-fixed.yml` - Compose file for running multiple drone swarms
- `start-swarms.sh` - Helper script for easy swarm management
- `README-MultiSwarm.md` - Detailed multi-swarm documentation

### 🛩️ **Single Drone Setup (Legacy)**
- `Dockerfile` - Original single-drone Docker image
- `docker-compose.yml` - Single-drone compose configuration
- `run_airsim_sitl_final.sh` - Single-drone startup script

### 🔧 **Utility Scripts**
- `px4_console.sh` - Access PX4 MAVLink console
- `px4_shell.sh` - Access PX4 system shell
- `sitl_kill.sh` - Stop PX4 processes

### 📁 **Directories**
- `settings/` - Configuration generators and AirSim settings templates
- `logs/` - Container log files (mounted volume)
- `shared_data/` - Shared data between containers (mounted volume)

## Quick Start

### Multi-Swarm (Recommended)
```bash
# Start 3 drones with unique ports (4561, 4562, 4563)
./start-swarms.sh swarm1-3

# Check status
./start-swarms.sh status

# Stop all
./start-swarms.sh stop
```

### Single Drone (Legacy)
```bash
# Start single drone on port 4560
docker-compose up -d

# Stop
docker-compose down
```

## Port Allocation

### Multi-Swarm Ports
- **Swarm 1**: AirSim TCP 4561-4570, QGC UDP 14550-14559
- **Swarm 2**: AirSim TCP 4571-4580, QGC UDP 14560-14569

### Single Drone Port
- **AirSim TCP**: 4560, **QGC UDP**: 14550

## Documentation

- **Multi-swarm setup**: See `README-MultiSwarm.md`
- **Settings generation**: See `settings/WORKFLOW_GUIDE.md`
- **Configuration validation**: See `settings/STANDARDIZED_WORKFLOW.md`

## Migration Note

If you were using the old multi-drone setup, migrate to the new multi-swarm setup:

```bash
# Old (don't use)
docker-compose -f docker-compose-scaled.yml up --scale px4-drone=3

# New (recommended) 
./start-swarms.sh swarm1-3
```

The new setup provides proper port allocation and eliminates connection conflicts.