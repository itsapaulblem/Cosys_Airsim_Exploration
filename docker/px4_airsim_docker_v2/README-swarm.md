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

## Cleanup

```bash
# Stop all containers
./tools/start-swarm.sh stop

# Remove all containers and networks
docker-compose -f docker-compose.ultra-swarm-delayed.yml down
docker-compose -f docker-compose.simple-swarm-delayed.yml down
```