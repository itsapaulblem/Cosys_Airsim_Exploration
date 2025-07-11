# Multi-Swarm PX4 Docker Setup for AirSim

This setup provides a scalable multi-swarm PX4 SITL environment for AirSim integration with proper port allocation per drone.

## Fixed Port Allocation ✅

Each drone now gets its own unique AirSim TCP port:

### Swarm 1 (Blue Team)
- **Drone 1**: AirSim TCP `4561`, QGC UDP `14550`, MAVLink UDP `18570`
- **Drone 2**: AirSim TCP `4562`, QGC UDP `14551`, MAVLink UDP `18571`  
- **Drone 3**: AirSim TCP `4563`, QGC UDP `14552`, MAVLink UDP `18572`

### Swarm 2 (Red Team) 
- **Drone 1**: AirSim TCP `4571`, QGC UDP `14560`, MAVLink UDP `18580`
- **Drone 2**: AirSim TCP `4572`, QGC UDP `14561`, MAVLink UDP `18581`

## Quick Start

### Using the Helper Script (Recommended)

```bash
# Make script executable (if needed)
chmod +x start-swarms.sh

# Start single drone
./start-swarms.sh single-drone

# Start 3 drones in swarm 1
./start-swarms.sh swarm1-3

# Start both swarms (3 + 2 drones)
./start-swarms.sh both-swarms

# Check status
./start-swarms.sh status

# View logs
./start-swarms.sh logs px4-swarm-1-drone-1

# Stop all
./start-swarms.sh stop
```

### Using Docker Compose Directly

```bash
# Start 3 drones in swarm 1
docker-compose -f docker-compose.multi-swarm-fixed.yml up \
  px4-swarm-1-drone-1 px4-swarm-1-drone-2 px4-swarm-1-drone-3 -d

# Start swarm 2 
docker-compose -f docker-compose.multi-swarm-fixed.yml --profile swarm2 up \
  px4-swarm-2-drone-1 px4-swarm-2-drone-2 -d

# Check status
docker-compose -f docker-compose.multi-swarm-fixed.yml ps

# Stop all
docker-compose -f docker-compose.multi-swarm-fixed.yml down
```

## AirSim Configuration

In your AirSim `settings.json`, configure each drone with its assigned port:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14550,
      "ControlPortRemote": 14550
    },
    "Drone2": {
      "VehicleType": "SimpleFlight", 
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4562,
      "ControlPortLocal": 14551,
      "ControlPortRemote": 14551
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "UseSerial": false, 
      "UseTcp": true,
      "TcpPort": 4563,
      "ControlPortLocal": 14552,
      "ControlPortRemote": 14552
    }
  }
}
```

## Architecture

### Files Overview
- `Dockerfile.multi-swarm.fixed` - Multi-swarm Docker image with proper port calculation
- `docker-compose.multi-swarm-fixed.yml` - Compose file with explicit drone definitions
- `start-swarms.sh` - Helper script for easy swarm management

### Port Calculation Logic
```bash
# AirSim TCP: swarm-1 uses 4561+, swarm-2 uses 4571+, etc.
airsim_tcp_port=$((4560 + (swarm_id - 1) * 10 + instance_num))

# QGC UDP: swarm-1 uses 14550+, swarm-2 uses 14560+, etc.  
qgc_port=$((14550 + (swarm_id - 1) * 10 + instance_num - 1))

# MAVLink UDP: swarm-1 uses 18570+, swarm-2 uses 18580+, etc.
mavlink_udp_port=$((18570 + (swarm_id - 1) * 10 + instance_num - 1))
```

## Health Monitoring

All containers include health checks and can be monitored:

```bash
# Check health status
docker-compose -f docker-compose.multi-swarm-fixed.yml ps

# Manual health check
docker exec px4-swarm-1-drone-1 /Scripts/swarm_health_check.sh
```

## Troubleshooting

### Common Issues

1. **Port conflicts**: Each drone now uses unique ports, conflicts should be resolved

2. **Container not starting**: Check logs with:
   ```bash
   ./start-swarms.sh logs px4-swarm-1-drone-1
   ```

3. **AirSim connection issues**: Verify the correct port in AirSim settings.json

4. **PX4 binary not found**: The image uses your existing `px4_airsim_docker-px4-single:latest` as base

### Debug Container

Start a debug container for troubleshooting:

```bash
docker-compose -f docker-compose.multi-swarm-fixed.yml --profile debug up px4-debug -d
docker exec -it px4-debug-swarm bash
```

## Performance Notes

- **Resource limits**: Each container is limited to 1 CPU and 1GB RAM
- **Scaling**: Can support up to 10 drones per swarm (ports 4561-4570, 4571-4580, etc.)
- **Network**: All containers use bridge network `airsim-swarm-network` (172.26.0.0/16)

## Integration with Cosys-AirSim

This setup is designed to work seamlessly with Cosys-AirSim's enhanced features:
- Multiple sensor types (GPU LiDAR, Echo, MarLocUwb) 
- Instance segmentation and annotation layers
- Ground truth data generation
- Deterministic object spawning
- Camera model enhancements

Connect each AirSim drone to its corresponding PX4 container using the port mapping above.