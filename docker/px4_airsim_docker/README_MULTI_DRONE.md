# Multi-Drone PX4 Docker Compose Setup

This directory contains a Docker Compose configuration that allows you to easily launch multiple PX4 SITL instances for multi-drone simulation with AirSim.

## 🚁 Overview

Instead of manually starting multiple PX4 instances, this setup uses Docker Compose to:
- Launch multiple containerized PX4 instances
- Each instance runs your exact command: `PX4_SYS_AUTOSTART=10016 ./build/px4_sitl_default/bin/px4 -i <instance_num>`
- Automatically manages port mappings to avoid conflicts
- Provides easy scaling from 1 to 5 drones (extensible)

## 📁 Files

- `docker-compose.yml` - Main Docker Compose configuration with profiles for 1-5 drones
- `Dockerfile.px4-instance` - Specialized Dockerfile for individual PX4 instances
- `launch_px4_swarm.bat` / `launch_px4_swarm.sh` - Easy launcher scripts
- `stop_px4_swarm.bat` / `stop_px4_swarm.sh` - Stop all instances

## 🚀 Quick Start

### Windows
```bash
# Launch 3 drones
./launch_px4_swarm.bat 3

# Stop all drones
./stop_px4_swarm.bat
```

### Linux/MacOS
```bash
# Make scripts executable (first time only)
chmod +x launch_px4_swarm.sh stop_px4_swarm.sh

# Launch 3 drones
./launch_px4_swarm.sh 3

# Stop all drones  
./stop_px4_swarm.sh
```

## 🔧 Port Mappings

Each drone instance gets unique ports:

| Drone | TCP Port | UDP Local | UDP Remote | micro-ROS |
|-------|----------|-----------|------------|-----------|
| 0 | 4560 | 14540 | 14580 | 8888 |
| 1 | 4561 | 14541 | 14581 | 8889 |
| 2 | 4562 | 14542 | 14582 | 8890 |
| 3 | 4563 | 14543 | 14583 | 8891 |
| 4 | 4564 | 14544 | 14584 | 8892 |

## ⚙️ AirSim Configuration

The launcher scripts automatically generate the required `settings.json` configuration. For 3 drones, use:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4_Drone0": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LockStep": true,
      "X": 0, "Y": 0, "Z": -2
    },
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "LockStep": true,
      "X": 1, "Y": 0, "Z": -2
    },
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4562,
      "ControlPortLocal": 14542,
      "ControlPortRemote": 14582,
      "LockStep": true,
      "X": 2, "Y": 0, "Z": -2
    }
  }
}
```

## 🔨 Manual Docker Compose Usage

If you prefer manual control:

```bash
# Build all images
docker-compose build

# Launch 1 drone
docker-compose --profile drone-1 up -d

# Launch 3 drones
docker-compose --profile drone-3 up -d

# View logs
docker-compose logs -f

# Stop specific profile
docker-compose --profile drone-3 down

# Stop all
docker-compose --profile drone-1 --profile drone-2 --profile drone-3 --profile drone-4 --profile drone-5 down
```

## 🏗️ How It Works

### Docker Compose Profiles
The setup uses Docker Compose profiles to control which containers start:
- `drone-1`: Starts 1 drone (instance 0)
- `drone-2`: Starts 2 drones (instances 0-1)
- `drone-3`: Starts 3 drones (instances 0-2)
- `drone-4`: Starts 4 drones (instances 0-3)
- `drone-5`: Starts 5 drones (instances 0-4)

### Container Architecture
Each container:
1. **Builds PX4-Autopilot** from source (v1.14.3)
2. **Runs your exact command**: `PX4_SYS_AUTOSTART=10016 ./build/px4_sitl_default/bin/px4 -i <instance>`
3. **Exposes standard ports** with unique external mappings
4. **Isolates instances** in separate containers

### Instance Isolation
- Each container has its own PX4 instance directory
- Separate log files and data directories
- No port conflicts between instances
- Independent startup/shutdown

## 🐛 Debugging

### View Container Logs
```bash
# All containers
docker-compose logs -f

# Specific drone
docker logs -f px4-drone-0
docker logs -f px4-drone-1
```

### Check Container Status
```bash
# List running containers
docker ps

# Check specific container
docker exec -it px4-drone-0 bash
```

### Verify PX4 Console Output
Each container should show:
```
______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.
INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [simulator] Waiting for simulator to connect on TCP port 4560
```

### Common Issues

1. **Port conflicts**: Ensure no other PX4 instances are running
2. **Docker not running**: Start Docker Desktop
3. **Permission issues**: Run scripts as administrator (Windows) or with sudo (Linux)
4. **Build failures**: Check Docker has sufficient memory (4GB+ recommended)

## 🔍 Advanced Configuration

### Adding More Drones
To support more than 5 drones, edit `docker-compose.yml`:

1. **Add new service block**:
```yaml
px4-instance-5:
  <<: *px4-base
  container_name: px4-drone-5
  environment:
    - PX4_INSTANCE=5
  ports:
    - "4565:4560/tcp"
    - "14545:14540/udp"
    - "14585:14580/udp"
    - "8893:8888/udp"
  profiles:
    - drone-6
```

2. **Update launcher scripts** to support `drone-6` profile

### Custom PX4 Settings
Modify environment variables in `docker-compose.yml`:
```yaml
environment:
  - PX4_HOME_LAT=47.641468      # Home latitude
  - PX4_HOME_LON=-122.140165    # Home longitude
  - PX4_HOME_ALT=0.0            # Home altitude
  - PX4_SYS_AUTOSTART=10016     # Vehicle type (10016 = Generic Quadrotor)
```

### Volume Persistence
To persist PX4 data between container restarts:
```yaml
volumes:
  - px4-logs-${PX4_INSTANCE}:/px4_workspace/PX4-Autopilot/build/px4_sitl_default/instance_${PX4_INSTANCE}
```

## 🚀 Integration with ROS2

This setup works seamlessly with the AirSim ROS2 wrapper:

1. **Start PX4 instances**: `./launch_px4_swarm.sh 3`
2. **Start AirSim**: Launch Unreal with multi-drone settings.json
3. **Start ROS2 wrapper**: `docker run -it --network host airsim-ros2:optimized`
4. **Control drones**: Use ROS2 services/topics for each drone namespace

## 📊 Performance Considerations

- **Memory**: Each PX4 instance uses ~100-200MB RAM
- **CPU**: Scales linearly with number of drones
- **Network**: Ensure firewall allows all configured ports
- **Build time**: First build takes 5-10 minutes, subsequent builds are cached

## 🔗 Related Documentation

- [AirSim Multi-Vehicle Setup](../../docs/px4_multi_vehicle.md)
- [PX4 SITL Documentation](../../docs/px4_sitl.md)
- [AirSim Settings Reference](../../docs/settings.md) 