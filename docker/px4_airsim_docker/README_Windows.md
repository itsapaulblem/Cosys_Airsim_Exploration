# Running PX4-AirSim Docker Container on Windows

This guide explains how to run the PX4-AirSim Docker container on Windows.

## Prerequisites

### 1. Install Docker Desktop
- Download from [docker.com](https://www.docker.com/products/docker-desktop/)
- Install with WSL2 backend enabled (recommended)
- Start Docker Desktop and ensure it's running

### 2. Install Git (if not already installed)
- Download from [git-scm.com](https://git-scm.com/download/win)

## Quick Start

### Option 1: Using the Batch Script (Easiest)
1. Open Command Prompt or PowerShell as Administrator
2. Navigate to this directory:
   ```cmd
   cd docker\px4_airsim_docker
   ```
3. Run the batch script:
   ```cmd
   run_windows.bat
   ```

### Option 2: Using Docker Compose
1. Open Command Prompt or PowerShell
2. Navigate to this directory:
   ```cmd
   cd docker\px4_airsim_docker
   ```
3. Build and run:
   ```cmd
   docker-compose up --build
   ```

### Option 3: Using Docker Commands Directly
```cmd
# Build the image
docker build -t px4-airsim .

# Run the container
docker run -it --rm ^
  -p 4560:4560 ^
  -p 14550:14550 ^
  -p 8888:8888 ^
  px4-airsim
```

## Port Mapping

The container exposes these ports on your Windows machine:
- **4560**: AirSim connection to PX4
- **14550**: MAVLink for Ground Control Station (QGroundControl)
- **8888**: micro-ROS agent for ROS2 communication

## Connecting to AirSim

1. Start AirSim on your Windows machine
2. In AirSim settings.json, configure PX4 connection:
   ```json
   {
     "SettingsVersion": 1.2,
     "SimMode": "Multirotor",
     "Vehicles": {
       "PX4": {
         "VehicleType": "PX4Multirotor",
         "UseSerial": false,
         "LockStep": true,
         "UseTcp": true,
         "TcpPort": 4560,
         "ControlPortLocal": 14540,
         "ControlPortRemote": 14580
       }
     }
   }
   ```

## Connecting Ground Control Station

1. Install QGroundControl from [qgroundcontrol.com](http://qgroundcontrol.com/)
2. Configure connection:
   - Connection Type: UDP
   - Port: 14550
   - Host: localhost (or your Docker host IP)

## Troubleshooting

### Docker Not Starting
- Ensure Docker Desktop is running
- Check Windows features: Hyper-V and WSL2 should be enabled
- Restart Docker Desktop if needed

### Port Conflicts
If you get port binding errors:
```cmd
# Check what's using the ports
netstat -an | findstr "4560\|14550\|8888"

# Stop conflicting services or change ports in docker-compose.yml
```

### Container Exits Immediately
Check logs:
```cmd
docker-compose logs px4-airsim
```

### Network Issues
- Ensure Windows Firewall allows Docker
- Check Docker Desktop network settings
- Try restarting Docker Desktop

## Advanced Usage

### Accessing Container Shell
```cmd
# While container is running, open another terminal:
docker exec -it px4-airsim-container bash
```

### Viewing Logs
```cmd
# Real-time logs
docker-compose logs -f px4-airsim

# Specific service logs
docker logs px4-airsim-container
```

### Stopping the Container
```cmd
# Graceful stop
docker-compose down

# Force stop
docker kill px4-airsim-container
```

## Performance Tips

1. **Allocate more resources** to Docker Desktop:
   - Settings → Resources → Advanced
   - Increase CPU and Memory limits

2. **Use WSL2 backend** for better performance

3. **Keep container data** in Docker volumes (not Windows filesystem)

## Integration with AirSim Development

This container provides PX4 SITL that can connect to your AirSim instance running on Windows. The typical workflow:

1. Start this Docker container (provides PX4 SITL)
2. Start AirSim on Windows (provides simulation environment)
3. Connect via TCP port 4560
4. Use QGroundControl or ROS2 for mission control

## Support

If you encounter issues:
1. Check Docker Desktop is running and healthy
2. Verify port availability
3. Check container logs for error messages
4. Ensure AirSim settings match the container configuration 