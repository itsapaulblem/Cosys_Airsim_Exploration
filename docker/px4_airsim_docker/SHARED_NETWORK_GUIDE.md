# AirSim Shared Network Configuration Guide

This guide explains how to set up and use the shared network that connects PX4 containers, ROS2 wrapper containers, and your Unreal Engine AirSim host.

## 🌐 Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                AirSim Shared Network                        │
│                172.25.0.0/16                               │
│                                                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │   PX4 Drones    │  │  ROS2 Wrapper   │  │  Windows    │ │
│  │                 │  │                 │  │  Host       │ │
│  │ 172.25.0.10-14  │  │ 172.25.0.20-21  │  │ (Unreal)    │ │
│  │                 │  │                 │  │             │ │
│  │ px4-drone-0     │  │ airsim-ros2     │  │ AirSim      │ │
│  │ px4-drone-1     │  │ airsim-ros2-vnc │  │ Simulation  │ │
│  │ px4-drone-2     │  │                 │  │             │ │
│  │ px4-drone-3     │  │                 │  │             │ │
│  │ px4-drone-4     │  │                 │  │             │ │
│  └─────────────────┘  └─────────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 📋 IP Address Assignments

### PX4 Containers
- `px4-drone-0`: `172.25.0.10` (TCP 4560, UDP 14540/14580)
- `px4-drone-1`: `172.25.0.11` (TCP 4561, UDP 14541/14581)
- `px4-drone-2`: `172.25.0.12` (TCP 4562, UDP 14542/14582)
- `px4-drone-3`: `172.25.0.13` (TCP 4563, UDP 14543/14583)
- `px4-drone-4`: `172.25.0.14` (TCP 4564, UDP 14544/14584)

### ROS2 Containers
- `airsim-ros2-wrapper`: `172.25.0.20` (UDP 7400-7402)
- `airsim-ros2-vnc`: `172.25.0.21` (VNC 5901/6901, UDP 7400-7402)

### Windows Host
- Access containers via: `host.docker.internal` or direct IP addresses
- Unreal AirSim binds to: `0.0.0.0:41451` (accessible from containers)

## 🚀 Usage Examples

### 1. Launch PX4 Only (2 drones)
```bash
./launch_full_stack.bat px4-only 2
```

### 2. Launch ROS2 Wrapper Only
```bash
./launch_full_stack.bat ros2-only
```

### 3. Launch ROS2 with VNC GUI
```bash
./launch_full_stack.bat ros2-vnc
```

### 4. Launch Full Stack (3 PX4 + ROS2)
```bash
./launch_full_stack.bat full-stack 3
```

### 5. Launch Full Stack with VNC
```bash
./launch_full_stack.bat full-vnc 1
```

## ⚙️ AirSim Configuration

### AirSim settings.json
Update your AirSim `settings.json` to bind to all interfaces:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ApiServerEndpoint": "0.0.0.0:41451",
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
    }
  }
}
```

### Multi-Drone Configuration
For multiple drones, the launch script will generate the complete configuration automatically.

## 🔧 Network Management

### Inspect Network
```bash
docker network inspect airsim-shared-network
```

### View Container IPs
```bash
docker ps --format "table {{.Names}}\t{{.Ports}}"
```

### Connect to Container Shell
```bash
docker exec -it px4-drone-0 /bin/bash
docker exec -it airsim-ros2-wrapper /bin/bash
```

### Test Connectivity
From inside a container:
```bash
# Test AirSim connection
nc -zv host.docker.internal 41451

# Test other container
ping 172.25.0.10  # PX4 drone 0
ping 172.25.0.20  # ROS2 wrapper
```

## 🐛 Troubleshooting

### Container Communication Issues
1. Check if containers are on the same network:
   ```bash
   docker network inspect airsim-shared-network
   ```

2. Test connectivity between containers:
   ```bash
   docker exec px4-drone-0 ping 172.25.0.20
   ```

### AirSim Connection Issues
1. Verify AirSim is binding to all interfaces:
   - Check `settings.json` has `"ApiServerEndpoint": "0.0.0.0:41451"`

2. Test from container:
   ```bash
   docker exec airsim-ros2-wrapper nc -zv host.docker.internal 41451
   ```

3. Check Windows firewall allows port 41451

### ROS2 Communication Issues
1. Verify ROS_DOMAIN_ID matches across containers:
   ```bash
   docker exec airsim-ros2-wrapper env | grep ROS_DOMAIN_ID
   ```

2. Check DDS discovery ports (7400-7402) are open

### VNC Access Issues
1. Wait 10 seconds after container start
2. Access via: http://localhost:6901/vnc.html
3. Password: `airsim123`

## 📊 Monitoring

### Real-time Logs
```bash
# All services
docker-compose logs -f

# Specific service
docker logs -f px4-drone-0
docker logs -f airsim-ros2-wrapper
```

### Resource Usage
```bash
docker stats
```

### Network Traffic
```bash
# Install tcpdump in container
docker exec -it airsim-ros2-wrapper apt-get update && apt-get install -y tcpdump

# Monitor traffic
docker exec -it airsim-ros2-wrapper tcpdump -i eth0
```

## 🔄 Advanced Configuration

### Custom Network Settings
Modify `docker-compose.yml` to change network configuration:

```yaml
networks:
  airsim-network:
    name: airsim-shared-network
    driver: bridge
    ipam:
      config:
        - subnet: 172.25.0.0/16  # Change subnet if needed
          gateway: 172.25.0.1
```

### External Network Access
To allow external machines to connect:

1. Create an external network:
   ```bash
   docker network create --driver bridge --subnet=172.25.0.0/16 airsim-external
   ```

2. Update `docker-compose.yml`:
   ```yaml
   networks:
     airsim-network:
       external: true
       name: airsim-external
   ```

## 🏁 Complete Workflow

1. **Start Services:**
   ```bash
   ./launch_full_stack.bat full-vnc 2
   ```

2. **Configure AirSim:**
   - Copy generated `settings.json` configuration
   - Start Unreal with AirSim

3. **Access ROS2:**
   - Open http://localhost:6901/vnc.html
   - Password: `airsim123`
   - Launch RViz2 to visualize data

4. **Monitor:**
   - Use `docker logs -f` to monitor services
   - Check network connectivity with `ping` commands

5. **Stop Services:**
   ```bash
   docker-compose --profile full-vnc down
   ```

This shared network setup provides seamless communication between all AirSim components while maintaining isolation and easy management. 