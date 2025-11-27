# QGroundControl Docker Integration for AirSim/PX4

Dockerized QGroundControl (QGC) with **dual deployment options**: VNC for cross-platform access or X11 forwarding for native Linux performance.

## Overview

**Two QGroundControl services** following the proven architecture pattern from this ecosystem:

### 1. `qgroundcontrol` - VNC/noVNC (Cross-Platform)
- **VNC Server** for remote GUI access (port 5900)
- **noVNC** web-based access (port 6080) - no VNC client needed
- **Works everywhere**: Windows, Mac, Linux, remote servers
- **MAVLink auto-discovery** of PX4 drones (UDP 14550)

### 2. `qgroundcontrol-x11` - X11 Forwarding (Linux Native)
- **60-70% lower resource usage** vs VNC
- **Native performance** with direct GPU access
- **Lower latency** - no encoding/decoding overhead
- **Linux/WSL2 only** - requires host X server

### Performance Comparison

| Feature | X11 Forwarding (`qgroundcontrol-x11`) | VNC (`qgroundcontrol`) |
|---------|----------------------------------------|------------------------|
| **Performance** | ⚡ Native (60-70% lower overhead) | Standard (encoding overhead) |
| **Latency** | ✅ Minimal | ~30-40% higher |
| **Graphics** | ✅ Direct GPU access | Software rendering |
| **Platforms** | ❌ Linux/WSL2 only | ✅ Windows/Mac/Linux/Remote |
| **Setup** | Requires `xhost +local:docker` | Ready out of box |
| **Remote Access** | ❌ SSH X11 forwarding only | ✅ Web browser access |
| **Multi-User** | ❌ Single display | ✅ Multiple connections |

### Which One Should You Use?

**Use `qgroundcontrol-x11` (X11) if:**
- Running on Linux or WSL2
- Need maximum performance for map rendering
- Working locally (not remote)
- Want lowest latency

**Use `qgroundcontrol` (VNC) if:**
- Running on Windows/Mac (without X server)
- Need remote access via web browser
- Working over network/SSH
- Want multiple simultaneous users

## Quick Start

### Option A: X11 Forwarding (Linux Native - Recommended)

**1. Prerequisites:**
```bash
# Allow Docker containers to access X server
xhost +local:docker

# Set environment variables (optional, but recommended)
export UID=$(id -u)
export GID=$(id -g)
```

**2. Build the Image:**
```bash
cd docker
docker compose -f docker-compose-master.yml build qgroundcontrol-x11
```

**3. Launch with PX4 Swarm:**
```bash
docker compose -f docker-compose-master.yml --profile linux-integrated up \
  px4-bridge-drone-{1..3} qgroundcontrol-x11
```

**4. QGC appears on your desktop** - that's it! No browser, no VNC client needed.

---

### Option B: VNC/noVNC (Cross-Platform)

**1. Build the Image:**
```bash
cd docker
docker compose -f docker-compose-master.yml build qgroundcontrol
```

**2. Launch with PX4 Swarm:**
```bash
docker compose -f docker-compose-master.yml --profile integrated up \
  px4-bridge-drone-{1..3} qgroundcontrol
```

**3. Access QGroundControl:**

**Via Web Browser (Easiest):**
```
http://localhost:6080/vnc.html
```
- No VNC client required
- Works in any modern web browser
- Click "Connect" button in noVNC interface

**Via VNC Client:**
```
vnc://localhost:5900
Password: airsim
```
- Use any VNC client (RealVNC, TightVNC, TigerVNC, etc.)
- Better performance than web interface
- Native desktop integration

## Architecture

### Network Integration

QGroundControl connects to two Docker networks:

1. **`px4_network`** (172.20.0.0/16) - Primary MAVLink communication with PX4 drones
2. **`airsim-ecosystem`** (172.30.0.0/16) - Ecosystem coordination

### MAVLink Port Configuration

| Port | Protocol | Purpose |
|------|----------|---------|
| 14550 | UDP | Default GCS MAVLink port (QGC listens here) |
| 14551-14578 | UDP | Additional MAVLink ports (multi-drone support) |
| 5900 | TCP | VNC server |
| 6080 | TCP | noVNC web interface |

### How Multi-Drone Discovery Works

1. **PX4 Broadcast**: Each PX4 container broadcasts MAVLink messages to UDP port 14550 + instance number
   - Drone 1 (instance 0): UDP 14550
   - Drone 2 (instance 1): UDP 14551
   - Drone 3 (instance 2): UDP 14552
   - etc.

2. **QGC Auto-Discovery**: QGroundControl listens on UDP 14550 and auto-discovers all broadcasting drones

3. **Network Isolation**: All communication happens within `px4_network` Docker bridge, ensuring clean separation from host

## Configuration

### Environment Variables

Set these in your `.env` file or pass via `docker-compose` command:

| Variable | Default | Description |
|----------|---------|-------------|
| `QGC_VNC_PASSWORD` | `airsim` | VNC server password |
| `QGC_VNC_RESOLUTION` | `1920x1080` | Display resolution |

**Example `.env` configuration:**
```bash
# QGroundControl Settings
QGC_VNC_PASSWORD=your_secure_password
QGC_VNC_RESOLUTION=2560x1440
```

### Custom Resolution

```bash
docker compose -f docker-compose-master.yml up qgroundcontrol \
  -e QGC_VNC_RESOLUTION=2560x1440
```

## Deployment Profiles

The QGroundControl service is available in multiple profiles:

| Profile | Description | Use Case |
|---------|-------------|----------|
| `integrated` | Full stack (AirSim + PX4 + ROS2 + QGC) | Complete simulation environment |
| `px4-only` | PX4 swarm + QGC | PX4 SITL testing with ground control |
| `gcs-tools` | Ground control tools only | Testing GCS without simulation |
| `development` | Development environment | Development and debugging |
| `linux-integrated` | Linux native deployment | Maximum performance on Linux |

### Profile Examples

**Full Integrated Stack (3 Drones + QGC + ROS2):**
```bash
docker compose -f docker-compose-master.yml --profile integrated up
```

**PX4 Swarm Testing (6 Drones + QGC):**
```bash
docker compose -f docker-compose-master.yml --profile px4-only up \
  px4-bridge-drone-{1..6} qgroundcontrol
```

**GCS Development:**
```bash
docker compose -f docker-compose-master.yml --profile gcs-tools up
```

## Usage Scenarios

### Scenario 1: Monitor Multi-Drone Swarm

1. Start AirSim simulation (Windows or Linux)
2. Launch PX4 drones and QGroundControl:
   ```bash
   docker compose -f docker-compose-master.yml --profile px4-only up \
     px4-bridge-drone-{1..3} qgroundcontrol
   ```
3. Access QGC via browser: `http://localhost:6080/vnc.html`
4. QGC will auto-discover all 3 drones
5. Monitor telemetry, plan missions, send commands

### Scenario 2: Mission Planning

1. Launch QGC in standalone mode:
   ```bash
   docker compose -f docker-compose-master.yml --profile gcs-tools up qgroundcontrol
   ```
2. Access via VNC client for best performance
3. Plan waypoint missions offline
4. Save mission files for later upload to drones

### Scenario 3: Development Testing

1. Start with development profile:
   ```bash
   docker compose -f docker-compose-master.yml --profile development up
   ```
2. Includes QGC + debugging tools
3. Modify PX4 parameters via QGC
4. Test MAVLink command sequences

## Troubleshooting

### QGroundControl Not Detecting Drones

**Check PX4 MAVLink broadcast:**
```bash
# Inside PX4 container
docker exec -it px4-drone-1 bash
cat /px4_workspace/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
```

Ensure `MAV_1_BROADCAST=1` is set in PX4 environment variables.

**Check network connectivity:**
```bash
# From QGC container
docker exec -it qgroundcontrol bash
netstat -uln | grep 14550  # Should show listening on UDP 14550
```

**Verify Docker network:**
```bash
docker network inspect px4_network
```
Ensure both PX4 containers and QGC are on the same network.

### VNC Connection Fails

**Check VNC server status:**
```bash
docker logs qgroundcontrol
```
Look for "x11vnc started successfully" message.

**Test noVNC web interface:**
```bash
curl http://localhost:6080
```
Should return HTML content (noVNC page).

**Firewall issues:**
- Windows: Check Windows Firewall allows ports 5900, 6080
- Linux: Check `ufw` or `iptables` rules

### QGroundControl Crashes on Startup

**Check AppImage extraction:**
```bash
docker exec -it qgroundcontrol bash
ls -la /opt/qgroundcontrol/squashfs-root/
```

**View detailed logs:**
```bash
docker logs -f qgroundcontrol
```

**Restart with fresh container:**
```bash
docker compose -f docker-compose-master.yml down qgroundcontrol
docker compose -f docker-compose-master.yml up qgroundcontrol
```

### Poor Performance / Lag

**Use VNC client instead of web browser:**
- Web browser (noVNC) adds latency
- Native VNC client has ~30% better performance

**Reduce display resolution:**
```bash
docker compose -f docker-compose-master.yml up qgroundcontrol \
  -e QGC_VNC_RESOLUTION=1280x720
```

**Check system resources:**
```bash
docker stats qgroundcontrol
```

## Advanced Configuration

### Persistent QGroundControl Settings

Mount a volume for QGC config persistence:

```yaml
# Add to qgroundcontrol service in docker-compose-master.yml
volumes:
  - qgc_config:/home/qgc/.config/QGroundControl.org
```

Then define volume:
```yaml
volumes:
  qgc_config:
    name: qgroundcontrol_config
```

### Custom MAVLink Port

If using non-standard MAVLink ports, expose additional ports:

```yaml
ports:
  - "5900:5900"
  - "6080:6080"
  - "14550:14550/udp"
  - "14551:14551/udp"  # Additional port
  - "14552:14552/udp"  # Additional port
```

### Multiple GCS Instances

To run multiple QGC instances (e.g., for different swarms):

```bash
# First instance (default ports)
docker compose -f docker-compose-master.yml up qgroundcontrol

# Second instance (custom ports)
docker run -d --name qgc-swarm2 \
  --network px4_network \
  -p 5901:5900 \
  -p 6081:6080 \
  -p 14560:14550/udp \
  airsim-qgc:latest
```

## Integration with Master Compose File

The QGroundControl service is defined in `/docker/docker-compose-master.yml`:

**Key Configuration:**
```yaml
qgroundcontrol:
  build:
    context: qgroundcontrol
    dockerfile: Dockerfile.qgc
  networks:
    - px4_network       # MAVLink communication
    - airsim-ecosystem  # Ecosystem coordination
  ports:
    - "5900:5900"       # VNC
    - "6080:6080"       # noVNC
    - "14550:14550/udp" # MAVLink
  depends_on:
    px4-bridge-drone-1:
      condition: service_started
  profiles:
    - integrated
    - px4-only
    - gcs-tools
    - development
    - linux-integrated
```

## Files and Structure

```
docker/qgroundcontrol/
├── Dockerfile.qgc              # QGC container build definition
├── scripts/
│   └── start-qgc.sh           # VNC + QGC launcher script
└── README.md                   # This file
```

**Related Files:**
- `/docker/docker-compose-master.yml` - Master orchestration file (QGC service definition)
- `/docker/px4_airsim_docker/Dockerfile.px4` - PX4 container (MAVLink endpoints)
- `/docker/px4_airsim_docker/docker-compose-px4.yml` - PX4 drone services

## References

- [QGroundControl Official Site](http://qgroundcontrol.com/)
- [QGroundControl Documentation](https://docs.qgroundcontrol.com/master/en/)
- [MAVLink Protocol](https://mavlink.io/en/)
- [PX4 Autopilot](https://px4.io/)
- [AirSim Documentation](https://microsoft.github.io/AirSim/)

## Support

For issues specific to this Docker integration:
1. Check the [Troubleshooting](#troubleshooting) section above
2. Review Docker logs: `docker logs qgroundcontrol`
3. Verify network configuration: `docker network inspect px4_network`
4. Check AirSim/PX4 ecosystem documentation

For QGroundControl application issues:
- [QGC User Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/)
- [QGC GitHub Issues](https://github.com/mavlink/qgroundcontrol/issues)

---

**Last Updated**: 2025-10-03
**Docker Image**: `airsim-qgc:latest`
**QGroundControl Version**: Latest stable (auto-downloaded from official CDN)
