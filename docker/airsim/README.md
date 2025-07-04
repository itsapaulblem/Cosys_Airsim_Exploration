# Cosys-AirSim Docker Container

This is a streamlined Docker setup for **Cosys-AirSim** running on **Unreal Engine 5.5**, designed to work with external PX4 instances.

## 🏗️ Architecture

- **Base Image**: `ghcr.io/epicgames/unreal-engine:runtime-5.5`
- **AirSim Version**: Cosys-AirSim (latest)
- **Python Client**: `cosysairsim`
- **Networking**: Host mode for easy PX4 connectivity

## 🚀 Quick Start

### 1. Build and Start Container

```bash
# Build and start with VNC access
docker-compose up --build

# Or run in headless mode
DISPLAY_MODE=headless docker-compose up --build

# Or use host display (Linux/WSL2 with X11)
DISPLAY_MODE=host docker-compose up --build
```

### 2. Access Methods

- **VNC**: Open http://localhost:5900 in a browser
- **API**: Connect to localhost:41451 for AirSim API
- **Direct**: Use host display if running on Linux/WSL2

### 3. Connect External PX4

Since this container uses host networking, your external PX4 instances can connect directly:

```bash
# Example: Start PX4 SITL pointing to AirSim
cd /path/to/PX4-Autopilot
make px4_sitl none_iris
```

PX4 will automatically connect to AirSim running in the container.

## ⚙️ Configuration

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `DISPLAY_MODE` | `vnc`\|`headless`\|`host` | `vnc` |
| `VNC_RESOLUTION` | VNC screen resolution | `1920x1080` |
| `SCREEN_WIDTH` | AirSim window width | `1280` |
| `SCREEN_HEIGHT` | AirSim window height | `720` |

### Custom Settings

Mount your own settings.json:

```yaml
volumes:
  - ./my_settings.json:/home/airsim_user/Documents/AirSim/settings.json:ro
```

### Default Vehicle Configuration

The container includes a single drone configured for PX4:

- **Name**: `Drone1`
- **TCP Port**: 4560
- **MAVLink Ports**: 14540 (local), 14580 (remote)
- **Position**: (0, 0, -2)

## 🔧 Usage Examples

### Python API

```python
import cosysairsim as airsim

# Connect to containerized AirSim
client = airsim.MultirotorClient('localhost')
client.confirmConnection()

# Your simulation code here...
```

### Multiple Vehicles

To add more vehicles, mount a custom settings.json with multiple vehicle definitions.

## 🐛 Troubleshooting

### Container Issues

```bash
# View logs
docker-compose logs cosys-airsim

# Access container shell
docker-compose exec cosys-airsim bash

# Test connectivity
docker-compose exec cosys-airsim python3 -c "import cosysairsim; print('✅ cosysairsim imported successfully')"
```

### PX4 Connection Issues

```bash
# Check if AirSim is listening
netstat -an | grep 4560

# Test MAVLink ports
nc -u localhost 14540
```

## 📁 File Structure

```
docker/airsim/
├── Dockerfile.binary          # Cosys-AirSim container definition
├── docker-compose.yml         # Simple orchestration
├── entrypoint.sh              # Container startup script
├── settings/
│   └── container_settings.json # Default AirSim configuration
└── README.md                  # This file
```

## 🔄 Integration with Existing PX4

This setup is designed to work seamlessly with your existing PX4 setup:

1. **WSL2 PX4**: PX4 running in WSL2 can connect directly via host networking
2. **Docker PX4**: Other PX4 containers can connect using Docker networking
3. **Native PX4**: Local PX4 installations connect via localhost

## 📚 References

- [Cosys-AirSim Documentation](https://cosys-lab.github.io/Cosys-AirSim/)
- [Cosys-AirSim Docker Guide](https://cosys-lab.github.io/Cosys-AirSim/docker_ubuntu/)
- [PX4 SITL with AirSim](https://docs.px4.io/main/en/simulation/airsim.html) 