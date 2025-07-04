# AirSim + PX4 Full Stack Docker Setup

This directory contains a complete Docker-based solution for running AirSim with PX4 SITL, eliminating networking complexities by running everything in containers on the same Docker network.

## 🚀 Quick Start

### Prerequisites

1. **Docker Desktop** installed and running
2. **NVIDIA Docker** (for GPU acceleration)
3. **GitHub Container Registry Access** for Unreal Engine images

```bash
# Authenticate with GitHub Container Registry
docker login ghcr.io
```

### Option 1: One-Command Setup (Recommended)

```bash
# Navigate to the docker directory
cd docker

# Run the full stack (this will download, build, and start everything)
./run_full_stack.sh
```

### Option 2: Manual Setup

```bash
# 1. Download Blocks environment
./download_blocks_env_binary.sh

# 2. Build the Docker images
docker-compose -f docker-compose-full-stack.yml build

# 3. Start the full stack
docker-compose -f docker-compose-full-stack.yml --profile full-stack up
```

## 📋 What's Included

### Services

- **airsim-blocks**: AirSim with Unreal Engine runtime and Blocks environment
- **px4-single**: PX4 SITL configured for AirSim integration

### Network Configuration

```
Docker Network: airsim-network (172.25.0.0/16)
├── AirSim Container: 172.25.0.20
└── PX4 Container: 172.25.0.10
```

### Port Mappings

- **14550/udp**: QGroundControl connection
- **14541/udp**: MAVLink control local
- **14581/udp**: MAVLink control remote

## 🔧 Management Commands

```bash
# View logs
docker-compose -f docker-compose-full-stack.yml logs -f

# Stop the stack
docker-compose -f docker-compose-full-stack.yml down

# Check container status
docker-compose -f docker-compose-full-stack.yml ps

# Access PX4 console
docker exec -it px4-full-stack /Scripts/px4_shell.sh

# Test GPS functionality
python test_full_stack_gps.py
```

## 🛠️ Configuration Files

### Core Files

- `docker-compose-full-stack.yml`: Main Docker Compose configuration
- `full_stack_settings.json`: AirSim settings optimized for Docker network
- `Dockerfile_binary`: AirSim runtime container
- `Dockerfile_source`: AirSim development container

### Helper Scripts

- `run_full_stack.sh`: Complete setup and launch script
- `download_blocks_env_binary.sh`: Downloads Blocks environment
- `test_full_stack_gps.py`: GPS functionality test suite

## 🧪 Testing

### GPS Functionality Test

```bash
cd docker
python test_full_stack_gps.py
```

This test will:
1. Verify Docker network connectivity
2. Monitor MAVLink traffic
3. Test AirSim connection
4. Check GPS home location setting
5. Test drone arming

### Manual Testing

```python
import cosysairsim as airsim
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Wait for GPS home location
for i in range(30):
    home = client.getHomeGeoPoint()
    if not math.isnan(home.latitude):
        print(f"GPS Home set: ({home.latitude}, {home.longitude})")
        break
    time.sleep(1)

# Arm and test
client.enableApiControl(True)
client.armDisarm(True)
print("Drone armed successfully!")
```

## 🔍 Troubleshooting

### Common Issues

1. **"No DISPLAY environment variable"**
   ```bash
   export DISPLAY=:0
   xhost +local:docker
   ```

2. **GPU not working**
   - Install nvidia-docker2
   - Verify with: `docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi`

3. **GitHub Container Registry access denied**
   ```bash
   docker login ghcr.io
   # Use your GitHub username and personal access token
   ```

4. **GPS home location not setting**
   - Check container logs: `docker logs px4-full-stack`
   - Verify network connectivity: `docker exec px4-full-stack ping -c 3 172.25.0.20`

### Debug Commands

```bash
# Check container network details
docker network inspect px4_airsim_docker_airsim-network

# Monitor PX4 logs in real-time
docker logs px4-full-stack --follow

# Check AirSim settings
docker exec airsim-blocks cat /home/airsim_user/Documents/AirSim/settings.json

# Test UDP connectivity
python -c "import socket; sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.sendto(b'test', ('172.25.0.10', 14581)); print('UDP OK')"
```

## 🌟 Advantages of This Setup

1. **No Network Configuration Hassles**: Everything runs on the same Docker network
2. **Isolated Environment**: Doesn't interfere with host system
3. **Reproducible**: Consistent setup across different machines
4. **Easy Scaling**: Can easily add more drones or services
5. **Version Control**: Complete environment in code

## 📁 Directory Structure

```
docker/
├── docker-compose-full-stack.yml    # Main Docker Compose file
├── full_stack_settings.json         # AirSim settings for Docker
├── run_full_stack.sh                # One-command setup script
├── download_blocks_env_binary.sh    # Blocks environment downloader
├── test_full_stack_gps.py           # GPS test suite
├── Dockerfile_binary                # AirSim runtime container
├── Dockerfile_source                # AirSim dev container
├── blocks_env/                      # Downloaded Blocks environment
└── px4_airsim_docker/              # PX4 container configuration
```

## 🚀 Next Steps

After the stack is running:

1. **Connect QGroundControl**: Use `localhost:14550`
2. **Run Python scripts**: Use the test script as a starting point
3. **Add more drones**: Modify the docker-compose file
4. **Custom environments**: Replace Blocks with your own Unreal project

## 📖 Further Reading

- [Cosys-AirSim Docker Documentation](https://cosys-lab.github.io/Cosys-AirSim/docker_ubuntu/)
- [PX4 SITL Documentation](https://docs.px4.io/main/en/simulation/)
- [Docker Networking Guide](https://docs.docker.com/network/)

---

For issues or questions, check the troubleshooting section or create an issue in the repository. 