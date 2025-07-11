# PX4 AirSim Docker Integration

This directory contains Docker configurations for running PX4 SITL with AirSim, supporting both host networking and bridge networking modes.

## 🗂️ Directory Structure

```
px4_airsim_docker_v2/
├── README.md                           # This file
├── Dockerfile.ultra-swarm             # Bridge networking Dockerfile
├── Dockerfile.ultra-swarm-host        # Host networking Dockerfile
├── docker-compose.ultra-swarm.yml     # Bridge networking compose
├── docker-compose.host-network.yml    # Host networking compose
├── config/                            # Configuration files
│   ├── gps_locations.json            # GPS coordinates for drones
│   └── px4_params.txt                 # PX4 parameters
├── docs/                              # Documentation
│   ├── README-UltraSwarm.md          # Bridge networking guide
│   └── README-HostNetwork.md          # Host networking guide
└── tools/                             # Scripts and utilities
    ├── quick-start-wsl2.sh           # Interactive WSL2 setup
    ├── start-ultra-swarm-wsl.sh      # WSL2 launcher
    ├── start-ultra-swarms.sh         # Bridge network launcher
    ├── test-connectivity.sh          # Connection testing
    ├── windows-firewall-setup.ps1    # Windows firewall config
    └── build-*.sh/bat                # Build scripts
```

## 🚀 Quick Start

### For WSL2 + Windows AirSim (Recommended)

**Option 1: Simplified Workflow (Uses existing PX4Scripts)**
```bash
# Build and launch with existing run_airsim_sitl.sh
./tools/start-simple-swarm-wsl.sh build       # First time only
./tools/start-simple-swarm-wsl.sh single      # Test with 1 drone
./tools/start-simple-swarm-wsl.sh test-3      # Test with 3 drones
```

**Option 2: Ultra-Swarm Workflow (Advanced)**
```bash
# Interactive setup with guidance
./tools/quick-start-wsl2.sh

# Or direct launch
./tools/start-ultra-swarm-wsl.sh single
```

### For Linux-only Setup

```bash
# Host networking (recommended)
docker-compose -f docker-compose.host-network.yml up

# Bridge networking
docker-compose -f docker-compose.ultra-swarm.yml up
```

## 📖 Documentation

- **[Ultra-Swarm Guide](docs/README-UltraSwarm.md)** - Complete guide for bridge networking setup
- **[Host Network Guide](docs/README-HostNetwork.md)** - Host networking configuration
- **[Main CLAUDE.md](../../CLAUDE.md)** - Development workflow and commands

## 🔧 Core Files

### Docker Configurations

| File | Purpose | Networking |
|------|---------|------------|
| `docker-compose.ultra-swarm.yml` | Multi-drone swarm | Bridge |
| `docker-compose.host-network.yml` | Host networking | Host |
| `Dockerfile.ultra-swarm` | Bridge networking image | Bridge |
| `Dockerfile.ultra-swarm-host` | Host networking image | Host |

### Network Mode Comparison

| Feature | Host Networking | Bridge Networking |
|---------|----------------|------------------|
| **WSL2 + Windows** | ✅ Supported | ❌ Complex setup |
| **Linux-only** | ✅ Simple | ✅ Isolated |
| **Port conflicts** | ⚠️ Possible | ✅ Avoided |
| **Performance** | ✅ Best | ✅ Good |
| **Debugging** | ✅ Easy | ⚠️ Network complexity |

## 🛠️ Tools and Scripts

### Launchers
- `tools/quick-start-wsl2.sh` - Interactive setup for WSL2
- `tools/start-ultra-swarm-wsl.sh` - WSL2-aware launcher
- `tools/start-ultra-swarms.sh` - Traditional bridge network launcher

### Utilities
- `tools/test-connectivity.sh` - Test AirSim connectivity
- `tools/windows-firewall-setup.ps1` - Configure Windows firewall
- `tools/generate_compose.py` - Generate custom configurations

### Build Scripts
- `tools/build-host-network.sh/.bat` - Build host networking images
- `tools/clean-and-build.sh/.bat` - Clean rebuild process

## ⚡ Quick Commands

```bash
# Test connectivity
./tools/test-connectivity.sh

# Start single drone (WSL2)
./tools/start-ultra-swarm-wsl.sh single

# Start 3 drones (WSL2)
./tools/start-ultra-swarm-wsl.sh test-3

# Start full swarm (9 drones)
./tools/start-ultra-swarm-wsl.sh swarm1-full

# Check status
./tools/start-ultra-swarm-wsl.sh status

# View logs
./tools/start-ultra-swarm-wsl.sh logs px4-swarm-1-drone-1

# Stop all
./tools/start-ultra-swarm-wsl.sh stop
```

## 🎯 Choosing the Right Setup

### Use Host Networking When:
- Running WSL2 with Windows AirSim
- Want simplest network configuration
- Need direct port access from host
- Debugging network issues

### Use Bridge Networking When:
- Running everything on Linux
- Want network isolation
- Need multiple isolated swarms
- Avoiding port conflicts

## 🆘 Troubleshooting

1. **Check environment**: `./tools/test-connectivity.sh`
2. **WSL2 issues**: See [WSL2 section in Ultra-Swarm guide](docs/README-UltraSwarm.md#method-1-host-networking-with-wsl2-windows--linux)
3. **Network problems**: Compare host vs bridge networking options
4. **Build issues**: Use `./tools/clean-and-build.sh`

## 📦 Dependencies

- Docker & Docker Compose
- AirSim (Windows or Linux)
- PX4 (built into containers)
- Python 3 (for scripts)
- netcat (for connectivity testing)

---

🚁 **Ready to fly?** Start with `./tools/quick-start-wsl2.sh` for WSL2 or `docker-compose -f docker-compose.host-network.yml up` for Linux!