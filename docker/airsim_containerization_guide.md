# AirSim Containerization Guide

This guide explains how to containerize AirSim to solve the networking issues between Docker containers and host-based AirSim.

## Why Containerize AirSim?

Based on our WSL2 vs Docker analysis, containerizing AirSim solves:
- **UDP packet loss**: All components share the same Docker network
- **Complex routing**: Eliminates host↔container network translation
- **Timeout issues**: Reduces network latency and improves reliability
- **GPS HOME_POSITION**: Ensures reliable MAVLink message delivery

## Current Multi-Network Architecture (docker-compose-master.yml)

The current setup uses a sophisticated multi-network architecture that solves most networking issues while keeping AirSim on the host:

### Network Topology
```
┌─── Windows Host ────┐   ┌──────────── Docker Multi-Network ──────────────┐
│                     │   │                                                │
│     AirSim          │◄──┤        airsim-ecosystem (172.30.0.0/16)        │
│   (Host: 41451)     │   │  ┌─────────────────────────────────────────────┤
│                     │   │  │ ┌─────────────┐  ┌─────────────────────────┐│
└─────────────────────┘   │  │ │ ROS2 Node   │  │   Ecosystem Monitor     ││
         ▲                │  │ │ (ros2)      │  │   (airsim-monitor)      ││
         │                │  │ │ VNC: 5901   │  │   Health checks         ││
         │                │  │ └─────────────┘  └─────────────────────────┘│
         │                │  └─────────────────────────────────────────────┤
    WSL2 vEthernet        │                                                │
    (172.XX.XXX.1)        │  px4_network (172.20.0.0/16)                   │
    detection via         │  ┌─────────────────────────────────────────────┤
    PowerShell tools      │  │ ┌─────────────┐  ┌─────────────┐  ┌───────┐ │
         │                │  │ │ PX4 Drone1  │  │ PX4 Drone2  │  │  ...  │ │
         │                │  │ │(px4-drone-1)│  │(px4-drone-2)│  │Drone6 │ │
         └────────────────┼──┤ │ TCP: 4560   │  │ TCP: 4561   │  │ 4565  │ │
                          │  │ │ UDP: 14540  │  │ UDP: 14541  │  │       │ │
                          │  │ └─────────────┘  └─────────────┘  └───────┘ │
                          │  └─────────────────────────────────────────────┤
                          │                                                │
                          │  ros2-multi-node-network (172.26.0.0/16)       │
                          │  ┌─────────────────────────────────────────────┤
                          │  │ ┌─────────────────────────────────────────┐ │
                          │  │ │        ROS2 Internal Network            │ │
                          │  │ │     (Multi-vehicle coordination)        │ │
                          │  │ └─────────────────────────────────────────┘ │
                          │  └─────────────────────────────────────────────┘
                          └────────────────────────────────────────────────┘
```

### Service Distribution by Network

**airsim-ecosystem (172.30.0.0/16)**
- ROS2 Multi-Node (ros2-node)
- Ecosystem Monitor (airsim-monitor) 
- Development Helper (airsim-dev-helper)
- Cross-ecosystem communication bridge

**px4_network (172.20.0.0/16)**
- PX4 Drone 1-7 containers (px4-drone-1 through px4-drone-7)
- Each drone with unique ports: 4560-4566 (TCP) and 14540-14546 (UDP)
- PX4 SITL instances with proper instance IDs (0-6)

**ros2-multi-node-network (172.26.0.0/16)**
- ROS2 internal coordination
- Multi-vehicle communication protocols

## Network Communication Patterns

### Current WSL2 IP Detection Flow
1. **PowerShell Detection**: `get-wsl2-ip.ps1` finds vEthernet (WSL) adapter IP
2. **Environment Variables**: `PX4_SIM_HOSTNAME` and `AIRSIM_HOST_IP` set to detected IP
3. **Container Communication**: All containers use detected IP to reach Windows AirSim
4. **Port Mapping**: Host ports exposed for external QGroundControl access

### Service Dependencies
```
┌─ Windows AirSim (Host: 172.18.144.1:41451)
│
├─ PX4 Drones (px4_network)
│  ├─ px4-drone-1 → AirSim:4560
│  ├─ px4-drone-2 → AirSim:4561
│  └─ ... → AirSim:456X
│
├─ ROS2 Ecosystem (airsim-ecosystem + ros2-multi-node-network)
│  ├─ ros2-node → AirSim:41451 (API calls)
│  └─ VNC Desktop → localhost:5901
│
└─ Monitoring (airsim-ecosystem)
   ├─ ecosystem-monitor → Health checks
   └─ dev-helper → Debugging tools
```

## Future Containerized AirSim Architecture

### Option 1: Full Stack Containerization (Future Goal)
```
┌─────────────────────────────────────────────────────────────────────┐
│                    airsim-ecosystem Network                         │
│                         172.30.0.0/16                               │
│                                                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │   AirSim    │  │ PX4 Drone1  │  │ PX4 Drone2  │  │  ROS2 Node  │ │
│  │ (Container) │  │172.30.0.10  │  │172.30.0.11  │  │172.30.0.20  │ │
│  │             │  │             │  │             │  │             │ │
│  │ API: 41451  │  │ TCP: 4560   │  │ TCP: 4561   │  │ VNC: 5901   │ │
│  │ UDP: 14xxx  │  │ UDP: 14540  │  │ UDP: 14541  │  │ ROS: 7400   │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │
│                                                                     │
│  Benefits:                                                          │
│  - No WSL2 IP detection needed                                      │
│  - Direct container-to-container communication                      │
│  - Eliminates UDP packet loss issues                                │
│  - Simplified networking configuration                              │
└─────────────────────────────────────────────────────────────────────┘
```

## Migration Strategy

### Phase 1: Current State (Implemented)
- Multi-network architecture with proper segmentation
- WSL2 IP detection via PowerShell tools
- External PowerShell-based IP detection (no container dependencies)
- Support for 1-7 configurable PX4 drones
- Cross-ecosystem monitoring and debugging

### Phase 2: Hybrid Optimization (Next Steps)
- Optimize network bridge performance
- Add AirSim container health monitoring
- Implement dynamic service discovery

### Phase 3: Full Containerization (Future)
- Package AirSim in Docker container
- Migrate to single-network architecture
- Eliminate host networking dependencies
- Add GPU passthrough for AirSim rendering

## Key Advantages of Current Architecture

1. **Network Isolation**: Separate networks prevent interference between subsystems
2. **Scalability**: Supports 1-7 drones with independent port assignments
3. **Reliability**: PowerShell-based IP detection eliminates container race conditions
4. **Monitoring**: Ecosystem health monitoring across all networks
5. **Debugging**: Development helper service with network diagnostic tools
6. **Modularity**: Profile-based deployment (integrated, px4-only, ros2-only, development)

## Troubleshooting Network Issues

### Common Problems
- **UDP Packet Loss**: Usually caused by Windows Firewall or WSL2 bridge issues
- **Connection Timeouts**: Check WSL2 IP detection and environment variables
- **Port Conflicts**: Ensure unique port assignments for each drone instance

### Diagnostic Commands
```bash
# Check network connectivity
docker exec -it airsim-dev-helper nc -zv 172.18.144.1 41451

# Monitor ecosystem health
docker logs -f airsim-monitor

# Test individual drone connections
for port in 4560 4561 4562; do
    nc -zv localhost $port
done
```
