# PX4 Remote Server Connectivity Architecture

## Overview
This document explains how PX4 connects to remote servers in the Cosys-AirSim Docker environment, based on analysis of the current infrastructure and PX4 MAVLink protocols.

## Current Architecture

### 1. MAVLink Communication Stack

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   AirSim UE5    │    │  PX4 Container  │    │  Remote Client  │
│  (Simulator)    │    │     (SITL)      │    │ (QGC/MAVSDK)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │ TCP 4561              │ UDP Multiple          │ UDP 14550+
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │ MAVLink Router  │
                    │  (Central Hub)  │
                    └─────────────────┘
```

### 2. Network Flow Analysis

#### A. AirSim ↔ PX4 Connection
- **Protocol**: TCP (reliable for HIL simulation)
- **Port**: 4561 (mapped from container to host)
- **Purpose**: Hardware-in-the-Loop sensor data and actuator commands
- **Direction**: Bidirectional

#### B. PX4 ↔ MAVLink Router Connection  
- **Protocol**: UDP (efficient for telemetry)
- **Port**: 14581 (local to container)
- **Purpose**: MAVLink message routing and distribution
- **Direction**: Bidirectional

#### C. Remote Client ↔ MAVLink Router Connection
- **Protocol**: UDP (standard for MAVLink)
- **Ports**: 14550 (QGC), 14540 (API), 14541+ (per-drone)
- **Purpose**: External control and monitoring
- **Direction**: Bidirectional

## PX4 MAVLink Configuration Details

### 1. Port Assignment Strategy
Based on `px4-rc.mavlink` and your container setup:

```bash
# Per-instance port calculation:
udp_offboard_port_local=$((14580+px4_instance))   # PX4 → MAVLink Router
udp_offboard_port_remote=$((14540+px4_instance))  # MAVLink Router → Remote
udp_gcs_port_local=$((18570+px4_instance))        # Ground Control Station
```

### 2. MAVLink Stream Configuration
From your `run_airsim_sitl_mavlink_router.sh`:

```bash
# PX4 parameters for MAVLink Router integration
param set MAV_SYS_ID $((1 + instance_num))        # Unique system ID
mavlink start -x -u $PX4_LOCAL_PORT -r 4000000    # High-rate local stream
```

### 3. Connection Modes

#### Mode 1: Direct UDP (Simple)
```
PX4 Container ──UDP:14540──→ Remote Server
     ↑
     └── Port mapping: 14540:14540/udp
```

#### Mode 2: MAVLink Router Hub (Current - Recommended)
```
PX4 Container ──UDP:14581──→ MAVLink Router ──UDP:14550──→ QGroundControl
                                   │
                                   ├──UDP:14540──→ API Clients
                                   │
                                   └──UDP:14541+──→ Per-drone Clients
```

## Docker Network Configuration

### 1. Bridge Network Setup
From `docker-compose-slim.yml`:

```yaml
networks:
  px4_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

### 2. Container Port Mappings
```yaml
px4-bridge-drone-1:
  ports:
    - "4561:4561/tcp"     # AirSim HIL connection
    - "14581:14581/udp"   # Control port remote
    - "14541:14541/udp"   # QGroundControl access
```

### 3. IP Address Assignment
- **Container**: 172.20.0.11 (drone 1), 172.20.0.12 (drone 2), etc.
- **Gateway**: 172.20.0.1 (Docker bridge gateway)
- **Host Access**: localhost or Docker host IP

## Current Connectivity Capabilities

### ✅ Working Connections
1. **Local QGroundControl**: localhost:14541-14549
2. **Local API Access**: localhost:14540
3. **AirSim Integration**: localhost:4561
4. **Container-to-container**: Internal Docker network
5. **LAN Access**: Docker host IP + ports

### 🔧 Missing for Remote Server Access
1. **Internet routing**: No public IP configuration
2. **Security layer**: No authentication or VPN
3. **Cloud deployment**: No cloud-specific configs
4. **Connection resilience**: No reconnection handling

## Remote Server Connection Methods

### Method 1: Direct Internet Exposure
```
Internet → Public IP:14540 → Docker Host → Container:14540 → PX4
```

**Pros**: Simple, direct
**Cons**: Security risks, no connection management

### Method 2: VPN Tunnel (Recommended)
```
Remote Client → VPN → Private Network → Docker Host → Container → PX4
```

**Pros**: Secure, encrypted, private network
**Cons**: Requires VPN setup

### Method 3: Cloud Relay Service
```
Remote Client → Cloud Service → WebSocket/HTTP → Docker Host → Container → PX4
```

**Pros**: Web-friendly, scalable, authentication
**Cons**: Protocol conversion complexity

## Configuration Variables for Remote Access

### 1. Environment Variables (Current)
```bash
PX4_SIM_HOSTNAME=172.28.240.1    # AirSim host IP
MAV_0_BROADCAST=1                # Enable broadcast
MAV_0_REMOTE_PORT=14541          # Remote target port
```

### 2. Required for Remote Server
```bash
REMOTE_SERVER_IP=203.0.113.100   # Public server IP
REMOTE_SERVER_PORT=14540         # Server listening port
SECURITY_MODE=vpn                # Security method
VPN_CONFIG_PATH=/path/to/vpn     # VPN configuration
```

## Next Steps for Remote Connectivity

1. **Cloud deployment templates** for AWS/GCP/Azure
2. **VPN integration** with WireGuard
3. **Reverse proxy setup** for secure routing
4. **Authentication mechanisms** for client validation
5. **Connection monitoring** and health checks

## Security Considerations

### Current Security Level: ⚠️ Development Only
- No authentication
- No encryption (except VPN)
- Open ports on Docker host
- No rate limiting

### Required for Production: 🛡️ Security Hardened
- Client authentication (certificates/tokens)
- Encrypted connections (TLS/VPN)
- Firewall rules and access controls
- Connection monitoring and logging
- Rate limiting and DDoS protection

---

*This document serves as the foundation for implementing secure remote server connectivity for PX4 in the Cosys-AirSim environment.*