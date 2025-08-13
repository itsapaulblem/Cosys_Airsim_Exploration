# Docker Networking Guide for AirSim-PX4 Integration

## Overview

This guide provides comprehensive information about Docker networking options for AirSim-PX4 integration. It covers the trade-offs between different networking approaches, implementation details, and troubleshooting strategies specific to containerized environments.

## Table of Contents

1. [Docker Networking Fundamentals](#docker-networking-fundamentals)
2. [Bridge Network Implementation](#bridge-network-implementation)
3. [Host Network Implementation](#host-network-implementation)
4. [Network Mode Comparison](#network-mode-comparison)
5. [Container-to-Host Communication](#container-to-host-communication)
6. [Port Mapping Strategies](#port-mapping-strategies)
7. [Multi-Vehicle Scaling](#multi-vehicle-scaling)
8. [Troubleshooting Docker Networks](#troubleshooting-docker-networks)
9. [Performance Optimization](#performance-optimization)
10. [Security Considerations](#security-considerations)

## Docker Networking Fundamentals

### Docker Network Modes

Docker provides several network modes, each with different characteristics:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        Docker Network Modes                                    │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                       Bridge Network                                    │   │
│  │                                                                         │   │
│  │  Default Mode: Yes                                                      │   │
│  │  Isolation: High                                                        │   │
│  │  Performance: Moderate                                                  │   │
│  │  Port Mapping: Required                                                 │   │
│  │  DNS: Custom resolution                                                 │   │
│  │  Use Case: Production, multiple containers                              │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Host Network                                     │   │
│  │                                                                         │   │
│  │  Default Mode: No                                                       │   │
│  │  Isolation: None                                                        │   │
│  │  Performance: High                                                      │   │
│  │  Port Mapping: Not needed                                               │   │
│  │  DNS: Host resolution                                                   │   │
│  │  Use Case: Development, single container                                │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Custom Network                                   │   │
│  │                                                                         │   │
│  │  Default Mode: No                                                       │   │
│  │  Isolation: Configurable                                                │   │
│  │  Performance: Configurable                                              │   │
│  │  Port Mapping: Optional                                                 │   │
│  │  DNS: Custom resolution                                                 │   │
│  │  Use Case: Complex multi-container setups                               │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### AirSim-PX4 Networking Requirements

AirSim-PX4 integration has specific networking requirements:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    AirSim-PX4 Network Requirements                              │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      HIL Connection (TCP)                               │   │
│  │                                                                         │   │
│  │  Direction: PX4 → AirSim                                                │   │
│  │  Protocol: TCP                                                          │   │
│  │  Port: 4561 (default)                                                  │   │
│  │  Reliability: Required                                                  │   │
│  │  Latency: 2-5ms acceptable                                             │   │
│  │  Bandwidth: 50-100KB/s                                                 │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      GCS Connection (UDP)                               │   │
│  │                                                                         │   │
│  │  Direction: Bidirectional                                              │   │
│  │  Protocol: UDP                                                          │   │
│  │  Ports: 14541/14581 (default)                                          │   │
│  │  Reliability: Best effort                                               │   │
│  │  Latency: <2ms preferred                                                │   │
│  │  Bandwidth: 10-20KB/s                                                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      Container Requirements                             │   │
│  │                                                                         │   │
│  │  Host Access: Required                                                  │   │
│  │  Port Binding: TCP and UDP                                              │   │
│  │  DNS Resolution: host.docker.internal                                  │   │
│  │  Network Isolation: Optional                                            │   │
│  │  Multi-Instance: Support multiple containers                            │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Bridge Network Implementation

### Bridge Network Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                     Docker Bridge Network Architecture                         │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         Host Machine                                    │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                      AirSim Process                             │   │   │
│  │  │                                                                 │   │   │
│  │  │  Bindings:                                                      │   │   │
│  │  │  • TCP 0.0.0.0:4561 (HIL)                                      │   │   │
│  │  │  • UDP 0.0.0.0:14541 (GCS Local)                               │   │   │
│  │  │  • UDP 0.0.0.0:14581 (GCS Remote)                              │   │   │
│  │  │  • TCP 0.0.0.0:41451 (API)                                     │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                   Docker Engine                                 │   │   │
│  │  │                                                                 │   │   │
│  │  │  Port Mapping:                                                  │   │   │
│  │  │  • 127.0.0.1:4561/tcp → 172.20.0.11:4561/tcp                 │   │   │
│  │  │  • 127.0.0.1:14541/udp → 172.20.0.11:14541/udp               │   │   │
│  │  │  • 127.0.0.1:14581/udp → 172.20.0.11:14581/udp               │   │   │
│  │  │                                                                 │   │   │
│  │  │  DNS Resolution:                                                │   │   │
│  │  │  • host.docker.internal → 172.17.0.1                          │   │   │
│  │  │  • Alternative: host.docker.internal → 127.0.0.1              │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                   Bridge Network                                │   │   │
│  │  │                  (px4_network)                                  │   │   │
│  │  │                 172.20.0.0/16                                   │   │   │
│  │  │                                                                 │   │   │
│  │  │  ┌─────────────────────────────────────────────────────────┐   │   │   │
│  │  │  │              PX4 Container                              │   │   │   │
│  │  │  │            IP: 172.20.0.11                             │   │   │   │
│  │  │  │                                                         │   │   │   │
│  │  │  │  Environment:                                           │   │   │   │
│  │  │  │  • PX4_SIM_HOSTNAME=host.docker.internal               │   │   │   │
│  │  │  │  • MAV_0_REMOTE_HOST=host.docker.internal              │   │   │   │
│  │  │  │  • MAV_0_REMOTE_PORT=4561                              │   │   │   │
│  │  │  │                                                         │   │   │   │
│  │  │  │  Connections:                                           │   │   │   │
│  │  │  │  • TCP connect to host.docker.internal:4561            │   │   │   │
│  │  │  │  • UDP connect to host.docker.internal:14541           │   │   │   │
│  │  │  │  • UDP connect to host.docker.internal:14581           │   │   │   │
│  │  │  └─────────────────────────────────────────────────────────┘   │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Bridge Network Configuration

#### Docker Compose Configuration

```yaml
# docker-compose-bridge.yml
version: '3.8'

networks:
  px4_network:
    driver: bridge
    ipam:
      driver: default
      config:
        - subnet: 172.20.0.0/16
          gateway: 172.20.0.1

x-px4-common: &px4-common
  build:
    context: .
    dockerfile: Dockerfile
  environment: &px4-env
    # Use host.docker.internal to reach AirSim from bridge network
    PX4_SIM_HOSTNAME: host.docker.internal
    PX4_SIM_MODEL: iris
    DELAYED_START: true
  networks:
    - px4_network
  restart: unless-stopped
  tty: true
  stdin_open: true

services:
  px4-bridge-drone-1:
    <<: *px4-common
    container_name: px4-bridge-drone-1
    hostname: px4-bridge-drone-1
    environment:
      <<: *px4-env
      PX4_INSTANCE: 1
    ports:
      # Explicit protocol specifications are crucial
      - "4561:4561/tcp"      # HIL connection (TCP)
      - "14541:14541/udp"    # GCS local port (UDP)
      - "14581:14581/udp"    # GCS remote port (UDP)
    networks:
      px4_network:
        ipv4_address: 172.20.0.11

  px4-bridge-drone-2:
    <<: *px4-common
    container_name: px4-bridge-drone-2
    hostname: px4-bridge-drone-2
    environment:
      <<: *px4-env
      PX4_INSTANCE: 2
    ports:
      - "4562:4562/tcp"      # HIL connection (TCP)
      - "14542:14542/udp"    # GCS local port (UDP)
      - "14582:14582/udp"    # GCS remote port (UDP)
    networks:
      px4_network:
        ipv4_address: 172.20.0.12

  px4-bridge-drone-3:
    <<: *px4-common
    container_name: px4-bridge-drone-3
    hostname: px4-bridge-drone-3
    environment:
      <<: *px4-env
      PX4_INSTANCE: 3
    ports:
      - "4563:4563/tcp"      # HIL connection (TCP)
      - "14543:14543/udp"    # GCS local port (UDP)
      - "14583:14583/udp"    # GCS remote port (UDP)
    networks:
      px4_network:
        ipv4_address: 172.20.0.13
```

#### AirSim Settings Configuration

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "ApiServerEndpoint": "0.0.0.0:41451",
  "Vehicles": {
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlIp": "127.0.0.1",           // Use localhost for bridge network
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    },
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4562,
      "ControlIp": "127.0.0.1",           // Use localhost for bridge network
      "ControlPortLocal": 14542,
      "ControlPortRemote": 14582,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    },
    "PX4_Drone3": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4563,
      "ControlIp": "127.0.0.1",           // Use localhost for bridge network
      "ControlPortLocal": 14543,
      "ControlPortRemote": 14583,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    }
  }
}
```

### Bridge Network Advantages

1. **Network Isolation**: Containers are isolated from host network
2. **Security**: Controlled access through port mapping
3. **Scalability**: Easy to add more containers
4. **Flexibility**: Can connect to multiple networks
5. **Docker Desktop Compatibility**: Works with Enhanced Container Isolation

### Bridge Network Limitations

1. **Performance Overhead**: Additional network translation layer
2. **Complexity**: Requires port mapping configuration
3. **DNS Dependencies**: Relies on host.docker.internal resolution
4. **Protocol Specification**: Must specify TCP/UDP in port mapping

## Host Network Implementation

### Host Network Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                      Docker Host Network Architecture                          │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         Host Machine                                    │   │
│  │                        (Shared Network)                                 │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                      AirSim Process                             │   │   │
│  │  │                                                                 │   │   │
│  │  │  Bindings:                                                      │   │   │
│  │  │  • TCP 0.0.0.0:4561 (HIL)                                      │   │   │
│  │  │  • UDP 0.0.0.0:14541 (GCS Local)                               │   │   │
│  │  │  • UDP 0.0.0.0:14581 (GCS Remote)                              │   │   │
│  │  │  • TCP 0.0.0.0:41451 (API)                                     │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                   Docker Engine                                 │   │   │
│  │  │                                                                 │   │   │
│  │  │  Network Mode: host                                             │   │   │
│  │  │  Port Mapping: Not required                                     │   │   │
│  │  │  DNS Resolution: Host system                                    │   │   │
│  │  │  Network Namespace: Shared with host                            │   │   │
│  │  │                                                                 │   │   │
│  │  │  ┌─────────────────────────────────────────────────────────┐   │   │   │
│  │  │  │              PX4 Container                              │   │   │   │
│  │  │  │         (Uses Host Network)                             │   │   │   │
│  │  │  │                                                         │   │   │   │
│  │  │  │  Environment:                                           │   │   │   │
│  │  │  │  • PX4_SIM_HOSTNAME=127.0.0.1                          │   │   │   │
│  │  │  │  • MAV_0_REMOTE_HOST=127.0.0.1                         │   │   │   │
│  │  │  │  • MAV_0_REMOTE_PORT=4561                              │   │   │   │
│  │  │  │                                                         │   │   │   │
│  │  │  │  Connections:                                           │   │   │   │
│  │  │  │  • TCP connect to 127.0.0.1:4561                       │   │   │   │
│  │  │  │  • UDP connect to 127.0.0.1:14541                      │   │   │   │
│  │  │  │  • UDP connect to 127.0.0.1:14581                      │   │   │   │
│  │  │  └─────────────────────────────────────────────────────────┘   │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Host Network Configuration

#### Docker Compose Configuration

```yaml
# docker-compose-host.yml
version: '3.8'

x-px4-common: &px4-common
  build:
    context: .
    dockerfile: Dockerfile
  environment: &px4-env
    # Use localhost for host network mode
    PX4_SIM_HOSTNAME: 127.0.0.1
    PX4_SIM_MODEL: iris
    DELAYED_START: true
  network_mode: host              # Key configuration
  restart: unless-stopped
  tty: true
  stdin_open: true

services:
  px4-host-drone-1:
    <<: *px4-common
    container_name: px4-host-drone-1
    hostname: px4-host-drone-1
    environment:
      <<: *px4-env
      PX4_INSTANCE: 1
    # No port mapping needed with host network

  px4-host-drone-2:
    <<: *px4-common
    container_name: px4-host-drone-2
    hostname: px4-host-drone-2
    environment:
      <<: *px4-env
      PX4_INSTANCE: 2
    # No port mapping needed with host network

  px4-host-drone-3:
    <<: *px4-common
    container_name: px4-host-drone-3
    hostname: px4-host-drone-3
    environment:
      <<: *px4-env
      PX4_INSTANCE: 3
    # No port mapping needed with host network
```

#### AirSim Settings Configuration

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "ApiServerEndpoint": "0.0.0.0:41451",
  "Vehicles": {
    "PX4_Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlIp": "127.0.0.1",           // Use localhost for host network
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    },
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4562,
      "ControlIp": "127.0.0.1",           // Use localhost for host network
      "ControlPortLocal": 14542,
      "ControlPortRemote": 14582,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    },
    "PX4_Drone3": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4563,
      "ControlIp": "127.0.0.1",           // Use localhost for host network
      "ControlPortLocal": 14543,
      "ControlPortRemote": 14583,
      "LocalHostIp": "0.0.0.0",           // Bind to all interfaces
      "LockStep": true
    }
  }
}
```

### Host Network Advantages

1. **Performance**: No network translation overhead
2. **Simplicity**: No port mapping required
3. **Compatibility**: Works like native processes
4. **Low Latency**: Direct hardware access
5. **Debugging**: Easier to troubleshoot

### Host Network Limitations

1. **Security**: No network isolation
2. **Port Conflicts**: Shared port space with host
3. **Multiple Instances**: Harder to scale
4. **Docker Desktop**: Limited support on some platforms
5. **Resource Sharing**: Potential conflicts with host services

## Network Mode Comparison

### Feature Comparison

| Feature | Bridge Network | Host Network | Custom Network |
|---------|---------------|--------------|----------------|
| **Isolation** | High | None | Configurable |
| **Performance** | Moderate | High | Variable |
| **Setup Complexity** | Moderate | Low | High |
| **Port Mapping** | Required | Not needed | Optional |
| **DNS Resolution** | Custom | Host | Custom |
| **Security** | Good | Limited | Configurable |
| **Scalability** | Excellent | Limited | Excellent |
| **Docker Desktop** | Full support | Limited | Full support |

### Use Case Recommendations

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          Use Case Recommendations                               │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Development                                      │   │
│  │                                                                         │   │
│  │  Recommendation: Host Network                                           │   │
│  │  Rationale:                                                             │   │
│  │  • Simplest configuration                                               │   │
│  │  • Easiest debugging                                                    │   │
│  │  • Best performance                                                     │   │
│  │  • No port mapping complexity                                           │   │
│  │                                                                         │   │
│  │  docker-compose.yml:                                                    │   │
│  │  network_mode: host                                                     │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Production                                       │   │
│  │                                                                         │   │
│  │  Recommendation: Bridge Network                                         │   │
│  │  Rationale:                                                             │   │
│  │  • Better security isolation                                            │   │
│  │  • Controlled access via port mapping                                   │   │
│  │  • Scalable to multiple instances                                       │   │
│  │  • Works with Docker Desktop                                            │   │
│  │                                                                         │   │
│  │  docker-compose.yml:                                                    │   │
│  │  networks: [px4_network]                                                │   │
│  │  ports: ["4561:4561/tcp", "14541:14541/udp"]                          │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Multi-Vehicle                                    │   │
│  │                                                                         │   │
│  │  Recommendation: Bridge Network                                         │   │
│  │  Rationale:                                                             │   │
│  │  • Excellent scalability                                                │   │
│  │  • Port allocation management                                           │   │
│  │  • Container isolation                                                  │   │
│  │  • Resource management                                                  │   │
│  │                                                                         │   │
│  │  docker-compose.yml:                                                    │   │
│  │  services: [px4-drone-1, px4-drone-2, ...]                            │   │
│  │  networks: [px4_network]                                                │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        CI/CD Pipeline                                   │   │
│  │                                                                         │   │
│  │  Recommendation: Bridge Network                                         │   │
│  │  Rationale:                                                             │   │
│  │  • Predictable networking                                               │   │
│  │  • Automated testing support                                            │   │
│  │  • Container orchestration                                              │   │
│  │  • Environment consistency                                              │   │
│  │                                                                         │   │
│  │  docker-compose.yml:                                                    │   │
│  │  networks: [test_network]                                               │   │
│  │  depends_on: [airsim-service]                                           │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Container-to-Host Communication

### DNS Resolution Strategies

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                       Container-to-Host DNS Resolution                          │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                    host.docker.internal                                 │   │
│  │                                                                         │   │
│  │  Availability: Docker Desktop, Docker Engine 20.10+                    │   │
│  │  Resolution: Host machine IP                                            │   │
│  │  Platform: Windows, macOS, Linux                                       │   │
│  │  Reliability: High                                                      │   │
│  │                                                                         │   │
│  │  Usage:                                                                 │   │
│  │  export PX4_SIM_HOSTNAME=host.docker.internal                          │   │
│  │                                                                         │   │
│  │  Verification:                                                          │   │
│  │  docker exec <container> nslookup host.docker.internal                 │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Gateway IP Detection                                │   │
│  │                                                                         │   │
│  │  Availability: All Docker installations                                 │   │
│  │  Resolution: Bridge network gateway                                     │   │
│  │  Platform: Universal                                                    │   │
│  │  Reliability: Moderate                                                  │   │
│  │                                                                         │   │
│  │  Usage:                                                                 │   │
│  │  GATEWAY_IP=$(docker network inspect bridge --format='{{range .IPAM.Config}}{{.Gateway}}{{end}}')  │   │
│  │  export PX4_SIM_HOSTNAME=$GATEWAY_IP                                    │   │
│  │                                                                         │   │
│  │  Verification:                                                          │   │
│  │  docker exec <container> ip route | grep default                       │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Static IP Configuration                             │   │
│  │                                                                         │   │
│  │  Availability: Manual configuration                                     │   │
│  │  Resolution: Hardcoded IP address                                       │   │
│  │  Platform: Environment-specific                                         │   │
│  │  Reliability: Low (IP changes)                                          │   │
│  │                                                                         │   │
│  │  Usage:                                                                 │   │
│  │  export PX4_SIM_HOSTNAME=192.168.1.100                                 │   │
│  │                                                                         │   │
│  │  Verification:                                                          │   │
│  │  docker exec <container> ping 192.168.1.100                            │   │
│  │                                                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Dynamic IP Resolution

```bash
#!/bin/bash
# Dynamic IP resolution script

# Function to detect host IP from container
detect_host_ip() {
    local method=$1
    
    case $method in
        "docker_internal")
            echo "host.docker.internal"
            ;;
        "gateway")
            # Get gateway IP from default route
            ip route | grep default | awk '{print $3}'
            ;;
        "bridge_inspect")
            # Get bridge network gateway (requires docker command)
            docker network inspect bridge --format='{{range .IPAM.Config}}{{.Gateway}}{{end}}'
            ;;
        "host_networking")
            echo "127.0.0.1"
            ;;
        *)
            echo "Unknown method: $method"
            return 1
            ;;
    esac
}

# Auto-detect best method
auto_detect_host_ip() {
    # Try host.docker.internal first
    if nslookup host.docker.internal > /dev/null 2>&1; then
        echo "host.docker.internal"
        return 0
    fi
    
    # Try gateway method
    local gateway_ip=$(detect_host_ip "gateway")
    if [ -n "$gateway_ip" ] && ping -c 1 "$gateway_ip" > /dev/null 2>&1; then
        echo "$gateway_ip"
        return 0
    fi
    
    # Fallback to localhost
    echo "127.0.0.1"
    return 0
}

# Usage
HOST_IP=$(auto_detect_host_ip)
export PX4_SIM_HOSTNAME=$HOST_IP
echo "Using host IP: $HOST_IP"
```

## Port Mapping Strategies

### Protocol-Specific Port Mapping

```yaml
# Correct port mapping with protocol specifications
services:
  px4-drone:
    ports:
      # TCP ports - reliable, connection-oriented
      - "4561:4561/tcp"        # HIL sensor data
      - "41451:41451/tcp"      # AirSim API (if needed)
      
      # UDP ports - low latency, connectionless
      - "14541:14541/udp"      # GCS local control
      - "14581:14581/udp"      # GCS remote control
      - "14550:14550/udp"      # QGroundControl (optional)
      
      # Common mistakes to avoid:
      # - "4561:4561"           # No protocol specified
      # - "14541:14541/tcp"     # Wrong protocol for UDP
      # - "4561/tcp:4561/tcp"   # Invalid format
```

### Port Allocation Strategy

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        Port Allocation Strategy                                 │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      Base Port Ranges                                   │   │
│  │                                                                         │   │
│  │  AirSim API: 41451                                                      │   │
│  │  HIL TCP: 4560-4569 (10 vehicles)                                       │   │
│  │  GCS UDP Local: 14540-14549 (10 vehicles)                               │   │
│  │  GCS UDP Remote: 14580-14589 (10 vehicles)                              │   │
│  │  QGroundControl: 14550-14559 (10 vehicles)                              │   │
│  │                                                                         │   │
│  │  Calculation:                                                           │   │
│  │  Vehicle N:                                                             │   │
│  │  • HIL TCP: 4560 + N                                                    │   │
│  │  • GCS Local: 14540 + N                                                 │   │
│  │  • GCS Remote: 14580 + N                                                │   │
│  │  • QGC: 14550 + N - 1                                                   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      Port Mapping Template                              │   │
│  │                                                                         │   │
│  │  px4-drone-${N}:                                                        │   │
│  │    ports:                                                               │   │
│  │      - "${HIL_PORT}:${HIL_PORT}/tcp"                                    │   │
│  │      - "${GCS_LOCAL_PORT}:${GCS_LOCAL_PORT}/udp"                       │   │
│  │      - "${GCS_REMOTE_PORT}:${GCS_REMOTE_PORT}/udp"                     │   │
│  │    environment:                                                         │   │
│  │      PX4_INSTANCE: ${N}                                                 │   │
│  │    networks:                                                            │   │
│  │      px4_network:                                                       │   │
│  │        ipv4_address: 172.20.0.${10+N}                                   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                      Automated Generation                               │   │
│  │                                                                         │   │
│  │  #!/bin/bash                                                            │   │
│  │  # Generate port mappings for N vehicles                                │   │
│  │                                                                         │   │
│  │  generate_port_mapping() {                                              │   │
│  │      local vehicle_count=$1                                             │   │
│  │      for i in $(seq 1 $vehicle_count); do                              │   │
│  │          local hil_port=$((4560 + i))                                   │   │
│  │          local gcs_local_port=$((14540 + i))                            │   │
│  │          local gcs_remote_port=$((14580 + i))                           │   │
│  │          echo "  px4-drone-$i:"                                         │   │
│  │          echo "    ports:"                                              │   │
│  │          echo "      - \"$hil_port:$hil_port/tcp\""                     │   │
│  │          echo "      - \"$gcs_local_port:$gcs_local_port/udp\""         │   │
│  │          echo "      - \"$gcs_remote_port:$gcs_remote_port/udp\""       │   │
│  │      done                                                               │   │
│  │  }                                                                      │   │
│  │                                                                         │   │
│  │  # Usage: generate_port_mapping 5                                       │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Multi-Vehicle Scaling

### Scaling Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                       Multi-Vehicle Scaling Architecture                       │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         Host Machine                                    │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                      AirSim Process                             │   │   │
│  │  │                                                                 │   │   │
│  │  │  Multiple Vehicle Support:                                      │   │   │
│  │  │  • TCP servers: 4561-4569                                       │   │   │
│  │  │  • UDP servers: 14541-14549, 14581-14589                       │   │   │
│  │  │  • API endpoint: 41451                                          │   │   │
│  │  │  • Resource management: Per-vehicle threads                     │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                   Docker Engine                                 │   │   │
│  │  │                                                                 │   │   │
│  │  │  Container Orchestration:                                       │   │   │
│  │  │  • Service scaling: 1-N containers                              │   │   │
│  │  │  • Resource limits: CPU, memory per container                   │   │   │
│  │  │  • Network management: Bridge network                           │   │   │
│  │  │  • Port allocation: Automatic mapping                           │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                   Bridge Network                                │   │   │
│  │  │                  (px4_network)                                  │   │   │
│  │  │                                                                 │   │   │
│  │  │  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │   │   │
│  │  │  │ PX4 Container 1 │ │ PX4 Container 2 │ │ PX4 Container N │   │   │   │
│  │  │  │ IP: 172.20.0.11 │ │ IP: 172.20.0.12 │ │ IP: 172.20.0.1N │   │   │   │
│  │  │  │ Instance: 1     │ │ Instance: 2     │ │ Instance: N     │   │   │   │
│  │  │  │ HIL: 4561       │ │ HIL: 4562       │ │ HIL: 456N       │   │   │   │
│  │  │  │ GCS: 14541/81   │ │ GCS: 14542/82   │ │ GCS: 1454N/8N   │   │   │   │
│  │  │  └─────────────────┘ └─────────────────┘ └─────────────────┘   │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Scalable Docker Compose

```yaml
# docker-compose-scalable.yml
version: '3.8'

networks:
  px4_network:
    driver: bridge
    ipam:
      driver: default
      config:
        - subnet: 172.20.0.0/16

x-px4-template: &px4-template
  build:
    context: .
    dockerfile: Dockerfile
  environment:
    PX4_SIM_HOSTNAME: host.docker.internal
    PX4_SIM_MODEL: iris
    DELAYED_START: true
  networks:
    - px4_network
  restart: unless-stopped
  tty: true
  stdin_open: true

services:
  # Generate multiple services using environment variables
  px4-drone-1:
    <<: *px4-template
    container_name: px4-drone-1
    environment:
      PX4_INSTANCE: 1
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "4561:4561/tcp"
      - "14541:14541/udp"
      - "14581:14581/udp"
    networks:
      px4_network:
        ipv4_address: 172.20.0.11

  px4-drone-2:
    <<: *px4-template
    container_name: px4-drone-2
    environment:
      PX4_INSTANCE: 2
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "4562:4562/tcp"
      - "14542:14542/udp"
      - "14582:14582/udp"
    networks:
      px4_network:
        ipv4_address: 172.20.0.12

  px4-drone-3:
    <<: *px4-template
    container_name: px4-drone-3
    environment:
      PX4_INSTANCE: 3
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "4563:4563/tcp"
      - "14543:14543/udp"
      - "14583:14583/udp"
    networks:
      px4_network:
        ipv4_address: 172.20.0.13

  # Add more services as needed...
```

### Dynamic Scaling Script

```bash
#!/bin/bash
# Dynamic scaling script for AirSim-PX4 containers

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose-scalable.yml"

# Function to generate docker-compose service
generate_service() {
    local instance=$1
    local hil_port=$((4560 + instance))
    local gcs_local_port=$((14540 + instance))
    local gcs_remote_port=$((14580 + instance))
    local ip_address="172.20.0.$((10 + instance))"
    
    cat << EOF
  px4-drone-$instance:
    <<: *px4-template
    container_name: px4-drone-$instance
    environment:
      PX4_INSTANCE: $instance
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "$hil_port:$hil_port/tcp"
      - "$gcs_local_port:$gcs_local_port/udp"
      - "$gcs_remote_port:$gcs_remote_port/udp"
    networks:
      px4_network:
        ipv4_address: $ip_address

EOF
}

# Function to generate AirSim settings
generate_airsim_settings() {
    local vehicle_count=$1
    local settings_file="$SCRIPT_DIR/settings-$vehicle_count-vehicles.json"
    
    cat << EOF > "$settings_file"
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "ApiServerEndpoint": "0.0.0.0:41451",
  "Vehicles": {
EOF

    for i in $(seq 1 $vehicle_count); do
        local hil_port=$((4560 + i))
        local gcs_local_port=$((14540 + i))
        local gcs_remote_port=$((14580 + i))
        local comma=""
        if [ $i -lt $vehicle_count ]; then
            comma=","
        fi
        
        cat << EOF >> "$settings_file"
    "PX4_Drone$i": {
      "VehicleType": "PX4Multirotor",
      "X": $((i * 5)),
      "Y": 0,
      "Z": -2,
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": $hil_port,
      "ControlIp": "127.0.0.1",
      "ControlPortLocal": $gcs_local_port,
      "ControlPortRemote": $gcs_remote_port,
      "LocalHostIp": "0.0.0.0",
      "LockStep": true
    }$comma
EOF
    done
    
    cat << EOF >> "$settings_file"
  }
}
EOF
    
    echo "Generated AirSim settings: $settings_file"
}

# Function to generate docker-compose file
generate_compose_file() {
    local vehicle_count=$1
    local output_file="$SCRIPT_DIR/docker-compose-$vehicle_count-vehicles.yml"
    
    cat << 'EOF' > "$output_file"
version: '3.8'

networks:
  px4_network:
    driver: bridge
    ipam:
      driver: default
      config:
        - subnet: 172.20.0.0/16

x-px4-template: &px4-template
  build:
    context: .
    dockerfile: Dockerfile
  environment:
    PX4_SIM_HOSTNAME: host.docker.internal
    PX4_SIM_MODEL: iris
    DELAYED_START: true
  networks:
    - px4_network
  restart: unless-stopped
  tty: true
  stdin_open: true

services:
EOF
    
    for i in $(seq 1 $vehicle_count); do
        generate_service $i >> "$output_file"
    done
    
    echo "Generated docker-compose file: $output_file"
}

# Function to scale deployment
scale_deployment() {
    local vehicle_count=$1
    
    if [ -z "$vehicle_count" ] || [ "$vehicle_count" -lt 1 ] || [ "$vehicle_count" -gt 10 ]; then
        echo "Error: Vehicle count must be between 1 and 10"
        return 1
    fi
    
    echo "Scaling deployment to $vehicle_count vehicles..."
    
    # Generate configuration files
    generate_compose_file $vehicle_count
    generate_airsim_settings $vehicle_count
    
    local compose_file="$SCRIPT_DIR/docker-compose-$vehicle_count-vehicles.yml"
    local settings_file="$SCRIPT_DIR/settings-$vehicle_count-vehicles.json"
    
    echo "Files generated:"
    echo "  Docker Compose: $compose_file"
    echo "  AirSim Settings: $settings_file"
    echo ""
    echo "To start deployment:"
    echo "  docker-compose -f $compose_file up -d"
    echo ""
    echo "To copy settings to AirSim:"
    echo "  cp $settings_file ~/Documents/AirSim/settings.json"
}

# Main script
case "$1" in
    "scale")
        scale_deployment "$2"
        ;;
    "start")
        local vehicle_count=${2:-1}
        local compose_file="$SCRIPT_DIR/docker-compose-$vehicle_count-vehicles.yml"
        if [ -f "$compose_file" ]; then
            docker-compose -f "$compose_file" up -d
        else
            echo "Error: Compose file not found. Run: $0 scale $vehicle_count"
            exit 1
        fi
        ;;
    "stop")
        local vehicle_count=${2:-1}
        local compose_file="$SCRIPT_DIR/docker-compose-$vehicle_count-vehicles.yml"
        if [ -f "$compose_file" ]; then
            docker-compose -f "$compose_file" down
        else
            echo "Error: Compose file not found."
            exit 1
        fi
        ;;
    "status")
        docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep px4-drone
        ;;
    *)
        echo "Usage: $0 {scale|start|stop|status} [vehicle_count]"
        echo ""
        echo "Commands:"
        echo "  scale N    - Generate configuration for N vehicles"
        echo "  start N    - Start N vehicle containers"
        echo "  stop N     - Stop N vehicle containers"
        echo "  status     - Show container status"
        echo ""
        echo "Examples:"
        echo "  $0 scale 5     # Generate config for 5 vehicles"
        echo "  $0 start 5     # Start 5 vehicle containers"
        echo "  $0 stop 5      # Stop 5 vehicle containers"
        echo "  $0 status      # Show container status"
        exit 1
        ;;
esac
```

## Troubleshooting Docker Networks

### Common Issues and Solutions

#### Issue 0: Network Namespace Mismatch in WSL2/Docker

**⚠️ CRITICAL**: This is the most common configuration error that causes partial connectivity failures.

**Symptoms:**
- HIL connection (TCP) works - PX4 shows "Simulator connected"
- GCS connection (UDP) fails - No telemetry in QGroundControl
- Only message ID 92 (HIL_SENSOR) appears in logs

**Root Cause:**
Network namespace separation causes `"127.0.0.1"` to refer to different loopback interfaces in different environments.

**Diagnosis:**
```bash
# Check if HIL is working but GCS is not
docker exec px4-drone-1 netstat -an | grep 4561    # Should show ESTABLISHED
docker exec px4-drone-1 netstat -an | grep 14541   # May show no connection

# Test UDP connectivity from container
docker exec px4-drone-1 nc -zvu host.docker.internal 14541
# vs
docker exec px4-drone-1 nc -zvu 127.0.0.1 14541
```

**Solutions by Environment:**

```json
// WSL2 Environment (Windows AirSim + WSL2 PX4)
{
  "LocalHostIp": "172.28.240.1",    // Windows host IP accessible from WSL2
  "ControlIp": "remote"             // Dynamic IP detection from HIL connection
}

// Docker Bridge Network (Host AirSim + Container PX4)
{
  "LocalHostIp": "0.0.0.0",         // Bind to all interfaces
  "ControlIp": "127.0.0.1"          // Use localhost via port mapping
}

// Docker Host Network (Same network namespace)
{
  "LocalHostIp": "0.0.0.0",         // Bind to all interfaces
  "ControlIp": "127.0.0.1"          // True localhost access
}
```

**Why This Matters:**
- `"remote"` triggers dynamic IP detection using the HIL connection's source IP
- `"127.0.0.1"` sends packets to the current process's localhost, which may be unreachable
- Network namespace isolation prevents cross-boundary localhost communication

#### Issue 1: Container Cannot Reach Host

```bash
# Symptoms
docker exec px4-drone-1 ping host.docker.internal
# ping: host.docker.internal: Name or service not known

# Diagnosis
docker exec px4-drone-1 cat /etc/resolv.conf
docker exec px4-drone-1 nslookup host.docker.internal

# Solutions
# 1. Update Docker Desktop
# 2. Use gateway IP
docker exec px4-drone-1 ip route | grep default
# 3. Add host entry
docker exec px4-drone-1 echo "172.17.0.1 host.docker.internal" >> /etc/hosts
```

#### Issue 2: Port Mapping Not Working

```bash
# Symptoms
docker exec px4-drone-1 nc -zv host.docker.internal 4561
# Connection refused

# Diagnosis
docker port px4-drone-1
netstat -an | grep 4561
docker-compose config | grep ports

# Solutions
# 1. Check protocol specification
ports:
  - "4561:4561/tcp"  # Correct
  - "4561:4561"      # Incorrect (no protocol)

# 2. Check AirSim binding
# AirSim should bind to 0.0.0.0:4561, not 127.0.0.1:4561

# 3. Check firewall
sudo ufw status
sudo ufw allow 4561/tcp
```

#### Issue 3: UDP Traffic Not Flowing

```bash
# Symptoms
# TCP HIL connection works, UDP GCS connection fails

# Diagnosis
docker exec px4-drone-1 nc -zvu host.docker.internal 14541
tcpdump -i any port 14541

# Solutions
# 1. Explicit UDP port mapping
ports:
  - "14541:14541/udp"  # Correct
  - "14541:14541/tcp"  # Incorrect protocol

# 2. Check AirSim ControlIP setting
{
  "ControlIp": "127.0.0.1"  // For bridge network
}

# 3. Test UDP connectivity
echo "test" | nc -u host.docker.internal 14541
```

#### Issue 4: Multiple Containers Port Conflicts

```bash
# Symptoms
# Second container fails to start with port already in use

# Diagnosis
docker-compose logs px4-drone-2
# Error: bind: address already in use

# Solutions
# 1. Check port allocation
docker-compose config | grep ports
# Each container must use unique ports

# 2. Verify instance configuration
# PX4_INSTANCE should be unique for each container

# 3. Check for running containers
docker ps | grep px4-drone
docker stop $(docker ps -q --filter="name=px4-drone")
```

### Diagnostic Commands

```bash
# Network connectivity testing
network_test() {
    local container_name=$1
    local host_ip=${2:-host.docker.internal}
    
    echo "Testing network connectivity for $container_name"
    
    # Test DNS resolution
    echo "=== DNS Resolution ==="
    docker exec $container_name nslookup $host_ip
    
    # Test TCP HIL connection
    echo "=== TCP HIL Connection ==="
    docker exec $container_name nc -zv $host_ip 4561
    
    # Test UDP GCS connection
    echo "=== UDP GCS Connection ==="
    docker exec $container_name nc -zvu $host_ip 14541
    
    # Test ping
    echo "=== Ping Test ==="
    docker exec $container_name ping -c 3 $host_ip
    
    # Show network configuration
    echo "=== Network Configuration ==="
    docker exec $container_name ip route
    docker exec $container_name ip addr show
}

# Port mapping verification
port_mapping_test() {
    local container_name=$1
    
    echo "Testing port mapping for $container_name"
    
    # Show Docker port mapping
    echo "=== Docker Port Mapping ==="
    docker port $container_name
    
    # Show host port bindings
    echo "=== Host Port Bindings ==="
    netstat -an | grep -E "(4561|14541|14581)"
    
    # Test from host
    echo "=== Host to Container Test ==="
    nc -zv 127.0.0.1 4561
    nc -zvu 127.0.0.1 14541
}

# Container health check
container_health_check() {
    local container_name=$1
    
    echo "Health check for $container_name"
    
    # Container status
    echo "=== Container Status ==="
    docker ps --filter="name=$container_name" --format="table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    
    # Resource usage
    echo "=== Resource Usage ==="
    docker stats $container_name --no-stream
    
    # Logs
    echo "=== Recent Logs ==="
    docker logs --tail 20 $container_name
    
    # Process list
    echo "=== Process List ==="
    docker exec $container_name ps aux
}

# Usage
# network_test px4-drone-1
# port_mapping_test px4-drone-1
# container_health_check px4-drone-1
```

## Performance Optimization

### Resource Limits

```yaml
# Resource-limited configuration
services:
  px4-drone-1:
    <<: *px4-template
    container_name: px4-drone-1
    # Resource limits
    deploy:
      resources:
        limits:
          cpus: '0.5'        # 50% of one CPU core
          memory: 512M       # 512MB memory limit
        reservations:
          cpus: '0.25'       # 25% guaranteed CPU
          memory: 256M       # 256MB guaranteed memory
    # Alternative syntax (docker-compose v2)
    mem_limit: 512m
    cpus: 0.5
    # Network performance
    sysctls:
      - net.core.rmem_max=16777216
      - net.core.wmem_max=16777216
    # Kernel parameters
    ulimits:
      nofile:
        soft: 65536
        hard: 65536
```

### Network Optimization

```yaml
# Network-optimized configuration
networks:
  px4_network:
    driver: bridge
    driver_opts:
      com.docker.network.bridge.name: px4-bridge
      com.docker.network.bridge.enable_icc: "true"
      com.docker.network.bridge.enable_ip_masquerade: "true"
      com.docker.network.bridge.host_binding_ipv4: "0.0.0.0"
      com.docker.network.mtu: 1500
    ipam:
      driver: default
      config:
        - subnet: 172.20.0.0/16
          gateway: 172.20.0.1
          ip_range: 172.20.0.0/24
```

### Container Optimization

```dockerfile
# Optimized Dockerfile for PX4 containers
FROM ubuntu:22.04

# Optimize for container
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install dependencies with cache optimization
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Pre-compile PX4 to reduce startup time
RUN git clone https://github.com/PX4/PX4-Autopilot.git --depth 1 --branch v1.14.0 && \
    cd PX4-Autopilot && \
    make px4_sitl_default && \
    make clean_context

# Optimize for network performance
RUN echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf && \
    echo 'net.core.wmem_max = 16777216' >> /etc/sysctl.conf && \
    echo 'net.ipv4.tcp_rmem = 4096 87380 16777216' >> /etc/sysctl.conf && \
    echo 'net.ipv4.tcp_wmem = 4096 65536 16777216' >> /etc/sysctl.conf

# Set working directory
WORKDIR /px4_workspace/PX4-Autopilot

# Start with optimized settings
CMD ["./build/px4_sitl_default/bin/px4", "-d", "-s", "etc/init.d-posix/rcS"]
```

## Security Considerations

### Network Security

```yaml
# Security-focused configuration
services:
  px4-drone-1:
    <<: *px4-template
    container_name: px4-drone-1
    # Security options
    security_opt:
      - no-new-privileges:true
      - apparmor:unconfined
    # Read-only filesystem
    read_only: true
    tmpfs:
      - /tmp:rw,exec,nosuid,size=1g
    # User namespace
    user: "1000:1000"
    # Capabilities
    cap_drop:
      - ALL
    cap_add:
      - NET_BIND_SERVICE
    # Network security
    ports:
      - "127.0.0.1:4561:4561/tcp"    # Bind to localhost only
      - "127.0.0.1:14541:14541/udp"  # Bind to localhost only
```

### Firewall Configuration

```bash
# Host firewall rules
#!/bin/bash
# Configure firewall for AirSim-PX4 containers

# Allow AirSim API
sudo ufw allow from 172.20.0.0/16 to any port 41451

# Allow HIL connections
sudo ufw allow from 172.20.0.0/16 to any port 4561:4569

# Allow GCS connections
sudo ufw allow from 172.20.0.0/16 to any port 14541:14549
sudo ufw allow from 172.20.0.0/16 to any port 14581:14589

# Block external access
sudo ufw deny from any to any port 4561:4569
sudo ufw deny from any to any port 14541:14549
sudo ufw deny from any to any port 14581:14589

# Enable firewall
sudo ufw enable
```

### Container Security

```yaml
# Security-hardened docker-compose
version: '3.8'

secrets:
  px4_config:
    file: ./px4_config.txt

services:
  px4-drone-1:
    <<: *px4-template
    container_name: px4-drone-1
    
    # Security configurations
    security_opt:
      - no-new-privileges:true
      - seccomp:unconfined
    
    # Resource limits
    deploy:
      resources:
        limits:
          cpus: '0.5'
          memory: 512M
          pids: 200
    
    # Network security
    ports:
      - "127.0.0.1:4561:4561/tcp"
      - "127.0.0.1:14541:14541/udp"
    
    # Filesystem security
    read_only: true
    tmpfs:
      - /tmp:rw,exec,nosuid,size=100m
      - /var/run:rw,exec,nosuid,size=10m
    
    # Secrets management
    secrets:
      - px4_config
    
    # Environment restrictions
    environment:
      - PX4_INSTANCE=1
      - PX4_SIM_HOSTNAME=host.docker.internal
      # Remove sensitive environment variables
```

## Quick Reference: Configuration by Environment

### WSL2 + Windows AirSim

```json
{
  "LocalHostIp": "172.28.240.1",
  "ControlIp": "remote",
  "UseTcp": true,
  "TcpPort": 4561,
  "ControlPortLocal": 14541,
  "ControlPortRemote": 14581
}
```

**Environment Setup:**
```bash
# In WSL2
export PX4_SIM_HOSTNAME=172.28.240.1
```

### Docker Bridge Network

```json
{
  "LocalHostIp": "0.0.0.0",
  "ControlIp": "127.0.0.1",
  "UseTcp": true,
  "TcpPort": 4561,
  "ControlPortLocal": 14541,
  "ControlPortRemote": 14581
}
```

**Environment Setup:**
```bash
# In Docker container
export PX4_SIM_HOSTNAME=host.docker.internal
```

**Docker Compose:**
```yaml
ports:
  - "4561:4561/tcp"
  - "14541:14541/udp"
  - "14581:14581/udp"
```

### Docker Host Network

```json
{
  "LocalHostIp": "0.0.0.0",
  "ControlIp": "127.0.0.1",
  "UseTcp": true,
  "TcpPort": 4561,
  "ControlPortLocal": 14541,
  "ControlPortRemote": 14581
}
```

**Environment Setup:**
```bash
# In Docker container
export PX4_SIM_HOSTNAME=127.0.0.1
```

**Docker Compose:**
```yaml
network_mode: host
```

### Native Linux

```json
{
  "LocalHostIp": "0.0.0.0",
  "ControlIp": "127.0.0.1",
  "UseTcp": true,
  "TcpPort": 4561,
  "ControlPortLocal": 14541,
  "ControlPortRemote": 14581
}
```

**Environment Setup:**
```bash
# Native Linux
export PX4_SIM_HOSTNAME=127.0.0.1
```

## Conclusion

This comprehensive Docker networking guide provides:

1. **Multiple Network Strategies**: Bridge and host network implementations with trade-offs
2. **Scalable Solutions**: Multi-vehicle support with automated configuration generation
3. **Troubleshooting Framework**: Systematic diagnosis and resolution of network issues
4. **Performance Optimization**: Resource management and network tuning
5. **Security Considerations**: Network isolation and access control

Key takeaways:
- **Bridge networks** provide better isolation and security but require careful port mapping
- **Host networks** offer better performance but with security trade-offs
- **Protocol specifications** are crucial for proper TCP/UDP port mapping
- **Container orchestration** enables scalable multi-vehicle deployments
- **Monitoring and diagnostics** are essential for troubleshooting network issues

The choice between bridge and host networking depends on your specific requirements for security, performance, and scalability. For development environments, host networking provides simplicity and performance. For production deployments, bridge networking offers better isolation and security.

Understanding these Docker networking patterns is essential for successful containerized AirSim-PX4 deployments across diverse environments and scales.