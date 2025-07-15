# AirSim-PX4 Network Architecture and Communication Flow

## Overview

This document provides a comprehensive analysis of the network architecture and communication flow between AirSim and PX4 SITL (Software-In-The-Loop) simulation. Understanding this architecture is crucial for troubleshooting connectivity issues, optimizing performance, and deploying in different environments (WSL2, Docker, native Linux).

## Table of Contents

1. [Dual Connection Architecture](#dual-connection-architecture)
2. [Network Flow Analysis](#network-flow-analysis)
3. [Environment-Specific Behavior](#environment-specific-behavior)
4. [Docker Network Solutions](#docker-network-solutions)
5. [ControlIP Resolution Logic](#controlip-resolution-logic)
6. [Port Mapping and Protocol Specifications](#port-mapping-and-protocol-specifications)
7. [Troubleshooting Network Issues](#troubleshooting-network-issues)
8. [Performance Considerations](#performance-considerations)

## Dual Connection Architecture

### Overview of Dual Connections

AirSim-PX4 integration uses **two separate network connections** for different purposes:

1. **HIL Connection (TCP)**: Hardware-In-The-Loop sensor data
2. **GCS Connection (UDP)**: Ground Control Station commands and telemetry

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                AirSim                                          │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                    MavLinkMultirotorApi                                 │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────┐  ┌─────────────────────────────────┐  │   │
│  │  │      HIL Connection         │  │      GCS Connection             │  │   │
│  │  │         (TCP)               │  │         (UDP)                   │  │   │
│  │  │                             │  │                                 │  │   │
│  │  │  • Sensor data (HIL_SENSOR) │  │  • Control commands             │  │   │
│  │  │  • GPS data                 │  │  • Telemetry requests           │  │   │
│  │  │  • IMU data                 │  │  • Parameter changes            │  │   │
│  │  │  • Magnetometer data        │  │  • Mission commands             │  │   │
│  │  │  • Barometer data           │  │  • Mode changes                 │  │   │
│  │  └─────────────────────────────┘  └─────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
                           │                           │
                           │ TCP (4561)                │ UDP (14541->14581)
                           │                           │
                           ▼                           ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                 PX4 SITL                                        │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                         MAVLink Router                                  │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────┐  ┌─────────────────────────────────┐  │   │
│  │  │     HIL Channel (TCP)       │  │     GCS Channel (UDP)           │  │   │
│  │  │       Port 4561             │  │   Ports 14541/14581             │  │   │
│  │  │                             │  │                                 │  │   │
│  │  │  • Receives sensor data     │  │  • Receives control commands    │  │   │
│  │  │  • Provides to flight stack│  │  • Sends telemetry data         │  │   │
│  │  │  • No control commands      │  │  • Handles parameter requests   │  │   │
│  │  │  • Read-only for HIL        │  │  • Bidirectional communication  │  │   │
│  │  └─────────────────────────────┘  └─────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Why Two Connections?

The dual connection architecture exists because:

1. **Protocol Limitations**: PX4 SITL HIL mode separates sensor data from control commands
2. **Performance**: TCP for reliable sensor data, UDP for low-latency control
3. **Scalability**: Different connections can be optimized independently
4. **Robustness**: If one connection fails, the other can continue operating

### Connection Initialization Sequence

```cpp
// From MavLinkMultirotorApi.hpp lines 1372-1467
void MavLinkMultirotorApi::connectToServerViaHil() {
    // Step 1: Initialize HIL Connection (TCP)
    if (connection_info.use_tcp) {
        connection_ = std::make_shared<mavlinkcom::MavLinkConnection>();
        remoteIpAddr = connection_->acceptTcp("hil", 
                                            connection_info_.local_host_ip, 
                                            connection_info.tcp_port);
    }
    
    // Step 2: Initialize GCS Connection (UDP) 
    if (connection_info_.control_ip_address != "") {
        if (remoteIpAddr == "127.0.0.1") {
            gcsConnection = mavlinkcom::MavLinkConnection::connectLocalUdp("gcs",
                                                                         connection_info_.local_host_ip,
                                                                         connection_info_.control_port_local);
        } else {
            gcsConnection = mavlinkcom::MavLinkConnection::connectRemoteUdp("gcs",
                                                                          connection_info_.local_host_ip,
                                                                          remoteIpAddr,
                                                                          connection_info_.control_port_remote);
        }
    }
}
```

## Network Flow Analysis

### WSL2 Environment (Working Configuration)

```
┌─────────────────┐    ┌─────────────────────────────────────────────────────────┐
│   Windows Host  │    │                    WSL2                                 │
│                 │    │                                                         │
│   ┌─────────────┴────┴─────────────────────────────────────────────────────┐   │
│   │                        AirSim                                          │   │
│   │                  IP: 172.28.240.1                                      │   │
│   │                 Port: 41451 (API)                                      │   │
│   │                                                                         │   │
│   │  TCP Server: 4561 ←─── HIL Connection ←─── PX4 SITL                   │   │
│   │  UDP Server: 14541 ←── GCS Connection ←─── PX4 SITL                   │   │
│   └─────────────────────────────────────────────────────────────────────────┘   │
│                 │                                                               │
│                 │ Windows Host IP: 172.28.240.1                                │
│                 │ (Accessible from WSL2)                                       │
│                 │                                                               │
│                 │    ┌─────────────────────────────────────────────────────┐   │
│                 │    │                 PX4 SITL                            │   │
│                 │    │            IP: 172.28.240.100                       │   │
│                 │    │                                                     │   │
│                 └────┤  TCP Client: 4561 ──► HIL Connection               │   │
│                      │  UDP Client: 14541 ──► GCS Connection              │   │
│                      │                                                     │   │
│                      │  Environment: PX4_SIM_HOSTNAME=172.28.240.1        │   │
│                      └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

**Why WSL2 Works:**
- WSL2 has direct access to Windows host network via IP `172.28.240.1`
- Both TCP and UDP connections can reach the same target host
- No network isolation between WSL2 and Windows host
- `PX4_SIM_HOSTNAME=172.28.240.1` works for both connections

### Docker Bridge Network (Fixed Configuration)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                               Host Machine                                      │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        AirSim                                          │   │
│  │                  IP: 127.0.0.1                                         │   │
│  │                 Port: 41451 (API)                                      │   │
│  │                                                                         │   │
│  │  TCP Server: 4561 ←─── Port Mapping ←─── Docker Bridge                │   │
│  │  UDP Server: 14541 ←── Port Mapping ←─── Docker Bridge                │   │
│  │  UDP Server: 14581 ←── Port Mapping ←─── Docker Bridge                │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                 │                                                               │
│                 │ Docker Port Mapping                                          │
│                 │ host.docker.internal → 127.0.0.1                            │
│                 │                                                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                    Docker Bridge Network                                │   │
│  │                      (172.20.0.0/16)                                   │   │
│  │                                                                         │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                 PX4 SITL Container                              │   │   │
│  │  │              IP: 172.20.0.11                                    │   │   │
│  │  │                                                                 │   │   │
│  │  │  TCP Client: 4561/tcp ──► host.docker.internal:4561           │   │   │
│  │  │  UDP Client: 14541/udp ──► host.docker.internal:14541         │   │   │
│  │  │  UDP Client: 14581/udp ──► host.docker.internal:14581         │   │   │
│  │  │                                                                 │   │   │
│  │  │  Environment: PX4_SIM_HOSTNAME=host.docker.internal            │   │   │
│  │  │  AirSim ControlIP: "127.0.0.1"                                 │   │   │
│  │  └─────────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

**Bridge Network Solution:**
- **Port Mapping**: Explicit TCP/UDP protocol specifications
- **DNS Resolution**: `host.docker.internal` resolves to host machine
- **ControlIP**: Set to `127.0.0.1` for localhost port mapping
- **Protocol Separation**: TCP for HIL, UDP for GCS connections

### Docker Host Network (Alternative Configuration)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                               Host Machine                                      │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        AirSim                                          │   │
│  │                  IP: 127.0.0.1                                         │   │
│  │                 Port: 41451 (API)                                      │   │
│  │                                                                         │   │
│  │  TCP Server: 4561 ←─── Direct Access ←─── Docker Host Network         │   │
│  │  UDP Server: 14541 ←── Direct Access ←─── Docker Host Network         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                 │                                                               │
│                 │ Shared Network Namespace                                     │
│                 │ (No network isolation)                                       │
│                 │                                                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                 PX4 SITL Container                                      │   │
│  │              (Host Network Mode)                                        │   │
│  │                                                                         │   │
│  │  TCP Client: 4561 ──► 127.0.0.1:4561                                  │   │
│  │  UDP Client: 14541 ──► 127.0.0.1:14541                                │   │
│  │                                                                         │   │
│  │  Environment: PX4_SIM_HOSTNAME=127.0.0.1                               │   │
│  │  AirSim ControlIP: "127.0.0.1"                                         │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

**Host Network Benefits:**
- **Direct Access**: No port mapping required
- **Performance**: Lower latency, no network translation
- **Simplicity**: Same configuration as native Linux
- **Compatibility**: Works with Docker Desktop limitations

## Environment-Specific Behavior

### WSL2 Environment

**Network Characteristics:**
- Windows host accessible via `172.28.240.1`
- WSL2 instance gets IP in `172.28.240.0/20` range
- Both TCP and UDP can reach Windows host
- No firewall restrictions between WSL2 and Windows

**Configuration:**
```bash
# PX4 SITL
export PX4_SIM_HOSTNAME=172.28.240.1

# AirSim settings.json
{
  "Vehicles": {
    "PX4_Drone1": {
      "ControlIp": "remote",  // Resolves to connecting IP
      "LocalHostIp": "0.0.0.0",
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581
    }
  }
}
```

### Docker Bridge Network

**Network Characteristics:**
- Containers get IPs in custom bridge network (e.g., `172.20.0.0/16`)
- Host accessible via `host.docker.internal`
- Port mapping required for host access
- Protocol-specific port mapping needed

**Configuration:**
```yaml
# docker-compose-bridge.yml
services:
  px4-bridge-drone-1:
    environment:
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "4561:4561/tcp"      # HIL connection
      - "14541:14541/udp"    # GCS local port
      - "14581:14581/udp"    # GCS remote port
    networks:
      px4_network:
        ipv4_address: 172.20.0.11
```

```json
// AirSim settings.json
{
  "Vehicles": {
    "PX4_Drone1": {
      "ControlIp": "127.0.0.1",  // Use localhost for port mapping
      "LocalHostIp": "0.0.0.0",
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581
    }
  }
}
```

### Docker Host Network

**Network Characteristics:**
- Container shares host network namespace
- No port mapping required
- Same network behavior as native Linux
- May conflict with existing services

**Configuration:**
```yaml
# docker-compose-host.yml
services:
  px4-host-drone-1:
    network_mode: host
    environment:
      PX4_SIM_HOSTNAME: 127.0.0.1
```

```json
// AirSim settings.json
{
  "Vehicles": {
    "PX4_Drone1": {
      "ControlIp": "127.0.0.1",
      "LocalHostIp": "0.0.0.0",
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581
    }
  }
}
```

## Docker Network Solutions

### Bridge Network Implementation

The bridge network solution addresses Docker Desktop limitations while maintaining network isolation:

#### Port Mapping Strategy

```yaml
# Explicit protocol specifications
ports:
  - "4561:4561/tcp"      # HIL connection (TCP)
  - "14541:14541/udp"    # Control port local (UDP)
  - "14581:14581/udp"    # Control port remote (UDP)
```

#### Network Configuration

```yaml
networks:
  px4_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
```

#### Container Configuration

```yaml
services:
  px4-bridge-drone-1:
    environment:
      PX4_SIM_HOSTNAME: host.docker.internal
    networks:
      px4_network:
        ipv4_address: 172.20.0.11
```

### Host Network Alternative

For environments where bridge networking doesn't work:

```yaml
services:
  px4-host-drone-1:
    network_mode: host
    environment:
      PX4_SIM_HOSTNAME: 127.0.0.1
```

**Trade-offs:**
- **Bridge Network**: Better isolation, more complex configuration
- **Host Network**: Simpler configuration, potential port conflicts

## ControlIP Resolution Logic

### AirSim ControlIP Resolution

The `ControlIp` setting in AirSim determines how the GCS connection is established:

```cpp
// From MavLinkMultirotorApi.hpp lines 1411-1444
if (connection_info_.control_ip_address != "") {
    if (!connection_info.use_tcp || connection_info_.control_ip_address != "remote") {
        remoteIpAddr = connection_info_.control_ip_address;
    }
    if (remoteIpAddr == "local" || remoteIpAddr == "localhost") {
        remoteIpAddr = "127.0.0.1";
    }
    
    if (remoteIpAddr == "127.0.0.1") {
        gcsConnection = mavlinkcom::MavLinkConnection::connectLocalUdp("gcs",
                                                                     connection_info_.local_host_ip,
                                                                     connection_info_.control_port_local);
    } else {
        gcsConnection = mavlinkcom::MavLinkConnection::connectRemoteUdp("gcs",
                                                                      connection_info_.local_host_ip,
                                                                      remoteIpAddr,
                                                                      connection_info_.control_port_remote);
    }
}
```

### ControlIP Values and Behavior

| ControlIP Value | Connection Type | Behavior |
|----------------|----------------|----------|
| `"remote"` | Remote UDP | Uses IP from TCP connection (HIL) |
| `"127.0.0.1"` | Local UDP | Binds to localhost, uses local port |
| `"localhost"` | Local UDP | Same as `"127.0.0.1"` |
| `"192.168.1.100"` | Remote UDP | Connects to specific IP |

### Critical Network Namespace Considerations

**⚠️ IMPORTANT**: The `ControlIp` setting is highly environment-sensitive due to network namespace separation:

- **WSL2 Environment**: `"127.0.0.1"` refers to the Windows host's localhost, NOT the WSL2 instance's localhost
- **Docker Containers**: `"127.0.0.1"` refers to the container's localhost, NOT the host's localhost
- **Native Linux**: `"127.0.0.1"` refers to the same machine's localhost

**Common Failure Pattern**: Setting `ControlIp: "127.0.0.1"` in WSL2 causes GCS UDP packets to be sent to Windows localhost instead of the PX4 instance's actual IP in the WSL2 network, breaking the control connection while HIL continues to work.

### Environment-Specific ControlIP Settings

| Environment | ControlIP | LocalHostIp | Reasoning |
|-------------|-----------|-------------|-----------|
| WSL2 | `"remote"` | `"172.28.240.1"` | PX4 connects from WSL2 IP, dynamic IP detection |
| Docker Bridge | `"127.0.0.1"` | `"0.0.0.0"` | Use port mapping via localhost |
| Docker Host | `"127.0.0.1"` | `"0.0.0.0"` | Direct localhost access |
| Native Linux | `"127.0.0.1"` | `"0.0.0.0"` | Same machine communication |

### Why "remote" Works in WSL2

The `"remote"` setting triggers **dynamic IP detection**:

1. **HIL Connection**: PX4 (in WSL2) connects to AirSim (on Windows) via TCP
2. **IP Extraction**: AirSim extracts the source IP from the incoming HIL connection (e.g., `172.28.240.100`)
3. **GCS Connection**: AirSim uses this extracted IP for the UDP GCS connection
4. **Success**: Both connections use the correct network path

### Why "127.0.0.1" Fails in WSL2

The `"127.0.0.1"` setting causes **network namespace mismatch**:

1. **HIL Connection**: Successfully established (TCP server on `172.28.240.1`)
2. **GCS Connection**: AirSim sends UDP packets to Windows localhost (`127.0.0.1`)
3. **Network Isolation**: WSL2 runs in separate network namespace
4. **Failure**: UDP packets never reach PX4 in WSL2 network

## Port Mapping and Protocol Specifications

### Standard Port Allocation

```
Instance 1: TCP 4561, UDP 14541/14581
Instance 2: TCP 4562, UDP 14542/14582
Instance 3: TCP 4563, UDP 14543/14583
...
Instance N: TCP 456N, UDP 1454N/1458N
```

### Docker Bridge Port Mapping

```yaml
# Complete port mapping for 3 drones
services:
  px4-bridge-drone-1:
    ports:
      - "4561:4561/tcp"      # HIL connection
      - "14541:14541/udp"    # Control local
      - "14581:14581/udp"    # Control remote
      
  px4-bridge-drone-2:
    ports:
      - "4562:4562/tcp"      # HIL connection
      - "14542:14542/udp"    # Control local
      - "14582:14582/udp"    # Control remote
      
  px4-bridge-drone-3:
    ports:
      - "4563:4563/tcp"      # HIL connection
      - "14543:14543/udp"    # Control local
      - "14583:14583/udp"    # Control remote
```

### Protocol-Specific Considerations

**TCP (HIL Connection):**
- Reliable, ordered delivery
- Connection-oriented
- Higher latency but guaranteed delivery
- Used for sensor data streams

**UDP (GCS Connection):**
- Unreliable, unordered delivery
- Connectionless
- Lower latency
- Used for real-time control commands

## Troubleshooting Network Issues

### Common Issues and Solutions

#### Issue 1: Only HIL Connection Working

**Symptoms:**
- PX4 shows "Simulator connected"
- Only message ID 92 (HIL_SENSOR) in logs
- No telemetry in QGroundControl

**Root Cause:**
- TCP HIL connection works
- UDP GCS connection fails

**Solution:**
```bash
# Check UDP port mapping
docker-compose -f docker-compose-bridge.yml config | grep -A 20 ports

# Test UDP connectivity
nc -u 127.0.0.1 14541

# Fix port mapping
ports:
  - "14541:14541/udp"  # Add /udp specification
```

#### Issue 2: Docker Containers Can't Reach AirSim

**Symptoms:**
- "Connection refused" errors
- AirSim API not reachable

**Root Cause:**
- Network isolation in bridge mode
- Incorrect hostname resolution

**Solution:**
```bash
# Update PX4_SIM_HOSTNAME
export PX4_SIM_HOSTNAME=host.docker.internal

# Or use host networking
network_mode: host
```

#### Issue 3: ControlIP Resolution Failures

**Symptoms:**
- HIL connection works
- GCS connection times out

**Root Cause:**
- Incorrect ControlIP setting
- Network topology mismatch

**Solution:**
```json
// For Docker bridge network
{
  "ControlIp": "127.0.0.1"  // Use localhost
}

// For WSL2
{
  "ControlIp": "remote"     // Use remote IP detection
}
```

### Diagnostic Commands

```bash
# Test network connectivity
./test_bridge_network.sh

# Check port mapping
docker port px4-bridge-drone-1

# Test TCP connection
nc -zv 127.0.0.1 4561

# Test UDP connection
nc -zvu 127.0.0.1 14541

# Monitor network traffic
tcpdump -i any port 4561 or port 14541
```

## Performance Considerations

### Network Latency

**Bridge Network:**
- Additional layer of network translation
- ~1-2ms additional latency
- Good for development/testing

**Host Network:**
- Direct host network access
- Minimal additional latency
- Better for performance-critical applications

### Connection Reliability

**TCP HIL Connection:**
- Guaranteed delivery
- Automatic retransmission
- Flow control

**UDP GCS Connection:**
- Best-effort delivery
- No retransmission
- Requires application-level reliability

### Scaling Considerations

**Bridge Network:**
- Better resource isolation
- Supports multiple instances easily
- Network bandwidth sharing

**Host Network:**
- Shared port space
- Potential conflicts
- Direct hardware access

## Conclusion

The AirSim-PX4 dual connection architecture provides:

1. **Separation of Concerns**: HIL sensor data vs GCS control commands
2. **Protocol Optimization**: TCP for reliability, UDP for performance
3. **Environment Flexibility**: Works across WSL2, Docker, and native Linux
4. **Scalability**: Multiple instances with port allocation
5. **Robustness**: Independent connection failure handling

Understanding this architecture is essential for:
- Troubleshooting connectivity issues
- Optimizing network performance
- Deploying in containerized environments
- Scaling to multiple vehicles
- Implementing custom networking solutions

The bridge network solution represents a significant advancement in Docker-based AirSim deployments, providing both network isolation and full connectivity functionality across all supported platforms.