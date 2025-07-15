# MAVLink Networking Troubleshooting Guide

This guide helps diagnose and fix MAVLink connectivity issues between PX4 SITL and AirSim across different environments (WSL2, Docker, native Linux).

## Quick Diagnosis

### 1. Test Your Environment
```bash
# Check your environment and get recommendations
cd docker_clean/config_generator/tools
python3 wsl2_detector.py --status

# Run comprehensive network tests
python3 network_tester.py
```

### 2. Quick Connectivity Test
```bash
# Test if AirSim is reachable
nc -zv <airsim_host> 41451

# Test if PX4 TCP ports are available
nc -zv <airsim_host> 4561
```

## Environment-Specific Solutions

### WSL2 Environment (Working Configuration)

**Why it works:**
- WSL2 has direct access to Windows host network
- IP `172.28.240.1` is the Windows host as seen from WSL2
- No network isolation between WSL2 and Windows

**Configuration:**
```bash
# In WSL2 terminal
export PX4_SIM_HOSTNAME=172.28.240.1
cd PX4-Autopilot/Scripts/
./run_airsim_sitl_working.sh 1
```

**Verification:**
```bash
# Check Windows IP from WSL2
ip route | grep default
# Should show: default via 172.28.240.1 dev eth0

# Test AirSim connectivity
nc -zv 172.28.240.1 41451
```

### Docker Environment (Fixed)

**Why it was failing:**
1. **Network Isolation**: Bridge networks can't reach host IP `172.28.240.1`
2. **Conflicting Configuration**: Mixed bridge and host network modes
3. **Port Binding Issues**: PX4 TCP ports not properly exposed
4. **Protocol Mismatch**: UDP ports mapped as TCP or without protocol specification

**Fixed Configuration:**

#### 1. Bridge Network Solution (Recommended)
```yaml
# docker-compose-bridge.yml
networks:
  px4_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

services:
  px4-bridge-drone-1:
    image: px4-ultra-swarm:latest
    environment:
      PX4_SIM_HOSTNAME: host.docker.internal
    ports:
      - "4561:4561/tcp"      # HIL connection - explicit TCP
      - "14541:14541/udp"    # GCS local - explicit UDP
      - "14581:14581/udp"    # GCS remote - explicit UDP
    networks:
      px4_network:
        ipv4_address: 172.20.0.11
```

#### 2. AirSim Settings for Bridge Network
```json
{
  "Vehicles": {
    "PX4_Drone1": {
      "ControlIp": "127.0.0.1",          // Use localhost for port mapping
      "LocalHostIp": "0.0.0.0",          // Bind to all interfaces
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581
    }
  }
}
```

#### 3. Host Network Alternative
```yaml
# docker-compose-host.yml
services:
  px4-host-drone-1:
    image: px4-ultra-swarm:latest
    network_mode: host
    environment:
      PX4_SIM_HOSTNAME: 127.0.0.1
```

#### 4. Auto-Detection Script
```bash
# Updated launcher automatically detects and configures IPs
./tools/start-ultra-swarm-wsl.sh single
```

#### 5. Manual Docker Run
```bash
# For bridge network testing
docker run --rm -it --network px4_network \
  -p 4561:4561/tcp -p 14541:14541/udp -p 14581:14581/udp \
  -e PX4_SIM_HOSTNAME=host.docker.internal \
  px4-ultra-swarm:latest

# For host network testing
docker run --rm -it --network host \
  -e PX4_SIM_HOSTNAME=127.0.0.1 \
  px4-ultra-swarm:latest
```

### Native Linux Environment

**Configuration:**
```bash
export PX4_SIM_HOSTNAME=localhost
# or if AirSim runs on different machine:
export PX4_SIM_HOSTNAME=<airsim_machine_ip>
```

## Common Issues and Solutions

### Issue 1: "Connection refused" to AirSim API

**Symptoms:**
```
❌ AirSim API not reachable at 172.28.240.1:41451
```

**Solutions:**
1. **Ensure AirSim is running**
   ```bash
   # Windows: Start AirSim (Blocks.exe or similar)
   # Check if API is listening
   netstat -an | grep 41451
   ```

2. **Configure AirSim API endpoint**
   ```json
   // settings.json
   {
     "ApiServerEndpoint": "0.0.0.0:41451"
   }
   ```

3. **Windows Firewall**
   ```powershell
   # Run as Administrator
   New-NetFirewallRule -DisplayName "AirSim API" -Direction Inbound -Port 41451 -Protocol TCP -Action Allow
   New-NetFirewallRule -DisplayName "PX4 TCP" -Direction Inbound -Port 4560-4569 -Protocol TCP -Action Allow
   ```

### Issue 2: Docker containers can't reach Windows host

**Symptoms:**
```
📦 Running in Docker container
⚠️ AirSim API not reachable at 172.28.240.1:41451
💡 Check Docker network mode (should be 'host' for direct access or bridge with port mapping)
```

**Solutions:**
1. **Use Bridge Network with Port Mapping (Recommended)**
   ```yaml
   # docker-compose-bridge.yml
   networks:
     px4_network:
       driver: bridge
       ipam:
         config:
           - subnet: 172.20.0.0/16
   
   services:
     px4-container:
       environment:
         PX4_SIM_HOSTNAME: host.docker.internal
       ports:
         - "4561:4561/tcp"
         - "14541:14541/udp"
         - "14581:14581/udp"
       networks:
         - px4_network
   ```

2. **Update AirSim Settings for Bridge Network**
   ```json
   {
     "Vehicles": {
       "PX4_Drone1": {
         "ControlIp": "127.0.0.1",
         "LocalHostIp": "0.0.0.0"
       }
     }
   }
   ```

3. **Alternative: Use Host Network Mode**
   ```yaml
   # docker-compose-host.yml
   services:
     px4-container:
       network_mode: host
       environment:
         PX4_SIM_HOSTNAME: 127.0.0.1
   ```

4. **Update Environment Variables**
   ```bash
   # Auto-update all configurations
   cd docker_clean/config_generator/tools
   python3 wsl2_detector.py --auto-update /path/to/docker/directory
   ```

### Issue 3: Port conflicts

**Symptoms:**
```
⚠️ TCP port 4561 already in use
```

**Solutions:**
1. **Stop conflicting services**
   ```bash
   # Find what's using the port
   sudo netstat -tulpn | grep 4561
   
   # Stop other PX4 instances
   pkill -f px4
   ```

2. **Use different instance numbers**
   ```bash
   # Start with different instance ID
   ./run_airsim_sitl_final.sh 2  # Uses ports 4562, 14551, etc.
   ```

### Issue 4: MAVLink connection drops

**Symptoms:**
- PX4 starts but shows "waiting for simulator"
- No MAVLink traffic in QGroundControl
- Only HIL_SENSOR messages (ID 92) in logs
- Missing HEARTBEAT and telemetry messages

**Solutions:**
1. **Check PX4 configuration**
   ```bash
   # In PX4 console
   param show MAV_0_CONFIG
   param show MAV_0_MODE
   
   # Should be:
   # MAV_0_CONFIG: 101 (TCP client)
   # MAV_0_MODE: 0 (Normal)
   ```

2. **Verify AirSim vehicle configuration**
   ```json
   // settings.json
   {
     "Vehicles": {
       "PX4_Drone1": {
         "VehicleType": "PX4Multirotor",
         "UseSerial": false,
         "UseTcp": true,
         "TcpPort": 4561,
         "ControlIp": "127.0.0.1",          // For bridge network
         "ControlPortLocal": 14541,
         "ControlPortRemote": 14581,
         "LocalHostIp": "0.0.0.0"
       }
     }
   }
   ```

3. **Check dual connection status**
   ```bash
   # Test HIL connection (TCP)
   nc -zv 127.0.0.1 4561
   
   # Test GCS connection (UDP)
   nc -zvu 127.0.0.1 14541
   nc -zvu 127.0.0.1 14581
   
   # Monitor both connections
   tcpdump -i any port 4561 or port 14541 or port 14581
   ```

4. **Verify Docker port mapping**
   ```bash
   # Check port mapping includes UDP
   docker port px4-bridge-drone-1
   # Should show:
   # 4561/tcp -> 0.0.0.0:4561
   # 14541/udp -> 0.0.0.0:14541
   # 14581/udp -> 0.0.0.0:14581
   ```

## Diagnostic Tools

### 1. Environment Detector
```bash
cd docker_clean/config_generator/tools
python3 wsl2_detector.py --status
```

**Output:**
```
🔍 Environment Detection
========================================
WSL2 Environment: ✓ Yes
Docker Container: ✗ No
Windows Host IP: 172.28.240.1
WSL2 IP: 172.28.240.100

🔗 Connectivity Tests (Target: 172.28.240.1)
--------------------------------------------------
AirSim API (41451): ✓
TCP 4561: ✓
TCP 4562: ✓
TCP 4563: ✓
```

### 2. Network Connectivity Tester
```bash
python3 network_tester.py

# Test specific host and port
python3 network_tester.py --hosts 172.28.240.1 --port 41451

# Quick test
python3 network_tester.py --quick
```

### 3. Docker Network Validation
```bash
# Check if container can reach host
docker exec -it px4-container nc -zv 172.28.240.1 41451

# Check network mode
docker inspect px4-container | grep NetworkMode
```

## Step-by-Step Troubleshooting

### Step 1: Verify Environment
```bash
# 1. Check if you're in WSL2, Docker, or native Linux
python3 wsl2_detector.py --get-environment

# 2. Get recommended settings
python3 wsl2_detector.py --get-settings
```

### Step 2: Test Basic Connectivity
```bash
# 3. Test if target host is reachable
ping 172.28.240.1

# 4. Test AirSim API
nc -zv 172.28.240.1 41451
```

### Step 3: Check AirSim Configuration
```bash
# 5. Verify AirSim is listening on all interfaces
netstat -an | grep 41451
# Should show: 0.0.0.0:41451 (not 127.0.0.1:41451)
```

### Step 4: Test Docker Network Mode
```bash
# 6. If using Docker, ensure host networking
docker-compose config | grep network_mode
# Should show: network_mode: host
```

### Step 5: Update Configuration
```bash
# 7. Auto-update all configurations
python3 wsl2_detector.py --auto-update /path/to/project
```

### Step 6: Test MAVLink Connection
```bash
# 8. Start PX4 and check for "Simulator connected"
./run_airsim_sitl_final.sh 1
# Should show: [init] Simulator connected
```

## Port Reference

### AirSim Ports
- **41451**: API Server (TCP)
- **4560+**: PX4 TCP connection (per vehicle)

### PX4 Ports  
- **4560+**: TCP to AirSim (per instance)
- **14550+**: MAVLink UDP to QGC (per instance)
- **18570+**: Internal MAVLink UDP (per instance)

### Port Calculation
```bash
# Instance 1: TCP 4561, QGC 14550, MAVLink 18570
# Instance 2: TCP 4562, QGC 14551, MAVLink 18571
# Instance N: TCP 456N, QGC 1454(N-1), MAVLink 1857(N-1)
```

## Configuration Files

### Updated WSL2 Launcher
`docker/px4_airsim_docker_v2/tools/start-ultra-swarm-wsl.sh`
- Auto-detects Windows host IP
- Updates docker-compose with correct hostname
- Validates connectivity before starting containers

### Enhanced Environment Detector  
`docker_clean/config_generator/tools/wsl2_detector.py`
- Detects WSL2, Docker, and native Linux environments
- Provides environment-specific recommendations
- Auto-updates configuration files

### Fixed Docker Compose
`docker/px4_airsim_docker_v2/docker-compose.ultra-swarm.yml`
- Uses host networking mode consistently
- Supports environment variable override for hostname
- Removed conflicting bridge network definitions

## Success Criteria

Your setup is working correctly when you see:

1. **Environment Detection**: ✓ Correct environment identified
2. **Network Connectivity**: ✓ AirSim API reachable
3. **PX4 Startup**: ✓ "Simulator connected" message
4. **HIL Connection**: ✓ TCP connection established on port 4561
5. **GCS Connection**: ✓ UDP connections established on ports 14541/14581
6. **MAVLink Traffic**: ✓ Both HIL_SENSOR and HEARTBEAT messages flowing
7. **Dual Connection Health**: ✓ Both connections active and healthy
8. **API Response**: ✓ Python client can connect to AirSim

```bash
# Final verification script
#!/bin/bash
echo "🔍 Comprehensive AirSim-PX4 Connection Test"
echo "=========================================="

# Test AirSim API
echo "1. Testing AirSim API connection..."
if nc -zv 127.0.0.1 41451 2>/dev/null; then
    echo "✅ AirSim API: CONNECTED"
else
    echo "❌ AirSim API: FAILED"
fi

# Test HIL connection
echo "2. Testing HIL connection (TCP)..."
if nc -zv 127.0.0.1 4561 2>/dev/null; then
    echo "✅ HIL TCP: CONNECTED"
else
    echo "❌ HIL TCP: FAILED"
fi

# Test GCS connections
echo "3. Testing GCS connections (UDP)..."
if nc -zvu 127.0.0.1 14541 2>/dev/null; then
    echo "✅ GCS Local UDP: CONNECTED"
else
    echo "❌ GCS Local UDP: FAILED"
fi

if nc -zvu 127.0.0.1 14581 2>/dev/null; then
    echo "✅ GCS Remote UDP: CONNECTED"
else
    echo "❌ GCS Remote UDP: FAILED"
fi

# Test Python client
echo "4. Testing Python client connection..."
python3 -c "
import airsim
try:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    state = client.getMultirotorState()
    print('✅ Python client: CONNECTED')
    print(f'   Vehicle armed: {state.armed}')
    print(f'   Vehicle position: {state.kinematics_estimated.position}')
except Exception as e:
    print(f'❌ Python client: FAILED - {e}')
"

echo ""
echo "🎉 Connection test complete!"
echo "If all tests pass, your AirSim-PX4 setup is working correctly."
```