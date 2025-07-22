# QGroundControl Remote Connection Setup

## Overview
This guide explains how to connect QGroundControl to your remote PX4 server in various deployment scenarios.

## Connection Methods

### 1. Direct Internet Connection (Basic)
**Use case**: Development, testing, no VPN setup
**Security**: Firewall-based protection only

#### Configuration
1. Open QGroundControl
2. Go to **Application Settings** → **Comm Links**
3. Click **Add** to create new connection
4. Configure connection:
   ```
   Name: PX4 Remote Server
   Type: UDP
   Automatically Connect on Start: ✓
   High Latency: ✓ (for internet connections)
   
   Listening Port: 14550
   Target Hosts: [YOUR_SERVER_IP]
   Port: 14550
   ```

#### Server IP Examples
```bash
# AWS EC2
YOUR_SERVER_IP=ec2-xxx-xxx-xxx-xxx.compute-1.amazonaws.com

# GCP Compute Engine  
YOUR_SERVER_IP=xxx.xxx.xxx.xxx

# Azure VM
YOUR_SERVER_IP=px4-server.eastus.cloudapp.azure.com

# VPS (DigitalOcean, Linode, etc.)
YOUR_SERVER_IP=xxx.xxx.xxx.xxx
```

### 2. VPN Connection (Recommended)
**Use case**: Production, secure access
**Security**: Encrypted VPN tunnel + optional authentication

#### Prerequisites
1. VPN client configured and connected (see CLIENT_CONNECTION_GUIDE.md)
2. VPN tunnel active (WireGuard status: Active)

#### Configuration
1. Verify VPN connection:
   ```bash
   ping 10.10.0.1  # VPN gateway
   ping 10.10.0.11 # PX4 drone 1
   ```

2. Configure QGroundControl:
   ```
   Name: PX4 VPN Server
   Type: UDP
   Automatically Connect on Start: ✓
   High Latency: ✗ (VPN provides low latency)
   
   Listening Port: 14550
   Target Hosts: 10.10.0.11
   Port: 14550
   ```

#### Multi-Drone VPN Access
For multiple drones, create separate connections:
```
Drone 1: 10.10.0.11:14550
Drone 2: 10.10.0.12:14551  
Drone 3: 10.10.0.13:14552
```

### 3. Authenticated Connection (Enterprise)
**Use case**: Enterprise deployments with user management
**Security**: VPN + HTTP authentication + SSL

#### Setup Process
1. Connect to VPN (as above)
2. Authenticate via web interface:
   ```bash
   curl -u username:password https://10.10.0.1:8443/api/auth
   ```
3. Use returned token for API access
4. Configure QGroundControl with authenticated endpoint

## Connection Troubleshooting

### Common Connection Issues

#### 1. "Vehicle did not respond to request for autopilot version"
**Causes**: Network connectivity, firewall, wrong IP/port

**Solutions**:
```bash
# Test basic connectivity
ping [SERVER_IP]

# Test UDP port access
nc -u [SERVER_IP] 14550

# Check firewall on server
sudo ufw status
sudo iptables -L | grep 14550
```

#### 2. "No Heartbeat" or Intermittent Connection  
**Causes**: High latency, packet loss, NAT issues

**Solutions**:
- Enable "High Latency" mode in QGC
- Increase connection timeout: **Vehicle Setup** → **Parameters** → **COM_HEARTBEAT_TO**
- Check network quality:
  ```bash
  ping -c 10 [SERVER_IP]
  traceroute [SERVER_IP]
  ```

#### 3. "Connection Refused"
**Causes**: Server not running, wrong port, firewall blocking

**Solutions**:
```bash
# Check server status
docker ps | grep px4

# Check port mappings
docker port px4-bridge-drone-1

# Test from server localhost
docker exec px4-bridge-drone-1 netstat -tulpn | grep 14550
```

#### 4. VPN Connection Issues
**Causes**: VPN config, firewall, wrong network settings

**Solutions**:
```bash
# Check VPN status
wg show  # Linux/macOS
# Or check WireGuard app status

# Test VPN connectivity
ping 10.10.0.1
ping 10.10.0.11

# Verify VPN routing
ip route | grep 10.10.0  # Linux
route -n get 10.10.0.1   # macOS
```

## Advanced Configuration

### Custom Message Rates
For remote connections, optimize message rates:

1. Go to **Vehicle Setup** → **Parameters**
2. Adjust message rates:
   ```
   COM_OBS_AVOID = 0      # Disable obstacle avoidance telemetry
   SDLOG_MODE = 1         # Reduce logging
   SYS_LOGGER = 0         # Disable detailed logging
   
   # Reduce high-frequency messages
   SENS_FLOW_RATE = 5     # Optical flow (default: 10)
   EKF2_AID_MASK = 1      # GPS only (disable vision/flow)
   ```

### Connection Profiles
Create different profiles for different scenarios:

#### Low Bandwidth Profile
```
Name: PX4 Remote (Low Bandwidth)
High Latency: ✓
Target Hosts: [SERVER_IP]
Port: 14550

# In Parameters:
MAV_USEHILGPS = 1
COM_RC_IN_MODE = 1
```

#### High Performance Profile
```
Name: PX4 Remote (High Performance)  
High Latency: ✗
Target Hosts: 10.10.0.11  # VPN only
Port: 14550

# In Parameters:
MAV_USEHILGPS = 0
COM_RC_IN_MODE = 0
```

### Multiple QGroundControl Instances
Run multiple QGC instances for different drones:

```bash
# Linux/macOS
./QGroundControl --instance 1 &
./QGroundControl --instance 2 &

# Windows
QGroundControl.exe --instance 1
QGroundControl.exe --instance 2
```

Each instance should connect to different drone IPs:
- Instance 1 → 10.10.0.11:14550 (Drone 1)
- Instance 2 → 10.10.0.12:14551 (Drone 2)

## Security Best Practices

### 1. Network Security
- Always use VPN for internet connections
- Avoid direct exposure of MAVLink ports
- Implement firewall rules on server
- Monitor connection logs

### 2. QGroundControl Security
- Keep QGC updated to latest version
- Use authenticated connections when available
- Avoid storing passwords in QGC settings
- Use secure networks for control

### 3. Connection Monitoring
Enable logging in QGroundControl:
1. **Application Settings** → **General**
2. **Save application data to logs directory**: ✓
3. Monitor logs in QGC logs directory

## Performance Optimization

### Network Optimization
```bash
# On server, optimize network stack
echo 'net.core.rmem_max = 134217728' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' >> /etc/sysctl.conf
sysctl -p
```

### QGroundControl Optimization
1. **Application Settings** → **General**
2. **UI Scaling**: Set appropriate for your display
3. **Video**: Disable if not needed for remote connections
4. **Audio**: Disable audio notifications for remote use

## Example Connection Scripts

### Automated Connection Test
```bash
#!/bin/bash
# test-qgc-connection.sh

SERVER_IP="10.10.0.11"  # Or your server IP
PORT="14550"

echo "Testing QGroundControl connection to $SERVER_IP:$PORT"

# Test basic connectivity
if ping -c 3 $SERVER_IP >/dev/null 2>&1; then
    echo "✓ Server reachable via ping"
else
    echo "✗ Server not reachable"
    exit 1
fi

# Test UDP port
if nc -u -z -w 3 $SERVER_IP $PORT >/dev/null 2>&1; then
    echo "✓ MAVLink port $PORT accessible"
else
    echo "✗ MAVLink port $PORT not accessible"
    exit 1
fi

# Test MAVLink heartbeat
echo "Testing MAVLink heartbeat..."
timeout 10 nc -u $SERVER_IP $PORT < /dev/null
echo "✓ Connection test completed"
```

### VPN Connection Validator
```bash
#!/bin/bash
# validate-vpn-qgc.sh

VPN_GATEWAY="10.10.0.1"
DRONE_IP="10.10.0.11"

# Check VPN status
if ! ip route | grep -q "$VPN_GATEWAY"; then
    echo "✗ VPN not connected. Please activate WireGuard."
    exit 1
fi

echo "✓ VPN connected"

# Test drone connectivity
if ping -c 2 $DRONE_IP >/dev/null 2>&1; then
    echo "✓ Drone reachable via VPN"
    echo "Ready for QGroundControl connection to $DRONE_IP:14550"
else
    echo "✗ Drone not reachable via VPN"
    exit 1
fi
```

Make scripts executable:
```bash
chmod +x test-qgc-connection.sh
chmod +x validate-vpn-qgc.sh
```

---

For additional help, see:
- [PX4 Remote Connectivity Architecture](../docs/PX4_REMOTE_CONNECTIVITY_ARCHITECTURE.md)
- [Client Connection Guide](../security/CLIENT_CONNECTION_GUIDE.md)
- [Troubleshooting Tools](../tools/README_TESTING_TOOLS.md)