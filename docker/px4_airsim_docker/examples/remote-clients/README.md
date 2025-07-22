# Remote Client Connection Examples

This directory contains comprehensive examples for connecting various clients to your remote PX4 server.

## Available Examples

### 1. QGroundControl Setup (`qgroundcontrol-setup.md`)
**Complete guide for QGroundControl remote connections**

**Features**:
- Direct internet connection setup
- VPN-secured connections  
- Multi-drone configurations
- Troubleshooting guides
- Performance optimization

**Use cases**:
- Manual drone control
- Mission planning
- Real-time monitoring
- Parameter configuration

### 2. MAVSDK Python Example (`mavsdk-python-example.py`)
**Production-ready MAVSDK client with multiple connection methods**

**Features**:
- Automatic connection handling
- System information retrieval
- Autonomous takeoff/landing
- Mission execution
- Telemetry monitoring

**Connection methods**:
```bash
# VPN connection (recommended)
python mavsdk-python-example.py --method vpn --demo info

# Direct internet connection
python mavsdk-python-example.py --method direct --server-ip YOUR_IP --demo takeoff

# Custom connection string
python mavsdk-python-example.py --connection udp://10.10.0.11:14540 --demo mission
```

### 3. Custom UDP Client (`custom-udp-client.py`)
**Low-level MAVLink communication example**

**Features**:
- Raw MAVLink message handling
- Custom protocol implementation
- Message parsing and generation
- Connection statistics
- Heartbeat management

**Use cases**:
- Custom MAVLink applications
- Protocol debugging
- Performance testing
- Educational purposes

## Quick Start

### Prerequisites
```bash
# For MAVSDK example
pip install mavsdk

# For basic examples (included in Python)
# socket, struct, time modules
```

### Test Your Connection
```bash
# 1. Test basic connectivity
ping YOUR_SERVER_IP

# 2. Test UDP port access
nc -u YOUR_SERVER_IP 14540

# 3. Run simple connection test
cd examples/remote-clients
python custom-udp-client.py --server-ip YOUR_SERVER_IP --demo connect
```

## Connection Scenarios

### Scenario 1: Local Development
**Setup**: Docker containers on localhost
**Security**: Local network only
**Examples**:
```bash
# QGroundControl: localhost:14550
# MAVSDK
python mavsdk-python-example.py --method local --demo info

# Custom UDP
python custom-udp-client.py --server-ip localhost --demo monitor
```

### Scenario 2: VPN Remote Access (Recommended)
**Setup**: WireGuard VPN tunnel
**Security**: Encrypted tunnel
**Examples**:
```bash
# Prerequisites: VPN connected
wg show  # Verify VPN status

# QGroundControl: 10.10.0.11:14550  
# MAVSDK
python mavsdk-python-example.py --method vpn --demo takeoff

# Custom UDP  
python custom-udp-client.py --server-ip 10.10.0.11 --demo version
```

### Scenario 3: Direct Internet Access
**Setup**: Public IP with firewall rules
**Security**: Firewall-based protection
**Examples**:
```bash
# QGroundControl: YOUR_SERVER_IP:14550
# MAVSDK
python mavsdk-python-example.py --method direct --server-ip YOUR_SERVER_IP --demo mission

# Custom UDP
python custom-udp-client.py --server-ip YOUR_SERVER_IP --demo params
```

### Scenario 4: Multi-Drone Access
**Setup**: Multiple PX4 instances
**Security**: Per-drone isolation
**Examples**:
```bash
# Drone 1: 10.10.0.11:14540
python mavsdk-python-example.py --connection udp://10.10.0.11:14540 --demo info

# Drone 2: 10.10.0.12:14541  
python mavsdk-python-example.py --connection udp://10.10.0.12:14541 --demo info

# Drone 3: 10.10.0.13:14542
python mavsdk-python-example.py --connection udp://10.10.0.13:14542 --demo info
```

## Protocol Reference

### MAVLink Port Mapping
```
Standard QGroundControl: 14550
API Access (MAVSDK):     14540
Per-drone QGC:          14541-14549
AirSim HIL:             4561-4569 (TCP)
```

### Message Flow
```
Client ──UDP──→ Server:14540 ──MAVLink Router──→ PX4 Container
       ←──UDP──         ←─────────────────────
```

### Common Message IDs
```python
HEARTBEAT = 0           # Connection keepalive
SYS_STATUS = 1          # System health
ATTITUDE = 30           # Vehicle orientation
GLOBAL_POSITION_INT = 33 # GPS position
LOCAL_POSITION_NED = 32 # Local position
VFR_HUD = 74           # HUD information
BATTERY_STATUS = 147    # Battery information
```

## Performance Guidelines

### Latency Optimization
- **VPN**: < 50ms typical latency
- **Internet**: 100-300ms depending on distance
- **Local**: < 1ms latency

### Bandwidth Requirements
- **Telemetry only**: ~10 kbps
- **Full telemetry**: ~50 kbps
- **With video**: 1+ Mbps

### Connection Stability
```python
# Implement connection retry logic
async def robust_connect(connection_string, max_retries=5):
    for attempt in range(max_retries):
        try:
            await drone.connect(connection_string)
            return True
        except Exception as e:
            if attempt < max_retries - 1:
                await asyncio.sleep(2 ** attempt)  # Exponential backoff
                continue
            raise e
```

## Troubleshooting

### Connection Issues
```bash
# Check server status
docker ps | grep px4

# Test network connectivity
ping YOUR_SERVER_IP
traceroute YOUR_SERVER_IP

# Test MAVLink port
nc -u -z YOUR_SERVER_IP 14540
```

### VPN Issues
```bash
# Check VPN status
wg show

# Test VPN connectivity
ping 10.10.0.1
ping 10.10.0.11

# Restart VPN if needed
wg-quick down wg0
wg-quick up wg0
```

### MAVLink Issues
```bash
# Monitor MAVLink messages
python custom-udp-client.py --server-ip YOUR_SERVER_IP --demo monitor

# Check message rates
# Look for HEARTBEAT messages every 1-5 seconds

# Verify system status
# Look for SYS_STATUS messages indicating healthy system
```

## Security Considerations

### VPN Best Practices
- Use unique client configurations
- Rotate keys regularly (every 90 days)
- Monitor connection logs
- Limit client count

### Firewall Configuration
```bash
# Server firewall (example)
ufw allow 51820/udp  # WireGuard VPN
ufw deny 14540:14560/udp  # Block direct MAVLink access
ufw allow from 10.10.0.0/24 to any port 14540:14560 proto udp  # Allow VPN access
```

### Connection Monitoring
```python
# Log all connections
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# In your client code
logger.info(f"Connecting to {connection_string}")
logger.info(f"Connection successful, system_id: {system_id}")
```

## Advanced Examples

### Custom Message Handler
```python
async def handle_custom_messages(drone):
    async for message in drone.core.raw_mavlink():
        if message.message_id == 30:  # ATTITUDE
            # Parse attitude message
            roll, pitch, yaw = parse_attitude(message.payload)
            print(f"Attitude: R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}")
```

### Multi-threaded Client
```python
import threading
import queue

def message_receiver(connection_string, message_queue):
    # Dedicated thread for receiving messages
    pass

def command_sender(connection_string, command_queue):
    # Dedicated thread for sending commands
    pass
```

### Connection Pool
```python
class PX4ConnectionPool:
    def __init__(self, server_ips):
        self.connections = {}
        for ip in server_ips:
            self.connections[ip] = System()
    
    async def get_connection(self, server_ip):
        if server_ip not in self.connections:
            self.connections[server_ip] = System()
        return self.connections[server_ip]
```

## Integration Examples

### ROS Integration
```python
import rospy
from geometry_msgs.msg import PoseStamped

def mavlink_to_ros_bridge():
    rospy.init_node('mavlink_ros_bridge')
    pose_pub = rospy.Publisher('/drone/pose', PoseStamped, queue_size=10)
    
    # Connect to PX4 and publish to ROS
```

### Web Interface
```javascript
// WebSocket MAVLink bridge
const ws = new WebSocket('ws://YOUR_SERVER:8765');
ws.onmessage = function(event) {
    const mavlink_data = JSON.parse(event.data);
    updateDroneDisplay(mavlink_data);
};
```

### Database Logging
```python
import sqlite3

def log_telemetry_to_database(telemetry_data):
    conn = sqlite3.connect('drone_telemetry.db')
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO telemetry (timestamp, lat, lon, alt, battery)
        VALUES (?, ?, ?, ?, ?)
    ''', telemetry_data)
    conn.commit()
```

---

For deployment-specific instructions, see:
- [Cloud Deployment Configurations](../cloud-deployments/)
- [Security Setup Guide](../security/)
- [Architecture Documentation](../docs/)