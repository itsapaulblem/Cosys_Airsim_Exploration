# PX4 Autopilot Integration with AirSim - Complete Guide

This document explains how PX4 autopilot integrates with AirSim through RPC and MAVLink communication layers.

## 🏗️ Architecture Overview

```
┌─────────────────┐    MAVLink     ┌─────────────────┐    RPC      ┌─────────────────┐
│   PX4 Autopilot │◄──────────────►│   AirSim Plugin │◄───────────►│   ROS2 Wrapper  │
│   (SITL/HITL)   │   TCP/UDP      │   (Unreal)      │  Port 41451 │   (Docker)      │
└─────────────────┘   Port 14560   └─────────────────┘             └─────────────────┘
        │                                   │                              │
        │                                   │                              │
    ┌───▼────┐                         ┌────▼─────┐                   ┌────▼────┐
    │QGroundControl                   │ Physics   │                   │ User    │
    │Mission Planning │                │ Simulation│                   │ Apps    │
    └─────────────────┘                └──────────┘                   └─────────┘
```

## 🔗 Communication Layers

### 1. **PX4 ↔ AirSim (MAVLink Protocol)**

#### MAVLink Connection Configuration
```cpp
// From AirSimSettings.hpp
struct MavLinkConnectionInfo {
    bool use_serial = true;           // false = use UDP/TCP
    
    // Serial connection (HITL)
    std::string serial_port = "*";    // Auto-detect
    int baud_rate = 115200;
    
    // UDP connection (SITL)
    std::string udp_address = "127.0.0.1";
    int udp_port = 14560;             // PX4 SITL port
    
    // TCP connection (Alternative)
    bool use_tcp = false;
    int tcp_port = 4560;
    
    // Ground Control Station channel
    std::string control_ip_address = "127.0.0.1";
    int control_port_local = 14540;   // Local GCS port
    int control_port_remote = 14580;  // Remote GCS port
    
    // Vehicle identifiers
    uint8_t sim_sysid = 142;          // Simulator system ID
    uint8_t vehicle_sysid = 135;      // Vehicle system ID
    uint8_t offboard_sysid = 134;     // Offboard control system ID
};
```

#### Data Flow: PX4 → AirSim
1. **Motor Commands**: PX4 sends PWM/actuator commands via MAVLink
2. **Flight Mode**: Current flight mode (Manual, Mission, Offboard, etc.)
3. **Arming State**: Armed/disarmed status
4. **Navigation Commands**: Waypoints, velocity setpoints, attitude commands

#### Data Flow: AirSim → PX4
1. **Sensor Data**: IMU, GPS, barometer, magnetometer readings
2. **State Estimation**: Position, velocity, attitude from simulation
3. **RC Override**: Remote control inputs (if simulated)

### 2. **AirSim ↔ ROS2 (RPC Protocol)**

#### RPC Communication Details
```cpp
// From MavLinkMultirotorApi.hpp
class MavLinkMultirotorApi : public MultirotorApiBase {
    void initialize(const AirSimSettings::MavLinkConnectionInfo& connection_info,
                   const SensorCollection* sensors, 
                   bool is_simulation);
    
    // Sensor data retrieval
    virtual const SensorCollection& getSensors() const override;
    
    // Flight control interface
    virtual bool takeoff(float timeout_sec) override;
    virtual bool land(float timeout_sec) override;
    virtual bool moveByVelocity(float vx, float vy, float vz, float duration) override;
};
```

## 🚁 Integration Modes

### Mode 1: Software-in-the-Loop (SITL)

**Setup**: PX4 runs as software process, AirSim provides simulation

```bash
# Terminal 1: Start PX4 SITL
cd /path/to/PX4-Autopilot
make px4_sitl gazebo

# Terminal 2: Start AirSim with PX4 settings
# Use settings.json with PX4 vehicle configuration

# Terminal 3: Start ROS2 wrapper
./run_simple.bat
```

**AirSim Settings for SITL**:
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "PX4Drone": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580
        }
    }
}
```

### Mode 2: Hardware-in-the-Loop (HITL)

**Setup**: Real PX4 hardware connected via serial, AirSim provides sensor simulation

```json
{
    "Vehicles": {
        "PX4Drone": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": true,
            "SerialPort": "COM3",      // Windows
            "BaudRate": 921600,
            "UseTcp": false
        }
    }
}
```

### Mode 3: SimpleFlight (AirSim Native)

**Setup**: AirSim's built-in flight controller (no PX4)

```json
{
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true
        }
    }
}
```

## 🔄 Data Flow Examples

### Example 1: Takeoff Command Flow

```
1. ROS2 User: ros2 service call /airsim_node/Drone1/takeoff
   ↓
2. ROS2 Wrapper: Calls AirSim RPC takeoff()
   ↓  
3. AirSim Plugin: If PX4 mode → Sends MAVLink TAKEOFF command
   ↓
4. PX4 Autopilot: Executes takeoff sequence, sends motor commands
   ↓
5. AirSim Physics: Applies forces to simulated drone
   ↓
6. AirSim Sensors: Generate IMU, GPS data based on new position
   ↓
7. PX4 Autopilot: Receives sensor feedback via MAVLink
   ↓
8. ROS2 Wrapper: Publishes updated state to ROS topics
```

### Example 2: Sensor Data Flow

```
AirSim Physics Engine
    ↓ (Simulated sensor readings)
MAVLink Telemetry Messages
    ↓ (Periodic sensor data)
PX4 State Estimation (EKF)
    ↓ (Filtered state)
MAVLink State Messages
    ↓ (Position, velocity, attitude)
AirSim RPC API
    ↓ (getMultirotorState())
ROS2 Wrapper
    ↓ (Published to topics)
/airsim_node/Drone1/odom_local_ned
```

## 🛠️ Setup Instructions

### Step 1: PX4 Configuration

```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Build for SITL
make px4_sitl_default

# Run with AirSim (instead of Gazebo)
make px4_sitl_default none_iris
```

### Step 2: AirSim Settings

Create `~/Documents/AirSim/settings.json`:
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor", 
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "UdpIp": "127.0.0.1",
            "UdpPort": 14560,
            "UseLogViewer": true
        }
    },
    "LocalHostIp": "127.0.0.1",
    "ApiServerEndpoint": "0.0.0.0:41451",
    "LogMessagesVisible": true
}
```

### Step 3: Launch Sequence

```bash
# 1. Start PX4 SITL
cd PX4-Autopilot
make px4_sitl none_iris

# 2. Start AirSim (Unreal Engine)
./AirSimNH.exe  # or your AirSim executable

# 3. Start ROS2 Wrapper
cd Cosys-AirSim/docker/airsim_ros2_wrapper
./run_simple.bat

# 4. Optional: Start QGroundControl
# Download and run QGroundControl for mission planning
```

## 🎮 Control Methods

### Method 1: QGroundControl
- **Mission Planning**: Create waypoint missions
- **Manual Control**: Direct stick inputs
- **Parameter Tuning**: Adjust PX4 parameters

### Method 2: ROS2 Commands
```bash
# Arm and takeoff
./ros2_exec.bat "ros2 service call /airsim_node/PX4/takeoff airsim_interfaces/srv/Takeoff '{wait_on_last_task: true}'"

# Offboard velocity control
./ros2_exec.bat "ros2 topic pub /airsim_node/PX4/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}'"
```

### Method 3: MAVSDK/Python
```python
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    await drone.action.arm()
    await drone.action.takeoff()
```

## 🔧 Troubleshooting

### Common Issues

#### 1. MAVLink Connection Failed
```bash
# Check port availability
netstat -an | grep 14560

# Verify PX4 is listening
nc -u 127.0.0.1 14560
```

#### 2. RPC Connection Failed
```bash
# Test AirSim RPC
telnet localhost 41451

# Check AirSim logs
# Look in AirSim output for connection errors
```

#### 3. Clock Synchronization Issues
- Use `"ClockType": "SteppableClock"` in AirSim settings
- Ensure `LockStep: true` in MAVLink connection

#### 4. Sensor Data Not Flowing
```bash
# Check MAVLink messages
mavproxy.py --master=udp:127.0.0.1:14551

# Monitor ROS2 topics
./ros2_exec.bat "ros2 topic hz /airsim_node/PX4/imu/imu"
```

## 📊 Performance Tuning

### Recommended Settings
```json
{
    "ClockType": "SteppableClock",
    "LocalHostIp": "127.0.0.1",
    "SimMode": "Multirotor",
    "EnableCollisionPassthrough": false,
    "ViewMode": "NoDisplay",          // For headless operation
    "Recording": {
        "RecordOnMove": false,
        "RecordInterval": 0.05
    }
}
```

### PX4 Parameters
```bash
# Increase EKF update rate
param set EKF2_PREDICT_US 10000

# Adjust simulation time factor
param set SIM_SPEED_FACTOR 1
```

## 🔗 Integration Benefits

### Advantages of PX4 + AirSim + ROS2
1. **Realistic Flight Control**: Real autopilot behavior
2. **Mission Planning**: Use QGroundControl for complex missions
3. **Algorithm Testing**: Test custom algorithms with real flight stack
4. **Multi-Vehicle**: Support for swarm operations
5. **Hardware Compatibility**: Same code works on real hardware

### Use Cases
- **Algorithm Development**: Test SLAM, path planning, computer vision
- **Mission Validation**: Validate missions before real flights
- **Failure Testing**: Simulate sensor failures, GPS dropouts
- **Swarm Research**: Multi-vehicle coordination algorithms

---

**Next Steps**: 
- See `docker/px4_airsim_docker/` for complete PX4 Docker setup
- Use `AirSim_Multi_Machine_Setup.md` for distributed testing
- Check PX4 documentation: https://docs.px4.io/main/en/simulation/airsim.html 