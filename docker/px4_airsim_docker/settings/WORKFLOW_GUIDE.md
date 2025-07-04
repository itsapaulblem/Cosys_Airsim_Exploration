# 🚁 Complete PX4-AirSim Multi-Drone Workflow Guide

## 📋 Overview

This guide walks you through the complete process from generating AirSim settings to running multiple PX4 drones in Docker containers with zero port conflicts.

## 🎯 Prerequisites

- Docker Desktop installed and running
- Python 3.x installed
- AirSim/Unreal Engine environment ready
- Windows PowerShell or Linux terminal

## 📊 Workflow Steps

### Step 1: Generate AirSim Settings.json

Choose one of these methods to create your AirSim configuration:

#### Method A: Auto-Generate Settings (Recommended)
```bash
# Generate settings for N drones (1-10)
python generate_settings.py 3

# Or specify custom output location
python generate_settings.py 5 ./custom_settings.json
```

**Features:**
- ✅ Automatic port calculation (no conflicts)
- ✅ Proper GPS sensor configuration
- ✅ PX4Multirotor vehicle type
- ✅ Grid-based drone positioning
- ✅ Includes your BP_SpiritPawn.BP_SpiritPawn_C pawn path

#### Method B: Manual Settings Creation
Edit `C:\Users\YourName\Documents\AirSim\settings.json` manually with your desired configuration.

### Step 2: Generate Docker Compose Configuration

Generate a static Docker Compose file from your settings.json:

```bash
python simple_generator.py
```

**Generated Files:**
- `docker-compose.generated.yml` - Static compose configuration
- `launch_generated.bat` - Automated launcher script

**Benefits:**
- ✅ Zero port conflicts guaranteed
- ✅ Perfect mapping from settings.json
- ✅ Static configuration (no runtime complexity)
- ✅ Predictable container names and IPs

### Step 3: Launch Docker Containers

Run the generated launcher:

```bash
# Windows
.\launch_generated.bat

# Or manually
docker-compose -f docker-compose.generated.yml up -d
```

**What Happens:**
1. Creates shared data directory
2. Builds PX4 Docker images
3. Creates airsim-network (172.25.0.0/16)
4. Starts containers with unique ports
5. Each PX4 instance waits for AirSim connection

### Step 4: Verify Container Status

Check that all containers are running:

```bash
# View container status
docker-compose -f docker-compose.generated.yml ps

# Check logs
docker-compose -f docker-compose.generated.yml logs

# View specific drone logs
docker logs px4-px4-drone1
```

**Success Indicators:**
- ✅ `Simulator connected on TCP port 456X`
- ✅ `INFO [tone_alarm] home set` (GPS working)
- ✅ `INFO [commander] Ready for takeoff!`

### Step 5: Start AirSim/Unreal Engine

1. **Launch your AirSim environment** (Blocks, custom map, etc.)
2. **Use your generated settings.json** (automatically loaded)
3. **Drones auto-connect** to waiting Docker containers

**Expected Result:**
- Each drone in AirSim connects to its corresponding Docker container
- GPS coordinates properly established
- All drones ready for flight

### Step 6: Control and Monitor

#### QGroundControl Connection
```bash
# Connect QGroundControl to specific drones:
# Drone 1: localhost:14550
# Drone 2: localhost:14551
# Drone N: localhost:1455(N-1)
```

#### Python API Control
```python
import airsim

# Connect to specific drones
client = airsim.MultirotorClient()
client.confirmConnection()

# List available vehicles
print(client.listVehicles())

# Control specific drone
client.enableApiControl(True, "PX4_Drone1")
client.armDisarm(True, "PX4_Drone1")
```

#### Docker Management
```bash
# View logs in real-time
docker-compose -f docker-compose.generated.yml logs -f

# Stop all containers
docker-compose -f docker-compose.generated.yml down

# Restart specific container
docker-compose -f docker-compose.generated.yml restart px4-px4-drone1

# Scale up/down (regenerate settings first)
python generate_settings.py 5
python simple_generator.py
.\launch_generated.bat
```

## 🔧 Port Mapping Reference

| Drone | AirSim TCP | MAVLink Local | MAVLink Remote | QGroundControl | Container IP |
|-------|------------|---------------|----------------|----------------|--------------|
| Drone1| 4561       | 14541         | 14581          | 14550          | 172.25.0.10  |
| Drone2| 4562       | 14542         | 14582          | 14551          | 172.25.0.11  |
| Drone3| 4563       | 14543         | 14583          | 14552          | 172.25.0.12  |
| DroneN| 456N       | 1454N         | 1458N          | 1455(N-1)      | 172.25.0.(9+N)|

## 🛠️ Troubleshooting

### GPS Issues
```bash
# Check GPS status
python diagnose_gps_docker.py

# Look for these success indicators:
# ✅ GPS Fix = 3
# ✅ Home coordinates established
# ✅ "INFO [tone_alarm] home set"
```

### Port Conflicts
```bash
# List used ports
netstat -an | grep 4561
netstat -an | grep 14541

# If conflicts exist, regenerate with different base ports
# (Edit generate_settings.py and modify port calculations)
```

### Container Issues
```bash
# Check container health
docker ps
docker inspect px4-px4-drone1

# Restart problematic container
docker-compose -f docker-compose.generated.yml restart px4-px4-drone1

# View detailed logs
docker logs px4-px4-drone1 --tail=50
```

### AirSim Connection Issues
```bash
# Verify AirSim API is accessible
curl http://localhost:41451/ping

# Check if containers can reach AirSim
docker exec px4-px4-drone1 ping host.docker.internal

# Verify network configuration
docker network inspect airsim-network
```

## 🚀 Advanced Usage

### Custom Configurations

#### Modify GPS Coordinates
Edit `generate_settings.py`:
```python
"OriginGeopoint": {
    "Latitude": YOUR_LAT,    # e.g., 1.3521 for Singapore
    "Longitude": YOUR_LON,   # e.g., 103.8198 for Singapore
    "Altitude": YOUR_ALT
}
```

#### Change Drone Positioning
Modify the position calculation in `generate_settings.py`:
```python
# Custom formation (modify these lines)
x_pos = (i - 1) * 5  # Change spacing
y_pos = 0           # Add Y offset
```

#### Add Custom Sensors
Extend the vehicle configuration with additional sensors in `generate_settings.py`.

### Scaling to More Drones

1. **Update port ranges** in `generate_settings.py`
2. **Modify IP address allocation** in `simple_generator.py`
3. **Ensure your network can handle the connections**

### Integration with Mission Planning

```python
# Example: Coordinated mission for multiple drones
import airsim
import asyncio

async def multi_drone_mission():
    clients = {}
    for i in range(1, 4):  # 3 drones
        client = airsim.MultirotorClient()
        clients[f"PX4_Drone{i}"] = client
        
        # Enable API control
        client.enableApiControl(True, f"PX4_Drone{i}")
        client.armDisarm(True, f"PX4_Drone{i}")
    
    # Coordinate takeoff
    tasks = []
    for drone_name, client in clients.items():
        task = client.takeoffAsync(vehicle_name=drone_name)
        tasks.append(task)
    
    await asyncio.gather(*tasks)
    print("All drones airborne!")

# Run mission
asyncio.run(multi_drone_mission())
```

## 📝 Summary

This workflow provides a **production-ready**, **conflict-free** solution for running multiple PX4 drones with AirSim. The static Docker Compose approach eliminates port conflicts and provides predictable, scalable drone management.

**Key Benefits:**
- ✅ **Zero port conflicts** - Each drone has unique ports
- ✅ **GPS issues resolved** - Proper network isolation
- ✅ **Scalable** - Easily add/remove drones
- ✅ **Automated** - One-command deployment
- ✅ **Production-ready** - Reliable and repeatable

**Files Generated:**
- `settings.json` - AirSim configuration
- `docker-compose.generated.yml` - Container definitions
- `launch_generated.bat` - Automated launcher
- Container logs and diagnostics

Ready for flight! 🚁✨ 