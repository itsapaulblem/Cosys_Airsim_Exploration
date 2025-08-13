# 🔧 Docker Networking Fix Guide

**Quick fix for "Vehicle does not have a valid GPS home location" errors**

## 🎯 The Issue

When running AirSim with Docker containers, you may encounter:
```
rpclib: function 'armDisarm' threw an exception: Vehicle does not have a valid GPS home location
```

This happens when:
- Docker containers can't establish TCP connections to AirSim
- PX4 HIL (Hardware-in-Loop) communication fails
- Missing TCP port mappings in docker-compose.yml

## ✅ The Solution

Use the `--enable-tcp-ports` flag when generating configurations:

```bash
# Navigate to config generator
cd /path/to/Cosys-AirSim/docker_clean/config_generator/tools

# Generate 2-drone config with TCP ports
python unified_generator.py multi --num-drones 2 --enable-tcp-ports

# Generate single drone with TCP ports
python unified_generator.py single --enable-tcp-ports
```

## 🔍 What This Does

**Without `--enable-tcp-ports`** (old behavior):
```yaml
ports:
- 14550:14550/udp  # QGroundControl
- 14541:14541/udp  # MAVLink Local
- 14581:14581/udp  # MAVLink Remote
# Missing TCP ports!
```

**With `--enable-tcp-ports`** (fixed):
```yaml
ports:
- 14550:14550/udp  # QGroundControl
- 14541:14541/udp  # MAVLink Local 
- 14581:14581/udp  # MAVLink Remote
- 4561:4561/tcp    # AirSim HIL ✅
```

## 🚀 Complete Fix Workflow

1. **Stop current containers**:
   ```bash
   cd /path/to/your/docker-compose/directory
   docker-compose down
   ```

2. **Generate fixed configuration**:
   ```bash
   cd /path/to/Cosys-AirSim/docker_clean/config_generator/tools
   python unified_generator.py multi --num-drones 2 --enable-tcp-ports --output-dir /tmp/fixed_config
   ```

3. **Copy fixed docker-compose.yml**:
   ```bash
   cp /tmp/fixed_config/docker-compose.yml /path/to/your/docker-compose/directory/
   ```

4. **Start with fixed configuration**:
   ```bash
   docker-compose up -d
   ```

5. **Test your mission**:
   ```bash
   python multi_drone_orbit_mission.py
   ```

## 📊 Verification

Check if TCP ports are properly exposed:
```bash
docker ps --format "table {{.Names}}\t{{.Ports}}"
```

You should see:
```
px4-drone1    ...4561:4561/tcp...
px4-drone2    ...4562:4562/tcp...
```

## 🔄 Alternative: Quick Manual Fix

If you have an existing docker-compose.yml, manually add TCP ports:

```yaml
services:
  px4-drone1:
    ports:
    - "14550:14550/udp"
    - "14541:14541/udp" 
    - "14581:14581/udp"
    - "4561:4561/tcp"    # Add this line
    
  px4-drone2:
    ports:
    - "14551:14550/udp"
    - "14542:14542/udp"
    - "14582:14582/udp" 
    - "4562:4562/tcp"    # Add this line
```

## 💡 Why This Works

- **Root Cause**: Docker container isolation prevents PX4 from connecting to AirSim
- **Solution**: TCP port mappings create bidirectional communication channels
- **Result**: PX4 containers can establish HIL connections → GPS home location works

## 🔧 Technical Details

The unified generator now supports two networking modes:

1. **Outbound Only** (default): PX4 connects to `host.docker.internal:456X`
2. **Bidirectional** (`--enable-tcp-ports`): Full TCP port mapping for complex setups

Use `--enable-tcp-ports` when:
- Running on Windows Docker Desktop
- Complex Docker networking setups
- Corporate/restricted networks
- Getting GPS home location errors

---

**This fix resolves the fundamental Docker networking issue that prevents AirSim-PX4 HIL communication!**