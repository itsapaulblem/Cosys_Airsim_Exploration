# Updated Unified Generator Guide - Directory Structure Fix

## ✅ **SOLUTION IMPLEMENTED**

The unified generator has been **updated** to create files in the correct directory structure as requested:

### 🎯 **Updated Behavior:**

```bash
cd /mnt/l/Cosys-AirSim/docker_clean/config_generator/tools

# Creates files in docker_clean/multi_drone/
python3 unified_generator.py multi --num-drones 3

# Creates files in docker_clean/single_drone/
python3 unified_generator.py single
```

## 📁 **New Directory Structure**

### **Multi-Drone Configuration:**
```bash
python3 unified_generator.py multi --num-drones 3
```
**Creates in `/docker_clean/multi_drone/`:**
- ✅ `docker-compose.yml` - Ready to run with `docker-compose up`
- ✅ `launch.sh/.bat` - Platform-specific launcher script
- ✅ `settings.json` - Local copy of AirSim settings
- ✅ **Auto-copied to** `~/Documents/AirSim/settings.json`

### **Single Drone Configuration:**
```bash
python3 unified_generator.py single
```
**Creates in `/docker_clean/single_drone/`:**
- ✅ `docker-compose.yml` - Ready to run with `docker-compose up`
- ✅ `launch.sh/.bat` - Platform-specific launcher script  
- ✅ `settings.json` - Local copy of AirSim settings
- ✅ **Auto-copied to** `~/Documents/AirSim/settings.json`

## 🚀 **Enhanced Features**

### **1. Fixed Docker Configuration**
- ✅ **Proper build context**: `../common` (references shared Dockerfile)
- ✅ **Script mounting**: `../common/scripts:/scripts` (references shared scripts)
- ✅ **HIL communication fix**: No TCP port mappings (prevents network loops)
- ✅ **Correct network**: `172.30.0.0/16` subnet with static IPs

### **2. BP_SpiritPawn Default**
All configurations automatically include:
```json
"PawnPaths": {
  "DefaultQuadrotor": {
    "PawnBP": "Class'/AirSim/Blueprints/BP_SpiritPawn.BP_SpiritPawn_C'"
  }
}
```

### **3. Platform-Specific Launchers**
- **Linux**: Generates `launch.sh` with executable permissions
- **Windows**: Generates `launch.bat` for Windows systems
- **Auto-detection**: Uses platform appropriate script by default

## 💡 **Usage Examples**

### **Quick Multi-Drone Setup:**
```bash
cd docker_clean/config_generator/tools
python3 unified_generator.py multi --num-drones 3

# Files created in docker_clean/multi_drone/
cd ../../multi_drone
./launch.sh  # Start containers
```

### **Single Drone Setup:**
```bash
cd docker_clean/config_generator/tools  
python3 unified_generator.py single

# Files created in docker_clean/single_drone/
cd ../../single_drone
./launch.sh  # Start containers
```

### **Custom Output (Override Default):**
```bash
# Still creates in custom directory if specified
python3 unified_generator.py multi --num-drones 5 --output-dir ./my_custom_config
```

## 🔧 **Generated Docker Compose Features**

### **Fixed HIL Communication:**
```yaml
ports:
  # TCP port mapping removed - PX4 connects outbound to AirSim on host
  # HIL connection: container -> host.docker.internal:456X (no mapping needed)
  - "14550:14550/udp"   # QGroundControl
  - "14541:14541/udp"   # MAVLink control local
  - "14582:14582/udp"   # MAVLink control remote
```

### **Proper Build Context:**
```yaml
build:
  context: ../common      # References shared Dockerfile
  dockerfile: Dockerfile
volumes:
  - px4-shared-data:/px4_data
  - ../common/scripts:/scripts  # References shared scripts
```

### **Correct Networking:**
```yaml
networks:
  airsim-network:
    ipv4_address: 172.30.0.10  # Static IP for each drone
```

## 📋 **Complete Workflow**

### **1. Generate Configuration**
```bash
cd docker_clean/config_generator/tools
python3 unified_generator.py multi --num-drones 3
```

### **2. Navigate and Start**
```bash
cd ../../multi_drone
./launch.sh  # or launch.bat on Windows
```

### **3. Start AirSim**
```bash
# AirSim reads settings from ~/Documents/AirSim/settings.json
# (automatically copied by generator)
./YourUnrealProject.exe
```

### **4. Ready to Fly!**
- ✅ HIL communication established
- ✅ GPS home location set
- ✅ `armDisarm()` works without errors
- ✅ Multi-drone missions ready

## 🆚 **Before vs After**

| Aspect | Before | After |
|--------|--------|-------|
| **Output Location** | Current directory | `docker_clean/multi_drone/` or `docker_clean/single_drone/` |
| **Manual Navigation** | Required | Not required - files in correct location |
| **Build Context** | Wrong path | `../common` (correct) |
| **Script Access** | Manual setup | Auto-mounted from `../common/scripts` |
| **Ready to Run** | Manual editing needed | Immediate `docker-compose up` |

## ✅ **Key Benefits**

### **1. Zero Manual Steps**
- **Direct execution**: `cd multi_drone && docker-compose up`
- **No file movement**: Generated in correct location
- **No path editing**: Build context automatically correct

### **2. Platform Compatibility**
- **Windows**: `.bat` launcher with Windows commands
- **Linux**: `.sh` launcher with proper permissions
- **Auto-detection**: Chooses correct platform automatically

### **3. Integration Ready**
- **Existing workflows**: Works with current directory structure
- **Testing scripts**: Compatible with existing test tools
- **Documentation**: Matches workflow guides

## 🎯 **Exact Commands You Requested**

### **Multi-Drone (3 drones):**
```bash
cd docker_clean/config_generator/tools
python3 unified_generator.py multi --num-drones 3

# Output:
# ✅ Generated: /path/to/docker_clean/multi_drone/docker-compose.yml
# ✅ Generated: /path/to/docker_clean/multi_drone/launch.sh
# ✅ Generated: /path/to/docker_clean/multi_drone/settings.json

cd ../../multi_drone
docker-compose up  # OR ./launch.sh
```

### **Single Drone:**
```bash
cd docker_clean/config_generator/tools
python3 unified_generator.py single

# Output:
# ✅ Generated: /path/to/docker_clean/single_drone/docker-compose.yml
# ✅ Generated: /path/to/docker_clean/single_drone/launch.sh
# ✅ Generated: /path/to/docker_clean/single_drone/settings.json

cd ../../single_drone  
docker-compose up  # OR ./launch.sh
```

**The unified generator now creates files exactly where you need them for immediate use!**