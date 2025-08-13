# 🚁 AirSim Configuration Generator

**Unified, standardized tools for generating AirSim settings.json and Docker Compose configurations**

This directory contains the production-ready, unified configuration generation tools for Cosys-AirSim. These tools replace the scattered legacy generators with a single, comprehensive workflow.

## 📁 Directory Structure

```
config_generator/
├── README.md                    # This file
├── tools/                       # Core generation and validation tools
│   ├── unified_generator.py     # Main configuration generator
│   ├── config_validator.py      # Configuration validation tool
│   └── migrate_to_unified.py    # Migration from legacy tools
├── templates/                   # Reference templates and examples
│   ├── airsim_px4_single_settings.json
│   ├── airsim_px4_multi_settings.json
│   └── Dockerfile
├── examples/                    # Example workflows and demonstrations
│   └── workflow_example.py      # Interactive workflow examples
└── docs/                        # Documentation
    └── STANDARDIZED_WORKFLOW.md # Comprehensive workflow guide
```

## 🚀 Quick Start

### Prerequisites

- Python 3.7+
- Docker Desktop
- **PyYAML package**: `pip install PyYAML` or `pip install pyyaml`

#### PyYAML Dependency

The unified generator can work **with or without** PyYAML:

- **With PyYAML**: Full functionality including RViz config generation
- **Without PyYAML**: Docker-compose files generated as text (RViz configs skipped)

**Installation Options:**
```bash
# Option 1: Standard pip
pip install PyYAML

# Option 2: Alternative name
pip install pyyaml

# Option 3: Windows with Anaconda
conda install pyyaml

# Option 4: Ubuntu/Debian
sudo apt install python3-yaml
```

### Basic Usage

```bash
# Navigate to the config generator tools
cd /path/to/Cosys-AirSim/docker_clean/config_generator/tools

# Generate single drone configuration
python unified_generator.py single

# Generate multi-drone configuration (3 drones)
python unified_generator.py multi --num-drones 3

# Validate any configuration
python config_validator.py --all

# Interactive examples and tutorials
cd ../examples
python workflow_example.py
```

## 🛠️ Core Tools

### 1. Unified Generator (`tools/unified_generator.py`)

**The primary tool for generating standardized configurations.**

```bash
# Basic modes
python unified_generator.py single                    # Single drone (PX4)
python unified_generator.py multi --num-drones 5      # 5 drones (PX4)
python unified_generator.py mixed                     # Mixed vehicles
python unified_generator.py ultra-swarm               # 27-drone ultra-swarm (3 swarms x 9 drones, PX4)
python unified_generator.py simpleflight --num-drones 10  # 10 SimpleFlight drones (no Docker)

# Camera integration
python unified_generator.py single --cameras front_rgb front_depth --create-rviz
python unified_generator.py multi --num-drones 2 --cameras front_rgb segmentation_cam --create-rviz

# Ultra-swarm configurations
python unified_generator.py ultra-swarm                                    # Full 27-drone setup
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 5  # 10 drones (2x5)
python unified_generator.py ultra-swarm --settings-only                   # Only generate settings.json

# Advanced options
python unified_generator.py multi --num-drones 3 --layout circle --gps-location 40.7128 -74.0060 10
python unified_generator.py single --output-dir ./my_config --platform linux
```

**Features:**
- ✅ Generates both settings.json AND docker-compose.yml
- ✅ **Ultra-swarm support** - Up to 27 drones (3 swarms × 9 drones each)
- ✅ **Camera integration** with 7 presets and RViz2 support
- ✅ **SubWindows** for live camera feeds in Unreal
- ✅ **BP_SpiritPawn** as default quadrotor pawn
- ✅ Automatic port conflict prevention with swarm-aware allocation
- ✅ Multiple vehicle types (multirotors, cars, mixed)
- ✅ Flexible positioning layouts (grid, line, circle)
- ✅ Custom GPS coordinates with per-swarm offsets
- ✅ Platform-specific launcher scripts

### 2. Configuration Validator (`tools/config_validator.py`)

**Comprehensive validation for generated configurations.**

```bash
# Validate everything
python config_validator.py --all

# Validate specific files
python config_validator.py --settings my_settings.json --compose my_compose.yml

# Check system prerequisites
python config_validator.py --system
```

**Validation Levels:**
- 🚨 **CRITICAL** - Must fix before proceeding
- ❌ **ERROR** - Should fix for proper operation  
- ⚠️ **WARNING** - May cause issues
- ℹ️ **INFO** - Informational only

### 3. Migration Tool (`tools/migrate_to_unified.py`)

**Migrate from legacy configuration tools.**

```bash
# Analyze current setup and migrate
python migrate_to_unified.py

# Options: migrate settings, compose, or both
# Automatically backs up original files
```

## 📖 Workflow Examples

### Example 1: Single Drone Development Setup

```bash
cd tools
python unified_generator.py single
python config_validator.py --all
cd ../../single_drone
./start_single.bat
```

### Example 2: Multi-Drone Research Setup

```bash
cd tools
python unified_generator.py multi --num-drones 4 --layout grid
python config_validator.py --all
# Copy generated files to your preferred location
```

### Example 2b: Ultra-Swarm Setup (27 Drones)

```bash
cd tools
# Generate full ultra-swarm (27 drones across 3 swarms)
python unified_generator.py ultra-swarm
python config_validator.py --all
cd ../../ultra_swarm
# Use the generated files with ultra-swarm Docker setup
```

### Example 2c: Partial Ultra-Swarm Setup

```bash
cd tools
# Generate smaller ultra-swarm (2 swarms, 5 drones each = 10 total)
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 5
python config_validator.py --all
# Perfect for testing swarm algorithms without full 27-drone overhead
```

### Example 3: Custom GPS Location (Tokyo)

```bash
cd tools
python unified_generator.py multi --num-drones 2 --gps-location 35.6762 139.6503 10
python config_validator.py --compatibility
```

### Example 4: SimpleFlight Development Setup

```bash
cd tools
# Generate SimpleFlight configuration (10 drones, no Docker required)
python unified_generator.py simpleflight --num-drones 10
python config_validator.py --all

# Launch AirSim and all drones are immediately available
# No Docker containers to manage!
```

### Example 5: SimpleFlight with Camera Integration

```bash
cd tools
# Generate SimpleFlight with cameras for computer vision research
python unified_generator.py simpleflight --num-drones 5 --cameras front_rgb downward_cam --create-rviz
python config_validator.py --all

# Perfect for CV/ML algorithm development
```

### Example 6: Maximum SimpleFlight Configuration

```bash
cd tools
# Generate maximum SimpleFlight setup (27 drones)
python unified_generator.py simpleflight --num-drones 27 --layout grid
python config_validator.py --all

# Ultimate stress testing without Docker overhead
```

### Example 7: Interactive Learning

```bash
cd examples
python workflow_example.py
# Follow the interactive menu to learn different workflows
```

## 🔄 Migration from Legacy Tools

If you're currently using:
- `generate_settings.py`
- `simple_generator.py`  
- Manual docker-compose creation
- Scattered configuration files

**Migration Path:**
1. Run the migration tool: `python tools/migrate_to_unified.py`
2. Review the migration report
3. Validate migrated configurations
4. Test with small setups first
5. Update your workflows to use unified tools

## 📋 Configuration Options

### Vehicle Types
- **PX4Multirotor** - PX4 SITL drones with Docker (supports ultra-swarm, 1-27 drones)
- **ArduCopter** - ArduCopter drones
- **SimpleFlight** - Built-in AirSim flight controller (supports 1-27 drones, no Docker required)
- **PhysXCar** - Ground vehicles
- **ComputerVision** - Camera-only mode

### SimpleFlight Configuration
- **Maximum Scale**: 27 drones (single swarm)
- **Vehicle Naming**: `Drone1`, `Drone2`, `Drone3`, etc.
- **Controller**: Built-in AirSim SimpleFlight controller
- **Benefits**: 
  - ✅ No Docker setup required
  - ✅ Fast startup and lightweight
  - ✅ Perfect for development and testing
  - ✅ Built-in physics simulation
  - ✅ Immediate API access via AirSim server
- **Connection**: All drones connect to single AirSim server (localhost:4561)
- **Use Cases**: Algorithm development, rapid prototyping, educational scenarios

### Ultra-Swarm Configuration
- **Maximum Scale**: 27 drones (3 swarms × 9 drones each)
- **Swarm Naming**: `PX4_Swarm1_Drone1`, `PX4_Swarm2_Drone1`, etc.
- **Port Allocation**: 
  - Swarm 1: TCP 4561-4569, UDP 14550-14558, MAVLink 18570-18578
  - Swarm 2: TCP 4571-4579, UDP 14560-14568, MAVLink 18580-18588
  - Swarm 3: TCP 4581-4589, UDP 14570-14578, MAVLink 18590-18598
- **GPS Locations**: Each swarm offset for realistic multi-area operations
  - Swarm 1: Seattle area (47.641468, -122.140165)
  - Swarm 2: Bellevue area (47.642468, -122.139165)
  - Swarm 3: Redmond area (47.643468, -122.138165)

### Layout Patterns
- **Grid** (default) - Arranges vehicles in rows of 5
- **Line** - Single line with 5m spacing
- **Circle** - Circular formation with 10m radius

### GPS Locations
Common coordinates for testing:
```bash
# Seattle (default)
--gps-location 47.641468 -122.140165 10

# New York City  
--gps-location 40.7128 -74.0060 10

# Tokyo
--gps-location 35.6762 139.6503 10

# London
--gps-location 51.5074 -0.1278 10
```

## 🔧 Integration with Docker Clean

This config generator integrates seamlessly with the docker_clean structure:

### Standard Multi-Drone Setup
```bash
# Generate configs with unified tools
cd config_generator/tools
python unified_generator.py multi --num-drones 3

# Copy generated docker-compose.yml to appropriate directory
cp docker-compose.yml ../../multi_drone/

# Launch using clean structure
cd ../../multi_drone
./start_multi.bat
```

### Ultra-Swarm Integration
```bash
# Generate ultra-swarm configurations
cd config_generator/tools
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 5

# Files are automatically placed in ../../ultra_swarm/
cd ../../ultra_swarm
ls -la  # You'll see settings.json and docker-compose.yml

# Use with the ultra-swarm Docker setup (if available)
# Or integrate with px4_airsim_docker_v2 for the full 27-drone setup
```

### Integration with Ultra-Swarm Docker Setup
```bash
# Generate settings.json for ultra-swarm Docker
cd config_generator/tools
python unified_generator.py ultra-swarm --settings-only

# Copy to ultra-swarm Docker directory
cp ../../ultra_swarm/settings.json ../../docker/px4_airsim_docker_v2/

# Use with ultra-swarm Docker commands
cd ../../docker/px4_airsim_docker_v2
./start-ultra-swarms.sh swarm1-full  # Start 9 drones with generated settings
```

## 📝 Best Practices

### 1. Always Validate
```bash
# After generating any configuration
python config_validator.py --all
```

### 2. Use Version Control
```bash
# Keep your configurations in git
git add settings.json docker-compose.yml
git commit -m "Add 3-drone configuration for experiment X"
```

### 3. Test Incrementally
- Start with single drone
- Validate and test
- Scale up to multi-drone (3-5 drones)
- Test swarm behaviors
- Move to ultra-swarm (start with 2 swarms × 3 drones)
- Scale to full 27-drone configuration only when needed
- Test each step thoroughly

### 4. Document Your Setups
```bash
# Generate configurations with descriptive names
python unified_generator.py multi --num-drones 3 --output-dir ./formations/triangle_formation
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 4 --output-dir ./experiments/swarm_coordination_test
```

### 5. Clean Up Regularly
```bash
# Use the validator to check for orphaned configurations
python config_validator.py --system
```

## 🚨 Troubleshooting

### PyYAML Import Error
```bash
# Error: ModuleNotFoundError: No module named 'yaml'

# Solution 1: Install PyYAML
pip install PyYAML

# Solution 2: Use without PyYAML (docker-compose still works)
python unified_generator.py multi --num-drones 2
# ⚠️ PyYAML not installed. Docker-compose files will be generated as text.

# Solution 3: Check Python environment
python -c "import yaml; print('PyYAML is installed')"

# Solution 4: Different Python environments
# If using conda:
conda install pyyaml
# If using system Python on Ubuntu:
sudo apt install python3-yaml
```

### Docker Networking Issues (GPS Home Location Errors)
```bash
# Error: "Vehicle does not have a valid GPS home location"
# This often indicates Docker networking problems

# Solution: Enable TCP port mappings
python unified_generator.py multi --num-drones 2 --enable-tcp-ports

# This adds TCP port mappings (4561-4565) to docker-compose.yml
# Required when PX4 containers can't connect outbound to AirSim

# Alternative: Manual port mapping check
docker ps --format "table {{.Names}}\t{{.Ports}}"
# Look for TCP ports like 4561:4561/tcp

# If still failing, restart entire Docker stack
docker-compose down && docker-compose up -d
```

### Port Conflicts
```bash
# Check what's using ports
netstat -an | grep 4561

# Regenerate with clean ports
python unified_generator.py single
```

### Docker Issues
```bash
# Verify Docker is running
python config_validator.py --system

# Clean Docker environment
docker system prune -f
```

### Configuration Errors
```bash
# Run comprehensive validation
python config_validator.py --all

# Check specific issues
python config_validator.py --compatibility
```

### Migration Issues
```bash
# Run migration analysis
python migrate_to_unified.py

# Check migration report for detailed guidance
```

## ✈️ SimpleFlight Advanced Usage

SimpleFlight is AirSim's built-in flight controller, perfect for rapid development and testing without Docker complexity.

### SimpleFlight Command Reference

```bash
# Basic SimpleFlight commands
python unified_generator.py simpleflight                          # Single SimpleFlight drone
python unified_generator.py simpleflight --num-drones 5           # 5 SimpleFlight drones
python unified_generator.py simpleflight --num-drones 15 --layout circle  # 15 drones in circle formation
python unified_generator.py simpleflight --num-drones 27          # Maximum 27 drones

# With cameras and visualization
python unified_generator.py simpleflight --num-drones 3 --cameras front_rgb --create-rviz
python unified_generator.py simpleflight --num-drones 10 --cameras front_rgb downward_cam segmentation_cam

# Custom positioning and GPS
python unified_generator.py simpleflight --num-drones 8 --layout grid --gps-location 40.7128 -74.0060 10

# Output options
python unified_generator.py simpleflight --num-drones 5 --settings-only  # Only generate settings.json
python unified_generator.py simpleflight --num-drones 10 --output-dir ./my_simpleflight_config
```

### SimpleFlight System Requirements

| Configuration | Drones | RAM | CPU | Use Case |
|---------------|--------|-----|-----|----------|
| Development | 1-3 | 2GB | 2 cores | Algorithm development |
| Testing | 5-10 | 4GB | 4 cores | Swarm behavior testing |
| Research | 15-20 | 8GB | 6 cores | Advanced simulations |
| Maximum | 27 | 12GB+ | 8+ cores | Stress testing |

### SimpleFlight Client Connection Examples

```python
# Basic SimpleFlight connection (single drone)
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.armDisarm(True, "Drone1")
client.takeoffAsync(vehicle_name="Drone1")

# Multi-drone SimpleFlight connection
import airsim

drone_names = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5"]
client = airsim.MultirotorClient()
client.confirmConnection()

# Initialize all drones
for drone_name in drone_names:
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)

# Coordinate takeoff
takeoff_tasks = []
for drone_name in drone_names:
    task = client.takeoffAsync(vehicle_name=drone_name)
    takeoff_tasks.append(task)

# Wait for all takeoffs to complete
for task in takeoff_tasks:
    task.join()

print("All SimpleFlight drones airborne!")

# Swarm formation example
import numpy as np

# Move drones to formation positions
for i, drone_name in enumerate(drone_names):
    x = i * 10  # 10m spacing
    y = 0
    z = -20  # 20m altitude
    
    client.moveToPositionAsync(x, y, z, 5, vehicle_name=drone_name)
```

### SimpleFlight Integration Workflow

```bash
# Step 1: Generate SimpleFlight configuration
cd config_generator/tools
python unified_generator.py simpleflight --num-drones 8 --cameras front_rgb
python config_validator.py --all

# Step 2: Launch AirSim (Unreal Engine)
# No additional setup required - drones are immediately available

# Step 3: Connect and control via Python API
python your_swarm_script.py

# Step 4: Monitor and debug
# Use AirSim's built-in visualization and debugging tools
```

### SimpleFlight vs PX4 Comparison

| Feature | SimpleFlight | PX4 Ultra-Swarm |
|---------|-------------|-----------------|
| Setup Complexity | ✅ Minimal | ❌ Docker required |
| Startup Time | ✅ Instant | ⚠️ 1-2 minutes |
| Resource Usage | ✅ Low | ❌ High |
| Realism | ⚠️ Simplified physics | ✅ Full flight stack |
| Scale | 27 drones | 27 drones |
| External Tools | ❌ Limited QGC support | ✅ Full MAVLink ecosystem |
| Development Speed | ✅ Very fast | ⚠️ Slower iteration |
| Education | ✅ Perfect for learning | ❌ Complex for beginners |

### SimpleFlight Performance Tips

1. **Start Small**: Begin with 1-3 drones for development
2. **Memory Management**: Monitor memory usage with Task Manager/htop  
3. **Camera Optimization**: Only add cameras when needed for computer vision
4. **Layout Planning**: Use appropriate layouts (grid, line, circle) for your use case
5. **API Efficiency**: Batch operations when possible in your client code

## 🚁 Ultra-Swarm Advanced Usage

The ultra-swarm mode is designed for large-scale drone simulation research and testing. Here's comprehensive usage guidance:

### Ultra-Swarm Command Reference

```bash
# Basic ultra-swarm commands
python unified_generator.py ultra-swarm                                    # Full 27-drone setup (3x9)
python unified_generator.py ultra-swarm --num-swarms 2                     # 2 swarms of 9 drones each (18 total)
python unified_generator.py ultra-swarm --drones-per-swarm 5               # 3 swarms of 5 drones each (15 total)
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 3  # 2 swarms of 3 drones each (6 total)

# Output options
python unified_generator.py ultra-swarm --settings-only                   # Only generate settings.json
python unified_generator.py ultra-swarm --docker-only                     # Only generate docker-compose.yml
python unified_generator.py ultra-swarm --output-dir ./my_ultra_config    # Custom output directory

# With cameras and RViz
python unified_generator.py ultra-swarm --cameras front_rgb --create-rviz # Add cameras to all drones
```

### Ultra-Swarm System Requirements

| Configuration | Drones | RAM | CPU | Storage | Use Case |
|---------------|--------|-----|-----|---------|----------|
| Testing | 6 (2×3) | 4GB | 4 cores | 2GB | Algorithm development |
| Research | 10 (2×5) | 8GB | 8 cores | 4GB | Swarm behavior studies |
| Competition | 18 (2×9) | 16GB | 12 cores | 8GB | Large-scale coordination |
| Maximum | 27 (3×9) | 24GB+ | 16+ cores | 12GB | Ultimate stress testing |

### Ultra-Swarm Port Allocation Details

The ultra-swarm mode uses intelligent port allocation to prevent conflicts:

```bash
# Swarm 1 (Blue Team) - Seattle area
PX4_Swarm1_Drone1: TCP 4561, QGC UDP 14550, MAVLink UDP 18570
PX4_Swarm1_Drone2: TCP 4562, QGC UDP 14551, MAVLink UDP 18571
...
PX4_Swarm1_Drone9: TCP 4569, QGC UDP 14558, MAVLink UDP 18578

# Swarm 2 (Red Team) - Bellevue area  
PX4_Swarm2_Drone1: TCP 4571, QGC UDP 14560, MAVLink UDP 18580
PX4_Swarm2_Drone2: TCP 4572, QGC UDP 14561, MAVLink UDP 18581
...
PX4_Swarm2_Drone9: TCP 4579, QGC UDP 14568, MAVLink UDP 18588

# Swarm 3 (Green Team) - Redmond area
PX4_Swarm3_Drone1: TCP 4581, QGC UDP 14570, MAVLink UDP 18590
PX4_Swarm3_Drone2: TCP 4582, QGC UDP 14571, MAVLink UDP 18591
...
PX4_Swarm3_Drone9: TCP 4589, QGC UDP 14578, MAVLink UDP 18598
```

### Ultra-Swarm Integration Workflow

```bash
# Step 1: Generate ultra-swarm configuration
cd config_generator/tools
python unified_generator.py ultra-swarm --num-swarms 2 --drones-per-swarm 4
python config_validator.py --all

# Step 2: Use with ultra-swarm Docker setup (if available)
cd ../../docker/px4_airsim_docker_v2
cp ../../ultra_swarm/settings.json .

# Step 3: Launch ultra-swarm
./start-ultra-swarms.sh dual-swarms  # Starts swarms 1 and 2

# Step 4: Monitor and manage
./start-ultra-swarms.sh status       # Check status
./start-ultra-swarms.sh health       # Health checks  
./start-ultra-swarms.sh ports        # Port allocations
```

### Ultra-Swarm Client Connection Examples

```python
# Python client example for ultra-swarm
import cosysairsim as airsim

# Connect to Swarm 1 (Blue Team)
swarm1_clients = []
for drone_id in range(1, 5):  # Drones 1-4
    port = 4560 + drone_id
    client = airsim.MultirotorClient(port=port)
    client.confirmConnection()
    swarm1_clients.append(client)
    print(f"Connected to Swarm 1, Drone {drone_id} on port {port}")

# Connect to Swarm 2 (Red Team)  
swarm2_clients = []
for drone_id in range(1, 5):  # Drones 1-4
    port = 4570 + drone_id
    client = airsim.MultirotorClient(port=port)
    client.confirmConnection()
    swarm2_clients.append(client)
    print(f"Connected to Swarm 2, Drone {drone_id} on port {port}")

# Coordinate multi-swarm operations
for clients in [swarm1_clients, swarm2_clients]:
    for client in clients:
        client.armDisarm(True)
        client.takeoffAsync()
```

### Ultra-Swarm Performance Tips

1. **Incremental Scaling**: Always test with fewer drones first
2. **Resource Monitoring**: Use `docker stats` to monitor container resource usage
3. **Network Optimization**: Consider Docker network configuration for high drone counts
4. **Selective Startup**: Use swarm profiles to start only needed swarms
5. **Cleanup**: Regularly clean up stopped containers: `docker system prune`

## 🎯 Choosing the Right Configuration Mode

### Decision Matrix

| Use Case | Recommended Mode | Why? |
|----------|------------------|------|
| **Learning AirSim** | `simpleflight --num-drones 1` | No setup complexity, instant results |
| **Algorithm Development** | `simpleflight --num-drones 3-5` | Fast iteration, minimal overhead |
| **Computer Vision Research** | `simpleflight --cameras front_rgb` | Built-in camera support, no Docker |
| **Swarm Behavior Testing** | `simpleflight --num-drones 10-15` | Lightweight multi-drone testing |
| **Educational Workshops** | `simpleflight --num-drones 5` | Easy for students, no Docker knowledge needed |
| **Realistic Flight Testing** | `multi --num-drones 3-5` | Full PX4 flight stack with MAVLink |
| **Hardware-in-Loop Prep** | `single` (PX4) | Matches real drone behavior |
| **Competition Preparation** | `ultra-swarm --num-swarms 2` | Large-scale coordination testing |
| **Research Publications** | `ultra-swarm` (full 27 drones) | Maximum scale demonstrations |
| **Mixed Vehicle Research** | `mixed` | Cars + drones in same simulation |

### Quick Start Recommendations

```bash
# 🎓 New to AirSim? Start here:
python unified_generator.py simpleflight --num-drones 1

# 🧪 Testing swarm algorithms?
python unified_generator.py simpleflight --num-drones 5 --layout grid

# 📸 Computer vision research?
python unified_generator.py simpleflight --num-drones 3 --cameras front_rgb downward_cam

# 🚁 Need realistic flight dynamics?
python unified_generator.py multi --num-drones 3

# 🏆 Going for maximum scale?
python unified_generator.py ultra-swarm --num-swarms 3 --drones-per-swarm 9
```

## 🔮 Future Enhancements

- **Web UI Configuration Builder** - Browser-based configuration tool
- **Mission Integration** - Direct export to mission planners
- **Configuration Profiles** - Save and reuse common setups
- **Auto-scaling** - Dynamic resource-based scaling
- **Cloud Integration** - Deploy to cloud container services
- **Ultra-Swarm Optimization** - Performance tuning for 50+ drone configurations
- **SimpleFlight Extensions** - Enhanced physics and sensor models for SimpleFlight

## 📞 Support

1. **Check validation first**: `python config_validator.py --all`
2. **Review documentation**: `docs/STANDARDIZED_WORKFLOW.md`
3. **Try examples**: `python examples/workflow_example.py`
4. **Check templates**: Review files in `templates/`
5. **Migration help**: `python tools/migrate_to_unified.py`

---

**The unified configuration generator provides a standardized, conflict-free, and production-ready approach to AirSim configuration management. With both SimpleFlight (lightweight, no Docker) and PX4 Ultra-Swarm (realistic, full flight stack) support, you can choose the perfect balance between simplicity and realism for your specific use case. Always validate your configurations before deployment!**