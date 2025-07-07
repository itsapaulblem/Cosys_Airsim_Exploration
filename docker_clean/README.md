# Cosys-AirSim Docker Setup - Clean Organization

This directory contains a cleaned up and reorganized Docker setup for Cosys-AirSim with PX4 integration.

## Directory Structure

```
docker_clean/
├── config_generator/       # 🆕 Unified configuration generator tools
│   ├── tools/             # Core generation and validation tools
│   │   ├── unified_generator.py    # Main configuration generator
│   │   ├── config_validator.py     # Configuration validation tool
│   │   └── migrate_to_unified.py   # Migration from legacy tools
│   ├── templates/         # Reference templates and examples
│   ├── examples/          # Interactive workflow examples
│   ├── docs/              # Comprehensive documentation
│   └── README.md          # Configuration generator guide
├── single_drone/          # Single drone configuration
│   ├── docker-compose.yml
│   └── start_single.bat
├── multi_drone/           # Multi-drone configuration  
│   ├── docker-compose.yml
│   └── start_multi.bat
├── airsim_ros2/          # ROS2 integration (future)
├── testing_scripts/      # Testing and monitoring tools
│   ├── test_connections.py
│   ├── monitor_containers.bat
│   └── cleanup_all.bat
├── common/               # Shared files
│   ├── Dockerfile
│   └── scripts/
└── README.md            # This file
```

## Quick Start

### 🆕 Generate Custom Configurations (Recommended)
```bash
# Navigate to configuration generator
cd config_generator/tools

# Generate single drone setup
python unified_generator.py single

# Generate 3-drone setup with automatic ports
python unified_generator.py multi --num-drones 3

# Validate any configuration
python config_validator.py --all

# Interactive examples and tutorials
cd ../examples
python workflow_example.py
```

### Use Pre-built Configurations

#### Single Drone
```bash
cd single_drone
start_single.bat
```

#### Multi Drone
```bash
cd multi_drone
start_multi.bat 3    # Start 3 drones
start_multi.bat all  # Start all 5 drones
```

### Testing
```bash
cd testing_scripts
python test_connections.py    # Test all connections
monitor_containers.bat        # Monitor container status
cleanup_all.bat              # Clean shutdown
```

## Key Improvements

### New Configuration Generator (v2.0)
1. **🆕 Unified Generation**: Single tool creates both settings.json AND docker-compose.yml
2. **🆕 Automatic Port Management**: Zero-conflict port assignment
3. **🆕 Multiple Vehicle Types**: Support for multirotors, cars, and mixed configurations  
4. **🆕 Flexible Layouts**: Grid, line, and circle positioning patterns
5. **🆕 Comprehensive Validation**: Built-in error checking and system verification
6. **🆕 Migration Support**: Seamless upgrade from legacy tools

### Fixed Issues (v1.0)
1. **Profile Conflicts**: Eliminated overlapping profiles that caused port conflicts
2. **Port Mapping**: Clear, non-conflicting port assignments
3. **Container Dependencies**: Proper startup order and isolation
4. **Network Configuration**: Consistent network settings

### Port Assignments
- **Single Drone**: 4561 (AirSim), 14550 (QGC), 14541/14581 (MAVLink)
- **Multi Drone**:
  - Drone 1: 4561, 14550, 14541/14581
  - Drone 2: 4562, 14551, 14542/14582  
  - Drone 3: 4563, 14552, 14543/14583
  - Drone 4: 4564, 14553, 14544/14584
  - Drone 5: 4565, 14554, 14545/14585

### Profile Structure
- **Single**: `single` profile only
- **Multi**: `drone-1`, `drone-2`, etc. (no overlaps)
- **Convenience**: `all` profile for all drones

## Testing and Debugging

### Connection Testing
The `test_connections.py` script automatically:
- Detects running containers
- Tests all TCP/UDP connections
- Generates appropriate AirSim settings.json
- Provides connection status summary

### Container Monitoring  
The `monitor_containers.bat` script provides:
- Real-time container status
- Port mapping overview
- Auto-refresh every 10 seconds

### Cleanup
The `cleanup_all.bat` script ensures:
- All containers stopped
- Networks cleaned
- No orphaned resources

## Usage Examples

### Start 2 drones only:
```bash
cd multi_drone
start_multi.bat 2
```

### Start specific drone:
```bash
cd multi_drone  
start_multi.bat drone-3
```

### Test connections:
```bash
cd testing_scripts
python test_connections.py
```

### Monitor in real-time:
```bash
cd testing_scripts
monitor_containers.bat
```

### Clean shutdown:
```bash
cd testing_scripts
cleanup_all.bat
```

## Migration from Old Setup

### From Legacy Tools (docker/px4_airsim_docker/)

The old setup had scattered tools and profile conflicts. The new unified workflow provides:

1. **🆕 Single Tool**: Replaces `generate_settings.py` + `simple_generator.py` + manual coordination
2. **🆕 Zero Conflicts**: Automatic port and resource management
3. **🆕 Validation**: Built-in error checking and system verification
4. **🆕 Migration Path**: Automated upgrade from legacy configurations

**Quick Migration:**
```bash
# From the config_generator/tools directory
python migrate_to_unified.py    # Analyze and migrate existing configs
python config_validator.py --all # Validate migrated setup
```

### From Old docker_clean (v1.0)

The original docker_clean structure is preserved, with new v2.0 tools added:

1. **Backward Compatible**: All existing scripts still work
2. **Enhanced Generation**: New unified tools for custom configurations
3. **Better Testing**: Improved validation and debugging
4. **Cleaner Organization**: Logical separation of tools and configs

## Next Steps

### For New Users
1. **Start with Config Generator**: `cd config_generator/tools && python unified_generator.py single`
2. **Learn the Workflow**: `cd config_generator/examples && python workflow_example.py`
3. **Validate Everything**: `python config_validator.py --all`
4. **Test Integration**: Launch AirSim and verify connections

### For Advanced Users
1. **Migrate Legacy Setups**: `python migrate_to_unified.py`
2. **Custom Configurations**: Use advanced generator options
3. **Multi-environment Testing**: Generate configs for different scenarios
4. **ROS2 Integration**: Combine with airsim_ros2 wrapper
5. **Automation**: Script your configuration generation workflow

### Development Roadmap
- [ ] Web UI configuration builder
- [ ] Mission planner integration  
- [ ] Cloud deployment templates
- [ ] Auto-scaling based on resources
- [ ] Configuration version control integration