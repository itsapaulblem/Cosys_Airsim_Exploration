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
- PyYAML package: `pip install pyyaml`

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
python unified_generator.py single                    # Single drone
python unified_generator.py multi --num-drones 5      # 5 drones
python unified_generator.py mixed                     # Mixed vehicles

# Camera integration
python unified_generator.py single --cameras front_rgb front_depth --create-rviz
python unified_generator.py multi --num-drones 2 --cameras front_rgb segmentation_cam --create-rviz

# Advanced options
python unified_generator.py multi --num-drones 3 --layout circle --gps-location 40.7128 -74.0060 10
python unified_generator.py single --output-dir ./my_config --platform linux
```

**Features:**
- ✅ Generates both settings.json AND docker-compose.yml
- ✅ **Camera integration** with 7 presets and RViz2 support
- ✅ **SubWindows** for live camera feeds in Unreal
- ✅ **BP_SpiritPawn** as default quadrotor pawn
- ✅ Automatic port conflict prevention
- ✅ Multiple vehicle types (multirotors, cars, mixed)
- ✅ Flexible positioning layouts (grid, line, circle)
- ✅ Custom GPS coordinates
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

### Example 3: Custom GPS Location (Tokyo)

```bash
cd tools
python unified_generator.py multi --num-drones 2 --gps-location 35.6762 139.6503 10
python config_validator.py --compatibility
```

### Example 4: Interactive Learning

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
- **PX4Multirotor** - PX4 SITL drones with Docker
- **ArduCopter** - ArduCopter drones
- **SimpleFlight** - Built-in AirSim flight controller
- **PhysXCar** - Ground vehicles
- **ComputerVision** - Camera-only mode

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
- Scale up to multi-drone
- Test each step thoroughly

### 4. Document Your Setups
```bash
# Generate configurations with descriptive names
python unified_generator.py multi --num-drones 3 --output-dir ./formations/triangle_formation
```

### 5. Clean Up Regularly
```bash
# Use the validator to check for orphaned configurations
python config_validator.py --system
```

## 🚨 Troubleshooting

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

## 🔮 Future Enhancements

- **Web UI Configuration Builder** - Browser-based configuration tool
- **Mission Integration** - Direct export to mission planners
- **Configuration Profiles** - Save and reuse common setups
- **Auto-scaling** - Dynamic resource-based scaling
- **Cloud Integration** - Deploy to cloud container services

## 📞 Support

1. **Check validation first**: `python config_validator.py --all`
2. **Review documentation**: `docs/STANDARDIZED_WORKFLOW.md`
3. **Try examples**: `python examples/workflow_example.py`
4. **Check templates**: Review files in `templates/`
5. **Migration help**: `python tools/migrate_to_unified.py`

---

**The unified configuration generator provides a standardized, conflict-free, and production-ready approach to AirSim configuration management. Always validate your configurations before deployment!**