# 🚁 Standardized AirSim Configuration Workflow

This document provides a comprehensive guide for the standardized workflow to generate AirSim settings.json and Docker Compose configurations using the unified generator tools.

## 📋 Overview

The standardized workflow consists of three main tools:

1. **`unified_generator.py`** - Main configuration generator
2. **`config_validator.py`** - Configuration validation tool  
3. **Legacy tools** - `generate_settings.py` and `simple_generator.py` (maintained for compatibility)

## 🎯 Quick Start

### Basic Multi-Drone Setup (Recommended)

```bash
# Generate 3 drones with Docker support
cd /path/to/Cosys-AirSim/docker/px4_airsim_docker/settings
python unified_generator.py multi --num-drones 3

# Validate the configuration
python config_validator.py --all

# Launch the system
cd ..
./launch.bat  # Windows
# or ./launch.sh  # Linux
```

### Single Drone Setup

```bash
# Generate single drone configuration
python unified_generator.py single

# Validate and launch
python config_validator.py --all
cd .. && ./launch.bat
```

## 🔧 Detailed Usage

### Unified Generator (`unified_generator.py`)

The unified generator is the primary tool for creating standardized configurations. It generates both `settings.json` and `docker-compose.yml` files with consistent port assignments and proper vehicle configurations.

#### Command Syntax

```bash
python unified_generator.py <mode> [options]
```

#### Available Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `single` | Single drone configuration | Basic testing, development |
| `multi` | Multiple drones | Swarm simulation, multi-agent testing |
| `mixed` | Mixed vehicle types | Complex scenarios with drones and cars |
| `custom` | Custom configuration | Advanced users (future feature) |

#### Common Options

| Option | Description | Default | Example |
|--------|-------------|---------|---------|
| `--num-drones N` | Number of drones (multi mode) | 3 | `--num-drones 5` |
| `--layout TYPE` | Drone layout pattern | grid | `--layout circle` |
| `--output-dir DIR` | Output directory | current | `--output-dir ./configs` |
| `--settings-only` | Generate only settings.json | false | `--settings-only` |
| `--docker-only` | Generate only docker-compose.yml | false | `--docker-only` |
| `--platform OS` | Target platform for launcher | auto-detect | `--platform linux` |
| `--gps-location LAT LON ALT` | GPS origin coordinates | Seattle | `--gps-location 40.7128 -74.0060 10` |

#### Examples

```bash
# Basic examples
python unified_generator.py single
python unified_generator.py multi --num-drones 5
python unified_generator.py mixed

# Advanced examples
python unified_generator.py multi --num-drones 8 --layout circle --gps-location 37.7749 -122.4194 50
python unified_generator.py single --output-dir ./test_config --platform linux
python unified_generator.py multi --num-drones 3 --settings-only
```

#### Layout Options

- **`grid`** (default) - Arranges drones in a grid pattern (5 per row)
- **`line`** - Arranges drones in a single line (5m spacing)  
- **`circle`** - Arranges drones in a circle (10m radius)

### Configuration Validator (`config_validator.py`)

The validator checks configurations for common issues, port conflicts, and system prerequisites.

#### Command Syntax

```bash
python config_validator.py [options]
```

#### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--settings FILE` | Settings file to validate | settings.json |
| `--compose FILE` | Docker compose file to validate | docker-compose.yml |
| `--compatibility` | Check file compatibility | false |
| `--system` | Check system prerequisites | false |
| `--all` | Run all validations | false |

#### Examples

```bash
# Validate all configurations
python config_validator.py --all

# Check specific files
python config_validator.py --settings my_settings.json --compose my_compose.yml

# Check only system prerequisites
python config_validator.py --system

# Check compatibility between files
python config_validator.py --compatibility
```

#### Validation Levels

- **🚨 CRITICAL** - Must be fixed before proceeding
- **❌ ERROR** - Should be fixed for proper operation
- **⚠️ WARNING** - May cause issues, recommended to fix
- **ℹ️ INFO** - Informational, no action required

## 📁 Generated Files

### Standard Output Files

When you run the unified generator, it creates:

```
output-directory/
├── settings.json              # AirSim configuration
├── docker-compose.yml         # Docker container definitions
└── launch.bat/.sh            # Platform-specific launcher script
```

### File Locations

- **`settings.json`** - Generated in output directory AND copied to `~/Documents/AirSim/settings.json`
- **`docker-compose.yml`** - Generated in output directory
- **Launcher script** - Generated in output directory with appropriate permissions

## 🔄 Complete Workflow

### 1. Plan Your Configuration

Decide on your simulation requirements:

- **Vehicle types**: Multirotors, cars, or mixed
- **Number of vehicles**: 1-10 recommended  
- **Layout**: How vehicles should be positioned
- **GPS location**: Real-world coordinates if needed

### 2. Generate Configuration

```bash
# Choose appropriate command for your needs
python unified_generator.py multi --num-drones 4 --layout grid

# For custom GPS location (e.g., New York City)
python unified_generator.py multi --num-drones 3 --gps-location 40.7128 -74.0060 10
```

### 3. Validate Configuration

```bash
# Run comprehensive validation
python config_validator.py --all

# Fix any CRITICAL or ERROR issues reported
# Review WARNING items and fix if needed
```

### 4. Launch System

```bash
# Move to Docker directory
cd ..

# Launch containers (Windows)
./launch.bat

# Or launch containers (Linux)
./launch.sh
```

### 5. Start AirSim

1. Launch your AirSim environment (Blocks, custom map, etc.)
2. The generated `settings.json` will be automatically loaded
3. Vehicles will connect to the Docker containers automatically

### 6. Monitor and Control

```bash
# Check container status
docker-compose ps

# View logs
docker-compose logs

# Access specific vehicle
docker exec -it px4-drone1 bash

# Stop all containers
docker-compose down
```

## 🚀 Advanced Scenarios

### Multi-GPS Origin Setup

For large-scale simulations across different geographic regions:

```bash
# Generate configurations for different regions
python unified_generator.py multi --num-drones 3 --gps-location 47.6062 -122.3321 10 --output-dir ./seattle
python unified_generator.py multi --num-drones 3 --gps-location 40.7128 -74.0060 10 --output-dir ./nyc
python unified_generator.py multi --num-drones 3 --gps-location 34.0522 -118.2437 10 --output-dir ./la
```

### Mixed Vehicle Simulation

```bash
# Generate mixed vehicle setup
python unified_generator.py mixed

# Manually edit the configuration for specific requirements
# The mixed mode provides a starting template
```

### Custom Port Ranges

If default ports conflict with your system, modify the base ports in `unified_generator.py`:

```python
# In ConfigGenerator.__init__()
self.base_tcp_port = 5560      # Instead of 4560
self.base_control_local = 15540 # Instead of 14540
self.base_control_remote = 15580 # Instead of 14580
self.base_qgc_port = 15549     # Instead of 14549
```

### High-Density Drone Swarms

For more than 10 drones, you'll need to:

1. Increase the limit in `unified_generator.py`
2. Ensure adequate network resources
3. Consider using multiple Docker hosts

```bash
# Example for 20 drones (after modifying limits)
python unified_generator.py multi --num-drones 20 --layout grid
```

## 🔧 Configuration Templates

### Template Files Generated

The unified generator creates different templates based on the mode:

#### Single Drone Template
- 1 PX4 multirotor
- Basic sensor suite (GPS, IMU, magnetometer, barometer)
- Standard port configuration
- Default Seattle GPS coordinates

#### Multi-Drone Template  
- N PX4 multirotors (configurable 1-10)
- Grid, line, or circle positioning
- Automatic port assignment (no conflicts)
- Shared GPS origin with unique positions

#### Mixed Vehicle Template
- 1 PX4 multirotor with Docker support
- 1 PhysX car (no Docker - runs in AirSim directly)
- Different sensor configurations per vehicle type
- Compatible port assignments

### Customizing Templates

To create your own templates, modify the template functions in `unified_generator.py`:

```python
def create_custom_config() -> SimulationConfig:
    """Create your custom configuration"""
    config = SimulationConfig()
    
    # Add your vehicles
    vehicle = VehicleConfig(
        name="MyDrone",
        vehicle_type=VehicleType.PX4_MULTIROTOR,
        position=Position(x=0, y=0, z=-2),
        sensors=SensorConfig(
            gps=GPSConfig(enabled=True),
            # Add custom sensors
            lidar_enabled=True,
            distance_enabled=True
        ),
        ports=NetworkPorts(
            tcp_port=4561,
            control_port_local=14541,
            control_port_remote=14581,
            qgc_port=14550
        )
    )
    
    config.vehicles.append(vehicle)
    return config
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### Port Conflicts
```bash
# Check what's using ports
netstat -an | grep 4561
netstat -an | grep 14541

# Validate port assignments
python config_validator.py --all
```

#### Docker Issues
```bash
# Check Docker status
docker --version
docker ps

# Clean up old containers
docker system prune -f

# Rebuild containers
docker-compose build --no-cache
```

#### GPS/Connection Issues
```bash
# Check container logs
docker-compose logs px4-drone1

# Look for successful GPS lock
# Should see: "INFO [tone_alarm] home set"

# Test AirSim API connectivity
curl http://localhost:41451/ping
```

#### File Permission Issues
```bash
# Linux: Make launcher executable
chmod +x launch.sh

# Windows: Run as administrator if needed
```

### Validation Error Reference

| Error Type | Cause | Solution |
|------------|-------|----------|
| Port conflict | Multiple vehicles using same port | Regenerate with `unified_generator.py` |
| Missing GPS | PX4 vehicle without GPS sensor | Add GPS sensor to vehicle config |
| Invalid JSON | Malformed settings.json | Validate JSON syntax |
| Docker unavailable | Docker not installed/running | Install Docker and start service |
| Position conflict | Multiple vehicles at same location | Use different layouts or spacing |

### Recovery Procedures

#### Reset to Working Configuration
```bash
# Generate fresh single drone config
python unified_generator.py single --output-dir ./backup

# Copy working files
cp ./backup/* .

# Validate
python config_validator.py --all
```

#### Clean Docker Environment
```bash
# Stop all containers
docker-compose down

# Remove all AirSim containers
docker container prune -f

# Remove AirSim network
docker network rm airsim-network 2>/dev/null || true

# Start fresh
./launch.bat
```

## 📚 Legacy Tool Compatibility

The standardized workflow maintains compatibility with existing tools:

### Legacy `generate_settings.py`
```bash
# Still works for simple multi-drone setups
python generate_settings.py 3

# Equivalent unified command
python unified_generator.py multi --num-drones 3 --settings-only
```

### Legacy `simple_generator.py`
```bash
# Still works for Docker compose generation
python simple_generator.py

# Equivalent unified command  
python unified_generator.py multi --num-drones 3 --docker-only
```

### Migration Path

To migrate from legacy tools:

1. **Backup existing configurations**
2. **Test with unified generator**
3. **Validate results**
4. **Update scripts to use unified tool**

```bash
# Backup
cp settings.json settings.json.backup
cp docker-compose.yml docker-compose.yml.backup

# Generate with unified tool
python unified_generator.py multi --num-drones 3

# Validate
python config_validator.py --all

# Test launch
./launch.bat
```

## 🔮 Future Enhancements

### Planned Features

- **Custom mode implementation** - Full custom configuration support
- **Web UI generator** - Browser-based configuration builder
- **Configuration profiles** - Save/load common setups
- **Auto-scaling support** - Dynamic drone count based on resources
- **Integration with mission planners** - Direct export to QGroundControl

### Contributing

To contribute to the standardized workflow:

1. **Test thoroughly** with various configurations
2. **Document changes** in this README
3. **Add validation rules** to `config_validator.py`
4. **Update templates** in `unified_generator.py`
5. **Maintain backward compatibility** with legacy tools

---

## 📞 Support

For issues with the standardized workflow:

1. **Run validation first**: `python config_validator.py --all`
2. **Check the troubleshooting section** above
3. **Review container logs**: `docker-compose logs`
4. **Test with single drone** if multi-drone fails
5. **Create minimal reproduction case** for bug reports

**Remember**: The unified generator provides a standardized, conflict-free approach to AirSim configuration management. Always validate your configurations before deployment!