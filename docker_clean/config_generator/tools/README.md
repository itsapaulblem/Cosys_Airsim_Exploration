# Configuration Generator Tools

**Core tools for generating and validating AirSim configurations**

## Quick Reference

```bash
# Generate configurations
python unified_generator.py single                    # Single drone
python unified_generator.py multi --num-drones 3      # 3 drones
python unified_generator.py mixed                     # Mixed vehicles

# Generate with cameras
python unified_generator.py single --cameras front_rgb front_depth --create-rviz
python unified_generator.py multi --num-drones 2 --cameras front_rgb segmentation_cam --create-rviz

# Validate configurations  
python config_validator.py --all                      # Validate everything
python config_validator.py --system                   # Check prerequisites

# Migration support
python migrate_to_unified.py                          # Migrate from legacy tools
```

## Tools Overview

| Tool | Purpose | Usage |
|------|---------|-------|
| `unified_generator.py` | Main configuration generator | `python unified_generator.py <mode> [options]` |
| `config_validator.py` | Configuration validation | `python config_validator.py [options]` |
| `migrate_to_unified.py` | Legacy tool migration | `python migrate_to_unified.py` |

## Common Commands

### Single Drone Development
```bash
python unified_generator.py single
python config_validator.py --all
```

### Multi-Drone Research  
```bash
python unified_generator.py multi --num-drones 5 --layout grid
python config_validator.py --all
```

### Camera Integration
```bash
python unified_generator.py single --cameras front_rgb front_depth --create-rviz
python unified_generator.py multi --num-drones 2 --cameras segmentation_cam --create-rviz
```

### Custom GPS Location
```bash
python unified_generator.py multi --num-drones 3 --gps-location 40.7128 -74.0060 10
python config_validator.py --compatibility
```

### Troubleshooting
```bash
python config_validator.py --system                   # Check system
python migrate_to_unified.py                          # Migration help
python unified_generator.py --help                    # Tool help
```

## Generated Files

- **`settings.json`** - AirSim configuration (auto-copied to ~/Documents/AirSim/)
- **`docker-compose.yml`** - Docker container definitions
- **`launch.bat/.sh`** - Platform-specific launcher script
- **`airsim_cameras.rviz`** - RViz2 configuration (when --create-rviz used)

## Next Steps

1. **Generate** your configuration
2. **Validate** with config_validator.py
3. **Launch** containers with generated launcher
4. **Start** AirSim/Unreal Engine
5. **Test** your setup

For detailed usage instructions, see `../USAGE_GUIDE.md` or `../docs/STANDARDIZED_WORKFLOW.md`.