# 🚀 Quick Usage Guide - AirSim Configuration Generator

**Get started with the unified configuration generator in 5 minutes**

## Prerequisites Check

```bash
# Ensure you have Python 3.7+ and Docker
python --version        # Should be 3.7+
docker --version        # Should be running
pip install pyyaml      # Install required dependency
```

## 🎯 Most Common Use Cases

### 1. Single Drone Development

```bash
cd tools
python unified_generator.py single
python config_validator.py --all
```

**Generated files:**
- `settings.json` (copied to ~/Documents/AirSim/)
- `docker-compose.yml`
- `launch.bat` (Windows) or `launch.sh` (Linux)

### 2. Multi-Drone Testing (3 drones)

```bash
cd tools
python unified_generator.py multi --num-drones 3
python config_validator.py --all
```

### 3. Research Setup with Custom GPS

```bash
cd tools
# Tokyo coordinates example
python unified_generator.py multi --num-drones 4 --gps-location 35.6762 139.6503 10
python config_validator.py --all
```

### 4. Mixed Vehicle Simulation

```bash
cd tools
python unified_generator.py mixed
python config_validator.py --all
```

## 🔧 Common Command Patterns

### Generate with Different Layouts

```bash
# Grid formation (default)
python unified_generator.py multi --num-drones 5 --layout grid

# Single line formation
python unified_generator.py multi --num-drones 5 --layout line

# Circular formation
python unified_generator.py multi --num-drones 5 --layout circle
```

### Custom Output Directory

```bash
# Generate in specific directory
python unified_generator.py single --output-dir ./my_experiment

# Generate only settings (no Docker)
python unified_generator.py single --settings-only

# Generate only Docker compose (no settings)
python unified_generator.py single --docker-only
```

### Platform-Specific Generation

```bash
# Force Linux launcher script
python unified_generator.py single --platform linux

# Force Windows launcher script  
python unified_generator.py single --platform windows
```

## 🔍 Validation and Troubleshooting

### Basic Validation

```bash
# Check everything
python config_validator.py --all

# Check only settings.json
python config_validator.py --settings settings.json

# Check only docker-compose.yml
python config_validator.py --compose docker-compose.yml

# Check system prerequisites
python config_validator.py --system
```

### Understanding Validation Output

- 🚨 **CRITICAL** - Must fix before proceeding
- ❌ **ERROR** - Should fix for proper operation
- ⚠️ **WARNING** - May cause issues  
- ℹ️ **INFO** - Informational only

### Common Issues and Fixes

#### Port Conflicts
```bash
# Problem: Ports already in use
# Solution: Check what's using them
netstat -an | grep 4561

# Or regenerate with different base ports
# (Edit unified_generator.py base port values)
```

#### Docker Not Available
```bash
# Problem: Docker not running
# Solution: Start Docker Desktop
docker ps   # Should show running containers
```

#### Permission Issues
```bash
# Problem: Cannot write files
# Solution: Check directory permissions
ls -la      # Linux
# Or run as administrator (Windows)
```

## 📝 File Locations

### Generated Files
- **Settings**: `./settings.json` AND `~/Documents/AirSim/settings.json`
- **Docker Compose**: `./docker-compose.yml`
- **Launcher**: `./launch.bat` or `./launch.sh`

### Templates (Reference)
- **Single Drone**: `../templates/airsim_px4_single_settings.json`
- **Multi-Drone**: `../templates/airsim_px4_multi_settings.json`
- **Dockerfile**: `../templates/Dockerfile`

## 🔄 Typical Workflow

### Development Workflow
1. **Generate** configuration for your needs
2. **Validate** the generated files
3. **Launch** Docker containers
4. **Start** AirSim/Unreal Engine
5. **Test** your setup
6. **Iterate** as needed

### Example Session
```bash
# Step 1: Navigate to tools
cd docker_clean/config_generator/tools

# Step 2: Generate configuration
python unified_generator.py multi --num-drones 2 --layout line

# Step 3: Validate
python config_validator.py --all

# Step 4: Review generated files
ls -la
cat settings.json        # Check settings
cat docker-compose.yml   # Check Docker setup

# Step 5: Launch (example for Windows)
./launch.bat

# Step 6: In another terminal, start AirSim
# (Launch your AirSim environment)

# Step 7: Test connection
cd ../examples
python workflow_example.py
```

## 🛠️ Advanced Usage

### Custom Vehicle Positions
```bash
# Generate specific layout for your needs
python unified_generator.py multi --num-drones 3 --layout grid

# Then manually edit settings.json for fine-tuning
# Vehicles are positioned at X, Y, Z coordinates
```

### Batch Generation
```bash
# Generate multiple configurations
python unified_generator.py single --output-dir ./configs/single
python unified_generator.py multi --num-drones 3 --output-dir ./configs/multi3  
python unified_generator.py multi --num-drones 5 --output-dir ./configs/multi5
```

### Integration with Scripts
```bash
#!/bin/bash
# setup_experiment.sh

echo "Setting up 4-drone circular formation..."
cd config_generator/tools
python unified_generator.py multi --num-drones 4 --layout circle --output-dir ../../experiment_1

echo "Validating configuration..."
cd ../../experiment_1
../config_generator/tools/python config_validator.py --all

echo "Starting containers..."
./launch.sh

echo "Experiment setup complete!"
```

## 🔍 Debugging Tips

### Check Generated Ports
```bash
# After generation, check port assignments
grep -E "(Port|port)" settings.json
grep -E "ports:" docker-compose.yml
```

### Test System Prerequisites
```bash
# Comprehensive system check
python config_validator.py --system

# Manual checks
docker ps                    # Docker working
python --version            # Python version
pip show pyyaml             # Required package
netstat -an | grep 4561     # Check if ports free
```

### Validate File Syntax
```bash
# Check JSON syntax
python -m json.tool settings.json

# Check YAML syntax  
python -c "import yaml; yaml.safe_load(open('docker-compose.yml'))"
```

## 📞 Getting Help

### Interactive Help
```bash
# Built-in help
python unified_generator.py --help
python config_validator.py --help

# Interactive examples
cd ../examples
python workflow_example.py
```

### Documentation
- **Comprehensive Guide**: `docs/STANDARDIZED_WORKFLOW.md`
- **Config Generator README**: `README.md`
- **Migration Help**: `python migrate_to_unified.py`

### Common Command Reference
```bash
# Quick reference for daily use
python unified_generator.py single                    # Single drone
python unified_generator.py multi --num-drones 3      # 3 drones  
python unified_generator.py mixed                     # Mixed vehicles
python config_validator.py --all                      # Validate all
python migrate_to_unified.py                          # Migrate legacy
python ../examples/workflow_example.py                # Learn interactively
```

---

**Remember: Always validate your configurations with `python config_validator.py --all` before deploying!**