# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important Instruction Reminders

### Core Development Principles
- Do what has been asked; nothing more, nothing less
- NEVER create files unless they're absolutely necessary for achieving your goal
- ALWAYS prefer editing an existing file to creating a new one
- NEVER proactively create documentation files (*.md) or README files. Only create documentation files if explicitly requested by the User

### Tool Usage Priority
1. Use Gemini CLI MCP (mcp__gemini-cli__ask-gemini) for:
   - Large codebase analysis and architecture planning
   - Multi-file reviews and cross-file pattern analysis
   - PDF processing and document analysis
   - When context window might exceed Claude's limit
   - Understanding complex system interactions across multiple components
2. Use direct tools (Read, Edit, Bash) for targeted operations on specific files
3. Use Task tool for searching when dealing with ambiguous file locations

### When to Use Gemini CLI
- Analyzing the entire AirLib architecture across multiple files
- Understanding sensor implementations across base classes and implementations
- Reviewing vehicle API implementations across different vehicle types
- Processing documentation PDFs or large markdown files
- When needing to analyze more than 5-10 files simultaneously
- When the combined file sizes might exceed 100KB

## Essential Commands

### Build Commands

#### Windows
```bash
# First time setup - run from x64 Native Tools Command Prompt for VS 2022
setup.bat

# Build the project
build.cmd                    # Build both Debug and Release
build.cmd --Debug           # Build Debug only
build.cmd --Release         # Build Release only
build.cmd --RelWithDebInfo  # Build with debug info
build.cmd --no-full-poly-car # Build without high-poly car assets
```

#### Linux
```bash
# First time setup
./setup.sh
./setup.sh --no-full-poly-car  # Without high-poly car assets

# Build the project
./build.sh          # Build Release
./build.sh --debug  # Build Debug
./build.sh --gcc    # Use GCC instead of Clang
```

### Testing
```bash
# Run unit tests (after building)
# Windows: Run AirLibUnitTests.exe from build output
# Linux: Run AirLibUnitTests from build output
```

### Clean Build
```bash
# Windows
clean.cmd
clean_rebuild.bat

# Linux
clean.sh
clean_rebuild.sh
```

### Configuration Generation (Recommended)
```bash
# Navigate to unified configuration generator
cd docker_clean/config_generator/tools

# Generate single drone setup with validation
python unified_generator.py single
python config_validator.py --all

# Generate multi-drone setup (3 drones with automatic ports)  
python unified_generator.py multi --num-drones 3
python config_validator.py --all

# Generate mixed vehicle setup (drones + cars)
python unified_generator.py mixed

# Interactive examples and tutorials
cd ../examples
python workflow_example.py

# Migration from legacy tools
cd ../tools
python migrate_to_unified.py
```

### Docker Commands
```bash
# Run AirSim in Docker
./airsim_docker.bat         # Windows
./docker/run_airsim_image_binary.sh  # Linux

# ROS2 integration
./airsim_ros2_docker.bat    # Windows with VNC

# Using generated configurations
cd docker_clean/config_generator/tools
python unified_generator.py multi --num-drones 3
./launch.bat                # Launch generated Docker setup

# Check container status
docker-compose ps

# Access running containers
docker exec -it <container_name> bash

# View container logs
docker logs <container_name>

# Common container operations
docker exec -it airsim_container ls -la  # Check file permissions
docker exec -it px4_container /bin/bash   # Access PX4 container
```

### Containerized Development Workflow
- ALWAYS use `docker exec` commands when accessing services in containers
- Remember that AirSim, PX4, and ROS2 may run in separate containers
- Use container names for service communication in Docker network
- Check container logs first when debugging issues
- Volume mounts enable live code reloading during development

## High-Level Architecture

### Core Components

1. **AirLib** - Core simulation library
   - `AirLib/include/` - Headers for all core functionality
   - `AirLib/src/` - Implementation files
   - Key abstractions:
     - `api/` - RPC API layer for client-server communication
     - `physics/` - Physics engine integration
     - `sensors/` - Sensor simulation (cameras, lidar, IMU, etc.)
     - `vehicles/` - Vehicle models (multirotor, car, computer vision)
     - `common/` - Shared utilities and settings

2. **Unreal Plugin** - Integration with Unreal Engine 5
   - `Unreal/Plugins/AirSim/` - UE5 plugin that drops into any Unreal project
   - Handles rendering, collision detection, and visual simulation
   - Communicates with AirLib through plugin interface

3. **Client Libraries**
   - `PythonClient/cosysairsim/` - Python API client
   - `Matlab/` - MATLAB toolbox for API access
   - Clients communicate via RPC over TCP (default port 41451)

4. **Vehicle Types**
   - Multirotor (PX4, ArduCopter, SimpleFlight)
   - Car (PhysX, SkidSteer vehicles)
   - ComputerVision mode (camera-only simulation)

5. **Sensor System**
   - Modular sensor architecture in `sensors/`
   - Each sensor type has Base class and Simple implementation
   - Custom Cosys-Lab sensors: GPU LiDAR, Echo (radar/sonar), MarLocUwb

### Key Design Patterns

1. **Settings System**
   - Central configuration through `settings.json`
   - Loaded in priority order (command line > executable folder > Documents/AirSim)
   - `AirSimSettings.hpp` defines all available settings

2. **RPC Architecture**
   - Server runs in simulation (AirLib)
   - Clients connect via RPC library (msgpack-rpc)
   - Async and sync API calls supported

3. **Plugin Architecture**
   - Vehicles, sensors, and physics engines are pluggable
   - Factory patterns for creating instances
   - Settings-driven instantiation

4. **Clock System**
   - Multiple clock types (wall clock, simulation time, etc.)
   - Scalable time for faster/slower simulation
   - Lockstep mode for deterministic simulation with PX4

### Important Cosys-Lab Modifications

1. **Annotation System** - Multi-layer ground truth generation with RGB/greyscale/texture options
2. **Instance Segmentation** - Per-object instance masks
3. **GPU LiDAR** - High-density LiDAR using GPU acceleration
4. **Echo Sensors** - Radar/sonar simulation
5. **Skid Steer Vehicles** - New vehicle model for robots like Husky
6. **External World Sensors** - Sensors can be placed independently of vehicles
7. **Dynamic Objects** - Deterministic random object spawning
8. **Enhanced Camera Models** - Chromatic aberration, motion blur, lens distortion

### Development Workflow

1. Modify code in AirLib for core functionality changes
2. Run build script to compile and copy to Unreal plugin
3. Test in Unreal Editor or packaged binaries
4. Use settings.json for runtime configuration
5. Client APIs automatically pick up new functionality through RPC

### Using Gemini CLI for Complex Analysis

Use `mcp__gemini-cli__ask-gemini` for these common tasks:

```bash
# Example: Analyze sensor architecture
"Analyze the sensor implementation architecture in AirLib/include/sensors and AirLib/src/sensors, focusing on the base class design pattern and how different sensor types extend it"

# Example: Review vehicle API structure
"Review all vehicle API implementations in AirLib/include/vehicles/*/api and explain how they share common interfaces"

# Example: Understand RPC communication
"Analyze the RPC server and client implementation across RpcLibServerBase, RpcLibClientBase and vehicle-specific implementations"

# Example: Multi-file feature implementation
"I need to add a new sensor type. Analyze existing sensor implementations (lidar, camera, imu) and provide a template following the same pattern"
```

Always use Gemini CLI when:
- The analysis spans multiple directories
- You need to understand inheritance hierarchies
- You're implementing new features that follow existing patterns
- The total code to analyze exceeds ~10,000 lines

### Key Files to Understand

- `AirLib/include/api/RpcLibServerBase.hpp` - API endpoint definitions
- `AirLib/include/common/AirSimSettings.hpp` - All settings structures
- `AirLib/include/vehicles/*/api/*ApiBase.hpp` - Vehicle-specific APIs
- `PythonClient/cosysairsim/client.py` - Python client implementation
- `Unreal/Plugins/AirSim/Source/SimMode*.cpp` - Unreal simulation modes