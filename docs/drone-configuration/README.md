# Drone Configuration Guide

This directory contains comprehensive documentation for configuring multirotor drones in Cosys-AirSim. The configuration system allows you to customize drone parameters including weight, rotor specifications, frame geometry, and flight characteristics.

## Documentation Structure

- **[Overview](overview.md)** - High-level architecture and configuration concepts
- **[Frame Types](frame-types.md)** - Pre-built frame configurations and custom frame creation
- **[Rotor Configuration](rotor-configuration.md)** - Detailed rotor parameter tuning
- **[Physical Properties](physical-properties.md)** - Mass, inertia, and aerodynamic properties
- **[Settings Integration](settings-integration.md)** - How to use settings.json for drone configuration
- **[Custom Drone Creation](custom-drone-creation.md)** - Step-by-step guide for creating custom drone types
- **[Performance Tuning](performance-tuning.md)** - Optimization guidelines for different use cases
- **[Troubleshooting](troubleshooting.md)** - Common issues and solutions

## Quick Start

1. **Choose a base frame type** from the available configurations (Quad, Hex, Octo)
2. **Configure physical properties** (mass, dimensions, rotor specs)
3. **Set up in settings.json** with your desired vehicle type and model
4. **Test and tune** performance parameters as needed

## Supported Vehicle Types

- **SimpleFlight** - Built-in flight controller for basic simulation
- **PX4** - Integration with PX4 autopilot stack
- **ArduCopter** - Integration with ArduPilot/ArduCopter

## Common Use Cases

- **Research drones** - Lightweight configurations for algorithm testing
- **Commercial drones** - Realistic parameters for real-world simulation
- **Heavy-lift drones** - High-payload configurations
- **Racing drones** - High-performance, low-latency setups
- **Swarm drones** - Optimized for multi-vehicle scenarios

## Getting Help

For questions or issues:
1. Check the [troubleshooting guide](troubleshooting.md)
2. Review the [custom drone creation guide](custom-drone-creation.md)
3. Examine the source code in `AirLib/include/vehicles/multirotor/`
4. Open an issue on the Cosys-AirSim repository

## Contributing

When adding new drone configurations or improving documentation:
1. Follow the existing code patterns in `MultiRotorParams.hpp`
2. Add comprehensive documentation for new parameters
3. Include example configurations and use cases
4. Test with different vehicle types and scenarios