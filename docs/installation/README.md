# Installation & Setup Documentation

This section contains comprehensive installation guides for Cosys-AirSim across different platforms and deployment scenarios.

## 🚀 Quick Start Options

### Option 1: Precompiled Binaries (Fastest)
- **[Run Packaged Binaries](run_packaged.md)** - Download and run pre-built AirSim environments
  - Windows and Linux binaries available
  - No compilation required
  - Perfect for testing and evaluation

### Option 2: Precompiled Plugin (Recommended)
- **[Install Precompiled Plugin](install_precompiled.md)** - Use precompiled AirSim plugin
  - Drop into existing Unreal projects
  - Faster than building from source
  - Supports custom environments

### Option 3: Build from Source (Full Control)
- **[Windows Installation](install_windows.md)** - Complete source build on Windows
- **[Linux Installation](install_linux.md)** - Complete source build on Linux

## 🐳 Containerized Development

### Docker Setup
- **[Docker Ubuntu Guide](docker_ubuntu.md)** - Docker-based development environment
  - Isolated development environment
  - Consistent across teams
  - Includes ROS2 integration

## 📋 Installation Comparison

| Method | Time | Flexibility | Use Case |
|--------|------|-------------|----------|
| **Precompiled Binaries** | 5 min | Low | Testing, evaluation |
| **Precompiled Plugin** | 15 min | Medium | Custom environments |
| **Source Build** | 60+ min | High | Development, research |
| **Docker** | 30 min | Medium | Team development |

## 🛠️ Platform-Specific Installation

### Windows Development
- **[Windows Source Build](install_windows.md)**
  - Visual Studio 2022 setup
  - Unreal Engine 5.x installation
  - AirSim compilation process
  - Common troubleshooting

### Linux Development  
- **[Linux Source Build](install_linux.md)**
  - Ubuntu 22.04+ support
  - GCC/Clang configuration
  - Dependency management
  - Build optimization

### Cross-Platform Considerations
- **Development Environment Setup**
- **Dependency Management**
- **Version Compatibility**
- **Performance Optimization**

## ⚙️ Prerequisites & Dependencies

### System Requirements

#### Minimum Requirements
- **OS**: Windows 10/11 or Ubuntu 20.04+
- **RAM**: 8GB (16GB recommended)
- **GPU**: DirectX 11 compatible (RTX series recommended)
- **Storage**: 50GB available space

#### Recommended Requirements
- **CPU**: 8+ cores (Intel i7/AMD Ryzen 7)
- **RAM**: 32GB for large environments
- **GPU**: RTX 3070 or better for optimal performance
- **Storage**: SSD with 100GB+ available space

### Software Dependencies

#### Windows
- Visual Studio 2022 with C++ workload
- Unreal Engine 5.x
- Git with LFS support
- Python 3.8+

#### Linux
- GCC 9+ or Clang 10+
- CMake 3.12+
- Git with LFS support
- Python 3.8+
- OpenGL/Vulkan drivers

## 🔧 Configuration & Setup

### Post-Installation Setup

1. **Verify Installation**
   ```bash
   # Test AirSim functionality
   # Check Unreal Editor integration
   # Validate API connectivity
   ```

2. **Configure Settings**
   - Create settings.json configuration
   - Set up vehicle parameters
   - Configure sensor settings

3. **Test Environment**
   - Launch Blocks environment
   - Test vehicle spawning
   - Verify sensor data flow

### Development Environment

#### IDE Setup
- **Visual Studio** (Windows) - C++ development
- **VS Code** - Cross-platform editing
- **CLion** - Alternative C++ IDE
- **Unreal Editor** - Environment development

#### Version Control
- Git configuration for large files
- LFS setup for assets
- Branching strategies
- Collaboration workflows

## 🚦 Getting Started Workflows

### For Researchers
1. **[Precompiled Binaries](run_packaged.md)** - Quick evaluation
2. **[Custom Environment](install_precompiled.md)** - Specific scenarios
3. **[Source Build](install_linux.md)** - Advanced customization

### For Developers
1. **[Source Build](install_windows.md)** or **[Linux](install_linux.md)** - Full development
2. **[Docker Environment](docker_ubuntu.md)** - Team collaboration
3. **Custom Plugin Development** - Advanced features

### For Production Deployment
1. **[Docker Containerization](docker_ubuntu.md)** - Scalable deployment
2. **[Packaged Distributions](run_packaged.md)** - End-user delivery
3. **Cloud Integration** - Large-scale simulation

## 🔍 Troubleshooting

### Common Installation Issues

#### Windows
- Visual Studio version compatibility
- Unreal Engine download problems
- Build compilation errors
- Plugin loading failures

#### Linux
- Dependency resolution
- Compiler version conflicts
- Graphics driver issues
- Permission problems

#### Docker
- Container build failures
- Volume mounting issues
- Network connectivity
- Performance optimization

### Debug Procedures
1. **Check Prerequisites** - Verify all dependencies
2. **Validate Environment** - Test basic functionality
3. **Review Logs** - Examine build and runtime logs
4. **Community Support** - Check GitHub issues and discussions

## 📚 Additional Resources

### Documentation Links
- **[Core Settings](../core/settings.md)** - Configuration reference
- **[Unreal Development](../unreal/)** - Environment creation
- **[PX4 Integration](../px4/)** - Flight controller setup
- **[ROS2 Integration](../ros2/)** - Robotics framework

### Community Resources
- **GitHub Repository** - Source code and issues
- **Discord Community** - Real-time support
- **Research Papers** - Academic references
- **YouTube Tutorials** - Visual guides

## 🎯 Next Steps

After successful installation:

1. **[Configure Your First Vehicle](../core/settings.md)**
2. **[Test API Connectivity](../core/apis.md)**
3. **[Explore Sample Environments](../unreal/unreal_blocks.md)**
4. **[Set Up Development Tools](../ros2/)** or **[PX4 Integration](../px4/)**

## 📋 Quick Installation Checklist

### Pre-Installation
- [ ] System meets minimum requirements
- [ ] Required software downloaded
- [ ] Development tools installed
- [ ] Network access available

### During Installation
- [ ] Follow platform-specific guide
- [ ] Verify each build step
- [ ] Note any warnings or errors
- [ ] Test intermediate functionality

### Post-Installation
- [ ] Verify AirSim launches correctly
- [ ] Test vehicle spawning
- [ ] Confirm API connectivity
- [ ] Set up development environment

Choose the installation method that best fits your needs and experience level. The precompiled options are great for getting started quickly, while source builds provide maximum flexibility for research and development.