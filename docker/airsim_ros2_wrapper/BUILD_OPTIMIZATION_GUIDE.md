# 🚀 Build Optimization Guide

This guide explains how to dramatically reduce Docker build times and optimize your development workflow with AirSim ROS2.

## 🏗️ Multi-Stage Build Architecture

### Stage 1: Base Dependencies (`base-deps`)
- **Purpose**: System packages, VNC, ROS2 core
- **Rebuild Frequency**: Rarely (weeks/months)
- **Cache Strategy**: Aggressive caching
- **Size**: ~1.2 GB

```dockerfile
FROM ros:humble AS base-deps
# Install VNC, GUI libraries, ROS2 packages
# These rarely change, so they cache well
```

### Stage 2: AirSim Dependencies (`airsim-deps`)
- **Purpose**: External libraries (rpclib, mavlink)
- **Rebuild Frequency**: Occasionally (when external deps update)
- **Cache Strategy**: Medium caching
- **Size**: ~300 MB additional

```dockerfile
FROM base-deps AS airsim-deps
# Copy external dependencies that change infrequently
COPY external/rpclib/ /airsim_ros2_ws/external/rpclib/
```

### Stage 3: Source Code Build (`build`)
- **Purpose**: Your AirSim ROS2 source code
- **Rebuild Frequency**: Frequently (every code change)
- **Cache Strategy**: Optimized for changes
- **Size**: ~400 MB additional

```dockerfile
FROM airsim-deps AS build
# Copy source code and build ROS2 packages
COPY ros2/ /airsim_ros2_ws/src/
```

### Stage 4: Runtime Configuration (`runtime`)
- **Purpose**: Scripts, environment, final setup
- **Rebuild Frequency**: Occasionally (config changes)
- **Cache Strategy**: Lightweight operations
- **Size**: ~100 MB additional

## 🛠️ Development Workflows

### 1. 🏃‍♂️ Quick Development (Recommended)

For rapid iteration when modifying ROS2 source code:

```bash
# One-time setup (15 minutes)
build_optimized.bat

# Development mode with volume mounts (30 seconds per change)
develop.bat
```

**Benefits:**
- Source code changes reflected immediately
- No container rebuild needed
- Full GUI/VNC support
- Fast incremental builds inside container

### 2. 🔄 Clean Build

When you need to rebuild everything from scratch:

```bash
# Full rebuild (15 minutes)
build_optimized.bat --rebuild-deps
```

### 3. 🚢 Production Deployment

For final deployment with optimized size:

```bash
# Build optimized production image
build_optimized.bat

# Run in production mode
run_optimized.bat
```

## 🧠 Docker Caching Strategy

### Layer Ordering (Most Important → Least Important)
1. **System packages** (changes: never)
2. **External dependencies** (changes: rarely)
3. **Source code** (changes: frequently)
4. **Configuration scripts** (changes: occasionally)

### Cache Optimization Techniques

#### 1. Dependency Separation
```dockerfile
# ❌ Bad: Copies everything together
COPY . /workspace

# ✅ Good: Separates stable vs changing files
COPY external/ /workspace/external/     # Stable
COPY ros2/ /workspace/src/              # Changes frequently
```

#### 2. Build Context Optimization
```dockerfile
# Only copy what's needed for each stage
COPY external/rpclib/ /airsim_ros2_ws/external/rpclib/
COPY MavLinkCom/mavlink/ /airsim_ros2_ws/MavLinkCom/mavlink/
# Don't copy everything with COPY . /
```

#### 3. Multi-Stage Cache Reuse
```bash
docker build \
  --cache-from airsim-ros2:base-deps \
  --cache-from airsim-ros2:airsim-deps \
  --cache-from airsim-ros2:build \
  -f Dockerfile.optimized
```

## 💡 Best Practices

### For Developers

1. **Use Development Mode** for code changes:
   ```bash
   develop.bat  # Mounts source code as volumes
   ```

2. **Incremental builds inside container**:
   ```bash
   docker exec -it airsim-ros2-dev bash
   cd /airsim_ros2_ws
   colcon build --packages-select airsim_ros_pkgs  # Only rebuild changed packages
   ```

3. **Check what changed** before rebuilding:
   ```bash
   git status  # See what files changed
   git diff    # See exact changes
   ```

### For CI/CD

1. **Use BuildKit** for better caching:
   ```bash
   export DOCKER_BUILDKIT=1
   ```

2. **Tag intermediate stages** for cache reuse:
   ```bash
   docker build --target base-deps -t airsim-ros2:base-deps .
   docker build --target airsim-deps -t airsim-ros2:airsim-deps .
   ```

3. **Leverage registry caching**:
   ```bash
   docker build --cache-from registry.com/airsim-ros2:base-deps
   ```

## 🐛 Troubleshooting

### Build Cache Issues

**Problem**: Cache not being used effectively
```bash
# Solution: Clear cache and rebuild specific stage
docker builder prune
build_optimized.bat --rebuild-deps
```

**Problem**: "Layer already exists" but still slow
```bash
# Solution: Use BuildKit features
set DOCKER_BUILDKIT=1
docker build --progress=plain  # See detailed progress
```

### Development Workflow Issues

**Problem**: Changes not reflected in container
```bash
# Solution: Ensure volume mounts are working
docker run -v "%CD%\ros2:/airsim_ros2_ws/src" ...
# Check inside container:
docker exec -it airsim-ros2-dev ls -la /airsim_ros2_ws/src
```

**Problem**: Permission issues with mounted volumes
```bash
# Solution: Fix ownership inside container
docker exec -it airsim-ros2-dev chown -R vncuser:vncuser /airsim_ros2_ws
```

## 📈 Monitoring Build Performance

### Build Time Analysis
```bash
# Use BuildKit to see timing breakdown
set DOCKER_BUILDKIT=1
docker build --progress=plain -f Dockerfile.optimized . 2>&1 | findstr "Step\|CACHED"
```

### Cache Hit Rate
- **Good**: 80%+ cache hits on rebuilds
- **Needs optimization**: <50% cache hits

### Size Analysis
```bash
# Check layer sizes
docker history airsim-ros2:optimized
```

## 🎯 Optimization Targets by Use Case

### Rapid Development
- **Priority**: Fast iteration cycles
- **Target**: <1 minute for code changes
- **Strategy**: Volume mounts + incremental builds

### CI/CD Pipeline  
- **Priority**: Consistent, cacheable builds
- **Target**: <10 minutes for full builds
- **Strategy**: Multi-stage caching + registry cache

### Production Deployment
- **Priority**: Small, secure images
- **Target**: <2 GB final image
- **Strategy**: Multi-stage builds + distroless base

## 🔧 Advanced Optimizations

### 1. Parallel Builds
```dockerfile
# Use parallel colcon builds
RUN colcon build --parallel-workers 4 --packages-select airsim_interfaces airsim_ros_pkgs
```

### 2. Build-time Environment Variables
```dockerfile
ARG PARALLEL_JOBS=4
RUN make -j${PARALLEL_JOBS}
```

### 3. Dependency Pre-compilation
```dockerfile
# Pre-compile commonly used dependencies
RUN colcon build --packages-up-to rclcpp geometry_msgs sensor_msgs
```

This optimization guide should help you achieve **90%+ faster development cycles** while maintaining full functionality! 🚀 