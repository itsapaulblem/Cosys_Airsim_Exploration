# ROS2 Multi-Node System for AirSim

**Multi-Vehicle ROS2 Architecture with Cross-Platform Support**

This Docker environment provides a modern, ultra-clean multi-node ROS2 system for AirSim simulation. Features automatic vehicle discovery, coordination node authority, and standardized transform frames - perfect for multi-vehicle scenarios and cross-platform development (Windows AirSim + Docker ROS2).

## Architecture Features

- **Perfect Node Naming**: Vehicle names ARE node names (`/Droan1`, `/PX4_Drone2`)
- **Coordination Authority**: Single `coordination_node` for global `world_ned` frame
- **RPC Auto-Discovery**: Automatic detection of active vehicles in AirSim
- **Cross-Platform**: Windows AirSim + Docker ROS2 seamlessly supported
- **Transform Fix**: No more stacked vehicles in RViz2!
- **Fault Isolation**: Individual vehicle failures don't affect others

## Quick Start

### Prerequisites

- Docker Desktop with docker-compose
- AirSim running with multiple vehicles configured
- Optional: VNC viewer for graphical access

### Three Launch Modes

**Primary: Multi-Node (Recommended)**
```bash
# Navigate to directory
cd /path/to/Cosys-AirSim/docker/airsim_ros2_wrapper

# Launch ultra-clean multi-node system (default mode)
docker-compose up -d

# Or explicitly specify multi-node mode
LAUNCH_MODE=multi docker-compose up -d
```

**Legacy: Monolithic Single-Node (Backward Compatibility)**
```bash
# Launch legacy single-node system
LAUNCH_MODE=legacy docker-compose up -d
```

**Custom: Manual Control**
```bash
# Launch container without auto-start for manual control
LAUNCH_MODE=custom docker-compose up -d

# Enter container for manual commands
docker exec -it ros2-multi-node bash
```

## Multi-Node Results

### Expected Node Structure
```bash
# Check discovered nodes
ros2 node list
# Output:
# /airsim_coordination_node  <- Global frame authority
# /Droan1                    <- Vehicle node (name from AirSim settings)
# /PX4_Drone2                <- Vehicle node (name from AirSim settings)
```

### Perfect Topic Structure
```bash
# Check ultra-clean topics
ros2 topic list | grep -E "/(Droan|PX4_)"
# Output:
# /Droan1/odom_local_ned     <- Individual vehicle odometry
# /Droan1/global_gps         <- Individual vehicle GPS
# /Droan1/imu                <- Individual vehicle IMU
# /PX4_Drone2/odom_local_ned <- Second vehicle odometry
# /PX4_Drone2/imu            <- Second vehicle IMU
```

### Transform Tree (Fixed!)
```bash
# Verify clean tf structure
ros2 run tf2_tools view_frames
# Expected hierarchy:
# map → world_ned → Droan1_base_link
#                 → PX4_Drone2_base_link
```

## VNC Access (Optional)

**Connect for Graphical Development:**
1. **VNC URL**: `localhost:5901`
2. **Password**: `ubuntu`
3. **Desktop**: XFCE with development tools

**Container Aliases Available:**
```bash
launch_multi    # Start ultra-clean multi-node
launch_legacy   # Start legacy monolithic
launch_rviz     # Open RViz2 visualization
build           # Build workspace
source_ws       # Source workspace
```

## Multi-Vehicle Control Examples

### Individual Vehicle Control
```bash
# Takeoff specific vehicle
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"

# Control specific vehicle
ros2 topic pub /Droan1/vel_cmd_body_frame geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}}"
```

### Global Coordination Control
```bash
# Takeoff all vehicles simultaneously
ros2 service call /airsim_coordination_node/takeoff_all airsim_interfaces/srv/Takeoff \
  "{wait_on_last_task: true}"

# Check system health
ros2 service call /airsim_coordination_node/health_check \
  airsim_interfaces/srv/ListSceneObjectTags
```

## Development Workflow

Open terminal in VNC desktop:

```bash
# 1. Build workspace (first time or after changes)
build                    # Full workspace build
build_interfaces         # Build interfaces only
build_pkgs              # Build packages only
source_ws               # Source workspace

# 2. Launch multi-node system (primary approach)
launch_multi             # Ultra-clean multi-node RPC discovery

# 3. Launch RViz2 for visualization
launch_rviz             # Open RViz2 with proper frame setup

# 4. Monitor and control vehicles
ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
```

### RViz2 Configuration for Multi-Node

**Important Frame Setup:**
1. **Fixed Frame**: Set to `world_ned` (not `map` or `odom`)
2. **Add TF Display**: See individual vehicle frames
3. **Vehicle Topics**: Add topics like `/Droan1/odom_local_ned`

```bash
# Launch RViz2 with multi-node awareness
rviz2

# Or use container alias
launch_rviz
```

### Debugging Multi-Node System

```bash
# Check RPC discovery
ros2 topic list | grep rpc_dynamic

# Verify coordination node
ros2 node info /airsim_coordination_node

# Check transform tree
ros2 run tf2_tools view_frames

# Monitor individual vehicles
ros2 topic echo /Droan1/odom_local_ned --once
ros2 topic echo /PX4_Drone2/imu --once
```

## Cross-Platform AirSim Integration

### Multi-Node Connection Architecture

The container seamlessly connects to AirSim via RPC auto-discovery:

- **Cross-Platform**: Windows AirSim + Docker ROS2 automatically supported
- **Auto-Discovery**: Vehicles discovered via direct RPC calls
- **Global Coordination**: Single coordination_node manages `world_ned` frame
- **Individual Nodes**: Each vehicle gets its own isolated ROS2 node

### Connection Configuration

**Environment Variables (All Configurable):**
```bash
# Multi-node system configuration
LAUNCH_MODE=multi                    # Primary: ultra-clean multi-node
ENABLE_COORDINATION=true             # Global coordination authority
RPC_TIMEOUT=10.0                     # RPC discovery timeout

# AirSim connection
AIRSIM_HOST_IP=host.docker.internal  # Auto-resolves to host
AIRSIM_HOST_PORT=41451               # Default AirSim API port

# Visualization
LAUNCH_RVIZ=false                    # Optional auto-launch RViz2
DEBUG=false                          # Enable diagnostic logging
```

## Topic Structure

**Individual Vehicle Topics (per discovered vehicle):**
```bash
# Perfect naming: vehicle names ARE node names
/Droan1/odom_local_ned              # Individual vehicle odometry
/Droan1/global_gps                  # Individual vehicle GPS
/Droan1/imu                         # Individual vehicle IMU
/Droan1/vel_cmd_body_frame          # Individual vehicle commands

# Second vehicle (automatically discovered)
/PX4_Drone2/odom_local_ned          # Second vehicle data
/PX4_Drone2/imu                     # Second vehicle sensors
/PX4_Drone2/takeoff                 # Second vehicle services
```

**Global Coordination Topics:**
```bash
/airsim_coordination_node/origin_geo_point     # Global GPS origin
/airsim_coordination_node/system_status        # All vehicles status
/airsim_coordination_node/takeoff_all          # Global commands
/airsim_coordination_node/health_check         # System health
```

## Configuration Options

### Launch Mode Selection

**Primary: Multi-Node**
```bash
LAUNCH_MODE=multi LAUNCH_RVIZ=true docker-compose up -d
```

**Backward Compatibility: Legacy Monolithic**
```bash
LAUNCH_MODE=legacy docker-compose up -d
```

**Development: Custom Manual Control**
```bash
LAUNCH_MODE=custom docker-compose up -d
docker exec -it ros2-multi-node bash
```

### Data Persistence

Docker volumes ensure development continuity:
- **ros2_multi_node_build**: Build artifacts for faster rebuilds
- **ros2_multi_node_install**: Installed packages persist
- **ros2_multi_node_logs**: Build and runtime logs
- **vnc_home**: User settings and desktop configuration

## Multi-Node Troubleshooting

### Transform/TF Issues (Fixed!)

**Problem**: Vehicles stacked at origin in RViz2
**Solution**: **Already fixed with coordination_node authority!**

```bash
# Verify fix is working
ros2 run tf2_tools view_frames
# Should show: map → world_ned → individual vehicle frames

# Check vehicle positions
ros2 topic echo /Droan1/odom_local_ned --once
ros2 topic echo /PX4_Drone2/odom_local_ned --once
```

### RPC Discovery Issues

```bash
# Test RPC connectivity
docker exec -it ros2-multi-node /debug_airsim_connection.sh

# Check vehicle discovery
ros2 node list | grep -E "(Droan|PX4_)"

# Verify coordination node
ros2 node info /airsim_coordination_node
```

### Cross-Platform Connection Issues

**Windows AirSim + Docker ROS2:**
```bash
# Ensure AirSim binding (Windows host)
netstat -an | findstr 41451

# Test from container
docker exec -it ros2-multi-node bash
python3 -c "import socket; s=socket.socket(); s.connect(('host.docker.internal', 41451)); print('Connected')"
```

## Container Management

### Quick Commands

```bash
# Start multi-node system
docker-compose up -d

# View multi-node logs
docker-compose logs -f

# Enter container for manual control
docker exec -it ros2-multi-node bash

# Stop system
docker-compose down

# Full cleanup (remove volumes)
docker-compose down -v
```

### Development Workflow

```bash
# 1. Edit source files on host (auto-mounted)
# 2. Build in container
docker exec -it ros2-multi-node bash
build && source_ws

# 3. Launch multi-node system
launch_multi

# 4. Test with RViz2
launch_rviz
```

## Quick Reference

| Component | Access | Configuration |
|-----------|--------|---------------|
| **Multi-Node System** | `LAUNCH_MODE=multi` | **Primary approach** |
| **Legacy System** | `LAUNCH_MODE=legacy` | Backward compatibility |
| **VNC Desktop** | `localhost:5901` | Password: `ubuntu` |
| **Container** | `ros2-multi-node` | Auto-configured networking |

### Environment Variables Summary

```bash
LAUNCH_MODE=multi                    # Ultra-clean multi-node (default)
ENABLE_COORDINATION=true             # Global frame authority
RPC_TIMEOUT=10.0                     # Discovery timeout
LAUNCH_RVIZ=false                    # Auto-launch visualization
AIRSIM_HOST_IP=host.docker.internal  # Cross-platform host resolution
```

---

## Multi-Node Benefits

**No More Stacked Vehicles**: Fixed tf coordination  
**Perfect Naming**: Vehicle names ARE node names  
**Fault Isolation**: One vehicle failure doesn't affect others  
**Cross-Platform**: Windows AirSim + Docker ROS2 seamlessly  
**Auto-Discovery**: Zero manual configuration needed  
**Global Coordination**: Single authority for world frames  

**Perfect for**: Multi-vehicle simulation, cross-platform development, fault-tolerant robotics systems!

---

*Ultra-clean multi-node ROS2 architecture that just works. Transform issues solved, discovery automated, coordination centralized.*
