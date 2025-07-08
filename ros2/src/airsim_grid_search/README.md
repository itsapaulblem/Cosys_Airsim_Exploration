# 🗺️ AirSim Grid Search Mission System

**Autonomous grid search mission planning and execution for AirSim drones using ROS2 action servers.**

This package provides a complete grid search mission system with waypoint generation, safety monitoring, and real-time mission control for AirSim simulation environments.

## 🚀 Features

### Core Capabilities
- **🎯 Grid Pattern Generation**: Boustrophedon (lawnmower) pattern with configurable spacing
- **🛡️ Safety Monitoring**: Battery, geofence, communication, and flight time monitoring  
- **📡 Real-time Control**: Pause, resume, abort, and modify missions dynamically
- **📊 Progress Tracking**: Detailed mission progress and statistics
- **🗺️ Coordinate Systems**: Support for both GPS and local NED coordinates
- **⚡ Async Execution**: Non-blocking mission execution with real-time feedback

### Safety Features
- Battery level monitoring with automatic return-home
- Geofence enforcement with violation detection
- Maximum flight time and distance limits
- Communication loss detection and handling
- Emergency stop and return-home procedures

### Advanced Features
- Path optimization for efficient coverage
- Configurable waypoint spacing and overlap
- Data collection coordination (photos, sensors)
- Multi-mission support (future)
- RViz2 visualization integration

## 📦 Package Structure

```
airsim_grid_search/
├── airsim_grid_search/           # Python package
│   ├── grid_search_server.py     # Main action server
│   ├── grid_generator.py         # Grid pattern generation
│   ├── waypoint_navigator.py     # Waypoint navigation
│   ├── safety_monitor.py         # Safety monitoring
│   └── grid_search_cli.py        # Command-line interface
├── launch/                       # Launch files
│   └── grid_search.launch.py     # Main launch file
├── config/                       # Configuration files
│   └── grid_search_params.yaml   # Default parameters
└── README.md                     # This file
```

## 🛠️ Installation

### Prerequisites
- ROS2 Humble
- AirSim with ROS2 wrapper
- Python 3.8+
- Required ROS2 packages: `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /path/to/your_ros2_ws

# Build the interfaces package first
colcon build --packages-select airsim_mission_interfaces

# Source the workspace
source install/setup.bash

# Build the grid search package
colcon build --packages-select airsim_grid_search

# Source again
source install/setup.bash
```

## 🚁 Quick Start

### 1. Launch AirSim and ROS2 Wrapper

```bash
# Start AirSim (in separate terminal)
# Launch your Unreal Engine AirSim environment

# Start AirSim ROS2 wrapper
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### 2. Launch Grid Search System

```bash
# Launch the complete grid search system
ros2 launch airsim_grid_search grid_search.launch.py vehicle_name:=drone_1

# Or launch just the action server
ros2 run airsim_grid_search grid_search_server
```

### 3. Generate a Test Grid

```bash
# Generate a simple test grid (50x50m area)
ros2 run airsim_grid_search grid_search_cli generate \
  --north 50 --south 0 --east 50 --west 0 \
  --altitude 20 --lane-spacing 15 --waypoint-spacing 10 \
  --show-waypoints
```

### 4. Execute a Mission

```bash
# Execute a grid search mission
ros2 run airsim_grid_search grid_search_cli execute \
  --vehicle drone_1 \
  --north 100 --south 0 --east 100 --west 0 \
  --altitude 25 --lane-spacing 20 --waypoint-spacing 10 \
  --speed 5 --auto-start --return-home
```

## 📋 Usage Examples

### Basic Grid Generation

```bash
# Local coordinates (meters)
ros2 run airsim_grid_search grid_search_cli generate \
  --north 100 --south 0 --east 100 --west 0 \
  --altitude 20 --lane-spacing 20 --waypoint-spacing 10

# GPS coordinates  
ros2 run airsim_grid_search grid_search_cli generate \
  --gps-coords \
  --north 47.642468 --south 47.641468 \
  --east -122.139165 --west -122.140165 \
  --altitude 30 --lane-spacing 25
```

### Mission Execution

```bash
# Simple mission with safety parameters
ros2 run airsim_grid_search grid_search_cli execute \
  --vehicle drone_1 \
  --north 50 --south 0 --east 50 --west 0 \
  --altitude 15 --speed 4 \
  --min-battery 25 --max-flight-time 600 \
  --auto-start
  
# Advanced mission with path optimization
ros2 run airsim_grid_search grid_search_cli execute \
  --vehicle drone_1 \
  --north 200 --south 0 --east 200 --west 0 \
  --altitude 30 --speed 6 \
  --lane-spacing 25 --waypoint-spacing 12 \
  --optimize --reverse-direction \
  --return-home --enable-geofence
```

### Mission Monitoring

```bash
# Check current mission status
ros2 run airsim_grid_search grid_search_cli status

# Monitor mission progress in real-time
ros2 topic echo /grid_search_status
```

## 🔧 Configuration

### Parameter Configuration

Edit `config/grid_search_params.yaml` to customize default behavior:

```yaml
grid_search_server:
  ros__parameters:
    vehicle_name: "drone_1"
    default_flight_speed: 5.0
    position_tolerance: 2.0
    safety_check_frequency: 2.0
    min_battery_percentage: 20.0
```

### Runtime Parameters

Many parameters can be overridden at launch time:

```bash
ros2 launch airsim_grid_search grid_search.launch.py \
  vehicle_name:=drone_2 \
  safety_check_frequency:=1.0 \
  position_tolerance:=1.5
```

## 🔌 ROS2 Interface

### Action Server

**Action**: `grid_search` (GridSearch)
- **Goal**: Search area, grid parameters, safety parameters
- **Result**: Mission completion status and statistics
- **Feedback**: Real-time progress and current waypoint

### Services

- **`generate_grid`** (GenerateGrid): Generate waypoint grid
- **`update_mission`** (UpdateMission): Pause/resume/modify active mission
- **`get_mission_status`** (GetMissionStatus): Get current mission status

### Topics

- **`/grid_search_status`** (GridSearchStatus): Real-time mission status
- **`/airsim_node/{vehicle}/odom_local_ned`**: Vehicle odometry (subscribed)
- **`/airsim_node/{vehicle}/global_gps`**: GPS position (subscribed)
- **`/airsim_node/{vehicle}/vel_cmd_body_frame`**: Velocity commands (published)

## 🛡️ Safety System

### Monitoring Parameters

```yaml
safety_params:
  max_flight_time: 1800.0          # seconds
  min_battery_percentage: 0.20     # 20%
  return_home_battery: 0.25        # 25%
  max_distance_from_home: 1000.0   # meters
  enable_geofence: true
  enable_return_home: true
```

### Safety Violations

The system monitors for:
- **Battery Low/Critical**: Automatic mission termination and return home
- **Geofence Violation**: Immediate stop and return to safe area
- **Communication Loss**: Configurable timeout with offline mode support
- **Maximum Distance**: Distance-based safety limits
- **Flight Time Limit**: Maximum mission duration enforcement

### Emergency Procedures

1. **Warning Level**: Log warning, continue mission
2. **Critical Level**: Pause mission, wait for resolution or timeout
3. **Emergency Level**: Immediate stop, initiate return home, prepare for emergency landing

## 🎮 Command Line Interface

### Available Commands

```bash
# Generate waypoint grid
grid_search_cli generate [options]

# Execute grid search mission  
grid_search_cli execute [options]

# Get mission status
grid_search_cli status [options]
```

### Common Options

```bash
--vehicle VEHICLE         # Vehicle name (default: drone_1)
--north METERS           # North boundary
--south METERS           # South boundary  
--east METERS            # East boundary
--west METERS            # West boundary
--altitude METERS        # Flight altitude
--lane-spacing METERS    # Distance between flight lines
--waypoint-spacing METERS # Distance between waypoints
--speed M_PER_SEC        # Flight speed
--gps-coords             # Use GPS coordinates
--auto-start             # Start mission immediately
--return-home            # Return home when complete
--optimize               # Enable path optimization
```

## 📊 Visualization

### RViz2 Integration

Launch with visualization:

```bash
ros2 launch airsim_grid_search grid_search.launch.py \
  launch_rviz:=true
```

Displays:
- **Planned waypoints**: Mission path visualization
- **Current position**: Real-time vehicle location
- **Progress indicators**: Completed vs remaining waypoints
- **Safety zones**: Geofence and distance limits

## 🧪 Testing

### Unit Tests

```bash
# Run unit tests
colcon test --packages-select airsim_grid_search

# View test results
colcon test-result --verbose
```

### Integration Testing

```bash
# Test grid generation
ros2 run airsim_grid_search grid_generator

# Test waypoint navigation
ros2 run airsim_grid_search waypoint_navigator

# Test safety monitoring
ros2 run airsim_grid_search safety_monitor
```

## 🔄 Development Workflow

### Adding New Features

1. **Grid Patterns**: Extend `grid_generator.py` with new pattern types
2. **Safety Checks**: Add monitoring logic to `safety_monitor.py`
3. **Navigation**: Enhance waypoint execution in `waypoint_navigator.py`
4. **UI/CLI**: Extend command-line interface in `grid_search_cli.py`

### Debugging

```bash
# Enable debug logging
ros2 param set /grid_search_server log_level DEBUG

# Monitor internal state
ros2 topic echo /grid_search_status

# Check safety violations
ros2 service call /get_mission_status airsim_mission_interfaces/srv/GetMissionStatus
```

## 🚨 Troubleshooting

### Common Issues

**Action server not available**
```bash
# Check if server is running
ros2 node list | grep grid_search

# Check action availability
ros2 action list | grep grid_search
```

**Vehicle not responding**
```bash
# Check AirSim connection
ros2 topic list | grep airsim_node

# Test vehicle control
ros2 service call /airsim_node/drone_1/takeoff airsim_interfaces/srv/Takeoff
```

**Mission fails immediately**
```bash
# Check safety parameters
ros2 param get /grid_search_server min_battery_percentage

# Review pre-flight checks
ros2 log grep grid_search | grep "pre-flight"
```

### Performance Tuning

```yaml
# Adjust navigation frequency
navigation_frequency: 20.0  # Hz (higher = more responsive)

# Modify safety check rate  
safety_check_frequency: 1.0  # Hz (lower = less CPU usage)

# Tune position tolerance
position_tolerance: 1.0  # meters (smaller = more precise)
```

## 📚 API Reference

### GridSearch Action

```python
# Goal
SearchArea search_area
GridParams grid_params  
SafetyParams safety_params
string mission_id
string vehicle_name
bool auto_start
bool return_home_on_completion

# Result
bool success
string result_message
SearchStats mission_stats
float32 final_battery_percentage
Duration total_execution_time

# Feedback  
MissionProgress progress
CurrentWaypoint current_waypoint
float32 completion_percentage
string status_message
```

### Python API Usage

```python
import rclpy
from rclpy.action import ActionClient
from airsim_mission_interfaces.action import GridSearch

# Create action client
client = ActionClient(node, GridSearch, 'grid_search')

# Create and send goal
goal = GridSearch.Goal()
# ... configure goal ...

future = client.send_goal_async(goal, feedback_callback=feedback_cb)
```

## 🤝 Contributing

### Development Setup

```bash
# Clone repository
git clone https://github.com/Cosys-Lab/Cosys-AirSim.git
cd Cosys-AirSim/ros2/src

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build in development mode
colcon build --symlink-install --packages-select airsim_grid_search
```

### Code Style

- Follow PEP 8 for Python code
- Use type hints where possible
- Add docstrings for all public functions
- Include unit tests for new features

### Pull Request Process

1. Create feature branch from main
2. Implement changes with tests
3. Update documentation
4. Submit pull request with description

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 🎯 Next Steps

Ready to use the grid search system! Start with a simple test mission:

```bash
# 1. Launch AirSim and ROS2 wrapper
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# 2. Launch grid search system  
ros2 launch airsim_grid_search grid_search.launch.py

# 3. Execute test mission
ros2 run airsim_grid_search grid_search_cli execute \
  --vehicle drone_1 --north 50 --south 0 --east 50 --west 0 \
  --altitude 20 --auto-start --return-home
```

**Happy autonomous flying!** 🚁✨