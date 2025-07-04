# Multi-Drone Orbit Mission

## Overview

The Multi-Drone Orbit Mission is a sophisticated autonomous flight system that coordinates multiple drones to perform synchronized orbital movements, where each drone visits the positions of all other drones in the simulation. This mission demonstrates advanced multi-agent coordination, path planning, and synchronized flight operations.

## How It Works

### Mission Architecture

The system is built on the **Generic Mission Framework** (`generic_mission.py`) which provides:
- 🛡️ **Safety Systems**: Collision detection, emergency landing, and error handling
- 📊 **Mission Analytics**: Performance tracking, efficiency metrics, and comprehensive reporting
- 🔄 **Template Method Pattern**: Standardized mission workflow with customizable mission-specific logic
- 📸 **Photo Management**: Automated photo capture and organization

### Core Mission Flow

```
1. 🔍 DISCOVERY PHASE
   ├── Auto-detect available drones in simulation
   ├── Validate minimum drone count (≥2 required)
   └── Initialize individual drone clients

2. 🚀 INITIALIZATION PHASE
   ├── Record initial positions of all drones
   ├── Setup multi-drone connections
   ├── Enable API control and arm all drones
   └── Calculate total orbit positions to visit

3. ✈️ TAKEOFF PHASE
   ├── Synchronized takeoff for all drones
   ├── Climb to target altitude simultaneously
   └── Stabilization and position verification

4. 🔄 ORBITAL EXECUTION PHASE
   ├── Create orbit sequences for each drone
   ├── Execute synchronized orbital movements
   ├── Stop at each position for specified duration
   ├── Capture photos at each orbit position
   └── Monitor for collisions and safety

5. 🏠 RETURN PHASE
   ├── Return all drones to original positions
   ├── Synchronized landing operations
   └── Generate comprehensive mission report
```

### Detailed Function Breakdown

#### 1. Drone Discovery (`_discover_available_drones()`)

**Purpose**: Automatically find all available drones in the simulation without requiring manual configuration.

**How it works**:
- Tests multiple common drone naming patterns:
  - `Drone1`, `Drone2`, `Drone3`...
  - `PX4_Drone1`, `PX4_Drone2`, `PX4_Drone3`...
  - `UAV1`, `UAV2`, `UAV3`...
  - `Multirotor1`, `Multirotor2`, `Multirotor3`...
- For each pattern, attempts to connect and get drone state
- Returns the first successful pattern match
- Limits to requested number of drones if specified

**Example Output**:
```
🔍 Discovering available drones in simulation...
  ✅ Found: PX4_Drone1
  ✅ Found: PX4_Drone2  
  ✅ Found: PX4_Drone3
  ✅ Found: PX4_Drone4
  Using drone pattern: PX4_DroneX
  🎯 Total drones available: 4
```

#### 2. Multi-Drone Connection Setup (`_setup_multi_drone_connections()`)

**Purpose**: Establish individual API connections for each drone to enable independent control.

**How it works**:
- Creates separate `MultirotorClient` instance for each drone
- Enables API control and arms each drone individually  
- Stores client references in `orbit_clients` dictionary
- Handles connection failures gracefully with error logging

**Key Features**:
- Independent control: Each drone can be commanded separately
- Error isolation: If one drone fails, others continue operating
- Parallel operations: Multiple drones can execute commands simultaneously

#### 3. Position Recording (`_record_initial_positions()`)

**Purpose**: Capture and store the starting positions of all drones for orbit calculations.

**How it works**:
- Queries each drone's current position using `getMultirotorState()`
- Stores positions in `initial_positions` dictionary with x, y, z coordinates
- These positions become the target locations for the orbital pattern
- Calculates total orbit positions: `len(drones) × len(drones)`

**Example**:
```python
# For 4 drones, each visits 3 other positions = 12 total orbit positions
initial_positions = {
    'PX4_Drone1': {'x': 5.0, 'y': 0.0, 'z': 0.0},
    'PX4_Drone2': {'x': -5.0, 'y': 0.0, 'z': 0.0},
    'PX4_Drone3': {'x': 5.0, 'y': 5.0, 'z': 0.0},
    'PX4_Drone4': {'x': -5.0, 'y': 5.0, 'z': 0.0}
}
```

#### 4. Synchronized Takeoff (`_execute_synchronized_takeoff()`)

**Purpose**: Launch all drones simultaneously and climb to mission altitude.

**How it works**:
1. **Parallel Takeoff**: Initiates `takeoffAsync()` for all drones simultaneously
2. **Future Management**: Stores futures and waits for all takeoffs to complete
3. **Altitude Climb**: Commands all drones to climb to target altitude using `moveToZAsync()`
4. **Verification**: Confirms all drones reached target altitude

**Synchronization Benefits**:
- Reduces total mission time
- Ensures coordinated start
- Prevents collision during takeoff phase

#### 5. Orbital Pattern Execution (`_execute_orbital_pattern()`)

**Purpose**: The core mission logic where each drone visits all other drone positions.

**Detailed Process**:

##### A. Orbit Sequence Generation
```python
# Example for 4 drones:
orbit_sequences = {
    'PX4_Drone1': ['PX4_Drone2', 'PX4_Drone3', 'PX4_Drone4'],
    'PX4_Drone2': ['PX4_Drone1', 'PX4_Drone3', 'PX4_Drone4'],  
    'PX4_Drone3': ['PX4_Drone1', 'PX4_Drone2', 'PX4_Drone4'],
    'PX4_Drone4': ['PX4_Drone1', 'PX4_Drone2', 'PX4_Drone3']
}
```

##### B. Step-by-Step Execution
For each orbit step:

1. **Orbital Approach Movement**:
   ```python
   # Calculate smooth approach position
   orbital_x = target_pos['x'] + orbit_radius * cos(step * π/4)
   orbital_y = target_pos['y'] + orbit_radius * sin(step * π/4)
   ```
   - Uses trigonometric functions for smooth circular approach
   - Prevents direct linear movement that could cause collisions
   - Configurable orbit radius (default: 2 meters)

2. **Precision Positioning**:
   - Move to exact target position at reduced speed (50% of normal speed)
   - Ensures accurate positioning at each drone's location

3. **Stopping and Hovering**:
   - Activate hover mode for stability
   - Stop for specified duration (default: 2 seconds)
   - Perfect for observation, photo capture, or sensor readings

4. **Photo Capture**:
   - Automatically capture photos at each position
   - Timestamped filenames with position information
   - Photos saved to organized directory structure

##### C. Collision Monitoring
- Continuous collision detection during movement
- Emergency procedures if collision limits exceeded
- Smart filtering to ignore landscape/terrain collisions

#### 6. Return to Home (`_return_all_drones_home()`)

**Purpose**: Safely return all drones to their original starting positions.

**Process**:
1. Calculate return paths to initial positions
2. Execute parallel return movements
3. Update distance tracking for efficiency metrics
4. Verify all drones reached home positions

### Advanced Features

#### Safety Systems

1. **Collision Detection**:
   - Real-time collision monitoring during flight
   - Configurable collision limits (default: 5 collisions max)
   - Automatic emergency landing if limits exceeded
   - Smart filtering of landscape/terrain collisions

2. **Error Recovery**:
   - Graceful handling of individual drone failures
   - Mission continuation with remaining operational drones
   - Comprehensive error logging and reporting

3. **Emergency Procedures**:
   - Emergency landing protocols
   - Mission abort capabilities
   - Manual intervention options

#### Mission Analytics

1. **Performance Metrics**:
   - Flight efficiency calculations
   - Distance tracking (planned vs. actual)
   - Mission timing and phase analysis
   - Speed and movement optimization metrics

2. **Mission-Specific Tracking**:
   - Orbit positions visited vs. planned
   - Photos captured per drone
   - Stop duration compliance
   - Cycle time measurements

3. **Quality Assessment**:
   - Mission grading system (A+ to D)
   - Safety score calculations
   - Efficiency ratings
   - Actionable recommendations

#### Photo Management

1. **Organized Storage**:
   ```
   photos/
   └── multi_drone_orbit_mission/
       └── Run_1_2024-01-15_14-30-25/
           ├── PX4_Drone1_at_PX4_Drone2_position_step_1.png
           ├── PX4_Drone1_at_PX4_Drone3_position_step_2.png
           └── ...
   ```

2. **Intelligent Naming**:
   - Drone identifier
   - Target position information
   - Step number for sequence tracking
   - Timestamp for uniqueness

## Usage Examples

### Basic Usage
```bash
# Run with auto-detected drones
python multi_drone_orbit_mission.py

# Specify number of drones
python multi_drone_orbit_mission.py --num_drones 3

# Custom parameters
python multi_drone_orbit_mission.py --altitude 15 --speed 3 --stop_duration 5
```

### Advanced Usage
```bash
# Preview mission without flying
python multi_drone_orbit_mission.py --preview

# Disable collision detection for testing
python multi_drone_orbit_mission.py --disable_collision_detection

# Custom orbit parameters
python multi_drone_orbit_mission.py --orbit_radius 3 --stop_duration 10

# Configure safety settings
python multi_drone_orbit_mission.py --max_collisions 3 --include_landscape_collisions
```

### Command Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--num_drones` | int | auto-detect | Number of drones to use |
| `--altitude` | float | 20 | Flight altitude in meters |
| `--speed` | float | 5 | Flight speed in m/s |
| `--orbit_radius` | float | 2 | Orbital approach radius in meters |
| `--stop_duration` | float | 2 | Stop time at each position in seconds |
| `--preview` | flag | False | Preview mission parameters only |
| `--disable_collision_detection` | flag | False | Disable collision detection |
| `--max_collisions` | int | 5 | Maximum collisions before emergency landing |
| `--include_landscape_collisions` | flag | False | Count landscape collisions |
| `--ignore_objects` | list | [] | Object names to ignore in collision detection |

## Mission Report Example

```
============================================================
MISSION ANALYSIS REPORT
============================================================

MISSION OVERVIEW
Status: SUCCESS
Total Duration: 127.3 seconds (2.1 minutes)
Mission Type: MultiDroneOrbitMission

PERFORMANCE METRICS
Distance Planned: 89.4m
Distance Actual: 94.2m
Flight Efficiency: 94.9%
Average Speed: 3.7m/s

SAFETY METRICS
Collisions Detected: 0/5
Collision Status: SAFE
Emergency Landing: NO

MISSION-SPECIFIC METRICS
Drones Used: 4
Orbit Positions Visited: 12/12
Total Stops Made: 12
Photos Captured: 12
Orbit Cycles Completed: 1
Average Cycle Time: 89.7s
Orbit Efficiency: 94.9%

QUALITY ASSESSMENT
Mission Score: 95/100
Grade: A+ (Exceptional)

NO ERRORS RECORDED
```

## Requirements

### AirSim Setup
- AirSim simulation environment
- Multiple drones configured in `settings.json`
- Appropriate simulation environment (recommended: Blocks or Neighborhood)

### Dependencies
- `cosysairsim` - AirSim Python API
- `numpy` - Numerical computations  
- `cv2 (OpenCV)` - Image processing for photo capture
- `math` - Mathematical calculations
- `time` - Timing and delays
- `datetime` - Timestamp generation

### Settings.json Example
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockSpeed": 1,
    "Vehicles": {
        "PX4_Drone1": {
            "VehicleType": "PX4Multirotor",
            "X": 5, "Y": 0, "Z": 0
        },
        "PX4_Drone2": {
            "VehicleType": "PX4Multirotor", 
            "X": -5, "Y": 0, "Z": 0
        },
        "PX4_Drone3": {
            "VehicleType": "PX4Multirotor",
            "X": 5, "Y": 5, "Z": 0
        },
        "PX4_Drone4": {
            "VehicleType": "PX4Multirotor",
            "X": -5, "Y": 5, "Z": 0
        }
    }
}
```

## Benefits and Applications

### Educational Value
- Demonstrates multi-agent coordination algorithms
- Shows practical implementation of orbital mechanics
- Illustrates safety systems in autonomous vehicles
- Provides comprehensive mission analysis

### Research Applications  
- Multi-drone coordination studies
- Path planning algorithm testing
- Collision avoidance system validation
- Performance optimization research

### Commercial Applications
- Aerial surveying and mapping
- Coordinated inspection missions
- Search and rescue operations
- Environmental monitoring

The Multi-Drone Orbit Mission provides a robust foundation for understanding and implementing sophisticated multi-agent drone operations while maintaining the highest safety standards and comprehensive mission analytics.