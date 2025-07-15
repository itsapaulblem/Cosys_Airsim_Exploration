# Multi-Agent Drone Auto-Discovery Scripts

This directory contains enhanced multi-agent drone scripts that automatically discover available vehicles and make them take off.

## Scripts Overview

### 1. `multi_agent_drone_auto.py` (Colosseum Version)
- **For**: Original AirSim/Colosseum users
- **Discovery Method**: Smart detection by trying common vehicle names
- **Features**: Works without `listVehicles()` API

### 2. `../PythonClient/multirotor/multi_agent_drone_auto_cosys.py` (Cosys-AirSim Version)  
- **For**: Cosys-AirSim users
- **Discovery Method**: Uses native `client.listVehicles()` API
- **Features**: More reliable discovery, enhanced sensor support

## Usage Instructions

### Prerequisites

1. **AirSim Running**: Make sure AirSim simulation is running
2. **Vehicle Configuration**: Ensure your `settings.json` contains vehicle definitions
3. **Python Environment**: Install required dependencies

### Example settings.json

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockSpeed": 1,
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 0, "Y": 0, "Z": -2
        },
        "Drone2": {
            "VehicleType": "SimpleFlight", 
            "X": 4, "Y": 0, "Z": -2
        },
        "Drone3": {
            "VehicleType": "SimpleFlight",
            "X": 8, "Y": 0, "Z": -2
        }
    }
}
```

### Running the Scripts

#### For Colosseum AirSim:
```bash
cd PythonClientColosseum/multirotor
python multi_agent_drone_auto.py
```

#### For Cosys-AirSim:
```bash
cd PythonClient/multirotor  
python multi_agent_drone_auto_cosys.py
```

## Script Features

### Auto-Discovery
- **Colosseum**: Tries common vehicle naming patterns (Drone1, Drone2, UAV1, etc.)
- **Cosys-AirSim**: Uses `listVehicles()` API for reliable discovery

### Multi-Vehicle Operations
1. **Setup**: Enables API control and arms all discovered vehicles
2. **Takeoff**: Coordinates simultaneous takeoff for all multirotors
3. **Formation**: Moves vehicles into a formation pattern
4. **State Monitoring**: Displays position and status of all vehicles
5. **Image Capture**: Takes photos from all vehicle cameras
6. **Cleanup**: Properly disarms and resets all vehicles

### Error Handling
- Individual vehicle failure doesn't stop other operations
- Clear error messages for debugging
- Graceful cleanup on exit or interruption

### Interactive Controls
- Press any key prompts at each major step
- Allows inspection of vehicle states
- User-controlled progression through demo

## Key Functions

### Vehicle Discovery
```python
# Colosseum version
vehicles = discover_vehicles(client, max_vehicles=10)

# Cosys-AirSim version  
vehicles = client.listVehicles()
```

### Batch Takeoff
```python
# Start takeoff for all vehicles
takeoff_futures = []
for vehicle in vehicles:
    future = client.takeoffAsync(vehicle_name=vehicle)
    takeoff_futures.append(future)

# Wait for completion
for future in takeoff_futures:
    future.join()
```

### Formation Movement
```python
# Move to formation positions
for i, vehicle in enumerate(vehicles):
    x = -5 + (i * 3)  # Spread along X axis
    y = 5 - (i * 2)   # Stagger Y positions
    z = -10           # Fixed altitude
    client.moveToPositionAsync(x, y, z, 5, vehicle_name=vehicle)
```

## Troubleshooting

### Common Issues

1. **No Vehicles Found**
   - Check `settings.json` vehicle configuration
   - Ensure AirSim is running and accessible
   - Verify vehicle names match expected patterns (for Colosseum version)

2. **Connection Failed**
   - Ensure AirSim is running on localhost:41451
   - Check firewall settings
   - Verify no other clients are blocking the connection

3. **Takeoff Failed**
   - Ensure vehicles are properly armed
   - Check for collisions or obstacles
   - Verify vehicles have enough space for takeoff

4. **API Version Mismatch**
   - Colosseum script: Use with original AirSim
   - Cosys-AirSim script: Use with Cosys-AirSim only

### Debug Tips

- Enable verbose logging in the scripts
- Check AirSim console for error messages
- Test with a single vehicle first
- Verify vehicle types match script expectations

## Advanced Usage

### Custom Vehicle Patterns (Colosseum)
Modify the `vehicle_patterns` list in `discover_vehicles()`:

```python
vehicle_patterns = [
    ["MyDrone1", "MyDrone2", "MyDrone3"],
    ["UAV_Alpha", "UAV_Beta", "UAV_Gamma"],
    # Add your custom patterns here
]
```

### Mixed Vehicle Types (Cosys-AirSim)
The Cosys-AirSim version supports multiple vehicle types:

```json
{
    "Vehicles": {
        "Drone1": {"VehicleType": "SimpleFlight"},
        "Car1": {"VehicleType": "PhysXCar"},
        "Camera1": {"VehicleType": "ComputerVision"}
    }
}
```

### Custom Formation Patterns
Modify the formation logic in `move_vehicle_to_formation()`:

```python
# Custom circular formation
import math
radius = 10
angle = (2 * math.pi * index) / total_vehicles
x = radius * math.cos(angle)
y = radius * math.sin(angle)
z = -10
```

## Integration with ROS2

These scripts can be easily adapted for ROS2 integration by:

1. Adding ROS2 publishers for vehicle states
2. Converting to ROS2 action servers for takeoff/movement
3. Using ROS2 parameters for configuration
4. Publishing sensor data to ROS2 topics

## Next Steps

- Experiment with different formation patterns
- Add mission planning capabilities
- Integrate with path planning algorithms
- Add real-time monitoring and control interfaces