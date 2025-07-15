# Settings Integration

## Overview

The `settings.json` file provides a convenient way to configure drone parameters without modifying C++ code. This guide covers how to integrate drone configurations with the settings system and manage multiple vehicle configurations.

## Settings File Structure

### Basic Vehicle Configuration

```json
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "X": 0, "Y": 0, "Z": -1,
      "Yaw": 0,
      "EnableCollisions": true,
      "EnablePassthroughOnCollisions": false,
      "AllowAPIAlways": true,
      "RC": {
        "RemoteControlID": 0,
        "AllowAPIWhenDisconnected": false
      }
    }
  }
}
```

### Vehicle Types

| Vehicle Type | Description | Flight Stack |
|-------------|-------------|--------------|
| `"SimpleFlight"` | Built-in flight controller | Internal |
| `"PX4"` | PX4 autopilot integration | External |
| `"ArduCopter"` | ArduPilot integration | External |
| `"ArduCopterSolo"` | 3DR Solo configuration | External |

## Model Selection

### PX4 Models

For PX4 vehicles, the `Model` field selects predefined configurations:

```json
{
  "Vehicles": {
    "BasicQuad": {
      "VehicleType": "PX4",
      "Model": "Generic"
    },
    "RacingDrone": {
      "VehicleType": "PX4", 
      "Model": "Blacksheep"
    },
    "PhotoDrone": {
      "VehicleType": "PX4",
      "Model": "Flamewheel"
    },
    "HeavyLifter": {
      "VehicleType": "PX4",
      "Model": "FlamewheelFLA"
    },
    "Hexacopter": {
      "VehicleType": "PX4",
      "Model": "Hexacopter"
    },
    "Octocopter": {
      "VehicleType": "PX4",
      "Model": "Octocopter"
    }
  }
}
```

### Model Specifications

#### Generic Quad
```json
{
  "Model": "Generic",
  "// Configuration": {
    "mass": "1.0 kg",
    "rotor_count": 4,
    "arm_length": "0.2275 m",
    "thrust_to_weight": "~1.6"
  }
}
```

#### Flamewheel
```json
{
  "Model": "Flamewheel",
  "// Configuration": {
    "mass": "1.635 kg", 
    "rotor_count": 4,
    "arm_length": "0.225 m",
    "max_rpm": 9500,
    "thrust_to_weight": "~2.0"
  }
}
```

#### Blacksheep (Team Blacksheep Discovery)
```json
{
  "Model": "Blacksheep",
  "// Configuration": {
    "mass": "2.0 kg",
    "rotor_count": 4,
    "asymmetric_arms": true,
    "racing_optimized": true
  }
}
```

## Multi-Vehicle Configuration

### Swarm Configuration

```json
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "Leader": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "X": 0, "Y": 0, "Z": -1,
      "Yaw": 0
    },
    "Follower1": {
      "VehicleType": "PX4", 
      "Model": "Generic",
      "X": 5, "Y": 0, "Z": -1,
      "Yaw": 0
    },
    "Follower2": {
      "VehicleType": "PX4",
      "Model": "Generic", 
      "X": 0, "Y": 5, "Z": -1,
      "Yaw": 0
    }
  }
}
```

### Mixed Vehicle Types

```json
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "ResearchDrone": {
      "VehicleType": "SimpleFlight",
      "Model": "Generic",
      "X": 0, "Y": 0, "Z": -1
    },
    "TestDrone": {
      "VehicleType": "PX4",
      "Model": "Flamewheel",
      "X": 10, "Y": 0, "Z": -1
    },
    "RacingDrone": {
      "VehicleType": "PX4",
      "Model": "Blacksheep", 
      "X": 20, "Y": 0, "Z": -1
    }
  }
}
```

## PX4 Integration Settings

### MAVLink Configuration

```json
{
  "Vehicles": {
    "PX4Drone": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "127.0.0.1",
      "LogViewerHostIp": "127.0.0.1",
      "LogViewerPort": 14388,
      "OffboardCompID": 1,
      "TargetCompID": 1,
      "VehicleCompID": 1,
      "QgcHostIp": "127.0.0.1",
      "QgcPort": 14550
    }
  }
}
```

### Multi-Vehicle PX4 Setup

```json
{
  "Vehicles": {
    "PX4_1": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "SitlIp": "127.0.0.1",
      "SitlPort": 14556,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14560,
      "UseSerial": false
    },
    "PX4_2": {
      "VehicleType": "PX4",
      "Model": "Generic", 
      "TcpPort": 4570,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14581,
      "SitlIp": "127.0.0.1",
      "SitlPort": 14557,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14561,
      "UseSerial": false
    }
  }
}
```

## Sensor Configuration

### Basic Sensor Setup

```json
{
  "Vehicles": {
    "SensorDrone": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "Sensors": {
        "IMU": {
          "SensorType": 2,
          "Enabled": true
        },
        "GPS": {
          "SensorType": 3,
          "Enabled": true
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled": true
        },
        "Barometer": {
          "SensorType": 1,
          "Enabled": true
        }
      }
    }
  }
}
```

### Camera Configuration

```json
{
  "Vehicles": {
    "CameraDrone": {
      "VehicleType": "PX4",
      "Model": "Flamewheel",
      "Cameras": {
        "front_center": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.30, "Y": 0.00, "Z": 0.00,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        },
        "bottom_center": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.00, "Y": 0.00, "Z": 0.10,
          "Pitch": -90.0, "Roll": 0.0, "Yaw": 0.0
        }
      }
    }
  }
}
```

## Advanced Configuration

### Custom Parameters

While the settings.json doesn't directly expose all physical parameters, you can create custom vehicle configurations by extending the code:

```cpp
// Create custom settings handler
class CustomVehicleSettings : public AirSimSettings::VehicleSetting {
public:
    // Custom parameters
    real_T custom_mass = 1.2f;
    real_T custom_arm_length = 0.25f;
    real_T custom_drag_coefficient = 0.3f;
    
    // Load from settings
    void loadFromSettings(const Settings& settings) {
        custom_mass = settings.getFloat("CustomMass", custom_mass);
        custom_arm_length = settings.getFloat("CustomArmLength", custom_arm_length);
        custom_drag_coefficient = settings.getFloat("CustomDrag", custom_drag_coefficient);
    }
};
```

### Environment-Specific Settings

```json
{
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "TestDrone": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "// Environment": "High altitude testing",
      "// AirDensity": 0.9,
      "// WindSpeed": 5.0
    }
  },
  "Wind": {
    "X": 2.0, "Y": 1.0, "Z": 0.0
  }
}
```

## Configuration Templates

### Research Template

```json
{
  "SimMode": "Multirotor",
  "ViewMode": "SpringArmChase",
  "Vehicles": {
    "ResearchDrone": {
      "VehicleType": "SimpleFlight",
      "Model": "Generic",
      "AutoCreate": true,
      "PawnBP": "DefaultQuadrotor",
      "EnableCollisions": true,
      "EnablePassthroughOnCollisions": false,
      "AllowAPIAlways": true,
      "RC": {
        "RemoteControlID": 0,
        "AllowAPIWhenDisconnected": false
      },
      "Sensors": {
        "IMU": {
          "SensorType": 2,
          "Enabled": true,
          "DrawDebugPoints": false
        },
        "GPS": {
          "SensorType": 3,
          "Enabled": true,
          "DrawDebugPoints": false
        }
      }
    }
  }
}
```

### Production Template

```json
{
  "SimMode": "Multirotor",
  "ViewMode": "NoDisplay",
  "Vehicles": {
    "ProductionDrone": {
      "VehicleType": "PX4",
      "Model": "Flamewheel",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4560,
      "EnableCollisions": true,
      "EnablePassthroughOnCollisions": false,
      "AllowAPIAlways": true,
      "Sensors": {
        "IMU": {"SensorType": 2, "Enabled": true},
        "GPS": {"SensorType": 3, "Enabled": true},
        "Magnetometer": {"SensorType": 4, "Enabled": true},
        "Barometer": {"SensorType": 1, "Enabled": true}
      },
      "Cameras": {
        "front": {
          "CaptureSettings": [
            {"ImageType": 0, "Width": 1920, "Height": 1080}
          ]
        }
      }
    }
  }
}
```

### Racing Template

```json
{
  "SimMode": "Multirotor",
  "ViewMode": "FPV",
  "Vehicles": {
    "RacingDrone": {
      "VehicleType": "PX4",
      "Model": "Blacksheep",
      "EnableCollisions": true,
      "EnablePassthroughOnCollisions": false,
      "AllowAPIAlways": true,
      "Cameras": {
        "fpv": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 120
            }
          ],
          "X": 0.10, "Y": 0.00, "Z": 0.00,
          "Pitch": -10.0, "Roll": 0.0, "Yaw": 0.0
        }
      }
    }
  }
}
```

## Settings File Management

### File Locations

Settings files are loaded in priority order:

1. **Command line**: `--settings path/to/settings.json`
2. **Executable directory**: `./settings.json`
3. **User directory**: `~/Documents/AirSim/settings.json`

### Multiple Configurations

```bash
# Different configurations for different scenarios
cp settings_research.json ~/Documents/AirSim/settings.json    # Research mode
cp settings_production.json ~/Documents/AirSim/settings.json  # Production mode
cp settings_racing.json ~/Documents/AirSim/settings.json     # Racing mode
```

### Environment Variables

```bash
# Set custom settings path
export AIRSIM_SETTINGS_PATH="/path/to/custom/settings.json"

# Use different settings for different environments
export AIRSIM_SETTINGS_PATH="/configs/development_settings.json"  # Development
export AIRSIM_SETTINGS_PATH="/configs/production_settings.json"   # Production
```

## Validation and Testing

### Settings Validation

```cpp
// Validate settings configuration
bool validateSettings(const AirSimSettings& settings) {
    // Check vehicle count
    if (settings.vehicles.size() == 0) {
        return false;
    }
    
    // Validate each vehicle
    for (const auto& vehicle : settings.vehicles) {
        // Check vehicle type
        if (vehicle.second.vehicle_type.empty()) {
            return false;
        }
        
        // Check model for PX4
        if (vehicle.second.vehicle_type == "PX4" && 
            vehicle.second.model.empty()) {
            return false;
        }
    }
    
    return true;
}
```

### Configuration Testing

```bash
# Test configuration with AirSim
./AirSimNH.exe --settings test_settings.json

# Validate JSON syntax
python -m json.tool settings.json

# Check for common issues
jq '.Vehicles | keys' settings.json  # List vehicle names
jq '.Vehicles[].VehicleType' settings.json  # List vehicle types
```

## Common Issues and Solutions

### 1. Vehicle Not Spawning

**Problem**: Vehicle doesn't appear in simulation
**Solutions**:
- Check `SimMode` is set to "Multirotor"
- Verify `VehicleType` is valid
- Ensure vehicle position is above ground (negative Z)
- Check for JSON syntax errors

### 2. PX4 Connection Issues

**Problem**: PX4 can't connect to AirSim
**Solutions**:
- Verify MAVLink port settings
- Check IP addresses and ports
- Ensure PX4 SITL is running
- Validate TCP/UDP configuration

### 3. Sensor Not Working

**Problem**: Sensors not providing data
**Solutions**:
- Check sensor is enabled in settings
- Verify sensor type is correct
- Ensure sensor position is valid
- Check API access permissions

### 4. Performance Issues

**Problem**: Simulation running slowly
**Solutions**:
- Reduce number of vehicles
- Disable unnecessary sensors
- Use "NoDisplay" view mode
- Optimize camera settings

## Best Practices

1. **Use version control** for settings files
2. **Create templates** for common configurations
3. **Test configurations** before deployment
4. **Document custom parameters** in comments
5. **Validate JSON syntax** before use
6. **Use meaningful vehicle names** for debugging
7. **Keep backup configurations** for different scenarios

## Next Steps

- Learn about [custom drone creation](custom-drone-creation.md) for advanced configurations
- Explore [performance tuning](performance-tuning.md) for optimization
- Review [troubleshooting guide](troubleshooting.md) for common issues
- Check out the [overview](overview.md) for architecture understanding