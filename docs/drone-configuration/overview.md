# Drone Configuration Overview

## Architecture

The Cosys-AirSim drone configuration system is built around a modular architecture that allows for flexible customization of multirotor vehicles. The system consists of several key components:

### Core Components

```
MultiRotorParams (Base Class)
├── Params Structure
│   ├── Physical Properties (mass, inertia, body_box)
│   ├── Rotor Configuration (count, poses, parameters)
│   └── Aerodynamic Properties (drag coefficients, restitution)
├── Frame Setup Methods
│   ├── setupFrameGenericQuad()
│   ├── setupFrameGenericHex()
│   └── setupFrameGenericOcto()
└── Firmware-Specific Implementations
    ├── SimpleFlightQuadXParams
    ├── Px4MultiRotorParams
    └── ArduCopterParams
```

### Key Files

| File | Purpose |
|------|---------|
| `MultiRotorParams.hpp` | Base class defining drone parameters structure |
| `RotorParams.hpp` | Individual rotor characteristics and physics |
| `MultiRotorParamsFactory.hpp` | Factory for creating vehicle configurations |
| `SimpleFlightQuadXParams.hpp` | SimpleFlight-specific parameters |
| `Px4MultiRotorParams.hpp` | PX4-specific parameters with model selection |

## Configuration Flow

```mermaid
graph TD
    A[settings.json] --> B[MultiRotorParamsFactory]
    B --> C[Vehicle Type Selection]
    C --> D[Firmware-Specific Params]
    D --> E[setupParams()]
    E --> F[Frame Configuration]
    F --> G[Rotor Positioning]
    G --> H[Physics Calculation]
    H --> I[Sensor Integration]
    I --> J[Ready for Simulation]
```

## Parameter Categories

### 1. Required Parameters

These must be configured for any drone:

- **`rotor_count`** - Number of rotors (4, 6, or 8)
- **`rotor_poses`** - 3D position and rotation direction of each rotor
- **`mass`** - Total vehicle mass in kg
- **`inertia`** - 3x3 inertia matrix (auto-calculated from geometry)
- **`body_box`** - Dimensions of the main body (x, y, z in meters)

### 2. Optional Parameters with Defaults

These have sensible defaults but can be customized:

- **`linear_drag_coefficient`** - Air resistance for translational motion (default: 0.325)
- **`angular_drag_coefficient`** - Air resistance for rotational motion (default: 0.325)
- **`restitution`** - Collision bounce factor (default: 0.55)
- **`friction`** - Surface friction coefficient (default: 0.5)
- **`rotor_params`** - Individual rotor characteristics

### 3. Rotor Parameters

Each rotor is configured with:

- **`C_T`** - Thrust coefficient (default: 0.109919)
- **`C_P`** - Torque coefficient (default: 0.040164)
- **`max_rpm`** - Maximum revolutions per minute (default: 6396.667)
- **`propeller_diameter`** - Propeller diameter in meters (default: 0.2286)
- **`air_density`** - Air density in kg/m³ (default: 1.225)

## Coordinate System

Cosys-AirSim uses the NED (North-East-Down) coordinate system:

- **X-axis** - Forward (North)
- **Y-axis** - Right (East)  
- **Z-axis** - Down
- **Rotations** - Right-hand rule around each axis

### Rotor Positioning

Rotors are positioned relative to the vehicle's center of gravity:

```
Quadcopter X Configuration:
     x-axis (forward)
  (2)  |   (0)
       |
  ------------ y-axis (right)
       |
  (1)  |   (3)

Rotor spinning directions:
- 0, 1: Counter-clockwise (CCW)
- 2, 3: Clockwise (CW)
```

## Physics Calculations

### Thrust and Torque

The system calculates rotor thrust and torque using:

```cpp
// Thrust in Newtons
thrust = C_T * air_density * n² * D⁴

// Torque in N·m  
torque = C_P * air_density * n² * D⁵ / (2π)
```

Where:
- `n` = revolutions per second
- `D` = propeller diameter in meters
- `C_T`, `C_P` = dimensionless coefficients from propeller database

### Inertia Matrix

The inertia matrix is automatically calculated from:

```cpp
// Body inertia (rectangular box approximation)
I_xx = box_mass/12 * (body_y² + body_z²)
I_yy = box_mass/12 * (body_x² + body_z²)  
I_zz = box_mass/12 * (body_x² + body_y²)

// Add motor contributions
for each rotor:
    I_xx += motor_mass * (rotor_y² + rotor_z²)
    I_yy += motor_mass * (rotor_x² + rotor_z²)
    I_zz += motor_mass * (rotor_x² + rotor_y²)
```

## Vehicle Type Integration

Different vehicle types use the same parameter structure but with different APIs:

### SimpleFlight
- Built-in flight controller
- Direct parameter usage
- Good for research and development

### PX4
- External PX4 autopilot integration
- MAVLink communication
- Production-ready flight stack
- Model-specific configurations

### ArduCopter  
- ArduPilot integration
- MAVLink communication
- Extensive parameter customization

## Configuration Best Practices

1. **Start with existing frame** - Use pre-built configurations as templates
2. **Validate mass limits** - Ensure total mass is within rotor thrust capabilities
3. **Consider real-world constraints** - Use realistic motor and propeller specifications
4. **Test incrementally** - Make small changes and test each modification
5. **Document custom configurations** - Keep notes on parameter choices and performance

## Next Steps

- Learn about specific [frame types](frame-types.md)
- Understand [rotor configuration](rotor-configuration.md) in detail
- Explore [physical properties](physical-properties.md) tuning
- Set up [settings.json integration](settings-integration.md)