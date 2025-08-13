# Rotor Configuration

## Overview

Rotor configuration is crucial for accurate drone simulation. The rotor parameters determine thrust generation, power consumption, and overall vehicle performance. This guide covers all aspects of rotor configuration in Cosys-AirSim.

## Rotor Parameters Structure

### Core Parameters (`RotorParams.hpp`)

```cpp
struct RotorParams {
    // Aerodynamic coefficients
    real_T C_T = 0.109919f;        // Thrust coefficient
    real_T C_P = 0.040164f;        // Torque coefficient
    
    // Physical properties
    real_T air_density = 1.225f;   // Air density (kg/m³)
    real_T max_rpm = 6396.667f;    // Maximum RPM
    real_T propeller_diameter = 0.2286f;  // Propeller diameter (m)
    real_T propeller_height = 0.01f;      // Propeller disk height (m)
    
    // Control system
    real_T control_signal_filter_tc = 0.005f;  // Time constant for filtering
    
    // Calculated values (auto-computed)
    real_T max_thrust;             // Maximum thrust (N)
    real_T max_torque;             // Maximum torque (N·m)
    real_T max_speed;              // Max angular velocity (rad/s)
};
```

## Thrust and Torque Physics

### Fundamental Equations

The rotor physics are based on propeller theory:

```cpp
// Thrust in Newtons
thrust = C_T × ρ × n² × D⁴

// Torque in Newton-meters
torque = C_P × ρ × n² × D⁵ / (2π)
```

Where:
- `C_T` = Thrust coefficient (dimensionless)
- `C_P` = Torque coefficient (dimensionless)
- `ρ` = Air density (kg/m³)
- `n` = Rotational speed (revolutions per second)
- `D` = Propeller diameter (meters)

### Coefficient Sources

The default coefficients are for **GWS 9×5 propellers**:
- `C_T = 0.109919` at 6396.667 RPM
- `C_P = 0.040164` at 6396.667 RPM

**Data source:** University of Illinois Urbana-Champaign Propeller Database
**URL:** http://m-selig.ae.illinois.edu/props/propDB.html

## Rotor Positioning

### Position Structure

```cpp
struct RotorPose {
    Vector3r position;           // Relative to center of gravity
    Vector3r normal;            // Thrust direction vector
    RotorTurningDirection direction;  // CW or CCW
};
```

### Coordinate System

All positions are relative to the vehicle's center of gravity in NED coordinates:
- **X-axis:** Forward (North)
- **Y-axis:** Right (East)
- **Z-axis:** Down

### Turning Directions

```cpp
enum class RotorTurningDirection : int {
    RotorTurningDirectionCCW = -1,  // Counter-clockwise
    RotorTurningDirectionCW = 1     // Clockwise
};
```

## Common Rotor Configurations

### 1. Standard Quadcopter Parameters

#### Generic Consumer Drone
```cpp
// DJI Phantom 2 inspired configuration
rotor_params.C_T = 0.109919f;
rotor_params.C_P = 0.040164f;
rotor_params.max_rpm = 6396.667f;
rotor_params.propeller_diameter = 0.2286f;  // 9 inches
rotor_params.air_density = 1.225f;
```

#### Racing Drone
```cpp
// High-performance racing configuration
rotor_params.C_T = 0.08f;        // Lower for higher RPM
rotor_params.C_P = 0.03f;        // Lower torque coefficient
rotor_params.max_rpm = 12000f;   // High RPM motors
rotor_params.propeller_diameter = 0.127f;  // 5 inches
```

#### Heavy-Lift Drone
```cpp
// Large propeller, high thrust configuration
rotor_params.C_T = 0.15f;        // Higher thrust coefficient
rotor_params.C_P = 0.06f;        // Higher torque coefficient
rotor_params.max_rpm = 5000f;    // Lower RPM for large props
rotor_params.propeller_diameter = 0.381f;  // 15 inches
```

### 2. Hexacopter Parameters

```cpp
// Optimized for stability and payload
rotor_params.C_T = 0.12f;
rotor_params.C_P = 0.045f;
rotor_params.max_rpm = 7000f;
rotor_params.propeller_diameter = 0.254f;  // 10 inches
```

### 3. Octocopter Parameters

```cpp
// Professional cinematography configuration
rotor_params.C_T = 0.13f;
rotor_params.C_P = 0.05f;
rotor_params.max_rpm = 6500f;
rotor_params.propeller_diameter = 0.279f;  // 11 inches
```

## Propeller Selection Guidelines

### Size vs. Performance Trade-offs

| Propeller Size | Thrust Efficiency | Responsiveness | Noise | Power Draw |
|----------------|-------------------|----------------|-------|------------|
| Small (5-7")   | Low               | High           | Low   | Low        |
| Medium (8-10") | Medium            | Medium         | Medium| Medium     |
| Large (12-15") | High              | Low            | High  | High       |

### Pitch Considerations

```cpp
// Typical pitch ratios for different applications
// Racing: High pitch (5x4.5, 6x4.5)
// Efficiency: Medium pitch (9x5, 10x5) 
// Stability: Low pitch (12x4, 15x4)
```

## Motor Specifications

### Brushless Motor Parameters

Common motor specifications and their rotor parameter equivalents:

#### Example: T-Motor MN3508
```cpp
// Real motor specs: 700KV, 25A max
rotor_params.max_rpm = 700 * 11.1f;  // KV × voltage
rotor_params.propeller_diameter = 0.330f;  // 13 inches
rotor_params.C_T = 0.14f;  // Estimated from thrust data
rotor_params.C_P = 0.055f; // Estimated from torque data
```

#### Example: DJI 2312E
```cpp
// Real motor specs: 960KV, 17A max
rotor_params.max_rpm = 960 * 14.8f;  // KV × voltage
rotor_params.propeller_diameter = 0.240f;  // 9.4 inches
rotor_params.C_T = 0.11f;
rotor_params.C_P = 0.042f;
```

## Thrust Calculation and Validation

### Maximum Thrust Calculation

```cpp
void calculateMaxThrust() {
    // Convert RPM to revolutions per second
    revolutions_per_second = max_rpm / 60.0f;
    
    // Calculate angular velocity
    max_speed = revolutions_per_second * 2.0f * M_PI;
    max_speed_square = max_speed * max_speed;
    
    // Calculate thrust and torque
    real_T nsquared = revolutions_per_second * revolutions_per_second;
    max_thrust = C_T * air_density * nsquared * pow(propeller_diameter, 4);
    max_torque = C_P * air_density * nsquared * pow(propeller_diameter, 5) / (2.0f * M_PI);
}
```

### Thrust-to-Weight Ratio

```cpp
// Calculate thrust-to-weight ratio for vehicle
float total_max_thrust = rotor_count * rotor_params.max_thrust;
float vehicle_weight = mass * 9.81f;  // Weight in Newtons
float thrust_to_weight = total_max_thrust / vehicle_weight;

// Recommended ratios:
// Stable flight: T/W > 1.5
// Aerobatic flight: T/W > 2.0
// Racing: T/W > 3.0
```

## Environmental Factors

### Air Density Effects

Air density affects thrust and power consumption:

```cpp
// Standard air density variations
real_T air_density_sea_level = 1.225f;     // kg/m³
real_T air_density_1000m = 1.112f;         // kg/m³ at 1000m altitude
real_T air_density_2000m = 1.007f;         // kg/m³ at 2000m altitude

// Temperature effects (approximate)
real_T air_density_cold = 1.292f;          // kg/m³ at 0°C
real_T air_density_hot = 1.184f;           // kg/m³ at 30°C
```

### Altitude Compensation

```cpp
// Altitude-based air density calculation
real_T calculateAirDensity(real_T altitude_m) {
    const real_T sea_level_density = 1.225f;
    const real_T scale_height = 8400.0f;  // meters
    
    return sea_level_density * exp(-altitude_m / scale_height);
}
```

## Advanced Configuration

### Dynamic Rotor Parameters

For advanced simulations, rotor parameters can be modified during runtime:

```cpp
// Example: Adaptive rotor parameters based on flight conditions
void updateRotorParams(real_T altitude, real_T temperature) {
    // Update air density
    rotor_params.air_density = calculateAirDensity(altitude);
    
    // Recalculate maximum thrust
    rotor_params.calculateMaxThrust();
    
    // Adjust control filtering based on conditions
    if (altitude > 1000.0f) {
        rotor_params.control_signal_filter_tc = 0.008f;  // More filtering
    }
}
```

### Rotor Failure Simulation

```cpp
// Simulate rotor failure
void simulateRotorFailure(int rotor_index) {
    // Set failed rotor thrust to zero
    rotor_params.max_thrust = 0.0f;
    rotor_params.max_torque = 0.0f;
    
    // Update vehicle dynamics to handle asymmetric thrust
}
```

## Testing and Validation

### Hover Performance Test

```cpp
// Test hover capability
void testHoverPerformance() {
    float hover_thrust_per_rotor = (mass * 9.81f) / rotor_count;
    float hover_throttle = hover_thrust_per_rotor / rotor_params.max_thrust;
    
    // Hover throttle should be 40-60% for good control authority
    assert(hover_throttle >= 0.4f && hover_throttle <= 0.6f);
}
```

### Thrust Validation

```cpp
// Validate thrust calculations against real-world data
void validateThrust() {
    // Example: Verify thrust at known RPM
    real_T test_rpm = 5000.0f;
    real_T test_n = test_rpm / 60.0f;
    real_T calculated_thrust = rotor_params.C_T * rotor_params.air_density * 
                               test_n * test_n * pow(rotor_params.propeller_diameter, 4);
    
    // Compare with manufacturer specifications
    real_T expected_thrust = 8.5f;  // Newtons from datasheet
    real_T error = abs(calculated_thrust - expected_thrust) / expected_thrust;
    
    assert(error < 0.1f);  // Less than 10% error
}
```

## Common Issues and Solutions

### 1. Insufficient Thrust

**Problem:** Drone cannot takeoff or maintain altitude
**Solutions:**
- Increase `C_T` coefficient
- Increase `max_rpm`
- Use larger propellers (increase `propeller_diameter`)
- Reduce vehicle mass

### 2. Oscillations and Instability

**Problem:** Drone oscillates or is difficult to control
**Solutions:**
- Adjust `control_signal_filter_tc` (increase for more filtering)
- Check rotor placement symmetry
- Verify inertia calculations
- Tune flight controller parameters

### 3. Unrealistic Performance

**Problem:** Drone performs better/worse than expected
**Solutions:**
- Validate `C_T` and `C_P` coefficients against real data
- Check air density settings
- Verify propeller diameter measurements
- Compare with manufacturer thrust curves

## Real-World Data Integration

### Obtaining Propeller Data

1. **Manufacturer specifications**
   - Thrust curves at various RPMs
   - Power consumption data
   - Efficiency curves

2. **Testing facilities**
   - Wind tunnel data
   - Test stand measurements
   - Flight test validation

3. **Databases**
   - UIUC Propeller Database
   - APC Propeller data
   - Academic research papers

### Converting Test Data

```cpp
// Convert thrust curve data to C_T coefficient
real_T calculateCT(real_T thrust_N, real_T rpm, real_T diameter_m) {
    real_T n = rpm / 60.0f;  // Convert to rev/sec
    real_T air_density = 1.225f;  // Standard conditions
    
    return thrust_N / (air_density * n * n * pow(diameter_m, 4));
}
```

## Next Steps

- Configure [physical properties](physical-properties.md) for complete vehicle setup
- Integrate with [settings.json](settings-integration.md) for easy parameter management
- Learn about [performance tuning](performance-tuning.md) for optimization
- Explore [custom drone creation](custom-drone-creation.md) for specialized applications