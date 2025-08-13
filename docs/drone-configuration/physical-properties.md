# Physical Properties Configuration

## Overview

Physical properties define the mass, inertia, and aerodynamic characteristics of the drone. These parameters directly affect flight dynamics, stability, and control response. Proper configuration is essential for realistic simulation.

## Mass Properties

### Total Mass Configuration

```cpp
// Total vehicle mass in kilograms
params.mass = 1.5f;  // Example: 1.5 kg drone

// Component mass breakdown
real_T motor_assembly_weight = 0.055f;  // Per motor (motor + ESC + prop)
real_T battery_weight = 0.3f;          // Battery pack
real_T payload_weight = 0.2f;          // Camera/gimbal/sensors
real_T frame_weight = 0.4f;            // Frame and electronics

// Validate total mass
real_T calculated_mass = rotor_count * motor_assembly_weight + 
                        battery_weight + payload_weight + frame_weight;
assert(abs(params.mass - calculated_mass) < 0.05f);
```

### Mass Distribution Guidelines

| Component | Typical % of Total Mass | Notes |
|-----------|------------------------|-------|
| Frame | 25-35% | Carbon fiber lighter than aluminum |
| Motors/ESCs | 20-30% | Scales with rotor count |
| Battery | 25-40% | Largest single component |
| Payload | 10-25% | Cameras, gimbals, sensors |
| Electronics | 5-15% | Flight controller, wiring |

### Mass Limits by Frame Type

```cpp
// Mass limits based on rotor configuration and default parameters
struct MassLimits {
    real_T min_mass;  // Minimum for stable flight
    real_T max_mass;  // Maximum for liftoff capability
};

// Default rotor parameters (C_T = 0.109919, max_rpm = 6396.667)
MassLimits quad_limits = {0.8f, 1.6f};    // 4 rotors
MassLimits hex_limits = {1.25f, 2.5f};    // 6 rotors  
MassLimits octo_limits = {1.6f, 3.2f};    // 8 rotors
```

## Inertia Matrix

### Automatic Calculation

The inertia matrix is automatically calculated using the body box approximation plus rotor contributions:

```cpp
void computeInertiaMatrix(Matrix3x3r& inertia, 
                         const Vector3r& body_box,
                         const vector<RotorPose>& rotor_poses,
                         real_T box_mass, 
                         real_T motor_assembly_weight) {
    
    // Initialize inertia matrix
    inertia = Matrix3x3r::Zero();
    
    // Body contribution (rectangular box approximation)
    inertia(0, 0) = box_mass / 12.0f * (body_box.y() * body_box.y() + 
                                       body_box.z() * body_box.z());
    inertia(1, 1) = box_mass / 12.0f * (body_box.x() * body_box.x() + 
                                       body_box.z() * body_box.z());
    inertia(2, 2) = box_mass / 12.0f * (body_box.x() * body_box.x() + 
                                       body_box.y() * body_box.y());
    
    // Add rotor contributions (point mass approximation)
    for (const auto& rotor : rotor_poses) {
        const Vector3r& pos = rotor.position;
        inertia(0, 0) += motor_assembly_weight * (pos.y() * pos.y() + pos.z() * pos.z());
        inertia(1, 1) += motor_assembly_weight * (pos.x() * pos.x() + pos.z() * pos.z());
        inertia(2, 2) += motor_assembly_weight * (pos.x() * pos.x() + pos.y() * pos.y());
    }
}
```

### Understanding Inertia Components

- **I_xx (Roll inertia)**: Resistance to rotation around X-axis (forward)
- **I_yy (Pitch inertia)**: Resistance to rotation around Y-axis (right)
- **I_zz (Yaw inertia)**: Resistance to rotation around Z-axis (down)

### Inertia Effects on Flight Characteristics

```cpp
// Higher inertia = More stable but less responsive
// Lower inertia = More responsive but less stable

// Example inertia values for different configurations
struct InertiaExamples {
    // Racing drone (lightweight, responsive)
    Matrix3x3r racing = {{0.025f, 0.0f, 0.0f},
                        {0.0f, 0.025f, 0.0f},
                        {0.0f, 0.0f, 0.05f}};
    
    // Photography drone (stable, smooth)
    Matrix3x3r photography = {{0.08f, 0.0f, 0.0f},
                              {0.0f, 0.08f, 0.0f},
                              {0.0f, 0.0f, 0.15f}};
};
```

## Body Dimensions

### Body Box Configuration

```cpp
// Body box defines the main body dimensions (not including arms)
params.body_box.x() = 0.18f;  // Length (forward/back)
params.body_box.y() = 0.11f;  // Width (left/right)
params.body_box.z() = 0.04f;  // Height (up/down)
```

### Sizing Guidelines

| Drone Type | Length | Width | Height | Volume |
|------------|--------|-------|--------|---------|
| Micro (<250g) | 0.08-0.12m | 0.08-0.12m | 0.02-0.04m | ~0.0001m³ |
| Consumer | 0.15-0.25m | 0.10-0.20m | 0.04-0.08m | ~0.001m³ |
| Professional | 0.30-0.50m | 0.20-0.40m | 0.08-0.15m | ~0.01m³ |
| Industrial | 0.50-1.0m | 0.40-0.80m | 0.15-0.30m | ~0.1m³ |

### Body Shape Approximations

```cpp
// Rectangular box (default)
// Good for: Most commercial drones
void setupRectangularBody(Params& params) {
    params.body_box.x() = 0.20f;
    params.body_box.y() = 0.15f;
    params.body_box.z() = 0.06f;
}

// Circular body approximation
// Good for: Racing drones, compact designs
void setupCircularBody(Params& params, real_T diameter, real_T height) {
    // Approximate circle with equivalent rectangular box
    real_T side_length = diameter * 0.886f;  // Circle to square equivalent
    params.body_box.x() = side_length;
    params.body_box.y() = side_length;
    params.body_box.z() = height;
}
```

## Aerodynamic Properties

### Drag Coefficients

```cpp
// Linear drag coefficient (affects maximum speed)
params.linear_drag_coefficient = 0.325f;  // Default

// Angular drag coefficient (affects rotation damping)
params.angular_drag_coefficient = 0.325f;  // Usually same as linear

// Drag coefficient effects:
// Higher values = More air resistance, lower top speed, more damping
// Lower values = Less air resistance, higher top speed, less damping
```

### Drag Coefficient Selection

| Application | Linear Drag | Angular Drag | Reasoning |
|-------------|-------------|--------------|-----------|
| Racing | 0.15-0.25 | 0.15-0.25 | Streamlined design |
| Photography | 0.3-0.5 | 0.3-0.5 | Stability over speed |
| Research | 0.325 | 0.325 | Default/balanced |
| Heavy-lift | 0.4-0.6 | 0.4-0.6 | Large, less streamlined |

### Drag Calculation

```cpp
// Linear drag force calculation
Vector3r drag_force = -params.linear_drag_coefficient * 
                     air_density * velocity.norm() * velocity;

// Angular drag torque calculation  
Vector3r drag_torque = -params.angular_drag_coefficient * 
                      air_density * angular_velocity.norm() * angular_velocity;
```

## Collision Properties

### Restitution (Bounce Factor)

```cpp
// Restitution coefficient (0 = no bounce, 1 = perfect bounce)
params.restitution = 0.55f;  // Default

// Restitution effects:
// 0.0f  = Completely inelastic collision (no bounce)
// 0.5f  = Moderate bounce (realistic for most materials)
// 1.0f  = Perfectly elastic collision (unrealistic)
```

### Friction Coefficient

```cpp
// Friction coefficient for surface interactions
params.friction = 0.5f;  // Default

// Material-specific friction values:
// Concrete: 0.6-0.8
// Grass: 0.3-0.5
// Wood: 0.4-0.6
// Metal: 0.2-0.4
```

## Configuration Examples

### 1. Racing Drone Configuration

```cpp
void setupRacingDrone(Params& params) {
    // Lightweight and responsive
    params.mass = 0.8f;
    
    // Compact body
    params.body_box.x() = 0.12f;
    params.body_box.y() = 0.10f;
    params.body_box.z() = 0.03f;
    
    // Low drag for high speed
    params.linear_drag_coefficient = 0.2f;
    params.angular_drag_coefficient = 0.2f;
    
    // Hard impacts expected
    params.restitution = 0.3f;
    params.friction = 0.6f;
}
```

### 2. Photography Drone Configuration

```cpp
void setupPhotographyDrone(Params& params) {
    // Stable and smooth
    params.mass = 2.0f;
    
    // Larger body for equipment
    params.body_box.x() = 0.25f;
    params.body_box.y() = 0.20f;
    params.body_box.z() = 0.08f;
    
    // Higher drag for stability
    params.linear_drag_coefficient = 0.45f;
    params.angular_drag_coefficient = 0.45f;
    
    // Gentle landings
    params.restitution = 0.4f;
    params.friction = 0.5f;
}
```

### 3. Research Drone Configuration

```cpp
void setupResearchDrone(Params& params) {
    // Balanced for algorithm testing
    params.mass = 1.2f;
    
    // Standard body size
    params.body_box.x() = 0.18f;
    params.body_box.y() = 0.12f;
    params.body_box.z() = 0.05f;
    
    // Default drag coefficients
    params.linear_drag_coefficient = 0.325f;
    params.angular_drag_coefficient = 0.325f;
    
    // Standard collision properties
    params.restitution = 0.55f;
    params.friction = 0.5f;
}
```

### 4. Heavy-Lift Drone Configuration

```cpp
void setupHeavyLiftDrone(Params& params) {
    // High payload capacity
    params.mass = 5.0f;
    
    // Large body for payload
    params.body_box.x() = 0.40f;
    params.body_box.y() = 0.30f;
    params.body_box.z() = 0.15f;
    
    // Higher drag due to size
    params.linear_drag_coefficient = 0.5f;
    params.angular_drag_coefficient = 0.5f;
    
    // Robust collision handling
    params.restitution = 0.6f;
    params.friction = 0.4f;
}
```

## Validation and Testing

### Mass Validation

```cpp
void validateMass(const Params& params) {
    // Check mass is within reasonable limits
    assert(params.mass > 0.1f && params.mass < 50.0f);
    
    // Check thrust-to-weight ratio
    real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
    real_T weight = params.mass * 9.81f;
    real_T twr = total_thrust / weight;
    
    // Should be able to hover with margin
    assert(twr > 1.2f);
    
    // Reasonable upper limit for safety
    assert(twr < 10.0f);
}
```

### Inertia Validation

```cpp
void validateInertia(const Params& params) {
    // Check inertia values are positive
    assert(params.inertia(0, 0) > 0.0f);
    assert(params.inertia(1, 1) > 0.0f);
    assert(params.inertia(2, 2) > 0.0f);
    
    // Check reasonable magnitude
    real_T max_inertia = params.mass * 0.5f;  // Upper bound estimate
    assert(params.inertia(0, 0) < max_inertia);
    assert(params.inertia(1, 1) < max_inertia);
    assert(params.inertia(2, 2) < max_inertia);
}
```

### Drag Validation

```cpp
void validateDrag(const Params& params) {
    // Check drag coefficients are reasonable
    assert(params.linear_drag_coefficient > 0.01f);
    assert(params.linear_drag_coefficient < 2.0f);
    assert(params.angular_drag_coefficient > 0.01f);
    assert(params.angular_drag_coefficient < 2.0f);
}
```

## Performance Implications

### Mass Effects

- **Low mass**: Higher acceleration, more responsive, less stable
- **High mass**: Lower acceleration, less responsive, more stable
- **Mass distribution**: Affects inertia and control characteristics

### Inertia Effects

- **Low inertia**: Quick response, potential oscillations
- **High inertia**: Smooth motion, slower response
- **Asymmetric inertia**: Affects coupled roll/pitch dynamics

### Drag Effects

- **Low drag**: Higher top speed, less damping
- **High drag**: Lower top speed, more damping, better stability

## Advanced Configurations

### Variable Mass Systems

```cpp
// Simulate payload drop or fuel consumption
void updateMass(real_T new_mass) {
    real_T mass_ratio = new_mass / params.mass;
    
    // Update mass
    params.mass = new_mass;
    
    // Scale inertia proportionally
    params.inertia *= mass_ratio;
    
    // Recalculate performance characteristics
    updatePerformanceMetrics();
}
```

### Environmental Adaptations

```cpp
// Adjust drag for different environments
void adaptToEnvironment(real_T air_density_ratio) {
    // Scale drag coefficients
    params.linear_drag_coefficient *= air_density_ratio;
    params.angular_drag_coefficient *= air_density_ratio;
    
    // Update rotor air density
    params.rotor_params.air_density *= air_density_ratio;
    params.rotor_params.calculateMaxThrust();
}
```

## Common Issues and Solutions

### 1. Unstable Flight

**Symptoms**: Oscillations, difficult to control
**Causes**: Low inertia, incorrect mass distribution
**Solutions**:
- Increase body dimensions
- Add mass to center of drone
- Increase drag coefficients

### 2. Sluggish Response

**Symptoms**: Slow to respond to controls
**Causes**: High inertia, high drag
**Solutions**:
- Reduce body size
- Optimize mass distribution
- Decrease drag coefficients

### 3. Unrealistic Bouncing

**Symptoms**: Drone bounces excessively on landing
**Causes**: High restitution coefficient
**Solutions**:
- Reduce restitution to 0.3-0.5
- Increase friction coefficient
- Adjust collision detection settings

## Next Steps

- Configure [settings.json integration](settings-integration.md) for parameter management
- Learn about [performance tuning](performance-tuning.md) for optimization
- Explore [custom drone creation](custom-drone-creation.md) workflows
- Review [troubleshooting guide](troubleshooting.md) for common issues