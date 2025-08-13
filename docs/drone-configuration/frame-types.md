# Drone Frame Types

## Pre-built Frame Configurations

Cosys-AirSim includes several pre-configured frame types that represent common drone configurations. These serve as starting points for custom configurations.

### Quadcopter Frames (4 rotors)

#### 1. Generic Quad (`setupFrameGenericQuad()`)
**Use case:** Basic research and development, algorithm testing

```cpp
// Configuration details
rotor_count: 4
mass: 1.0 kg
arm_length: 0.2275 m (all arms equal)
body_box: 0.180 × 0.11 × 0.040 m
motor_weight: 0.055 kg each
rotor_z_offset: 0.025 m
```

**Mass limits:** 0.8 kg - 1.6 kg (with default rotor parameters)

#### 2. DJI Flamewheel (`setupFrameFlamewheel()`)
**Use case:** Realistic consumer drone simulation

```cpp
// Configuration details  
rotor_count: 4
mass: 1.635 kg
arm_length: 0.225 m (all arms equal)
body_box: 0.16 × 0.10 × 0.14 m
motor_weight: 0.052 kg each
rotor_z_offset: 0.15 m

// Custom rotor parameters
C_T: 0.11 (higher thrust)
C_P: 0.047 (higher torque)
max_rpm: 9500
linear_drag_coefficient: 1.3 (4x default for realism)
```

#### 3. Flamewheel FLA (`setupFrameFlamewheelFLA()`)
**Use case:** Heavy-lift applications, cargo drones

```cpp
// Configuration details
rotor_count: 4
mass: 2.25 kg
arm_length: 0.225 m (all arms equal)
body_box: 0.16 × 0.10 × 0.14 m
motor_weight: 0.1 kg each
rotor_z_offset: 0.15 m

// High-performance rotor parameters
C_T: 0.2 (very high thrust)
C_P: 0.1 (very high torque)
max_rpm: 9324
```

#### 4. Team Blacksheep Discovery (`setupFrameBlacksheep()`)
**Use case:** FPV racing, asymmetric designs

```cpp
// Configuration details
rotor_count: 4
mass: 2.0 kg
motor_weight: 0.052 kg each
body_box: 0.20 × 0.12 × 0.04 m
rotor_z_offset: 0.025 m

// Asymmetric arm configuration
arm_lengths: [0.22, 0.255, 0.22, 0.255] m
arm_angles: [-55°, 125°, 55°, -125°] from forward
```

### Hexacopter Frames (6 rotors)

#### Generic Hex (`setupFrameGenericHex()`)
**Use case:** Increased redundancy, higher payload capacity

```cpp
// Configuration details
rotor_count: 6
mass: 1.0 kg (can support up to 2.5 kg)
arm_length: 0.2275 m (all arms equal)
body_box: 0.180 × 0.11 × 0.040 m
motor_weight: 0.055 kg each
rotor_z_offset: 0.025 m

// Rotor arrangement (angles from forward)
// 0: 0°, 1: 180°, 2: 30°, 3: 210°, 4: 60°, 5: 240°
```

**Advantages:**
- Fault tolerance (can fly with one rotor failure)
- Higher payload capacity
- Better stability in windy conditions

### Octocopter Frames (8 rotors)

#### Generic Octo (`setupFrameGenericOcto()`)
**Use case:** Professional cinematography, heavy-lift operations

```cpp
// Configuration details
rotor_count: 8
mass: 1.0 kg (can support up to 3.2 kg)
arm_length: 0.2275 m (all arms equal)
body_box: 0.180 × 0.11 × 0.040 m
motor_weight: 0.055 kg each
rotor_z_offset: 0.025 m

// Complex rotor arrangement for optimal thrust distribution
// Mix of 22.5° and 67.5° angles for balanced performance
```

**Advantages:**
- Maximum redundancy (can fly with multiple rotor failures)
- Highest payload capacity
- Best stability and control authority

## Rotor Arrangement Patterns

### Quadcopter X Pattern
```
Standard X configuration with 45° arm angles:

     Forward (X)
        |
   (2)  |  (0)    Motor 0: Front-right, CCW
        |          Motor 1: Back-left, CCW  
 -------|------- Right (Y)
        |          Motor 2: Front-left, CW
   (1)  |  (3)    Motor 3: Back-right, CW
        |
```

### Hexacopter Pattern
```
6-rotor configuration with 60° spacing:

     Forward (X)
        |
    (2) | (4)     Even motors (0,2,4): CW
     \  |  /      Odd motors (1,3,5): CCW
(1)-----|-----(0) Right (Y)
     /  |  \
    (5) | (3)
        |
```

### Octocopter Pattern
```
8-rotor configuration with mixed angles:

     Forward (X)
        |
  (4)   |   (0)   Inner ring: 22.5° intervals
     \  |  /      Outer ring: 67.5° intervals
(6)-------|-----(2) Right (Y)
     /  |  \      Alternating CW/CCW pattern
  (5)   |   (7)
        |
     (1) | (3)
```

## Creating Custom Frame Types

### Step 1: Define Basic Parameters

```cpp
void setupFrameCustom(Params& params) {
    // Set rotor count
    params.rotor_count = 4;  // or 6, 8
    
    // Define arm lengths (per rotor)
    std::vector<real_T> arm_lengths = {0.25f, 0.25f, 0.25f, 0.25f};
    
    // Set total mass
    params.mass = 1.5f;
    
    // Motor specifications
    real_T motor_assembly_weight = 0.06f;
    real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;
}
```

### Step 2: Configure Rotor Parameters

```cpp
// Custom rotor specifications
params.rotor_params.C_T = 0.12f;           // Thrust coefficient
params.rotor_params.C_P = 0.05f;           // Torque coefficient
params.rotor_params.max_rpm = 8000;        // Maximum RPM
params.rotor_params.propeller_diameter = 0.254f;  // 10-inch props
params.rotor_params.calculateMaxThrust();   // Recalculate limits
```

### Step 3: Define Body Geometry

```cpp
// Body dimensions (affects inertia calculation)
params.body_box.x() = 0.20f;  // Length
params.body_box.y() = 0.15f;  // Width
params.body_box.z() = 0.06f;  // Height

// Rotor vertical offset from center of mass
real_T rotor_z = 0.03f;
```

### Step 4: Position Rotors

For standard patterns:
```cpp
// Use built-in methods
initializeRotorQuadX(params.rotor_poses, params.rotor_count, 
                     arm_lengths.data(), rotor_z);
```

For custom patterns:
```cpp
// Define custom angles and directions
real_T arm_angles[] = {30, 150, 210, 330};  // Degrees from forward
RotorTurningDirection directions[] = {
    RotorTurningDirection::RotorTurningDirectionCCW,
    RotorTurningDirection::RotorTurningDirectionCCW,
    RotorTurningDirection::RotorTurningDirectionCW,
    RotorTurningDirection::RotorTurningDirectionCW
};

initializeRotors(params.rotor_poses, params.rotor_count,
                 arm_lengths.data(), arm_angles, directions, rotor_z);
```

### Step 5: Calculate Inertia

```cpp
// Automatically compute inertia matrix
computeInertiaMatrix(params.inertia, params.body_box, 
                     params.rotor_poses, box_mass, motor_assembly_weight);
```

## Frame Selection Guidelines

### Research Applications
- **Generic Quad**: Algorithm development, basic testing
- **Generic Hex**: Fault tolerance research, multi-rotor studies

### Commercial Simulation
- **Flamewheel**: Consumer drones, photography
- **Flamewheel FLA**: Professional cinematography, surveying

### Racing/Performance
- **Blacksheep**: FPV racing, agile maneuvers
- **Custom lightweight**: Optimized for speed and responsiveness

### Industrial Applications
- **Generic Octo**: Heavy-lift, critical missions
- **Custom high-capacity**: Specialized payloads, long endurance

## Performance Characteristics

| Frame Type | Agility | Stability | Payload | Redundancy |
|------------|---------|-----------|---------|------------|
| Quad       | High    | Medium    | Low     | None       |
| Hex        | Medium  | High      | Medium  | Good       |
| Octo       | Low     | Very High | High    | Excellent  |

## Mass and Thrust Calculations

### Thrust Requirements
For stable flight, total thrust must exceed weight by at least 20%:

```cpp
// Minimum thrust requirement
float min_thrust = mass * 9.81f * 1.2f;  // 20% margin

// Check if rotors can provide sufficient thrust
float max_available_thrust = rotor_count * rotor_params.max_thrust;
if (max_available_thrust < min_thrust) {
    // Need more powerful rotors or reduce mass
}
```

### Hover Throttle Estimation
```cpp
// Approximate hover throttle percentage
float hover_throttle = (mass * 9.81f) / max_available_thrust;
// Should be around 40-60% for good control authority
```

## Testing and Validation

### 1. Mass Balance Check
```cpp
// Verify mass is within safe operating range
float min_mass = 0.5f * rotor_count * rotor_params.max_thrust / 9.81f;
float max_mass = 0.8f * rotor_count * rotor_params.max_thrust / 9.81f;
assert(mass >= min_mass && mass <= max_mass);
```

### 2. Geometric Validation
```cpp
// Check for rotor collision
for (int i = 0; i < rotor_count; i++) {
    for (int j = i + 1; j < rotor_count; j++) {
        float distance = (rotor_poses[i].position - rotor_poses[j].position).norm();
        assert(distance > rotor_params.propeller_diameter);
    }
}
```

### 3. Performance Testing
- Test takeoff capability
- Verify stability in hover
- Check response to control inputs
- Validate flight envelope limits

## Next Steps

- Configure [rotor parameters](rotor-configuration.md) for your specific motors
- Understand [physical properties](physical-properties.md) tuning
- Set up [settings.json integration](settings-integration.md)
- Create [custom drone types](custom-drone-creation.md)