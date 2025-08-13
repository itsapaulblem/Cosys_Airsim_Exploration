# Custom Drone Creation

## Overview

This guide walks you through creating custom drone configurations in Cosys-AirSim. You'll learn how to extend the existing framework to create specialized drone types with custom parameters, frame geometries, and performance characteristics.

## Step-by-Step Guide

### Step 1: Create Custom Parameter Class

First, create a new header file for your custom drone parameters:

```cpp
// File: AirLib/include/vehicles/multirotor/firmwares/custom/CustomDroneParams.hpp

#ifndef msr_airlib_vehicles_CustomDrone_hpp
#define msr_airlib_vehicles_CustomDrone_hpp

#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"

namespace msr {
namespace airlib {

class CustomDroneParams : public MultiRotorParams {
public:
    CustomDroneParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, 
                     std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory) {
        connection_info_ = getConnectionInfo(vehicle_setting);
    }

    virtual ~CustomDroneParams() = default;

    virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override {
        unique_ptr<MultirotorApiBase> api(new MavLinkMultirotorApi());
        auto api_ptr = static_cast<MavLinkMultirotorApi*>(api.get());
        api_ptr->initialize(connection_info_, &getSensors(), true);
        return api;
    }

    virtual void setupParams() override {
        auto& params = getParams();
        
        // Call your custom frame setup
        setupFrameCustomDrone(params);
    }

protected:
    virtual const SensorFactory* getSensorFactory() const override {
        return sensor_factory_.get();
    }

private:
    void setupFrameCustomDrone(Params& params) {
        // Your custom configuration here
        configureCustomFrame(params);
    }

    void configureCustomFrame(Params& params) {
        // Step 2 implementation goes here
    }

    static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(
        const AirSimSettings::MavLinkVehicleSetting& vehicle_setting) {
        return vehicle_setting.connection_info;
    }

    AirSimSettings::MavLinkConnectionInfo connection_info_;
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

} // namespace airlib
} // namespace msr

#endif
```

### Step 2: Configure Custom Frame

Implement your custom frame configuration:

```cpp
void configureCustomFrame(Params& params) {
    // === ROTOR CONFIGURATION ===
    params.rotor_count = 6;  // Hexacopter configuration
    
    // Custom arm lengths (can be different for each rotor)
    std::vector<real_T> arm_lengths = {
        0.30f,  // Rotor 0: Front
        0.30f,  // Rotor 1: Back
        0.25f,  // Rotor 2: Front-left
        0.25f,  // Rotor 3: Back-right
        0.25f,  // Rotor 4: Front-right
        0.25f   // Rotor 5: Back-left
    };
    
    // === MASS CONFIGURATION ===
    params.mass = 2.5f;  // Total mass in kg
    
    // Component masses
    real_T motor_assembly_weight = 0.08f;  // Heavier motors for hex
    real_T battery_weight = 0.6f;          // Larger battery
    real_T payload_weight = 0.4f;          // Camera/gimbal
    real_T frame_weight = 0.8f;            // Carbon fiber frame
    real_T electronics_weight = 0.2f;      // FC, ESCs, wiring
    
    // Validate total mass
    real_T calculated_mass = params.rotor_count * motor_assembly_weight + 
                           battery_weight + payload_weight + 
                           frame_weight + electronics_weight;
    
    // Adjust if needed
    if (abs(params.mass - calculated_mass) > 0.1f) {
        params.mass = calculated_mass;
    }
    
    real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;
    
    // === ROTOR SPECIFICATIONS ===
    // High-performance rotor parameters
    params.rotor_params.C_T = 0.13f;           // Higher thrust coefficient
    params.rotor_params.C_P = 0.05f;           // Higher power coefficient
    params.rotor_params.max_rpm = 7500;        // Higher RPM capability
    params.rotor_params.propeller_diameter = 0.279f;  // 11-inch propellers
    params.rotor_params.air_density = 1.225f;  // Sea level density
    params.rotor_params.calculateMaxThrust();  // Recalculate thrust limits
    
    // === BODY DIMENSIONS ===
    // Larger body for payload
    params.body_box.x() = 0.25f;  // Length
    params.body_box.y() = 0.20f;  // Width
    params.body_box.z() = 0.08f;  // Height
    
    // Rotor vertical offset
    real_T rotor_z = 0.04f;  // 4cm below center of mass
    
    // === ROTOR POSITIONING ===
    // Use hexacopter pattern with custom arm lengths
    initializeRotorHexX(params.rotor_poses, params.rotor_count, 
                       arm_lengths.data(), rotor_z);
    
    // === AERODYNAMIC PROPERTIES ===
    // Optimized for stability
    params.linear_drag_coefficient = 0.4f;   // Higher drag for stability
    params.angular_drag_coefficient = 0.4f;  // Better damping
    
    // === COLLISION PROPERTIES ===
    params.restitution = 0.5f;  // Moderate bounce
    params.friction = 0.6f;     // Good grip on surfaces
    
    // === COMPUTE INERTIA ===
    computeInertiaMatrix(params.inertia, params.body_box, 
                        params.rotor_poses, box_mass, motor_assembly_weight);
}
```

### Step 3: Add to Factory

Update the `MultiRotorParamsFactory` to include your custom drone:

```cpp
// File: AirLib/include/vehicles/multirotor/MultiRotorParamsFactory.hpp

#include "vehicles/multirotor/firmwares/custom/CustomDroneParams.hpp"

// In createConfig method, add:
else if (vehicle_setting->vehicle_type == "CustomDrone") {
    config.reset(new CustomDroneParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), 
                                      sensor_factory));
}
```

### Step 4: Update Build System

Add your new files to the build system:

```cmake
# In cmake/AirLib/CMakeLists.txt, add:
${AIRSIM_ROOT}/AirLib/include/vehicles/multirotor/firmwares/custom/CustomDroneParams.hpp
```

### Step 5: Configure in Settings

Use your custom drone in `settings.json`:

```json
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "MyCustomDrone": {
      "VehicleType": "CustomDrone",
      "Model": "CustomHex",
      "X": 0, "Y": 0, "Z": -1,
      "Yaw": 0,
      "EnableCollisions": true
    }
  }
}
```

## Advanced Customization Examples

### 1. Asymmetric Octocopter

```cpp
void setupAsymmetricOcto(Params& params) {
    params.rotor_count = 8;
    params.mass = 4.0f;
    
    // Asymmetric arm configuration for specialized payload
    std::vector<real_T> arm_lengths = {
        0.35f,  // Front rotors - longer for stability
        0.35f,
        0.30f,  // Side rotors - medium length
        0.30f,
        0.25f,  // Back rotors - shorter for agility
        0.25f,
        0.30f,  // Remaining sides
        0.30f
    };
    
    // Custom rotor angles for asymmetric design
    std::vector<real_T> arm_angles = {
        0.0f,    // Front
        180.0f,  // Back
        45.0f,   // Front-right
        225.0f,  // Back-left
        315.0f,  // Front-left
        135.0f,  // Back-right
        90.0f,   // Right
        270.0f   // Left
    };
    
    // Alternating rotor directions
    std::vector<RotorTurningDirection> rotor_directions = {
        RotorTurningDirection::RotorTurningDirectionCW,
        RotorTurningDirection::RotorTurningDirectionCW,
        RotorTurningDirection::RotorTurningDirectionCCW,
        RotorTurningDirection::RotorTurningDirectionCCW,
        RotorTurningDirection::RotorTurningDirectionCCW,
        RotorTurningDirection::RotorTurningDirectionCCW,
        RotorTurningDirection::RotorTurningDirectionCW,
        RotorTurningDirection::RotorTurningDirectionCW
    };
    
    // Heavy-duty rotor specs
    params.rotor_params.C_T = 0.16f;
    params.rotor_params.C_P = 0.062f;
    params.rotor_params.max_rpm = 6000;
    params.rotor_params.propeller_diameter = 0.330f;  // 13-inch props
    params.rotor_params.calculateMaxThrust();
    
    // Large body for industrial payload
    params.body_box.x() = 0.40f;
    params.body_box.y() = 0.30f;
    params.body_box.z() = 0.12f;
    
    real_T rotor_z = 0.06f;
    
    // Use generic rotor positioning with custom parameters
    initializeRotors(params.rotor_poses, params.rotor_count,
                    arm_lengths.data(), arm_angles.data(), 
                    rotor_directions.data(), rotor_z);
    
    // Calculate inertia
    real_T motor_weight = 0.12f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box, 
                        params.rotor_poses, box_mass, motor_weight);
}
```

### 2. Variable Pitch Quadcopter

```cpp
void setupVariablePitchQuad(Params& params) {
    params.rotor_count = 4;
    params.mass = 1.8f;
    
    // Standard arm lengths
    std::vector<real_T> arm_lengths(4, 0.25f);
    
    // Variable pitch rotor parameters
    params.rotor_params.C_T = 0.15f;  // Higher for variable pitch
    params.rotor_params.C_P = 0.08f;  // Higher power requirement
    params.rotor_params.max_rpm = 4000;  // Lower RPM, higher torque
    params.rotor_params.propeller_diameter = 0.305f;  // 12-inch props
    params.rotor_params.calculateMaxThrust();
    
    // Reinforced body for variable pitch mechanism
    params.body_box.x() = 0.22f;
    params.body_box.y() = 0.16f;
    params.body_box.z() = 0.10f;  // Taller for mechanisms
    
    // Higher drag due to complexity
    params.linear_drag_coefficient = 0.45f;
    params.angular_drag_coefficient = 0.45f;
    
    real_T rotor_z = 0.08f;  // Higher mount for pitch mechanism
    
    initializeRotorQuadX(params.rotor_poses, params.rotor_count,
                        arm_lengths.data(), rotor_z);
    
    // Heavier motors due to variable pitch
    real_T motor_weight = 0.15f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

### 3. Coaxial Helicopter

```cpp
void setupCoaxialHeli(Params& params) {
    params.rotor_count = 2;  // Two counter-rotating rotors
    params.mass = 1.5f;
    
    // Large coaxial rotors
    params.rotor_params.C_T = 0.18f;
    params.rotor_params.C_P = 0.09f;
    params.rotor_params.max_rpm = 3000;  // Lower RPM for large rotors
    params.rotor_params.propeller_diameter = 0.60f;  // 24-inch rotors
    params.rotor_params.calculateMaxThrust();
    
    // Tall narrow body
    params.body_box.x() = 0.15f;
    params.body_box.y() = 0.12f;
    params.body_box.z() = 0.25f;  // Tall for coaxial setup
    
    // Position rotors coaxially
    params.rotor_poses.clear();
    Vector3r unit_z(0, 0, -1);
    
    // Upper rotor (counter-clockwise)
    params.rotor_poses.emplace_back(
        Vector3r(0, 0, -0.10f),  // 10cm above center
        unit_z,
        RotorTurningDirection::RotorTurningDirectionCCW
    );
    
    // Lower rotor (clockwise)
    params.rotor_poses.emplace_back(
        Vector3r(0, 0, 0.10f),   // 10cm below center
        unit_z,
        RotorTurningDirection::RotorTurningDirectionCW
    );
    
    // Higher drag due to rotor interaction
    params.linear_drag_coefficient = 0.6f;
    params.angular_drag_coefficient = 0.6f;
    
    // Calculate inertia
    real_T motor_weight = 0.2f;  // Heavy motors for large rotors
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

## Dynamic Parameter Adjustment

### Runtime Parameter Modification

```cpp
class DynamicCustomDroneParams : public CustomDroneParams {
public:
    // Add methods for runtime adjustment
    void adjustMass(real_T new_mass) {
        auto& params = getParams();
        real_T mass_ratio = new_mass / params.mass;
        
        params.mass = new_mass;
        params.inertia *= mass_ratio;  // Scale inertia
        
        // Notify simulation of parameter change
        notifyParameterChange();
    }
    
    void adjustDragCoefficient(real_T new_drag) {
        auto& params = getParams();
        params.linear_drag_coefficient = new_drag;
        params.angular_drag_coefficient = new_drag;
        
        notifyParameterChange();
    }
    
    void adjustRotorPerformance(real_T thrust_scale) {
        auto& params = getParams();
        params.rotor_params.C_T *= thrust_scale;
        params.rotor_params.calculateMaxThrust();
        
        notifyParameterChange();
    }
    
private:
    void notifyParameterChange() {
        // Trigger recalculation in simulation
        // Implementation depends on your needs
    }
};
```

### Adaptive Configuration

```cpp
void setupAdaptiveDrone(Params& params) {
    // Base configuration
    params.rotor_count = 4;
    params.mass = 1.2f;
    
    // Adaptive rotor parameters based on environment
    real_T altitude = getCurrentAltitude();
    real_T air_density = calculateAirDensity(altitude);
    
    // Adjust parameters for altitude
    params.rotor_params.air_density = air_density;
    params.rotor_params.C_T = 0.11f * (1.225f / air_density);  // Compensate for density
    params.rotor_params.max_rpm = 6500 * sqrt(1.225f / air_density);  // Compensate RPM
    params.rotor_params.calculateMaxThrust();
    
    // Adjust drag for air density
    params.linear_drag_coefficient = 0.325f * (air_density / 1.225f);
    params.angular_drag_coefficient = 0.325f * (air_density / 1.225f);
    
    // Standard frame setup
    std::vector<real_T> arm_lengths(4, 0.23f);
    params.body_box.x() = 0.18f;
    params.body_box.y() = 0.12f;
    params.body_box.z() = 0.05f;
    
    real_T rotor_z = 0.03f;
    initializeRotorQuadX(params.rotor_poses, params.rotor_count,
                        arm_lengths.data(), rotor_z);
    
    real_T motor_weight = 0.06f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

## Testing and Validation

### Custom Validation Functions

```cpp
class CustomDroneValidator {
public:
    static bool validateCustomDrone(const MultiRotorParams::Params& params) {
        // Custom validation logic
        return validateMass(params) && 
               validateRotorConfiguration(params) &&
               validatePerformance(params);
    }
    
private:
    static bool validateMass(const MultiRotorParams::Params& params) {
        // Check mass is reasonable
        if (params.mass < 0.5f || params.mass > 25.0f) {
            return false;
        }
        
        // Check thrust-to-weight ratio
        real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
        real_T twr = total_thrust / (params.mass * 9.81f);
        
        return twr > 1.2f && twr < 5.0f;
    }
    
    static bool validateRotorConfiguration(const MultiRotorParams::Params& params) {
        // Check rotor count is supported
        if (params.rotor_count < 3 || params.rotor_count > 12) {
            return false;
        }
        
        // Check rotor positions are valid
        for (const auto& rotor : params.rotor_poses) {
            if (rotor.position.norm() < 0.1f || rotor.position.norm() > 2.0f) {
                return false;
            }
        }
        
        return true;
    }
    
    static bool validatePerformance(const MultiRotorParams::Params& params) {
        // Check performance parameters are reasonable
        if (params.linear_drag_coefficient < 0.01f || 
            params.linear_drag_coefficient > 2.0f) {
            return false;
        }
        
        if (params.rotor_params.max_thrust < 1.0f || 
            params.rotor_params.max_thrust > 50.0f) {
            return false;
        }
        
        return true;
    }
};
```

### Integration Testing

```cpp
void testCustomDroneIntegration() {
    // Create test settings
    AirSimSettings::MavLinkVehicleSetting test_setting;
    test_setting.vehicle_type = "CustomDrone";
    test_setting.vehicle_name = "TestDrone";
    
    // Create sensor factory
    auto sensor_factory = std::make_shared<SensorFactory>();
    
    // Create drone parameters
    auto drone_params = std::make_unique<CustomDroneParams>(test_setting, sensor_factory);
    
    // Initialize
    drone_params->initialize(&test_setting);
    
    // Validate
    const auto& params = drone_params->getParams();
    assert(CustomDroneValidator::validateCustomDrone(params));
    
    // Test API creation
    auto api = drone_params->createMultirotorApi();
    assert(api != nullptr);
    
    // Test flight characteristics
    testFlightCharacteristics(params);
}
```

## Integration with Existing Systems

### Sensor Integration

```cpp
void setupCustomSensors(const AirSimSettings::VehicleSetting* vehicle_setting) {
    // Add custom sensors specific to your drone type
    AirSimSettings::SensorSettings custom_sensor;
    custom_sensor.sensor_type = SensorBase::SensorType::Distance;
    custom_sensor.sensor_name = "CustomRangeFinder";
    custom_sensor.enabled = true;
    
    // Add to vehicle settings
    const_cast<AirSimSettings::VehicleSetting*>(vehicle_setting)->sensors
        .insert({"CustomRangeFinder", custom_sensor});
}
```

### Flight Controller Integration

```cpp
void setupCustomFlightController(const AirSimSettings::MavLinkVehicleSetting& setting) {
    // Configure custom flight controller parameters
    // This could include custom PID gains, filter settings, etc.
    
    // Example: Custom PID gains for your drone type
    mavlink_param_set_t param_msg;
    param_msg.target_system = 1;
    param_msg.target_component = 1;
    
    // Set custom roll rate P gain
    strcpy(param_msg.param_id, "MC_ROLLRATE_P");
    param_msg.param_value = 0.15f;  // Custom value for your drone
    
    // Send to flight controller
    // Implementation depends on your MAVLink setup
}
```

## Best Practices

### 1. Parameter Documentation

```cpp
class WellDocumentedDroneParams : public MultiRotorParams {
    /**
     * Custom drone configuration for specialized aerial photography
     * 
     * Features:
     * - 6 rotors for redundancy
     * - Optimized for camera stability
     * - Extended flight time configuration
     * - Payload capacity: 1.5kg
     * 
     * Performance characteristics:
     * - Max speed: 15 m/s
     * - Hover time: 25 minutes
     * - Max altitude: 4000m
     * - Operating temperature: -10°C to +50°C
     */
    
    struct DroneSpecs {
        static constexpr real_T MAX_PAYLOAD = 1.5f;     // kg
        static constexpr real_T HOVER_TIME = 25.0f;     // minutes
        static constexpr real_T MAX_ALTITUDE = 4000.0f; // meters
        static constexpr real_T MIN_TEMP = -10.0f;      // Celsius
        static constexpr real_T MAX_TEMP = 50.0f;       // Celsius
    };
};
```

### 2. Modular Design

```cpp
class ModularDroneParams : public MultiRotorParams {
private:
    // Separate configuration modules
    void configureFrame(Params& params);
    void configureRotors(Params& params);
    void configureAerodynamics(Params& params);
    void configurePayload(Params& params);
    
    // Module-specific validation
    bool validateFrameConfig(const Params& params);
    bool validateRotorConfig(const Params& params);
    bool validateAerodynamicsConfig(const Params& params);
    
public:
    virtual void setupParams() override {
        auto& params = getParams();
        
        configureFrame(params);
        configureRotors(params);
        configureAerodynamics(params);
        configurePayload(params);
        
        // Validate each module
        assert(validateFrameConfig(params));
        assert(validateRotorConfig(params));
        assert(validateAerodynamicsConfig(params));
    }
};
```

### 3. Configuration Templates

```cpp
namespace DroneTemplates {
    // Racing drone template
    void applyRacingTemplate(MultiRotorParams::Params& params) {
        params.mass = 0.8f;
        params.linear_drag_coefficient = 0.2f;
        params.angular_drag_coefficient = 0.2f;
        // ... racing-specific parameters
    }
    
    // Photography drone template
    void applyPhotographyTemplate(MultiRotorParams::Params& params) {
        params.mass = 2.0f;
        params.linear_drag_coefficient = 0.45f;
        params.angular_drag_coefficient = 0.45f;
        // ... photography-specific parameters
    }
    
    // Industrial drone template
    void applyIndustrialTemplate(MultiRotorParams::Params& params) {
        params.mass = 5.0f;
        params.linear_drag_coefficient = 0.6f;
        params.angular_drag_coefficient = 0.6f;
        // ... industrial-specific parameters
    }
}
```

## Common Pitfalls and Solutions

### 1. Incorrect Mass Distribution

**Problem**: Drone flips or is unstable
**Solution**: Ensure proper center of mass calculation

```cpp
// Calculate center of mass
Vector3r calculateCenterOfMass(const vector<RotorPose>& rotors, 
                              real_T motor_weight, real_T body_mass) {
    Vector3r com = Vector3r::Zero();
    real_T total_mass = body_mass;
    
    // Add motor contributions
    for (const auto& rotor : rotors) {
        com += rotor.position * motor_weight;
        total_mass += motor_weight;
    }
    
    com /= total_mass;
    return com;
}
```

### 2. Inadequate Thrust

**Problem**: Drone can't take off or maintain altitude
**Solution**: Validate thrust-to-weight ratio

```cpp
void validateThrustToWeight(const MultiRotorParams::Params& params) {
    real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
    real_T weight = params.mass * 9.81f;
    real_T twr = total_thrust / weight;
    
    if (twr < 1.5f) {
        throw std::runtime_error("Insufficient thrust for stable flight");
    }
    
    if (twr > 4.0f) {
        // Warning: Very high thrust-to-weight ratio
        // Consider reducing rotor power or increasing mass
    }
}
```

### 3. Unrealistic Parameters

**Problem**: Simulation behaves unrealistically
**Solution**: Use real-world data for validation

```cpp
void validateAgainstRealWorld(const MultiRotorParams::Params& params) {
    // Compare with similar real-world drones
    struct RealWorldBenchmarks {
        real_T dji_phantom_mass = 1.38f;
        real_T dji_phantom_thrust = 6.0f;  // N per rotor
        real_T racing_drone_mass = 0.7f;
        real_T racing_drone_thrust = 4.0f;
    };
    
    // Validate against benchmarks
    // Implementation depends on your specific drone type
}
```

## Next Steps

After creating your custom drone:

1. **Test thoroughly** in simulation
2. **Validate against real-world data** if available
3. **Document your configuration** for future reference
4. **Share with the community** if it's a common use case
5. **Consider contributing** to the main codebase

## Additional Resources

- [Performance Tuning Guide](performance-tuning.md) for optimization
- [Troubleshooting Guide](troubleshooting.md) for common issues
- [Settings Integration](settings-integration.md) for configuration management
- [Frame Types](frame-types.md) for understanding existing configurations
- [Rotor Configuration](rotor-configuration.md) for detailed rotor setup