# Troubleshooting Guide

## Overview

This guide helps you diagnose and resolve common issues with drone configuration in Cosys-AirSim. Issues are organized by symptoms with systematic debugging approaches.

## Quick Diagnostic Checklist

```cpp
// Run this basic validation before deeper troubleshooting
bool quickDiagnostic(const MultiRotorParams::Params& params) {
    // 1. Check mass is positive and reasonable
    if (params.mass <= 0.0f || params.mass > 50.0f) {
        std::cout << "ERROR: Invalid mass: " << params.mass << std::endl;
        return false;
    }
    
    // 2. Check thrust-to-weight ratio
    real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
    real_T twr = total_thrust / (params.mass * 9.81f);
    if (twr < 1.1f) {
        std::cout << "ERROR: Insufficient thrust-to-weight ratio: " << twr << std::endl;
        return false;
    }
    
    // 3. Check rotor count and positioning
    if (params.rotor_count < 3 || params.rotor_count != params.rotor_poses.size()) {
        std::cout << "ERROR: Invalid rotor configuration" << std::endl;
        return false;
    }
    
    // 4. Check inertia matrix
    if (params.inertia(0,0) <= 0 || params.inertia(1,1) <= 0 || params.inertia(2,2) <= 0) {
        std::cout << "ERROR: Invalid inertia matrix" << std::endl;
        return false;
    }
    
    return true;
}
```

## Flight Behavior Issues

### 1. Drone Won't Take Off

**Symptoms**: Drone sits on ground, motors spin but no lift

**Possible Causes**:
- Insufficient thrust-to-weight ratio
- Incorrect rotor direction
- Mass too high for available thrust
- Rotor parameters misconfigured

**Debugging Steps**:

```cpp
void diagnoseTakeoffFailure(const MultiRotorParams::Params& params) {
    // Check thrust-to-weight ratio
    real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
    real_T weight = params.mass * 9.81f;
    real_T twr = total_thrust / weight;
    
    std::cout << "Thrust-to-weight ratio: " << twr << std::endl;
    
    if (twr < 1.1f) {
        std::cout << "PROBLEM: Insufficient thrust" << std::endl;
        std::cout << "Solutions:" << std::endl;
        std::cout << "- Increase rotor C_T coefficient" << std::endl;
        std::cout << "- Increase max RPM" << std::endl;
        std::cout << "- Use larger propellers" << std::endl;
        std::cout << "- Reduce mass" << std::endl;
    }
    
    // Check rotor direction balance
    int cw_count = 0, ccw_count = 0;
    for (const auto& rotor : params.rotor_poses) {
        if (rotor.direction == RotorTurningDirection::RotorTurningDirectionCW) {
            cw_count++;
        } else {
            ccw_count++;
        }
    }
    
    if (abs(cw_count - ccw_count) > 1) {
        std::cout << "WARNING: Unbalanced rotor directions" << std::endl;
        std::cout << "CW rotors: " << cw_count << ", CCW rotors: " << ccw_count << std::endl;
    }
    
    // Check individual rotor thrust
    std::cout << "Max thrust per rotor: " << params.rotor_params.max_thrust << " N" << std::endl;
    std::cout << "Required thrust per rotor: " << weight / params.rotor_count << " N" << std::endl;
}
```

**Solutions**:
```cpp
void fixTakeoffIssue(MultiRotorParams::Params& params) {
    // Solution 1: Increase thrust coefficient
    params.rotor_params.C_T *= 1.3f;
    params.rotor_params.calculateMaxThrust();
    
    // Solution 2: Increase RPM capability
    params.rotor_params.max_rpm *= 1.2f;
    params.rotor_params.calculateMaxThrust();
    
    // Solution 3: Use larger propellers
    params.rotor_params.propeller_diameter *= 1.1f;
    params.rotor_params.calculateMaxThrust();
    
    // Solution 4: Reduce mass (last resort)
    if (params.rotor_count * params.rotor_params.max_thrust < params.mass * 9.81f * 1.2f) {
        params.mass *= 0.8f;
        // Recalculate inertia
        real_T motor_weight = 0.055f;
        real_T box_mass = params.mass - params.rotor_count * motor_weight;
        computeInertiaMatrix(params.inertia, params.body_box,
                            params.rotor_poses, box_mass, motor_weight);
    }
}
```

### 2. Drone Flips on Takeoff

**Symptoms**: Drone immediately flips or tumbles when trying to take off

**Possible Causes**:
- Incorrect rotor turning directions
- Asymmetric thrust distribution
- Wrong rotor positioning
- Center of mass offset

**Debugging Steps**:

```cpp
void diagnoseFlipOnTakeoff(const MultiRotorParams::Params& params) {
    std::cout << "Checking rotor configuration..." << std::endl;
    
    // Check rotor turning directions
    for (size_t i = 0; i < params.rotor_poses.size(); i++) {
        std::cout << "Rotor " << i << ": position (" 
                  << params.rotor_poses[i].position.x() << ", "
                  << params.rotor_poses[i].position.y() << ", "
                  << params.rotor_poses[i].position.z() << "), direction: "
                  << (params.rotor_poses[i].direction == RotorTurningDirection::RotorTurningDirectionCW ? "CW" : "CCW")
                  << std::endl;
    }
    
    // Check for symmetric rotor placement
    Vector3r center_of_rotors(0, 0, 0);
    for (const auto& rotor : params.rotor_poses) {
        center_of_rotors += rotor.position;
    }
    center_of_rotors /= params.rotor_count;
    
    std::cout << "Rotor center: (" << center_of_rotors.x() << ", " 
              << center_of_rotors.y() << ", " << center_of_rotors.z() << ")" << std::endl;
    
    if (center_of_rotors.norm() > 0.05f) {
        std::cout << "WARNING: Rotor placement is asymmetric" << std::endl;
    }
    
    // Check for proper moment balance
    Vector3r net_torque(0, 0, 0);
    for (const auto& rotor : params.rotor_poses) {
        real_T direction_multiplier = static_cast<real_T>(rotor.direction);
        net_torque.z() += direction_multiplier;
    }
    
    if (abs(net_torque.z()) > 0.1f) {
        std::cout << "ERROR: Unbalanced yaw torque: " << net_torque.z() << std::endl;
    }
}
```

**Solutions**:
```cpp
void fixFlipOnTakeoff(MultiRotorParams::Params& params) {
    // For quadcopter, ensure proper X configuration
    if (params.rotor_count == 4) {
        // Recreate proper quad X configuration
        std::vector<real_T> arm_lengths(4, 0.25f);
        real_T rotor_z = 0.025f;
        
        initializeRotorQuadX(params.rotor_poses, params.rotor_count,
                            arm_lengths.data(), rotor_z);
    }
    
    // For hexacopter, ensure proper hex configuration
    else if (params.rotor_count == 6) {
        std::vector<real_T> arm_lengths(6, 0.25f);
        real_T rotor_z = 0.025f;
        
        initializeRotorHexX(params.rotor_poses, params.rotor_count,
                           arm_lengths.data(), rotor_z);
    }
    
    // Recalculate inertia with corrected rotor positions
    real_T motor_weight = 0.055f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

### 3. Unstable Hover (Oscillations)

**Symptoms**: Drone hovers but oscillates continuously

**Possible Causes**:
- Insufficient damping
- Too high control gains
- Resonance frequencies
- Incorrect inertia calculation

**Debugging Steps**:

```cpp
void diagnoseHoverOscillations(const MultiRotorParams::Params& params) {
    // Check damping ratio
    real_T damping_ratio = params.angular_drag_coefficient / sqrt(params.inertia(2, 2));
    std::cout << "Damping ratio: " << damping_ratio << std::endl;
    
    if (damping_ratio < 0.1f) {
        std::cout << "PROBLEM: Insufficient damping" << std::endl;
    }
    
    // Check inertia ratios
    real_T inertia_ratio_xy = params.inertia(0, 0) / params.inertia(1, 1);
    real_T inertia_ratio_z = params.inertia(2, 2) / params.inertia(0, 0);
    
    std::cout << "Inertia ratios - X/Y: " << inertia_ratio_xy 
              << ", Z/X: " << inertia_ratio_z << std::endl;
    
    if (abs(inertia_ratio_xy - 1.0f) > 0.2f) {
        std::cout << "WARNING: Asymmetric X-Y inertia" << std::endl;
    }
    
    // Check natural frequency
    real_T control_authority = params.rotor_count * params.rotor_params.max_thrust / params.mass;
    real_T natural_freq = sqrt(control_authority) / (2 * M_PI);
    std::cout << "Estimated natural frequency: " << natural_freq << " Hz" << std::endl;
    
    if (natural_freq < 1.0f) {
        std::cout << "WARNING: Very low natural frequency" << std::endl;
    } else if (natural_freq > 10.0f) {
        std::cout << "WARNING: Very high natural frequency" << std::endl;
    }
}
```

**Solutions**:
```cpp
void fixHoverOscillations(MultiRotorParams::Params& params) {
    // Increase damping
    params.angular_drag_coefficient *= 2.0f;
    params.linear_drag_coefficient *= 1.5f;
    
    // Increase inertia for stability
    params.body_box.x() *= 1.1f;
    params.body_box.y() *= 1.1f;
    params.body_box.z() *= 1.1f;
    
    // Add mass for stability
    params.mass *= 1.1f;
    
    // Recalculate inertia
    real_T motor_weight = 0.055f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

### 4. Sluggish Response

**Symptoms**: Drone responds slowly to control inputs

**Possible Causes**:
- High inertia
- Low thrust-to-weight ratio
- Excessive damping
- Large body size

**Debugging Steps**:

```cpp
void diagnoseSluggiResponse(const MultiRotorParams::Params& params) {
    // Check thrust-to-weight ratio
    real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
    real_T twr = total_thrust / (params.mass * 9.81f);
    std::cout << "Thrust-to-weight ratio: " << twr << std::endl;
    
    if (twr < 2.0f) {
        std::cout << "Low thrust-to-weight ratio affects responsiveness" << std::endl;
    }
    
    // Check inertia values
    std::cout << "Inertia values:" << std::endl;
    std::cout << "Roll (Ixx): " << params.inertia(0, 0) << std::endl;
    std::cout << "Pitch (Iyy): " << params.inertia(1, 1) << std::endl;
    std::cout << "Yaw (Izz): " << params.inertia(2, 2) << std::endl;
    
    // Check damping
    std::cout << "Damping coefficients:" << std::endl;
    std::cout << "Linear: " << params.linear_drag_coefficient << std::endl;
    std::cout << "Angular: " << params.angular_drag_coefficient << std::endl;
    
    if (params.angular_drag_coefficient > 0.8f) {
        std::cout << "Excessive damping may cause sluggish response" << std::endl;
    }
}
```

**Solutions**:
```cpp
void fixSluggiResponse(MultiRotorParams::Params& params) {
    // Increase thrust-to-weight ratio
    params.rotor_params.C_T *= 1.2f;
    params.rotor_params.calculateMaxThrust();
    
    // Reduce inertia by decreasing body size
    params.body_box.x() *= 0.9f;
    params.body_box.y() *= 0.9f;
    params.body_box.z() *= 0.9f;
    
    // Reduce damping
    params.angular_drag_coefficient *= 0.7f;
    params.linear_drag_coefficient *= 0.8f;
    
    // Recalculate inertia with smaller body
    real_T motor_weight = 0.055f;
    real_T box_mass = params.mass - params.rotor_count * motor_weight;
    computeInertiaMatrix(params.inertia, params.body_box,
                        params.rotor_poses, box_mass, motor_weight);
}
```

## Configuration Issues

### 5. Settings Not Loading

**Symptoms**: Custom drone type not recognized, using default configuration

**Possible Causes**:
- Vehicle type not registered in factory
- JSON syntax errors
- Missing model definition
- Incorrect file paths

**Debugging Steps**:

```cpp
void debugSettingsLoading() {
    std::cout << "Checking settings loading..." << std::endl;
    
    // Check if settings file exists
    std::string settings_path = "~/Documents/AirSim/settings.json";
    std::ifstream file(settings_path);
    if (!file.good()) {
        std::cout << "ERROR: Settings file not found at " << settings_path << std::endl;
        return;
    }
    
    // Check JSON syntax
    try {
        nlohmann::json settings;
        file >> settings;
        std::cout << "Settings JSON is valid" << std::endl;
        
        // Check for vehicles section
        if (settings.contains("Vehicles")) {
            std::cout << "Found " << settings["Vehicles"].size() << " vehicle(s)" << std::endl;
            
            for (auto& [name, config] : settings["Vehicles"].items()) {
                std::cout << "Vehicle: " << name << std::endl;
                
                if (config.contains("VehicleType")) {
                    std::cout << "  Type: " << config["VehicleType"] << std::endl;
                } else {
                    std::cout << "  ERROR: Missing VehicleType" << std::endl;
                }
                
                if (config.contains("Model")) {
                    std::cout << "  Model: " << config["Model"] << std::endl;
                }
            }
        } else {
            std::cout << "ERROR: No Vehicles section found" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "ERROR: JSON parsing failed: " << e.what() << std::endl;
    }
}
```

**Solutions**:
```json
// Correct settings.json format
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4",
      "Model": "Generic",
      "X": 0, "Y": 0, "Z": -1,
      "Yaw": 0,
      "EnableCollisions": true,
      "AllowAPIAlways": true
    }
  }
}
```

### 6. PX4 Connection Issues

**Symptoms**: PX4 can't connect to AirSim, timeout errors

**Possible Causes**:
- Incorrect MAVLink ports
- Firewall blocking connections
- PX4 not running
- IP address conflicts

**Debugging Steps**:

```cpp
void debugPX4Connection() {
    std::cout << "Checking PX4 connection..." << std::endl;
    
    // Check if PX4 process is running
    int result = system("ps aux | grep px4");
    if (result != 0) {
        std::cout << "WARNING: PX4 process not found" << std::endl;
    }
    
    // Check port availability
    std::vector<int> ports = {4560, 14540, 14580, 14550};
    for (int port : ports) {
        std::cout << "Checking port " << port << ": ";
        
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            std::cout << "OPEN" << std::endl;
        } else {
            std::cout << "CLOSED" << std::endl;
        }
        close(sock);
    }
}
```

**Solutions**:
```json
// Correct PX4 settings
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
      "SitlIp": "127.0.0.1",
      "SitlPort": 14556,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14560
    }
  }
}
```

### 7. Compilation Errors

**Symptoms**: Build fails with custom drone configuration

**Possible Causes**:
- Missing header includes
- Incorrect namespace usage
- Linking errors
- Missing CMake entries

**Common Solutions**:

```cpp
// Ensure proper includes
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

// Use correct namespace
namespace msr {
namespace airlib {
    // Your code here
}
}

// Ensure factory registration
// In MultiRotorParamsFactory.hpp
#include "vehicles/multirotor/firmwares/custom/YourCustomParams.hpp"

// In createConfig method
else if (vehicle_setting->vehicle_type == "YourCustomType") {
    config.reset(new YourCustomParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), 
                                     sensor_factory));
}
```

## Performance Issues

### 8. Simulation Running Slowly

**Symptoms**: Low frame rate, stuttering, delays

**Possible Causes**:
- Too many vehicles
- Complex sensor configurations
- High-resolution cameras
- Excessive logging

**Debugging Steps**:

```cpp
void diagnosePerformance() {
    std::cout << "Performance diagnostics:" << std::endl;
    
    // Check CPU usage
    system("top -n 1 | grep AirSim");
    
    // Check memory usage
    system("ps aux | grep AirSim | awk '{print $4, $5, $6}'");
    
    // Check GPU usage (if applicable)
    system("nvidia-smi");
}
```

**Solutions**:
```cpp
void optimizePerformance() {
    // Reduce vehicle count
    // Use simpler sensor configurations
    // Lower camera resolution
    // Disable unnecessary logging
    // Use "NoDisplay" mode for batch processing
}
```

### 9. Memory Leaks

**Symptoms**: Gradually increasing memory usage over time

**Possible Causes**:
- Sensor data not being freed
- Continuous data logging
- Circular references in custom code

**Debugging Steps**:

```cpp
void debugMemoryUsage() {
    // Monitor memory usage over time
    std::cout << "Memory usage monitoring..." << std::endl;
    
    // Use valgrind for detailed analysis
    // valgrind --tool=memcheck --leak-check=full ./AirSim
    
    // Check for common leak patterns
    // - Unmatched new/delete
    // - Circular shared_ptr references
    // - Static containers that grow indefinitely
}
```

## Sensor Configuration Issues

### 10. Sensor Data Not Available

**Symptoms**: Sensor APIs return empty or invalid data

**Possible Causes**:
- Sensor not enabled in settings
- Incorrect sensor type
- API permissions
- Sensor positioning issues

**Debugging Steps**:

```cpp
void debugSensorConfiguration() {
    std::cout << "Checking sensor configuration..." << std::endl;
    
    // Check if sensors are enabled
    // Check sensor positions
    // Verify API access
    // Test sensor data retrieval
}
```

**Solutions**:
```json
// Correct sensor configuration
{
  "Vehicles": {
    "Drone1": {
      "VehicleType": "PX4",
      "Model": "Generic",
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
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled": true,
          "DrawDebugPoints": false
        }
      }
    }
  }
}
```

## Systematic Debugging Approach

### Debug Information Collection

```cpp
class DebugInfoCollector {
public:
    void collectAllDebugInfo(const MultiRotorParams::Params& params) {
        std::cout << "=== DRONE CONFIGURATION DEBUG INFO ===" << std::endl;
        
        printBasicInfo(params);
        printRotorInfo(params);
        printPhysicsInfo(params);
        printPerformanceInfo(params);
        
        std::cout << "=== END DEBUG INFO ===" << std::endl;
    }
    
private:
    void printBasicInfo(const MultiRotorParams::Params& params) {
        std::cout << "Basic Configuration:" << std::endl;
        std::cout << "  Mass: " << params.mass << " kg" << std::endl;
        std::cout << "  Rotor count: " << params.rotor_count << std::endl;
        std::cout << "  Body dimensions: " << params.body_box.x() << "x" 
                  << params.body_box.y() << "x" << params.body_box.z() << " m" << std::endl;
    }
    
    void printRotorInfo(const MultiRotorParams::Params& params) {
        std::cout << "Rotor Configuration:" << std::endl;
        std::cout << "  C_T: " << params.rotor_params.C_T << std::endl;
        std::cout << "  C_P: " << params.rotor_params.C_P << std::endl;
        std::cout << "  Max RPM: " << params.rotor_params.max_rpm << std::endl;
        std::cout << "  Propeller diameter: " << params.rotor_params.propeller_diameter << " m" << std::endl;
        std::cout << "  Max thrust per rotor: " << params.rotor_params.max_thrust << " N" << std::endl;
        
        real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
        real_T twr = total_thrust / (params.mass * 9.81f);
        std::cout << "  Total thrust: " << total_thrust << " N" << std::endl;
        std::cout << "  Thrust-to-weight ratio: " << twr << std::endl;
    }
    
    void printPhysicsInfo(const MultiRotorParams::Params& params) {
        std::cout << "Physics Properties:" << std::endl;
        std::cout << "  Inertia (Ixx, Iyy, Izz): " << params.inertia(0,0) << ", " 
                  << params.inertia(1,1) << ", " << params.inertia(2,2) << std::endl;
        std::cout << "  Linear drag: " << params.linear_drag_coefficient << std::endl;
        std::cout << "  Angular drag: " << params.angular_drag_coefficient << std::endl;
        std::cout << "  Restitution: " << params.restitution << std::endl;
        std::cout << "  Friction: " << params.friction << std::endl;
    }
    
    void printPerformanceInfo(const MultiRotorParams::Params& params) {
        std::cout << "Performance Estimates:" << std::endl;
        
        // Estimate hover thrust percentage
        real_T hover_thrust = params.mass * 9.81f;
        real_T total_max_thrust = params.rotor_count * params.rotor_params.max_thrust;
        real_T hover_percentage = (hover_thrust / total_max_thrust) * 100.0f;
        std::cout << "  Hover throttle: " << hover_percentage << "%" << std::endl;
        
        // Estimate natural frequency
        real_T control_authority = total_max_thrust / params.mass;
        real_T natural_freq = sqrt(control_authority) / (2 * M_PI);
        std::cout << "  Natural frequency: " << natural_freq << " Hz" << std::endl;
        
        // Estimate damping ratio
        real_T damping_ratio = params.angular_drag_coefficient / sqrt(params.inertia(2,2));
        std::cout << "  Damping ratio: " << damping_ratio << std::endl;
    }
};
```

### Automated Issue Detection

```cpp
class IssueDetector {
public:
    std::vector<std::string> detectIssues(const MultiRotorParams::Params& params) {
        std::vector<std::string> issues;
        
        // Check for critical issues
        if (!checkThrustAdequacy(params)) {
            issues.push_back("CRITICAL: Insufficient thrust for takeoff");
        }
        
        if (!checkRotorBalance(params)) {
            issues.push_back("CRITICAL: Unbalanced rotor configuration");
        }
        
        if (!checkInertiaMatrix(params)) {
            issues.push_back("CRITICAL: Invalid inertia matrix");
        }
        
        // Check for performance issues
        if (!checkStabilityMargins(params)) {
            issues.push_back("WARNING: Poor stability margins");
        }
        
        if (!checkResponseTime(params)) {
            issues.push_back("WARNING: Slow response characteristics");
        }
        
        if (!checkEfficiency(params)) {
            issues.push_back("WARNING: Poor efficiency");
        }
        
        return issues;
    }
    
private:
    bool checkThrustAdequacy(const MultiRotorParams::Params& params) {
        real_T total_thrust = params.rotor_count * params.rotor_params.max_thrust;
        real_T weight = params.mass * 9.81f;
        return total_thrust > weight * 1.2f;  // 20% margin
    }
    
    bool checkRotorBalance(const MultiRotorParams::Params& params) {
        int cw_count = 0, ccw_count = 0;
        for (const auto& rotor : params.rotor_poses) {
            if (rotor.direction == RotorTurningDirection::RotorTurningDirectionCW) {
                cw_count++;
            } else {
                ccw_count++;
            }
        }
        return abs(cw_count - ccw_count) <= 1;
    }
    
    bool checkInertiaMatrix(const MultiRotorParams::Params& params) {
        return params.inertia(0,0) > 0 && params.inertia(1,1) > 0 && params.inertia(2,2) > 0;
    }
    
    bool checkStabilityMargins(const MultiRotorParams::Params& params) {
        real_T damping_ratio = params.angular_drag_coefficient / sqrt(params.inertia(2,2));
        return damping_ratio > 0.1f && damping_ratio < 2.0f;
    }
    
    bool checkResponseTime(const MultiRotorParams::Params& params) {
        real_T control_authority = params.rotor_count * params.rotor_params.max_thrust / params.mass;
        real_T natural_freq = sqrt(control_authority) / (2 * M_PI);
        return natural_freq > 1.0f && natural_freq < 20.0f;
    }
    
    bool checkEfficiency(const MultiRotorParams::Params& params) {
        real_T hover_thrust = params.mass * 9.81f;
        real_T total_max_thrust = params.rotor_count * params.rotor_params.max_thrust;
        real_T hover_percentage = hover_thrust / total_max_thrust;
        return hover_percentage > 0.3f && hover_percentage < 0.7f;
    }
};
```

## Getting Help

### When to Ask for Help

1. **After following this guide** - Try the relevant troubleshooting steps first
2. **With specific symptoms** - Describe what you observe, not what you think is wrong
3. **With configuration details** - Include relevant parameter values
4. **With error messages** - Copy exact error text and stack traces

### Information to Include

```cpp
void generateHelpRequest(const MultiRotorParams::Params& params) {
    std::cout << "=== HELP REQUEST INFO ===" << std::endl;
    
    // Basic system info
    std::cout << "System: " << getSystemInfo() << std::endl;
    std::cout << "AirSim version: " << getAirSimVersion() << std::endl;
    
    // Configuration info
    DebugInfoCollector collector;
    collector.collectAllDebugInfo(params);
    
    // Issue detection
    IssueDetector detector;
    auto issues = detector.detectIssues(params);
    
    std::cout << "Detected issues:" << std::endl;
    for (const auto& issue : issues) {
        std::cout << "  " << issue << std::endl;
    }
    
    // Settings file
    std::cout << "Settings file content:" << std::endl;
    // Include relevant parts of settings.json
    
    std::cout << "=== END HELP REQUEST ===" << std::endl;
}
```

### Community Resources

- **GitHub Issues**: For bug reports and feature requests
- **Documentation**: Check the other guides in this directory
- **Forums**: Community discussions and solutions
- **Code Examples**: Look at existing configurations for reference

## Next Steps

If you're still experiencing issues:

1. **Review related documentation**:
   - [Overview](overview.md) for architecture understanding
   - [Frame Types](frame-types.md) for configuration examples
   - [Performance Tuning](performance-tuning.md) for optimization

2. **Try minimal configurations** to isolate the problem

3. **Test with known-good configurations** before customizing

4. **Consider contributing** solutions back to the documentation

Remember: Most configuration issues stem from incorrect parameter values rather than fundamental problems with the system. Systematic debugging and validation usually reveals the root cause.