# Custom ROS2 Service Development Guide for AirSim

This guide shows you how to create custom ROS2 services for AirSim drone control. We'll build two example services: **FlyCircle** (make drone fly in a circle) and **ChangeAltitude** (change drone altitude).

## 🎯 Overview

Creating custom services with AirSim ROS2 is straightforward and follows a standard pattern:

1. **Define Service Interface** (.srv files)
2. **Update CMakeLists.txt** (register new services)  
3. **Implement Service Logic** (C++ callback functions)
4. **Register Service** (in ROS wrapper constructor)
5. **Build and Test** (compile and use)

## 📋 Prerequisites

- AirSim ROS2 Docker environment running
- Basic understanding of ROS2 services
- C++ programming knowledge
- Text editor for code modifications

## 🔧 Step-by-Step Development

### Step 1: Define Service Interfaces

First, create the service interface files in the `airsim_interfaces` package.

#### Create FlyCircle Service

```bash
# Create service file
./ros2_exec.bat "touch /airsim_ros2_ws/src/airsim_interfaces/srv/FlyCircle.srv"
```

**File: `ros2/src/airsim_interfaces/srv/FlyCircle.srv`**
```cpp
# Request: Parameters for circular flight
float64 radius              # Circle radius in meters (e.g., 10.0)
float64 altitude            # Flight altitude in meters (e.g., -20.0, negative is up)
float64 speed               # Flight speed in m/s (e.g., 2.0)
float64 duration            # Duration to fly in seconds (0 = infinite)
string center_frame         # "body" or "world" - center relative to current position or world origin
string vehicle_name         # Vehicle name (e.g., "Drone1")
bool wait_on_last_task     # Wait for previous command to complete
---
# Response: Result of the operation
bool success               # True if command was accepted
string message            # Status message or error description
float64 estimated_duration # Estimated time to complete the circle
```

#### Create ChangeAltitude Service

**File: `ros2/src/airsim_interfaces/srv/ChangeAltitude.srv`**
```cpp
# Request: Altitude change parameters
float64 target_altitude    # Target altitude in meters (negative is up)
float64 climb_rate        # Climb/descent rate in m/s (e.g., 1.0)
string reference_frame    # "relative" or "absolute"
string vehicle_name       # Vehicle name (e.g., "Drone1") 
bool wait_on_last_task   # Wait for previous command to complete
---
# Response: Result of the operation
bool success             # True if command was accepted
string message          # Status message or error description
float64 current_altitude # Current altitude before change
```

### Step 2: Update Interface Package Configuration

#### Update CMakeLists.txt

**File: `ros2/src/airsim_interfaces/CMakeLists.txt`**
```cmake
# Find the existing rosidl_generate_interfaces section and add:
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing services ...
  "srv/FlyCircle.srv"
  "srv/ChangeAltitude.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
)
```

### Step 3: Update Main Package Headers

#### Add Service Includes

**File: `ros2/src/airsim_ros_pkgs/include/airsim_ros_wrapper.h`**

Add these includes with the other service includes:
```cpp
#include <airsim_interfaces/srv/fly_circle.hpp>
#include <airsim_interfaces/srv/change_altitude.hpp>
```

Add these service declarations in the private section of `AirsimROSWrapper` class:
```cpp
private:
    // ... existing service callbacks ...
    
    // Custom service callbacks
    bool fly_circle_srv_cb(const std::shared_ptr<airsim_interfaces::srv::FlyCircle::Request> request, 
                          const std::shared_ptr<airsim_interfaces::srv::FlyCircle::Response> response, 
                          const std::string& vehicle_name);
    
    bool change_altitude_srv_cb(const std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Request> request, 
                               const std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Response> response, 
                               const std::string& vehicle_name);

    // Service server declarations
    rclcpp::Service<airsim_interfaces::srv::FlyCircle>::SharedPtr fly_circle_srvr_;
    rclcpp::Service<airsim_interfaces::srv::ChangeAltitude>::SharedPtr change_altitude_srvr_;
```

### Step 4: Implement Service Logic

#### Add Service Registration

**File: `ros2/src/airsim_ros_pkgs/src/airsim_ros_wrapper.cpp`**

In the `create_ros_pubs_from_settings_json()` function, add service registration for each drone:

```cpp
// Find the section where takeoff and land services are created and add:
if (airsim_mode_ == AIRSIM_MODE::DRONE) {
    auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
    
    // ... existing service creation ...
    
    // Custom services
    std::function<bool(std::shared_ptr<airsim_interfaces::srv::FlyCircle::Request>, 
                      std::shared_ptr<airsim_interfaces::srv::FlyCircle::Response>)> 
        fcn_fly_circle_srvr = std::bind(&AirsimROSWrapper::fly_circle_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
    drone->fly_circle_srvr_ = nh_->create_service<airsim_interfaces::srv::FlyCircle>(topic_prefix + "/fly_circle", fcn_fly_circle_srvr);

    std::function<bool(std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Request>, 
                      std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Response>)> 
        fcn_change_altitude_srvr = std::bind(&AirsimROSWrapper::change_altitude_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
    drone->change_altitude_srvr_ = nh_->create_service<airsim_interfaces::srv::ChangeAltitude>(topic_prefix + "/change_altitude", fcn_change_altitude_srvr);
}
```

#### Implement FlyCircle Service Callback

Add this function to `airsim_ros_wrapper.cpp`:

```cpp
bool AirsimROSWrapper::fly_circle_srv_cb(const std::shared_ptr<airsim_interfaces::srv::FlyCircle::Request> request,
                                         const std::shared_ptr<airsim_interfaces::srv::FlyCircle::Response> response,
                                         const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        RCLCPP_INFO(nh_->get_logger(), "Fly circle service called for vehicle: %s", vehicle_name.c_str());
        
        // Validate parameters
        if (request->radius <= 0) {
            response->success = false;
            response->message = "Radius must be positive";
            return true;
        }
        
        if (request->speed <= 0) {
            response->success = false;
            response->message = "Speed must be positive";
            return true;
        }

        // Get current position as circle center
        auto vehicle_it = vehicle_name_ptr_map_.find(vehicle_name);
        if (vehicle_it == vehicle_name_ptr_map_.end()) {
            response->success = false;
            response->message = "Vehicle not found: " + vehicle_name;
            return true;
        }

        // Get current state
        auto state = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->getMultirotorState(vehicle_name);
        msr::airlib::Vector3r center_pos = state.getPosition();
        
        // Adjust center position based on frame
        if (request->center_frame == "world") {
            center_pos = msr::airlib::Vector3r(0, 0, request->altitude);
        } else {
            center_pos.z() = request->altitude;
        }

        // Calculate circle parameters
        double circumference = 2 * M_PI * request->radius;
        double estimated_duration = circumference / request->speed;
        
        // Create circular path using velocity commands
        // This is a simplified implementation - in practice, you'd want a more sophisticated trajectory
        
        // For demonstration, we'll move to starting position first
        msr::airlib::Vector3r start_pos = center_pos + msr::airlib::Vector3r(request->radius, 0, 0);
        
        // Use AirSim API to move to start position
        if (request->wait_on_last_task) {
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveToPositionAsync(
                start_pos.x(), start_pos.y(), start_pos.z(), request->speed, vehicle_name)->waitOnLastTask();
        }
        
        response->success = true;
        response->message = "Circle flight initiated";
        response->estimated_duration = estimated_duration;
        
        RCLCPP_INFO(nh_->get_logger(), "Circle flight started for %s: radius=%.2f, speed=%.2f", 
                   vehicle_name.c_str(), request->radius, request->speed);
        
        return true;
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "FlyCircle service failed: %s", msg.c_str());
        response->success = false;
        response->message = "RPC error: " + msg;
        return true;
    }
}
```

#### Implement ChangeAltitude Service Callback

```cpp
bool AirsimROSWrapper::change_altitude_srv_cb(const std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Request> request,
                                              const std::shared_ptr<airsim_interfaces::srv::ChangeAltitude::Response> response,
                                              const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        RCLCPP_INFO(nh_->get_logger(), "Change altitude service called for vehicle: %s", vehicle_name.c_str());
        
        // Get current state
        auto state = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->getMultirotorState(vehicle_name);
        msr::airlib::Vector3r current_pos = state.getPosition();
        
        response->current_altitude = current_pos.z();
        
        // Calculate target position
        msr::airlib::Vector3r target_pos = current_pos;
        
        if (request->reference_frame == "absolute") {
            target_pos.z() = request->target_altitude;
        } else { // relative
            target_pos.z() += request->target_altitude;
        }
        
        // Validate climb rate
        if (request->climb_rate <= 0) {
            response->success = false;
            response->message = "Climb rate must be positive";
            return true;
        }
        
        // Execute altitude change
        auto future = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveToPositionAsync(
            target_pos.x(), target_pos.y(), target_pos.z(), request->climb_rate, vehicle_name);
        
        if (request->wait_on_last_task) {
            future->waitOnLastTask();
        }
        
        response->success = true;
        response->message = "Altitude change command sent";
        
        RCLCPP_INFO(nh_->get_logger(), "Altitude change for %s: %.2f -> %.2f at %.2f m/s", 
                   vehicle_name.c_str(), current_pos.z(), target_pos.z(), request->climb_rate);
        
        return true;
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "ChangeAltitude service failed: %s", msg.c_str());
        response->success = false;
        response->message = "RPC error: " + msg;
        return true;
    }
}
```

#### Update MultiRotorROS Class

In the header file, add service declarations to the `MultiRotorROS` class:

```cpp
class MultiRotorROS : public VehicleROS
{
public:
    // ... existing members ...
    
    // Custom services
    rclcpp::Service<airsim_interfaces::srv::FlyCircle>::SharedPtr fly_circle_srvr_;
    rclcpp::Service<airsim_interfaces::srv::ChangeAltitude>::SharedPtr change_altitude_srvr_;
};
```

### Step 5: Build and Deploy

#### Build in Docker Container

```bash
# Build the updated ROS2 workspace
./ros2_exec.bat "cd /airsim_ros2_ws && colcon build --packages-select airsim_interfaces airsim_ros_pkgs"

# Source the updated workspace
./ros2_exec.bat "source /airsim_ros2_ws/install/setup.bash"
```

#### Restart Container

```bash
# Stop current container
docker stop airsim_ros2_container

# Start with updated build
./run_simple.bat
```

## 🧪 Testing Your Custom Services

### Test FlyCircle Service

```bash
# Basic circle flight
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '{radius: 10.0, altitude: -20.0, speed: 2.0, duration: 30.0, center_frame: \"body\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"

# Large circle around world origin
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/fly_circle airsim_interfaces/srv/FlyCircle '{radius: 50.0, altitude: -30.0, speed: 5.0, duration: 0.0, center_frame: \"world\", vehicle_name: \"Drone1\", wait_on_last_task: false}'"
```

### Test ChangeAltitude Service

```bash
# Absolute altitude change
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '{target_altitude: -15.0, climb_rate: 1.5, reference_frame: \"absolute\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"

# Relative altitude change (climb 5 meters)
./ros2_exec.bat "ros2 service call /airsim_node/Drone1/change_altitude airsim_interfaces/srv/ChangeAltitude '{target_altitude: -5.0, climb_rate: 2.0, reference_frame: \"relative\", vehicle_name: \"Drone1\", wait_on_last_task: true}'"
```

### Check Service Availability

```bash
# List all services
./ros2_exec.bat "ros2 service list | grep -E '(fly_circle|change_altitude)'"

# Get service interface details
./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/FlyCircle"
./ros2_exec.bat "ros2 interface show airsim_interfaces/srv/ChangeAltitude"
```

## 🎯 Advanced Development Patterns

### 1. **Asynchronous Service Implementation**

For complex maneuvers, use asynchronous execution:

```cpp
// In service callback
std::thread execution_thread([this, request, vehicle_name]() {
    // Long-running operation
    execute_complex_maneuver(request, vehicle_name);
});
execution_thread.detach();

response->success = true;
response->message = "Command initiated asynchronously";
```

### 2. **State Machine Services**

For multi-step operations:

```cpp
enum class FlightState { IDLE, TAKING_OFF, FLYING_CIRCLE, LANDING };

class CircleFlight {
    FlightState state_ = FlightState::IDLE;
    double current_angle_ = 0.0;
    
    void update() {
        switch(state_) {
            case FlightState::FLYING_CIRCLE:
                execute_circle_step();
                break;
            // ... other states
        }
    }
};
```

### 3. **Parameter Validation**

Always validate service parameters:

```cpp
bool validate_circle_params(const FlyCircle::Request& req, FlyCircle::Response& res) {
    if (req.radius <= 0 || req.radius > 1000) {
        res.success = false;
        res.message = "Radius must be between 0 and 1000 meters";
        return false;
    }
    // ... more validation
    return true;
}
```

### 4. **Multi-Vehicle Services**

Create services that control multiple drones:

```cpp
bool fly_formation_circle_srv_cb(const FormationCircle::Request& req, FormationCircle::Response& res) {
    for (const auto& vehicle_name : req.vehicle_names) {
        // Start circle for each vehicle with offset
        start_circle_for_vehicle(vehicle_name, req.radius, offset);
    }
}
```

## 🔧 Complete Development Workflow

### Development Checklist

- [ ] **Design**: Plan service interface and parameters
- [ ] **Define**: Create .srv interface files  
- [ ] **Update**: Modify CMakeLists.txt and package.xml
- [ ] **Implement**: Add C++ callback functions
- [ ] **Register**: Create service servers in constructor
- [ ] **Build**: Compile with colcon build
- [ ] **Test**: Use ros2 service call to test
- [ ] **Debug**: Check logs for errors
- [ ] **Document**: Add usage examples

### Best Practices

1. **Thread Safety**: Always use `std::lock_guard<std::mutex> guard(control_mutex_);`
2. **Error Handling**: Wrap AirSim calls in try-catch blocks
3. **Parameter Validation**: Check all input parameters
4. **Logging**: Use RCLCPP_INFO/WARN/ERROR for debugging
5. **Response Messages**: Provide meaningful success/error messages
6. **Timeouts**: Consider adding timeout parameters for long operations

### Common Pitfalls to Avoid

1. **Blocking Operations**: Don't block service callbacks for too long
2. **Missing Mutex**: Always lock when accessing AirSim client
3. **Invalid Vehicle Names**: Check if vehicle exists before operations
4. **Coordinate Frame Confusion**: Be clear about NED vs ENU frames
5. **Missing Error Handling**: Always handle RPC exceptions

## 📚 Additional Resources

### AirSim API Reference
- **MultirotorRpcLibClient**: Main drone control interface
- **moveToPositionAsync()**: Position-based movement
- **moveByVelocityAsync()**: Velocity-based movement  
- **getMultirotorState()**: Get current drone state

### ROS2 Service Development
- [ROS2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Custom Interface Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

### AirSim Documentation
- [AirSim APIs](https://microsoft.github.io/AirSim/apis/)
- [Python Examples](https://github.com/Microsoft/AirSim/tree/main/PythonClient)

---

**With this guide, you can create any custom drone behavior as a ROS2 service!** The pattern is consistent and scales well for complex autonomous flight operations. 