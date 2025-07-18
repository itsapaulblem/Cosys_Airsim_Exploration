# Adding New Services, Messages, and Actions to AirSim ROS2

This guide explains how to add new ROS2 services, messages, and actions to the AirSim ROS2 package. We'll use the recently added `SetAltitude` service as a practical example.

## Table of Contents

1. [Package Structure Overview](#package-structure-overview)
2. [Adding New Services](#adding-new-services)
3. [Adding New Messages](#adding-new-messages)
4. [Adding New Actions](#adding-new-actions)
5. [Building and Testing](#building-and-testing)
6. [Best Practices](#best-practices)
7. [Troubleshooting](#troubleshooting)

## Package Structure Overview

The AirSim ROS2 workspace has two main packages:

```
ros2/src/
├── airsim_interfaces/          # Interface definitions (messages, services, actions)
│   ├── msg/                    # Message definitions (.msg files)
│   ├── srv/                    # Service definitions (.srv files)
│   ├── action/                 # Action definitions (.action files)
│   ├── CMakeLists.txt          # Build configuration for interfaces
│   └── package.xml             # Package metadata
└── airsim_ros_pkgs/           # Main implementation package
    ├── src/                    # C++ source files
    ├── include/                # C++ header files
    ├── scripts/                # Python scripts and examples
    ├── launch/                 # Launch files
    ├── CMakeLists.txt          # Build configuration
    └── package.xml             # Package metadata
```

## Adding New Services

### Step 1: Create the Service Definition

Create a new `.srv` file in `airsim_interfaces/srv/`:

```bash
# Example: SetAltitude.srv
touch ros2/src/airsim_interfaces/srv/YourNewService.srv
```

**Service file format:**
```
# Request fields (input parameters)
float64 parameter1
string parameter2
bool parameter3
---
# Response fields (output parameters)
bool success
string message
```

**Real example** (`SetAltitude.srv`):
```
#Request : expects altitude setpoint via z (in meters, NED coordinate system)
#Request : negative z means higher altitude (NED convention)
float64 z
float64 velocity
string vehicle_name
bool wait_on_last_task
---
#Response : success=true if altitude change completed successfully
bool success
string message
```

### Step 2: Register the Service in CMakeLists.txt

Edit `airsim_interfaces/CMakeLists.txt` and add your service to the `rosidl_generate_interfaces` section:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing messages ...
  "srv/SetGPSPosition.srv"
  "srv/Takeoff.srv"
  "srv/TakeoffGroup.srv"
  "srv/Land.srv"
  "srv/LandGroup.srv"
  "srv/Reset.srv"
  "srv/RefreshInstanceSegmentation.srv"
  "srv/RefreshObjectTransforms.srv"
  "srv/SetLocalPosition.srv"
  "srv/SetAltitude.srv"              # <- Add your new service here
  "srv/YourNewService.srv"           # <- Your new service
  "srv/ListSceneObjectTags.srv" 
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs LIBRARY_NAME ${PROJECT_NAME}
)
```

### Step 3: Add Include in Header File

Edit `airsim_ros_pkgs/include/airsim_ros_wrapper.h`:

```cpp
// Add include for your service
#include <airsim_interfaces/srv/your_new_service.hpp>
```

**Example:**
```cpp
#include <airsim_interfaces/srv/set_altitude.hpp>
```

### Step 4: Add Service Server Declaration

In the same header file, add the service server and callback declarations:

```cpp
// In the appropriate class (MultiRotorROS for drone services)
class MultiRotorROS : public VehicleROS
{
public:
    // ... existing members ...
    rclcpp::Service<airsim_interfaces::srv::YourNewService>::SharedPtr your_service_srvr_;
};

// In the private section of AirsimROSWrapper class
bool your_service_srv_cb(const std::shared_ptr<airsim_interfaces::srv::YourNewService::Request> request, 
                        const std::shared_ptr<airsim_interfaces::srv::YourNewService::Response> response, 
                        const std::string& vehicle_name);
```

**Example:**
```cpp
// Service server in MultiRotorROS class
rclcpp::Service<airsim_interfaces::srv::SetAltitude>::SharedPtr set_altitude_srvr_;

// Callback declaration
bool set_altitude_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Request> request, 
                        const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Response> response, 
                        const std::string& vehicle_name);
```

### Step 5: Create the Service Server

Edit `airsim_ros_pkgs/src/airsim_ros_wrapper.cpp` to create the service server in the constructor:

```cpp
// In the drone initialization section (around line 220)
if(airsim_mode_ == AIRSIM_MODE::DRONE) {
    auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
    
    // ... existing service creation ...
    
    // Add your new service
    std::function<bool(std::shared_ptr<airsim_interfaces::srv::YourNewService::Request>, 
                      std::shared_ptr<airsim_interfaces::srv::YourNewService::Response>)> 
        fcn_your_service_srvr = std::bind(&AirsimROSWrapper::your_service_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
    drone->your_service_srvr_ = nh_->create_service<airsim_interfaces::srv::YourNewService>(
        topic_prefix + "/your_service", fcn_your_service_srvr);
}
```

### Step 6: Implement the Service Callback

Add the callback implementation in the same file:

```cpp
bool AirsimROSWrapper::your_service_srv_cb(
    std::shared_ptr<airsim_interfaces::srv::YourNewService::Request> request, 
    std::shared_ptr<airsim_interfaces::srv::YourNewService::Response> response, 
    const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    try {
        // Your service logic here
        RCLCPP_INFO(nh_->get_logger(), "Processing service request for vehicle '%s'", vehicle_name.c_str());
        
        // Call AirSim API
        // Example: static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->someMethod(...);
        
        response->success = true;
        response->message = "Service completed successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
        RCLCPP_ERROR(nh_->get_logger(), "Exception in service callback: %s", e.what());
    }

    return true;
}
```

## Adding New Messages

### Step 1: Create the Message Definition

Create a new `.msg` file in `airsim_interfaces/msg/`:

```bash
touch ros2/src/airsim_interfaces/msg/YourNewMessage.msg
```

**Message file format:**
```
# Header (optional but recommended for timestamped data)
std_msgs/Header header

# Your custom fields
float64 field1
string field2
geometry_msgs/Vector3 field3
bool field4
```

**Example** (`CustomDroneState.msg`):
```
std_msgs/Header header
string vehicle_name
float64 altitude
float64 battery_level
bool is_armed
geometry_msgs/Vector3 velocity
```

### Step 2: Register the Message in CMakeLists.txt

Edit `airsim_interfaces/CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalAngleEulerCmd.msg"
  "msg/GimbalAngleQuatCmd.msg"
  "msg/GPSYaw.msg"
  # ... existing messages ...
  "msg/YourNewMessage.msg"           # <- Add your new message here
  # ... existing services ...
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs LIBRARY_NAME ${PROJECT_NAME}
)
```

### Step 3: Add Include and Publisher/Subscriber

In `airsim_ros_pkgs/include/airsim_ros_wrapper.h`:

```cpp
// Add include
#include <airsim_interfaces/msg/your_new_message.hpp>

// Add publisher/subscriber in appropriate class
class MultiRotorROS : public VehicleROS
{
public:
    rclcpp::Publisher<airsim_interfaces::msg::YourNewMessage>::SharedPtr your_message_pub_;
    rclcpp::Subscription<airsim_interfaces::msg::YourNewMessage>::SharedPtr your_message_sub_;
};

// Add callback declaration
void your_message_cb(const airsim_interfaces::msg::YourNewMessage::SharedPtr msg, const std::string& vehicle_name);
```

### Step 4: Create Publisher/Subscriber

In `airsim_ros_pkgs/src/airsim_ros_wrapper.cpp`:

```cpp
// Create publisher
drone->your_message_pub_ = nh_->create_publisher<airsim_interfaces::msg::YourNewMessage>(
    topic_prefix + "/your_message", 10);

// Create subscriber (if needed)
std::function<void(const airsim_interfaces::msg::YourNewMessage::SharedPtr)> fcn_your_message_sub = 
    std::bind(&AirsimROSWrapper::your_message_cb, this, _1, vehicle_ros->vehicle_name_);
drone->your_message_sub_ = nh_->create_subscription<airsim_interfaces::msg::YourNewMessage>(
    topic_prefix + "/your_message_cmd", 1, fcn_your_message_sub);
```

### Step 5: Implement Message Handling

```cpp
// Publishing a message
void AirsimROSWrapper::publish_your_message(const std::string& vehicle_name) {
    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    
    airsim_interfaces::msg::YourNewMessage msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = world_frame_id_;
    // Fill in your message fields
    msg.field1 = some_value;
    msg.field2 = "some_string";
    
    drone->your_message_pub_->publish(msg);
}

// Handling received messages
void AirsimROSWrapper::your_message_cb(const airsim_interfaces::msg::YourNewMessage::SharedPtr msg, const std::string& vehicle_name) {
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    // Process the received message
    RCLCPP_INFO(nh_->get_logger(), "Received message for vehicle '%s'", vehicle_name.c_str());
    // Your processing logic here
}
```

## Adding New Actions

ROS2 Actions are used for long-running tasks that provide feedback and can be preempted. They're perfect for operations like "fly to waypoint" or "execute mission" where you need progress updates.

### Step 1: Create the Action Definition

Create a new `.action` file in `airsim_interfaces/action/`:

```bash
touch ros2/src/airsim_interfaces/action/YourNewAction.action
```

**Action file format:**
```
# Goal (what you want to achieve)
float64 target_parameter
string mission_type
bool enable_feedback
---
# Result (final outcome)
bool success
string message
float64 final_value
---
# Feedback (progress updates)
float64 current_progress
string current_status
float64 time_elapsed
```

**Example** (`FlyToWaypoint.action`):
```
# Goal: Fly to a specific waypoint with altitude
geometry_msgs/Point target_position
float64 target_altitude
float64 velocity
string vehicle_name
bool enable_collision_avoidance
---
# Result: Final outcome of the flight
bool success
string message
geometry_msgs/Point final_position
float64 final_altitude
float64 total_distance_traveled
---
# Feedback: Progress updates during flight
float64 progress_percentage
geometry_msgs/Point current_position
float64 current_altitude
float64 distance_remaining
string current_status
```

### Step 2: Register the Action in CMakeLists.txt

Edit `airsim_interfaces/CMakeLists.txt` and add your action to the `rosidl_generate_interfaces` section:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/GimbalAngleEulerCmd.msg"
  "msg/GimbalAngleQuatCmd.msg"
  "msg/GPSYaw.msg"
  # ... existing messages ...
  
  # Services
  "srv/SetGPSPosition.srv"
  "srv/Takeoff.srv"
  "srv/SetAltitude.srv"
  # ... existing services ...
  
  # Actions
  "action/FlyToWaypoint.action"        # <- Add your new action here
  "action/YourNewAction.action"        # <- Your new action
  
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs LIBRARY_NAME ${PROJECT_NAME}
)
```

### Step 3: Add Include in Header File

Edit `airsim_ros_pkgs/include/airsim_ros_wrapper.h`:

```cpp
// Add include for your action
#include <airsim_interfaces/action/your_new_action.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
```

**Example:**
```cpp
#include <airsim_interfaces/action/fly_to_waypoint.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
```

### Step 4: Add Action Server Declaration

In the same header file, add the action server and callback declarations:

```cpp
// In the appropriate class (MultiRotorROS for drone actions)
class MultiRotorROS : public VehicleROS
{
public:
    // ... existing members ...
    using YourNewAction = airsim_interfaces::action::YourNewAction;
    using GoalHandleYourNewAction = rclcpp_action::ServerGoalHandle<YourNewAction>;
    
    rclcpp_action::Server<YourNewAction>::SharedPtr your_action_server_;
};

// In the private section of AirsimROSWrapper class
rclcpp_action::GoalResponse handle_your_action_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const YourNewAction::Goal> goal);

rclcpp_action::CancelResponse handle_your_action_cancel(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle);

void handle_your_action_accepted(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle);

void execute_your_action(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle,
    const std::string& vehicle_name);
```

**Example:**
```cpp
// Action server in MultiRotorROS class
using FlyToWaypoint = airsim_interfaces::action::FlyToWaypoint;
using GoalHandleFlyToWaypoint = rclcpp_action::ServerGoalHandle<FlyToWaypoint>;

rclcpp_action::Server<FlyToWaypoint>::SharedPtr fly_to_waypoint_server_;

// Callback declarations
rclcpp_action::GoalResponse handle_fly_to_waypoint_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FlyToWaypoint::Goal> goal);

rclcpp_action::CancelResponse handle_fly_to_waypoint_cancel(
    const std::shared_ptr<GoalHandleFlyToWaypoint> goal_handle);

void handle_fly_to_waypoint_accepted(
    const std::shared_ptr<GoalHandleFlyToWaypoint> goal_handle);

void execute_fly_to_waypoint(
    const std::shared_ptr<GoalHandleFlyToWaypoint> goal_handle,
    const std::string& vehicle_name);
```

### Step 5: Create the Action Server

Edit `airsim_ros_pkgs/src/airsim_ros_wrapper.cpp` to create the action server in the constructor:

```cpp
// In the drone initialization section (around line 220)
if(airsim_mode_ == AIRSIM_MODE::DRONE) {
    auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
    
    // ... existing service creation ...
    
    // Add your new action server
    drone->your_action_server_ = rclcpp_action::create_server<YourNewAction>(
        nh_,
        topic_prefix + "/your_action",
        std::bind(&AirsimROSWrapper::handle_your_action_goal, this, _1, _2),
        std::bind(&AirsimROSWrapper::handle_your_action_cancel, this, _1),
        std::bind(&AirsimROSWrapper::handle_your_action_accepted, this, _1),
        rcl_action_server_get_default_options(),
        callback_group_);
}
```

### Step 6: Implement the Action Callbacks

Add the callback implementations in the same file:

```cpp
// Goal handling
rclcpp_action::GoalResponse AirsimROSWrapper::handle_your_action_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const YourNewAction::Goal> goal)
{
    RCLCPP_INFO(nh_->get_logger(), "Received action goal request");
    
    // Validate goal parameters
    if (goal->target_parameter < 0) {
        RCLCPP_WARN(nh_->get_logger(), "Invalid goal parameter: %f", goal->target_parameter);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel handling
rclcpp_action::CancelResponse AirsimROSWrapper::handle_your_action_cancel(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle)
{
    RCLCPP_INFO(nh_->get_logger(), "Received request to cancel action");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Accepted goal handling
void AirsimROSWrapper::handle_your_action_accepted(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle)
{
    // Execute the action in a separate thread
    std::thread{std::bind(&AirsimROSWrapper::execute_your_action, this, goal_handle, vehicle_name_)}.detach();
}

// Action execution
void AirsimROSWrapper::execute_your_action(
    const std::shared_ptr<GoalHandleYourNewAction> goal_handle,
    const std::string& vehicle_name)
{
    RCLCPP_INFO(nh_->get_logger(), "Executing action for vehicle '%s'", vehicle_name.c_str());
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<YourNewAction::Feedback>();
    auto result = std::make_shared<YourNewAction::Result>();
    
    // Action execution loop
    for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Action was cancelled";
            goal_handle->canceled(result);
            RCLCPP_INFO(nh_->get_logger(), "Action cancelled");
            return;
        }
        
        // Update progress
        feedback->current_progress = i;
        feedback->current_status = "Processing step " + std::to_string(i);
        feedback->time_elapsed = i * 0.1; // Example timing
        
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        
        // Your action logic here
        try {
            // Call AirSim API
            // Example: static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->someMethod(...);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            result->success = false;
            result->message = std::string("Error: ") + e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(nh_->get_logger(), "Action failed: %s", e.what());
            return;
        }
    }
    
    // Check if goal is done
    if (rclcpp::ok()) {
        result->success = true;
        result->message = "Action completed successfully";
        result->final_value = 100.0;
        goal_handle->succeed(result);
        RCLCPP_INFO(nh_->get_logger(), "Action succeeded");
    }
}
```

### Step 7: Create Action Client Example

Create a Python example script `airsim_ros_pkgs/scripts/your_action_client_example.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from airsim_interfaces.action import YourNewAction

class YourActionClient(Node):
    def __init__(self):
        super().__init__('your_action_client')
        self._action_client = ActionClient(self, YourNewAction, '/airsim_node/Drone1/your_action')

    def send_goal(self, target_parameter, mission_type):
        goal_msg = YourNewAction.Goal()
        goal_msg.target_parameter = target_parameter
        goal_msg.mission_type = mission_type
        goal_msg.enable_feedback = True

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.current_progress}% - {feedback.current_status}')

def main(args=None):
    rclpy.init(args=args)
    action_client = YourActionClient()
    action_client.send_goal(50.0, "test_mission")
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Building and Testing

### Step 1: Build the Interfaces Package

```bash
cd /path/to/your/workspace
colcon build --packages-select airsim_interfaces
```

### Step 2: Build the Main Package

```bash
colcon build --packages-select airsim_ros_pkgs
```

### Step 3: Source the Workspace

```bash
source install/setup.bash
```

### Step 4: Test Your Service/Message

**For services:**
```bash
# Check if service exists
ros2 service list | grep your_service

# Check service type
ros2 service type /airsim_node/Drone1/your_service

# Call service
ros2 service call /airsim_node/Drone1/your_service airsim_interfaces/srv/YourNewService "{field1: 1.0, field2: 'test'}"
```

**For messages:**
```bash
# Check if topic exists
ros2 topic list | grep your_message

# Check message type
ros2 topic info /airsim_node/Drone1/your_message

# Echo messages
ros2 topic echo /airsim_node/Drone1/your_message
```

**For actions:**
```bash
# Check if action exists
ros2 action list | grep your_action

# Check action type
ros2 action info /airsim_node/Drone1/your_action

# Send action goal
ros2 action send_goal /airsim_node/Drone1/your_action airsim_interfaces/action/YourNewAction "{target_parameter: 50.0, mission_type: 'test'}"

# Send action goal with feedback
ros2 action send_goal /airsim_node/Drone1/your_action airsim_interfaces/action/YourNewAction "{target_parameter: 50.0, mission_type: 'test'}" --feedback
```

## Best Practices

### 1. Naming Conventions

- **Services**: Use PascalCase (e.g., `SetAltitude`, `GetDroneState`)
- **Messages**: Use PascalCase (e.g., `DroneState`, `SensorData`)
- **Actions**: Use PascalCase (e.g., `FlyToWaypoint`, `ExecuteMission`)
- **Fields**: Use snake_case (e.g., `vehicle_name`, `target_altitude`)
- **Topics**: Use snake_case (e.g., `/set_altitude`, `/drone_state`)
- **Action endpoints**: Use snake_case (e.g., `/fly_to_waypoint`, `/execute_mission`)

### 2. Service Design

- Always include `success` (bool) and `message` (string) in responses
- Use meaningful parameter names and add comments
- Include `vehicle_name` for multi-drone support
- Consider async vs sync behavior with `wait_on_last_task`

### 3. Message Design

- Include `std_msgs/Header` for timestamped data
- Use standard geometry_msgs types when possible
- Keep messages focused and cohesive
- Document units and coordinate systems

### 4. Action Design

- **Goal**: Define clear, achievable objectives with validation
- **Result**: Include success status, descriptive messages, and final values
- **Feedback**: Provide meaningful progress updates (percentage, status, timing)
- Use standard geometry_msgs types for positions and orientations
- Include `vehicle_name` for multi-drone support
- Implement proper cancellation handling
- Consider timeout mechanisms for long-running actions

### 5. Error Handling

```cpp
bool YourService::callback(...) {
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        // Validate inputs
        if (request->parameter < 0) {
            response->success = false;
            response->message = "Invalid parameter: must be positive";
            return true;
        }
        
        // Your logic here
        
        response->success = true;
        response->message = "Operation completed successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
        RCLCPP_ERROR(nh_->get_logger(), "Exception: %s", e.what());
    }
    
    return true;
}
```

### 6. Documentation

- Create example scripts in `scripts/` directory
- Add comprehensive README documentation
- Include usage examples in multiple languages (Python, C++, CLI)

## Troubleshooting

### Common Build Issues

1. **Interface not found:**
   ```bash
   # Make sure you built interfaces first
   colcon build --packages-select airsim_interfaces
   source install/setup.bash
   ```

2. **Service/Message not generated:**
   - Check CMakeLists.txt includes your files
   - Verify proper syntax in .srv/.msg files
   - Clean build and rebuild

3. **Runtime service not available:**
   - Ensure AirSim wrapper is running
   - Check vehicle names match
   - Verify service creation in constructor

### Debugging Commands

```bash
# List all available services
ros2 service list

# List all available topics
ros2 topic list

# List all available actions
ros2 action list

# Check interface definitions
ros2 interface show airsim_interfaces/srv/YourService
ros2 interface show airsim_interfaces/msg/YourMessage
ros2 interface show airsim_interfaces/action/YourAction

# Check node info
ros2 node info /airsim_node
```

## Example: Complete SetAltitude Implementation

Here's the complete implementation that was added:

**Files created/modified:**
1. `airsim_interfaces/srv/SetAltitude.srv`
2. `airsim_interfaces/CMakeLists.txt` (updated)
3. `airsim_ros_pkgs/include/airsim_ros_wrapper.h` (updated)
4. `airsim_ros_pkgs/src/airsim_ros_wrapper.cpp` (updated)
5. `airsim_ros_pkgs/scripts/set_altitude_example.py` (new)
6. `airsim_ros_pkgs/scripts/set_altitude_cli.py` (new)
7. `airsim_ros_pkgs/README_SetAltitude.md` (new)

This serves as a complete reference implementation for adding new services to the AirSim ROS2 package.

## Development Workflow with Docker

### Using Volume Mounting for Development

The new `Dockerfile.ros2_vnc` supports volume mounting for seamless development:

```bash
# Build the new development-friendly Docker image
docker/airsim_ros2_wrapper/VNC/airsim_docker.bat build --dockerfile docker\airsim_ros2_wrapper\VNC\Dockerfile.ros2_vnc

# Run with volume mounting (default behavior)
docker/airsim_ros2_wrapper/VNC/airsim_docker.bat run

# Run without volume mounting
docker/airsim_ros2_wrapper/VNC/airsim_docker.bat run --no-volume-mount
```

### Development Features

**Volume Mounting:**
- Your local `ros2/src` folder is mounted to `/airsim_ros2_ws/src` in the container
- Changes made in your IDE are immediately reflected in the container
- No need to rebuild the Docker image for interface changes

**Default Directory:**
- VNC session starts in `/airsim_ros2_ws` (no more `cd ../../` needed!)
- Direct access to `launch_airsim_ros2.sh` script
- Convenient workspace navigation

**Development Aliases:**
- `build` - Build the entire workspace
- `build_interfaces` - Build only airsim_interfaces package
- `build_pkgs` - Build only airsim_ros_pkgs package
- `source_ws` - Source the workspace
- `clean_build` - Clean and rebuild everything

**Helper Scripts:**
- `/build_workspace.sh` - Complete workspace build
- `/build_interfaces.sh` - Quick interface rebuild
- `/launch_airsim_ros2.sh` - Launch AirSim ROS2 node

### Typical Development Workflow

1. **Edit interfaces on your local machine** (in your IDE)
   ```bash
   # Edit files in ros2/src/airsim_interfaces/
   # Add new .srv, .msg, or .action files
   # Update CMakeLists.txt
   ```

2. **Build interfaces in the container**
   ```bash
   # In VNC terminal or docker exec
   build_interfaces
   source_ws
   ```

3. **Edit implementation files** (also on local machine)
   ```bash
   # Edit files in ros2/src/airsim_ros_pkgs/
   # Update headers, source files, scripts
   ```

4. **Build packages in the container**
   ```bash
   # In VNC terminal
   build_pkgs
   source_ws
   ```

5. **Test your changes**
   ```bash
   # Launch AirSim ROS2 node
   ./launch_airsim_ros2.sh
   
   # Test services/actions in another terminal
   ros2 service list
   ros2 action list
   ```

### Volume Mount Path Details

**Host Path:** `L:\Cosys-AirSim\ros2\src`
**Container Path:** `/airsim_ros2_ws/src`

This means:
- `ros2/src/airsim_interfaces/` → `/airsim_ros2_ws/src/airsim_interfaces/`
- `ros2/src/airsim_ros_pkgs/` → `/airsim_ros2_ws/src/airsim_ros_pkgs/`

Changes to any files in these directories are immediately available in both locations.

## Next Steps

After adding your service/message/action:

1. **Test thoroughly** with real AirSim simulation
2. **Create example scripts** for different use cases
3. **Document usage** in README files
4. **Consider edge cases** and error conditions
5. **Add unit tests** if appropriate
6. **Use the development Docker container** for rapid iteration

For more complex integrations, consider studying existing implementations in the codebase and the AirSim API documentation. 