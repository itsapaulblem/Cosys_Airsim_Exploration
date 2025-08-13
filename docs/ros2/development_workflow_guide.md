# ROS2 Development Workflow Guide
## Adding New Topics, Actions, and Services in Cosys-AirSim

This guide provides a step-by-step workflow for adding new ROS2 interfaces to the Cosys-AirSim project.

## 📋 Prerequisites

- Docker environment set up (`./airsim_ros2_docker.bat run`)
- Basic understanding of ROS2 concepts
- Access to AirSim source code

## 🎯 Phase 1: Requirements Analysis

### 1.1 Identify Interface Type
- **Topic (.msg)** - For continuous data streams (sensor data, state updates)
- **Service (.srv)** - For request-response operations (takeoff, land, set position)
- **Action (.action)** - For long-running operations with feedback (missions, navigation)

### 1.2 Define Data Structure
- Determine input/output parameters
- Consider existing message types for reuse
- Plan for future extensibility

## 🔵 Phase 2: Interface Definition

### 2.1 Create Interface Files

**For Topics:**
```bash
# Create message file
touch ros2/src/airsim_interfaces/msg/YourNewMessage.msg
```

**For Services:**
```bash
# Create service file
touch ros2/src/airsim_interfaces/srv/YourNewService.srv
```

**For Actions:**
```bash
# Create action file
touch ros2/src/airsim_mission_interfaces/action/YourNewAction.action
```

### 2.2 Define Interface Structure

**Message Format (.msg):**
```
# YourNewMessage.msg
std_msgs/Header header
float64 parameter1
string parameter2
geometry_msgs/Vector3 position
```

**Service Format (.srv):**
```
# YourNewService.srv
# Request
float64 input_param
string vehicle_name
---
# Response
bool success
string message
```

**Action Format (.action):**
```
# YourNewAction.action
# Goal
geometry_msgs/Point target_position
float64 timeout
---
# Result
bool success
string result_message
---
# Feedback
float64 progress_percentage
geometry_msgs/Point current_position
```

### 2.3 Update CMakeLists.txt

**For airsim_interfaces:**
```cmake
# Add to ros2/src/airsim_interfaces/CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing messages ...
  "msg/YourNewMessage.msg"
  "srv/YourNewService.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
```

**For airsim_mission_interfaces:**
```cmake
# Add to ros2/src/airsim_mission_interfaces/CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... existing actions ...
  "action/YourNewAction.action"
  DEPENDENCIES std_msgs geometry_msgs airsim_interfaces
)
```

## 🟢 Phase 3: Implementation

### 3.1 Modify airsim_ros_wrapper.cpp

**For Topics:**
```cpp
// In airsim_ros_wrapper.hpp - Add publisher/subscriber declarations
rclcpp::Publisher<airsim_interfaces::msg::YourNewMessage>::SharedPtr your_new_publisher_;
rclcpp::Subscription<airsim_interfaces::msg::YourNewMessage>::SharedPtr your_new_subscriber_;

// In airsim_ros_wrapper.cpp - Initialize in constructor
your_new_publisher_ = this->create_publisher<airsim_interfaces::msg::YourNewMessage>(
    "your_new_topic", 10);
your_new_subscriber_ = this->create_subscription<airsim_interfaces::msg::YourNewMessage>(
    "your_new_topic_cmd", 10, 
    std::bind(&AirsimROSWrapper::your_new_callback, this, std::placeholders::_1));

// Add callback function
void AirsimROSWrapper::your_new_callback(const airsim_interfaces::msg::YourNewMessage::SharedPtr msg) {
    // Process the message
    // Call AirSim API through airsim_client_
}
```

**For Services:**
```cpp
// In airsim_ros_wrapper.hpp - Add service server declaration
rclcpp::Service<airsim_interfaces::srv::YourNewService>::SharedPtr your_new_service_;

// In airsim_ros_wrapper.cpp - Initialize in constructor
your_new_service_ = this->create_service<airsim_interfaces::srv::YourNewService>(
    "your_new_service",
    std::bind(&AirsimROSWrapper::your_new_service_callback, this, 
              std::placeholders::_1, std::placeholders::_2));

// Add service callback
void AirsimROSWrapper::your_new_service_callback(
    const std::shared_ptr<airsim_interfaces::srv::YourNewService::Request> request,
    std::shared_ptr<airsim_interfaces::srv::YourNewService::Response> response) {
    try {
        // Process request using airsim_client_
        response->success = true;
        response->message = "Success";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}
```

**For Actions:**
```cpp
// In airsim_ros_wrapper.hpp - Add action server declaration
rclcpp_action::Server<airsim_mission_interfaces::action::YourNewAction>::SharedPtr your_new_action_server_;

// In airsim_ros_wrapper.cpp - Initialize in constructor
your_new_action_server_ = rclcpp_action::create_server<airsim_mission_interfaces::action::YourNewAction>(
    this, "your_new_action",
    std::bind(&AirsimROSWrapper::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AirsimROSWrapper::handle_cancel, this, std::placeholders::_1),
    std::bind(&AirsimROSWrapper::handle_accepted, this, std::placeholders::_1));

// Add action callbacks (goal, cancel, accepted)
```

### 3.2 Add Multi-Vehicle Support

```cpp
// For multi-vehicle operations, use vehicle_name parameter
std::string vehicle_name = request->vehicle_name.empty() ? vehicle_name_ : request->vehicle_name;
```

## 🟠 Phase 4: Build Process

### 4.1 Build Interfaces
```bash
# In Docker container
build_interfaces
# OR
colcon build --packages-select airsim_interfaces airsim_mission_interfaces
```

### 4.2 Build Main Package
```bash
# In Docker container
build_pkgs
# OR
colcon build --packages-select airsim_ros_pkgs
```

### 4.3 Source Workspace
```bash
# In Docker container
source_ws
# OR
source install/setup.bash
```

## 🔴 Phase 5: Testing & Validation

### 5.1 Launch AirSim ROS2 Node
```bash
# In Docker container
./launch_airsim_ros2.sh
# OR
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

### 5.2 Test Interface

**For Topics:**
```bash
# List topics
ros2 topic list

# Echo topic
ros2 topic echo /airsim_node/drone_1/your_new_topic

# Publish to topic
ros2 topic pub /airsim_node/drone_1/your_new_topic_cmd airsim_interfaces/msg/YourNewMessage '{parameter1: 1.0, parameter2: "test"}'
```

**For Services:**
```bash
# List services
ros2 service list

# Call service
ros2 service call /airsim_node/drone_1/your_new_service airsim_interfaces/srv/YourNewService '{input_param: 1.0, vehicle_name: "drone_1"}'
```

**For Actions:**
```bash
# List actions
ros2 action list

# Send goal
ros2 action send_goal /airsim_node/drone_1/your_new_action airsim_mission_interfaces/action/YourNewAction '{target_position: {x: 0.0, y: 0.0, z: -10.0}, timeout: 30.0}'
```

## 🟣 Phase 6: Integration Testing

### 6.1 Multi-Vehicle Testing
```bash
# Test with multiple vehicles
ros2 service call /airsim_node/drone_1/your_new_service airsim_interfaces/srv/YourNewService '{...}'
ros2 service call /airsim_node/drone_2/your_new_service airsim_interfaces/srv/YourNewService '{...}'
```

### 6.2 Visualization
```bash
# Launch RViz2 (if applicable)
ros2 launch airsim_ros_pkgs airsim_node.launch.py enable_rviz:=true

# Use rqt tools
rqt_graph
rqt_topic
rqt_service_caller
```

## 🚀 Best Practices

### Code Organization
- Follow existing naming conventions
- Use appropriate namespaces for multi-vehicle support
- Add proper error handling and logging
- Include comprehensive documentation

### Performance Considerations
- Use appropriate QoS settings for publishers/subscribers
- Consider message frequency and size
- Implement proper threading for long-running operations

### Testing Strategy
- Test with single and multiple vehicles
- Verify error conditions and edge cases
- Test integration with existing functionality
- Validate performance under load

## 📁 Key Files Summary

```
ros2/src/
├── airsim_interfaces/
│   ├── msg/YourNewMessage.msg
│   ├── srv/YourNewService.srv
│   └── CMakeLists.txt
├── airsim_mission_interfaces/
│   ├── action/YourNewAction.action
│   └── CMakeLists.txt
└── airsim_ros_pkgs/
    ├── src/airsim_ros_wrapper.cpp
    ├── include/airsim_ros_pkgs/airsim_ros_wrapper.hpp
    └── launch/airsim_node.launch.py
```

## 🔗 Related Documentation

- [ROS2 Usage Guide](ros2/ROS2_USAGE_GUIDE.md)
- [Adding Services and Messages](ros2/README_Adding_Services_Messages.md)
- [Docker Development Workflow](docker_clean/README.md)

## 🐛 Troubleshooting

### Common Issues
1. **Build failures**: Check CMakeLists.txt syntax and dependencies
2. **Interface not found**: Ensure proper build order (interfaces first)
3. **Service not callable**: Verify service server initialization
4. **Action feedback issues**: Check action server implementation

### Debug Commands
```bash
# Check interface generation
ros2 interface show airsim_interfaces/msg/YourNewMessage

# Monitor node activity
ros2 node info /airsim_node

# Check parameter values
ros2 param list /airsim_node
```

This workflow ensures systematic development of ROS2 interfaces that integrate seamlessly with the Cosys-AirSim project.