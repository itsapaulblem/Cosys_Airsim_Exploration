# Mission Action Servers - ROS2 Architecture Documentation

This document provides a comprehensive technical explanation of how the mission action servers work in the AirSim ROS2 multi-vehicle architecture.

## Table of Contents

1. [ROS2 Action Server Architecture Overview](#ros2-action-server-architecture-overview)
2. [Mission Action Server Implementation](#mission-action-server-implementation)
3. [The Complete Action Lifecycle](#the-complete-action-lifecycle)
4. [Mission-Specific Actions](#mission-specific-actions)
5. [Common Issues and Solutions](#common-issues-and-solutions)
6. [Integration with AirSim](#integration-with-airsim)
7. [Best Practices and Design Patterns](#best-practices-and-design-patterns)
8. [Debugging Guide](#debugging-guide)

---

## ROS2 Action Server Architecture Overview

### What are ROS2 Actions?

ROS2 Actions are designed for **long-running, goal-oriented tasks** that need:
- **Feedback** during execution (progress updates)
- **Cancellation** capability (ability to abort mid-execution)
- **Result** reporting (final outcome and data)

This makes them perfect for autonomous vehicle missions, which are exactly this type of task.

### Why Actions vs Services vs Topics?

| Communication Type | Use Case | Example |
|-------------------|----------|---------|
| **Topics** | Continuous data streams | Sensor data, position updates |
| **Services** | Quick request-response | Get vehicle status, arm vehicle |
| **Actions** | Long-running tasks with feedback | Search mission, navigation, tracking |

### The Three-Phase Communication Model

```
Client                    Action Server
  |                            |
  |-------- Goal ------------>|   Phase 1: Goal Request
  |<------- Accept/Reject ----|
  |                            |
  |<------- Feedback ---------|   Phase 2: Execution (repeated)
  |<------- Feedback ---------|
  |<------- Feedback ---------|
  |                            |
  |<------- Result -----------|   Phase 3: Completion
```

---

## Mission Action Server Implementation

### Core Architecture Pattern

```cpp
class MissionMultirotorNode : public MultirotorNode
{
public:
    // Action type aliases for cleaner code
    using SearchAreaAction = mission_search_interfaces::action::SearchArea;
    using SearchAreaActionServer = rclcpp_action::Server<SearchAreaAction>;

private:
    // Action server instances
    std::shared_ptr<SearchAreaActionServer> search_area_action_server_;
    
    // Execution state management
    std::atomic<bool> mission_active_{false};
    std::atomic<bool> mission_cancellation_requested_{false};
    std::thread mission_execution_thread_;
};
```

### Action Server Creation Process

**1. Naming Convention:**
```cpp
void MissionMultirotorNode::setup_mission_action_servers()
{
    // Pattern: /VehicleName/actions/action_name
    std::string search_area_action = vehicle_name_ + "/actions/search_area";
    
    search_area_action_server_ = rclcpp_action::create_server<SearchAreaAction>(
        this,                                                    // Node pointer
        search_area_action,                                     // Action name
        std::bind(&MissionMultirotorNode::handle_search_area_goal, this, _1, _2),     // Goal callback
        std::bind(&MissionMultirotorNode::handle_search_area_cancel, this, _1),       // Cancel callback
        std::bind(&MissionMultirotorNode::handle_search_area_accepted, this, _1),     // Accepted callback
        rcl_action_server_get_default_options(),               // Default options
        mission_callback_group_);                               // Callback group for concurrency
}
```

**2. Callback Registration:**
- **Goal Callback**: Validates incoming goals, returns ACCEPT/REJECT
- **Cancel Callback**: Handles cancellation requests
- **Accepted Callback**: Starts execution in separate thread

### Thread Management Strategy

**Why Separate Threads?**
- Action execution runs in dedicated threads to prevent blocking the ROS2 event loop
- Main thread continues handling other callbacks (sensors, services, etc.)
- Cancellation can be handled asynchronously

```cpp
void MissionMultirotorNode::handle_search_area_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    // Clean up any previous mission thread
    if (mission_execution_thread_.joinable()) {
        mission_execution_thread_.join();
    }
    
    // Start new mission in separate thread
    mission_execution_thread_ = std::thread(&MissionMultirotorNode::execute_search_area, this, goal_handle);
}
```

---

## The Complete Action Lifecycle

### Phase 1: Goal Validation

```cpp
rclcpp_action::GoalResponse MissionMultirotorNode::handle_search_area_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SearchAreaAction::Goal> goal)
{
    // 1. Check if already busy
    if (mission_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal - mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 2. Validate goal parameters
    if (goal->search_boundary.points.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid search boundary - need at least 3 points");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 3. Smart default handling (accept 0.0 as "use default")
    if (goal->search_altitude != 0.0f && (goal->search_altitude < 5.0f || goal->search_altitude > 150.0f)) {
        RCLCPP_WARN(this->get_logger(), "Search altitude %f outside safe range (5-150m). Use 0.0 for default.", 
                    goal->search_altitude);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

**Key Validation Principles:**
- **Mutual Exclusion**: Only one mission at a time per vehicle
- **Safety Bounds**: Altitude, speed, and area constraints
- **Default Handling**: 0.0 values treated as "use system defaults"

### Phase 2: Execution with Feedback

```cpp
void MissionMultirotorNode::execute_search_area(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    // Initialize execution state
    mission_active_.store(true);
    mission_cancellation_requested_.store(false);
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SearchAreaAction::Feedback>();
    auto result = std::make_shared<SearchAreaAction::Result>();
    
    try {
        // Step 1: Apply default values
        float effective_altitude = goal->search_altitude > 0.0f ? goal->search_altitude : 25.0f;
        float effective_speed = goal->search_speed > 0.0f ? goal->search_speed : 5.0f;
        
        // Step 2: Ensure vehicle readiness
        if (!ensure_mission_ready(effective_altitude)) {
            result->success = false;
            result->completion_reason = "vehicle_preparation_failed";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        // Step 3: Generate waypoints
        auto waypoints = generate_search_waypoints(goal->search_boundary, 
                                                  goal->search_pattern,
                                                  effective_spacing, 
                                                  effective_altitude);
        
        // Step 4: Execute navigation with periodic feedback
        for (const auto& waypoint : waypoints) {
            if (mission_cancellation_requested_.load()) break;
            
            // Navigate to waypoint
            bool success = navigate_to_waypoint(waypoint, effective_speed);
            
            // Update feedback
            feedback->waypoints_completed++;
            feedback->progress_percentage = (float(feedback->waypoints_completed) / waypoints.size()) * 100.0f;
            goal_handle->publish_feedback(feedback);
        }
        
        // Step 5: Report final result
        result->success = !mission_cancellation_requested_.load();
        result->completion_reason = mission_cancellation_requested_.load() ? "cancelled" : "completed";
        goal_handle->succeed(result);
        
    } catch (const std::exception& e) {
        result->success = false;
        result->completion_reason = "exception";
        goal_handle->abort(result);
    }
    
    mission_active_.store(false);
}
```

**Execution Patterns:**
- **State Machine**: Clear phases with state transitions
- **Error Recovery**: Exception handling with meaningful error reporting
- **Cancellation Points**: Regular checks for cancellation requests
- **Progress Feedback**: Real-time updates to client

### Phase 3: Cancellation Handling

```cpp
rclcpp_action::CancelResponse MissionMultirotorNode::handle_search_area_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for search area goal");
    
    // Set atomic flag that execution thread will check
    mission_cancellation_requested_.store(true);
    
    return rclcpp_action::CancelResponse::ACCEPT;
}
```

**Graceful Cancellation:**
- Sets atomic flag that execution thread monitors
- Execution thread checks flag at safe cancellation points
- Current waypoint completed before stopping
- Vehicle left in safe, stable state

---

## Mission-Specific Actions

### 1. SearchArea Action

**Purpose:** Systematic area coverage using various search patterns

**Key Parameters:**
- `search_boundary` (geometry_msgs/Polygon): Area to search
- `search_pattern` (string): "spiral", "lawnmower", "grid", "random"
- `search_altitude` (float32): Flight altitude (0.0 = use default 25m)
- `search_speed` (float32): Flight speed (0.0 = use default 5 m/s)
- `pattern_spacing` (float32): Distance between search lines (0.0 = use default 15m)

**Execution Flow:**
1. **Validate** search area and parameters
2. **Prepare** vehicle (takeoff if needed)
3. **Generate** waypoint pattern based on algorithm
4. **Execute** waypoint sequence with progress feedback
5. **Report** coverage statistics and detections

**Search Pattern Algorithms:**

```cpp
// Spiral Pattern: Archimedes spiral from center outward
std::vector<geometry_msgs::msg::Point> generate_spiral_pattern(
    const geometry_msgs::msg::Polygon& boundary, float spacing, float altitude)
{
    auto center = get_polygon_center(boundary);
    float max_radius = calculate_max_distance_from_center(boundary, center);
    
    float a = spacing / (2.0f * M_PI);  // Spiral constant
    float angle = 0.0f;
    std::vector<geometry_msgs::msg::Point> waypoints;
    
    while (true) {
        float radius = a * angle;
        if (radius > max_radius) break;
        
        geometry_msgs::msg::Point waypoint;
        waypoint.x = center.x + radius * std::cos(angle);
        waypoint.y = center.y + radius * std::sin(angle);
        waypoint.z = -altitude;  // NED coordinates (negative Z is up)
        
        waypoints.push_back(waypoint);
        angle += 0.1f;  // Angular step
    }
    
    return waypoints;
}
```

### 2. NavigateToTarget Action

**Purpose:** Navigate to specific point with optional investigation

**Key Parameters:**
- `target_location` (geometry_msgs/Point): Destination coordinates
- `target_id` (string): Unique identifier for target
- `approach_altitude` (float32): Altitude for approach (0.0 = use default 25m)
- `navigation_speed` (float32): Flight speed (0.0 = use default 5 m/s)
- `approach_pattern` (string): "direct", "spiral", "orbit", "hover_above"
- `standoff_distance` (float32): Minimum distance from target
- `investigation_time` (Duration): Time to spend investigating

**Execution Phases:**
1. **Navigation Phase**: Move to target location
2. **Approach Phase**: Execute approach pattern (spiral, orbit, etc.)
3. **Investigation Phase**: Hover and collect data
4. **Documentation Phase**: Capture images/video if requested

### 3. TrackTarget Action

**Purpose:** Continuously follow and monitor moving target

**Key Parameters:**
- `target_id` (string): Target identifier
- `initial_target_location` (geometry_msgs/Point): Last known position
- `tracking_altitude` (float32): Preferred tracking altitude
- `tracking_distance` (float32): Preferred distance from target
- `tracking_mode` (string): "active", "passive", "predictive"
- `max_tracking_time` (Duration): Maximum tracking duration

**Execution Strategy:**
1. **Acquisition**: Navigate to initial target location
2. **Visual Lock**: Establish sensor contact with target
3. **Tracking Loop**: Maintain distance and visual contact
4. **Prediction**: Anticipate target movement patterns
5. **Documentation**: Record target behavior and movement

---

## Common Issues and Solutions

### Issue 1: `vehicle_preparation_failed` Error

**Symptoms:**
- Goal accepted but immediately aborts with "vehicle_preparation_failed"
- Mission never starts executing

**Root Cause:**
Missing utility functions in the codebase:
- `get_vehicle_state_with_retry()` - RPC retry logic
- `is_vehicle_ready_for_mission()` - Altitude/state validation

**Current Code Issue:**
```cpp
bool MissionMultirotorNode::ensure_mission_ready(float mission_altitude)
{
    // PROBLEM: These functions don't exist!
    auto state = get_vehicle_state_with_retry(multirotor_client, 3);  // Missing function
    if (is_vehicle_ready_for_mission(current_altitude, mission_altitude)) {  // Missing function
        return true;
    }
    // ...
}
```

**Solution:**
Implement missing utility functions:

```cpp
// Add to mission_multirotor_node.hpp
std::optional<msr::airlib::MultirotorState> get_vehicle_state_with_retry(
    msr::airlib::MultirotorRpcLibClient* client, int max_retries);
bool is_vehicle_ready_for_mission(float current_altitude, float mission_altitude);

// Implementation in mission_multirotor_node.cpp
std::optional<msr::airlib::MultirotorState> MissionMultirotorNode::get_vehicle_state_with_retry(
    msr::airlib::MultirotorRpcLibClient* client, int max_retries)
{
    for (int attempt = 0; attempt < max_retries; ++attempt) {
        try {
            auto state = client->getMultirotorState(vehicle_name_);
            return state;  // Success
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "RPC attempt %d failed: %s", attempt + 1, e.what());
            if (attempt < max_retries - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }
    return std::nullopt;  // All attempts failed
}

bool MissionMultirotorNode::is_vehicle_ready_for_mission(float current_altitude, float mission_altitude)
{
    const float MIN_FLIGHT_ALTITUDE = 3.0f;
    const float ALTITUDE_TOLERANCE = 5.0f;
    
    // Check if already airborne
    bool is_airborne = current_altitude >= MIN_FLIGHT_ALTITUDE;
    
    // Check if at appropriate altitude
    bool altitude_ok = std::abs(current_altitude - mission_altitude) <= ALTITUDE_TOLERANCE;
    
    return is_airborne && altitude_ok;
}
```

### Issue 2: Goal Rejection with Default Values

**Symptoms:**
- Goals rejected when using 0.0 for parameters
- Error: "outside safe range"

**Solution:**
Already implemented in latest code - 0.0 values are treated as "use defaults"

### Issue 3: Slow Mission Status Updates

**Symptoms:**
- `ros2 topic echo /VehicleName/mission/status` very slow
- Timeouts when getting vehicle state

**Root Cause:**
Synchronous RPC calls blocking status publication

**Solution:**
Implement asynchronous status publishing with cached state

---

## Integration with AirSim

### Coordinate System Conversion

**AirSim uses NED (North-East-Down) coordinates:**
- **X**: North (positive = forward)
- **Y**: East (positive = right) 
- **Z**: Down (positive = down, **negative = up**)

**ROS2 typically uses ENU (East-North-Up):**
- **X**: East
- **Y**: North
- **Z**: Up

**Key Conversion Points:**
```cpp
// Setting altitude in AirSim (negative for "up")
waypoint.z = -altitude;  

// Reading altitude from AirSim (absolute value for height above ground)
float current_altitude = std::abs(pos.z());
```

### RPC Communication Pattern

```cpp
// Always use try-catch for RPC calls
try {
    auto multirotor_client = dynamic_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
    if (!multirotor_client) {
        throw std::runtime_error("Failed to cast to MultirotorRpcLibClient");
    }
    
    // Execute RPC command
    auto future = multirotor_client->moveToPositionAsync(
        waypoint.x, waypoint.y, waypoint.z, speed, vehicle_name_);
    
    // Poll for completion with timeout
    while (!future.is_ready() && !mission_cancellation_requested_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
} catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "AirSim RPC error: %s", e.what());
    return false;
}
```

---

## Best Practices and Design Patterns

### 1. Naming Conventions

**Pattern:** `/VehicleName/category/specific_name`

```cpp
// Action servers
std::string search_area_action = vehicle_name_ + "/actions/search_area";
std::string navigate_action = vehicle_name_ + "/actions/navigate_to_target";

// Services  
std::string get_capabilities_service = vehicle_name_ + "/services/get_capabilities";

// Topics
std::string mission_status_topic = vehicle_name_ + "/mission/status";
std::string detection_topic = vehicle_name_ + "/detections/target";
```

**Benefits:**
- Clear namespace separation
- Easy to discover services/actions per vehicle
- No naming conflicts in multi-vehicle scenarios

### 2. Atomic State Management

```cpp
class MissionMultirotorNode {
private:
    // Thread-safe state variables
    std::atomic<bool> mission_active_{false};
    std::atomic<bool> mission_cancellation_requested_{false};
    
    // Mutex for complex state
    std::mutex mission_state_mutex_;
    
    // Progress tracking (updated by execution thread, read by feedback)
    std::atomic<float> current_mission_progress_{0.0f};
};
```

**Why Atomic Variables?**
- Thread-safe read/write between main thread and execution thread
- No need for mutexes for simple boolean/numeric values
- Lockless performance for frequently accessed variables

### 3. Callback Groups for Concurrency

```cpp
// Constructor
MissionMultirotorNode::MissionMultirotorNode(const std::string& vehicle_name, ...)
    : MultirotorNode(vehicle_name, host_ip, host_port)
{
    // Create separate callback group for mission operations
    mission_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // This allows mission actions to run concurrently with other callbacks
}
```

### 4. Default Value Handling Pattern

```cpp
// In goal validation
float effective_altitude = goal->search_altitude > 0.0f ? goal->search_altitude : 25.0f;
float effective_speed = goal->search_speed > 0.0f ? goal->search_speed : 5.0f;

// Log what's actually being used
RCLCPP_INFO(this->get_logger(), "Using effective values - altitude: %.1fm%s, speed: %.1fm/s%s", 
            effective_altitude, (goal->search_altitude == 0.0f ? " (default)" : ""),
            effective_speed, (goal->search_speed == 0.0f ? " (default)" : ""));
```

### 5. Error Recovery Strategies

**Graceful Degradation:**
```cpp
try {
    // Attempt primary mission logic
    bool success = execute_primary_mission();
    if (!success) {
        // Try fallback approach
        success = execute_fallback_mission();
    }
    return success;
} catch (const std::exception& e) {
    // Log error and return safe state
    RCLCPP_ERROR(this->get_logger(), "Mission failed: %s", e.what());
    return_to_safe_state();
    return false;
}
```

---

## Debugging Guide

### 1. Check Action Server Status

```bash
# List all available actions
ros2 action list

# Check specific action server
ros2 action info /PX4_Drone1/actions/search_area

# Expected output:
# Action: /PX4_Drone1/actions/search_area
# Action clients: 0
# Action servers: 1
```

### 2. Monitor Action Execution

```bash
# Send goal with feedback monitoring
ros2 action send_goal /PX4_Drone1/actions/search_area \
mission_search_interfaces/action/SearchArea \
"{
  search_boundary: {
    points: [{x: 0.0, y: 0.0, z: 0.0}, {x: 30.0, y: 0.0, z: 0.0}, {x: 30.0, y: 30.0, z: 0.0}, {x: 0.0, y: 30.0, z: 0.0}]
  },
  search_pattern: 'spiral',
  search_altitude: 25.0
}" --feedback
```

### 3. Check Mission Status

```bash
# Monitor mission status updates
ros2 topic echo /PX4_Drone1/mission/status

# Check current vehicle state
ros2 topic echo /PX4_Drone1/odom_local_ned --once
```

### 4. Node Introspection

```bash
# Check node status
ros2 node info /PX4_Drone1

# Expected mission-related interfaces:
# Action Servers:
#   /PX4_Drone1/actions/search_area: mission_search_interfaces/action/SearchArea
#   /PX4_Drone1/actions/navigate_to_target: mission_search_interfaces/action/NavigateToTarget
#   /PX4_Drone1/actions/track_target: mission_search_interfaces/action/TrackTarget
# Publishers:
#   /PX4_Drone1/mission/status: mission_search_interfaces/msg/MissionStatus
```

### 5. Common Error Messages and Solutions

| Error Message | Cause | Solution |
|---------------|-------|----------|
| "Waiting for action server to become available..." | Action server not running or wrong name | Check `ros2 action list`, verify node is running |
| "Goal was rejected" | Validation failed | Check parameter values, use defaults (0.0) |
| "vehicle_preparation_failed" | Missing utility functions | Implement `get_vehicle_state_with_retry` and `is_vehicle_ready_for_mission` |
| "RPC timeout" | AirSim connection issue | Check AirSim is running, network connectivity |
| "Goal finished with status: ABORTED" | Exception during execution | Check logs for specific error |

### 6. Log Analysis

**Important log patterns to watch for:**
```bash
# Goal acceptance
[INFO] [PX4_Drone1]: Accepting search area goal - altitude: 25.0fm, speed: 5.0fm/s

# Mission state changes  
[INFO] [PX4_Drone1]: Search area goal accepted - starting execution
[INFO] [PX4_Drone1]: Using effective values - altitude: 25.0fm, speed: 5.0fm/s, spacing: 15.0fm

# Vehicle preparation
[INFO] [PX4_Drone1]: Ensuring vehicle ready for mission at altitude 25.0fm
[INFO] [PX4_Drone1]: Current vehicle position: (0.0, 0.0, -25.0), altitude: 25.0fm

# Mission progress
[INFO] [PX4_Drone1]: Generated 15 waypoints for spiral pattern
[INFO] [PX4_Drone1]: Search area mission completed successfully
```

**Error patterns:**
```bash
# RPC issues
[ERROR] [PX4_Drone1]: Failed to cast to MultirotorRpcLibClient - AirSim connection issue

# Missing functions
[ERROR] [PX4_Drone1]: Failed to prepare vehicle for mission

# Validation failures
[WARN] [PX4_Drone1]: Search altitude 200.0 outside safe range (5-150m). Use 0.0 for default.
```

---

## Conclusion

The mission action server architecture provides a robust, scalable foundation for autonomous vehicle operations. Key strengths include:

1. **Thread Safety**: Atomic state management prevents race conditions
2. **Graceful Cancellation**: Safe mission termination at any point
3. **Real-time Feedback**: Progress monitoring and status updates
4. **Error Recovery**: Comprehensive exception handling
5. **Modularity**: Each action type handles specific mission requirements
6. **Integration**: Seamless AirSim communication with retry logic

The current `vehicle_preparation_failed` issue can be resolved by implementing the missing utility functions highlighted in this document. Once those are added, the system will provide reliable mission execution capabilities for multi-vehicle autonomous operations.

For development and testing, always start with simple goals using default parameters (0.0 values) and gradually add complexity as the system proves stable.