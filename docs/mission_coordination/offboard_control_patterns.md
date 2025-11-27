# 🎮 Offboard Control Patterns for Mission Execution
## PX4 SITL Offboard Control with Mission Coordination in Cosys-AirSim

This guide covers advanced offboard control patterns for executing autonomous missions with PX4 SITL, focusing on real-time waypoint navigation, dynamic mission replanning, and emergency abort procedures.

## 📋 Prerequisites

- **PX4 SITL Integration** (see `px4_sitl_mission_integration.md`)
- **Mission Coordination System** implemented
- **MAVROS** configured for offboard control
- **Ultra-Clean ROS2 Architecture** understanding

## 🎯 Offboard Control Architecture

### Control Flow Overview
```
Mission Coordinator
├── 📋 Mission Planning
├── 🎯 Waypoint Generation  
├── 🚁 Vehicle Assignment
└── 📡 Real-Time Control
    ├── Position Setpoints (50Hz)
    ├── Velocity Commands (50Hz)
    ├── Attitude Control (100Hz)
    └── Emergency Override (Immediate)
```

### PX4 Flight Mode Integration
```
PX4 Flight Modes for Missions:
├── OFFBOARD → Direct ROS2 control
├── AUTO.MISSION → PX4 waypoint following
├── AUTO.RTL → Return-to-launch
└── AUTO.LAND → Emergency landing
```

## 🛠️ 1. Core Offboard Control Implementation

### 1.1 Mission-Aware Offboard Controller

```cpp
// mission_offboard_controller.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mission_search_interfaces/action/navigate_to_target.hpp>
#include <mission_search_interfaces/msg/mission_status.hpp>

class MissionOffboardController : public rclcpp::Node
{
public:
    MissionOffboardController(const std::string& vehicle_name);
    
    // Core control methods
    bool enableOffboardMode();
    bool armVehicle();
    bool setPosition(const geometry_msgs::msg::Point& position, double yaw = 0.0);
    bool setVelocity(const geometry_msgs::msg::Vector3& velocity, double yaw_rate = 0.0);
    bool landAtCurrentPosition();
    bool returnToLaunch();
    
    // Mission-specific control
    bool executeWaypointSequence(const std::vector<geometry_msgs::msg::Point>& waypoints);
    bool abortMission();
    bool pauseMission();
    bool resumeMission();
    
    // Safety and monitoring
    bool isVehicleReady() const;
    double getDistanceToTarget() const;
    geometry_msgs::msg::Point getCurrentPosition() const;

private:
    void setupOffboardInterface();
    void setupMissionInterface();
    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();
    
    // Control state management
    void updateControlState();
    bool waitForPositionReached(const geometry_msgs::msg::Point& target, 
                               double tolerance = 1.0, 
                               double timeout = 30.0);
    
    // Safety checks
    bool performPreflightChecks();
    bool isPositionValid(const geometry_msgs::msg::Point& position);
    bool isWithinOperationalLimits();
    
    // Emergency procedures
    void handleEmergencyStop();
    void handleConnectionLoss();
    void handleLowBattery();

private:
    std::string vehicle_name_;
    
    // MAVROS interfaces
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    
    // Mission interfaces
    rclcpp::Publisher<mission_search_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;
    rclcpp_action::Server<mission_search_interfaces::action::NavigateToTarget>::SharedPtr navigate_action_server_;
    
    // Control state
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Mission state
    enum class MissionState {
        IDLE,
        PREFLIGHT,
        ARMED,
        TAKEOFF,
        MISSION_ACTIVE,
        RETURNING,
        LANDING,
        EMERGENCY
    };
    
    MissionState mission_state_;
    std::vector<geometry_msgs::msg::Point> current_waypoints_;
    size_t current_waypoint_index_;
    geometry_msgs::msg::Point target_position_;
    
    // Configuration
    double control_frequency_;
    double position_tolerance_;
    double max_horizontal_speed_;
    double max_vertical_speed_;
    double max_altitude_;
    double home_altitude_;
};

// Implementation
MissionOffboardController::MissionOffboardController(const std::string& vehicle_name)
    : Node(vehicle_name + "_offboard_controller")
    , vehicle_name_(vehicle_name)
    , mission_state_(MissionState::IDLE)
    , current_waypoint_index_(0)
    , control_frequency_(50.0)  // 50Hz control loop
    , position_tolerance_(1.0)  // 1 meter tolerance
    , max_horizontal_speed_(10.0)  // 10 m/s max horizontal
    , max_vertical_speed_(3.0)     // 3 m/s max vertical
    , max_altitude_(120.0)         // 120m max altitude
    , home_altitude_(-10.0)        // -10m home altitude (NED)
{
    setupOffboardInterface();
    setupMissionInterface();
    
    RCLCPP_INFO(this->get_logger(), 
        "Mission offboard controller ready for %s", vehicle_name_.c_str());
}

void MissionOffboardController::setupOffboardInterface()
{
    // MAVROS state subscriber
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        std::bind(&MissionOffboardController::mavrosStateCallback, this, std::placeholders::_1)
    );
    
    // Local position subscriber
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", 10,
        std::bind(&MissionOffboardController::localPositionCallback, this, std::placeholders::_1)
    );
    
    // Position target publisher (high frequency)
    rclcpp::QoS setpoint_qos(10);
    setpoint_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    setpoint_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", setpoint_qos
    );
    
    // Service clients
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming"
    );
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode"
    );
    
    // Control timer (50Hz)
    control_timer_ = this->create_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
        std::bind(&MissionOffboardController::controlLoop, this)
    );
}

void MissionOffboardController::setupMissionInterface()
{
    // Mission status publisher
    mission_status_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionStatus>(
        "/" + vehicle_name_ + "/mission_status", 10
    );
    
    // Navigation action server
    navigate_action_server_ = rclcpp_action::create_server<mission_search_interfaces::action::NavigateToTarget>(
        this,
        "/" + vehicle_name_ + "/actions/navigate_to_target",
        [this](const rclcpp_action::GoalUUID& uuid, 
                std::shared_ptr<const mission_search_interfaces::action::NavigateToTarget::Goal> goal) {
            return this->handleNavigationGoal(uuid, goal);
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::NavigateToTarget>> goal_handle) {
            return this->handleNavigationCancel(goal_handle);
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::NavigateToTarget>> goal_handle) {
            this->executeNavigation(goal_handle);
        }
    );
}

bool MissionOffboardController::enableOffboardMode()
{
    // Wait for connection
    while (rclcpp::ok() && !current_state_.connected) {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Send some setpoints before switching to offboard
    mavros_msgs::msg::PositionTarget setpoint;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.header.frame_id = "map";
    setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                        mavros_msgs::msg::PositionTarget::IGNORE_VY |
                        mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    
    // Set initial position (current position)
    setpoint.position.x = current_pose_.pose.position.x;
    setpoint.position.y = current_pose_.pose.position.y;
    setpoint.position.z = current_pose_.pose.position.z;
    setpoint.yaw = 0.0;
    
    // Send setpoints for 2 seconds
    auto start_time = this->get_clock()->now();
    while (rclcpp::ok() && (this->get_clock()->now() - start_time).seconds() < 2.0) {
        setpoint.header.stamp = this->get_clock()->now();
        setpoint_pub_->publish(setpoint);
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // Request offboard mode
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";
    
    auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_mode_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = set_mode_future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Offboard mode enabled");
            mission_state_ = MissionState::ARMED;
            return true;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to enable offboard mode");
    return false;
}

bool MissionOffboardController::armVehicle()
{
    if (!isVehicleReady()) {
        RCLCPP_ERROR(this->get_logger(), "Vehicle not ready for arming");
        return false;
    }
    
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;
    
    auto arm_future = arming_client_->async_send_request(arm_request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = arm_future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
            return true;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle");
    return false;
}

bool MissionOffboardController::setPosition(const geometry_msgs::msg::Point& position, double yaw)
{
    if (!isPositionValid(position)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid position command");
        return false;
    }
    
    target_position_ = position;
    
    mavros_msgs::msg::PositionTarget setpoint;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.header.frame_id = "map";
    setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                        mavros_msgs::msg::PositionTarget::IGNORE_VY |
                        mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    
    setpoint.position.x = position.x;
    setpoint.position.y = position.y;
    setpoint.position.z = position.z;
    setpoint.yaw = yaw;
    
    setpoint_pub_->publish(setpoint);
    
    RCLCPP_INFO(this->get_logger(), 
        "Position command sent: (%.2f, %.2f, %.2f) yaw=%.2f",
        position.x, position.y, position.z, yaw);
    
    return true;
}

bool MissionOffboardController::executeWaypointSequence(const std::vector<geometry_msgs::msg::Point>& waypoints)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty waypoint sequence");
        return false;
    }
    
    current_waypoints_ = waypoints;
    current_waypoint_index_ = 0;
    mission_state_ = MissionState::MISSION_ACTIVE;
    
    RCLCPP_INFO(this->get_logger(), 
        "Starting waypoint sequence with %zu waypoints", waypoints.size());
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (mission_state_ != MissionState::MISSION_ACTIVE) {
            RCLCPP_WARN(this->get_logger(), "Mission aborted at waypoint %zu", i);
            return false;
        }
        
        const auto& waypoint = waypoints[i];
        current_waypoint_index_ = i;
        
        RCLCPP_INFO(this->get_logger(), 
            "Navigating to waypoint %zu/%zu: (%.2f, %.2f, %.2f)",
            i + 1, waypoints.size(), waypoint.x, waypoint.y, waypoint.z);
        
        // Set position and wait for arrival
        if (!setPosition(waypoint)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set waypoint %zu", i);
            return false;
        }
        
        if (!waitForPositionReached(waypoint, position_tolerance_, 30.0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint %zu", i);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", i + 1);
    }
    
    mission_state_ = MissionState::IDLE;
    RCLCPP_INFO(this->get_logger(), "Waypoint sequence completed successfully");
    return true;
}

void MissionOffboardController::controlLoop()
{
    updateControlState();
    
    // Publish current setpoint to maintain offboard mode
    if (mission_state_ == MissionState::MISSION_ACTIVE || 
        mission_state_ == MissionState::ARMED) {
        
        mavros_msgs::msg::PositionTarget setpoint;
        setpoint.header.stamp = this->get_clock()->now();
        setpoint.header.frame_id = "map";
        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                            mavros_msgs::msg::PositionTarget::IGNORE_VY |
                            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        
        setpoint.position.x = target_position_.x;
        setpoint.position.y = target_position_.y;
        setpoint.position.z = target_position_.z;
        setpoint.yaw = 0.0;
        
        setpoint_pub_->publish(setpoint);
    }
    
    // Publish mission status
    mission_search_interfaces::msg::MissionStatus status;
    status.header.stamp = this->get_clock()->now();
    status.vehicle_name = vehicle_name_;
    status.status = static_cast<int>(mission_state_);
    status.current_waypoint = static_cast<int>(current_waypoint_index_);
    status.total_waypoints = static_cast<int>(current_waypoints_.size());
    
    mission_status_pub_->publish(status);
}

bool MissionOffboardController::waitForPositionReached(const geometry_msgs::msg::Point& target, 
                                                      double tolerance, 
                                                      double timeout)
{
    auto start_time = this->get_clock()->now();
    
    while (rclcpp::ok()) {
        // Check timeout
        auto elapsed = this->get_clock()->now() - start_time;
        if (elapsed.seconds() > timeout) {
            RCLCPP_WARN(this->get_logger(), "Position reached timeout");
            return false;
        }
        
        // Check if position reached
        double distance = std::sqrt(
            std::pow(target.x - current_pose_.pose.position.x, 2) +
            std::pow(target.y - current_pose_.pose.position.y, 2) +
            std::pow(target.z - current_pose_.pose.position.z, 2)
        );
        
        if (distance < tolerance) {
            return true;
        }
        
        // Continue sending setpoints
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    return false;
}

bool MissionOffboardController::isVehicleReady() const
{
    return current_state_.connected && 
           current_state_.mode == "OFFBOARD" &&
           !current_waypoints_.empty();
}
```

### 1.2 Advanced Velocity Control for Dynamic Missions

```cpp
// velocity_controller.hpp
class VelocityMissionController : public MissionOffboardController
{
public:
    VelocityMissionController(const std::string& vehicle_name)
        : MissionOffboardController(vehicle_name)
    {
        setupVelocityControl();
    }
    
    // Velocity-based mission execution
    bool executeVelocityMission(const std::vector<geometry_msgs::msg::Vector3>& velocity_sequence,
                               const std::vector<double>& durations);
    
    bool followPath(const std::vector<geometry_msgs::msg::Point>& path, double speed = 5.0);
    bool orbitPosition(const geometry_msgs::msg::Point& center, double radius, double speed = 3.0);
    bool performSearch8Pattern(const geometry_msgs::msg::Point& center, double width, double height);

private:
    void setupVelocityControl();
    bool setVelocityCommand(const geometry_msgs::msg::Vector3& velocity, double yaw_rate = 0.0);
    
    // Path following algorithms
    geometry_msgs::msg::Vector3 calculatePathFollowingVelocity(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& target_pos,
        double desired_speed);
    
    geometry_msgs::msg::Vector3 calculateOrbitVelocity(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& center,
        double radius,
        double angular_speed);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    double max_velocity_xy_;
    double max_velocity_z_;
    double path_following_gain_;
};

bool VelocityMissionController::executeVelocityMission(
    const std::vector<geometry_msgs::msg::Vector3>& velocity_sequence,
    const std::vector<double>& durations)
{
    if (velocity_sequence.size() != durations.size()) {
        RCLCPP_ERROR(this->get_logger(), "Velocity sequence and durations size mismatch");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Starting velocity mission with %zu segments", velocity_sequence.size());
    
    for (size_t i = 0; i < velocity_sequence.size(); ++i) {
        const auto& velocity = velocity_sequence[i];
        double duration = durations[i];
        
        RCLCPP_INFO(this->get_logger(), 
            "Segment %zu: velocity(%.2f, %.2f, %.2f) for %.2fs",
            i + 1, velocity.x, velocity.y, velocity.z, duration);
        
        auto start_time = this->get_clock()->now();
        
        while ((this->get_clock()->now() - start_time).seconds() < duration) {
            if (!setVelocityCommand(velocity)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send velocity command");
                return false;
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    
    // Stop at end of mission
    geometry_msgs::msg::Vector3 zero_velocity;
    setVelocityCommand(zero_velocity);
    
    RCLCPP_INFO(this->get_logger(), "Velocity mission completed");
    return true;
}

bool VelocityMissionController::followPath(const std::vector<geometry_msgs::msg::Point>& path, double speed)
{
    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty path");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Following path with %zu points at %.2f m/s", path.size(), speed);
    
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& target = path[i];
        
        while (rclcpp::ok()) {
            auto current_pos = getCurrentPosition();
            
            // Calculate velocity command toward target
            auto velocity_cmd = calculatePathFollowingVelocity(current_pos, target, speed);
            
            if (!setVelocityCommand(velocity_cmd)) {
                return false;
            }
            
            // Check if close enough to target
            double distance = std::sqrt(
                std::pow(target.x - current_pos.x, 2) +
                std::pow(target.y - current_pos.y, 2) +
                std::pow(target.z - current_pos.z, 2)
            );
            
            if (distance < position_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "Reached path point %zu", i + 1);
                break;
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    
    // Stop at end
    geometry_msgs::msg::Vector3 zero_velocity;
    setVelocityCommand(zero_velocity);
    
    return true;
}

geometry_msgs::msg::Vector3 VelocityMissionController::calculatePathFollowingVelocity(
    const geometry_msgs::msg::Point& current_pos,
    const geometry_msgs::msg::Point& target_pos,
    double desired_speed)
{
    // Calculate direction vector
    geometry_msgs::msg::Vector3 direction;
    direction.x = target_pos.x - current_pos.x;
    direction.y = target_pos.y - current_pos.y;
    direction.z = target_pos.z - current_pos.z;
    
    // Calculate distance
    double distance = std::sqrt(
        direction.x * direction.x +
        direction.y * direction.y +
        direction.z * direction.z
    );
    
    // Normalize and scale by desired speed
    geometry_msgs::msg::Vector3 velocity;
    if (distance > 0.01) {  // Avoid division by zero
        double scale = std::min(desired_speed, distance * path_following_gain_) / distance;
        velocity.x = direction.x * scale;
        velocity.y = direction.y * scale;
        velocity.z = direction.z * scale;
        
        // Apply velocity limits
        velocity.x = std::clamp(velocity.x, -max_velocity_xy_, max_velocity_xy_);
        velocity.y = std::clamp(velocity.y, -max_velocity_xy_, max_velocity_xy_);
        velocity.z = std::clamp(velocity.z, -max_velocity_z_, max_velocity_z_);
    }
    
    return velocity;
}
```

## 🚨 2. Failsafe and Emergency Procedures

### 2.1 Mission Abort and Emergency Landing

```cpp
// emergency_procedures.hpp
class EmergencyMissionHandler : public rclcpp::Node
{
public:
    EmergencyMissionHandler(const std::string& vehicle_name);
    
    // Emergency procedures
    bool executeEmergencyLanding();
    bool executeReturnToLaunch();
    bool executeEmergencyAbort();
    bool holdPosition();
    
    // Mission safety monitoring
    void enableSafetyMonitoring();
    void disableSafetyMonitoring();
    
    // Emergency triggers
    void triggerLowBatteryProcedure();
    void triggerConnectionLossProcedure();
    void triggerGeofenceViolationProcedure();
    void triggerObstacleAvoidanceProcedure();

private:
    void setupEmergencyInterface();
    void safetyMonitoringLoop();
    void checkBatteryLevel();
    void checkConnectionStatus();
    void checkGeofenceBoundaries();
    void checkSystemHealth();
    
    bool sendRTLCommand();
    bool sendLandCommand();
    bool sendHoldCommand();

private:
    std::string vehicle_name_;
    
    // Emergency state
    enum class EmergencyState {
        NORMAL,
        LOW_BATTERY,
        CONNECTION_LOST,
        GEOFENCE_VIOLATION,
        SYSTEM_FAILURE,
        MANUAL_ABORT
    };
    
    EmergencyState emergency_state_;
    
    // MAVROS interfaces for emergency
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr emergency_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr emergency_command_client_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    
    // Safety parameters
    double critical_battery_level_;
    double warning_battery_level_;
    double connection_timeout_;
    geometry_msgs::msg::Polygon geofence_boundary_;
    double max_operational_altitude_;
    
    // Monitoring
    rclcpp::TimerBase::SharedPtr safety_timer_;
    bool safety_monitoring_enabled_;
    rclcpp::Time last_heartbeat_;
};

bool EmergencyMissionHandler::executeEmergencyLanding()
{
    RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY LANDING INITIATED");
    
    emergency_state_ = EmergencyState::MANUAL_ABORT;
    
    // Send LAND mode command to PX4
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "AUTO.LAND";
    
    auto mode_future = emergency_mode_client_->async_send_request(set_mode_request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = mode_future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "✅ Emergency landing mode activated");
            
            // Monitor landing progress
            return monitorEmergencyLanding();
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to activate emergency landing");
    return false;
}

bool EmergencyMissionHandler::executeReturnToLaunch()
{
    RCLCPP_WARN(this->get_logger(), "🔄 RETURN TO LAUNCH INITIATED");
    
    return sendRTLCommand();
}

bool EmergencyMissionHandler::sendRTLCommand()
{
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "AUTO.RTL";
    
    auto mode_future = emergency_mode_client_->async_send_request(set_mode_request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = mode_future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "✅ Return to launch mode activated");
            return true;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to activate return to launch");
    return false;
}

void EmergencyMissionHandler::safetyMonitoringLoop()
{
    if (!safety_monitoring_enabled_) {
        return;
    }
    
    checkBatteryLevel();
    checkConnectionStatus();
    checkGeofenceBoundaries();
    checkSystemHealth();
    
    // Take action based on emergency state
    switch (emergency_state_) {
        case EmergencyState::LOW_BATTERY:
            triggerLowBatteryProcedure();
            break;
            
        case EmergencyState::CONNECTION_LOST:
            triggerConnectionLossProcedure();
            break;
            
        case EmergencyState::GEOFENCE_VIOLATION:
            triggerGeofenceViolationProcedure();
            break;
            
        case EmergencyState::SYSTEM_FAILURE:
            executeEmergencyLanding();
            break;
            
        default:
            break;
    }
}

void EmergencyMissionHandler::triggerLowBatteryProcedure()
{
    static bool low_battery_rtl_sent = false;
    
    if (!low_battery_rtl_sent) {
        RCLCPP_WARN(this->get_logger(), 
            "🔋 Low battery detected - initiating return to launch");
        
        if (executeReturnToLaunch()) {
            low_battery_rtl_sent = true;
        }
    }
}

void EmergencyMissionHandler::triggerConnectionLossProcedure()
{
    RCLCPP_ERROR(this->get_logger(), 
        "📡 Connection lost - PX4 failsafe should activate automatically");
    
    // PX4 will automatically handle connection loss based on COM_OF_LOSS_T parameter
    // Additional custom handling can be added here
}
```

### 2.2 Real-Time Mission Replanning

```cpp
// mission_replanner.hpp
class RealTimeMissionReplanner : public rclcpp::Node
{
public:
    RealTimeMissionReplanner(const std::string& vehicle_name);
    
    // Mission replanning triggers
    bool replanForObstacle(const geometry_msgs::msg::Point& obstacle_position, double obstacle_radius);
    bool replanForWeather(const std::string& weather_condition);
    bool replanForBatteryLevel(double remaining_battery_percent);
    bool replanForNewTarget(const geometry_msgs::msg::Point& new_target_position);
    
    // Dynamic mission modification
    bool insertWaypoint(size_t index, const geometry_msgs::msg::Point& waypoint);
    bool removeWaypoint(size_t index);
    bool modifyWaypoint(size_t index, const geometry_msgs::msg::Point& new_position);
    bool optimizeMissionPath();

private:
    void setupReplanningInterface();
    std::vector<geometry_msgs::msg::Point> generateAlternatePath(
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal,
        const std::vector<geometry_msgs::msg::Point>& obstacles);
    
    double calculatePathLength(const std::vector<geometry_msgs::msg::Point>& path);
    bool isPathClear(const std::vector<geometry_msgs::msg::Point>& path);
    
    // A* pathfinding for obstacle avoidance
    std::vector<geometry_msgs::msg::Point> aStarPathfinding(
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal,
        const std::vector<geometry_msgs::msg::Point>& obstacles);

private:
    std::string vehicle_name_;
    std::vector<geometry_msgs::msg::Point> current_mission_path_;
    std::vector<geometry_msgs::msg::Point> known_obstacles_;
    
    // Mission replanning publishers
    rclcpp::Publisher<mission_search_interfaces::msg::WaypointArray>::SharedPtr replan_pub_;
    rclcpp::Subscription<mission_search_interfaces::msg::TargetDetection>::SharedPtr obstacle_sub_;
    
    // Replanning parameters
    double replanning_horizon_;  // How far ahead to replan
    double obstacle_avoidance_margin_;  // Safety margin around obstacles
    double max_deviation_distance_;  // Max allowed deviation from original path
};

bool RealTimeMissionReplanner::replanForObstacle(const geometry_msgs::msg::Point& obstacle_position, 
                                                double obstacle_radius)
{
    RCLCPP_WARN(this->get_logger(), 
        "🚧 Obstacle detected at (%.2f, %.2f, %.2f) - replanning mission",
        obstacle_position.x, obstacle_position.y, obstacle_position.z);
    
    // Add obstacle to known obstacles
    known_obstacles_.push_back(obstacle_position);
    
    // Get current position
    auto current_position = getCurrentPosition();
    
    // Find next waypoint that's affected by obstacle
    size_t affected_waypoint_index = findFirstAffectedWaypoint(obstacle_position, obstacle_radius);
    
    if (affected_waypoint_index >= current_mission_path_.size()) {
        RCLCPP_INFO(this->get_logger(), "Obstacle doesn't affect remaining mission path");
        return true;
    }
    
    // Generate alternate path around obstacle
    std::vector<geometry_msgs::msg::Point> alternate_path;
    
    // Path from current position to first unaffected waypoint
    auto goal_position = current_mission_path_[affected_waypoint_index];
    alternate_path = generateAlternatePath(current_position, goal_position, known_obstacles_);
    
    if (alternate_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate alternate path around obstacle");
        return false;
    }
    
    // Replace affected portion of mission
    std::vector<geometry_msgs::msg::Point> new_mission_path;
    
    // Add alternate path
    new_mission_path.insert(new_mission_path.end(), alternate_path.begin(), alternate_path.end());
    
    // Add remaining original waypoints
    new_mission_path.insert(new_mission_path.end(), 
                           current_mission_path_.begin() + affected_waypoint_index + 1,
                           current_mission_path_.end());
    
    // Update mission
    current_mission_path_ = new_mission_path;
    
    // Publish replanned mission
    publishReplanedMission(new_mission_path);
    
    RCLCPP_INFO(this->get_logger(), 
        "✅ Mission replanned around obstacle - new path has %zu waypoints",
        new_mission_path.size());
    
    return true;
}

std::vector<geometry_msgs::msg::Point> RealTimeMissionReplanner::generateAlternatePath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    const std::vector<geometry_msgs::msg::Point>& obstacles)
{
    // Use A* pathfinding algorithm for obstacle avoidance
    return aStarPathfinding(start, goal, obstacles);
}

bool RealTimeMissionReplanner::replanForBatteryLevel(double remaining_battery_percent)
{
    if (remaining_battery_percent > 30.0) {
        return true;  // No replanning needed
    }
    
    RCLCPP_WARN(this->get_logger(), 
        "🔋 Low battery (%.1f%%) - replanning for shorter mission",
        remaining_battery_percent);
    
    // Calculate how much mission can be completed with remaining battery
    double estimated_flight_time = remaining_battery_percent * 0.01 * 30.0 * 60.0;  // 30 min max flight time
    double average_speed = 8.0;  // m/s average mission speed
    double remaining_range = estimated_flight_time * average_speed;
    
    // Find waypoints within remaining range
    auto current_position = getCurrentPosition();
    std::vector<geometry_msgs::msg::Point> shortened_mission;
    
    double total_distance = 0.0;
    for (const auto& waypoint : current_mission_path_) {
        double distance_to_waypoint = calculateDistance(
            shortened_mission.empty() ? current_position : shortened_mission.back(),
            waypoint
        );
        
        if (total_distance + distance_to_waypoint > remaining_range * 0.8) {  // 80% safety margin
            break;
        }
        
        shortened_mission.push_back(waypoint);
        total_distance += distance_to_waypoint;
    }
    
    // Add return-to-home waypoint
    geometry_msgs::msg::Point home_position;
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = -10.0;  // Home altitude
    shortened_mission.push_back(home_position);
    
    // Update mission
    current_mission_path_ = shortened_mission;
    publishReplanedMission(shortened_mission);
    
    RCLCPP_INFO(this->get_logger(), 
        "✅ Mission shortened to %zu waypoints due to low battery",
        shortened_mission.size());
    
    return true;
}
```

## 🔧 3. Troubleshooting and Best Practices

### 3.1 Common Offboard Control Issues

#### Issue: Vehicle Not Responding to Offboard Commands
```bash
# Debug checklist:
# 1. Check PX4 mode
pxh> commander status
# Should show: "Main state: OFFBOARD"

# 2. Check MAVLink connection
pxh> mavlink status
# Should show active connection to GCS/companion computer

# 3. Check setpoint stream
ros2 topic hz /mavros/setpoint_raw/local
# Should show ~50Hz

# 4. Check vehicle state
ros2 topic echo /mavros/state --once
# connected: true, armed: true, mode: "OFFBOARD"
```

#### Issue: Position Control Instability
```cpp
// Solution: Tune position controller parameters
void tunePositionController() {
    // Reduce gains for smoother flight
    param_set("MPC_XY_P", 0.8);     // Default: 1.0
    param_set("MPC_Z_P", 1.0);      // Default: 1.0
    param_set("MPC_XY_VEL_P", 0.12); // Default: 0.15
    param_set("MPC_Z_VEL_P", 0.15);  // Default: 0.2
    
    // Reduce acceleration limits
    param_set("MPC_ACC_HOR_MAX", 3.0);  // Default: 5.0
    param_set("MPC_ACC_UP_MAX", 2.0);   // Default: 4.0
    
    param_save();
}
```

### 3.2 Performance Optimization

#### High-Frequency Control Loop Optimization
```cpp
class OptimizedOffboardController : public MissionOffboardController
{
private:
    void setupOptimizedControl() {
        // Use dedicated callback group for control
        control_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // High priority timer for control
        control_timer_ = this->create_timer(
            std::chrono::milliseconds(20),  // 50Hz
            std::bind(&OptimizedOffboardController::controlLoop, this),
            control_callback_group_
        );
        
        // Separate thread for control processing
        control_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        control_executor_->add_callback_group(control_callback_group_);
        
        control_thread_ = std::thread([this]() {
            control_executor_->spin();
        });
    }
    
private:
    rclcpp::CallbackGroup::SharedPtr control_callback_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> control_executor_;
    std::thread control_thread_;
};
```

## 🎯 Conclusion

This comprehensive offboard control pattern guide provides:

### ✅ Key Capabilities
- **High-Frequency Control**: 50Hz position/velocity commands for smooth flight
- **Mission Integration**: Seamless integration with mission coordination actions
- **Emergency Procedures**: Robust failsafe and abort mechanisms
- **Real-Time Replanning**: Dynamic mission modification for obstacles and constraints
- **Safety Monitoring**: Continuous monitoring of battery, connection, and system health

### 🔗 Integration Points
- **Mission Actions**: Direct integration with `SearchArea`, `NavigateToTarget` actions
- **PX4 SITL**: Native offboard mode control with MAVLink
- **Emergency Systems**: Automatic failsafe triggers and recovery procedures
- **Real-Time Adaptation**: Dynamic mission replanning based on conditions

### 📁 Related Files
- **Mission Coordination**: `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_coordination_node.cpp`
- **Vehicle Nodes**: `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_multirotor_node.cpp`
- **PX4 Integration**: `L:\Cosys-AirSim\docs\px4\px4_sitl.md`
- **Ultra-Clean Architecture**: `L:\Cosys-AirSim\ros2\README_MULTIROTOR_ARCHITECTURE.md`

This offboard control system enables precise, safe, and efficient autonomous mission execution with PX4 SITL in the Cosys-AirSim environment.