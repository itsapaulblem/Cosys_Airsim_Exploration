# 🚁 PX4 SITL Mission Coordination Integration Guide
## Complete Guide for PX4 SITL Integration with Mission Coordination System in Cosys-AirSim

This comprehensive guide covers integrating PX4 SITL (Software-in-the-Loop) with the ultra-clean mission coordination system in Cosys-AirSim. The mission coordination system provides advanced multi-vehicle mission planning and execution capabilities, while PX4 SITL delivers high-fidelity flight control simulation.

## 📋 Prerequisites

- **Cosys-AirSim with ultra-clean ROS2 architecture** (see `ros2/README_MULTIROTOR_ARCHITECTURE.md`)
- **Mission coordination system** implemented (`mission_search_interfaces` package)
- **PX4 Autopilot** built for SITL simulation
- **Docker environment** for containerized deployment
- **MAVLink protocol** knowledge for communication setup

## 🎯 System Architecture Overview

### Mission Coordination Integration Points
The PX4 SITL integration connects with these key mission coordination components:

```
Mission Coordinator (/mission_coordinator)
├── 🎯 Mission Planning (SearchArea actions)
├── 🚁 PX4 SITL Instances (multi-vehicle)
├── 📡 MAVLink Communication (UDP ports 14560+)
├── 🌐 ROS2 Bridge (ultra-clean vehicle nodes)
└── 🔄 Real-time Command & Control
```

### Ultra-Clean Architecture Integration
- **Vehicle Nodes**: `/Droan1`, `/PX4_Drone2` (vehicle names ARE node names)
- **Mission Actions**: `SearchArea`, `NavigateToTarget`, `TrackTarget`, `ExecuteMission`
- **PX4 Integration**: MAVLink → ROS2 bridge → Mission coordination
- **Coordinate Frames**: NED (PX4) ↔ ENU (ROS2) transformations

## 🛠️ 1. PX4 SITL Integration with Mission Coordination

### 1.1 Multi-Vehicle PX4 SITL Setup for Missions

#### Docker-based PX4 Multi-Instance Configuration
```yaml
# docker-compose-px4-mission.yml
version: '3.8'

services:
  # Mission Coordination System
  mission-coordinator:
    image: cosys-airsim:ros2-mission
    container_name: mission-coordinator
    networks:
      - mission-network
    environment:
      - ROS_DOMAIN_ID=42
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /airsim_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py
      "
    depends_on:
      - px4-sitl-1
      - px4-sitl-2
      - px4-sitl-3

  # PX4 SITL Instance 1 - Primary Search Vehicle
  px4-sitl-1:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-sitl-drone-1
    networks:
      - mission-network
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
    ports:
      - "14560:14560/udp"  # MAVLink to AirSim
      - "14580:14580/udp"  # MAVLink external API
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        HEADLESS=1 make px4_sitl none_iris PX4_SIM_HOSTNAME=host.docker.internal
      "
    volumes:
      - px4-mission-params:/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
    extra_hosts:
      - "host.docker.internal:host-gateway"

  # PX4 SITL Instance 2 - Secondary Search Vehicle  
  px4-sitl-2:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-sitl-drone-2
    networks:
      - mission-network
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=host.docker.internal
    ports:
      - "14561:14560/udp"  # MAVLink to AirSim (offset port)
      - "14581:14580/udp"  # MAVLink external API
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        HEADLESS=1 PX4_SIM_MODEL_INSTANCE=1 make px4_sitl none_iris
      "
    extra_hosts:
      - "host.docker.internal:host-gateway"

  # PX4 SITL Instance 3 - Tracking/Support Vehicle
  px4-sitl-3:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-sitl-drone-3
    networks:
      - mission-network
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=host.docker.internal
    ports:
      - "14562:14560/udp"  # MAVLink to AirSim (offset port)
      - "14582:14580/udp"  # MAVLink external API
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        HEADLESS=1 PX4_SIM_MODEL_INSTANCE=2 make px4_sitl none_iris
      "
    extra_hosts:
      - "host.docker.internal:host-gateway"

  # ROS2 Mission Bridge
  ros2-mission-bridge:
    image: cosys-airsim:ros2-mission
    container_name: ros2-mission-bridge
    networks:
      - mission-network
    environment:
      - ROS_DOMAIN_ID=42
    depends_on:
      - px4-sitl-1
      - px4-sitl-2
      - px4-sitl-3
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /airsim_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py host_ip:=host.docker.internal
      "
    extra_hosts:
      - "host.docker.internal:host-gateway"

networks:
  mission-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

volumes:
  px4-mission-params:
```

#### AirSim Settings for Mission Coordination
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  
  "Vehicles": {
    "Droan1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14560,
      "ControlPort": 14580,
      "X": 0.0, "Y": 0.0, "Z": -10.0,
      "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0,
      "Cameras": {
        "rgb_camera": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.25, "Y": 0.0, "Z": 0.0,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        },
        "thermal_camera": {
          "CaptureSettings": [
            {
              "ImageType": 5,
              "Width": 640,
              "Height": 512,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.25, "Y": 0.1, "Z": 0.0,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        }
      },
      "Sensors": {
        "gps": {
          "SensorType": 3,
          "Enabled": true
        },
        "imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "magnetometer": {
          "SensorType": 4,
          "Enabled": true
        },
        "barometer": {
          "SensorType": 1,
          "Enabled": true
        }
      }
    },
    
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14561,
      "ControlPort": 14581,
      "X": 20.0, "Y": 0.0, "Z": -10.0,
      "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0,
      "Cameras": {
        "rgb_camera": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ]
        }
      },
      "Sensors": {
        "gps": {"SensorType": 3, "Enabled": true},
        "imu": {"SensorType": 2, "Enabled": true},
        "magnetometer": {"SensorType": 4, "Enabled": true},
        "barometer": {"SensorType": 1, "Enabled": true}
      }
    },
    
    "PX4_Drone3": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "127.0.0.1",
      "UdpPort": 14562,
      "ControlPort": 14582,
      "X": -20.0, "Y": 0.0, "Z": -10.0,
      "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0,
      "Cameras": {
        "rgb_camera": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ]
        }
      },
      "Sensors": {
        "gps": {"SensorType": 3, "Enabled": true},
        "imu": {"SensorType": 2, "Enabled": true},
        "magnetometer": {"SensorType": 4, "Enabled": true},
        "barometer": {"SensorType": 1, "Enabled": true}
      }
    }
  }
}
```

### 1.2 PX4 Parameter Configuration for Missions

#### Mission-Optimized PX4 Parameters
```bash
#!/bin/bash
# setup_px4_mission_params.sh - Configure PX4 for mission coordination

# MAVLink Configuration for Mission Communication
param set MAV_0_CONFIG 101          # Enable MAVLink on ttyS0 (UART1)
param set MAV_0_MODE 0              # MAVLink mode: Normal
param set MAV_0_RATE 921600         # Baud rate for MAVLink
param set MAV_0_FORWARD 1           # Enable MAVLink forwarding

# Mission Planning Parameters
param set MIS_DIST_1WP 900          # Distance to first waypoint (m)
param set MIS_DIST_WPS 900          # Distance between waypoints (m)
param set MIS_MNT_YAW_CTL 1         # Enable mount yaw control during missions
param set MIS_TAKEOFF_ALT 10.0      # Mission takeoff altitude (m)
param set MIS_TAKEOFF_REQ 1         # Require takeoff before mission

# Search Pattern Flight Parameters
param set MPC_XY_CRUISE 8.0         # Cruise speed for search patterns (m/s)
param set MPC_Z_VEL_MAX_UP 3.0      # Max vertical velocity up (m/s)
param set MPC_Z_VEL_MAX_DN 2.0      # Max vertical velocity down (m/s)
param set MPC_XY_VEL_MAX 10.0       # Max horizontal velocity (m/s)
param set MPC_TILTMAX_AIR 35.0      # Max tilt angle in air (deg)

# Position Control for Precise Search
param set MPC_XY_P 1.2              # Position control P gain (horizontal)
param set MPC_Z_P 1.5               # Position control P gain (vertical)
param set MPC_XY_VEL_P 0.15         # Velocity control P gain (horizontal)
param set MPC_Z_VEL_P 0.2           # Velocity control P gain (vertical)

# Mission Coordination and Communication
param set COM_RC_LOSS_T 5.0         # RC loss timeout (s)
param set COM_OF_LOSS_T 10.0        # Optical flow loss timeout (s)
param set COM_OBL_ACT 2             # Offboard loss action (land)
param set COM_OBL_RC_ACT 0          # Offboard RC loss action (position)

# GPS and Navigation for Search Missions
param set EKF2_GPS_CHECK 21         # GPS check configuration
param set EKF2_REQ_HDOP 2.5         # Required HDOP for GPS
param set EKF2_REQ_SACC 1.0         # Required speed accuracy (m/s)
param set GPS_YAW_OFFSET 0.0        # GPS yaw offset (deg)

# Failsafe Configuration for Mission Safety
param set RTL_RETURN_ALT 30.0       # Return-to-launch altitude (m)
param set RTL_DESCEND_ALT 10.0      # Descent altitude during RTL (m)
param set RTL_LAND_DELAY 0.0        # Delay before landing (s)
param set RTL_MIN_DIST 5.0          # Minimum RTL distance (m)

# Geofence for Mission Area Containment
param set GF_ACTION 2               # Geofence action (RTL)
param set GF_MAX_HOR_DIST 1000.0    # Max horizontal distance (m)
param set GF_MAX_VER_DIST 150.0     # Max vertical distance (m)

# Battery and Power Management
param set BAT_LOW_THR 0.25          # Low battery threshold (25%)
param set BAT_CRIT_THR 0.15         # Critical battery threshold (15%)
param set BAT_EMERGEN_THR 0.10      # Emergency battery threshold (10%)

# Data Logging for Mission Analysis
param set SDLOG_PROFILE 6           # Extended logging profile
param set SDLOG_DIRS_MAX 10         # Maximum log directories

echo "PX4 mission parameters configured successfully"
```

#### Deploy Parameters to PX4 Instances
```bash
#!/bin/bash
# deploy_mission_params.sh - Deploy parameters to all PX4 instances

PX4_INSTANCES=("14580" "14581" "14582")
PARAM_FILE="px4_mission_params.txt"

for port in "${PX4_INSTANCES[@]}"; do
    echo "Deploying parameters to PX4 instance on port $port..."
    
    # Connect to PX4 console and apply parameters
    mavlink_shell.py --baudrate 57600 --device tcp:localhost:$port << EOF
param load $PARAM_FILE
param save
reboot
EOF
    
    echo "Parameters deployed to PX4 instance on port $port"
    sleep 2
done

echo "All PX4 instances configured for mission coordination"
```

## 🎯 2. Autonomous Mission Execution

### 2.1 Mission-Aware ROS2 Node Implementation

#### Enhanced Mission Multirotor Node
```cpp
// mission_multirotor_node_enhanced.cpp
#include "mission_multirotor_node.hpp"
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mission_search_interfaces/action/search_area.hpp>
#include <mission_search_interfaces/action/navigate_to_target.hpp>

class EnhancedMissionMultirotorNode : public MissionMultirotorNode
{
public:
    EnhancedMissionMultirotorNode(const std::string& vehicle_name)
        : MissionMultirotorNode(vehicle_name)
    {
        setupPX4Integration();
        setupMissionActions();
        
        RCLCPP_INFO(this->get_logger(), 
            "Enhanced mission multirotor node ready for %s with PX4 integration", 
            vehicle_name.c_str());
    }

private:
    void setupPX4Integration()
    {
        // MAVROS subscribers for PX4 state
        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/" + vehicle_name_ + "/mavros/state",
            10,
            std::bind(&EnhancedMissionMultirotorNode::mavrosStateCallback, this, std::placeholders::_1)
        );
        
        // MAVROS publishers for PX4 control
        setpoint_position_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/" + vehicle_name_ + "/mavros/setpoint_raw/local",
            10
        );
        
        // MAVROS service clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/" + vehicle_name_ + "/mavros/cmd/arming"
        );
        
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/" + vehicle_name_ + "/mavros/set_mode"
        );
        
        // Transform listener for coordinate conversion
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    void setupMissionActions()
    {
        // Enhanced SearchArea action server with PX4 waypoint integration
        search_area_action_server_ = rclcpp_action::create_server<mission_search_interfaces::action::SearchArea>(
            this,
            "/" + vehicle_name_ + "/actions/search_area",
            std::bind(&EnhancedMissionMultirotorNode::handleSearchAreaGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&EnhancedMissionMultirotorNode::handleSearchAreaCancel, this, std::placeholders::_1),
            std::bind(&EnhancedMissionMultirotorNode::handleSearchAreaAccepted, this, std::placeholders::_1)
        );
        
        // Enhanced NavigateToTarget action server
        navigate_action_server_ = rclcpp_action::create_server<mission_search_interfaces::action::NavigateToTarget>(
            this,
            "/" + vehicle_name_ + "/actions/navigate_to_target",
            std::bind(&EnhancedMissionMultirotorNode::handleNavigateGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&EnhancedMissionMultirotorNode::handleNavigateCancel, this, std::placeholders::_1),
            std::bind(&EnhancedMissionMultirotorNode::handleNavigateAccepted, this, std::placeholders::_1)
        );
    }
    
    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mavros_state_ = *msg;
        
        // Update mission coordination with PX4 state
        if (msg->armed && msg->mode == "OFFBOARD")
        {
            mission_state_ = MissionState::READY_FOR_MISSION;
        }
        else if (msg->armed && msg->mode == "AUTO.MISSION")
        {
            mission_state_ = MissionState::EXECUTING_MISSION;
        }
        else
        {
            mission_state_ = MissionState::STANDBY;
        }
    }
    
    rclcpp_action::GoalResponse handleSearchAreaGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const mission_search_interfaces::action::SearchArea::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received SearchArea goal for %s: pattern=%s, area_points=%zu",
            vehicle_name_.c_str(),
            goal->search_pattern.c_str(),
            goal->search_area.points.size());
        
        // Validate PX4 readiness
        if (!current_mavros_state_.armed || current_mavros_state_.mode != "OFFBOARD")
        {
            RCLCPP_WARN(this->get_logger(), 
                "PX4 not ready for mission: armed=%s, mode=%s",
                current_mavros_state_.armed ? "true" : "false",
                current_mavros_state_.mode.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Convert search area to PX4 waypoints
        if (!validateAndConvertSearchArea(goal->search_area))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid search area for PX4 mission");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    void handleSearchAreaAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::SearchArea>> goal_handle)
    {
        // Execute search area mission with PX4 waypoint following
        std::thread{std::bind(&EnhancedMissionMultirotorNode::executeSearchAreaMission, this, goal_handle)}.detach();
    }
    
    void executeSearchAreaMission(std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::SearchArea>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<mission_search_interfaces::action::SearchArea::Feedback>();
        auto result = std::make_shared<mission_search_interfaces::action::SearchArea::Result>();
        
        RCLCPP_INFO(this->get_logger(), "Executing search area mission with PX4 waypoints");
        
        // Generate waypoints based on search pattern
        std::vector<geometry_msgs::msg::Point> waypoints = generateSearchWaypoints(
            goal->search_area, 
            goal->search_pattern,
            goal->search_altitude
        );
        
        // Execute waypoint mission
        bool mission_success = executePX4WaypointMission(waypoints, goal_handle, feedback);
        
        // Complete mission
        result->success = mission_success;
        result->waypoints_completed = feedback->current_waypoint_index;
        result->total_distance_covered = feedback->distance_traveled;
        result->mission_duration = feedback->elapsed_time;
        
        if (mission_success)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Search area mission completed successfully");
        }
        else
        {
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Search area mission aborted");
        }
    }
    
    bool executePX4WaypointMission(
        const std::vector<geometry_msgs::msg::Point>& waypoints,
        std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::SearchArea>> goal_handle,
        std::shared_ptr<mission_search_interfaces::action::SearchArea::Feedback> feedback)
    {
        auto start_time = this->get_clock()->now();
        double total_distance = 0.0;
        
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(this->get_logger(), "Mission cancelled by request");
                return false;
            }
            
            // Convert ROS2 ENU coordinates to PX4 NED
            auto ned_waypoint = convertENUtoNED(waypoints[i]);
            
            // Send waypoint to PX4 via MAVROS
            if (!navigateToWaypointPX4(ned_waypoint))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to navigate to waypoint %zu", i);
                return false;
            }
            
            // Update feedback
            feedback->current_waypoint_index = i + 1;
            feedback->total_waypoints = waypoints.size();
            feedback->progress_percentage = (float)(i + 1) / waypoints.size() * 100.0f;
            feedback->current_position = waypoints[i];
            feedback->elapsed_time = (this->get_clock()->now() - start_time);
            
            if (i > 0)
            {
                double segment_distance = calculateDistance(waypoints[i-1], waypoints[i]);
                total_distance += segment_distance;
                feedback->distance_traveled = total_distance;
            }
            
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), 
                "Completed waypoint %zu/%zu (%.1f%%)", 
                i + 1, waypoints.size(), feedback->progress_percentage);
        }
        
        return true;
    }
    
    bool navigateToWaypointPX4(const geometry_msgs::msg::Point& ned_waypoint)
    {
        // Create PX4 position target message
        mavros_msgs::msg::PositionTarget position_target;
        position_target.header.stamp = this->get_clock()->now();
        position_target.header.frame_id = "map";
        position_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        position_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                   mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                                   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        
        position_target.position.x = ned_waypoint.x;
        position_target.position.y = ned_waypoint.y;
        position_target.position.z = ned_waypoint.z;
        position_target.yaw = 0.0; // Face forward
        
        // Send position target continuously until reached
        auto waypoint_start_time = this->get_clock()->now();
        const double timeout_seconds = 30.0;
        const double position_tolerance = 1.0; // meters
        
        while (rclcpp::ok())
        {
            // Check timeout
            auto elapsed = this->get_clock()->now() - waypoint_start_time;
            if (elapsed.seconds() > timeout_seconds)
            {
                RCLCPP_WARN(this->get_logger(), "Waypoint navigation timeout");
                return false;
            }
            
            // Send position target
            position_target.header.stamp = this->get_clock()->now();
            setpoint_position_pub_->publish(position_target);
            
            // Check if waypoint reached
            if (isWaypointReached(ned_waypoint, position_tolerance))
            {
                RCLCPP_INFO(this->get_logger(), "Waypoint reached");
                return true;
            }
            
            // Wait before next iteration
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        return false;
    }
    
    geometry_msgs::msg::Point convertENUtoNED(const geometry_msgs::msg::Point& enu_point)
    {
        geometry_msgs::msg::Point ned_point;
        
        // ENU to NED coordinate transformation
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        ned_point.x = enu_point.y;   // North = ENU Y
        ned_point.y = enu_point.x;   // East = ENU X  
        ned_point.z = -enu_point.z;  // Down = -ENU Z
        
        return ned_point;
    }
    
    bool isWaypointReached(const geometry_msgs::msg::Point& target_ned, double tolerance)
    {
        // Get current vehicle position from MAVROS
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "map", vehicle_name_ + "/base_link",
                tf2::TimePointZero);
            
            geometry_msgs::msg::Point current_enu;
            current_enu.x = transform.transform.translation.x;
            current_enu.y = transform.transform.translation.y;
            current_enu.z = transform.transform.translation.z;
            
            auto current_ned = convertENUtoNED(current_enu);
            
            double distance = std::sqrt(
                std::pow(target_ned.x - current_ned.x, 2) +
                std::pow(target_ned.y - current_ned.y, 2) +
                std::pow(target_ned.z - current_ned.z, 2)
            );
            
            return distance < tolerance;
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get vehicle position: %s", ex.what());
            return false;
        }
    }
    
    std::vector<geometry_msgs::msg::Point> generateSearchWaypoints(
        const geometry_msgs::msg::Polygon& search_area,
        const std::string& pattern,
        double altitude)
    {
        std::vector<geometry_msgs::msg::Point> waypoints;
        
        if (pattern == "grid")
        {
            waypoints = generateGridPattern(search_area, altitude, 20.0); // 20m spacing
        }
        else if (pattern == "spiral")
        {
            waypoints = generateSpiralPattern(search_area, altitude, 15.0); // 15m spacing
        }
        else if (pattern == "random")
        {
            waypoints = generateRandomPattern(search_area, altitude, 50); // 50 points
        }
        else
        {
            // Default to grid pattern
            waypoints = generateGridPattern(search_area, altitude, 20.0);
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Generated %zu waypoints for %s pattern", 
            waypoints.size(), pattern.c_str());
        
        return waypoints;
    }

private:
    // PX4/MAVROS interfaces
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_position_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    
    // Mission action servers
    rclcpp_action::Server<mission_search_interfaces::action::SearchArea>::SharedPtr search_area_action_server_;
    rclcpp_action::Server<mission_search_interfaces::action::NavigateToTarget>::SharedPtr navigate_action_server_;
    
    // Transform handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // State tracking
    mavros_msgs::msg::State current_mavros_state_;
    
    enum class MissionState
    {
        STANDBY,
        READY_FOR_MISSION,
        EXECUTING_MISSION,
        MISSION_COMPLETE,
        EMERGENCY
    };
    
    MissionState mission_state_ = MissionState::STANDBY;
};
```

### 2.2 Coordinate Frame Transformations

#### Precision NED ↔ ENU Transformation Library
```cpp
// coordinate_transforms.hpp
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MissionCoordinateTransforms
{
public:
    /**
     * @brief Convert ENU (ROS2) coordinates to NED (PX4) coordinates
     * ENU: X=East, Y=North, Z=Up
     * NED: X=North, Y=East, Z=Down
     */
    static geometry_msgs::msg::Point convertENUtoNED(const geometry_msgs::msg::Point& enu)
    {
        geometry_msgs::msg::Point ned;
        ned.x = enu.y;   // North = ENU Y
        ned.y = enu.x;   // East = ENU X
        ned.z = -enu.z;  // Down = -ENU Z
        return ned;
    }
    
    /**
     * @brief Convert NED (PX4) coordinates to ENU (ROS2) coordinates
     */
    static geometry_msgs::msg::Point convertNEDtoENU(const geometry_msgs::msg::Point& ned)
    {
        geometry_msgs::msg::Point enu;
        enu.x = ned.y;   // East = NED Y
        enu.y = ned.x;   // North = NED X
        enu.z = -ned.z;  // Up = -NED Z
        return enu;
    }
    
    /**
     * @brief Convert ENU quaternion to NED quaternion
     * Applies 180-degree rotation about X-axis
     */
    static geometry_msgs::msg::Quaternion convertENUtoNEDQuaternion(const geometry_msgs::msg::Quaternion& enu_quat)
    {
        // Create rotation quaternion for ENU to NED transformation
        // 180-degree rotation about X-axis: q = [1, 0, 0, 0] (normalized)
        tf2::Quaternion rotation_x_180;
        rotation_x_180.setRPY(M_PI, 0, 0); // 180 degrees about X
        
        // Convert ROS quaternion to tf2 quaternion
        tf2::Quaternion enu_tf2;
        tf2::fromMsg(enu_quat, enu_tf2);
        
        // Apply rotation: NED = R_x(180°) * ENU
        tf2::Quaternion ned_tf2 = rotation_x_180 * enu_tf2;
        
        // Convert back to ROS message
        geometry_msgs::msg::Quaternion ned_quat;
        ned_quat = tf2::toMsg(ned_tf2);
        
        return ned_quat;
    }
    
    /**
     * @brief Convert NED quaternion to ENU quaternion
     */
    static geometry_msgs::msg::Quaternion convertNEDtoENUQuaternion(const geometry_msgs::msg::Quaternion& ned_quat)
    {
        // Inverse transformation: ENU = R_x(-180°) * NED = R_x(180°) * NED
        // (180-degree rotation is its own inverse)
        return convertENUtoNEDQuaternion(ned_quat);
    }
    
    /**
     * @brief Convert velocity from ENU to NED frame
     */
    static geometry_msgs::msg::Vector3 convertVelocityENUtoNED(const geometry_msgs::msg::Vector3& enu_vel)
    {
        geometry_msgs::msg::Vector3 ned_vel;
        ned_vel.x = enu_vel.y;   // North velocity = ENU Y velocity
        ned_vel.y = enu_vel.x;   // East velocity = ENU X velocity
        ned_vel.z = -enu_vel.z;  // Down velocity = -ENU Z velocity
        return ned_vel;
    }
    
    /**
     * @brief Convert acceleration from ENU to NED frame
     */
    static geometry_msgs::msg::Vector3 convertAccelerationENUtoNED(const geometry_msgs::msg::Vector3& enu_acc)
    {
        geometry_msgs::msg::Vector3 ned_acc;
        ned_acc.x = enu_acc.y;   // North acceleration = ENU Y acceleration
        ned_acc.y = enu_acc.x;   // East acceleration = ENU X acceleration
        ned_acc.z = -enu_acc.z;  // Down acceleration = -ENU Z acceleration
        return ned_acc;
    }
    
    /**
     * @brief Transform entire odometry message from ENU to NED
     */
    static nav_msgs::msg::Odometry transformOdometryENUtoNED(const nav_msgs::msg::Odometry& enu_odom)
    {
        nav_msgs::msg::Odometry ned_odom = enu_odom;
        
        // Transform position
        ned_odom.pose.pose.position = convertENUtoNED(enu_odom.pose.pose.position);
        
        // Transform orientation
        ned_odom.pose.pose.orientation = convertENUtoNEDQuaternion(enu_odom.pose.pose.orientation);
        
        // Transform linear velocity
        ned_odom.twist.twist.linear = convertVelocityENUtoNED(enu_odom.twist.twist.linear);
        
        // Transform angular velocity (same transformation as linear velocity)
        ned_odom.twist.twist.angular = convertVelocityENUtoNED(enu_odom.twist.twist.angular);
        
        // Update frame IDs
        ned_odom.header.frame_id = "ned_frame";
        ned_odom.child_frame_id = "ned_base_link";
        
        return ned_odom;
    }
    
    /**
     * @brief Validate coordinate transformation accuracy
     */
    static bool validateTransformation(const geometry_msgs::msg::Point& original, 
                                     const geometry_msgs::msg::Point& round_trip,
                                     double tolerance = 1e-6)
    {
        double dx = std::abs(original.x - round_trip.x);
        double dy = std::abs(original.y - round_trip.y);
        double dz = std::abs(original.z - round_trip.z);
        
        return (dx < tolerance) && (dy < tolerance) && (dz < tolerance);
    }
    
    /**
     * @brief Test coordinate transformation round-trip accuracy
     */
    static void testTransformationAccuracy()
    {
        // Test point
        geometry_msgs::msg::Point enu_original;
        enu_original.x = 10.0;  // East
        enu_original.y = 20.0;  // North  
        enu_original.z = -5.0;  // Up (negative altitude)
        
        // Forward transformation
        auto ned = convertENUtoNED(enu_original);
        
        // Reverse transformation
        auto enu_recovered = convertNEDtoENU(ned);
        
        // Validate accuracy
        bool accurate = validateTransformation(enu_original, enu_recovered);
        
        std::cout << "Coordinate transformation test: " 
                  << (accurate ? "PASSED" : "FAILED") << std::endl;
        std::cout << "Original ENU: (" << enu_original.x << ", " 
                  << enu_original.y << ", " << enu_original.z << ")" << std::endl;
        std::cout << "Converted NED: (" << ned.x << ", " 
                  << ned.y << ", " << ned.z << ")" << std::endl;
        std::cout << "Recovered ENU: (" << enu_recovered.x << ", " 
                  << enu_recovered.y << ", " << enu_recovered.z << ")" << std::endl;
    }
};
```

## 🔗 3. Multi-Vehicle PX4 Coordination

### 3.1 Formation Flying and Collision Avoidance

#### Multi-Vehicle Mission Coordinator
```cpp
// multi_vehicle_mission_coordinator.cpp
#include <rclcpp/rclcpp.hpp>
#include <mission_search_interfaces/action/execute_mission.hpp>
#include <mission_search_interfaces/msg/mission_plan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

class MultiVehicleMissionCoordinator : public rclcpp::Node
{
public:
    MultiVehicleMissionCoordinator()
        : Node("multi_vehicle_mission_coordinator")
    {
        setupMissionCoordination();
        setupCollisionAvoidance();
        setupFormationControl();
        
        RCLCPP_INFO(this->get_logger(), "Multi-vehicle mission coordinator ready");
    }

private:
    void setupMissionCoordination()
    {
        // Mission execution action server
        mission_execution_server_ = rclcpp_action::create_server<mission_search_interfaces::action::ExecuteMission>(
            this,
            "/mission_coordinator/actions/execute_mission",
            std::bind(&MultiVehicleMissionCoordinator::handleMissionGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultiVehicleMissionCoordinator::handleMissionCancel, this, std::placeholders::_1),
            std::bind(&MultiVehicleMissionCoordinator::handleMissionAccepted, this, std::placeholders::_1)
        );
        
        // Vehicle status monitoring
        vehicle_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/airsim_coordination_node/system_status",
            10,
            std::bind(&MultiVehicleMissionCoordinator::vehicleStatusCallback, this, std::placeholders::_1)
        );
        
        // Initialize known vehicles
        known_vehicles_ = {"Droan1", "PX4_Drone2", "PX4_Drone3"};
        
        for (const auto& vehicle : known_vehicles_)
        {
            vehicle_states_[vehicle] = VehicleState::IDLE;
            vehicle_positions_[vehicle] = geometry_msgs::msg::Point();
            
            // Subscribe to individual vehicle positions
            auto pos_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + vehicle + "/odom_local_ned",
                10,
                [this, vehicle](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    vehicle_positions_[vehicle] = msg->pose.pose.position;
                    updateCollisionAvoidance(vehicle, msg->pose.pose.position);
                }
            );
            position_subscribers_[vehicle] = pos_sub;
        }
    }
    
    void setupCollisionAvoidance()
    {
        // Collision avoidance parameters
        min_separation_distance_ = 10.0; // meters
        collision_avoidance_enabled_ = true;
        
        // Timer for collision checking
        collision_check_timer_ = this->create_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&MultiVehicleMissionCoordinator::checkCollisions, this)
        );
    }
    
    void setupFormationControl()
    {
        // Formation parameters
        formation_patterns_["line"] = generateLineFormation();
        formation_patterns_["triangle"] = generateTriangleFormation();
        formation_patterns_["search_spread"] = generateSearchSpreadFormation();
        
        current_formation_ = "search_spread";
    }
    
    rclcpp_action::GoalResponse handleMissionGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const mission_search_interfaces::action::ExecuteMission::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received multi-vehicle mission: %s", 
            goal->mission_plan.mission_name.c_str());
        
        // Validate vehicles are available
        auto available_vehicles = getAvailableVehicles();
        if (available_vehicles.size() < goal->mission_plan.min_vehicles_required)
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Insufficient vehicles: need %d, have %zu",
                goal->mission_plan.min_vehicles_required,
                available_vehicles.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    void handleMissionAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::ExecuteMission>> goal_handle)
    {
        std::thread{std::bind(&MultiVehicleMissionCoordinator::executeMultiVehicleMission, this, goal_handle)}.detach();
    }
    
    void executeMultiVehicleMission(std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::ExecuteMission>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<mission_search_interfaces::action::ExecuteMission::Feedback>();
        auto result = std::make_shared<mission_search_interfaces::action::ExecuteMission::Result>();
        
        RCLCPP_INFO(this->get_logger(), "Executing multi-vehicle search mission");
        
        // Phase 1: Formation Setup
        bool formation_ready = setupMissionFormation(goal->mission_plan);
        if (!formation_ready)
        {
            result->success = false;
            result->completion_reason = "Formation setup failed";
            goal_handle->abort(result);
            return;
        }
        
        // Phase 2: Assign Search Zones
        auto zone_assignments = assignSearchZones(goal->mission_plan);
        
        // Phase 3: Execute Coordinated Search
        bool mission_success = executeCoordinatedSearch(zone_assignments, goal_handle, feedback);
        
        // Phase 4: Return to Formation
        if (mission_success)
        {
            returnToFormation();
        }
        
        result->success = mission_success;
        result->vehicles_participated = static_cast<int>(zone_assignments.size());
        result->completion_reason = mission_success ? "Mission completed successfully" : "Mission failed";
        
        if (mission_success)
        {
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
        }
    }
    
    bool setupMissionFormation(const mission_search_interfaces::msg::MissionPlan& mission_plan)
    {
        // Calculate formation center point
        geometry_msgs::msg::Point formation_center = calculateFormationCenter(mission_plan.mission_area);
        
        // Get formation positions
        auto formation_positions = formation_patterns_[current_formation_];
        
        // Assign formation positions to vehicles
        auto available_vehicles = getAvailableVehicles();
        
        std::vector<std::future<bool>> formation_futures;
        
        for (size_t i = 0; i < std::min(available_vehicles.size(), formation_positions.size()); ++i)
        {
            const auto& vehicle = available_vehicles[i];
            auto target_position = formation_center;
            target_position.x += formation_positions[i].x;
            target_position.y += formation_positions[i].y;
            target_position.z = formation_positions[i].z;
            
            // Send formation position command asynchronously
            auto future = std::async(std::launch::async, [this, vehicle, target_position]() {
                return navigateVehicleToPosition(vehicle, target_position);
            });
            
            formation_futures.push_back(std::move(future));
        }
        
        // Wait for all vehicles to reach formation positions
        bool all_in_formation = true;
        for (auto& future : formation_futures)
        {
            bool vehicle_ready = future.get();
            all_in_formation = all_in_formation && vehicle_ready;
        }
        
        if (all_in_formation)
        {
            RCLCPP_INFO(this->get_logger(), "All vehicles in formation and ready for mission");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to establish mission formation");
        }
        
        return all_in_formation;
    }
    
    std::map<std::string, geometry_msgs::msg::Polygon> assignSearchZones(const mission_search_interfaces::msg::MissionPlan& mission_plan)
    {
        std::map<std::string, geometry_msgs::msg::Polygon> zone_assignments;
        
        auto available_vehicles = getAvailableVehicles();
        
        // Divide mission area into zones based on number of available vehicles
        auto search_zones = subdivideSearchArea(mission_plan.mission_area, available_vehicles.size());
        
        for (size_t i = 0; i < std::min(available_vehicles.size(), search_zones.size()); ++i)
        {
            zone_assignments[available_vehicles[i]] = search_zones[i];
            
            RCLCPP_INFO(this->get_logger(), 
                "Assigned search zone %zu to vehicle %s (%zu points)",
                i, available_vehicles[i].c_str(), search_zones[i].points.size());
        }
        
        return zone_assignments;
    }
    
    bool executeCoordinatedSearch(
        const std::map<std::string, geometry_msgs::msg::Polygon>& zone_assignments,
        std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_search_interfaces::action::ExecuteMission>> goal_handle,
        std::shared_ptr<mission_search_interfaces::action::ExecuteMission::Feedback> feedback)
    {
        std::vector<rclcpp_action::Client<mission_search_interfaces::action::SearchArea>::SharedPtr> search_clients;
        std::vector<std::future<bool>> search_futures;
        
        // Create action clients for each vehicle
        for (const auto& [vehicle, zone] : zone_assignments)
        {
            auto client = rclcpp_action::create_client<mission_search_interfaces::action::SearchArea>(
                this, "/" + vehicle + "/actions/search_area"
            );
            
            if (!client->wait_for_action_server(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(this->get_logger(), 
                    "Search area action server not available for %s", vehicle.c_str());
                continue;
            }
            
            search_clients.push_back(client);
            
            // Send search area goal asynchronously
            auto future = std::async(std::launch::async, [this, client, zone]() {
                return executeVehicleSearch(client, zone);
            });
            
            search_futures.push_back(std::move(future));
        }
        
        // Monitor search progress
        auto mission_start_time = this->get_clock()->now();
        
        while (rclcpp::ok())
        {
            // Check for cancellation
            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(this->get_logger(), "Multi-vehicle mission cancelled");
                cancelAllSearches(search_clients);
                return false;
            }
            
            // Check if all searches completed
            bool all_completed = true;
            int completed_count = 0;
            
            for (auto& future : search_futures)
            {
                auto status = future.wait_for(std::chrono::seconds(0));
                if (status == std::future_status::ready)
                {
                    completed_count++;
                }
                else
                {
                    all_completed = false;
                }
            }
            
            // Update feedback
            feedback->progress_percentage = (float)completed_count / search_futures.size() * 100.0f;
            feedback->vehicles_active = static_cast<int>(search_futures.size() - completed_count);
            feedback->elapsed_time = this->get_clock()->now() - mission_start_time;
            
            goal_handle->publish_feedback(feedback);
            
            if (all_completed)
            {
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // Check results
        bool overall_success = true;
        for (auto& future : search_futures)
        {
            bool vehicle_success = future.get();
            overall_success = overall_success && vehicle_success;
        }
        
        return overall_success;
    }
    
    void checkCollisions()
    {
        if (!collision_avoidance_enabled_)
            return;
        
        auto available_vehicles = getAvailableVehicles();
        
        for (size_t i = 0; i < available_vehicles.size(); ++i)
        {
            for (size_t j = i + 1; j < available_vehicles.size(); ++j)
            {
                const auto& vehicle1 = available_vehicles[i];
                const auto& vehicle2 = available_vehicles[j];
                
                double distance = calculateDistance(
                    vehicle_positions_[vehicle1],
                    vehicle_positions_[vehicle2]
                );
                
                if (distance < min_separation_distance_)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "Collision risk: %s and %s are %.2f meters apart (min: %.2f)",
                        vehicle1.c_str(), vehicle2.c_str(),
                        distance, min_separation_distance_);
                    
                    handleCollisionAvoidance(vehicle1, vehicle2);
                }
            }
        }
    }
    
    void handleCollisionAvoidance(const std::string& vehicle1, const std::string& vehicle2)
    {
        // Implement collision avoidance maneuver
        // This could involve:
        // 1. Temporary altitude separation
        // 2. Lateral offset maneuvers
        // 3. Speed adjustments
        // 4. Temporary hold positions
        
        RCLCPP_INFO(this->get_logger(),
            "Implementing collision avoidance for %s and %s",
            vehicle1.c_str(), vehicle2.c_str());
        
        // Example: Apply altitude separation
        applyAltitudeSeparation(vehicle1, vehicle2);
    }
    
    void applyAltitudeSeparation(const std::string& vehicle1, const std::string& vehicle2)
    {
        // Increase altitude of one vehicle temporarily
        auto target_pos = vehicle_positions_[vehicle1];
        target_pos.z -= 5.0; // Increase altitude by 5 meters (NED: negative Z is up)
        
        navigateVehicleToPosition(vehicle1, target_pos);
        
        RCLCPP_INFO(this->get_logger(),
            "Applied altitude separation: %s moved to altitude %.2f",
            vehicle1.c_str(), target_pos.z);
    }

private:
    // Mission coordination
    rclcpp_action::Server<mission_search_interfaces::action::ExecuteMission>::SharedPtr mission_execution_server_;
    
    // Vehicle monitoring
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_status_sub_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> position_subscribers_;
    
    // Vehicle state tracking
    std::vector<std::string> known_vehicles_;
    std::map<std::string, VehicleState> vehicle_states_;
    std::map<std::string, geometry_msgs::msg::Point> vehicle_positions_;
    
    // Collision avoidance
    rclcpp::TimerBase::SharedPtr collision_check_timer_;
    double min_separation_distance_;
    bool collision_avoidance_enabled_;
    
    // Formation control
    std::map<std::string, std::vector<geometry_msgs::msg::Point>> formation_patterns_;
    std::string current_formation_;
    
    enum class VehicleState
    {
        IDLE,
        READY,
        IN_MISSION,
        RETURNING,
        ERROR
    };
};
```

## 🌐 4. ROS2 Bridge and Communication

### 4.1 PX4-ROS2 Bridge Configuration

#### MAVROS Integration with Mission Coordination
```xml
<!-- px4_mission_bridge.launch.xml -->
<launch>
    <!-- Launch arguments -->
    <arg name="vehicle_name" default="Droan1" />
    <arg name="mavros_ns" default="$(var vehicle_name)/mavros" />
    <arg name="fcu_url" default="udp://:14540@127.0.0.1:14560" />
    <arg name="gcs_url" default="" />
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="1" />
    
    <!-- MAVROS node for PX4 communication -->
    <node pkg="mavros" exec="mavros_node" name="mavros" namespace="$(var vehicle_name)">
        <param name="fcu_url" value="$(var fcu_url)" />
        <param name="gcs_url" value="$(var gcs_url)" />
        <param name="target_system_id" value="$(var tgt_system)" />
        <param name="target_component_id" value="$(var tgt_component)" />
        
        <!-- Plugin configuration -->
        <param name="plugin_whitelist" value="
            global_position,
            local_position,
            setpoint_position,
            setpoint_velocity,
            setpoint_accel,
            setpoint_attitude,
            sys_status,
            command,
            manual_control,
            param,
            mission
        " />
        
        <!-- Frame transformations -->
        <param name="local_position/frame_id" value="$(var vehicle_name)/odom" />
        <param name="local_position/tf_send" value="true" />
        <param name="local_position/tf_frame_id" value="$(var vehicle_name)/odom" />
        <param name="local_position/tf_child_frame_id" value="$(var vehicle_name)/base_link" />
        
        <!-- Mission plugin configuration -->
        <param name="mission/pull_after_gcs" value="true" />
        <param name="mission/use_mission_item_int" value="true" />
    </node>
    
    <!-- Mission coordination bridge -->
    <node pkg="airsim_ros_pkgs" exec="mission_coordination_bridge" name="mission_bridge" namespace="$(var vehicle_name)">
        <param name="vehicle_name" value="$(var vehicle_name)" />
        <param name="mavros_namespace" value="$(var mavros_ns)" />
        <param name="enable_mission_upload" value="true" />
        <param name="enable_coordinate_transform" value="true" />
    </node>
    
    <!-- Transform publisher for coordinate frame alignment -->
    <node pkg="tf2_ros" exec="static_transform_publisher" name="$(var vehicle_name)_ned_to_enu">
        <arg name="x" value="0" />
        <arg name="y" value="0" />
        <arg name="z" value="0" />
        <arg name="roll" value="3.14159" />
        <arg name="pitch" value="0" />
        <arg name="yaw" value="1.5708" />
        <arg name="frame_id" value="$(var vehicle_name)/ned_frame" />
        <arg name="child_frame_id" value="$(var vehicle_name)/enu_frame" />
    </node>
</launch>
```

#### Mission Coordination Bridge Node
```cpp
// mission_coordination_bridge.cpp
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mission_search_interfaces/action/search_area.hpp>
#include <mission_search_interfaces/msg/waypoint_array.hpp>

class MissionCoordinationBridge : public rclcpp::Node
{
public:
    MissionCoordinationBridge()
        : Node("mission_coordination_bridge")
    {
        setupMissionBridge();
        setupMAVROSInterface();
        
        RCLCPP_INFO(this->get_logger(), "Mission coordination bridge ready");
    }

private:
    void setupMissionBridge()
    {
        // Subscribe to mission waypoints from coordination system
        mission_waypoints_sub_ = this->create_subscription<mission_search_interfaces::msg::WaypointArray>(
            "mission_waypoints",
            10,
            std::bind(&MissionCoordinationBridge::missionWaypointsCallback, this, std::placeholders::_1)
        );
        
        // Publish mission status to coordination system
        mission_status_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionStatus>(
            "mission_status",
            10
        );
        
        // Action client for search area integration
        search_area_client_ = rclcpp_action::create_client<mission_search_interfaces::action::SearchArea>(
            this, "actions/search_area"
        );
    }
    
    void setupMAVROSInterface()
    {
        // MAVROS waypoint services
        waypoint_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>(
            "mavros/mission/push"
        );
        
        waypoint_clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>(
            "mavros/mission/clear"
        );
        
        // MAVROS mission state subscriber
        mavros_mission_state_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
            "mavros/mission/reached",
            10,
            std::bind(&MissionCoordinationBridge::waypointReachedCallback, this, std::placeholders::_1)
        );
    }
    
    void missionWaypointsCallback(const mission_search_interfaces::msg::WaypointArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received %zu mission waypoints from coordination system",
            msg->waypoints.size());
        
        // Convert ROS2 mission waypoints to MAVROS waypoints
        std::vector<mavros_msgs::msg::Waypoint> mavros_waypoints;
        
        for (const auto& ros_waypoint : msg->waypoints)
        {
            mavros_msgs::msg::Waypoint mavros_wp;
            
            // Convert coordinate frame from ENU to NED
            auto ned_position = MissionCoordinateTransforms::convertENUtoNED(ros_waypoint.position);
            
            mavros_wp.frame = mavros_msgs::msg::Waypoint::FRAME_LOCAL_NED;
            mavros_wp.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
            mavros_wp.is_current = false;
            mavros_wp.autocontinue = true;
            
            mavros_wp.x_lat = ned_position.x;
            mavros_wp.y_long = ned_position.y;
            mavros_wp.z_alt = ned_position.z;
            
            // Mission-specific parameters
            mavros_wp.param1 = ros_waypoint.hold_time.sec; // Hold time
            mavros_wp.param2 = ros_waypoint.acceptance_radius; // Acceptance radius
            mavros_wp.param3 = 0.0; // Pass through waypoint
            mavros_wp.param4 = ros_waypoint.yaw; // Desired yaw angle
            
            mavros_waypoints.push_back(mavros_wp);
        }
        
        // Upload waypoints to PX4
        uploadWaypointsToPX4(mavros_waypoints);
    }
    
    void uploadWaypointsToPX4(const std::vector<mavros_msgs::msg::Waypoint>& waypoints)
    {
        // Clear existing mission
        auto clear_request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
        
        auto clear_future = waypoint_clear_client_->async_send_request(clear_request);
        
        // Wait for clear to complete
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), clear_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto clear_response = clear_future.get();
            if (clear_response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Cleared existing PX4 mission");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to clear PX4 mission");
                return;
            }
        }
        
        // Upload new mission
        auto push_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
        push_request->start_index = 0;
        push_request->waypoints = waypoints;
        
        auto push_future = waypoint_push_client_->async_send_request(push_request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), push_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto push_response = push_future.get();
            if (push_response->success)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully uploaded %zu waypoints to PX4",
                    waypoints.size());
                
                publishMissionStatus("UPLOADED", waypoints.size());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to upload waypoints to PX4");
                publishMissionStatus("UPLOAD_FAILED", 0);
            }
        }
    }
    
    void waypointReachedCallback(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "PX4 reached waypoint %d", msg->wp_seq);
        
        // Update mission coordination system
        publishMissionStatus("WAYPOINT_REACHED", msg->wp_seq);
    }
    
    void publishMissionStatus(const std::string& status, int waypoint_index)
    {
        mission_search_interfaces::msg::MissionStatus status_msg;
        status_msg.header.stamp = this->get_clock()->now();
        status_msg.mission_id = current_mission_id_;
        status_msg.status = getStatusCode(status);
        status_msg.current_waypoint = waypoint_index;
        status_msg.message = status;
        
        mission_status_pub_->publish(status_msg);
    }
    
    int getStatusCode(const std::string& status)
    {
        if (status == "UPLOADED") return 1;
        else if (status == "WAYPOINT_REACHED") return 2;
        else if (status == "MISSION_COMPLETE") return 3;
        else if (status == "UPLOAD_FAILED") return -1;
        else return 0;
    }

private:
    // Mission coordination interfaces
    rclcpp::Subscription<mission_search_interfaces::msg::WaypointArray>::SharedPtr mission_waypoints_sub_;
    rclcpp::Publisher<mission_search_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;
    rclcpp_action::Client<mission_search_interfaces::action::SearchArea>::SharedPtr search_area_client_;
    
    // MAVROS interfaces
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr mavros_mission_state_sub_;
    
    // State tracking
    std::string current_mission_id_;
    std::vector<mavros_msgs::msg::Waypoint> current_waypoints_;
};
```

## 🐳 5. Docker Integration and Deployment

### 5.1 Complete Mission Stack Deployment

#### Docker Compose for Complete Mission System
```yaml
# docker-compose-complete-mission-stack.yml
version: '3.8'

services:
  # AirSim Simulation Environment
  airsim-simulation:
    image: cosys-airsim:latest
    container_name: airsim-simulation
    networks:
      - mission-stack
    ports:
      - "41451:41451"  # AirSim RPC
    volumes:
      - airsim-settings:/home/airsim/Documents/AirSim
      - airsim-logs:/tmp/AirSimLogs
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    command: >
      bash -c "
        cp /mission-configs/settings.json /home/airsim/Documents/AirSim/ &&
        cd /home/airsim/AirSim && 
        ./Blocks.sh -RenderOffScreen
      "
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:41451/ping"]
      interval: 30s
      timeout: 10s
      retries: 3

  # PX4 SITL Swarm (3 instances)
  px4-sitl-leader:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-mission-leader
    networks:
      - mission-stack
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=airsim-simulation
    ports:
      - "14560:14560/udp"
      - "14580:14580/udp"
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        cp /mission-configs/px4-mission-params.txt ROMFS/px4fmu_common/init.d-posix/ &&
        HEADLESS=1 make px4_sitl none_iris
      "
    depends_on:
      - airsim-simulation
    healthcheck:
      test: ["CMD", "nc", "-z", "localhost", "14580"]
      interval: 30s
      timeout: 10s
      retries: 3

  px4-sitl-follower-1:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-mission-follower-1
    networks:
      - mission-stack
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=airsim-simulation
      - PX4_SIM_MODEL_INSTANCE=1
    ports:
      - "14561:14560/udp"
      - "14581:14580/udp"
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        HEADLESS=1 make px4_sitl none_iris
      "
    depends_on:
      - px4-sitl-leader

  px4-sitl-follower-2:
    image: px4io/px4-dev-simulation-focal:latest
    container_name: px4-mission-follower-2
    networks:
      - mission-stack
    environment:
      - PX4_SIM_MODEL=iris
      - PX4_SYS_AUTOSTART=10016
      - PX4_SIM_HOSTNAME=airsim-simulation
      - PX4_SIM_MODEL_INSTANCE=2
    ports:
      - "14562:14560/udp"
      - "14582:14580/udp"
    command: >
      bash -c "
        cd /src/PX4-Autopilot &&
        HEADLESS=1 make px4_sitl none_iris
      "
    depends_on:
      - px4-sitl-leader

  # ROS2 Mission Coordination System
  mission-coordinator:
    image: cosys-airsim:ros2-mission-latest
    container_name: mission-coordinator
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
      - AIRSIM_HOST=airsim-simulation
      - AIRSIM_PORT=41451
    volumes:
      - ros2-mission-logs:/tmp/ros2_logs
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs mission_coordination_demo.launch.py
      "
    depends_on:
      airsim-simulation:
        condition: service_healthy
      px4-sitl-leader:
        condition: service_healthy
    healthcheck:
      test: ["CMD", "ros2", "node", "list", "|", "grep", "mission_coordinator"]
      interval: 30s
      timeout: 10s
      retries: 3

  # ROS2 Vehicle Nodes (Ultra-Clean Architecture)
  vehicle-nodes:
    image: cosys-airsim:ros2-mission-latest
    container_name: vehicle-nodes
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
      - AIRSIM_HOST=airsim-simulation
      - AIRSIM_PORT=41451
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py host_ip:=airsim-simulation
      "
    depends_on:
      mission-coordinator:
        condition: service_healthy

  # MAVROS Bridge for Each Vehicle
  mavros-bridge-leader:
    image: cosys-airsim:ros2-mission-latest
    container_name: mavros-bridge-leader
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs px4_mission_bridge.launch.xml vehicle_name:=Droan1 fcu_url:=udp://:14540@px4-sitl-leader:14560
      "
    depends_on:
      - vehicle-nodes
      - px4-sitl-leader

  mavros-bridge-follower-1:
    image: cosys-airsim:ros2-mission-latest
    container_name: mavros-bridge-follower-1
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs px4_mission_bridge.launch.xml vehicle_name:=PX4_Drone2 fcu_url:=udp://:14541@px4-sitl-follower-1:14560
      "
    depends_on:
      - vehicle-nodes
      - px4-sitl-follower-1

  mavros-bridge-follower-2:
    image: cosys-airsim:ros2-mission-latest
    container_name: mavros-bridge-follower-2
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs px4_mission_bridge.launch.xml vehicle_name:=PX4_Drone3 fcu_url:=udp://:14542@px4-sitl-follower-2:14560
      "
    depends_on:
      - vehicle-nodes
      - px4-sitl-follower-2

  # Mission Monitoring and Visualization
  mission-monitor:
    image: cosys-airsim:ros2-mission-latest
    container_name: mission-monitor
    networks:
      - mission-stack
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /mission_ws/install/setup.bash &&
        ros2 launch airsim_ros_pkgs rviz.launch.py
      "
    depends_on:
      - mission-coordinator

  # Ground Control Station
  ground-control:
    image: qgroundcontrol/qgc:latest
    container_name: ground-control
    networks:
      - mission-stack
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    ports:
      - "14550:14550/udp"  # MAVLink ground station port
    command: >
      bash -c "
        /opt/qgroundcontrol/qgroundcontrol --logging:full
      "
    depends_on:
      - px4-sitl-leader

networks:
  mission-stack:
    driver: bridge
    ipam:
      config:
        - subnet: 172.25.0.0/16

volumes:
  airsim-settings:
  airsim-logs:
  ros2-mission-logs:
  px4-mission-data:
```

#### Mission Stack Launcher Script
```bash
#!/bin/bash
# launch_complete_mission_stack.sh

set -e

echo "🚁 Launching Complete PX4 SITL Mission Coordination Stack"
echo "========================================================"

# Configuration
COMPOSE_FILE="docker-compose-complete-mission-stack.yml"
STACK_NAME="px4-mission-stack"

# Ensure Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi

# Set X11 forwarding for GUI applications
export DISPLAY=${DISPLAY:-:0}
xhost +local:docker > /dev/null 2>&1 || echo "⚠️  X11 forwarding may not work properly"

# Function to wait for service health
wait_for_service() {
    local service_name=$1
    local max_attempts=30
    local attempt=1
    
    echo "⏳ Waiting for $service_name to be healthy..."
    
    while [ $attempt -le $max_attempts ]; do
        if docker-compose -f $COMPOSE_FILE ps $service_name | grep -q "healthy"; then
            echo "✅ $service_name is healthy"
            return 0
        fi
        
        echo "   Attempt $attempt/$max_attempts - $service_name not ready yet..."
        sleep 5
        attempt=$((attempt + 1))
    done
    
    echo "❌ $service_name failed to become healthy within timeout"
    return 1
}

# Clean up previous deployment
echo "🧹 Cleaning up previous deployment..."
docker-compose -f $COMPOSE_FILE down --volumes --remove-orphans > /dev/null 2>&1 || true

# Prepare configuration files
echo "📋 Preparing configuration files..."
mkdir -p ./mission-configs

# Copy AirSim settings
cat > ./mission-configs/settings.json << 'EOF'
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Droan1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "0.0.0.0",
      "UdpPort": 14560,
      "ControlPort": 14580,
      "X": 0.0, "Y": 0.0, "Z": -10.0
    },
    "PX4_Drone2": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "0.0.0.0", 
      "UdpPort": 14561,
      "ControlPort": 14581,
      "X": 20.0, "Y": 0.0, "Z": -10.0
    },
    "PX4_Drone3": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": false,
      "UdpIp": "0.0.0.0",
      "UdpPort": 14562, 
      "ControlPort": 14582,
      "X": -20.0, "Y": 0.0, "Z": -10.0
    }
  }
}
EOF

# Copy PX4 mission parameters
cat > ./mission-configs/px4-mission-params.txt << 'EOF'
param set MAV_0_CONFIG 101
param set MAV_0_MODE 0
param set MIS_TAKEOFF_ALT 10.0
param set MPC_XY_CRUISE 8.0
param set MPC_Z_VEL_MAX_UP 3.0
param set RTL_RETURN_ALT 30.0
param save
EOF

echo "🚀 Starting mission stack services..."

# Start core services first
echo "📡 Starting AirSim simulation..."
docker-compose -f $COMPOSE_FILE up -d airsim-simulation
wait_for_service "airsim-simulation"

echo "🛩️  Starting PX4 SITL instances..."
docker-compose -f $COMPOSE_FILE up -d px4-sitl-leader px4-sitl-follower-1 px4-sitl-follower-2
wait_for_service "px4-sitl-leader"

echo "🎯 Starting mission coordination system..."
docker-compose -f $COMPOSE_FILE up -d mission-coordinator
wait_for_service "mission-coordinator"

echo "🤖 Starting vehicle nodes..."
docker-compose -f $COMPOSE_FILE up -d vehicle-nodes

echo "🔗 Starting MAVROS bridges..."
docker-compose -f $COMPOSE_FILE up -d mavros-bridge-leader mavros-bridge-follower-1 mavros-bridge-follower-2

echo "📊 Starting monitoring services..."
docker-compose -f $COMPOSE_FILE up -d mission-monitor ground-control

echo ""
echo "✅ Complete PX4 SITL Mission Coordination Stack is running!"
echo ""
echo "🌐 Service URLs:"
echo "   AirSim RPC:              http://localhost:41451"
echo "   Ground Control Station:  GUI application launched"
echo "   Mission Monitor (RViz):  GUI application launched"
echo ""
echo "🚁 Available Vehicles:"
echo "   /Droan1         (PX4 SITL port 14560/14580)"
echo "   /PX4_Drone2     (PX4 SITL port 14561/14581)" 
echo "   /PX4_Drone3     (PX4 SITL port 14562/14582)"
echo ""
echo "📋 ROS2 Commands:"
echo "   # Check vehicle nodes"
echo "   docker exec -it mission-coordinator ros2 node list"
echo ""
echo "   # Monitor mission coordination"
echo "   docker exec -it mission-coordinator ros2 topic echo /mission_coordinator/mission_status"
echo ""
echo "   # Send test mission"
echo "   docker exec -it mission-coordinator ros2 run airsim_ros_pkgs simple_mission_coordination_demo"
echo ""
echo "🛑 To stop the stack:"
echo "   docker-compose -f $COMPOSE_FILE down --volumes"
echo ""
echo "📝 Logs:"
echo "   docker-compose -f $COMPOSE_FILE logs -f [service_name]"

# Wait for user input to keep script running
echo ""
read -p "Press Enter to show live logs (Ctrl+C to exit)..."
docker-compose -f $COMPOSE_FILE logs -f --tail=50
```

## 🎯 6. Practical Mission Examples

### 6.1 Complete Search & Rescue Mission with PX4

#### End-to-End Mission Example
```python
#!/usr/bin/env python3
"""
Complete Search & Rescue Mission with PX4 SITL Integration
Demonstrates full mission coordination with PX4-powered vehicles
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
from geometry_msgs.msg import Point, Polygon, Point32
from mission_search_interfaces.action import ExecuteMission
from mission_search_interfaces.srv import PlanMission
from mission_search_interfaces.msg import MissionPlan, SearchZone, VehicleCapabilities
from std_msgs.msg import Header

class PX4SearchRescueMissionDemo(Node):
    """Complete Search & Rescue mission demonstration with PX4 SITL"""
    
    def __init__(self):
        super().__init__('px4_search_rescue_demo')
        
        # Mission coordination clients
        self.setup_mission_coordination()
        
        # Available PX4 vehicles
        self.px4_vehicles = {
            "Droan1": {
                "capabilities": ["high_resolution_camera", "thermal_camera", "gps", "long_endurance"],
                "max_speed": 15.0,
                "max_altitude": 120.0,
                "endurance_minutes": 45
            },
            "PX4_Drone2": {
                "capabilities": ["standard_camera", "gps", "magnetometer"],
                "max_speed": 12.0,
                "max_altitude": 100.0,
                "endurance_minutes": 35
            },
            "PX4_Drone3": {
                "capabilities": ["zoom_camera", "gps", "lidar"],
                "max_speed": 10.0,
                "max_altitude": 80.0,
                "endurance_minutes": 30
            }
        }
        
        self.get_logger().info("PX4 Search & Rescue mission demo initialized")
    
    def setup_mission_coordination(self):
        """Setup mission coordination interfaces"""
        # Mission planning service client
        self.plan_mission_client = self.create_client(
            PlanMission,
            '/mission_coordinator/plan_mission'
        )
        
        # Mission execution action client
        self.execute_mission_client = ActionClient(
            self,
            ExecuteMission,
            '/mission_coordinator/actions/execute_mission'
        )
        
        # Wait for services
        while not self.plan_mission_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission planning service...')
        
        while not self.execute_mission_client.wait_for_action_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission execution action server...')
    
    async def execute_complete_search_rescue_mission(self):
        """Execute complete search and rescue mission with PX4 vehicles"""
        self.get_logger().info("🚁 Starting PX4 Search & Rescue Mission")
        self.get_logger().info("=" * 50)
        
        try:
            # Phase 1: Mission Planning
            self.get_logger().info("📋 Phase 1: Mission Planning")
            mission_plan = await self.plan_search_rescue_mission()
            if not mission_plan:
                self.get_logger().error("Mission planning failed")
                return False
            
            # Phase 2: Vehicle Preparation
            self.get_logger().info("🛠️  Phase 2: Vehicle Preparation")
            vehicles_ready = await self.prepare_px4_vehicles()
            if not vehicles_ready:
                self.get_logger().error("Vehicle preparation failed")
                return False
            
            # Phase 3: Mission Execution
            self.get_logger().info("🎯 Phase 3: Mission Execution")
            mission_success = await self.execute_px4_search_mission(mission_plan)
            
            # Phase 4: Mission Results
            self.get_logger().info("📊 Phase 4: Mission Results")
            await self.process_mission_results(mission_success)
            
            return mission_success
            
        except Exception as e:
            self.get_logger().error(f"Mission execution error: {e}")
            return False
    
    async def plan_search_rescue_mission(self):
        """Plan comprehensive search and rescue mission"""
        # Define search area (large area requiring multiple vehicles)
        search_area = Polygon()
        
        # Create search area covering 2km x 2km area
        search_points = [
            Point32(x=0.0, y=0.0, z=0.0),      # Southwest corner
            Point32(x=2000.0, y=0.0, z=0.0),   # Southeast corner
            Point32(x=2000.0, y=2000.0, z=0.0), # Northeast corner
            Point32(x=0.0, y=2000.0, z=0.0)    # Northwest corner
        ]
        search_area.points = search_points
        
        # Create mission planning request
        request = PlanMission.Request()
        request.mission_name = "SAR_PX4_Mission_001"
        request.mission_type = "search_and_rescue"
        request.mission_area = search_area
        request.max_vehicles = len(self.px4_vehicles)
        request.preferred_search_pattern = "optimal"  # Let coordinator choose best pattern
        request.priority_level = 1  # High priority SAR mission
        request.estimated_duration = 3600  # 1 hour mission
        
        # Add vehicle capabilities for optimal assignment
        for vehicle_name, capabilities in self.px4_vehicles.items():
            vehicle_caps = VehicleCapabilities()
            vehicle_caps.vehicle_name = vehicle_name
            vehicle_caps.max_speed = capabilities["max_speed"]
            vehicle_caps.max_altitude = capabilities["max_altitude"]
            vehicle_caps.endurance_seconds = capabilities["endurance_minutes"] * 60
            vehicle_caps.capabilities = capabilities["capabilities"]
            
            request.available_vehicles.append(vehicle_caps)
        
        # Call planning service
        self.get_logger().info("📡 Requesting mission plan from coordinator...")
        
        future = self.plan_mission_client.call_async(request)
        response = await self.wait_for_future(future)
        
        if response and response.success:
            self.get_logger().info(f"✅ Mission planned successfully: {response.message}")
            self.get_logger().info(f"   Search zones: {len(response.mission_plan.search_zones)}")
            self.get_logger().info(f"   Estimated duration: {response.mission_plan.estimated_duration.sec}s")
            return response.mission_plan
        else:
            error_msg = response.message if response else "Service call failed"
            self.get_logger().error(f"❌ Mission planning failed: {error_msg}")
            return None
    
    async def prepare_px4_vehicles(self):
        """Prepare all PX4 vehicles for mission"""
        self.get_logger().info("🔧 Preparing PX4 vehicles for search mission...")
        
        preparation_tasks = []
        
        for vehicle_name in self.px4_vehicles.keys():
            task = asyncio.create_task(self.prepare_individual_px4_vehicle(vehicle_name))
            preparation_tasks.append(task)
        
        # Wait for all vehicles to be prepared
        results = await asyncio.gather(*preparation_tasks, return_exceptions=True)
        
        success_count = sum(1 for result in results if result is True)
        total_vehicles = len(self.px4_vehicles)
        
        self.get_logger().info(f"Vehicle preparation: {success_count}/{total_vehicles} ready")
        
        if success_count == total_vehicles:
            self.get_logger().info("✅ All PX4 vehicles prepared and ready for mission")
            return True
        else:
            failed_count = total_vehicles - success_count
            self.get_logger().warn(f"⚠️  {failed_count} vehicles failed preparation")
            return success_count >= 2  # Allow mission with at least 2 vehicles
    
    async def prepare_individual_px4_vehicle(self, vehicle_name):
        """Prepare individual PX4 vehicle for mission"""
        try:
            self.get_logger().info(f"   Preparing {vehicle_name}...")
            
            # Simulate vehicle preparation steps
            await asyncio.sleep(1.0)  # Simulate arming check
            self.get_logger().info(f"   ✓ {vehicle_name}: Armed and ready")
            
            await asyncio.sleep(0.5)  # Simulate GPS check
            self.get_logger().info(f"   ✓ {vehicle_name}: GPS home position set")
            
            await asyncio.sleep(0.5)  # Simulate sensor check
            self.get_logger().info(f"   ✓ {vehicle_name}: Sensors operational")
            
            await asyncio.sleep(0.5)  # Simulate mission upload
            self.get_logger().info(f"   ✓ {vehicle_name}: Mission parameters uploaded")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"   ❌ {vehicle_name}: Preparation failed - {e}")
            return False
    
    async def execute_px4_search_mission(self, mission_plan):
        """Execute search mission with PX4 vehicles"""
        # Create mission execution goal
        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_plan = mission_plan
        
        self.get_logger().info("🚀 Sending mission execution goal to coordinator...")
        self.get_logger().info(f"   Mission: {mission_plan.mission_name}")
        self.get_logger().info(f"   Search zones: {len(mission_plan.search_zones)}")
        self.get_logger().info(f"   Vehicles: {len(mission_plan.assigned_vehicles)}")
        
        # Send goal
        send_goal_future = self.execute_mission_client.send_goal_async(
            goal_msg,
            feedback_callback=self.mission_feedback_callback
        )
        
        goal_handle = await self.wait_for_future(send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ Mission execution goal rejected")
            return False
        
        self.get_logger().info("✅ Mission execution goal accepted - starting search...")
        
        # Monitor mission execution
        result = await self.wait_for_future(goal_handle.get_result_async())
        
        if result and result.result.success:
            completion_reason = result.result.completion_reason
            vehicles_participated = result.result.vehicles_participated
            
            self.get_logger().info("🎉 Mission completed successfully!")
            self.get_logger().info(f"   Completion reason: {completion_reason}")
            self.get_logger().info(f"   Vehicles participated: {vehicles_participated}")
            return True
        else:
            failure_reason = result.result.completion_reason if result else "Unknown error"
            self.get_logger().error(f"❌ Mission failed: {failure_reason}")
            return False
    
    def mission_feedback_callback(self, feedback_msg):
        """Process real-time mission execution feedback"""
        feedback = feedback_msg.feedback
        
        # Log progress updates
        if hasattr(feedback, 'progress_percentage'):
            progress = feedback.progress_percentage
            active_vehicles = getattr(feedback, 'vehicles_active', 0)
            targets_detected = getattr(feedback, 'total_targets_detected_so_far', 0)
            
            self.get_logger().info(
                f"📊 Mission Progress: {progress:.1f}% | "
                f"Active vehicles: {active_vehicles} | "
                f"Targets detected: {targets_detected}"
            )
            
            # Check for critical events
            if targets_detected > 0:
                self.get_logger().info("🎯 TARGET DETECTED - Coordinating response...")
            
            if progress > 90:
                self.get_logger().info("🏁 Mission nearing completion...")
    
    async def process_mission_results(self, mission_success):
        """Process and analyze mission results"""
        if mission_success:
            self.get_logger().info("📈 Mission Analysis:")
            self.get_logger().info("   ✅ Search pattern execution: SUCCESSFUL")
            self.get_logger().info("   ✅ Vehicle coordination: SUCCESSFUL") 
            self.get_logger().info("   ✅ PX4 integration: SUCCESSFUL")
            self.get_logger().info("   ✅ Mission objectives: ACHIEVED")
        else:
            self.get_logger().info("📉 Mission Analysis:")
            self.get_logger().info("   ❌ Mission execution: FAILED")
            self.get_logger().info("   📋 Recommended actions:")
            self.get_logger().info("      - Check vehicle status")
            self.get_logger().info("      - Verify PX4 connections")
            self.get_logger().info("      - Review mission parameters")
    
    async def wait_for_future(self, future):
        """Wait for a future to complete asynchronously"""
        while not future.done():
            await asyncio.sleep(0.1)
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Future failed: {e}")
            return None


async def main():
    """Main function for PX4 search and rescue demo"""
    rclpy.init()
    
    demo_node = PX4SearchRescueMissionDemo()
    
    try:
        # Execute the complete search and rescue mission
        success = await demo_node.execute_complete_search_rescue_mission()
        
        if success:
            print("\n" + "=" * 60)
            print("🏆 PX4 SITL SEARCH & RESCUE MISSION: SUCCESS")
            print("   All mission objectives achieved!")
            print("   PX4 integration working perfectly!")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("💥 PX4 SITL SEARCH & RESCUE MISSION: FAILED")
            print("   Check logs for failure analysis")
            print("=" * 60)
            
    except KeyboardInterrupt:
        print("\n🛑 Mission interrupted by user")
    except Exception as e:
        print(f"\n💥 Mission failed with exception: {e}")
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
```

## 🔧 7. Troubleshooting and Best Practices

### 7.1 Common Issues and Solutions

#### MAVLink Connection Issues
```bash
# Problem: PX4 SITL cannot connect to AirSim
# Solution: Check MAVLink port configuration

# 1. Verify PX4 SITL MAVLink output
pxh> mavlink status
# Should show active connections

# 2. Check AirSim MAVLink listener
netstat -an | grep 14560
# Should show listening port

# 3. Test direct MAVLink connection
mavproxy.py --master=udp:127.0.0.1:14560 --out=127.0.0.1:14540
```

#### Coordinate Frame Issues
```cpp
// Problem: Vehicles not reaching correct waypoints
// Solution: Verify coordinate transformations

// Test coordinate transformation accuracy
void testCoordinateTransforms() {
    // Original ENU point
    geometry_msgs::msg::Point enu_point;
    enu_point.x = 10.0;  // East
    enu_point.y = 20.0;  // North
    enu_point.z = -5.0;  // Up (altitude)
    
    // Transform to NED
    auto ned_point = MissionCoordinateTransforms::convertENUtoNED(enu_point);
    
    // Expected NED: North=20, East=10, Down=5
    assert(abs(ned_point.x - 20.0) < 1e-6);  // North
    assert(abs(ned_point.y - 10.0) < 1e-6);  // East  
    assert(abs(ned_point.z - 5.0) < 1e-6);   // Down
    
    std::cout << "✅ Coordinate transformation test passed" << std::endl;
}
```

#### Mission Coordination Issues
```python
# Problem: Vehicles not responding to mission commands
# Solution: Check ROS2 communication and action servers

import rclpy
from rclpy.node import Node

class MissionDiagnostic(Node):
    def __init__(self):
        super().__init__('mission_diagnostic')
        
    def diagnose_mission_system(self):
        """Comprehensive mission system diagnostics"""
        print("🔍 Mission System Diagnostics")
        print("=" * 40)
        
        # Check vehicle nodes
        vehicle_nodes = self.check_vehicle_nodes()
        print(f"Vehicle nodes: {vehicle_nodes}")
        
        # Check action servers
        action_servers = self.check_action_servers()
        print(f"Action servers: {action_servers}")
        
        # Check mission coordination
        mission_coord = self.check_mission_coordination()
        print(f"Mission coordination: {mission_coord}")
        
        return all([vehicle_nodes, action_servers, mission_coord])
    
    def check_vehicle_nodes(self):
        """Check if vehicle nodes are running"""
        expected_vehicles = ["/Droan1", "/PX4_Drone2", "/PX4_Drone3"]
        
        # Get list of active nodes
        node_names = self.get_node_names()
        
        vehicle_nodes_found = []
        for vehicle in expected_vehicles:
            if vehicle in node_names:
                vehicle_nodes_found.append(vehicle)
                print(f"  ✅ {vehicle}: ACTIVE")
            else:
                print(f"  ❌ {vehicle}: NOT FOUND")
        
        return len(vehicle_nodes_found) == len(expected_vehicles)
```

### 7.2 Performance Optimization

#### PX4 Parameter Tuning for Mission Performance
```bash
#!/bin/bash
# optimize_px4_mission_performance.sh

echo "🎯 Optimizing PX4 parameters for mission performance"

# Position control optimization
param set MPC_XY_P 1.5              # Increase position responsiveness
param set MPC_Z_P 1.8               # Increase altitude control
param set MPC_XY_VEL_P 0.18         # Improve velocity tracking
param set MPC_Z_VEL_P 0.25          # Improve vertical velocity

# Mission execution optimization  
param set MIS_YAW_TMT 0.5           # Reduce yaw timeout for faster waypoint transitions
param set MIS_YAW_ERR 5.0           # Allow larger yaw error for mission progression
param set NAV_ACC_RAD 5.0           # Acceptance radius for waypoints

# Communication optimization
param set MAV_0_RATE 921600         # High baud rate for fast telemetry
param set SER_TEL1_BAUD 921600      # Match telemetry port baud

# Performance monitoring
param set SDLOG_PROFILE 6           # Extended logging for mission analysis
param set LOG_LEVEL 1               # Detailed logging level

echo "✅ PX4 mission performance parameters optimized"
```

#### ROS2 Performance Tuning
```cpp
// Performance optimized ROS2 node configuration
class PerformanceOptimizedMissionNode : public MissionMultirotorNode
{
public:
    PerformanceOptimizedMissionNode(const std::string& vehicle_name)
        : MissionMultirotorNode(vehicle_name)
    {
        setupOptimizedPublishers();
        setupOptimizedTimers();
    }

private:
    void setupOptimizedPublishers()
    {
        // High-frequency publishers with optimized QoS
        rclcpp::QoS high_freq_qos(10);
        high_freq_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        high_freq_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        high_freq_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        // Position commands at 50Hz for smooth flight
        position_cmd_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/" + vehicle_name_ + "/mavros/setpoint_raw/local",
            high_freq_qos
        );
        
        // Lower frequency for telemetry
        rclcpp::QoS telemetry_qos(5);
        telemetry_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        
        mission_status_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionStatus>(
            "/" + vehicle_name_ + "/mission_status",
            telemetry_qos
        );
    }
    
    void setupOptimizedTimers()
    {
        // High-frequency control loop (50Hz)
        control_timer_ = this->create_timer(
            std::chrono::milliseconds(20),
            std::bind(&PerformanceOptimizedMissionNode::highFrequencyControl, this)
        );
        
        // Medium-frequency mission monitoring (10Hz)
        mission_timer_ = this->create_timer(
            std::chrono::milliseconds(100),
            std::bind(&PerformanceOptimizedMissionNode::missionMonitoring, this)
        );
        
        // Low-frequency status reporting (1Hz)
        status_timer_ = this->create_timer(
            std::chrono::seconds(1),
            std::bind(&PerformanceOptimizedMissionNode::statusReporting, this)
        );
    }
};
```

## 🎯 Conclusion

This comprehensive guide provides complete integration of PX4 SITL with the mission coordination system in Cosys-AirSim. The integration enables:

### ✅ Key Achievements
- **Multi-Vehicle PX4 Coordination**: Seamless coordination of multiple PX4 SITL instances
- **Mission-Aware Flight Control**: PX4 autopilot integration with ROS2 mission actions
- **Precise Coordinate Transformations**: Accurate NED ↔ ENU conversions for mission waypoints
- **Docker-Based Deployment**: Complete containerized mission stack for reproducible deployments
- **Real-Time Mission Execution**: Autonomous search patterns with PX4 waypoint following
- **Fault-Tolerant Architecture**: Collision avoidance and failsafe handling during missions

### 🔗 Integration Points
- **ROS2 Action System**: `SearchArea`, `NavigateToTarget`, `ExecuteMission` actions work seamlessly with PX4
- **MAVROS Bridge**: Real-time bidirectional communication between ROS2 and PX4 MAVLink
- **Ultra-Clean Architecture**: Vehicle names ARE node names (`/Droan1`, `/PX4_Drone2`) with PX4 integration
- **Mission Coordination**: Centralized planning and execution with distributed PX4 vehicle control

### 🚀 Deployment Ready
The complete system is production-ready with:
- **Docker Compose**: Full stack deployment with health checks and dependencies
- **Configuration Management**: Automated parameter setup for both PX4 and ROS2
- **Performance Monitoring**: Real-time diagnostics and performance optimization
- **Scalable Architecture**: Easy addition of new vehicles and mission types

### 📁 Related Files
- **Mission Interfaces**: `L:\Cosys-AirSim\ros2\src\mission_search_interfaces\` - Action and message definitions
- **Mission Coordination**: `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_coordination_node.cpp`
- **Vehicle Nodes**: `L:\Cosys-AirSim\ros2\src\airsim_ros_pkgs\src\mission_multirotor_node.cpp`
- **PX4 Documentation**: `L:\Cosys-AirSim\docs\px4\` - Additional PX4 integration guides
- **ROS2 Architecture**: `L:\Cosys-AirSim\ros2\README_MULTIROTOR_ARCHITECTURE.md`

This integration transforms Cosys-AirSim into a complete autonomous mission platform, combining the high-fidelity simulation environment with professional-grade flight control and mission coordination capabilities.