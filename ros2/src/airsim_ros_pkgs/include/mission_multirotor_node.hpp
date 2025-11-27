#pragma once

#include "multirotor_node.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

// Mission search interfaces
#include <mission_search_interfaces/action/search_area.hpp>
#include <mission_search_interfaces/action/navigate_to_target.hpp>
#include <mission_search_interfaces/action/track_target.hpp>
#include <mission_search_interfaces/srv/set_search_pattern.hpp>
#include <mission_search_interfaces/srv/get_vehicle_capabilities.hpp>
#include <mission_search_interfaces/msg/mission_status.hpp>
#include <mission_search_interfaces/msg/target_detection.hpp>
#include <mission_search_interfaces/msg/vehicle_capabilities.hpp>
#include <mission_search_interfaces/msg/waypoint_array.hpp>
#include <mission_search_interfaces/msg/mission_event.hpp>

// Standard ROS2 messages for geometry and navigation
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

// Service interfaces for event-generating flight services
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>


// STL containers and utilities
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <future>
#include <optional>
#include <algorithm>

using namespace std::chrono_literals;

/**
 * @brief Mission-capable multirotor node extending ultra-clean architecture
 * 
 * Extends MultirotorNode with mission-based search and rescue capabilities:
 * - Action servers for long-running missions (SearchArea, NavigateToTarget, TrackTarget)
 * - Services for mission configuration and capability queries  
 * - Topics for mission status and target detection publishing
 * - Ultra-clean naming: /VehicleName/actions/search_area, /VehicleName/services/set_search_pattern
 */
class MissionMultirotorNode : public MultirotorNode
{
public:
    using SearchAreaAction = mission_search_interfaces::action::SearchArea;
    using NavigateToTargetAction = mission_search_interfaces::action::NavigateToTarget;
    using TrackTargetAction = mission_search_interfaces::action::TrackTarget;
    
    using SearchAreaActionServer = rclcpp_action::Server<SearchAreaAction>;
    using NavigateToTargetActionServer = rclcpp_action::Server<NavigateToTargetAction>;
    using TrackTargetActionServer = rclcpp_action::Server<TrackTargetAction>;

    /**
     * @brief Constructor following ultra-clean naming conventions
     * @param vehicle_name Vehicle name that becomes node name (e.g., "Droan1")
     * @param host_ip AirSim server IP address
     * @param host_port AirSim server port
     */
    MissionMultirotorNode(const std::string& vehicle_name, 
                         const std::string& host_ip = "localhost", 
                         uint16_t host_port = 41451);
    
    virtual ~MissionMultirotorNode() = default;

private:
    // === INITIALIZATION METHODS ===
    
    /**
     * @brief Initialize mission-specific publishers, services, and action servers
     * Called after base class initialization
     */
    void initialize_mission_capabilities();
    
    /**
     * @brief Setup mission-related publishers following ultra-clean naming
     * Topics: /VehicleName/mission/status, /VehicleName/detections/target
     */
    void setup_mission_publishers();
    
    /**
     * @brief Setup mission services following ultra-clean naming
     * Services: /VehicleName/services/set_search_pattern, /VehicleName/services/get_capabilities
     */
    void setup_mission_services();
    
    /**
     * @brief Setup action servers for long-running mission tasks
     * Actions: /VehicleName/actions/search_area, /VehicleName/actions/navigate_to_target, etc.
     */
    void setup_mission_action_servers();
    
    /**
     * @brief Initialize mission event detection parameters and state tracking
     */
    void initialize_event_detection();

    // === VIRTUAL METHOD OVERRIDES ===

    /**
     * @brief Override to prevent base class from creating flight services
     * Mission node will create its own event-generating versions
     */
    bool should_create_flight_services() const override { return false; }

    /**
     * @brief Override base class state change processing for mission event detection
     */
    void process_state_changes() override;

    // === ENHANCED FLIGHT SERVICES WITH EVENT GENERATION ===

    /**
     * @brief Setup mission node's own takeoff/land services with event generation
     */
    void setup_mission_flight_services();

    /**
     * @brief Enhanced takeoff service with event generation
     * @param request Takeoff service request
     * @param response Takeoff service response
     * @return True if service handled successfully
     */
    bool mission_takeoff_callback(
        const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);

    /**
     * @brief Enhanced land service with event generation
     * @param request Land service request
     * @param response Land service response
     * @return True if service handled successfully
     */
    bool mission_land_callback(
        const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Land::Response> response);

    // === MISSION EVENT DETECTION ===
    
    /**
     * @brief Detect and publish mission events based on state changes
     * Monitors position, velocity, altitude changes and publishes events
     */
    void detect_and_publish_mission_events();
    
    /**
     * @brief Check for significant position change
     * @param current_pos Current vehicle position
     * @return True if position changed beyond threshold
     */
    bool detect_significant_position_change(const geometry_msgs::msg::Point& current_pos);
    
    /**
     * @brief Check for significant velocity change
     * @param current_vel Current vehicle velocity
     * @return True if velocity changed beyond threshold
     */
    bool detect_significant_velocity_change(const geometry_msgs::msg::Vector3& current_vel);
    
    /**
     * @brief Check for significant altitude change
     * @param current_alt Current vehicle altitude
     * @return True if altitude changed beyond threshold
     */
    bool detect_significant_altitude_change(float current_alt);
    
    /**
     * @brief Classify event type based on change patterns
     * @param change_type Primary change detected
     * @param altitude_change Altitude difference
     * @return Event type string
     */
    std::string classify_event_type(const std::string& change_type, float altitude_change);
    
    /**
     * @brief Determine likely event source (ROS2_ACTION, DIRECT_CLIENT, etc.)
     * @return Event source string
     */
    std::string determine_event_source();
    
    /**
     * @brief Create mission event message with current state
     * @param event_type Type of event detected
     * @return Populated MissionEvent message
     */
    mission_search_interfaces::msg::MissionEvent create_mission_event(const std::string& event_type);
    
    /**
     * @brief Publish mission event message
     * @param event Mission event to publish
     */
    void publish_mission_event(const mission_search_interfaces::msg::MissionEvent& event);

    /**
     * @brief Generate immediate service-based event
     * @param event_type Type of event (TAKEOFF, LANDING)
     * @param event_source Source of event (SERVICE_CALL)
     * @param service_name Name of service called
     * @param call_id Unique identifier for service call
     */
    void generate_service_event(const std::string& event_type, const std::string& event_source,
                               const std::string& service_name, const std::string& call_id);

    // === ACTION SERVER CALLBACKS ===
    
    /**
     * @brief Handle new search area goal requests
     * @param uuid Goal UUID
     * @param goal Search area goal parameters
     * @return Goal response (ACCEPT/REJECT)
     */
    rclcpp_action::GoalResponse handle_search_area_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SearchAreaAction::Goal> goal);
    
    /**
     * @brief Handle search area goal cancellation requests
     * @param goal_handle Goal handle to cancel
     * @return Cancel response (ACCEPT/REJECT)
     */
    rclcpp_action::CancelResponse handle_search_area_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle);
    
    /**
     * @brief Handle accepted search area goals - start execution
     * @param goal_handle Accepted goal handle
     */
    void handle_search_area_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle);
    
    /**
     * @brief Execute search area mission in dedicated thread
     * @param goal_handle Goal handle for feedback/result publishing
     */
    void execute_search_area(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle);

    /**
     * @brief Handle navigate to target goal requests
     */
    rclcpp_action::GoalResponse handle_navigate_to_target_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToTargetAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_navigate_to_target_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle);
    
    void handle_navigate_to_target_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle);
    
    void execute_navigate_to_target(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle);

    /**
     * @brief Handle track target goal requests
     */
    rclcpp_action::GoalResponse handle_track_target_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TrackTargetAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_track_target_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle);
    
    void handle_track_target_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle);
    
    void execute_track_target(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle);

    // === SERVICE CALLBACKS ===
    
    /**
     * @brief Configure search pattern for this vehicle
     * Service: /VehicleName/services/set_search_pattern
     */
    bool set_search_pattern_callback(
        const std::shared_ptr<mission_search_interfaces::srv::SetSearchPattern::Request> request,
        std::shared_ptr<mission_search_interfaces::srv::SetSearchPattern::Response> response);
    
    /**
     * @brief Get vehicle capabilities and current status
     * Service: /VehicleName/services/get_capabilities
     */
    bool get_vehicle_capabilities_callback(
        const std::shared_ptr<mission_search_interfaces::srv::GetVehicleCapabilities::Request> request,
        std::shared_ptr<mission_search_interfaces::srv::GetVehicleCapabilities::Response> response);

    // === MISSION EXECUTION METHODS ===
    
    /**
     * @brief Generate waypoints for specified search pattern
     * @param boundary Search area boundary polygon
     * @param pattern_type Pattern type: "spiral", "lawnmower", "grid", "random"
     * @param spacing Distance between search paths
     * @param altitude Search altitude
     * @return Vector of waypoints for the pattern
     */
    std::vector<geometry_msgs::msg::Point> generate_search_waypoints(
        const geometry_msgs::msg::Polygon& boundary,
        const std::string& pattern_type,
        float spacing,
        float altitude);
    
    /**
     * @brief Generate spiral search pattern waypoints
     */
    std::vector<geometry_msgs::msg::Point> generate_spiral_pattern(
        const geometry_msgs::msg::Polygon& boundary,
        float spacing,
        float altitude);
    
    /**
     * @brief Generate lawnmower search pattern waypoints
     */
    std::vector<geometry_msgs::msg::Point> generate_lawnmower_pattern(
        const geometry_msgs::msg::Polygon& boundary,
        float spacing,
        float altitude);
    
    /**
     * @brief Generate grid search pattern waypoints
     */
    std::vector<geometry_msgs::msg::Point> generate_grid_pattern(
        const geometry_msgs::msg::Polygon& boundary,
        float spacing,
        float altitude);
    
    /**
     * @brief Execute waypoint navigation with progress feedback
     * @param waypoints Waypoint sequence to follow
     * @param speed Flight speed
     * @param goal_handle Goal handle for feedback publishing
     * @return True if all waypoints reached successfully
     */
    bool execute_waypoint_navigation(
        const std::vector<geometry_msgs::msg::Point>& waypoints,
        float speed,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle);
    
    /**
     * @brief Perform target detection at current position
     * @return Vector of detected targets
     */
    std::vector<mission_search_interfaces::msg::TargetDetection> perform_target_detection();
    
    /**
     * @brief Calculate area of polygon boundary
     * @param boundary Polygon boundary
     * @return Area in square meters
     */
    float calculate_polygon_area(const geometry_msgs::msg::Polygon& boundary);
    
    /**
     * @brief Get polygon center point
     * @param boundary Polygon boundary
     * @return Center point of polygon
     */
    geometry_msgs::msg::Point get_polygon_center(const geometry_msgs::msg::Polygon& boundary);
    
    /**
     * @brief Navigate smoothly to a waypoint with AirSim API
     * @param waypoint Target waypoint
     * @param speed Flight speed
     * @return True if waypoint reached successfully
     */
    bool navigate_to_waypoint(const geometry_msgs::msg::Point& waypoint, float speed);
    
    /**
     * @brief Check if waypoint is reached within tolerance
     * @param waypoint Target waypoint
     * @param tolerance Distance tolerance in meters
     * @return True if within tolerance
     */
    bool is_waypoint_reached(const geometry_msgs::msg::Point& waypoint, float tolerance = 2.0f);
    
    /**
     * @brief Execute spiral approach to target
     * @param target_point Target position
     * @param speed Flight speed
     * @param radius Spiral radius
     * @return True if approach successful
     */
    bool execute_spiral_approach(const geometry_msgs::msg::Point& target_point, float speed, float radius);
    
    /**
     * @brief Execute target investigation phase
     * @param goal Navigate to target goal
     * @param feedback Feedback message to update
     * @param goal_handle Goal handle for feedback
     * @return True if investigation successful
     */
    bool execute_target_investigation(
        std::shared_ptr<const NavigateToTargetAction::Goal> goal,
        std::shared_ptr<NavigateToTargetAction::Feedback> feedback,
        std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle);

    // === UTILITY METHODS ===
    
    /**
     * @brief Publish mission status update
     * @param status Mission status enum
     * @param progress Progress percentage (0.0-100.0)
     * @param activity Current activity description
     */
    void publish_mission_status(uint8_t status, float progress, const std::string& activity);
    
    /**
     * @brief Publish target detection event
     * @param detection Target detection information
     */
    void publish_target_detection(const mission_search_interfaces::msg::TargetDetection& detection);
    
    /**
     * @brief Get current vehicle capabilities
     * @return VehicleCapabilities message with current status
     */
    mission_search_interfaces::msg::VehicleCapabilities get_current_capabilities();
    
    /**
     * @brief Reset all mission state variables for a new mission
     * Clears targets detected, area covered, waypoints completed, etc.
     */
    void reset_mission_state();
    
    /**
     * @brief Ensure vehicle is ready for mission execution
     * Checks AirSim connection, vehicle state, and performs takeoff if needed
     * @param mission_altitude Desired altitude for mission
     * @return true if vehicle is ready, false if preparation failed
     */
    bool ensure_mission_ready(float mission_altitude);

    /**
     * @brief Call enhanced takeoff service with target altitude
     * Uses the enhanced takeoff service for robust altitude positioning
     * @param target_altitude Desired altitude for mission
     * @return true if takeoff and positioning successful
     */
    bool call_enhanced_takeoff_service(float target_altitude);
    
    /**
     * @brief Check if vehicle needs takeoff and execute if necessary
     * @param target_altitude Altitude to reach
     * @return true if takeoff successful or already airborne
     */
    bool ensure_vehicle_airborne(float target_altitude);
    
    /**
     * @brief Get vehicle state with retry logic for RPC reliability
     * @param client AirSim multirotor client pointer
     * @param max_retries Maximum number of retry attempts
     * @return Vehicle state if successful, nullopt if all retries failed
     */
    std::optional<msr::airlib::MultirotorState> get_vehicle_state_with_retry(
        msr::airlib::MultirotorRpcLibClient* client, int max_retries);
    
    /**
     * @brief Check if vehicle is ready for mission execution
     * @param current_altitude Current vehicle altitude
     * @param mission_altitude Desired mission altitude
     * @return true if vehicle is at appropriate altitude and state
     */
    bool is_vehicle_ready_for_mission(float current_altitude, float mission_altitude);
    
    /**
     * @brief Wait for takeoff completion with timeout
     * @param client AirSim multirotor client pointer
     * @param timeout_seconds Maximum time to wait for takeoff
     * @return true if takeoff completed successfully within timeout
     */
    bool wait_for_takeoff_completion(msr::airlib::MultirotorRpcLibClient* client, int timeout_seconds);

    // === MEMBER VARIABLES ===
    
    // Mission action servers
    std::shared_ptr<SearchAreaActionServer> search_area_action_server_;
    std::shared_ptr<NavigateToTargetActionServer> navigate_to_target_action_server_;
    std::shared_ptr<TrackTargetActionServer> track_target_action_server_;
    
    // Mission services
    rclcpp::Service<mission_search_interfaces::srv::SetSearchPattern>::SharedPtr set_search_pattern_service_;
    rclcpp::Service<mission_search_interfaces::srv::GetVehicleCapabilities>::SharedPtr get_capabilities_service_;
    
    // Mission publishers
    rclcpp::Publisher<mission_search_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<mission_search_interfaces::msg::TargetDetection>::SharedPtr target_detection_pub_;
    rclcpp::Publisher<mission_search_interfaces::msg::MissionEvent>::SharedPtr mission_event_pub_;
    
    // Mission state tracking
    std::atomic<bool> mission_active_{false};
    std::atomic<bool> mission_cancellation_requested_{false};
    std::string current_mission_id_;
    std::string current_activity_;
    float current_mission_progress_ = 0.0f;
    
    // Search configuration
    struct SearchConfig {
        std::string pattern_type = "spiral";
        float search_altitude = 20.0f;
        float search_speed = 5.0f;
        float pattern_spacing = 10.0f;
        bool enable_detection = true;
        float detection_confidence_threshold = 0.5f;
    } search_config_;
    
    // Thread management
    std::thread mission_execution_thread_;
    std::mutex mission_state_mutex_;
    
    // Callback group for mission operations
    rclcpp::CallbackGroup::SharedPtr mission_callback_group_;

    // Enhanced flight services with event generation
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr mission_takeoff_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr mission_land_service_;
    
    // Mission performance tracking (thread-safe)
    std::chrono::steady_clock::time_point mission_start_time_;
    std::atomic<uint32_t> total_targets_detected_{0};
    std::atomic<float> total_area_covered_{0.0f};
    std::atomic<uint32_t> waypoints_completed_{0};
    
    // === MISSION EVENT DETECTION STATE ===
    
    // Previous state tracking for event detection
    geometry_msgs::msg::Point previous_position_;
    geometry_msgs::msg::Vector3 previous_velocity_;
    float previous_altitude_;
    std::chrono::steady_clock::time_point last_event_time_;
    std::chrono::steady_clock::time_point last_significant_movement_;
    uint64_t event_sequence_number_;
    
    // Event detection thresholds (configurable parameters)
    double position_change_threshold_;      // 0.5 meters
    double velocity_change_threshold_;      // 1.0 m/s  
    double altitude_change_threshold_;      // 0.3 meters
    double event_time_threshold_;           // 1.0 seconds between similar events
    
    // Event state tracking
    bool event_detection_initialized_;

    // Service call tracking for event correlation
    struct ServiceCallInfo {
        std::string call_id;
        std::string service_name;
        std::chrono::steady_clock::time_point timestamp;
        bool completed = false;
        bool success = false;
    };
    std::vector<ServiceCallInfo> pending_service_calls_;
    std::mutex service_calls_mutex_;
};