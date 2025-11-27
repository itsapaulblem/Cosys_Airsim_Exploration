#pragma once

#include "coordination_node.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

// Mission search interfaces
#include <mission_search_interfaces/action/execute_mission.hpp>
#include <mission_search_interfaces/srv/plan_mission.hpp>
#include <mission_search_interfaces/srv/assign_search_zone.hpp>
#include <mission_search_interfaces/srv/get_mission_status.hpp>
#include <mission_search_interfaces/msg/mission_plan.hpp>
#include <mission_search_interfaces/msg/mission_status.hpp>
#include <mission_search_interfaces/msg/search_zone.hpp>
#include <mission_search_interfaces/msg/vehicle_capabilities.hpp>

// Standard ROS2 for coordination
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/string.hpp>

// STL containers and utilities
#include <unordered_map>
#include <atomic>
#include <mutex>

/**
 * @brief Mission Coordination Node extending ultra-clean architecture
 * 
 * Provides sophisticated mission orchestration while maintaining simplicity:
 * - Automatically discovers vehicles with ultra-clean naming (/Droan1, /PX4_Drone2)
 * - Assigns search zones based on vehicle capabilities
 * - Coordinates multi-vehicle missions with global oversight
 * - Monitors mission progress and handles task reallocation
 * - Provides single point of control for complex search operations
 */
class MissionCoordinationNode : public CoordinationNode
{
public:
    using ExecuteMissionAction = mission_search_interfaces::action::ExecuteMission;
    using ExecuteMissionActionServer = rclcpp_action::Server<ExecuteMissionAction>;
    
    /**
     * @brief Constructor - initializes mission orchestration capabilities
     */
    MissionCoordinationNode();
    
    virtual ~MissionCoordinationNode() = default;

private:
    // === INITIALIZATION METHODS ===
    
    /**
     * @brief Initialize mission orchestration services and action servers
     */
    void initialize_mission_orchestration();
    
    /**
     * @brief Setup mission coordination services
     * Services: /mission_coordinator/plan_mission, /mission_coordinator/assign_zones
     */
    void setup_mission_services();
    
    /**
     * @brief Setup mission publishers for global status
     * Topics: /mission_coordinator/mission_status, /mission_coordinator/zone_assignments
     */
    void setup_mission_publishers();
    
    /**
     * @brief Setup action server for mission execution
     * Action: /mission_coordinator/actions/execute_mission
     */
    void setup_mission_action_server();
    
    /**
     * @brief Discover available vehicles using ultra-clean naming
     * Finds nodes like /Droan1, /PX4_Drone2 and queries their capabilities
     */
    void discover_mission_vehicles();

    // === SERVICE CALLBACKS ===
    
    /**
     * @brief Plan multi-vehicle mission based on requirements
     * Service: /mission_coordinator/plan_mission
     */
    bool plan_mission_callback(
        const std::shared_ptr<mission_search_interfaces::srv::PlanMission::Request> request,
        std::shared_ptr<mission_search_interfaces::srv::PlanMission::Response> response);
    
    /**
     * @brief Assign search zones to available vehicles
     * Service: /mission_coordinator/assign_zones
     */
    bool assign_search_zone_callback(
        const std::shared_ptr<mission_search_interfaces::srv::AssignSearchZone::Request> request,
        std::shared_ptr<mission_search_interfaces::srv::AssignSearchZone::Response> response);
    
    /**
     * @brief Get overall mission status from all vehicles
     * Service: /mission_coordinator/get_mission_status
     */
    bool get_mission_status_callback(
        const std::shared_ptr<mission_search_interfaces::srv::GetMissionStatus::Request> request,
        std::shared_ptr<mission_search_interfaces::srv::GetMissionStatus::Response> response);

    // === ACTION SERVER CALLBACKS ===
    
    /**
     * @brief Handle execute mission goal requests
     */
    rclcpp_action::GoalResponse handle_execute_mission_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMissionAction::Goal> goal);
    
    /**
     * @brief Handle mission cancellation requests
     */
    rclcpp_action::CancelResponse handle_execute_mission_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle);
    
    /**
     * @brief Handle accepted mission goals - start orchestration
     */
    void handle_execute_mission_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle);
    
    /**
     * @brief Execute mission orchestration in dedicated thread
     */
    void execute_mission_orchestration(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle);

    // === MISSION PLANNING ALGORITHMS ===
    
    /**
     * @brief Intelligent zone assignment based on vehicle capabilities
     * @param zones Search zones to assign
     * @param available_vehicles Map of vehicle capabilities
     * @return Optimal zone-to-vehicle assignments
     */
    std::unordered_map<std::string, std::string> optimize_zone_assignments(
        const std::vector<mission_search_interfaces::msg::SearchZone>& zones,
        const std::unordered_map<std::string, mission_search_interfaces::msg::VehicleCapabilities>& available_vehicles);
    
    /**
     * @brief Calculate vehicle capability score for zone assignment
     * @param capabilities Vehicle capabilities
     * @param zone Search zone requirements
     * @return Capability score (higher = better match)
     */
    float calculate_capability_score(
        const mission_search_interfaces::msg::VehicleCapabilities& capabilities,
        const mission_search_interfaces::msg::SearchZone& zone);
    
    /**
     * @brief Create search zones from mission requirements
     * @param mission_area Overall mission area
     * @param num_zones Number of zones to create
     * @param pattern_preference Preferred search patterns
     * @return Generated search zones
     */
    std::vector<mission_search_interfaces::msg::SearchZone> create_search_zones(
        const geometry_msgs::msg::Polygon& mission_area,
        int num_zones,
        const std::string& pattern_preference = "optimal");
    
    /**
     * @brief Subdivide polygon into smaller zones
     * @param boundary Original boundary
     * @param num_subdivisions Number of subdivisions per axis
     * @return Vector of sub-zone boundaries
     */
    std::vector<geometry_msgs::msg::Polygon> subdivide_area(
        const geometry_msgs::msg::Polygon& boundary,
        int num_subdivisions);

    // === MISSION MONITORING AND COORDINATION ===
    
    /**
     * @brief Monitor ongoing missions and handle failures
     * Called periodically to check mission progress
     */
    void monitor_active_missions();
    
    /**
     * @brief Handle vehicle failure during mission
     * @param failed_vehicle Vehicle that failed
     * @param mission_id Current mission ID
     */
    void handle_vehicle_failure(const std::string& failed_vehicle, const std::string& mission_id);
    
    /**
     * @brief Reallocate failed vehicle's tasks to other vehicles
     * @param failed_zone Zone that needs new assignment
     * @param available_vehicles Currently available vehicles
     * @return New vehicle assignment or empty if none available
     */
    std::string reallocate_search_zone(
        const mission_search_interfaces::msg::SearchZone& failed_zone,
        const std::unordered_map<std::string, mission_search_interfaces::msg::VehicleCapabilities>& available_vehicles);
    
    /**
     * @brief Publish global mission status update
     * @param status Current mission status
     * @param message Human-readable status message
     */
    void publish_mission_status_update(uint8_t status, const std::string& message);
    
    /**
     * @brief Send search area command to specific vehicle
     * @param vehicle_name Target vehicle following ultra-clean naming
     * @param zone Search zone to assign
     * @return Success status
     */
    bool send_search_command_to_vehicle(
        const std::string& vehicle_name,
        const mission_search_interfaces::msg::SearchZone& zone);

    // === UTILITY METHODS ===
    
    /**
     * @brief Query vehicle capabilities using ultra-clean service names
     * @param vehicle_name Vehicle name (e.g., "Droan1")
     * @return Vehicle capabilities or nullptr if unavailable
     */
    std::shared_ptr<mission_search_interfaces::msg::VehicleCapabilities> 
    query_vehicle_capabilities(const std::string& vehicle_name);
    
    /**
     * @brief Check if vehicle is available for new missions
     * @param vehicle_name Vehicle to check
     * @return True if available, false if busy/offline
     */
    bool is_vehicle_available(const std::string& vehicle_name);
    
    /**
     * @brief Calculate estimated mission duration
     * @param zones All search zones
     * @param assignments Zone-to-vehicle assignments
     * @return Estimated total mission time in seconds
     */
    double estimate_mission_duration(
        const std::vector<mission_search_interfaces::msg::SearchZone>& zones,
        const std::unordered_map<std::string, std::string>& assignments);

    // === MEMBER VARIABLES ===
    
    // Mission action server
    std::shared_ptr<ExecuteMissionActionServer> execute_mission_action_server_;
    
    // Mission services
    rclcpp::Service<mission_search_interfaces::srv::PlanMission>::SharedPtr plan_mission_service_;
    rclcpp::Service<mission_search_interfaces::srv::AssignSearchZone>::SharedPtr assign_zone_service_;
    rclcpp::Service<mission_search_interfaces::srv::GetMissionStatus>::SharedPtr get_status_service_;
    
    // Mission publishers
    rclcpp::Publisher<mission_search_interfaces::msg::MissionStatus>::SharedPtr global_mission_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_assignments_pub_;
    
    // Mission state tracking
    std::atomic<bool> mission_active_{false};
    std::string current_mission_id_;
    std::unordered_map<std::string, std::string> current_zone_assignments_;  // zone_id -> vehicle_name
    std::unordered_map<std::string, mission_search_interfaces::msg::VehicleCapabilities> discovered_vehicles_;
    
    // Mission monitoring
    rclcpp::TimerBase::SharedPtr mission_monitor_timer_;
    std::chrono::steady_clock::time_point mission_start_time_;
    
    // Thread management
    std::thread mission_execution_thread_;
    std::mutex mission_state_mutex_;
    
    // Callback group for mission operations
    rclcpp::CallbackGroup::SharedPtr mission_callback_group_;
    
    // Mission statistics
    uint32_t total_missions_completed_ = 0;
    uint32_t total_zones_assigned_ = 0;
    uint32_t total_vehicles_coordinated_ = 0;
};