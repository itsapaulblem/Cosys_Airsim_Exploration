#include "mission_coordination_node.hpp"
#include <cmath>
#include <algorithm>
#include <random>

using namespace std::placeholders;
using namespace std::chrono_literals;

MissionCoordinationNode::MissionCoordinationNode()
    : CoordinationNode()  // Initialize base coordination functionality
{
    RCLCPP_INFO(this->get_logger(), 
                "Initializing Mission Coordination Node");
    
    // Initialize mission orchestration after base class setup
    initialize_mission_orchestration();
    
    RCLCPP_INFO(this->get_logger(), 
                "Mission orchestration ready - Coordinating vehicles");
}

void MissionCoordinationNode::initialize_mission_orchestration()
{
    // Create separate callback group for mission operations
    mission_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Setup mission components
    setup_mission_services();
    setup_mission_publishers();
    setup_mission_action_server();
    
    // Start vehicle discovery
    discover_mission_vehicles();
    
    // Start mission monitoring timer (every 2 seconds)
    mission_monitor_timer_ = this->create_wall_timer(
        2s,
        std::bind(&MissionCoordinationNode::monitor_active_missions, this));
}

void MissionCoordinationNode::setup_mission_services()
{
    // naming: Global coordination services
    plan_mission_service_ = this->create_service<mission_search_interfaces::srv::PlanMission>(
        "/mission_coordinator/plan_mission",
        std::bind(&MissionCoordinationNode::plan_mission_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        mission_callback_group_);
    
    assign_zone_service_ = this->create_service<mission_search_interfaces::srv::AssignSearchZone>(
        "/mission_coordinator/assign_zones",
        std::bind(&MissionCoordinationNode::assign_search_zone_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        mission_callback_group_);
    
    get_status_service_ = this->create_service<mission_search_interfaces::srv::GetMissionStatus>(
        "/mission_coordinator/get_mission_status",
        std::bind(&MissionCoordinationNode::get_mission_status_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        mission_callback_group_);
    
    RCLCPP_INFO(this->get_logger(), "Mission coordination services created");
}

void MissionCoordinationNode::setup_mission_publishers()
{
    // Global mission status publishing
    global_mission_status_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionStatus>(
        "/mission_coordinator/mission_status", 10);
    
    zone_assignments_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/mission_coordinator/zone_assignments", 10);
    
    RCLCPP_INFO(this->get_logger(), "Mission coordination publishers created");
}

void MissionCoordinationNode::setup_mission_action_server()
{
    // Global mission execution action server
    execute_mission_action_server_ = rclcpp_action::create_server<ExecuteMissionAction>(
        this,
        "/mission_coordinator/actions/execute_mission",
        std::bind(&MissionCoordinationNode::handle_execute_mission_goal, this, _1, _2),
        std::bind(&MissionCoordinationNode::handle_execute_mission_cancel, this, _1),
        std::bind(&MissionCoordinationNode::handle_execute_mission_accepted, this, _1),
        rcl_action_server_get_default_options(),
        mission_callback_group_);
    
    RCLCPP_INFO(this->get_logger(), "Mission execution action server created: /mission_coordinator/actions/execute_mission");
}

void MissionCoordinationNode::discover_mission_vehicles()
{
    // In a real implementation, this would use ROS2 node discovery
    // For now, we'll use the vehicle_names_ from the base coordination node
    // Plus common vehicle naming patterns
    
    std::vector<std::string> potential_vehicles = {"Droan1", "PX4_Drone2", "SimpleFlight3"};
    
    // Add any vehicles from base coordination node configuration
    for (const auto& vehicle : vehicle_names_) {
        potential_vehicles.push_back(vehicle);
    }
    
    discovered_vehicles_.clear();
    
    for (const auto& vehicle_name : potential_vehicles) {
        auto capabilities = query_vehicle_capabilities(vehicle_name);
        if (capabilities) {
            discovered_vehicles_[vehicle_name] = *capabilities;
            RCLCPP_INFO(this->get_logger(), "Discovered mission-capable vehicle: %s", vehicle_name.c_str());
        }
    }
    
    total_vehicles_coordinated_ = discovered_vehicles_.size();
    RCLCPP_INFO(this->get_logger(), "Vehicle discovery complete: %zu mission-capable vehicles found", 
                discovered_vehicles_.size());
}

// === SERVICE CALLBACK IMPLEMENTATIONS ===

bool MissionCoordinationNode::plan_mission_callback(
    const std::shared_ptr<mission_search_interfaces::srv::PlanMission::Request> request,
    std::shared_ptr<mission_search_interfaces::srv::PlanMission::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received mission planning request: %s", 
                request->mission_name.c_str());
    
    // Create mission plan
    auto& plan = response->mission_plan;
    plan.mission_id = request->mission_name + "_" + std::to_string(std::time(nullptr));
    plan.mission_name = request->mission_name;
    plan.mission_type = request->mission_type.empty() ? "search_and_rescue" : request->mission_type;
    plan.priority_level = 1;  // High priority by default
    
    // Generate search zones from mission area
    auto zones = create_search_zones(
        request->mission_area, 
        std::min(static_cast<int>(discovered_vehicles_.size()), static_cast<int>(request->max_vehicles)),
        request->preferred_search_pattern);
    
    plan.search_zones = zones;
    
    // Assign zones to available vehicles
    auto assignments = optimize_zone_assignments(zones, discovered_vehicles_);
    
    // Add participating vehicles to plan
    for (const auto& assignment : assignments) {
        if (std::find(plan.assigned_vehicles.begin(), plan.assigned_vehicles.end(), 
                     assignment.second) == plan.assigned_vehicles.end()) {
            plan.assigned_vehicles.push_back(assignment.second);
        }
    }
    
    // Calculate estimated duration
    double estimated_duration = estimate_mission_duration(zones, assignments);
    plan.max_mission_time.sec = static_cast<int32_t>(estimated_duration);
    plan.max_mission_time.nanosec = 0;
    
    response->success = true;
    response->message = "Mission plan created successfully with " + 
                       std::to_string(zones.size()) + " zones and " + 
                       std::to_string(plan.assigned_vehicles.size()) + " vehicles";
    
    RCLCPP_INFO(this->get_logger(), "Mission plan created: %s", response->message.c_str());
    return true;
}

bool MissionCoordinationNode::assign_search_zone_callback(
    const std::shared_ptr<mission_search_interfaces::srv::AssignSearchZone::Request> request,
    std::shared_ptr<mission_search_interfaces::srv::AssignSearchZone::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Assigning search zones for strategy: %s", 
                request->allocation_strategy.c_str());
    
    // Create zones from mission area
    auto zones = create_search_zones(
        request->mission_area, 
        request->num_zones == 0 ? discovered_vehicles_.size() : request->num_zones,
        request->default_search_pattern);
    
    // Optimize zone assignments
    auto assignments = optimize_zone_assignments(zones, discovered_vehicles_);
    
    response->assigned_zones.clear();
    response->unassigned_vehicles.clear();
    
    // Build assigned zones with vehicle assignments
    for (const auto& assignment : assignments) {
        // Find the zone
        auto zone_it = std::find_if(zones.begin(), zones.end(),
            [&assignment](const auto& zone) { return zone.zone_id == assignment.first; });
        
        if (zone_it != zones.end()) {
            auto assigned_zone = *zone_it;
            // Add vehicle assignment info (could be stored in zone description)
            response->assigned_zones.push_back(assigned_zone);
        }
    }
    
    // Find unassigned vehicles
    for (const auto& vehicle : request->available_vehicles) {
        bool assigned = false;
        for (const auto& assignment : assignments) {
            if (assignment.second == vehicle) {
                assigned = true;
                break;
            }
        }
        if (!assigned) {
            response->unassigned_vehicles.push_back(vehicle);
        }
    }
    
    response->total_zones_created = zones.size();
    response->total_mission_area_sq_m = 10000.0f;  // Calculate from mission_area polygon
    
    // Store assignments for monitoring
    {
        std::lock_guard<std::mutex> lock(mission_state_mutex_);
        current_zone_assignments_ = assignments;
        total_zones_assigned_ += assignments.size();
    }
    
    response->success = true;
    response->message = "Created " + std::to_string(zones.size()) + " zones, assigned " + 
                       std::to_string(assignments.size()) + " to vehicles";
    
    RCLCPP_INFO(this->get_logger(), "Zone assignment complete: %s", response->message.c_str());
    return true;
}

bool MissionCoordinationNode::get_mission_status_callback(
    const std::shared_ptr<mission_search_interfaces::srv::GetMissionStatus::Request> request,
    std::shared_ptr<mission_search_interfaces::srv::GetMissionStatus::Response> response)
{
    std::lock_guard<std::mutex> lock(mission_state_mutex_);
    
    if (request->mission_id.empty() || request->mission_id == current_mission_id_) {
        // Return current mission status
        response->mission_status.mission_id = current_mission_id_;
        response->mission_status.vehicle_name = "mission_coordinator";
        response->mission_status.status = mission_active_.load() ? 2 : 0;  // ACTIVE or IDLE
        response->mission_status.progress_percentage = mission_active_.load() ? 50.0f : 0.0f;  // Simplified
        response->mission_status.targets_detected = 0;  // Would aggregate from vehicles
        response->mission_status.current_activity = mission_active_.load() ? 
            "Coordinating multi-vehicle search" : "Awaiting mission assignment";
        
        response->success = true;
        response->message = "Current mission status retrieved";
    } else {
        response->success = false;
        response->message = "Mission ID not found: " + request->mission_id;
    }
    
    return true;
}

// === ACTION SERVER CALLBACK IMPLEMENTATIONS ===

rclcpp_action::GoalResponse MissionCoordinationNode::handle_execute_mission_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMissionAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received mission execution request: %s", 
                goal->mission_plan.mission_name.c_str());
    
    // Validate mission plan
    if (goal->mission_plan.search_zones.empty()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting mission - no search zones defined");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->mission_plan.assigned_vehicles.empty()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting mission - no vehicles assigned");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if mission already active
    if (mission_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting mission - another mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    (void)uuid;  // Suppress unused parameter warning
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionCoordinationNode::handle_execute_mission_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received mission cancellation request");
    
    mission_active_.store(false);
    
    (void)goal_handle;  // Suppress unused parameter warning
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionCoordinationNode::handle_execute_mission_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Mission execution goal accepted - starting orchestration");
    
    // Execute in separate thread to not block action server
    if (mission_execution_thread_.joinable()) {
        mission_execution_thread_.join();
    }
    
    mission_execution_thread_ = std::thread(
        &MissionCoordinationNode::execute_mission_orchestration, this, goal_handle);
}

void MissionCoordinationNode::execute_mission_orchestration(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMissionAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting mission orchestration thread");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteMissionAction::Feedback>();
    auto result = std::make_shared<ExecuteMissionAction::Result>();
    
    // Set mission as active
    {
        std::lock_guard<std::mutex> lock(mission_state_mutex_);
        mission_active_.store(true);
        current_mission_id_ = goal->mission_plan.mission_id;
        mission_start_time_ = std::chrono::steady_clock::now();
    }
    
    // Phase 1: Assign zones to vehicles
    RCLCPP_INFO(this->get_logger(), "Phase 1: Assigning search zones to vehicles");
    
    auto assignments = optimize_zone_assignments(goal->mission_plan.search_zones, discovered_vehicles_);
    
    if (assignments.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to assign any zones to vehicles");
        result->success = false;
        result->completion_reason = "no_vehicle_assignments";
        goal_handle->abort(result);
        mission_active_.store(false);
        return;
    }
    
    // Phase 2: Send search commands to vehicles
    RCLCPP_INFO(this->get_logger(), "Phase 2: Sending search commands to vehicles");
    
    uint32_t successful_assignments = 0;
    for (const auto& assignment : assignments) {
        // Find the zone
        auto zone_it = std::find_if(goal->mission_plan.search_zones.begin(), 
                                   goal->mission_plan.search_zones.end(),
            [&assignment](const auto& zone) { return zone.zone_id == assignment.first; });
        
        if (zone_it != goal->mission_plan.search_zones.end()) {
            bool success = send_search_command_to_vehicle(assignment.second, *zone_it);
            if (success) {
                successful_assignments++;
                RCLCPP_INFO(this->get_logger(), "Assigned zone %s to vehicle %s", 
                           assignment.first.c_str(), assignment.second.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to assign zone %s to vehicle %s", 
                           assignment.first.c_str(), assignment.second.c_str());
            }
        }
    }
    
    // Phase 3: Monitor mission progress
    RCLCPP_INFO(this->get_logger(), "Phase 3: Monitoring mission progress");
    
    const auto mission_timeout = std::chrono::seconds(goal->mission_plan.max_mission_time.sec + 600);  // Add 10 minutes buffer
    auto mission_deadline = std::chrono::steady_clock::now() + mission_timeout;
    
    while (mission_active_.load() && rclcpp::ok()) {
        // Check for cancellation
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Mission cancelled by user");
            result->success = false;
            result->completion_reason = "cancelled";
            goal_handle->canceled(result);
            mission_active_.store(false);
            return;
        }
        
        // Check timeout
        if (std::chrono::steady_clock::now() > mission_deadline) {
            RCLCPP_WARN(this->get_logger(), "Mission timeout exceeded");
            result->success = false;
            result->completion_reason = "timeout";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        // Update progress feedback
        auto elapsed = std::chrono::steady_clock::now() - mission_start_time_;
        auto elapsed_minutes = std::chrono::duration_cast<std::chrono::minutes>(elapsed).count();
        
        feedback->progress_percentage = std::min(90.0f, 
            (static_cast<float>(elapsed_minutes) / (goal->mission_plan.max_mission_time.sec / 60.0f)) * 100.0f);
        feedback->vehicles_completed = 0;  // Would query individual vehicles
        feedback->total_targets_detected_so_far = 0;  // Would aggregate from vehicles
        feedback->vehicles_active = successful_assignments;
        feedback->current_mission_phase = "coordinating_search";
        feedback->estimated_remaining_time.sec = std::max(0, 
            static_cast<int>(goal->mission_plan.max_mission_time.sec / 60) - static_cast<int>(elapsed_minutes)) * 60;
        feedback->estimated_remaining_time.nanosec = 0;
        
        // Set elapsed time
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
        feedback->elapsed_time.sec = static_cast<int32_t>(elapsed_seconds);
        feedback->elapsed_time.nanosec = 0;
        
        goal_handle->publish_feedback(feedback);
        
        // Simplified completion check (in real implementation, would check individual vehicle progress)
        if (elapsed_minutes >= (goal->mission_plan.max_mission_time.sec / 60)) {
            break;
        }
        
        std::this_thread::sleep_for(5s);  // Update every 5 seconds
    }
    
    // Mission completed successfully
    RCLCPP_INFO(this->get_logger(), "Mission orchestration completed");
    
    result->success = true;
    result->completion_reason = "completed";
    result->mission_area_covered_sq_m = 10000.0f;  // Placeholder
    result->total_targets_detected = 0;  // Would aggregate from vehicles
    result->participating_vehicles_count = successful_assignments;
    // zones_completed field not in interface - would track differently
    
    auto final_elapsed = std::chrono::steady_clock::now() - mission_start_time_;
    result->total_mission_time.sec = std::chrono::duration_cast<std::chrono::seconds>(final_elapsed).count();
    result->total_mission_time.nanosec = 0;
    
    goal_handle->succeed(result);
    mission_active_.store(false);
    total_missions_completed_++;
    
    RCLCPP_INFO(this->get_logger(), "Mission completed successfully in %d seconds", 
                result->total_mission_time.sec);
}

// === MISSION PLANNING ALGORITHM IMPLEMENTATIONS ===

std::unordered_map<std::string, std::string> MissionCoordinationNode::optimize_zone_assignments(
    const std::vector<mission_search_interfaces::msg::SearchZone>& zones,
    const std::unordered_map<std::string, mission_search_interfaces::msg::VehicleCapabilities>& available_vehicles)
{
    std::unordered_map<std::string, std::string> assignments;
    
    if (zones.empty() || available_vehicles.empty()) {
        return assignments;
    }
    
    // Create sorted lists for optimal matching
    std::vector<mission_search_interfaces::msg::SearchZone> sorted_zones = zones;
    std::sort(sorted_zones.begin(), sorted_zones.end(), 
        [](const auto& a, const auto& b) { return a.priority < b.priority; });  // 1 = highest priority
    
    std::vector<std::string> available_vehicle_names;
    for (const auto& vehicle : available_vehicles) {
        if (is_vehicle_available(vehicle.first)) {
            available_vehicle_names.push_back(vehicle.first);
        }
    }
    
    // Assign zones to vehicles using greedy algorithm
    for (const auto& zone : sorted_zones) {
        if (available_vehicle_names.empty()) {
            break;  // No more vehicles available
        }
        
        // Find best vehicle for this zone
        std::string best_vehicle;
        float best_score = -1.0f;
        
        for (const auto& vehicle_name : available_vehicle_names) {
            auto vehicle_it = available_vehicles.find(vehicle_name);
            if (vehicle_it != available_vehicles.end()) {
                float score = calculate_capability_score(vehicle_it->second, zone);
                if (score > best_score) {
                    best_score = score;
                    best_vehicle = vehicle_name;
                }
            }
        }
        
        if (!best_vehicle.empty() && best_score > 0.1f) {  // Minimum capability threshold
            assignments[zone.zone_id] = best_vehicle;
            
            // Remove assigned vehicle from available list (one zone per vehicle for now)
            available_vehicle_names.erase(
                std::remove(available_vehicle_names.begin(), available_vehicle_names.end(), best_vehicle),
                available_vehicle_names.end());
        }
    }
    
    return assignments;
}

float MissionCoordinationNode::calculate_capability_score(
    const mission_search_interfaces::msg::VehicleCapabilities& capabilities,
    const mission_search_interfaces::msg::SearchZone& zone)
{
    float score = 0.0f;
    
    // Base score from battery level
    score += capabilities.battery_percentage * 0.01f;  // 0.0 to 1.0
    
    // Speed capability (prefer faster vehicles for larger zones)
    float area_factor = zone.area_sq_m / 10000.0f;  // Normalize to 1.0 for 10k sq meters
    score += (capabilities.max_speed_ms / 20.0f) * std::min(area_factor, 1.0f);
    
    // Altitude capability match
    float altitude_match = 1.0f;
    if (capabilities.max_altitude_m < zone.max_altitude || 
        capabilities.min_altitude_m > zone.min_altitude) {
        altitude_match = 0.5f;  // Penalize altitude mismatch
    }
    score *= altitude_match;
    
    // Special sensor requirements
    if (zone.requires_special_sensors) {
        if (capabilities.has_thermal_camera || capabilities.has_night_vision) {
            score += 0.5f;  // Bonus for special sensors
        } else {
            score *= 0.5f;  // Penalty for lacking required sensors
        }
    }
    
    // Terrain difficulty penalty
    float terrain_penalty = (zone.terrain_difficulty - 1) * 0.1f;  // 0.0 to 0.4 penalty
    score *= (1.0f - terrain_penalty);
    
    return std::max(0.0f, score);
}

// === UTILITY METHOD IMPLEMENTATIONS ===

std::shared_ptr<mission_search_interfaces::msg::VehicleCapabilities> 
MissionCoordinationNode::query_vehicle_capabilities(const std::string& vehicle_name)
{
    // In real implementation, this would call the service:
    // /VehicleName/services/get_capabilities
    // For now, return simulated capabilities
    
    auto capabilities = std::make_shared<mission_search_interfaces::msg::VehicleCapabilities>();
    capabilities->vehicle_name = vehicle_name;
    capabilities->vehicle_type = "multirotor";
    capabilities->max_speed_ms = 15.0f;
    capabilities->max_altitude_m = 120.0f;
    capabilities->min_altitude_m = 5.0f;
    capabilities->battery_percentage = 90.0f;
    capabilities->available_cameras = {"rgb", "depth"};
    capabilities->has_thermal_camera = (vehicle_name == "PX4_Drone2");
    capabilities->detection_range_m = 100.0f;
    capabilities->available_for_mission = true;
    
    return capabilities;
}

bool MissionCoordinationNode::is_vehicle_available(const std::string& vehicle_name)
{
    // In real implementation, this would check vehicle status
    // For now, assume discovered vehicles are available
    return discovered_vehicles_.find(vehicle_name) != discovered_vehicles_.end();
}

bool MissionCoordinationNode::send_search_command_to_vehicle(
    const std::string& vehicle_name,
    const mission_search_interfaces::msg::SearchZone& zone)
{
    // In real implementation, this would call the action:
    // /VehicleName/actions/search_area
    RCLCPP_INFO(this->get_logger(), 
                "Would send search command to %s for zone %s (pattern: %s)", 
                vehicle_name.c_str(), zone.zone_id.c_str(), zone.search_pattern.c_str());
    
    // Simulate successful assignment
    return true;
}

void MissionCoordinationNode::monitor_active_missions()
{
    if (mission_active_.load()) {
        // In real implementation, this would monitor vehicle progress
        // and handle failures/reallocations
        publish_mission_status_update(2, "Coordinating active search mission");
    }
}

std::vector<mission_search_interfaces::msg::SearchZone> MissionCoordinationNode::create_search_zones(
    const geometry_msgs::msg::Polygon& mission_area,
    int num_zones,
    const std::string& pattern_preference)
{
    std::vector<mission_search_interfaces::msg::SearchZone> zones;
    
    // Simple zone generation - subdivide area into rectangular zones
    auto sub_areas = subdivide_area(mission_area, std::max(1, static_cast<int>(std::sqrt(num_zones))));
    
    for (size_t i = 0; i < sub_areas.size() && i < static_cast<size_t>(num_zones); ++i) {
        mission_search_interfaces::msg::SearchZone zone;
        zone.zone_id = "zone_" + std::to_string(i + 1);
        zone.boundary_polygon = sub_areas[i];
        zone.search_pattern = (pattern_preference == "optimal") ? "spiral" : pattern_preference;
        zone.min_altitude = 20.0f;
        zone.max_altitude = 50.0f;
        zone.priority = static_cast<uint8_t>(1 + (i % 3));  // Rotate priorities 1-3
        zone.area_sq_m = 2500.0f;  // Simplified calculation
        zone.terrain_difficulty = 2;
        zone.requires_special_sensors = false;
        
        zones.push_back(zone);
    }
    
    return zones;
}

std::vector<geometry_msgs::msg::Polygon> MissionCoordinationNode::subdivide_area(
    const geometry_msgs::msg::Polygon& boundary,
    int num_subdivisions)
{
    std::vector<geometry_msgs::msg::Polygon> sub_areas;
    
    if (boundary.points.empty() || num_subdivisions <= 0) {
        return sub_areas;
    }
    
    // Simple rectangular subdivision for demonstration
    // In real implementation, would use sophisticated geometric algorithms
    
    // Find bounding box
    float min_x = boundary.points[0].x, max_x = boundary.points[0].x;
    float min_y = boundary.points[0].y, max_y = boundary.points[0].y;
    
    for (const auto& point : boundary.points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Create grid subdivisions
    float width = (max_x - min_x) / num_subdivisions;
    float height = (max_y - min_y) / num_subdivisions;
    
    for (int i = 0; i < num_subdivisions; ++i) {
        for (int j = 0; j < num_subdivisions; ++j) {
            geometry_msgs::msg::Polygon sub_area;
            
            // Create rectangular sub-area
            geometry_msgs::msg::Point32 p1, p2, p3, p4;
            p1.x = min_x + i * width;
            p1.y = min_y + j * height;
            p2.x = min_x + (i + 1) * width;
            p2.y = min_y + j * height;
            p3.x = min_x + (i + 1) * width;
            p3.y = min_y + (j + 1) * height;
            p4.x = min_x + i * width;
            p4.y = min_y + (j + 1) * height;
            
            sub_area.points = {p1, p2, p3, p4};
            sub_areas.push_back(sub_area);
        }
    }
    
    return sub_areas;
}

double MissionCoordinationNode::estimate_mission_duration(
    const std::vector<mission_search_interfaces::msg::SearchZone>& zones,
    const std::unordered_map<std::string, std::string>& assignments)
{
    double max_duration = 0.0;
    
    // Calculate maximum time across all assignments (parallel execution)
    for (const auto& assignment : assignments) {
        auto zone_it = std::find_if(zones.begin(), zones.end(),
            [&assignment](const auto& zone) { return zone.zone_id == assignment.first; });
        
        if (zone_it != zones.end()) {
            // Estimate based on area and search pattern
            double estimated_time = zone_it->area_sq_m / 300.0;  // ~300 sq_m/second coverage
            max_duration = std::max(max_duration, estimated_time);
        }
    }
    
    return max_duration;
}

void MissionCoordinationNode::publish_mission_status_update(uint8_t status, const std::string& message)
{
    mission_search_interfaces::msg::MissionStatus status_msg;
    status_msg.mission_id = current_mission_id_;
    status_msg.vehicle_name = "mission_coordinator";
    status_msg.status = status;
    status_msg.progress_percentage = 0.0f;  // Would calculate based on vehicle feedback
    status_msg.current_activity = message;
    
    global_mission_status_pub_->publish(status_msg);
}