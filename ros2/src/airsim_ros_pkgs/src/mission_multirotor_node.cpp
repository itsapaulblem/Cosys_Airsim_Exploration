#include "mission_multirotor_node.hpp"
#include <cmath>
#include <algorithm>
#include <random>

using namespace std::placeholders;

MissionMultirotorNode::MissionMultirotorNode(const std::string& vehicle_name, 
                                           const std::string& host_ip, 
                                           uint16_t host_port)
    : MultirotorNode(vehicle_name, host_ip, host_port)
{
    RCLCPP_INFO(this->get_logger(), 
                "Initializing Mission Multirotor Node for vehicle: %s", vehicle_name.c_str());
    
    // Initialize mission capabilities after base class setup
    initialize_mission_capabilities();
    
    RCLCPP_INFO(this->get_logger(), 
                "Mission capabilities initialized for %s - Ready for mission operations", 
                vehicle_name.c_str());
}

void MissionMultirotorNode::initialize_mission_capabilities()
{
    // Create callback group for mission operations (separate from sensor processing)
    mission_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Setup mission components
    setup_mission_publishers();
    setup_mission_services();
    setup_mission_action_servers();

    // Setup enhanced flight services with event generation
    setup_mission_flight_services();

    // Initialize mission event detection
    initialize_event_detection();
}

void MissionMultirotorNode::setup_mission_publishers()
{
    // /VehicleName/mission/status
    std::string mission_status_topic = vehicle_name_ + "/mission/status";
    mission_status_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionStatus>(
        mission_status_topic, 10);
    
    // /VehicleName/detections/target  
    std::string target_detection_topic = vehicle_name_ + "/detections/target";
    target_detection_pub_ = this->create_publisher<mission_search_interfaces::msg::TargetDetection>(
        target_detection_topic, 10);
    
    // /VehicleName/mission/events
    std::string mission_event_topic = vehicle_name_ + "/mission/events";
    mission_event_pub_ = this->create_publisher<mission_search_interfaces::msg::MissionEvent>(
        mission_event_topic, 50);  // Higher queue for event logging
    
    RCLCPP_INFO(this->get_logger(), "Mission publishers created: %s, %s, %s", 
                mission_status_topic.c_str(), target_detection_topic.c_str(), mission_event_topic.c_str());
}

void MissionMultirotorNode::setup_mission_services()
{
    // /VehicleName/services/set_search_pattern
    std::string set_pattern_service = vehicle_name_ + "/services/set_search_pattern";
    set_search_pattern_service_ = this->create_service<mission_search_interfaces::srv::SetSearchPattern>(
        set_pattern_service,
        std::bind(&MissionMultirotorNode::set_search_pattern_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        mission_callback_group_);
    
    // /VehicleName/services/get_capabilities  
    std::string get_capabilities_service = vehicle_name_ + "/services/get_capabilities";
    get_capabilities_service_ = this->create_service<mission_search_interfaces::srv::GetVehicleCapabilities>(
        get_capabilities_service,
        std::bind(&MissionMultirotorNode::get_vehicle_capabilities_callback, this, _1, _2),
        rmw_qos_profile_services_default,
        mission_callback_group_);
    
    RCLCPP_INFO(this->get_logger(), "Mission services created: %s, %s", 
                set_pattern_service.c_str(), get_capabilities_service.c_str());
}

void MissionMultirotorNode::setup_mission_action_servers()
{
    // /VehicleName/actions/search_area
    std::string search_area_action = vehicle_name_ + "/actions/search_area";
    search_area_action_server_ = rclcpp_action::create_server<SearchAreaAction>(
        this,
        search_area_action,
        std::bind(&MissionMultirotorNode::handle_search_area_goal, this, _1, _2),
        std::bind(&MissionMultirotorNode::handle_search_area_cancel, this, _1),
        std::bind(&MissionMultirotorNode::handle_search_area_accepted, this, _1),
        rcl_action_server_get_default_options(),
        mission_callback_group_);
    
    // /VehicleName/actions/navigate_to_target
    std::string navigate_action = vehicle_name_ + "/actions/navigate_to_target";
    navigate_to_target_action_server_ = rclcpp_action::create_server<NavigateToTargetAction>(
        this,
        navigate_action,
        std::bind(&MissionMultirotorNode::handle_navigate_to_target_goal, this, _1, _2),
        std::bind(&MissionMultirotorNode::handle_navigate_to_target_cancel, this, _1),
        std::bind(&MissionMultirotorNode::handle_navigate_to_target_accepted, this, _1),
        rcl_action_server_get_default_options(),
        mission_callback_group_);
    
    // /VehicleName/actions/track_target
    std::string track_action = vehicle_name_ + "/actions/track_target";
    track_target_action_server_ = rclcpp_action::create_server<TrackTargetAction>(
        this,
        track_action,
        std::bind(&MissionMultirotorNode::handle_track_target_goal, this, _1, _2),
        std::bind(&MissionMultirotorNode::handle_track_target_cancel, this, _1),
        std::bind(&MissionMultirotorNode::handle_track_target_accepted, this, _1),
        rcl_action_server_get_default_options(),
        mission_callback_group_);
    
    RCLCPP_INFO(this->get_logger(), "Mission action servers created: %s, %s, %s", 
                search_area_action.c_str(), navigate_action.c_str(), track_action.c_str());
}

// ============================================================================
// SEARCH AREA ACTION IMPLEMENTATION  
// ============================================================================

rclcpp_action::GoalResponse MissionMultirotorNode::handle_search_area_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SearchAreaAction::Goal> goal)
{
    (void)uuid;  // Suppress unused parameter warning
    
    RCLCPP_INFO(this->get_logger(), "Received search area goal for pattern: %s", 
                goal->search_pattern.c_str());
    
    // Check if already running a mission
    if (mission_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal - mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Basic validation with default value support
    if (goal->search_boundary.points.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid search boundary - need at least 3 points");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Allow 0.0 values as "use defaults" - only reject negative or excessive values
    if (goal->search_altitude != 0.0f && (goal->search_altitude < 5.0f || goal->search_altitude > 150.0f)) {
        RCLCPP_WARN(this->get_logger(), "Search altitude %f outside safe range (5-150m). Use 0.0 for default.", 
                    goal->search_altitude);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Validate speed and spacing if provided (accept 0.0 as default)
    if (goal->search_speed < 0.0f || goal->search_speed > 15.0f) {
        RCLCPP_WARN(this->get_logger(), "Search speed %f outside safe range (0-15m/s). Use 0.0 for default.", 
                    goal->search_speed);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->pattern_spacing < 0.0f || goal->pattern_spacing > 100.0f) {
        RCLCPP_WARN(this->get_logger(), "Pattern spacing %f outside safe range (0-100m). Use 0.0 for default.", 
                    goal->pattern_spacing);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Log effective values (showing defaults when 0.0)
    float effective_altitude = goal->search_altitude > 0.0f ? goal->search_altitude : 25.0f;
    float effective_speed = goal->search_speed > 0.0f ? goal->search_speed : 5.0f;
    float effective_spacing = goal->pattern_spacing > 0.0f ? goal->pattern_spacing : 15.0f;
    
    RCLCPP_INFO(this->get_logger(), "Accepting search area goal - altitude: %.1fm%s, speed: %.1fm/s%s, spacing: %.1fm%s", 
                effective_altitude, (goal->search_altitude == 0.0f ? " (default)" : ""),
                effective_speed, (goal->search_speed == 0.0f ? " (default)" : ""),
                effective_spacing, (goal->pattern_spacing == 0.0f ? " (default)" : ""));
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionMultirotorNode::handle_search_area_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for search area goal");
    
    // Set cancellation flag for mission execution thread
    mission_cancellation_requested_.store(true);
    
    (void)goal_handle;  // Suppress unused parameter warning
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionMultirotorNode::handle_search_area_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Search area goal accepted - starting execution");
    
    // Execute in separate thread to not block action server
    if (mission_execution_thread_.joinable()) {
        mission_execution_thread_.join();  // Wait for any previous mission to complete
    }
    
    mission_execution_thread_ = std::thread(&MissionMultirotorNode::execute_search_area, this, goal_handle);
}

void MissionMultirotorNode::execute_search_area(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting search area execution thread");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SearchAreaAction::Feedback>();
    auto result = std::make_shared<SearchAreaAction::Result>();
    
    // Initialize mission state
    mission_active_.store(true);
    mission_cancellation_requested_.store(false);
    mission_start_time_ = std::chrono::steady_clock::now();
    current_mission_id_ = "search_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    // Publish initial mission status
    publish_mission_status(mission_search_interfaces::msg::MissionStatus::ACTIVE, 0.0f, "Starting search");
    
    try {
        // Apply defaults for zero values
        float effective_altitude = goal->search_altitude > 0.0f ? goal->search_altitude : 25.0f;
        float effective_speed = goal->search_speed > 0.0f ? goal->search_speed : 5.0f;
        float effective_spacing = goal->pattern_spacing > 0.0f ? goal->pattern_spacing : 15.0f;
        
        RCLCPP_INFO(this->get_logger(), "Using effective values - altitude: %.1fm, speed: %.1fm/s, spacing: %.1fm", 
                    effective_altitude, effective_speed, effective_spacing);
        
        // Ensure vehicle is ready for mission (includes takeoff if needed)
        current_activity_ = "Preparing vehicle for mission";
        publish_mission_status(mission_search_interfaces::msg::MissionStatus::ACTIVE, 2.0f, current_activity_);
        
        if (!ensure_mission_ready(effective_altitude)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare vehicle for mission");
            result->success = false;
            result->completion_reason = "vehicle_preparation_failed";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        // Generate waypoints for search pattern
        current_activity_ = "Generating search pattern";
        publish_mission_status(mission_search_interfaces::msg::MissionStatus::ACTIVE, 5.0f, current_activity_);
        
        auto waypoints = generate_search_waypoints(
            goal->search_boundary, 
            goal->search_pattern,
            effective_spacing,
            effective_altitude);
        
        if (waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate search waypoints");
            result->success = false;
            result->completion_reason = "waypoint_generation_failed";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for search pattern", waypoints.size());
        
        // Area tracking is handled by execute_waypoint_navigation function
        
        feedback->waypoints_remaining = waypoints.size();
        feedback->current_area_covered_sq_m = 0.0;
        goal_handle->publish_feedback(feedback);
        
        // Execute waypoint navigation with progress feedback
        current_activity_ = "Executing search pattern";
        publish_mission_status(mission_search_interfaces::msg::MissionStatus::ACTIVE, 10.0f, current_activity_);
        
        bool navigation_successful = execute_waypoint_navigation(waypoints, effective_speed, goal_handle);
        
        // Mission completion
        auto mission_end_time = std::chrono::steady_clock::now();
        auto mission_duration = std::chrono::duration_cast<std::chrono::seconds>(mission_end_time - mission_start_time_);
        
        if (mission_cancellation_requested_.load()) {
            RCLCPP_INFO(this->get_logger(), "Search mission cancelled by user");
            result->success = false;
            result->completion_reason = "cancelled";
            goal_handle->canceled(result);
        } else if (navigation_successful) {
            RCLCPP_INFO(this->get_logger(), "Search area mission completed successfully");
            result->success = true;
            result->completion_reason = "completed";
            result->total_search_time = rclcpp::Duration(mission_duration);
            result->area_covered_sq_m = total_area_covered_;
            result->targets_detected = total_targets_detected_;
            result->waypoints_completed = waypoints_completed_;
            
            goal_handle->succeed(result);
            publish_mission_status(mission_search_interfaces::msg::MissionStatus::COMPLETED, 100.0f, "Mission completed");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Search mission failed during navigation");
            result->success = false;
            result->completion_reason = "navigation_failed";
            goal_handle->abort(result);
            publish_mission_status(mission_search_interfaces::msg::MissionStatus::FAILED, 
                                 current_mission_progress_, "Mission failed");
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during search execution: %s", e.what());
        result->success = false;
        result->completion_reason = "exception";
        goal_handle->abort(result);
        publish_mission_status(mission_search_interfaces::msg::MissionStatus::FAILED, 
                             current_mission_progress_, "Mission failed with exception");
    }
    
    // Clean up mission state
    mission_active_.store(false);
    current_activity_ = "Idle";
    RCLCPP_INFO(this->get_logger(), "Search area execution thread completed");
}

// ============================================================================
// NAVIGATION AND PATTERN GENERATION
// ============================================================================

std::vector<geometry_msgs::msg::Point> MissionMultirotorNode::generate_search_waypoints(
    const geometry_msgs::msg::Polygon& boundary,
    const std::string& pattern_type,
    float spacing,
    float altitude)
{
    RCLCPP_INFO(this->get_logger(), "Generating %s search pattern with %.1fm spacing", 
                pattern_type.c_str(), spacing);
    
    if (pattern_type == "spiral") {
        return generate_spiral_pattern(boundary, spacing, altitude);
    } else if (pattern_type == "lawnmower") {
        return generate_lawnmower_pattern(boundary, spacing, altitude);
    } else if (pattern_type == "grid") {
        return generate_grid_pattern(boundary, spacing, altitude);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown search pattern: %s", pattern_type.c_str());
        return {};
    }
}

std::vector<geometry_msgs::msg::Point> MissionMultirotorNode::generate_spiral_pattern(
    const geometry_msgs::msg::Polygon& boundary,
    float spacing,
    float altitude)
{
    std::vector<geometry_msgs::msg::Point> waypoints;
    
    // Get center and approximate radius of search area
    geometry_msgs::msg::Point center = get_polygon_center(boundary);
    
    // Calculate approximate radius (distance to farthest point)
    float max_radius = 0.0f;
    for (const auto& point : boundary.points) {
        float dx = point.x - center.x;
        float dy = point.y - center.y;
        float distance = std::sqrt(dx*dx + dy*dy);
        max_radius = std::max(max_radius, distance);
    }
    
    // Generate Archimedes spiral: r = a * theta
    float a = spacing / (2.0f * M_PI);  // Spiral constant for desired spacing
    float angle = 0.0f;
    float angle_step = 0.1f;  // Radians per step
    
    RCLCPP_INFO(this->get_logger(), "Generating spiral pattern: center=(%.1f,%.1f), radius=%.1fm", 
                center.x, center.y, max_radius);
    
    while (true) {
        float radius = a * angle;
        
        if (radius > max_radius) {
            break;  // Reached outer boundary
        }
        
        geometry_msgs::msg::Point waypoint;
        waypoint.x = center.x + radius * std::cos(angle);
        waypoint.y = center.y + radius * std::sin(angle);
        waypoint.z = -altitude;  // AirSim uses NED coordinates (negative Z is up)
        
        waypoints.push_back(waypoint);
        angle += angle_step;
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for spiral pattern", waypoints.size());
    return waypoints;
}

std::vector<geometry_msgs::msg::Point> MissionMultirotorNode::generate_lawnmower_pattern(
    const geometry_msgs::msg::Polygon& boundary,
    float spacing,
    float altitude)
{
    std::vector<geometry_msgs::msg::Point> waypoints;
    
    if (boundary.points.empty()) {
        return waypoints;
    }
    
    // Find bounding box of the polygon
    float min_x = boundary.points[0].x, max_x = boundary.points[0].x;
    float min_y = boundary.points[0].y, max_y = boundary.points[0].y;
    
    for (const auto& point : boundary.points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Lawnmower pattern bounds: x=[%.1f,%.1f], y=[%.1f,%.1f]", 
                min_x, max_x, min_y, max_y);
    
    // Generate lawnmower pattern (alternating east-west sweeps)
    bool going_east = true;
    float current_y = min_y;
    
    while (current_y <= max_y) {
        if (going_east) {
            // Add waypoint at west end
            geometry_msgs::msg::Point wp_west;
            wp_west.x = min_x;
            wp_west.y = current_y;
            wp_west.z = -altitude;
            waypoints.push_back(wp_west);
            
            // Add waypoint at east end
            geometry_msgs::msg::Point wp_east;
            wp_east.x = max_x;
            wp_east.y = current_y;
            wp_east.z = -altitude;
            waypoints.push_back(wp_east);
        } else {
            // Add waypoint at east end
            geometry_msgs::msg::Point wp_east;
            wp_east.x = max_x;
            wp_east.y = current_y;
            wp_east.z = -altitude;
            waypoints.push_back(wp_east);
            
            // Add waypoint at west end
            geometry_msgs::msg::Point wp_west;
            wp_west.x = min_x;
            wp_west.y = current_y;
            wp_west.z = -altitude;
            waypoints.push_back(wp_west);
        }
        
        going_east = !going_east;
        current_y += spacing;
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for lawnmower pattern", waypoints.size());
    return waypoints;
}

std::vector<geometry_msgs::msg::Point> MissionMultirotorNode::generate_grid_pattern(
    const geometry_msgs::msg::Polygon& boundary,
    float spacing,
    float altitude)
{
    std::vector<geometry_msgs::msg::Point> waypoints;
    
    if (boundary.points.empty()) {
        return waypoints;
    }
    
    // Find bounding box
    float min_x = boundary.points[0].x, max_x = boundary.points[0].x;
    float min_y = boundary.points[0].y, max_y = boundary.points[0].y;
    
    for (const auto& point : boundary.points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Generate grid points
    for (float y = min_y; y <= max_y; y += spacing) {
        for (float x = min_x; x <= max_x; x += spacing) {
            geometry_msgs::msg::Point waypoint;
            waypoint.x = x;
            waypoint.y = y;
            waypoint.z = -altitude;
            waypoints.push_back(waypoint);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for grid pattern", waypoints.size());
    return waypoints;
}

bool MissionMultirotorNode::execute_waypoint_navigation(
    const std::vector<geometry_msgs::msg::Point>& waypoints,
    float speed,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAreaAction>> goal_handle)
{
    auto feedback = std::make_shared<SearchAreaAction::Feedback>();
    
    waypoints_completed_ = 0;
    
    for (size_t i = 0; i < waypoints.size() && !mission_cancellation_requested_.load(); ++i) {
        const auto& waypoint = waypoints[i];
        
        // Update feedback
        feedback->progress_percentage = (float(i) / waypoints.size()) * 100.0f;
        feedback->current_position = waypoint;
        feedback->waypoints_remaining = waypoints.size() - i;
        feedback->current_activity = "Navigating to waypoint " + std::to_string(i + 1);
        
        current_mission_progress_ = feedback->progress_percentage;
        current_activity_ = feedback->current_activity;
        
        // Navigate to waypoint
        bool navigation_success = navigate_to_waypoint(waypoint, speed);
        
        if (!navigation_success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint %zu", i + 1);
            return false;
        }
        
        waypoints_completed_++;
        
        // Perform detection if at waypoint
        auto detections = perform_target_detection();
        for (const auto& detection : detections) {
            publish_target_detection(detection);
            total_targets_detected_++;
        }
        
        feedback->targets_detected_so_far = total_targets_detected_;
        goal_handle->publish_feedback(feedback);
        
        // Small delay for detection processing
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return !mission_cancellation_requested_.load();
}

bool MissionMultirotorNode::navigate_to_waypoint(const geometry_msgs::msg::Point& waypoint, float speed)
{
    try {
        // Convert to AirSim coordinates and navigate using state client for low-latency navigation
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "State client not initialized for navigation");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Navigating to waypoint: (%.1f, %.1f, %.1f) at %.1fm/s", 
                    waypoint.x, waypoint.y, waypoint.z, speed);
        
        // Use moveToPositionAsync for smooth navigation
        multirotor_client->moveToPositionAsync(
            waypoint.x, waypoint.y, waypoint.z, speed, 60.0f, // 60 second timeout
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(),
            -1, 1, vehicle_name_);
        
        // Poll for navigation completion with proper timeout
        const int max_wait_seconds = 60;  // Maximum wait time for navigation
        const int check_interval_ms = 500;  // Check every 500ms
        int total_wait_time = 0;
        
        RCLCPP_INFO(this->get_logger(), "Polling for navigation completion...");
        
        while (total_wait_time < max_wait_seconds * 1000 && !mission_cancellation_requested_.load()) {
            // Check if we're close enough to the waypoint
            if (is_waypoint_reached(waypoint, 3.0f)) {
                RCLCPP_INFO(this->get_logger(), "Waypoint reached within tolerance");
                return true;
            }
            
            // Small delay before next check
            std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
            total_wait_time += check_interval_ms;
        }
        
        if (mission_cancellation_requested_.load()) {
            RCLCPP_INFO(this->get_logger(), "Navigation cancelled by user request");
            return false;
        }
        
        if (total_wait_time >= max_wait_seconds * 1000) {
            RCLCPP_WARN(this->get_logger(), "Navigation timeout after %d seconds", max_wait_seconds);
            // Still check if we reached the waypoint despite timeout
        }
        
        // Final verification that waypoint was reached
        bool waypoint_reached = is_waypoint_reached(waypoint, 3.0f);
        
        if (waypoint_reached) {
            RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint within tolerance");
        }
        
        return waypoint_reached;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Navigation error: %s", e.what());
        return false;
    }
}

bool MissionMultirotorNode::is_waypoint_reached(const geometry_msgs::msg::Point& waypoint, float tolerance)
{
    try {
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast to MultirotorRpcLibClient in waypoint check");
            return false;
        }
        
        auto state = multirotor_client->getMultirotorState(vehicle_name_);
        auto pos = state.getPosition();
        
        float dx = pos.x() - waypoint.x;
        float dy = pos.y() - waypoint.y;
        float dz = pos.z() - waypoint.z;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Separate horizontal and vertical distance for better debugging
        float horizontal_distance = std::sqrt(dx*dx + dy*dy);
        float vertical_distance = std::abs(dz);
        
        bool reached = distance <= tolerance;
        
        if (reached) {
            RCLCPP_DEBUG(this->get_logger(), "Waypoint reached: distance=%.2fm (tolerance=%.2fm)", 
                        distance, tolerance);
        } else {
            RCLCPP_DEBUG(this->get_logger(), 
                        "Waypoint check: distance=%.2fm (tolerance=%.2fm), horizontal=%.2fm, vertical=%.2fm\n"
                        "  Current: (%.1f, %.1f, %.1f)\n"
                        "  Target:  (%.1f, %.1f, %.1f)", 
                        distance, tolerance, horizontal_distance, vertical_distance,
                        pos.x(), pos.y(), pos.z(),
                        waypoint.x, waypoint.y, waypoint.z);
        }
        
        return reached;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error checking waypoint: %s", e.what());
        return false;
    }
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

float MissionMultirotorNode::calculate_polygon_area(const geometry_msgs::msg::Polygon& boundary)
{
    if (boundary.points.size() < 3) return 0.0f;
    
    // Shoelace formula for polygon area
    float area = 0.0f;
    size_t n = boundary.points.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += (boundary.points[i].x * boundary.points[j].y);
        area -= (boundary.points[j].x * boundary.points[i].y);
    }
    
    return std::abs(area) / 2.0f;
}

geometry_msgs::msg::Point MissionMultirotorNode::get_polygon_center(const geometry_msgs::msg::Polygon& boundary)
{
    geometry_msgs::msg::Point center;
    center.x = 0.0; center.y = 0.0; center.z = 0.0;
    
    if (boundary.points.empty()) return center;
    
    for (const auto& point : boundary.points) {
        center.x += point.x;
        center.y += point.y;
    }
    
    center.x /= boundary.points.size();
    center.y /= boundary.points.size();
    
    return center;
}

std::vector<mission_search_interfaces::msg::TargetDetection> MissionMultirotorNode::perform_target_detection()
{
    std::vector<mission_search_interfaces::msg::TargetDetection> detections;
    
    // Placeholder for actual target detection implementation
    // In a real implementation, this would:
    // 1. Get camera images from AirSim
    // 2. Run computer vision algorithms (YOLO, etc.)
    // 3. Process detection results
    // 4. Create TargetDetection messages
    
    // For now, return empty vector (no detections)
    return detections;
}

void MissionMultirotorNode::publish_mission_status(uint8_t status, float progress, const std::string& activity)
{
    try {
        mission_search_interfaces::msg::MissionStatus msg;
        
        // Fill header with timestamp
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";  // Reference frame
        
        // Mission identification
        msg.mission_id = current_mission_id_.empty() ? "unknown" : current_mission_id_;
        msg.vehicle_name = vehicle_name_;
        
        // Mission state
        msg.status = status;
        msg.progress_percentage = std::clamp(progress, 0.0f, 100.0f);  // Ensure valid range
        msg.current_activity = activity;
        
        // Mission metrics (thread-safe access to atomic variables)
        msg.targets_detected = total_targets_detected_.load();
        msg.waypoints_completed = waypoints_completed_.load();
        msg.area_covered_sq_m = total_area_covered_.load();
        
        // Timing information
        if (mission_active_.load()) {
            auto elapsed = std::chrono::steady_clock::now() - mission_start_time_;
            msg.mission_start_time = rclcpp::Time(
                std::chrono::duration_cast<std::chrono::nanoseconds>(mission_start_time_.time_since_epoch()).count());
            
            // Estimated completion time
            if (progress > 0.1f) {  // Only estimate if we have meaningful progress
                auto estimated_total_time = elapsed * (100.0f / progress);
                auto remaining_time = estimated_total_time - elapsed;
                msg.estimated_remaining_time = rclcpp::Duration(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(remaining_time));
            }
        }
        
        // Enhanced status messages for better debugging
        switch (status) {
            case mission_search_interfaces::msg::MissionStatus::ACTIVE:
                RCLCPP_DEBUG(this->get_logger(), "Mission status: ACTIVE - %s (%.1f%% complete)", 
                           activity.c_str(), progress);
                break;
            case mission_search_interfaces::msg::MissionStatus::COMPLETED:
                RCLCPP_INFO(this->get_logger(), "Mission status: COMPLETED - %s", activity.c_str());
                break;
            case mission_search_interfaces::msg::MissionStatus::FAILED:
                RCLCPP_ERROR(this->get_logger(), "Mission status: FAILED - %s", activity.c_str());
                break;
            case mission_search_interfaces::msg::MissionStatus::CANCELLED:
                RCLCPP_WARN(this->get_logger(), "Mission status: CANCELLED - %s", activity.c_str());
                break;
            default:
                RCLCPP_DEBUG(this->get_logger(), "Mission status: %d - %s", status, activity.c_str());
        }
        
        // Publish status (non-blocking)
        mission_status_pub_->publish(msg);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish mission status: %s", e.what());
    }
}

void MissionMultirotorNode::publish_target_detection(const mission_search_interfaces::msg::TargetDetection& detection)
{
    try {
        // Validate detection before publishing
        if (detection.detection_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "Target detection missing detection_id - skipping publish");
            return;
        }
        
        if (detection.target_type.empty()) {
            RCLCPP_WARN(this->get_logger(), "Target detection missing target_type - skipping publish");
            return;
        }
        
        // Publish the detection
        target_detection_pub_->publish(detection);
        
        // Enhanced logging with more context
        RCLCPP_INFO(this->get_logger(), 
                   "Published target detection [%s]: %s at (%.1f, %.1f, %.1f) with %.1f%% confidence", 
                   detection.detection_id.c_str(),
                   detection.target_type.c_str(), 
                   detection.world_position.x, 
                   detection.world_position.y,
                   detection.world_position.z,
                   detection.confidence_score * 100.0f);
        
        // Update detection counter
        total_targets_detected_.fetch_add(1);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish target detection: %s", e.what());
    }
}

// ============================================================================
// SERVICE CALLBACKS (Placeholder implementations)
// ============================================================================

bool MissionMultirotorNode::set_search_pattern_callback(
    const std::shared_ptr<mission_search_interfaces::srv::SetSearchPattern::Request> request,
    std::shared_ptr<mission_search_interfaces::srv::SetSearchPattern::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting search pattern: %s", request->pattern_type.c_str());
    
    // Store search configuration
    search_config_.pattern_type = request->pattern_type;
    search_config_.search_altitude = request->search_altitude;
    search_config_.search_speed = request->search_speed;
    search_config_.pattern_spacing = request->pattern_spacing;
    search_config_.enable_detection = request->enable_detection;
    search_config_.detection_confidence_threshold = request->detection_confidence_threshold;
    
    response->success = true;
    response->message = "Search pattern configured successfully";
    response->pattern_valid = true;
    response->estimated_coverage_area_sq_m = calculate_polygon_area(request->search_boundary);
    
    return true;
}

bool MissionMultirotorNode::get_vehicle_capabilities_callback(
    const std::shared_ptr<mission_search_interfaces::srv::GetVehicleCapabilities::Request> request,
    std::shared_ptr<mission_search_interfaces::srv::GetVehicleCapabilities::Response> response)
{
    (void)request;  // Suppress unused parameter warning
    
    response->success = true;
    response->message = "Vehicle capabilities retrieved";
    response->capabilities = get_current_capabilities();
    response->currently_available = !mission_active_.load();
    
    return true;
}

mission_search_interfaces::msg::VehicleCapabilities MissionMultirotorNode::get_current_capabilities()
{
    mission_search_interfaces::msg::VehicleCapabilities capabilities;
    
    capabilities.vehicle_name = vehicle_name_;
    capabilities.vehicle_type = "multirotor";
    capabilities.max_speed_ms = 15.0f;  // Reasonable max speed for search operations
    capabilities.max_altitude_m = 150.0f;
    capabilities.min_altitude_m = 5.0f;
    
    // Placeholder values - would be populated from actual vehicle state
    capabilities.battery_percentage = 85.0f;
    capabilities.available_for_mission = !mission_active_.load();
    capabilities.current_mission_id = mission_active_.load() ? current_mission_id_ : "";
    
    // Detection capabilities
    capabilities.available_cameras = {"front_center", "bottom_center"};
    capabilities.detection_types = {"person", "vehicle", "object"};
    capabilities.detection_range_m = 100.0f;
    capabilities.detection_accuracy = 0.8f;
    
    return capabilities;
}

void MissionMultirotorNode::reset_mission_state()
{
    RCLCPP_INFO(this->get_logger(), "Resetting mission state for new mission");
    
    // Reset mission counters and metrics (atomic variables)
    total_targets_detected_.store(0);
    total_area_covered_.store(0.0f);
    waypoints_completed_.store(0);
    current_mission_progress_ = 0.0f;
    
    // Reset mission timing
    mission_start_time_ = std::chrono::steady_clock::now();
    current_activity_ = "Preparing mission";
    
    // Reset detection state
    // Note: Don't reset mission_active_ or mission_cancellation_requested_ here
    // as they are managed by the action execution flow
    
    RCLCPP_DEBUG(this->get_logger(), "Mission state reset completed");
}

bool MissionMultirotorNode::ensure_mission_ready(float mission_altitude)
{
    RCLCPP_INFO(this->get_logger(), "Ensuring vehicle ready for mission at altitude %.1fm", mission_altitude);
    
    try {
        // Check AirSim client connection
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast to MultirotorRpcLibClient - AirSim connection issue");
            return false;
        }
        
        // Test connection with retry logic
        auto state = get_vehicle_state_with_retry(multirotor_client, 3);
        if (!state.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle state after retries - connection issue");
            return false;
        }
        
        auto pos = state.value().getPosition();
        float current_altitude = std::abs(pos.z());  // AirSim uses NED coordinates
        
        RCLCPP_INFO(this->get_logger(), "Current vehicle position: (%.1f, %.1f, %.1f), altitude: %.1fm", 
                    pos.x(), pos.y(), pos.z(), current_altitude);
        
        // Reset mission state for fresh start
        reset_mission_state();
        
        // Smart altitude check - if already flying at reasonable altitude, don't force takeoff
        if (is_vehicle_ready_for_mission(current_altitude, mission_altitude)) {
            RCLCPP_INFO(this->get_logger(), "Vehicle already ready for mission at current altitude %.1fm", current_altitude);
            return true;
        }

        // ENHANCED APPROACH: Use enhanced takeoff service for robust altitude positioning
        RCLCPP_INFO(this->get_logger(), "Using enhanced takeoff service to position vehicle at mission altitude %.1fm", mission_altitude);

        if (!call_enhanced_takeoff_service(mission_altitude)) {
            RCLCPP_ERROR(this->get_logger(), "Enhanced takeoff service failed for mission altitude %.1fm", mission_altitude);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Vehicle is ready for mission execution");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during mission readiness check: %s", e.what());
        return false;
    }
}

bool MissionMultirotorNode::call_enhanced_takeoff_service(float target_altitude)
{
    try {
        // Create service client for enhanced takeoff
        std::string takeoff_service_name = vehicle_name_ + "/takeoff";
        auto takeoff_client = this->create_client<airsim_interfaces::srv::Takeoff>(takeoff_service_name);

        RCLCPP_INFO(this->get_logger(), "Calling enhanced takeoff service: %s with target altitude %.1fm",
                   takeoff_service_name.c_str(), target_altitude);

        // Wait for service to be available
        if (!takeoff_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Enhanced takeoff service %s not available after 10s",
                        takeoff_service_name.c_str());
            return false;
        }

        // Create request with target altitude
        auto request = std::make_shared<airsim_interfaces::srv::Takeoff::Request>();
        request->wait_on_last_task = true;  // Wait for completion for mission reliability
        request->target_altitude = target_altitude;

        RCLCPP_INFO(this->get_logger(), "Sending enhanced takeoff request: target_altitude=%.1fm, wait_on_last_task=true",
                   target_altitude);

        // Call service synchronously for mission reliability
        auto future = takeoff_client->async_send_request(request);

        // Wait for response with timeout
        auto status = future.wait_for(std::chrono::seconds(120));  // Give 2 minutes for takeoff+positioning

        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Enhanced takeoff service call timed out after 120s");
            return false;
        }

        auto response = future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(),
                "Enhanced takeoff successful! Final altitude: %.1fm (target: %.1fm)",
                response->final_altitude, target_altitude);

            // Verify final altitude is reasonable for mission
            const float MISSION_ALTITUDE_TOLERANCE = 5.0f;
            if (std::abs(response->final_altitude - target_altitude) <= MISSION_ALTITUDE_TOLERANCE) {
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Final altitude %.1fm not close enough to target %.1fm (tolerance: %.1fm) - proceeding anyway",
                    response->final_altitude, target_altitude, MISSION_ALTITUDE_TOLERANCE);
                return true;  // Proceed anyway - close enough for mission operation
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Enhanced takeoff service failed. Final altitude: %.1fm (target: %.1fm)",
                response->final_altitude, target_altitude);
            return false;
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during enhanced takeoff service call: %s", e.what());
        return false;
    }
}

bool MissionMultirotorNode::ensure_vehicle_airborne(float target_altitude)
{
    try {
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get AirSim client for takeoff check");
            return false;
        }
        
        // Get current position with retry
        auto state_opt = get_vehicle_state_with_retry(multirotor_client, 3);
        if (!state_opt.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle state for takeoff check");
            return false;
        }
        
        auto state = state_opt.value();
        auto pos = state.getPosition();
        float current_altitude = std::abs(pos.z());  // AirSim uses NED coordinates (negative Z is up)
        
        RCLCPP_INFO(this->get_logger(), "Current altitude: %.1fm, target: %.1fm", 
                    current_altitude, target_altitude);
        
        // Improved altitude check logic
        const float MIN_FLIGHT_ALTITUDE = 3.0f;  // Minimum altitude to be considered airborne
        const float ALTITUDE_TOLERANCE = 5.0f;   // Tolerance for altitude matching
        
        // If target is 0 or very low, use a reasonable minimum flight altitude
        float effective_target = std::max(target_altitude, MIN_FLIGHT_ALTITUDE);
        
        // Check if vehicle is already airborne and at appropriate altitude
        bool is_airborne = current_altitude >= MIN_FLIGHT_ALTITUDE;
        bool altitude_acceptable = (current_altitude >= (effective_target - ALTITUDE_TOLERANCE)) && 
                                 (current_altitude <= (effective_target + ALTITUDE_TOLERANCE + 10.0f));
        
        if (is_airborne && altitude_acceptable) {
            RCLCPP_INFO(this->get_logger(), "Vehicle already airborne at acceptable altitude (%.1fm vs target %.1fm)", 
                       current_altitude, effective_target);
            return true;
        }
        
        if (is_airborne && current_altitude > effective_target) {
            RCLCPP_INFO(this->get_logger(), "Vehicle already airborne above target altitude (%.1fm vs %.1fm) - continuing", 
                       current_altitude, effective_target);
            return true;
        }
        
        // Vehicle needs takeoff or altitude adjustment
        if (!is_airborne) {
            RCLCPP_INFO(this->get_logger(), "Vehicle on ground (%.1fm), executing takeoff to %.1fm", 
                       current_altitude, effective_target);
            
            // Execute takeoff
            multirotor_client->takeoffAsync(60.0f, vehicle_name_);  // 60 second timeout
            
            // Wait for takeoff to complete
            if (!wait_for_takeoff_completion(multirotor_client, 30)) {
                return false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Vehicle airborne but at %.1fm, adjusting to %.1fm", 
                       current_altitude, effective_target);
        }
        
        // Navigate to exact target altitude if needed
        auto final_state = get_vehicle_state_with_retry(multirotor_client, 2);
        if (final_state.has_value()) {
            auto final_pos = final_state.value().getPosition();
            float final_altitude = std::abs(final_pos.z());
            
            if (std::abs(final_altitude - effective_target) > ALTITUDE_TOLERANCE) {
                RCLCPP_INFO(this->get_logger(), "Fine-tuning altitude from %.1fm to %.1fm", 
                           final_altitude, effective_target);
                geometry_msgs::msg::Point climb_waypoint;
                climb_waypoint.x = final_pos.x();
                climb_waypoint.y = final_pos.y();
                climb_waypoint.z = -effective_target;  // NED coordinates
                
                return navigate_to_waypoint(climb_waypoint, 3.0f);  // Slow climb
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during takeoff: %s", e.what());
        return false;
    }
}

// ============================================================================
// NAVIGATE TO TARGET ACTION IMPLEMENTATION  
// ============================================================================

rclcpp_action::GoalResponse MissionMultirotorNode::handle_navigate_to_target_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const NavigateToTargetAction::Goal> goal)
{
    (void)uuid;
    
    RCLCPP_INFO(this->get_logger(), "Received navigate to target goal for target: %s", 
                goal->target_id.c_str());
    
    // Check if already running a mission
    if (mission_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal - mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Basic validation with default value support
    if (goal->target_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid goal - target ID cannot be empty");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Allow 0.0 values as "use defaults" - only reject negative or excessive values
    if (goal->approach_altitude != 0.0f && (goal->approach_altitude < 5.0f || goal->approach_altitude > 150.0f)) {
        RCLCPP_WARN(this->get_logger(), "Approach altitude %f outside safe range (5-150m). Use 0.0 for default.", 
                    goal->approach_altitude);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Accept 0.0 as "use default speed", only reject negative or excessive speeds
    if (goal->navigation_speed < 0.0f || goal->navigation_speed > 20.0f) {
        RCLCPP_WARN(this->get_logger(), "Navigation speed %f outside safe range (0-20m/s). Use 0.0 for default.", 
                    goal->navigation_speed);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Log the effective values (showing defaults when 0.0)
    float effective_altitude = goal->approach_altitude > 0.0f ? goal->approach_altitude : 25.0f;
    float effective_speed = goal->navigation_speed > 0.0f ? goal->navigation_speed : 5.0f;
    
    RCLCPP_INFO(this->get_logger(), "Goal validation passed - altitude: %.1fm%s, speed: %.1fm/s%s", 
                effective_altitude, (goal->approach_altitude == 0.0f ? " (default)" : ""),
                effective_speed, (goal->navigation_speed == 0.0f ? " (default)" : ""));
    
    RCLCPP_INFO(this->get_logger(), "Accepting navigate to target goal - pattern: %s, speed: %.1fm/s", 
                (goal->approach_pattern.empty() ? "direct (default)" : goal->approach_pattern.c_str()), effective_speed);
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionMultirotorNode::handle_navigate_to_target_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for navigate to target goal");
    
    mission_cancellation_requested_.store(true);
    
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionMultirotorNode::handle_navigate_to_target_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Navigate to target goal accepted - starting execution");
    
    // Execute in separate thread to not block action server
    if (mission_execution_thread_.joinable()) {
        mission_execution_thread_.join();
    }
    
    mission_execution_thread_ = std::thread(&MissionMultirotorNode::execute_navigate_to_target, this, goal_handle);
}

void MissionMultirotorNode::execute_navigate_to_target(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting navigate to target execution thread");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToTargetAction::Feedback>();
    auto result = std::make_shared<NavigateToTargetAction::Result>();
    
    // Initialize mission state
    mission_active_.store(true);
    mission_cancellation_requested_.store(false);
    mission_start_time_ = std::chrono::steady_clock::now();
    
    try {
        // Apply defaults for zero values
        float effective_altitude = goal->approach_altitude > 0.0f ? goal->approach_altitude : 25.0f;
        float effective_speed = goal->navigation_speed > 0.0f ? goal->navigation_speed : 5.0f;
        float effective_standoff = goal->standoff_distance > 0.0f ? goal->standoff_distance : 2.0f;
        std::string effective_pattern = goal->approach_pattern.empty() ? "direct" : goal->approach_pattern;
        
        RCLCPP_INFO(this->get_logger(), "Using effective values - altitude: %.1fm, speed: %.1fm/s, standoff: %.1fm, pattern: %s", 
                    effective_altitude, effective_speed, effective_standoff, effective_pattern.c_str());
        
        // Ensure vehicle is ready for mission (includes takeoff if needed)
        feedback->current_phase = "preparing";
        feedback->phase_description = "Preparing vehicle for navigation";
        goal_handle->publish_feedback(feedback);
        
        if (!ensure_mission_ready(effective_altitude)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare vehicle for navigation mission");
            result->success = false;
            result->completion_reason = "vehicle_preparation_failed";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        // Phase 1: Navigation to target
        feedback->current_phase = "navigating";
        feedback->phase_description = "Navigating to target location";
        goal_handle->publish_feedback(feedback);
        
        // Calculate distance and bearing to target
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "State client not initialized for mission operations");
            result->success = false;
            result->completion_reason = "client_error";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        auto current_pos = current_state.getPosition();
        
        float dx = goal->target_location.x - current_pos.x();
        float dy = goal->target_location.y - current_pos.y();
        float total_distance = std::sqrt(dx*dx + dy*dy);
        
        RCLCPP_INFO(this->get_logger(), "Target distance: %.1fm, approaching at altitude %.1fm", 
                    total_distance, effective_altitude);
        
        // Navigate to approach position (considering standoff distance)
        geometry_msgs::msg::Point approach_point;
        if (effective_standoff > 0.0f) {
            // Calculate approach point with standoff distance
            float approach_distance = total_distance - effective_standoff;
            if (approach_distance > 0.0f) {
                float approach_ratio = approach_distance / total_distance;
                approach_point.x = current_pos.x() + dx * approach_ratio;
                approach_point.y = current_pos.y() + dy * approach_ratio;
            } else {
                approach_point = goal->target_location;
            }
        } else {
            approach_point = goal->target_location;
        }
        approach_point.z = -effective_altitude;
        
        // Execute navigation with feedback
        bool navigation_successful = true;
        
        if (effective_pattern == "direct") {
            navigation_successful = navigate_to_waypoint(approach_point, effective_speed);
        } else if (effective_pattern == "spiral") {
            float effective_radius = goal->orbit_radius > 0.0f ? goal->orbit_radius : 10.0f;
            navigation_successful = execute_spiral_approach(approach_point, effective_speed, effective_radius);
        } else {
            // Default to direct navigation
            navigation_successful = navigate_to_waypoint(approach_point, effective_speed);
        }
        
        if (!navigation_successful || mission_cancellation_requested_.load()) {
            if (mission_cancellation_requested_.load()) {
                result->success = false;
                result->completion_reason = "aborted";
                goal_handle->canceled(result);
            } else {
                result->success = false;
                result->completion_reason = "navigation_failed";
                goal_handle->abort(result);
            }
            mission_active_.store(false);
            return;
        }
        
        // Phase 2: Investigation (if specified)
        if (goal->investigation_time.sec > 0 || goal->investigation_time.nanosec > 0) {
            feedback->current_phase = "investigating";
            feedback->phase_description = "Investigating target";
            goal_handle->publish_feedback(feedback);
            
            // Execute investigation phase
            bool investigation_successful = execute_target_investigation(goal, feedback, goal_handle);
            
            if (!investigation_successful && !mission_cancellation_requested_.load()) {
                result->success = false;
                result->completion_reason = "investigation_failed";
                goal_handle->abort(result);
                mission_active_.store(false);
                return;
            }
        }
        
        // Mission completion
        auto mission_end_time = std::chrono::steady_clock::now();
        auto mission_duration = std::chrono::duration_cast<std::chrono::seconds>(mission_end_time - mission_start_time_);
        
        if (mission_cancellation_requested_.load()) {
            result->success = false;
            result->completion_reason = "aborted";
            goal_handle->canceled(result);
        } else {
            RCLCPP_INFO(this->get_logger(), "Navigate to target mission completed successfully");
            result->success = true;
            result->completion_reason = "completed";
            result->target_reached = true;
            result->navigation_time = rclcpp::Duration(mission_duration);
            result->investigation_completed = (goal->investigation_time.sec > 0 || goal->investigation_time.nanosec > 0);
            
            // Calculate final distance to target
            auto final_state = multirotor_client->getMultirotorState(vehicle_name_);
            auto final_pos = final_state.getPosition();
            float final_dx = goal->target_location.x - final_pos.x();
            float final_dy = goal->target_location.y - final_pos.y();
            result->final_distance_to_target = std::sqrt(final_dx*final_dx + final_dy*final_dy);
            
            result->final_position.x = final_pos.x();
            result->final_position.y = final_pos.y();
            result->final_position.z = final_pos.z();
            
            goal_handle->succeed(result);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during navigate to target execution: %s", e.what());
        result->success = false;
        result->completion_reason = "exception";
        goal_handle->abort(result);
    }
    
    mission_active_.store(false);
    RCLCPP_INFO(this->get_logger(), "Navigate to target execution thread completed");
}

// ============================================================================
// TRACK TARGET ACTION IMPLEMENTATION
// ============================================================================

rclcpp_action::GoalResponse MissionMultirotorNode::handle_track_target_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TrackTargetAction::Goal> goal)
{
    (void)uuid;
    
    RCLCPP_INFO(this->get_logger(), "Received track target goal for target: %s", 
                goal->target_id.c_str());
    
    // Check if already running a mission
    if (mission_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal - mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Basic validation
    if (goal->target_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid goal - target ID cannot be empty");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->tracking_altitude < 5.0f || goal->tracking_altitude > 150.0f) {
        RCLCPP_WARN(this->get_logger(), "Tracking altitude %f outside safe range (5-150m)", 
                    goal->tracking_altitude);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (goal->max_tracking_speed <= 0.0f || goal->max_tracking_speed > 25.0f) {
        RCLCPP_WARN(this->get_logger(), "Max tracking speed %f outside safe range (0-25m/s)", 
                    goal->max_tracking_speed);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "Accepting track target goal - mode: %s, max speed: %.1fm/s", 
                goal->tracking_mode.c_str(), goal->max_tracking_speed);
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionMultirotorNode::handle_track_target_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request for track target goal");
    
    mission_cancellation_requested_.store(true);
    
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionMultirotorNode::handle_track_target_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Track target goal accepted - starting execution");
    
    // Execute in separate thread to not block action server
    if (mission_execution_thread_.joinable()) {
        mission_execution_thread_.join();
    }
    
    mission_execution_thread_ = std::thread(&MissionMultirotorNode::execute_track_target, this, goal_handle);
}

void MissionMultirotorNode::execute_track_target(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrackTargetAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting track target execution thread");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrackTargetAction::Feedback>();
    auto result = std::make_shared<TrackTargetAction::Result>();
    
    // Initialize mission state
    mission_active_.store(true);
    mission_cancellation_requested_.store(false);
    mission_start_time_ = std::chrono::steady_clock::now();
    
    try {
        // Ensure vehicle is ready for mission (includes takeoff if needed)
        feedback->status_message = "Preparing vehicle for tracking mission";
        goal_handle->publish_feedback(feedback);
        
        if (!ensure_mission_ready(goal->tracking_altitude)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare vehicle for tracking mission");
            result->success = false;
            result->completion_reason = "vehicle_preparation_failed";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "State client not initialized for mission operations");
            result->success = false;
            result->completion_reason = "client_error";
            goal_handle->abort(result);
            mission_active_.store(false);
            return;
        }
        
        // Navigate to initial target position
        geometry_msgs::msg::Point tracking_position = goal->initial_target_location;
        tracking_position.z = -goal->tracking_altitude;
        
        RCLCPP_INFO(this->get_logger(), "Navigating to initial target position: (%.1f, %.1f, %.1f)", 
                    tracking_position.x, tracking_position.y, tracking_position.z);
        
        bool initial_navigation_successful = navigate_to_waypoint(tracking_position, goal->max_tracking_speed);
        
        if (!initial_navigation_successful || mission_cancellation_requested_.load()) {
            result->success = false;
            result->completion_reason = mission_cancellation_requested_.load() ? "aborted" : "navigation_failed";
            goal_handle->canceled(result);
            mission_active_.store(false);
            return;
        }
        
        // Execute tracking loop
        auto tracking_start_time = std::chrono::steady_clock::now();
        auto max_tracking_duration = std::chrono::seconds(goal->max_tracking_time.sec) + 
                                   std::chrono::nanoseconds(goal->max_tracking_time.nanosec);
        
        geometry_msgs::msg::Point current_target_location = goal->initial_target_location;
        uint32_t images_captured = 0;
        uint32_t target_lost_count = 0;
        
        while (!mission_cancellation_requested_.load()) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = current_time - tracking_start_time;
            
            // Check if max tracking time exceeded
            if (elapsed_time >= max_tracking_duration) {
                RCLCPP_INFO(this->get_logger(), "Max tracking time reached");
                break;
            }
            
            // Update feedback
            auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(elapsed_time);
            auto max_duration = std::chrono::duration_cast<std::chrono::seconds>(max_tracking_duration);
            feedback->progress_percentage = (float(total_duration.count()) / float(max_duration.count())) * 100.0f;
            
            auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
            auto current_pos = current_state.getPosition();
            
            feedback->current_position.x = current_pos.x();
            feedback->current_position.y = current_pos.y();
            feedback->current_position.z = current_pos.z();
            feedback->current_target_location = current_target_location;
            
            float dx = current_target_location.x - current_pos.x();
            float dy = current_target_location.y - current_pos.y();
            feedback->current_distance_to_target = std::sqrt(dx*dx + dy*dy);
            
            // Simple tracking logic (in real implementation would use computer vision)
            feedback->target_in_sight = (feedback->current_distance_to_target < 50.0f);
            feedback->visual_tracking_quality = feedback->target_in_sight ? 0.8f : 0.0f;
            feedback->tracking_difficulty = "moderate";
            feedback->current_target_behavior = "moving";
            
            if (!feedback->target_in_sight) {
                target_lost_count++;
                feedback->target_behaving_unexpectedly = true;
            } else {
                feedback->target_behaving_unexpectedly = false;
            }
            
            feedback->images_captured_so_far = images_captured;
            feedback->status_message = "Tracking target " + goal->target_id;
            
            goal_handle->publish_feedback(feedback);
            
            // Simulate target movement and tracking (placeholder)
            if (feedback->target_in_sight) {
                // Maintain tracking distance
                if (feedback->current_distance_to_target > goal->tracking_distance * 1.5f) {
                    // Move closer to target
                    geometry_msgs::msg::Point new_position;
                    float approach_factor = 0.7f;
                    new_position.x = current_pos.x() + dx * approach_factor;
                    new_position.y = current_pos.y() + dy * approach_factor;
                    new_position.z = -goal->tracking_altitude;
                    
                    navigate_to_waypoint(new_position, goal->max_tracking_speed * 0.5f);
                }
                
                // Capture images periodically
                if (goal->image_capture_interval > 0.0f) {
                    images_captured++;
                }
            }
            
            // Sleep for tracking update interval
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Mission completion
        auto mission_end_time = std::chrono::steady_clock::now();
        auto mission_duration = std::chrono::duration_cast<std::chrono::seconds>(mission_end_time - mission_start_time_);
        
        if (mission_cancellation_requested_.load()) {
            result->success = false;
            result->completion_reason = "aborted";
            goal_handle->canceled(result);
        } else {
            RCLCPP_INFO(this->get_logger(), "Track target mission completed successfully");
            result->success = true;
            result->completion_reason = "target_stopped";
            result->total_tracking_time = rclcpp::Duration(mission_duration);
            result->images_captured = images_captured;
            result->final_target_location = current_target_location;
            result->target_lost_incidents = target_lost_count;
            result->final_target_classification = "tracked_object";
            result->final_confidence_score = 0.8f;
            
            goal_handle->succeed(result);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during track target execution: %s", e.what());
        result->success = false;
        result->completion_reason = "exception";
        goal_handle->abort(result);
    }
    
    mission_active_.store(false);
    RCLCPP_INFO(this->get_logger(), "Track target execution thread completed");
}

// ============================================================================
// HELPER METHODS FOR NAVIGATION AND INVESTIGATION
// ============================================================================

bool MissionMultirotorNode::execute_spiral_approach(const geometry_msgs::msg::Point& target_point, float speed, float radius)
{
    try {
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "State client not initialized for mission operations");
            return false;
        }
        
        // Get current state for spiral calculation
        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        
        // Calculate spiral parameters
        float effective_radius = std::max(radius, 10.0f);  // Minimum 10m radius
        int num_spiral_points = 8;  // Number of points in spiral
        
        // Generate spiral waypoints around target
        for (int i = 0; i < num_spiral_points && !mission_cancellation_requested_.load(); ++i) {
            float angle = (float(i) / float(num_spiral_points)) * 2.0f * M_PI;
            float spiral_radius = effective_radius * (1.0f - float(i) / float(num_spiral_points * 2));
            
            geometry_msgs::msg::Point spiral_point;
            spiral_point.x = target_point.x + spiral_radius * std::cos(angle);
            spiral_point.y = target_point.y + spiral_radius * std::sin(angle);
            spiral_point.z = target_point.z;  // Maintain target altitude
            
            RCLCPP_DEBUG(this->get_logger(), "Spiral point %d: (%.1f, %.1f, %.1f)", 
                        i, spiral_point.x, spiral_point.y, spiral_point.z);
            
            bool navigation_success = navigate_to_waypoint(spiral_point, speed);
            if (!navigation_success) {
                RCLCPP_WARN(this->get_logger(), "Failed to reach spiral waypoint %d", i);
                return false;
            }
            
            // Brief pause at each spiral point for stability
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Finally navigate directly to target
        return navigate_to_waypoint(target_point, speed * 0.7f);  // Slower final approach
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in spiral approach: %s", e.what());
        return false;
    }
}

bool MissionMultirotorNode::execute_target_investigation(
    std::shared_ptr<const NavigateToTargetAction::Goal> goal,
    std::shared_ptr<NavigateToTargetAction::Feedback> feedback,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToTargetAction>> goal_handle)
{
    try {
        auto investigation_start_time = std::chrono::steady_clock::now();
        auto investigation_duration = std::chrono::seconds(goal->investigation_time.sec) + 
                                    std::chrono::nanoseconds(goal->investigation_time.nanosec);
        
        RCLCPP_INFO(this->get_logger(), "Starting target investigation for %d seconds", 
                    static_cast<int>(investigation_duration.count()));
        
        uint32_t images_captured = 0;
        std::vector<std::string> completed_angles;
        
        // Investigation loop
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = current_time - investigation_start_time;
            
            if (elapsed_time >= investigation_duration || mission_cancellation_requested_.load()) {
                break;
            }
            
            // Update feedback
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed_time);
            auto total_seconds = std::chrono::duration_cast<std::chrono::seconds>(investigation_duration);
            feedback->investigation_completeness = float(elapsed_seconds.count()) / float(total_seconds.count());
            feedback->images_captured_so_far = images_captured;
            feedback->completed_camera_angles = completed_angles;
            
            goal_handle->publish_feedback(feedback);
            
            // Simulate image capture and different camera angles
            if (goal->capture_detailed_images) {
                images_captured++;
                
                // Simulate capturing different angles
                if (elapsed_seconds.count() % 3 == 0) {  // Every 3 seconds
                    if (std::find(completed_angles.begin(), completed_angles.end(), "front") == completed_angles.end()) {
                        completed_angles.push_back("front");
                        RCLCPP_INFO(this->get_logger(), "Captured front angle image");
                    }
                } else if (elapsed_seconds.count() % 3 == 1) {
                    if (std::find(completed_angles.begin(), completed_angles.end(), "side") == completed_angles.end()) {
                        completed_angles.push_back("side");
                        RCLCPP_INFO(this->get_logger(), "Captured side angle image");
                    }
                } else {
                    if (std::find(completed_angles.begin(), completed_angles.end(), "top") == completed_angles.end()) {
                        completed_angles.push_back("top");
                        RCLCPP_INFO(this->get_logger(), "Captured top angle image");
                    }
                }
            }
            
            // Brief pause for investigation processing
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        RCLCPP_INFO(this->get_logger(), "Investigation completed - captured %d images, %zu angles", 
                    images_captured, completed_angles.size());
        
        return !mission_cancellation_requested_.load();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during target investigation: %s", e.what());
        return false;
    }
}

// ============================================================================
// UTILITY FUNCTIONS FOR RELIABLE RPC COMMUNICATION
// ============================================================================

std::optional<msr::airlib::MultirotorState> MissionMultirotorNode::get_vehicle_state_with_retry(
    msr::airlib::MultirotorRpcLibClient* client, int max_retries)
{
    for (int attempt = 0; attempt < max_retries; ++attempt) {
        try {
            RCLCPP_DEBUG(this->get_logger(), "RPC attempt %d: Getting vehicle state for %s", 
                        attempt + 1, vehicle_name_.c_str());
            
            auto state = client->getMultirotorState(vehicle_name_);
            
            RCLCPP_DEBUG(this->get_logger(), "RPC attempt %d successful - got vehicle state", attempt + 1);
            return state;  // Success
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "RPC attempt %d/%d failed for vehicle %s: %s", 
                       attempt + 1, max_retries, vehicle_name_.c_str(), e.what());
            
            if (attempt < max_retries - 1) {
                // Progressive backoff: 200ms, 500ms, 1000ms
                int delay_ms = 200 + (attempt * 300);
                RCLCPP_DEBUG(this->get_logger(), "Retrying in %dms...", delay_ms);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "All %d RPC attempts failed for vehicle %s", 
                max_retries, vehicle_name_.c_str());
    return std::nullopt;  // All attempts failed
}

bool MissionMultirotorNode::is_vehicle_ready_for_mission(float current_altitude, float mission_altitude)
{
    const float MIN_FLIGHT_ALTITUDE = 3.0f;    // Minimum altitude to be considered airborne
    const float ALTITUDE_TOLERANCE = 8.0f;     // Tolerance for altitude matching (increased for robustness)
    const float MAX_REASONABLE_ALTITUDE = 200.0f;  // Sanity check for altitude readings
    
    RCLCPP_DEBUG(this->get_logger(), "Checking vehicle readiness: current=%.1fm, mission=%.1fm", 
                current_altitude, mission_altitude);
    
    // Sanity check - reject obviously invalid altitude readings
    if (current_altitude < 0.0f || current_altitude > MAX_REASONABLE_ALTITUDE) {
        RCLCPP_WARN(this->get_logger(), "Invalid altitude reading: %.1fm (expected 0-%.1fm)", 
                   current_altitude, MAX_REASONABLE_ALTITUDE);
        return false;
    }
    
    // If target is 0 or very low, use a reasonable minimum flight altitude
    float effective_mission_altitude = std::max(mission_altitude, MIN_FLIGHT_ALTITUDE);
    
    // Check if vehicle is airborne
    bool is_airborne = current_altitude >= MIN_FLIGHT_ALTITUDE;
    if (!is_airborne) {
        RCLCPP_INFO(this->get_logger(), "Vehicle not airborne (%.1fm < %.1fm) - needs takeoff", 
                   current_altitude, MIN_FLIGHT_ALTITUDE);
        return false;
    }
    
    // Check if at appropriate altitude for mission
    float altitude_diff = std::abs(current_altitude - effective_mission_altitude);
    bool altitude_acceptable = altitude_diff <= ALTITUDE_TOLERANCE;
    
    if (!altitude_acceptable) {
        RCLCPP_INFO(this->get_logger(), 
                   "Vehicle altitude %.1fm not suitable for mission altitude %.1fm (diff=%.1fm > tolerance=%.1fm)", 
                   current_altitude, effective_mission_altitude, altitude_diff, ALTITUDE_TOLERANCE);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Vehicle ready for mission - airborne at %.1fm, mission altitude %.1fm", 
               current_altitude, effective_mission_altitude);
    return true;
}

bool MissionMultirotorNode::wait_for_takeoff_completion(msr::airlib::MultirotorRpcLibClient* client, int timeout_seconds)
{
    const float MIN_TAKEOFF_ALTITUDE = 5.0f;  // Minimum altitude to consider takeoff complete
    const int CHECK_INTERVAL_MS = 500;        // Check every 500ms
    
    int elapsed_time = 0;
    
    RCLCPP_INFO(this->get_logger(), "Waiting for takeoff completion (timeout: %d seconds)", timeout_seconds);
    
    while (elapsed_time < timeout_seconds * 1000) {
        try {
            // Check current altitude with retry logic
            auto state_opt = get_vehicle_state_with_retry(client, 2);
            if (state_opt.has_value()) {
                auto pos = state_opt.value().getPosition();
                float current_altitude = std::abs(pos.z());
                
                RCLCPP_DEBUG(this->get_logger(), "Takeoff progress: %.1fm (target: %.1fm)", 
                            current_altitude, MIN_TAKEOFF_ALTITUDE);
                
                // Check if we've reached minimum takeoff altitude
                if (current_altitude >= MIN_TAKEOFF_ALTITUDE) {
                    RCLCPP_INFO(this->get_logger(), "Takeoff completed successfully - reached %.1fm", current_altitude);
                    return true;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to get vehicle state during takeoff check");
            }
            
            // Check for cancellation
            if (mission_cancellation_requested_.load()) {
                RCLCPP_WARN(this->get_logger(), "Takeoff cancelled by user request");
                return false;
            }
            
            // Wait before next check
            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_INTERVAL_MS));
            elapsed_time += CHECK_INTERVAL_MS;
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception during takeoff monitoring: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_INTERVAL_MS));
            elapsed_time += CHECK_INTERVAL_MS;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Takeoff timeout after %d seconds", timeout_seconds);
    return false;
}

// ============================================================================
// MISSION EVENT DETECTION IMPLEMENTATION
// ============================================================================

void MissionMultirotorNode::initialize_event_detection()
{
    RCLCPP_INFO(this->get_logger(), "Initializing mission event detection for vehicle: %s", vehicle_name_.c_str());

    // Initialize event detection parameters
    position_change_threshold_ = 0.5;  // 0.5 meters
    velocity_change_threshold_ = 1.0;  // 1.0 m/s
    altitude_change_threshold_ = 0.3;  // 0.3 meters
    event_time_threshold_ = 1.0;       // 1.0 seconds between similar events

    // Initialize state tracking
    previous_position_ = geometry_msgs::msg::Point();
    previous_velocity_ = geometry_msgs::msg::Vector3();
    previous_altitude_ = 0.0f;
    event_sequence_number_ = 0;
    event_detection_initialized_ = false;

    // Initialize timing
    last_event_time_ = std::chrono::steady_clock::now();
    last_significant_movement_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Mission event detection initialized - thresholds: pos=%.1fm, vel=%.1fm/s, alt=%.1fm",
                position_change_threshold_, velocity_change_threshold_, altitude_change_threshold_);
}

void MissionMultirotorNode::setup_mission_flight_services()
{
    std::string topic_prefix = vehicle_name_ + "/";

    try {
        // Create enhanced takeoff service with event generation
        mission_takeoff_service_ = this->create_service<airsim_interfaces::srv::Takeoff>(
            topic_prefix + "takeoff",
            std::bind(&MissionMultirotorNode::mission_takeoff_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            mission_callback_group_);

        mission_land_service_ = this->create_service<airsim_interfaces::srv::Land>(
            topic_prefix + "land",
            std::bind(&MissionMultirotorNode::mission_land_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            mission_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Mission flight services created for %s:", vehicle_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Enhanced takeoff: %stakeoff", topic_prefix.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Enhanced land: %sland", topic_prefix.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup mission flight services for %s: %s", vehicle_name_.c_str(), e.what());
    }
}

bool MissionMultirotorNode::mission_takeoff_callback(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    try {
        // Generate immediate TAKEOFF event before calling AirSim API
        auto current_time = std::chrono::steady_clock::now();
        std::string call_id = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count());

        RCLCPP_INFO(this->get_logger(), "Mission takeoff service called for %s (ID: %s)", vehicle_name_.c_str(), call_id.c_str());

        // Generate SERVICE_CALL event immediately
        generate_service_event("TAKEOFF", "SERVICE_CALL", "takeoff", call_id);

        // Use parent class enhanced takeoff implementation
        // (includes API control, arming, takeoff, and altitude positioning)
        auto temp_response = std::make_shared<airsim_interfaces::srv::Takeoff::Response>();
        bool callback_success = MultirotorNode::takeoff_callback(request, temp_response);

        // Copy result from enhanced callback
        response->success = temp_response->success && callback_success;

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Mission takeoff sequence completed for: %s", vehicle_name_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Mission takeoff sequence failed for: %s", vehicle_name_.c_str());
        }
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in mission takeoff for %s: %s", vehicle_name_.c_str(), e.what());
        response->success = false;
        return false;
    }
}

bool MissionMultirotorNode::mission_land_callback(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    try {
        // Generate immediate LANDING event before calling AirSim API
        auto current_time = std::chrono::steady_clock::now();
        std::string call_id = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count());

        RCLCPP_INFO(this->get_logger(), "Mission land service called for %s (ID: %s)", vehicle_name_.c_str(), call_id.c_str());

        // Generate SERVICE_CALL event immediately
        generate_service_event("LANDING", "SERVICE_CALL", "land", call_id);

        // Use parent class enhanced land implementation
        // (includes landing execution, disarming, and API control cleanup)
        auto temp_response = std::make_shared<airsim_interfaces::srv::Land::Response>();
        bool callback_success = MultirotorNode::land_callback(request, temp_response);

        // Copy result from enhanced callback
        response->success = temp_response->success && callback_success;

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Mission landing sequence completed for: %s", vehicle_name_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Mission landing sequence failed for: %s", vehicle_name_.c_str());
        }
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in mission landing for %s: %s", vehicle_name_.c_str(), e.what());
        response->success = false;
        return false;
    }
}



void MissionMultirotorNode::generate_service_event(const std::string& event_type, const std::string& event_source,
                                                   const std::string& service_name, const std::string& call_id)
{
    try {
        auto event = create_mission_event(event_type);

        // Override event source to indicate service call origin
        event.event_source = event_source;
        event.event_description = "Vehicle " + vehicle_name_ + " " + event_type + " initiated via " + service_name + " service";

        // Add service call metadata
        event.tags.push_back("service_call_id:" + call_id);
        event.tags.push_back("service_name:" + service_name);
        event.tags.push_back("source:mission_node");

        // Set high confidence for service-based events
        event.confidence_score = 0.95f;

        // Current state information (if available)
        try {
            auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
            if (multirotor_client) {
                auto drone_state = multirotor_client->getMultirotorState(vehicle_name_);
                auto current_pos = drone_state.getPosition();

                event.current_position.x = current_pos.x();
                event.current_position.y = current_pos.y();
                event.current_position.z = current_pos.z();
                event.current_altitude = std::abs(current_pos.z());

                event.previous_position = previous_position_;
                event.previous_altitude = previous_altitude_;

                // Set movement metrics to 0 for immediate service events
                event.distance_moved = 0.0f;
                event.altitude_change = 0.0f;
                event.speed_change = 0.0f;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not get current state for service event: %s", e.what());
        }

        publish_mission_event(event);

        RCLCPP_INFO(this->get_logger(), "Generated SERVICE-BASED %s event for %s (ID: %s)",
                    event_type.c_str(), vehicle_name_.c_str(), call_id.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate service event for %s: %s", vehicle_name_.c_str(), e.what());
    }
}

void MissionMultirotorNode::process_state_changes()
{
    // Override base class virtual method to implement mission event detection
    detect_and_publish_mission_events();
}

void MissionMultirotorNode::detect_and_publish_mission_events()
{
    try {
        auto current_time = std::chrono::steady_clock::now();
        
        // Skip detection if too soon since last event
        auto time_since_last_event = current_time - last_event_time_;
        if (time_since_last_event < std::chrono::duration<double>(event_time_threshold_)) {
            return;
        }
        
        // Get current state from multirotor client
        auto multirotor_client = get_state_client<msr::airlib::MultirotorRpcLibClient>();
        if (!multirotor_client) {
            return; // Skip if client not available
        }
        
        auto drone_state = multirotor_client->getMultirotorState(vehicle_name_);
        auto current_pos = drone_state.getPosition();
        auto current_vel = drone_state.kinematics_estimated.twist.linear;
        
        // Convert AirSim coordinates to ROS message format
        geometry_msgs::msg::Point current_position;
        current_position.x = current_pos.x();
        current_position.y = current_pos.y();
        current_position.z = current_pos.z();
        
        geometry_msgs::msg::Vector3 current_velocity;
        current_velocity.x = current_vel.x();
        current_velocity.y = current_vel.y();
        current_velocity.z = current_vel.z();
        
        float current_altitude = std::abs(current_pos.z()); // AirSim uses NED coordinates
        
        // Skip first iteration to establish baseline
        if (!event_detection_initialized_) {
            previous_position_ = current_position;
            previous_velocity_ = current_velocity;
            previous_altitude_ = current_altitude;
            event_detection_initialized_ = true;
            return;
        }
        
        // Detect significant changes with service call context awareness
        std::string event_type = "";
        std::string enhanced_source = determine_event_source();

        // Check if we have recent service calls that might explain the movement
        bool has_recent_takeoff_call = false;
        bool has_recent_land_call = false;
        {
            std::lock_guard<std::mutex> lock(service_calls_mutex_);
            auto now = std::chrono::steady_clock::now();
            for (auto& call : pending_service_calls_) {
                auto time_since_call = std::chrono::duration_cast<std::chrono::seconds>(now - call.timestamp).count();
                if (time_since_call < 10) {  // Within 10 seconds of service call
                    if (call.service_name == "takeoff" && !call.completed) {
                        has_recent_takeoff_call = true;
                    } else if (call.service_name == "land" && !call.completed) {
                        has_recent_land_call = true;
                    }
                }
            }
        }

        // Enhanced event classification with service call context
        if (has_recent_takeoff_call && current_altitude > previous_altitude_ + 0.5f) {
            event_type = "TAKEOFF_CONFIRMED";
            enhanced_source = "SERVICE_CORRELATION";
        } else if (has_recent_land_call && current_altitude < previous_altitude_ - 0.3f) {
            event_type = "LANDING_CONFIRMED";
            enhanced_source = "SERVICE_CORRELATION";
        }
        // Traditional altitude-based detection (for cases without service calls)
        else if (previous_altitude_ < 1.0f && current_altitude > 3.0f) {
            event_type = "TAKEOFF";
        } else if (previous_altitude_ > 3.0f && current_altitude < 1.0f) {
            event_type = "LANDING";
        }
        // Check for movement events
        else if (detect_significant_position_change(current_position)) {
            event_type = "MOVEMENT";
        }
        // Check for velocity changes (command execution detection)
        else if (detect_significant_velocity_change(current_velocity)) {
            event_type = "COMMAND_START";
        }
        
        // Publish event if detected
        if (!event_type.empty()) {
            auto event = create_mission_event(event_type);

            // Use enhanced source if available
            if (enhanced_source != "UNKNOWN") {
                event.event_source = enhanced_source;
            }
            
            // Populate state information
            event.previous_position = previous_position_;
            event.current_position = current_position;
            event.previous_velocity = previous_velocity_;
            event.current_velocity = current_velocity;
            event.previous_altitude = previous_altitude_;
            event.current_altitude = current_altitude;
            
            // Calculate movement analysis
            float dx = current_position.x - previous_position_.x;
            float dy = current_position.y - previous_position_.y;
            float dz = current_position.z - previous_position_.z;
            event.distance_moved = std::sqrt(dx*dx + dy*dy + dz*dz);
            event.altitude_change = current_altitude - previous_altitude_;
            
            float prev_speed = std::sqrt(previous_velocity_.x*previous_velocity_.x + 
                                       previous_velocity_.y*previous_velocity_.y + 
                                       previous_velocity_.z*previous_velocity_.z);
            float curr_speed = std::sqrt(current_velocity.x*current_velocity.x + 
                                       current_velocity.y*current_velocity.y + 
                                       current_velocity.z*current_velocity.z);
            event.speed_change = std::abs(curr_speed - prev_speed);
            
            auto elapsed_time = current_time - last_event_time_;
            event.time_since_last_event = rclcpp::Duration(
                std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed_time));
            
            // Set detection triggers
            event.significant_position_change = (event_type == "MOVEMENT" || event_type == "TAKEOFF" || event_type == "LANDING");
            event.significant_velocity_change = (event_type == "COMMAND_START");
            event.significant_altitude_change = (std::abs(event.altitude_change) > altitude_change_threshold_);
            
            publish_mission_event(event);

            // Mark service calls as completed if this event confirms them
            if (event_type == "TAKEOFF_CONFIRMED" || event_type == "LANDING_CONFIRMED") {
                std::lock_guard<std::mutex> lock(service_calls_mutex_);
                for (auto& call : pending_service_calls_) {
                    if ((event_type == "TAKEOFF_CONFIRMED" && call.service_name == "takeoff") ||
                        (event_type == "LANDING_CONFIRMED" && call.service_name == "land")) {
                        call.completed = true;
                        call.success = true;
                        RCLCPP_INFO(this->get_logger(), "Service call %s confirmed by movement for %s",
                                   call.call_id.c_str(), vehicle_name_.c_str());
                    }
                }
            }
            
            // Update state tracking
            previous_position_ = current_position;
            previous_velocity_ = current_velocity;
            previous_altitude_ = current_altitude;
            last_event_time_ = current_time;
            
            if (event_type == "MOVEMENT") {
                last_significant_movement_ = current_time;
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "Exception in mission event detection: %s", e.what());
    }
}

bool MissionMultirotorNode::detect_significant_position_change(const geometry_msgs::msg::Point& current_pos)
{
    float dx = current_pos.x - previous_position_.x;
    float dy = current_pos.y - previous_position_.y;
    float dz = current_pos.z - previous_position_.z;
    
    float horizontal_distance = std::sqrt(dx*dx + dy*dy);
    float vertical_distance = std::abs(dz);
    
    return (horizontal_distance > position_change_threshold_) || 
           (vertical_distance > altitude_change_threshold_);
}

bool MissionMultirotorNode::detect_significant_velocity_change(const geometry_msgs::msg::Vector3& current_vel)
{
    float prev_speed = std::sqrt(previous_velocity_.x*previous_velocity_.x + 
                               previous_velocity_.y*previous_velocity_.y + 
                               previous_velocity_.z*previous_velocity_.z);
    float curr_speed = std::sqrt(current_vel.x*current_vel.x + 
                               current_vel.y*current_vel.y + 
                               current_vel.z*current_vel.z);
    
    return std::abs(curr_speed - prev_speed) > velocity_change_threshold_;
}

bool MissionMultirotorNode::detect_significant_altitude_change(float current_alt)
{
    return std::abs(current_alt - previous_altitude_) > altitude_change_threshold_;
}

std::string MissionMultirotorNode::classify_event_type(const std::string& change_type, float altitude_change)
{
    if (change_type == "altitude") {
        if (previous_altitude_ < 1.0f && altitude_change > 2.0f) {
            return "TAKEOFF";
        } else if (previous_altitude_ > 3.0f && altitude_change < -2.0f) {
            return "LANDING";
        }
    }
    return "MOVEMENT";
}

std::string MissionMultirotorNode::determine_event_source()
{
    // Check if mission is currently active
    if (mission_active_.load()) {
        return "ROS2_ACTION";
    }
    
    // Check time since last significant movement for patterns
    auto time_since_movement = std::chrono::steady_clock::now() - last_significant_movement_;
    if (time_since_movement < std::chrono::seconds(2)) {
        // Recent movement suggests direct client commands
        return "DIRECT_CLIENT";
    }
    
    return "UNKNOWN";
}

mission_search_interfaces::msg::MissionEvent MissionMultirotorNode::create_mission_event(const std::string& event_type)
{
    mission_search_interfaces::msg::MissionEvent event;
    
    // Header with timestamp
    event.header.stamp = this->get_clock()->now();
    event.header.frame_id = vehicle_name_;
    
    // Event identification
    event.event_id = vehicle_name_ + "_event_" + std::to_string(++event_sequence_number_);
    event.vehicle_name = vehicle_name_;
    event.sequence_number = event_sequence_number_;
    
    // Event classification
    event.event_type = event_type;
    event.event_source = determine_event_source();
    event.event_description = "Vehicle " + vehicle_name_ + " " + event_type + " detected";
    
    // Mission context (leverage existing mission infrastructure)
    if (mission_active_.load()) {
        event.active_mission_id = current_mission_id_;
        event.mission_phase = "EXECUTION"; // Could be enhanced based on current_activity_
        event.mission_progress_percentage = current_mission_progress_;
    } else {
        event.active_mission_id = "";
        event.mission_phase = "IDLE";
        event.mission_progress_percentage = 0.0f;
    }
    
    // Additional metadata
    event.tags = {"auto_detected"};
    if (mission_active_.load()) {
        event.tags.push_back("mission_context");
    }
    event.confidence_score = 0.8f; // High confidence for direct state monitoring
    
    return event;
}

void MissionMultirotorNode::publish_mission_event(const mission_search_interfaces::msg::MissionEvent& event)
{
    try {
        // Publish event
        mission_event_pub_->publish(event);
        
        // Enhanced logging with mission context
        std::string mission_context = event.active_mission_id.empty() ? 
            "" : " [Mission: " + event.active_mission_id + "]";
            
        RCLCPP_INFO(this->get_logger(), 
                   "Mission Event Published [%s]: %s %s - moved %.2fm%s", 
                   event.event_id.c_str(),
                   event.event_type.c_str(),
                   event.event_source.c_str(),
                   event.distance_moved,
                   mission_context.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish mission event: %s", e.what());
    }
}

