#include "coordination_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <future>

CoordinationNode::CoordinationNode()
    : Node("airsim_coordination")
    , host_ip_("localhost")
    , host_port_(41451)
    , world_frame_id_("world_ned")
{
    // Declare parameters with defaults
    this->declare_parameter("host_ip", host_ip_);
    this->declare_parameter("host_port", static_cast<int>(host_port_));
    this->declare_parameter("world_frame_id", world_frame_id_);
    this->declare_parameter("vehicle_names", std::vector<std::string>());
    
    // Get parameters
    this->get_parameter("host_ip", host_ip_);
    int port_param = static_cast<int>(host_port_);  // Initialize with default
    this->get_parameter("host_port", port_param);
    host_port_ = static_cast<uint16_t>(port_param);
    this->get_parameter("world_frame_id", world_frame_id_);
    this->get_parameter("vehicle_names", vehicle_names_);
    
    RCLCPP_INFO(this->get_logger(), "Initializing coordination node...");
    RCLCPP_INFO(this->get_logger(), "Target: %s:%d", host_ip_.c_str(), host_port_);
    RCLCPP_INFO(this->get_logger(), "Managing %zu vehicles", vehicle_names_.size());
    
    // Initialize services and publishers first
    setup_global_services();
    setup_publishers();
    
    connect_to_airsim();
    
    // Start coordination timer (slower rate for coordination tasks)
    coordination_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),  // 2Hz for coordination
        std::bind(&CoordinationNode::coordination_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Coordination node ready");
}

void CoordinationNode::connect_to_airsim()
{
    try {
        airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        airsim_client_->confirmConnection();
        RCLCPP_INFO(this->get_logger(), "Connected to AirSim at %s:%d", 
                   host_ip_.c_str(), host_port_);
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not connect to AirSim: %s. Will retry in timer.", e.what());
        airsim_client_.reset(); // Clear failed connection
    }
}

void CoordinationNode::setup_global_services()
{
    // Global reset service
    reset_all_service_ = this->create_service<airsim_interfaces::srv::Reset>(
        "reset_all",
        std::bind(&CoordinationNode::reset_all_callback, this, 
                 std::placeholders::_1, std::placeholders::_2));
    
    // Global takeoff service
    takeoff_all_service_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        "takeoff_all",
        std::bind(&CoordinationNode::takeoff_all_callback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Global land service
    land_all_service_ = this->create_service<airsim_interfaces::srv::Land>(
        "land_all",
        std::bind(&CoordinationNode::land_all_callback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Pause/unpause simulation
    pause_service_ = this->create_service<std_srvs::srv::SetBool>(
        "pause_simulation",
        std::bind(&CoordinationNode::pause_simulation_callback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Vehicle health monitoring
    health_check_service_ = this->create_service<airsim_interfaces::srv::ListSceneObjectTags>(
        "health_check",
        std::bind(&CoordinationNode::health_check_callback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Global services initialized");
}

void CoordinationNode::setup_publishers()
{
    // GPS origin publisher (shared across all vehicles)
    origin_geo_point_pub_ = this->create_publisher<airsim_interfaces::msg::GPSYaw>(
        "origin_geo_point", rclcpp::QoS(10).reliable());
    
    // System status publisher
    system_status_pub_ = this->create_publisher<airsim_interfaces::msg::StringArray>(
        "system_status", rclcpp::QoS(10).reliable());
    
    // Clock publisher for simulation time
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
        "clock", rclcpp::QoS(10).reliable());
    
    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
}

void CoordinationNode::coordination_timer_callback()
{
    // Single timer callback implementation
    if (!airsim_client_) {
        // Try to reconnect periodically (every 4 cycles = 2 seconds)
        static int retry_count = 0;
        if (++retry_count % 4 == 0) {
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to AirSim...");
            connect_to_airsim();
        }
        return;
    }
    
    // Publish GPS origin
    try {
        if (origin_geo_point_msg_.latitude == 0.0 && origin_geo_point_msg_.longitude == 0.0) {
            auto origin = airsim_client_->getHomeGeoPoint("");
            origin_geo_point_msg_.latitude = origin.latitude;
            origin_geo_point_msg_.longitude = origin.longitude;
            origin_geo_point_msg_.altitude = origin.altitude;
            origin_geo_point_msg_.yaw = 0.0;
            RCLCPP_INFO(this->get_logger(), "GPS origin set: %.6f, %.6f", 
                       origin.latitude, origin.longitude);
        }
        origin_geo_point_pub_->publish(origin_geo_point_msg_);
    }
    catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to get GPS origin: %s", e.what());
    }
    
    // Publish simulation clock
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = this->get_clock()->now();
    clock_pub_->publish(clock_msg);
    
    // Monitor vehicle health
    publish_system_status();
}

void CoordinationNode::publish_system_status()
{
    if (!airsim_client_) {
        return;
    }
    
    airsim_interfaces::msg::StringArray status_msg;
    status_msg.header.stamp = this->get_clock()->now();
    status_msg.header.frame_id = world_frame_id_;
    
    for (const auto& vehicle_name : vehicle_names_) {
        try {
            auto state = airsim_client_->getMultirotorState(vehicle_name);
            status_msg.data.push_back(vehicle_name + ": READY");
        }
        catch (const std::exception& e) {
            status_msg.data.push_back(vehicle_name + ": ERROR - " + std::string(e.what()));
        }
    }
    
    system_status_pub_->publish(status_msg);
}

bool CoordinationNode::reset_all_callback(
    const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
{
    (void)request; // Suppress unused parameter warning
    
    if (!airsim_client_) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "No AirSim connection for reset");
        return false;
    }
    
    try {
        RCLCPP_INFO(this->get_logger(), "Resetting all vehicles...");
        airsim_client_->reset();
        
        // Wait for reset to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Reset completed successfully");
        return true;
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Reset failed: %s", e.what());
        return false;
    }
}

bool CoordinationNode::takeoff_all_callback(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    if (!airsim_client_) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "No AirSim connection for takeoff");
        return false;
    }
    
    try {
        RCLCPP_INFO(this->get_logger(), "Taking off all vehicles...");
        
        // Enable API control and arm all vehicles first
        for (const auto& vehicle_name : vehicle_names_) {
            try {
                airsim_client_->enableApiControl(true, vehicle_name);
                airsim_client_->armDisarm(true, vehicle_name);
                RCLCPP_INFO(this->get_logger(), "Armed vehicle: %s", vehicle_name.c_str());
            }
            catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to arm %s: %s", vehicle_name.c_str(), e.what());
            }
        }
        
        // Start takeoff for all vehicles
        std::vector<std::future<bool>> futures;
        for (const auto& vehicle_name : vehicle_names_) {
            try {
                auto future = airsim_client_->takeoffAsync(20.0f, vehicle_name);
                if (request->wait_on_last_task) {
                    futures.push_back(std::async(std::launch::async, [future]() {
                        future->waitOnLastTask();
                        return true;
                    }));
                }
                RCLCPP_INFO(this->get_logger(), "Takeoff initiated for: %s", vehicle_name.c_str());
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to takeoff %s: %s", vehicle_name.c_str(), e.what());
            }
        }
        
        // Wait for completion if requested
        if (request->wait_on_last_task) {
            for (auto& future : futures) {
                future.wait();
            }
        }
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Takeoff completed for all vehicles");
        return true;
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %s", e.what());
        return false;
    }
}

bool CoordinationNode::land_all_callback(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    if (!airsim_client_) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "No AirSim connection for landing");
        return false;
    }
    
    try {
        RCLCPP_INFO(this->get_logger(), "Landing all vehicles...");
        
        std::vector<std::future<bool>> futures;
        for (const auto& vehicle_name : vehicle_names_) {
            try {
                auto future = airsim_client_->landAsync(60.0f, vehicle_name);
                if (request->wait_on_last_task) {
                    futures.push_back(std::async(std::launch::async, [future]() {
                        future->waitOnLastTask();
                        return true;
                    }));
                }
                RCLCPP_INFO(this->get_logger(), "Landing initiated for: %s", vehicle_name.c_str());
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to land %s: %s", vehicle_name.c_str(), e.what());
            }
        }
        
        // Wait for completion if requested
        if (request->wait_on_last_task) {
            for (auto& future : futures) {
                future.wait();
            }
        }
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Landing completed for all vehicles");
        return true;
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Landing failed: %s", e.what());
        return false;
    }
}

bool CoordinationNode::pause_simulation_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!airsim_client_) {
        response->success = false;
        response->message = "No AirSim connection";
        return false;
    }
    
    try {
        airsim_client_->simPause(request->data);
        response->success = true;
        response->message = request->data ? "Simulation paused" : "Simulation resumed";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        return true;
    }
    catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to change simulation state: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return false;
    }
}

bool CoordinationNode::health_check_callback(
    const std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Request> request,
    std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Response> response)
{
    (void)request; // Suppress unused parameter warning
    
    if (!airsim_client_) {
        RCLCPP_WARN(this->get_logger(), "No AirSim connection for health check");
        return true; // Return empty response if no connection
    }
    
    for (const auto& vehicle_name : vehicle_names_) {
        try {
            auto state = airsim_client_->getMultirotorState(vehicle_name);
            response->tags.push_back(vehicle_name + "_HEALTHY");
        }
        catch (const std::exception& e) {
            response->tags.push_back(vehicle_name + "_ERROR");
            RCLCPP_WARN(this->get_logger(), "Health check failed for %s: %s", 
                       vehicle_name.c_str(), e.what());
        }
    }
    return true;
}