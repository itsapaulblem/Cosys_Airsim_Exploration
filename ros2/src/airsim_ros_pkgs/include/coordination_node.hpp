#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <thread>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/list_scene_object_tags.hpp>
#include <airsim_interfaces/msg/gps_yaw.hpp>
#include <airsim_interfaces/msg/string_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class CoordinationNode : public rclcpp::Node
{
public:
    CoordinationNode();
    ~CoordinationNode() = default;

private:
    // Setup methods
    void connect_to_airsim();
    void setup_global_services();
    void setup_publishers();
    
    // Timer callback
    void coordination_timer_callback();
    void publish_system_status();
    
    // Service callbacks
    bool reset_all_callback(
        const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);
        
    bool takeoff_all_callback(
        const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);
        
    bool land_all_callback(
        const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Land::Response> response);
        
    bool pause_simulation_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        
    bool health_check_callback(
        const std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Request> request,
        std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Response> response);

private:
    // Connection parameters
    std::string host_ip_;
    uint16_t host_port_;
    std::string world_frame_id_;
    std::vector<std::string> vehicle_names_;
    
    // AirSim client
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    
    // Services
    rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_all_service_;
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_all_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_all_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::Service<airsim_interfaces::srv::ListSceneObjectTags>::SharedPtr health_check_service_;
    
    // Publishers
    rclcpp::Publisher<airsim_interfaces::msg::GPSYaw>::SharedPtr origin_geo_point_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::StringArray>::SharedPtr system_status_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr coordination_timer_;
    
    // Messages
    airsim_interfaces::msg::GPSYaw origin_geo_point_msg_;
};