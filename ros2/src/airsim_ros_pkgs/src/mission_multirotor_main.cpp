#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "mission_multirotor_node.hpp"

/**
 * @brief Main executable for Mission Multirotor Node
 * 
 * This creates a mission-capable multirotor node that extends the base functionality
 * with search and rescue mission capabilities including SearchArea, NavigateToTarget, 
 * and TrackTarget actions following ultra-clean naming conventions.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Create a temporary node to get parameters from launch file
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_node");
    
    // Declare and get parameters passed from launch file
    temp_node->declare_parameter("vehicle_name", "Drone1");
    temp_node->declare_parameter("host_ip", "localhost");
    temp_node->declare_parameter("host_port", 41451);
    
    std::string vehicle_name = temp_node->get_parameter("vehicle_name").as_string();
    std::string host_ip = temp_node->get_parameter("host_ip").as_string();
    uint16_t host_port = static_cast<uint16_t>(temp_node->get_parameter("host_port").as_int());
    
    // Destroy temporary node
    temp_node.reset();
    
    try {
        // Create mission-capable multirotor node with parameters from launch file
        auto node = std::make_shared<MissionMultirotorNode>(vehicle_name, host_ip, host_port);
        
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "Starting Mission Multirotor Node for vehicle: %s", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "Mission capabilities available:");
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "  - Action servers: /%s/actions/search_area", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "                   /%s/actions/navigate_to_target", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "                   /%s/actions/track_target", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "  - Services:      /%s/services/set_search_pattern", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "                   /%s/services/get_capabilities", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "  - Topics:        /%s/mission/status", vehicle_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mission_multirotor_main"), 
                    "                   /%s/detections/target", vehicle_name.c_str());
        
        // Initialize the node
        node->initialize_common();
        
        // Use multi-threaded executor for concurrent mission and sensor processing
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mission_multirotor_main"), 
                     "Failed to create mission multirotor node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}