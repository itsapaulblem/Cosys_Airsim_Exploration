#include <rclcpp/rclcpp.hpp>
#include "localization_node.hpp"

/**
 * @brief REP 105 Compliant Localization Node Main
 * 
 * This executable creates a standalone localization component that follows
 * REP 105 Frame Authorities specification. It should be launched alongside
 * vehicle nodes to provide proper map→odom transform authority.
 * 
 * Usage:
 *   ros2 run airsim_ros_pkgs localization_node --ros-args -p vehicle_name:=drone_1
 * 
 * REP 105 Compliance:
 * - Receives odom→base_link from odometry sources 
 * - Publishes dynamic map→odom (not static!)
 * - Uses AirSim ground truth as "perfect localization sensor"
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Get parameters from launch file
    auto temp_node = rclcpp::Node::make_shared("temp_param_node");
    temp_node->declare_parameter<std::string>("vehicle_name", "drone_1");
    temp_node->declare_parameter<std::string>("host_ip", "localhost");
    temp_node->declare_parameter<int>("host_port", 41451);
    
    std::string vehicle_name = temp_node->get_parameter("vehicle_name").as_string();
    std::string host_ip = temp_node->get_parameter("host_ip").as_string();
    uint16_t host_port = static_cast<uint16_t>(temp_node->get_parameter("host_port").as_int());
    temp_node.reset();
    
    // Create REP 105 compliant localization node with proper AirSim connection
    auto localization_node = std::make_shared<LocalizationNode>(vehicle_name, host_ip, host_port);
    
    RCLCPP_INFO(rclcpp::get_logger("localization_main"), 
        "Starting REP 105 Localization Node for vehicle: %s at %s:%d", 
        vehicle_name.c_str(), host_ip.c_str(), host_port);
    RCLCPP_INFO(rclcpp::get_logger("localization_main"), 
        "Frame Authorities: This node publishes map→%s/odom transform dynamically", vehicle_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("localization_main"), 
        "REP 105 Compliance: Receives %s/odom→%s/base_link from odometry source", 
        vehicle_name.c_str(), vehicle_name.c_str());
    
    try {
        rclcpp::spin(localization_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("localization_main"), 
            "Localization node failed: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("localization_main"), 
        "Localization node shutdown for vehicle: %s", vehicle_name.c_str());
    
    rclcpp::shutdown();
    return 0;
}