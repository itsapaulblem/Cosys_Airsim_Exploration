#include "multirotor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::string vehicle_name = "Drone1"; // Default
    
    // Get vehicle name from command line argument
    if (argc > 1) {
        vehicle_name = std::string(argv[1]);
    }
    
    try {
        auto node = std::make_shared<MultirotorNode>(vehicle_name);
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("multirotor_main"), 
                     "Failed to create multirotor node %s: %s", vehicle_name.c_str(), e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}