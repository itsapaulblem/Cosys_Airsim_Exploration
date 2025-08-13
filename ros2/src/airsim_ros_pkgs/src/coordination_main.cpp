#include "coordination_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CoordinationNode>();

    RCLCPP_INFO(node->get_logger(), "Starting AirSim coordination node");

    try {
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in coordination node: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}