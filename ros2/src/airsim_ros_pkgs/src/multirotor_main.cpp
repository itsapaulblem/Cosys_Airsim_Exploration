#include "multirotor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Create a temporary node to get parameters from launch file
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_node");
    
    // Declare and get parameters passed from launch file (match mission version)
    temp_node->declare_parameter("vehicle_name", "Drone1");
    temp_node->declare_parameter("host_ip", "localhost");
    temp_node->declare_parameter("host_port", 41451);
    
    std::string vehicle_name = temp_node->get_parameter("vehicle_name").as_string();
    std::string host_ip = temp_node->get_parameter("host_ip").as_string();
    uint16_t host_port = static_cast<uint16_t>(temp_node->get_parameter("host_port").as_int());
    
    // Destroy temporary node
    temp_node.reset();
    
    try {
        auto node = std::make_shared<MultirotorNode>(vehicle_name);

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8); // 8 threads
        executor.add_node(node);
        executor.spin();

    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("multirotor_main"), 
                     "Failed to create multirotor node %s: %s", vehicle_name.c_str(), e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}