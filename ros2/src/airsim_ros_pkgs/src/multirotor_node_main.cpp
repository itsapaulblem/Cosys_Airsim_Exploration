/**
 * Example main function for MultirotorNode with MultiThreadedExecutor
 * 
 * This demonstrates how to run the MultirotorNode with proper threading
 * to enable concurrent execution of different callback groups, which is
 * essential for the camera frame rate optimization.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "multirotor_node.hpp"

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create MultiThreadedExecutor instead of SingleThreadedExecutor
    // This allows different callback groups to run concurrently
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 
        4  // Number of threads - adjust based on your system
    );
    
    // Create the multirotor node
    // Adjust these parameters based on your AirSim setup
    std::string vehicle_name = "Drone1";
    std::string host_ip = "127.0.0.1";
    uint16_t host_port = 41451;
    
    auto node = std::make_shared<MultirotorNode>(vehicle_name, host_ip, host_port);
    
    // Add node to executor
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting MultirotorNode with MultiThreadedExecutor...");
    
    // Spin the executor - this will run until shutdown
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Executor error: %s", e.what());
    }
    
    // Cleanup
    rclcpp::shutdown();
    
    return 0;
}