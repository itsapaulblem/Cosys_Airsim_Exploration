/**
 * @file mission_coordination_main.cpp
 * @brief Main executable for Mission Coordination Node
 * 
 * Provides centralized coordination for multi-vehicle missions using ultra-clean architecture.
 * Coordinates vehicles with naming pattern: /Droan1, /PX4_Drone2, etc.
 * 
 * Services provided:
 * - /mission_coordinator/plan_mission
 * - /mission_coordinator/assign_zones  
 * - /mission_coordinator/get_mission_status
 * 
 * Actions provided:
 * - /mission_coordinator/actions/execute_mission
 * 
 * Topics published:
 * - /mission_coordinator/mission_status
 * - /mission_coordinator/zone_assignments
 */

#include "mission_coordination_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

// Global node pointer for signal handling
std::shared_ptr<MissionCoordinationNode> g_coordination_node = nullptr;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rclcpp::get_logger("mission_coordination_main"), 
                    "Received SIGINT, shutting down mission coordination gracefully...");
        
        if (g_coordination_node) {
            rclcpp::shutdown();
        }
    }
}

int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Setup signal handling
    signal(SIGINT, signal_handler);
    
    try {
        // Create mission coordination node
        g_coordination_node = std::make_shared<MissionCoordinationNode>();
        
        RCLCPP_INFO(g_coordination_node->get_logger(), 
                    "🚁 Mission Coordination Node started");
        RCLCPP_INFO(g_coordination_node->get_logger(), 
                    "Ultra-Clean Architecture: Coordinating /Droan1, /PX4_Drone2, etc.");
        RCLCPP_INFO(g_coordination_node->get_logger(), 
                    "Services: /mission_coordinator/plan_mission, /mission_coordinator/assign_zones");
        RCLCPP_INFO(g_coordination_node->get_logger(), 
                    "Actions: /mission_coordinator/actions/execute_mission");
        
        // Use MultiThreadedExecutor for handling multiple callbacks
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(g_coordination_node);
        
        // Spin and handle callbacks
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mission_coordination_main"), 
                     "Exception in mission coordination: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("mission_coordination_main"), 
                "Mission coordination node shutdown complete");
    
    rclcpp::shutdown();
    return 0;
}