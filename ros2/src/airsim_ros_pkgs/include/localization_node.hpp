#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <atomic>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/StrictMode.hpp"

/**
 * @brief REP 105 Compliant Localization Component
 * 
 * This component follows REP 105 Frame Authorities specification:
 * - Receives odom→base_link transforms from odometry sources
 * - Computes map→base_link using AirSim ground truth "localization"
 * - Publishes DYNAMIC map→odom transform (not static!)
 * 
 * REP 105 Quote: "the localization component does not broadcast the transform 
 * from map to base_link. Instead, it first receives the transform from odom 
 * to base_link, and uses this information to broadcast the transform from 
 * map to odom."
 */
class LocalizationNode : public rclcpp::Node
{
public:
    explicit LocalizationNode(const std::string& vehicle_name, const std::string& host_ip = "localhost", uint16_t host_port = 41451);
    virtual ~LocalizationNode() = default;

private:
    // REP 105 Frame IDs
    std::string vehicle_name_;
    std::string map_frame_id_;       // "map"
    std::string odom_frame_id_;      // "vehicle_name/odom" 
    std::string base_link_frame_id_; // "vehicle_name/base_link" (matches vehicle_node_base.cpp)
    
    // AirSim connection parameters
    std::string host_ip_;
    uint16_t host_port_;

    // ROS2 Infrastructure
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // AirSim Client for ground truth localization
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    
    // Timer for publishing dynamic map→odom transform
    rclcpp::TimerBase::SharedPtr localization_timer_;
    
    // GPS-derived spawn offset for coordinate consistency with vehicle nodes
    // THREAD-SAFETY: All variables are atomic to prevent torn reads/writes during concurrent access
    // Localization timer at 50Hz reads these while initialization thread writes them
    // Without atomic protection, torn reads cause garbage values → SEGFAULT
    std::atomic<double> spawn_offset_x_{0.0};
    std::atomic<double> spawn_offset_y_{0.0};
    std::atomic<double> spawn_offset_z_{0.0};
    std::atomic<bool> spawn_offset_initialized_{false};
    
    /**
     * @brief REP 105 Compliant Localization Callback
     * 
     * Implements the proper frame authority pattern:
     * 1. Listen to odom→base_link from odometry source
     * 2. Get map→base_link from AirSim ground truth
     * 3. Compute and publish map→odom = map→base_link * (odom→base_link)^-1
     */
    void publish_localization_transform();
    
    /**
     * @brief Get ground truth pose from AirSim
     * @return Transform from map to base_link using AirSim ground truth
     */
    geometry_msgs::msg::TransformStamped get_map_to_base_link_ground_truth();
    
    /**
     * @brief Initialize AirSim client connection
     */
    void initialize_airsim_client();
    
    /**
     * @brief Initialize GPS-derived spawn offset for coordinate consistency with vehicle nodes
     */
    void initialize_spawn_offset();
    
    /**
     * @brief Convert AirSim pose to ROS transform
     */
    geometry_msgs::msg::TransformStamped convert_airsim_pose_to_transform(
        const msr::airlib::Pose& pose, const std::string& frame_id, const std::string& child_frame_id);
};