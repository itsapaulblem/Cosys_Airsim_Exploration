#pragma once

#include "vehicles/VehicleNodeBase.h"
#include "vehicles/computervision/api/ComputerVisionRpcLibClient.hpp"
#include <airsim_interfaces/msg/computer_vision_state.hpp>

/**
 * @brief ROS2 node for camera-only computer vision mode
 * 
 * This class implements a standalone ROS2 node for computer vision mode,
 * which provides camera and sensor data without vehicle dynamics or control.
 * This mode is ideal for visual perception tasks, mapping, and camera-based research.
 */
class ComputerVisionNode : public VehicleNodeBase
{
public:
    /**
     * @brief Constructor for ComputerVisionNode
     * @param vehicle_name Unique name of the computer vision entity
     * @param vehicle_config Vehicle configuration from settings.json
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control (limited for CV mode)
     */
    ComputerVisionNode(const std::string& vehicle_name,
                       const VehicleSetting& vehicle_config,
                       const std::string& host_ip = "localhost",
                       uint16_t host_port = 41451,
                       const std::string& world_frame_id = "world",
                       bool enable_api_control = true);

    virtual ~ComputerVisionNode();

protected:
    /**
     * @brief Virtual method implementations from VehicleNodeBase
     */
    
    /**
     * @brief Update computer vision state from AirSim
     * @return Current simulation timestamp
     */
    rclcpp::Time update_state() override;

    /**
     * @brief Update and execute CV-specific commands (primarily gimbal control)
     */
    void update_commands() override;

    /**
     * @brief Create ComputerVisionRpcLibClient for CV communication
     */
    void create_airsim_client() override;

    /**
     * @brief Setup CV-specific publishers and subscribers
     */
    void setup_vehicle_ros_interface() override;

    /**
     * @brief Publish computer vision state information
     */
    void publish_vehicle_state() override;

private:
    // Computer vision-specific state
    msr::airlib::ComputerVisionApiBase::ComputerVisionState curr_computer_vision_state_;
    airsim_interfaces::msg::ComputerVisionState computer_vision_state_msg_;
    
    // ROS publisher for computer vision state
    rclcpp::Publisher<airsim_interfaces::msg::ComputerVisionState>::SharedPtr computer_vision_state_pub_;
    
    /**
     * @brief Utility methods for CV-specific conversions
     */
    nav_msgs::msg::Odometry get_odom_msg_from_computer_vision_state(
        const msr::airlib::ComputerVisionApiBase::ComputerVisionState& computer_vision_state) const;
    
    airsim_interfaces::msg::ComputerVisionState get_roscomputervisionstate_msg_from_computer_vision_state(
        const msr::airlib::ComputerVisionApiBase::ComputerVisionState& computer_vision_state) const;
    
    /**
     * @brief Get typed computer vision RPC client
     */
    msr::airlib::ComputerVisionRpcLibClient* get_computer_vision_client();
};