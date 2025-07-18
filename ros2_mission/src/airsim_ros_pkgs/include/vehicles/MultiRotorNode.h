#pragma once

#include "vehicles/VehicleNodeBase.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/set_altitude.hpp>
#include <airsim_interfaces/srv/set_local_position.hpp>

/**
 * @brief ROS2 node for individual multirotor/drone vehicles
 * 
 * This class implements a standalone ROS2 node for multirotor vehicles (drones),
 * providing independent operation, fault isolation, and parallel processing.
 * Each drone runs as its own node with dedicated RPC connections to AirSim.
 */
class MultiRotorNode : public VehicleNodeBase
{
public:
    /**
     * @brief Constructor for MultiRotorNode
     * @param vehicle_name Unique name of the drone
     * @param vehicle_config Vehicle configuration from settings.json
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control for this drone
     */
    MultiRotorNode(const std::string& vehicle_name,
                   const VehicleSetting& vehicle_config,
                   const std::string& host_ip = "localhost",
                   uint16_t host_port = 41451,
                   const std::string& world_frame_id = "world",
                   bool enable_api_control = true);

    virtual ~MultiRotorNode();

protected:
    /**
     * @brief Virtual method implementations from VehicleNodeBase
     */
    
    /**
     * @brief Update drone state from AirSim
     * @return Current simulation timestamp
     */
    rclcpp::Time update_state() override;

    /**
     * @brief Update and execute drone-specific commands
     */
    void update_commands() override;

    /**
     * @brief Create MultirrotorRpcLibClient for drone communication
     */
    void create_airsim_client() override;

    /**
     * @brief Setup drone-specific publishers and subscribers
     */
    void setup_vehicle_ros_interface() override;

    /**
     * @brief Publish drone-specific state information
     */
    void publish_vehicle_state() override;

private:
    // Drone-specific state
    msr::airlib::MultirotorState curr_drone_state_;
    
    // Velocity command handling
    bool has_vel_cmd_;
    VelCmd vel_cmd_;
    double vel_cmd_duration_;
    
    // ROS subscribers for velocity commands
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_body_frame_sub_;
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_world_frame_sub_;
    
    // ROS services for drone control
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_srvr_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_srvr_;
    rclcpp::Service<airsim_interfaces::srv::SetAltitude>::SharedPtr set_altitude_srvr_;
    rclcpp::Service<airsim_interfaces::srv::SetLocalPosition>::SharedPtr set_local_position_srvr_;
    
    /**
     * @brief Velocity command callbacks
     */
    void vel_cmd_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    void vel_cmd_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    
    /**
     * @brief Service callbacks for drone control
     */
    bool takeoff_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
                       const std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);
    
    bool land_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
                     const std::shared_ptr<airsim_interfaces::srv::Land::Response> response);
    
    bool set_altitude_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Request> request,
                            const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Response> response);
    
    bool set_local_position_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Request> request,
                                  const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Response> response);
    
    /**
     * @brief Utility methods for drone-specific conversions
     */
    nav_msgs::msg::Odometry get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const;
    VelCmd get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg) const;
    VelCmd get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, const msr::airlib::Quaternionr& orientation) const;
    
    /**
     * @brief Get typed multirotor RPC client
     */
    msr::airlib::MultirotorRpcLibClient* get_multirotor_client();
    
    // Service timeout constants
    static constexpr double TAKEOFF_TIMEOUT_SEC = 20.0;
    static constexpr double LAND_TIMEOUT_SEC = 60.0;
    static constexpr double POSITION_TIMEOUT_SEC = 30.0;
    
    // Default velocity command duration
    static constexpr double DEFAULT_VEL_CMD_DURATION = 0.05;  // 50ms
};