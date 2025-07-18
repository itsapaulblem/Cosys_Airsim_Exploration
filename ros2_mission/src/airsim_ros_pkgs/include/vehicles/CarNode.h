#pragma once

#include "vehicles/VehicleNodeBase.h"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include <airsim_interfaces/msg/car_controls.hpp>
#include <airsim_interfaces/msg/car_state.hpp>

/**
 * @brief ROS2 node for individual car/ground vehicles
 * 
 * This class implements a standalone ROS2 node for ground vehicles (cars),
 * providing independent operation, fault isolation, and parallel processing.
 * Each car runs as its own node with dedicated RPC connections to AirSim.
 */
class CarNode : public VehicleNodeBase
{
public:
    /**
     * @brief Constructor for CarNode
     * @param vehicle_name Unique name of the car
     * @param vehicle_config Vehicle configuration from settings.json
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control for this car
     */
    CarNode(const std::string& vehicle_name,
            const VehicleSetting& vehicle_config,
            const std::string& host_ip = "localhost",
            uint16_t host_port = 41451,
            const std::string& world_frame_id = "world",
            bool enable_api_control = true);

    virtual ~CarNode();

protected:
    /**
     * @brief Virtual method implementations from VehicleNodeBase
     */
    
    /**
     * @brief Update car state from AirSim
     * @return Current simulation timestamp
     */
    rclcpp::Time update_state() override;

    /**
     * @brief Update and execute car-specific commands
     */
    void update_commands() override;

    /**
     * @brief Create CarRpcLibClient for car communication
     */
    void create_airsim_client() override;

    /**
     * @brief Setup car-specific publishers and subscribers
     */
    void setup_vehicle_ros_interface() override;

    /**
     * @brief Publish car-specific state information
     */
    void publish_vehicle_state() override;

private:
    // Car-specific state
    msr::airlib::CarApiBase::CarState curr_car_state_;
    airsim_interfaces::msg::CarState car_state_msg_;
    
    // Car command handling
    bool has_car_cmd_;
    msr::airlib::CarApiBase::CarControls car_cmd_;
    
    // ROS subscriber for car controls
    rclcpp::Subscription<airsim_interfaces::msg::CarControls>::SharedPtr car_cmd_sub_;
    
    // ROS publisher for car state
    rclcpp::Publisher<airsim_interfaces::msg::CarState>::SharedPtr car_state_pub_;
    
    /**
     * @brief Car control command callback
     */
    void car_cmd_cb(const airsim_interfaces::msg::CarControls::SharedPtr msg);
    
    /**
     * @brief Utility methods for car-specific conversions
     */
    nav_msgs::msg::Odometry get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const;
    airsim_interfaces::msg::CarState get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const;
    
    /**
     * @brief Get typed car RPC client
     */
    msr::airlib::CarRpcLibClient* get_car_client();
};