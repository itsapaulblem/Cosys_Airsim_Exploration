#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <memory> 
#include <string>
#include <vector>

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "vehicles/computervision/api/ComputerVisionRpcLibClient.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <airsim_interfaces/srv/reset.hpp>
#include <airsim_interfaces/msg/environment.hpp>

using namespace msr::airlib;

class VehicleNodeBase : public rclcpp::Node
{
public:
    VehicleNodeBase(const std::string& vehicle_name, 
                    const std::string& host_ip = "localhost", 
                    uint16_t host_port = 41451);
    virtual ~VehicleNodeBase() = default;

    // Initialize the node
    void initialize_common();

protected:
    // Virtual methods to be implemented by derived classes
    virtual void initialize_vehicle_client();
    virtual void setup_vehicle_publishers();
    virtual void setup_vehicle_subscribers();
    virtual void setup_vehicle_services();
    virtual void update_vehicle_state();
    virtual void publish_vehicle_state();
    virtual void handle_vehicle_commands();
    virtual void process_images();
    virtual void process_lidar();
    virtual void process_gpulidar();
    virtual void process_echo();

    // Common functionality
    void setup_callback_groups();
    bool establish_connections();
    void setup_publishers();
    void setup_services();
    void setup_timers();

    // Timer callbacks
    void state_timer_callback();
    void image_timer_callback();
    void lidar_timer_callback();
    void gpulidar_timer_callback();
    void echo_timer_callback();

    // Utility methods
    void publish_odometry_tf(const nav_msgs::msg::Odometry& odom_msg);
    void publish_static_transforms();
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
    void handle_rpc_error(const rpc::rpc_error& e, const std::string& context);

    // Service callbacks
    bool reset_callback(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
                       std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);

    // Vehicle configuration
    std::string vehicle_name_;
    std::string host_ip_;
    uint16_t host_port_;
    std::string world_frame_id_;
    std::string odom_frame_id_;

    // Timer frequencies
    double state_timer_freq_;
    double image_timer_freq_;
    double lidar_timer_freq_;
    double gpulidar_timer_freq_;
    double echo_timer_freq_;

    // RPC clients
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_images_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_lidar_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_gpulidar_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_echo_;

    // Callback groups for parallel processing
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr gpulidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr echo_callback_group_;

    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr gpulidar_timer_;
    rclcpp::TimerBase::SharedPtr echo_timer_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::Environment>::SharedPtr env_pub_;

    // Services
    rclcpp::Service<airsim_interfaces::srv::Reset>::SharedPtr reset_service_;

    // TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
    std::vector<geometry_msgs::msg::TransformStamped> static_tf_msg_vec_;
};