#pragma once

#include "vehicle_node_base.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/set_local_position.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <airsim_interfaces/msg/environment.hpp>

#include <mutex>
#include <vector>

using namespace msr::airlib;

class MultirotorNode : public VehicleNodeBase
{
public:
    MultirotorNode(const std::string& vehicle_name, 
                   const std::string& host_ip = "localhost", 
                   uint16_t host_port = 41451);
    virtual ~MultirotorNode() = default;

    struct VelCmd {
        float x = 0.0f;
        float y = 0.0f; 
        float z = 0.0f;
        msr::airlib::DrivetrainType drivetrain;
        msr::airlib::YawMode yaw_mode;
    };

protected:
    void initialize_vehicle_client() override;
    void setup_vehicle_publishers() override;
    void setup_vehicle_subscribers() override;
    void setup_vehicle_services() override;
    void update_vehicle_state() override;
    void publish_vehicle_state() override;
    void handle_vehicle_commands() override;
    void process_images() override;
    void process_lidar() override;
    void process_gpulidar() override;
    void process_echo() override;

private:
    void setup_drone_publishers();
    void setup_drone_subscribers();
    void setup_drone_services();
    void setup_sensor_publishers();

    void vel_cmd_body_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    void vel_cmd_world_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);

    bool takeoff_callback(
        const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);
    
    bool land_callback(
        const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Land::Response> response);

    VelCmd get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg);
    VelCmd get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                   const msr::airlib::Quaternionr& orientation);
    nav_msgs::msg::Odometry get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state);

    rclcpp::Time stamp_;
    nav_msgs::msg::Odometry curr_odom_;
    sensor_msgs::msg::NavSatFix gps_sensor_msg_;
    airsim_interfaces::msg::Environment env_msg_;
    msr::airlib::MultirotorState curr_drone_state_;
    
    std::mutex cmd_mutex_;
    bool has_vel_cmd_ = false;
    VelCmd current_vel_cmd_;
    
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_body_sub_;
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_world_sub_;
    
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_service_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr baro_pub_;

    // Messages
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;
    sensor_msgs::msg::Range baro_msg_;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs_;

    rclcpp::TimerBase::SharedPtr timer_;
};