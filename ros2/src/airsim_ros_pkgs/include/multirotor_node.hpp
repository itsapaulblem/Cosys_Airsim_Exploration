#pragma once

#include "vehicle_node_base.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <airsim_interfaces/msg/environment.hpp>
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/srv/gps_waypoint.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/set_local_position.hpp>
#include <airsim_interfaces/srv/search_target.hpp>
#include <airsim_interfaces/srv/track_target.hpp>
#include <airsim_interfaces/msg/target_detection.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mutex>
#include <chrono>

using namespace msr::airlib;

class MultirotorNode : public VehicleNodeBase
{
public:
    MultirotorNode(const std::string& vehicle_name, 
                   const std::string& host_ip = "localhost", 
                   uint16_t host_port = 41451);
    virtual ~MultirotorNode() = default;

protected:
    virtual void initialize_vehicle_client() override;
    void setup_sensor_publishers();
    virtual void setup_vehicle_publishers() override;
    virtual void setup_vehicle_subscribers() override;
    virtual void setup_vehicle_services() override;
    virtual void process_images() override;
    virtual void process_lidar() override;
    virtual void publish_vehicle_state() override;

private:
    // Velocity command structure
    struct VelCmd {
        float x, y, z;
        msr::airlib::DrivetrainType drivetrain;
        msr::airlib::YawMode yaw_mode;
    };

    // Service callbacks
    bool takeoff_callback(
        const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);
        
    bool land_callback(
        const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Land::Response> response);
        
    bool set_local_position_callback(
        const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Request> request,
        std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Response> response);
        
    bool gps_waypoint_callback(
        const std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Request> request,
        std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Response> response);

    // Search and tracking services
    bool search_target_callback(
        const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> request,
        std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> response);
        
    bool track_target_callback(
        const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> request,
        std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> response);

    // Subscriber callbacks
    void vel_cmd_body_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    void vel_cmd_world_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    void twist_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // LiDAR processing for target detection
    void process_lidar_for_targets();
    bool detect_circular_spline(const std::vector<float>& point_cloud, 
                               float& center_x, float& center_y, float& radius, float& confidence);

    // Sensor publishing methods
    void publish_imu_data();
    void publish_magnetometer_data();
    void publish_barometer_data();

    // Utility methods
    void update_vehicle_state();
    void handle_vehicle_commands();
    std::pair<double, double> gps_to_ned(double lat, double lon, double home_lat, double home_lon);
    bool validate_gps_coordinates(double lat, double lon);
    VelCmd get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg);
    VelCmd get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                   const msr::airlib::Quaternionr& orientation);
    nav_msgs::msg::Odometry get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state);
    void process_gpulidar();
    void process_echo();
    void publish_gps_data();
    void publish_environment_data();
    void publish_tf_data();
    void publish_system_status();
    void publish_static_transforms();

private:
    // Publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr baro_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::Environment>::SharedPtr env_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    bool static_transforms_published_;

    // Target detection publisher
    rclcpp::Publisher<airsim_interfaces::msg::TargetDetection>::SharedPtr target_detection_pub_;

    // Subscribers
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_body_frame_sub_;
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_world_frame_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;

    // Services
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_service_;
    rclcpp::Service<airsim_interfaces::srv::SetLocalPosition>::SharedPtr set_local_position_service_;
    rclcpp::Service<airsim_interfaces::srv::GpsWaypoint>::SharedPtr gps_waypoint_service_;
    
    // Search and tracking services
    rclcpp::Service<airsim_interfaces::srv::SearchTarget>::SharedPtr search_target_service_;
    rclcpp::Service<airsim_interfaces::srv::TrackTarget>::SharedPtr track_target_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    rclcpp::Time stamp_;
    msr::airlib::MultirotorState curr_drone_state_;
    nav_msgs::msg::Odometry curr_odom_;
    
    // Target tracking state
    struct DetectedTarget {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float radius = 0.0f;
        float confidence = 0.0f;
        std::chrono::steady_clock::time_point last_seen;
    };
    DetectedTarget current_target_;
    std::mutex target_mutex_;
};