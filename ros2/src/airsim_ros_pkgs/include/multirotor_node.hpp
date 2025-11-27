#pragma once

#include "vehicle_node_base.hpp"
#include <atomic>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/lidar/GPULidarSimpleParams.hpp"
#include "sensors/echo/EchoSimpleParams.hpp"

#include <airsim_interfaces/msg/environment.hpp>
// #include <airsim_interfaces/msg/target_detection.hpp>  // TODO: Create this message if needed
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/srv/gps_waypoint.hpp>
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>
#include <airsim_interfaces/srv/set_local_position.hpp>
#include <airsim_interfaces/srv/search_target.hpp>
#include <airsim_interfaces/srv/track_target.hpp>
#include <airsim_interfaces/srv/enable_api_control.hpp>
#include <airsim_interfaces/srv/takeoff_group.hpp>
#include <airsim_interfaces/srv/land_group.hpp>
#include <airsim_interfaces/srv/refresh_instance_segmentation.hpp>
#include <airsim_interfaces/srv/refresh_object_transforms.hpp>
#include <airsim_interfaces/msg/vel_cmd_group.hpp>
#include <airsim_interfaces/msg/instance_segmentation_list.hpp>
#include <airsim_interfaces/msg/object_transforms_list.hpp>
#include <airsim_interfaces/msg/string_array.hpp>
#include <airsim_interfaces/msg/altimeter.hpp>
#include <airsim_interfaces/msg/gimbal_angle_euler_cmd.hpp>
#include <airsim_interfaces/msg/gimbal_angle_quat_cmd.hpp>
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
#include <image_transport/image_transport.hpp>

#include <mutex>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

using namespace msr::airlib;

// Note: SensorPublisher and ImageRequest structs are now defined in vehicle_node_base.hpp

// Gimbal command structure
struct GimbalCmd
{
    std::string vehicle_name;
    std::string camera_name;
    msr::airlib::Quaternionr target_quat;
};

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
        float duration = 0.0f;
        msr::airlib::DrivetrainType drivetrain;
        msr::airlib::YawMode yaw_mode;
    };

public:
    void initialize_common();

protected:
    // Override VehicleNodeBase virtual methods for parallel processing
    virtual void initialize_vehicle_client() override;
    virtual void setup_vehicle_publishers() override;
    virtual void setup_vehicle_subscribers() override;
    virtual void setup_vehicle_services() override;
    virtual void update_vehicle_state() override;
    virtual void publish_vehicle_state() override;
    virtual void handle_vehicle_commands() override;
    
    // Parallel sensor processing overrides
    virtual void process_images() override;
    virtual void process_lidar() override;
    virtual void process_gpulidar() override;
    virtual void process_echo() override;

    void setup_sensor_publishers();

    // Virtual methods from base class (specific implementations)
    nav_msgs::msg::Odometry get_vehicle_odometry() override;
    msr::airlib::LidarData get_lidar_data_for_sensor(const std::string& sensor_name, const std::string& vehicle_name) override;

    // Vehicle-specific control methods
    void setup_vehicle_control_subscribers();
    void setup_vehicle_control_services();
    void process_vehicle_commands();

    // Service callbacks (protected so derived classes can access them)
    bool takeoff_callback(
        const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);

    bool land_callback(
        const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
        std::shared_ptr<airsim_interfaces::srv::Land::Response> response);

    bool gps_waypoint_callback(
        const std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Request> request,
        std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Response> response);

    bool enable_api_control_callback(
        const std::shared_ptr<airsim_interfaces::srv::EnableAPIControl::Request> request,
        std::shared_ptr<airsim_interfaces::srv::EnableAPIControl::Response> response);

    // Search and tracking services
    bool search_target_callback(
        const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> request,
        std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> response);
    bool track_target_callback(
        const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> request,
        std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> response);

    // Enhanced takeoff helper function
    bool position_to_target_altitude(msr::airlib::MultirotorRpcLibClient* client,
                                    float target_altitude,
                                    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response);

private:
    void setup_drone_publishers();
    void setup_drone_subscribers();
    void setup_drone_services();

    // Settings-driven configuration (from wrapper pattern)
    void create_publishers_from_settings();
    void discover_all_sensors();
    void setup_camera_publishers_from_settings(const msr::airlib::AirSimSettings::VehicleSetting& vehicle_setting,
                                               const std::string& topic_prefix);
    void setup_sensor_publishers_from_settings(const msr::airlib::AirSimSettings::VehicleSetting& vehicle_setting,
                                               const std::string& topic_prefix);
    void setup_instance_segmentation_publisher(const std::string& topic_prefix);
    void setup_object_transforms_publisher(const std::string& topic_prefix);

    // Note: Sensor discovery functions are implemented in base class
    void create_static_tf_for_lidar_sensors();
    void create_static_tf_for_all_sensors();
    void create_world_ned_transform();

    // Enhanced sensor processing methods with wrapper improvements
    void process_settings_driven_lidar();
    void process_legacy_lidar();
    sensor_msgs::msg::PointCloud2 create_lidar_msg_from_data(const msr::airlib::LidarData& lidar_data,
                                                             const std::string& sensor_name);
    void process_settings_driven_sensors();

    // Transform and pose utilities
    void append_static_sensor_tf(const std::string& sensor_name,
                                const msr::airlib::Pose& sensor_pose,
                                const std::string& frame_suffix = "_link");
    void append_static_lidar_tf(const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_params);
    void append_static_gpulidar_tf(const std::string& gpulidar_name, const msr::airlib::GPULidarSimpleParams& gpulidar_params);
    void append_static_echo_tf(const std::string& echo_name, const msr::airlib::EchoSimpleParams& echo_params);
    geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position,
                                                               const msr::airlib::Quaternionr& orientation);
    void convert_tf_msg_to_ros(geometry_msgs::msg::TransformStamped& tf_msg);

    // Camera info generation
    sensor_msgs::msg::CameraInfo generate_camera_info(const std::string& camera_name,
                                                      const msr::airlib::AirSimSettings::CameraSetting& camera_setting,
                                                      const msr::airlib::AirSimSettings::CaptureSetting& capture_setting) const;

    // Note: Sensor conversion functions are implemented in base class

    // Conditional service creation pattern for derived class customization
    virtual bool should_create_flight_services() const { return true; }

    // Subscriber callbacks
    void vel_cmd_body_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    void vel_cmd_world_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg);
    // TODO: Create TargetDetection.msg in airsim_interfaces if motion detection is needed
    // void motion_detection_callback(const airsim_interfaces::msg::TargetDetection::SharedPtr msg);

    // Sensor publishing methods
    void publish_imu_data();
    void publish_magnetometer_data();
    void publish_barometer_data();
    void publish_gps_data();
    void publish_environment_data();
    void publish_tf_data();
    void publish_system_status();
    void publish_static_transforms();

    // Utility methods
    std::pair<double, double> gps_to_ned(double lat, double lon, double home_lat, double home_lon);
    bool validate_gps_coordinates(double lat, double lon);

    VelCmd get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg);
    VelCmd get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                   const msr::airlib::Quaternionr& orientation);
    nav_msgs::msg::Odometry get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state);

    // Motion target structure (replaces old target detection)
    struct MotionTargetInfo {
        float x, y, z;
        float confidence;
        int track_id;
        std::chrono::steady_clock::time_point last_seen;

        MotionTargetInfo() : x(0), y(0), z(0), confidence(0), track_id(-1),
                            last_seen(std::chrono::steady_clock::now()) {}
    };

    MotionTargetInfo current_motion_target_;
    std::mutex motion_target_mutex_;

private:
    // Publishers (multirotor-specific)
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr baro_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::Environment>::SharedPtr env_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    // TODO: Create TargetDetection.msg in airsim_interfaces if motion detection is needed
    // rclcpp::Publisher<airsim_interfaces::msg::TargetDetection>::SharedPtr target_detection_pub_;

    // Spawn position offset storage (to maintain relative positions after takeoff)
    // THREAD-SAFETY: All variables are atomic to prevent torn reads/writes during concurrent access
    // Multiple threads (state timer, LiDAR timer, initialization) read these concurrently at 50Hz+
    // Without atomic protection, torn reads cause garbage values → SEGFAULT
    std::atomic<bool> spawn_offset_initialized_{false};
    std::atomic<double> spawn_offset_x_{0.0};
    std::atomic<double> spawn_offset_y_{0.0};
    std::atomic<double> spawn_offset_z_{0.0};

    // GLOBAL STATIC mutex for spawn offset initialization across ALL multirotor instances
    // Prevents simultaneous initialization by multiple drones that could corrupt member variables
    // CRITICAL: Each drone must calculate its own spawn offset atomically without interference
    static std::mutex spawn_offset_mutex_;

    // LiDAR timeout performance monitoring
    std::atomic<uint64_t> lidar_calls_successful_{0};  // Calls completed within timeout
    std::atomic<uint64_t> lidar_calls_timeout_{0};     // Calls that timed out
    std::atomic<uint64_t> lidar_calls_error_{0};       // Calls that failed with errors

    // Transform broadcasting
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    bool static_transforms_published_;

    // Separate velocity command tracking for body and world frames
    std::mutex vel_cmd_mutex_;
    mutable std::mutex publish_state_mutex_;  // CRITICAL: Protects concurrent publisher calls (ROS2 publishers NOT thread-safe)
    bool has_new_vel_cmd_body_frame_ = false;
    bool has_new_vel_cmd_world_frame_ = false;
    airsim_interfaces::msg::VelCmd vel_cmd_body_frame_;
    airsim_interfaces::msg::VelCmd vel_cmd_world_frame_;

    // Subscribers
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_body_frame_sub_;
    rclcpp::Subscription<airsim_interfaces::msg::VelCmd>::SharedPtr vel_cmd_world_frame_sub_;
    // TODO: Create TargetDetection.msg in airsim_interfaces if motion detection is needed
    // rclcpp::Subscription<airsim_interfaces::msg::TargetDetection>::SharedPtr motion_detection_sub_;

    // Services
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_srv_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_srv_;
    rclcpp::Service<airsim_interfaces::srv::GpsWaypoint>::SharedPtr gps_waypoint_srv_;
    rclcpp::Service<airsim_interfaces::srv::SearchTarget>::SharedPtr search_target_srv_;
    rclcpp::Service<airsim_interfaces::srv::TrackTarget>::SharedPtr track_target_srv_;
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_service_;
    rclcpp::Service<airsim_interfaces::srv::EnableAPIControl>::SharedPtr enable_api_control_service_;

    // Thread-safe state management for parallel processing
    std::mutex state_mutex_;
    rclcpp::Time stamp_;
    msr::airlib::MultirotorState vehicle_state_;

    // Velocity command state with thread safety
    airsim_interfaces::msg::VelCmd vel_cmd_body_frame_;
    airsim_interfaces::msg::VelCmd vel_cmd_world_frame_;
    bool has_new_vel_cmd_body_frame_ = false;
    bool has_new_vel_cmd_world_frame_ = false;
    std::mutex vel_cmd_mutex_;
};