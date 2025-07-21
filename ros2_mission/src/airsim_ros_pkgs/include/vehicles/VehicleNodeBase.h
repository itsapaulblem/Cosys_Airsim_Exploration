#pragma once

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/lidar/GPULidarSimpleParams.hpp"
#include "sensors/echo/EchoSimpleParams.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "api/RpcLibClientBase.hpp"

#include <airsim_interfaces/msg/gimbal_angle_euler_cmd.hpp>
#include <airsim_interfaces/msg/gimbal_angle_quat_cmd.hpp>
#include <airsim_interfaces/msg/gps_yaw.hpp>
#include <airsim_interfaces/srv/refresh_instance_segmentation.hpp>
#include <airsim_interfaces/srv/refresh_object_transforms.hpp>
#include <airsim_interfaces/msg/instance_segmentation_label.hpp>
#include <airsim_interfaces/msg/instance_segmentation_list.hpp>
#include <airsim_interfaces/msg/object_transforms_list.hpp>
#include <airsim_interfaces/msg/string_array.hpp>
#include <airsim_interfaces/msg/environment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <airsim_interfaces/msg/altimeter.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

// Point cloud types from original wrapper
struct PointXYZRGBI
{
    PCL_ADD_POINT4D;
    float intensity;
    union
    {
        struct
        {
            uint8_t b;
            uint8_t g;
            uint8_t r;
        };
        float rgb;
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
)

// Utility structures from original wrapper
struct VelCmd
{
    double x;
    double y;
    double z;
    msr::airlib::DrivetrainType drivetrain;
    msr::airlib::YawMode yaw_mode;
    std::string vehicle_name;
};

struct GimbalCmd
{
    std::string vehicle_name;
    std::string camera_name;
    msr::airlib::Quaternionr target_quat;
};

template <typename T>
struct SensorPublisher
{
    msr::airlib::SensorBase::SensorType sensor_type;
    std::string sensor_name;
    typename rclcpp::Publisher<T>::SharedPtr publisher;
};

/**
 * @brief Base class for individual vehicle nodes in the multi-node AirSim ROS2 architecture
 * 
 * This class provides the foundation for vehicle-specific nodes, handling common functionality
 * like sensor publishing, transform management, and RPC communication with AirSim.
 * Each vehicle runs as an independent ROS2 node, providing fault isolation and parallel processing.
 */
class VehicleNodeBase : public rclcpp::Node
{
public:
    using AirSimSettings = msr::airlib::AirSimSettings;
    using SensorBase = msr::airlib::SensorBase;
    using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
    using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;
    using LidarSetting = msr::airlib::AirSimSettings::LidarSetting;
    using GPULidarSetting = msr::airlib::AirSimSettings::GPULidarSetting;
    using EchoSetting = msr::airlib::AirSimSettings::EchoSetting;
    using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
    using ImageRequest = msr::airlib::ImageCaptureBase::ImageRequest;
    using ImageResponse = msr::airlib::ImageCaptureBase::ImageResponse;
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;
    /**
     * @brief Constructor for VehicleNodeBase
     * @param vehicle_name Unique name of the vehicle
     * @param vehicle_config Vehicle configuration from settings.json
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control for this vehicle
     */
    VehicleNodeBase(const std::string& vehicle_name, 
                    const VehicleSetting& vehicle_config,
                    const std::string& host_ip = "localhost",
                    uint16_t host_port = 41451,
                    const std::string& world_frame_id = "world",
                    bool enable_api_control = true);

    virtual ~VehicleNodeBase();

    /**
     * @brief Initialize RPC connection to AirSim and setup vehicle
     */
    virtual void initialize();

protected:
    // Core vehicle properties
    std::string vehicle_name_;
    VehicleSetting vehicle_config_;
    std::string host_ip_;
    uint16_t host_port_;
    std::string world_frame_id_;
    std::string odom_frame_id_;
    bool enable_api_control_;

    // RPC clients for AirSim communication
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_images_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_lidar_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_gpulidar_;
    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_echo_;

    // Vehicle state and messaging
    nav_msgs::msg::Odometry curr_odom_;
    sensor_msgs::msg::NavSatFix gps_sensor_msg_;
    airsim_interfaces::msg::Environment env_msg_;
    rclcpp::Time stamp_;

    // ROS publishers for common vehicle data
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_local_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_gps_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::Environment>::SharedPtr env_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::InstanceSegmentationList>::SharedPtr instance_segmentation_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::ObjectTransformsList>::SharedPtr object_transforms_pub_;

    // Sensor publishers
    std::vector<SensorPublisher<airsim_interfaces::msg::Altimeter>> barometer_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::Imu>> imu_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::NavSatFix>> gps_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::MagneticField>> magnetometer_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::Range>> distance_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> lidar_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> lidar_labels_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> gpulidar_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> echo_active_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> echo_active_labels_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> echo_passive_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> echo_passive_labels_pubs_;

    // Transform management
    std::vector<geometry_msgs::msg::TransformStamped> static_tf_msg_vec_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

    // Services for instance segmentation and object transforms
    rclcpp::Service<airsim_interfaces::srv::RefreshInstanceSegmentation>::SharedPtr instance_segmentation_refresh_srvr_;
    rclcpp::Service<airsim_interfaces::srv::RefreshObjectTransforms>::SharedPtr object_transforms_refresh_srvr_;

    // Timers for different update rates
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr gpulidar_timer_;
    rclcpp::TimerBase::SharedPtr echo_timer_;

    // Callback groups for parallel processing
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;
    rclcpp::CallbackGroup::SharedPtr image_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr gpulidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr echo_callback_group_;

    // Control synchronization
    std::mutex control_mutex_;

    // Gimbal control
    bool has_gimbal_cmd_;
    GimbalCmd gimbal_cmd_;
    rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleQuatCmd>::SharedPtr gimbal_angle_quat_cmd_sub_;
    rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>::SharedPtr gimbal_angle_euler_cmd_sub_;

    // Image processing infrastructure
    typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
    std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
    std::vector<image_transport::Publisher> image_pub_vec_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_pub_vec_;
    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msg_vec_;

    /**
     * @brief Virtual methods to be implemented by derived vehicle types
     */
    
    /**
     * @brief Update vehicle-specific state from AirSim
     * @return Current simulation timestamp
     */
    virtual rclcpp::Time update_state() = 0;

    /**
     * @brief Update and execute vehicle-specific commands
     */
    virtual void update_commands() = 0;

    /**
     * @brief Create vehicle-specific RPC client
     */
    virtual void create_airsim_client() = 0;

    /**
     * @brief Setup vehicle-specific publishers and subscribers
     */
    virtual void setup_vehicle_ros_interface() = 0;

    /**
     * @brief Publish vehicle-specific state information
     */
    virtual void publish_vehicle_state() = 0;

    /**
     * @brief Timer callback methods
     */
    virtual void state_timer_cb();
    virtual void image_timer_cb();
    virtual void lidar_timer_cb();
    virtual void gpulidar_timer_cb();
    virtual void echo_timer_cb();

    /**
     * @brief Common initialization methods
     */
    void setup_common_publishers();
    void setup_common_services();
    void setup_sensor_publishers();
    void setup_timers();
    void setup_callback_groups();

    /**
     * @brief RPC client initialization
     */
    void initialize_rpc_clients();

    /**
     * @brief Transform and TF management
     */
    void publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg);
    void update_and_publish_static_transforms();
    void append_static_vehicle_tf(const VehicleSetting& vehicle_setting);
    void append_static_camera_tf(const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting);
    void append_static_gpulidar_tf(const std::string& gpulidar_name, const msr::airlib::GPULidarSimpleParams& gpulidar_setting);
    void append_static_echo_tf(const std::string& echo_name, const msr::airlib::EchoSimpleParams& echo_setting);

    /**
     * @brief Service callbacks
     */
    bool instance_segmentation_refresh_cb(const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Request> request,
                                         const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Response> response);
    bool object_transforms_refresh_cb(const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Request> request,
                                     const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Response> response);

    /**
     * @brief Gimbal control callbacks
     */
    void gimbal_angle_quat_cmd_cb(const airsim_interfaces::msg::GimbalAngleQuatCmd::SharedPtr msg);
    void gimbal_angle_euler_cmd_cb(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr msg);

    /**
     * @brief Image processing methods
     */
    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec);
    void setup_camera_publishers();
    sensor_msgs::msg::CameraInfo generate_cam_info(const std::string& camera_name, 
                                                   const CameraSetting& camera_setting, 
                                                   const CaptureSetting& capture_setting) const;
    std::shared_ptr<sensor_msgs::msg::Image> get_img_msg_from_response(const ImageResponse& img_response, 
                                                                       const rclcpp::Time curr_ros_time, 
                                                                       const std::string frame_id);
    std::shared_ptr<sensor_msgs::msg::Image> get_depth_img_msg_from_response(const ImageResponse& img_response, 
                                                                             const rclcpp::Time curr_ros_time, 
                                                                             const std::string frame_id);

    /**
     * @brief Utility methods for AirSim <-> ROS conversions
     */
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;
    nav_msgs::msg::Odometry get_odom_msg_from_kinematic_state(const msr::airlib::Kinematics::State& kinematics_estimated) const;
    airsim_interfaces::msg::GPSYaw get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const;
    airsim_interfaces::msg::Altimeter get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const;
    sensor_msgs::msg::Range get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const;
    airsim_interfaces::msg::InstanceSegmentationList get_instance_segmentation_list_msg_from_airsim() const;
    airsim_interfaces::msg::ObjectTransformsList get_object_transforms_list_msg_from_airsim(rclcpp::Time timestamp) const;
    sensor_msgs::msg::PointCloud2 get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const;
    airsim_interfaces::msg::StringArray get_lidar_labels_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const;
    sensor_msgs::msg::PointCloud2 get_gpulidar_msg_from_airsim(const msr::airlib::GPULidarData& gpulidar_data) const;
    sensor_msgs::msg::PointCloud2 get_active_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data) const;
    airsim_interfaces::msg::StringArray get_active_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data) const;
    sensor_msgs::msg::PointCloud2 get_passive_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data) const;
    airsim_interfaces::msg::StringArray get_passive_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data) const;
    sensor_msgs::msg::NavSatFix get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const;
    sensor_msgs::msg::MagneticField get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const;
    airsim_interfaces::msg::Environment get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const;
    geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation);
    geometry_msgs::msg::Transform get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion);

    /**
     * @brief Sensor publisher creation utility
     */
    template <typename T>
    const SensorPublisher<T> create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                     SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS);

    /**
     * @brief Pose utility methods
     */
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, GPULidarSetting& gpulidar_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, EchoSetting& echo_setting) const;
    msr::airlib::Pose get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::GeoPoint get_origin_geo_point() const;

private:
    // Timer update rates (can be made configurable via parameters)
    static constexpr double STATE_TIMER_RATE = 0.01;  // 100Hz
    static constexpr double IMAGE_TIMER_RATE = 0.05;  // 20Hz
    static constexpr double LIDAR_TIMER_RATE = 0.01;  // 100Hz
    static constexpr double GPULIDAR_TIMER_RATE = 0.01;  // 100Hz
    static constexpr double ECHO_TIMER_RATE = 0.01;  // 100Hz

    // Frame ID constants
    static constexpr char AIRSIM_FRAME_ID[] = "world";
    static constexpr char AIRSIM_ODOM_FRAME_ID[] = "odom_local";
};