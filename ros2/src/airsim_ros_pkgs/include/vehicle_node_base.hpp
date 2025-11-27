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
#include <airsim_interfaces/msg/altimeter.hpp>
#include <airsim_interfaces/msg/string_array.hpp>
#include <airsim_interfaces/msg/instance_segmentation_list.hpp>
#include <airsim_interfaces/msg/object_transforms_list.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <image_transport/image_transport.hpp>
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/lidar/GPULidarSimpleParams.hpp"
#include "sensors/echo/EchoSimpleParams.hpp"

#include <unordered_map>
#include <vector>
#include <string>

using namespace msr::airlib;

// Forward declarations and type aliases (moved from AirsimROSWrapper)
using AirSimSettings = msr::airlib::AirSimSettings;
using SensorBase = msr::airlib::SensorBase;
using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;
using LidarSetting = msr::airlib::AirSimSettings::LidarSetting;
using GPULidarSetting = msr::airlib::AirSimSettings::GPULidarSetting;
using EchoSetting = msr::airlib::AirSimSettings::EchoSetting;
using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;

// Templated sensor publisher pattern
template <typename T>
struct SensorPublisher
{
    msr::airlib::SensorBase::SensorType sensor_type;
    std::string sensor_name;
    typename rclcpp::Publisher<T>::SharedPtr publisher;
};

// Image request structure for dynamic camera handling
struct ImageRequest
{
    std::string camera_name;
    msr::airlib::ImageCaptureBase::ImageType image_type;
    bool pixels_as_float;
    bool compress;
    std::string annotation_name;
    
    ImageRequest(const std::string& cam_name, 
                 msr::airlib::ImageCaptureBase::ImageType img_type, 
                 bool as_float = false, 
                 bool compressed = false, 
                 const std::string& annotation = "")
        : camera_name(cam_name), image_type(img_type), pixels_as_float(as_float), 
          compress(compressed), annotation_name(annotation) {}
};

class VehicleNodeBase : public rclcpp::Node
{
public:
    // AIRSIM_MODE enum (moved from AirsimROSWrapper)
    enum class AIRSIM_MODE : unsigned
    {
        DRONE,
        CAR,
        COMPUTERVISION
    };

    // Static constants (moved from AirsimROSWrapper)
    static constexpr char CAM_YML_NAME[] = "camera_name";
    static constexpr char WIDTH_YML_NAME[] = "image_width";
    static constexpr char HEIGHT_YML_NAME[] = "image_height";
    static constexpr char K_YML_NAME[] = "camera_matrix";
    static constexpr char D_YML_NAME[] = "distortion_coefficients";
    static constexpr char R_YML_NAME[] = "rectification_matrix";
    static constexpr char P_YML_NAME[] = "projection_matrix";
    static constexpr char DMODEL_YML_NAME[] = "distortion_model";

    VehicleNodeBase(const std::string& vehicle_name, 
                    const std::string& host_ip = "localhost", 
                    uint16_t host_port = 41451,
                    const std::shared_ptr<rclcpp::CallbackGroup> callbackGroup = nullptr
                );
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
    virtual void process_state_changes();  // NEW: Virtual hook for state change processing (mission events, etc.)
    virtual void publish_vehicle_state();
    virtual void process_vehicle_commands();
    virtual void process_images();       // LEGACY - replaced by individual camera processing

    // Individual camera processing functions for parallel execution
    virtual void process_camera_1();    // Process camera 1 individually
    virtual void process_camera_2();    // Process camera 2 individually
    virtual void process_camera_3();    // Process camera 3 individually
    virtual void process_camera_4();    // Process camera 4 individually

    // Helper function for processing specific camera indices
    void process_specific_cameras(const std::vector<size_t>& camera_indices, const std::string& timer_name);
    virtual void process_lidar();
    virtual void process_gpulidar();
    virtual void process_echo();

    // Common sensor processing methods 
    void process_settings_driven_lidar();
    void process_legacy_lidar(); 
    void publish_sensor_data();
    
    // Settings-driven configuration 
    void create_publishers_from_settings();
    void setup_camera_publishers_from_settings(const VehicleSetting& vehicle_setting,
                                               const std::string& topic_prefix);
    void setup_sensor_publishers_from_settings(const VehicleSetting& vehicle_setting,
                                               const std::string& topic_prefix);
    void setup_default_sensor_publishers(const std::string& topic_prefix);
    void setup_default_camera_publishers(const std::string& topic_prefix);
    void setup_instance_segmentation_publisher(const std::string& topic_prefix);
    void setup_object_transforms_publisher(const std::string& topic_prefix);
    
    // Templated sensor publisher creation
    template <typename T>
    const SensorPublisher<T> create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                     SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS)
    {
        if(!sensor_type_name.empty())
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing " << sensor_type_name << " '" << sensor_name << "'");

        SensorPublisher<T> sensor_publisher;
        sensor_publisher.sensor_name = sensor_name;
        sensor_publisher.sensor_type = sensor_type;
        sensor_publisher.publisher = this->create_publisher<T>(topic_name, QoS);
        return sensor_publisher;
    }
    
    // Dynamic sensor discovery 
    void discover_lidar_sensors();
    void discover_gpulidar_sensors();
    void discover_echo_sensors();
    void discover_camera_sensors();
    
    // Transform and TF management 
    // void create_static_tf_for_lidar_sensors();
    void create_odom_local_ned_transform();
    // void create_fallback_map_transform();  // Fallback map → world_ned transform
    void create_vehicle_map_to_odom_transform();  // REP 105: map → {vehicle}/odom transform
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
                                                      const CameraSetting& camera_setting,
                                                      const CaptureSetting& capture_setting) const;

    // Missing method declarations (moved from AirsimROSWrapper)
    void create_ros_pubs_from_settings_json();
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
    void append_static_camera_tf(const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_camera_tf_basic(const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf_basic(const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_params);
    geometry_msgs::msg::Transform get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const;
    
    // Sensor data conversion utilities 
    sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const;
    
    sensor_msgs::msg::MagneticField get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const;
    airsim_interfaces::msg::Altimeter get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& baro_data) const;
    sensor_msgs::msg::Range get_range_msg_from_airsim(const msr::airlib::DistanceSensorData& distance_data) const;

    // Virtual methods for vehicle-specific operations
    virtual nav_msgs::msg::Odometry get_vehicle_odometry() = 0;
    virtual void setup_vehicle_control_subscribers() {}
    virtual void setup_vehicle_control_services() {}
    
    // Virtual method for LiDAR data access (derived class provides client access)
    virtual msr::airlib::LidarData get_lidar_data_for_sensor(const std::string& sensor_name, const std::string& vehicle_name) = 0;
    
    // Client type enumeration for dual-client architecture
    enum class ClientType {
        SENSOR,  // High-bandwidth: LiDAR, images
        STATE    // Low-latency: IMU, GPS, barometer
    };

    // Thread-safe client access methods for derived classes
    template<typename RpcClientType>
    RpcClientType* get_sensor_client() const {
        std::lock_guard<std::mutex> lock(sensor_client_mutex_);
        rpc_calls_total_.fetch_add(1);
        return static_cast<RpcClientType*>(sensor_client_.get());
    }
    
    template<typename RpcClientType>
    RpcClientType* get_state_client() const {
        std::lock_guard<std::mutex> lock(state_client_mutex_);
        rpc_calls_total_.fetch_add(1);
        return static_cast<RpcClientType*>(state_client_.get());
    }
    
    // Thread-safe RPC call wrapper with error tracking for sensor client
    template<typename Func>
    auto safe_sensor_rpc_call(Func&& func) const -> decltype(func()) {
        std::lock_guard<std::mutex> lock(sensor_client_mutex_);
        rpc_calls_total_.fetch_add(1);
        try {
            auto result = func();
            rpc_calls_successful_.fetch_add(1);
            return result;
        } catch (const std::exception& e) {
            rpc_calls_failed_.fetch_add(1);
            throw;
        }
    }
    
    // Thread-safe RPC call wrapper with error tracking for state client
    template<typename Func>
    auto safe_state_rpc_call(Func&& func) const -> decltype(func()) {
        std::lock_guard<std::mutex> lock(state_client_mutex_);
        rpc_calls_total_.fetch_add(1);
        try {
            auto result = func();
            rpc_calls_successful_.fetch_add(1);
            return result;
        } catch (const std::exception& e) {
            rpc_calls_failed_.fetch_add(1);
            throw;
        }
    }

protected:
    // GLOBAL STATIC mutex for ALL AirSim RPC calls - shared across ALL vehicle instances and ALL sensors
    // Prevents concurrent RPC requests (GPS, LiDAR, cameras, IMU, etc.) to AirSim server which causes SEGFAULT
    // CRITICAL: AirSim RPC library cannot handle concurrent requests - ALL sensor data fetching must be serialized
    static std::mutex airsim_rpc_mutex_;

public:
    // Common functionality
    bool establish_connections();
    void setup_publishers();
    void setup_services();
    void setup_timers();
    void setup_individual_camera_timers();  // Setup individual camera timers for parallel processing

    // Timer callbacks
    void state_timer_callback();
    void image_timer_callback();      // LEGACY - replaced by individual camera callbacks
    void lidar_timer_callback();
    void gpulidar_timer_callback();
    void echo_timer_callback();

    // Individual camera callback functions for parallel processing
    void camera_1_timer_callback();   // Camera 1 individual processing
    void camera_2_timer_callback();   // Camera 2 individual processing
    void camera_3_timer_callback();   // Camera 3 individual processing
    void camera_4_timer_callback();   // Camera 4 individual processing

    // Settings and configuration utilities
    std::string findVehicleInSettings(const msr::airlib::Settings& settings_json, const std::string& rpc_vehicle_name);

    // Utility methods
    void publish_odometry_tf(const nav_msgs::msg::Odometry& odom_msg);
    void publish_static_transforms();
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const;
    void handle_rpc_error(const rpc::rpc_error& e, const std::string& context);
    
    // Point cloud coordinate transformation utility
    void fixPointCloud(std::vector<float>& data, int offset, std::vector<int> flip_indexes);

    // Service callbacks
    bool reset_callback(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
                       std::shared_ptr<airsim_interfaces::srv::Reset::Response> response);

    // Vehicle configuration
    std::string vehicle_name_;
    std::string host_ip_;
    uint16_t host_port_;
    
    // REP 105 compliant frame IDs with namespace support
    std::string robot_namespace_;    // e.g., "robot1", "robot2" (for multi-robot)
    std::string map_frame_id_;       // "map" (global stationary frame)
    std::string odom_frame_id_;      // "{namespace}/odom" (local continuous frame)  
    std::string base_link_frame_id_; // "{namespace}/base_link" (robot body frame)

    // Timer frequencies
    double state_timer_freq_;
    double image_timer_freq_;
    double lidar_timer_freq_;
    double gpulidar_timer_freq_;
    double echo_timer_freq_;

    // RPC timeout configuration
    int sensor_rpc_timeout_ms_;  // Timeout for sensor RPC calls (LiDAR, images) in milliseconds
    int state_rpc_timeout_ms_;   // Timeout for state RPC calls (IMU, GPS, baro) in milliseconds

    // Dual specialized RPC clients for performance optimization
    std::unique_ptr<msr::airlib::RpcLibClientBase> sensor_client_;  // High-bandwidth: LiDAR, images
    std::unique_ptr<msr::airlib::RpcLibClientBase> state_client_;   // Low-latency: IMU, GPS, barometer
    mutable std::mutex sensor_client_mutex_;  // Thread safety for sensor client access
    mutable std::mutex state_client_mutex_;   // Thread safety for state client access
    mutable std::mutex lidar_callback_mutex_; // CRITICAL: Prevents concurrent LiDAR callback execution with RAII exception safety
    mutable std::mutex state_callback_mutex_; // CRITICAL: Prevents concurrent state callback execution
    
    // Performance monitoring for RPC operations
    mutable std::atomic<uint64_t> rpc_calls_total_{0};
    mutable std::atomic<uint64_t> rpc_calls_successful_{0};
    mutable std::atomic<uint64_t> rpc_calls_failed_{0};

    // CRITICAL: Initialization guard to prevent timer callbacks before RPC clients/publishers are ready
    // Prevents race condition where state/lidar callbacks execute before initialize_common() completes
    std::atomic<bool> initialization_complete_{false};


    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr image_timer_;        // LEGACY - replaced by individual camera timers
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr gpulidar_timer_;
    rclcpp::TimerBase::SharedPtr echo_timer_;

    // Individual camera timers for parallel processing (performance optimization)
    rclcpp::TimerBase::SharedPtr camera_1_timer_;     // Camera 1 individual timer
    rclcpp::TimerBase::SharedPtr camera_2_timer_;     // Camera 2 individual timer
    rclcpp::TimerBase::SharedPtr camera_3_timer_;     // Camera 3 individual timer
    rclcpp::TimerBase::SharedPtr camera_4_timer_;     // Camera 4 individual timer

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

    // Settings-driven sensor management 
    std::vector<SensorPublisher<sensor_msgs::msg::Imu>> imu_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::MagneticField>> magnetometer_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::Altimeter>> barometer_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::Range>> distance_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> lidar_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> lidar_labels_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> gpulidar_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> echo_active_pubs_;
    std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> echo_passive_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> echo_active_labels_pubs_;
    std::vector<SensorPublisher<airsim_interfaces::msg::StringArray>> echo_passive_labels_pubs_;
    
    // Camera management (image_transport for better performance)
    std::vector<std::pair<std::vector<ImageRequest>, std::string>> camera_request_vehicle_pairs_;
    std::vector<image_transport::Publisher> image_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs_;
    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs_;
    std::unique_ptr<image_transport::ImageTransport> image_transport_;

    // Individual camera assignment for parallel processing (performance optimization)
    std::vector<size_t> camera_1_indices_;   // Camera indices assigned to camera_1_timer
    std::vector<size_t> camera_2_indices_;   // Camera indices assigned to camera_2_timer
    std::vector<size_t> camera_3_indices_;   // Camera indices assigned to camera_3_timer
    std::vector<size_t> camera_4_indices_;   // Camera indices assigned to camera_4_timer
    size_t active_camera_timers_;            // Number of active camera timers (1-4)
    
    // Instance segmentation and object transforms (wrapper features)
    rclcpp::Publisher<airsim_interfaces::msg::InstanceSegmentationList>::SharedPtr instance_segmentation_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::ObjectTransformsList>::SharedPtr object_transforms_pub_;
    
    // Dynamic sensor management (enhanced)
    std::vector<std::string> lidar_sensor_names_;  
    std::vector<std::string> gpulidar_sensor_names_;
    std::vector<std::string> echo_sensor_names_;
    std::vector<std::string> camera_sensor_names_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs_map_;
    
    // Legacy support (for backward compatibility)
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> legacy_lidar_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> legacy_camera_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> legacy_camera_info_pubs_;
    
    // Image type mapping (from wrapper)
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;

    // Missing member variables (moved from AirsimROSWrapper)
    AIRSIM_MODE airsim_mode_ = AIRSIM_MODE::DRONE;
    
    // Current sensor data (required for state publishing)
    nav_msgs::msg::Odometry curr_odom_;
    airsim_interfaces::msg::Environment env_msg_;
    sensor_msgs::msg::NavSatFix gps_sensor_msg_;
    
    // Legacy sensor publishers (for backward compatibility)
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<airsim_interfaces::msg::Altimeter>::SharedPtr baro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // Legacy sensor messages (for backward compatibility)
    sensor_msgs::msg::MagneticField mag_msg_;
    airsim_interfaces::msg::Altimeter baro_msg_;
    sensor_msgs::msg::Imu imu_msg_;

    // Enhanced 4-tier callback group architecture for optimal performance isolation
    std::shared_ptr<rclcpp::CallbackGroup> lidar_sensor_group_;   // LiDAR, GPU LiDAR (MutuallyExclusive)
    std::shared_ptr<rclcpp::CallbackGroup> camera_sensor_group_;  // Cameras, images (MutuallyExclusive) - LEGACY
    std::shared_ptr<rclcpp::CallbackGroup> light_sensor_group_;   // IMU, GPS, baro, echo (MutuallyExclusive)
    std::shared_ptr<rclcpp::CallbackGroup> control_group_;        // Commands, services (MutuallyExclusive)
    std::shared_ptr<rclcpp::CallbackGroup> cb_;                   // Legacy fallback group

    // Individual camera callback groups for parallel processing (performance optimization)
    std::shared_ptr<rclcpp::CallbackGroup> camera_1_group_;       // Camera 1 individual processing
    std::shared_ptr<rclcpp::CallbackGroup> camera_2_group_;       // Camera 2 individual processing
    std::shared_ptr<rclcpp::CallbackGroup> camera_3_group_;       // Camera 3 individual processing
    std::shared_ptr<rclcpp::CallbackGroup> camera_4_group_;       // Camera 4 individual processing
    
    // ROS clock (for compatibility)
    struct {
        rclcpp::Time clock;
    } ros_clock_;
};