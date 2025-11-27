#include "vehicle_node_base.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "common/AirSimSettings.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::placeholders;

// Static constants definitions (moved from AirsimROSWrapper)
constexpr char VehicleNodeBase::CAM_YML_NAME[];
constexpr char VehicleNodeBase::WIDTH_YML_NAME[];
constexpr char VehicleNodeBase::HEIGHT_YML_NAME[];
constexpr char VehicleNodeBase::K_YML_NAME[];
constexpr char VehicleNodeBase::D_YML_NAME[];
constexpr char VehicleNodeBase::R_YML_NAME[];
constexpr char VehicleNodeBase::P_YML_NAME[];
constexpr char VehicleNodeBase::DMODEL_YML_NAME[];

// Static image type mapping (moved from multirotor_node)
const std::unordered_map<int, std::string> VehicleNodeBase::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" },
    { 8, "OpticalFlow" },
    { 9, "OpticalFlowVis" },
    { 10, "Lighting" },
    { 11, "Annotation" },
};

// Global static AirSim RPC mutex definition - shared across ALL vehicle instances and ALL sensors
// Serializes ALL AirSim RPC calls (GPS, LiDAR, cameras, IMU, etc.) to prevent concurrent access
// CRITICAL FIX: Prevents SEGFAULT from concurrent RPC requests to AirSim server
std::mutex VehicleNodeBase::airsim_rpc_mutex_;

VehicleNodeBase::VehicleNodeBase(const std::string& vehicle_name, 
                                 const std::string& host_ip, 
                                 uint16_t host_port,
                                 const std::shared_ptr<rclcpp::CallbackGroup> callbackGroup)
    : Node(vehicle_name)  // Ultra-clean: node name IS vehicle name
    , vehicle_name_(vehicle_name)
    , host_ip_(host_ip)
    , host_port_(host_port)
    // REP 105 COMPLIANT: Initialize frame IDs to match your naming preference
    , robot_namespace_(vehicle_name)                        // Use vehicle name as namespace
    , map_frame_id_("map")                                  // Standard global frame  
    , odom_frame_id_(vehicle_name + "/odom")                // Per-vehicle odom frame: drone_1/odom, drone_2/odom
    , base_link_frame_id_(vehicle_name + "/base_link")     // Per-vehicle base_link: drone_1/base_link, drone_2/base_link
    , state_timer_freq_(0.02)        // 50Hz for state updates (IMU, GPS, baro) - balanced performance
    , image_timer_freq_(0.1)         // 10Hz for camera processing (dedicated camera callback group)  
    , lidar_timer_freq_(0.02)         // 10Hz for LiDAR processing (dedicated LiDAR callback group - prevents bottleneck)
    , gpulidar_timer_freq_(0.1)      // 10Hz for GPU LiDAR (shares LiDAR callback group)
    , echo_timer_freq_(0.05)         // 20Hz for echo/radar sensors (shares light sensor group)
    , sensor_rpc_timeout_ms_(30)  // 30ms timeout for sensor RPC calls (LiDAR, images)
    , state_rpc_timeout_ms_(5)    // 5ms timeout for state RPC calls (IMU, GPS, baro)
    // Initialize dual specialized clients
    , sensor_client_(nullptr)
    , state_client_(nullptr)
    // Initialize missing member variables
    , airsim_mode_(AIRSIM_MODE::DRONE)
    // Create enhanced 4-tier callback groups for optimal performance isolation
    , lidar_sensor_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , camera_sensor_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , light_sensor_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , control_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , cb_(callbackGroup ? callbackGroup : control_group_)
    // Create individual camera callback groups for parallel processing (performance optimization)
    , camera_1_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , camera_2_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , camera_3_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , camera_4_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    // Initialize camera assignment tracking
    , active_camera_timers_(0)
{
    // Dual client architecture for performance optimization
    // Derived class will initialize both sensor_client_ and state_client_ with appropriate specialized clients
    // Thread safety provided by separate mutexes for each client type

    ros_clock_.clock = rclcpp::Time(0);

    // Declare parameters
    this->declare_parameter("vehicle_name", vehicle_name);
    this->declare_parameter("host_ip", host_ip);
    this->declare_parameter("host_port", static_cast<int>(host_port));
    this->declare_parameter("map_frame_id", map_frame_id_);
    this->declare_parameter("odom_frame_id", odom_frame_id_);
    this->declare_parameter("base_link_frame_id", base_link_frame_id_);

    this->declare_parameter("state_timer_freq", state_timer_freq_);
    this->declare_parameter("image_timer_freq", image_timer_freq_);
    this->declare_parameter("lidar_timer_freq", lidar_timer_freq_);
    this->declare_parameter("gpulidar_timer_freq", gpulidar_timer_freq_);
    this->declare_parameter("echo_timer_freq", echo_timer_freq_);
    
<<<<<<< HEAD
    // Get parameters
=======
    // Dual-client timeout parameters
    this->declare_parameter("sensor_rpc_timeout_ms", sensor_rpc_timeout_ms_);
    this->declare_parameter("state_rpc_timeout_ms", state_rpc_timeout_ms_);
    
    // Get parameters
    this->get_parameter("vehicle_name", vehicle_name_);
>>>>>>> main
    this->get_parameter("host_ip", host_ip_);
    int port_param = static_cast<int>(host_port_);
    this->get_parameter("host_port", port_param);
    host_port_ = static_cast<uint16_t>(port_param);
<<<<<<< HEAD
    this->get_parameter("world_frame_id", world_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
=======
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_link_frame_id", base_link_frame_id_);
>>>>>>> main
    
    this->get_parameter("state_timer_freq", state_timer_freq_);
    this->get_parameter("image_timer_freq", image_timer_freq_);
    this->get_parameter("lidar_timer_freq", lidar_timer_freq_);
    this->get_parameter("gpulidar_timer_freq", gpulidar_timer_freq_);
    this->get_parameter("echo_timer_freq", echo_timer_freq_);
<<<<<<< HEAD

    // Phase 2.2: Set up isolated callback groups
    setup_callback_groups();

    // Phase 2.1: Initialize independent RPC clients per vehicle
    airsim_client_images_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_lidar_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_gpulidar_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_echo_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    
    RCLCPP_INFO(this->get_logger(), "Created vehicle node for: %s", vehicle_name_.c_str());
}

// Phase 2.2: Isolated Callback Groups Setup
void VehicleNodeBase::setup_callback_groups()
{
    state_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gpulidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    echo_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}
=======
    
    // Get dual-client timeout parameters
    this->get_parameter("sensor_rpc_timeout_ms", sensor_rpc_timeout_ms_);
    this->get_parameter("state_rpc_timeout_ms", state_rpc_timeout_ms_);

    // Load settings from AirSim server before determining mode
    try {
        RCLCPP_INFO(this->get_logger(), "Loading settings from AirSim server at %s:%d...", host_ip_.c_str(), host_port_);
        msr::airlib::RpcLibClientBase temp_client(host_ip_, host_port_);
        temp_client.confirmConnection();
        std::string settings_text = temp_client.getSettingsString();
        
        if (!settings_text.empty()) {
            // Initialize AirSim settings with fetched data
            msr::airlib::AirSimSettings::initializeSettings(settings_text);
            
            // Parse settings JSON to get SimMode explicitly
            msr::airlib::Settings settings_json = msr::airlib::Settings::loadJSonString(settings_text);
            std::string sim_mode = settings_json.getString("SimMode", "Multirotor");  // Default to Multirotor
            
            RCLCPP_INFO(this->get_logger(), "SimMode from settings: '%s'", sim_mode.c_str());
            
            // Handle empty or missing SimMode
            if (sim_mode.empty()) {
                sim_mode = "Multirotor";
                RCLCPP_WARN(this->get_logger(), "Empty Sim Mode in settings, defaulting to 'Multirotor'");
            }
            
            // Load settings into singleton with explicit SimMode
            msr::airlib::AirSimSettings::singleton().load([&]() -> std::string {
                return sim_mode;
            });
            
            RCLCPP_INFO(this->get_logger(), "Successfully loaded settings from AirSim server");
            RCLCPP_INFO(this->get_logger(), "Sim Mode: '%s'", 
                        msr::airlib::AirSimSettings::singleton().simmode_name.c_str());
                        
            // Check if our vehicle exists in the settings with name mapping
            std::string mapped_vehicle_name = findVehicleInSettings(settings_json, vehicle_name_);
            if (!mapped_vehicle_name.empty() && mapped_vehicle_name != vehicle_name_) {
                RCLCPP_INFO(this->get_logger(), "Vehicle name mapping: '%s' -> '%s' in settings", 
                           vehicle_name_.c_str(), mapped_vehicle_name.c_str());
            } else if (mapped_vehicle_name.empty()) {
                RCLCPP_WARN(this->get_logger(), "Vehicle '%s' not found in settings, using defaults", vehicle_name_.c_str());
            }
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Empty settings received from server, using defaults");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load settings from AirSim: %s", e.what());
        RCLCPP_WARN(this->get_logger(), "Defaulting to DRONE mode for %s", vehicle_name_.c_str());
        airsim_mode_ = AIRSIM_MODE::DRONE;
    }

    // Determine the simulation mode based on loaded settings or defaults
    std::string actual_simmode = msr::airlib::AirSimSettings::singleton().simmode_name;
    
    RCLCPP_INFO(this->get_logger(), "Determining simulation mode from: '%s'", actual_simmode.c_str());
    
    if (!actual_simmode.empty()) {
        if (actual_simmode == msr::airlib::AirSimSettings::kSimModeTypeMultirotor) {
            airsim_mode_ = AIRSIM_MODE::DRONE;
            RCLCPP_INFO(this->get_logger(), "Setting ROS wrapper to DRONE mode");
        }
        else if (actual_simmode == msr::airlib::AirSimSettings::kSimModeTypeCar || 
                 actual_simmode == msr::airlib::AirSimSettings::kSimModeTypeSkidVehicle) {
            airsim_mode_ = AIRSIM_MODE::CAR;
            RCLCPP_INFO(this->get_logger(), "Setting ROS wrapper to CAR mode");
        } else if (actual_simmode == msr::airlib::AirSimSettings::kSimModeTypeComputerVision) {
            airsim_mode_ = AIRSIM_MODE::COMPUTERVISION;
            RCLCPP_INFO(this->get_logger(), "Setting ROS wrapper to COMPUTERVISION mode");
        } else {
            // Unknown mode - default to drone for multirotor vehicles
            RCLCPP_WARN(this->get_logger(), "Unknown SimMode '%s', defaulting to DRONE mode", actual_simmode.c_str());
            airsim_mode_ = AIRSIM_MODE::DRONE;
        }
    } else {
        // No mode specified - use default
        airsim_mode_ = AIRSIM_MODE::DRONE;
        RCLCPP_INFO(this->get_logger(), "Using default DRONE mode for vehicle: %s", vehicle_name_.c_str());
    }

    // Note: initialize_common() will be called from main() after shared_ptr creation
    // This avoids shared_from_this() issues in constructor
    
    RCLCPP_INFO(this->get_logger(), "Vehicle node constructor completed for: %s", vehicle_name_.c_str());
}

>>>>>>> main

void VehicleNodeBase::initialize_common()
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
<<<<<<< HEAD
    if (!establish_connections()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish connections for vehicle: %s", vehicle_name_.c_str());
        return;
    }
    
    // Initialize vehicle-specific client
    initialize_vehicle_client();
    
    // Setup ROS components
    setup_publishers();
    setup_services();
    setup_timers();
    
    RCLCPP_INFO(this->get_logger(), "Vehicle node initialized successfully: %s", vehicle_name_.c_str());
=======
    // Initialize image transport (needed for camera publishers)
    try {
        image_transport_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Image transport initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not initialize image transport: %s", e.what());
        RCLCPP_WARN(this->get_logger(), "Camera topics may not work, but sensor topics should be fine");
    }
    
    // Note: establish_connections() is now called by derived classes after they initialize their specific client
    
    create_ros_pubs_from_settings_json();

    setup_services();
    setup_timers();

    // NOTE: initialization_complete_ flag is set by derived classes after their specific setup
    // This ensures timer callbacks don't execute until ALL initialization is complete

}

// ===== SETTINGS-DRIVEN CONFIGURATION: Publishers =====

void VehicleNodeBase::create_ros_pubs_from_settings_json()
{

    RCLCPP_INFO(this->get_logger(), "Creating publishers from settings now"); 
    std::string topic_prefix = vehicle_name_ + "/";
    
    // Common publishers for all vehicle types with ultra-clean naming
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "odom", 10);  // REP 105 standard topic naming
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix + "global_gps", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>(topic_prefix + "environment", 10);
    
    // REP 105 FRAME AUTHORITIES: map→odom transform should be published by localization component, not static
    // REMOVED: create_vehicle_map_to_odom_transform(); - violates REP 105 frame authorities
    // The map→odom transform MUST be published dynamically by a localization component
    RCLCPP_INFO(this->get_logger(), "Odometry publisher created: topic='%s', frame_id='%s' → '%s'", 
               (topic_prefix + "odom").c_str(), odom_frame_id_.c_str(), base_link_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "REP 105 Compliance: map→%s transform will be published by localization component", 
               odom_frame_id_.c_str());
    
    // Call virtual method for vehicle-specific publishers
    create_publishers_from_settings();
    
    // DISABLED: Initialize dynamic sensor discovery (causes duplicate publishers with different topic names)
    // discover_lidar_sensors();
}

void VehicleNodeBase::create_publishers_from_settings()
{
    try {
        RCLCPP_INFO(this->get_logger(), "Creating publishers from AirSim settings for vehicle: %s", vehicle_name_.c_str());
        
        // Check if this vehicle exists in settings
        const auto& vehicles = msr::airlib::AirSimSettings::singleton().vehicles;
        auto vehicle_it = vehicles.find(vehicle_name_);
        
        if (vehicle_it == vehicles.end()) {
            RCLCPP_WARN(this->get_logger(), "Vehicle %s not found in settings.json, using legacy discovery", vehicle_name_.c_str());
            return;
        }

        auto& vehicle_setting = vehicle_it->second;
        set_nans_to_zeros_in_pose(*vehicle_setting);
        
        // Topic prefixing: vehicle_name + "/"
        std::string topic_prefix = vehicle_name_ + "/";
        
        // Setup camera publishers from settings
        setup_camera_publishers_from_settings(*vehicle_setting, topic_prefix);
        
        // Setup default camera publishers if not already configured
        setup_default_camera_publishers(topic_prefix);
        
        // Setup sensor publishers from settings  
        setup_sensor_publishers_from_settings(*vehicle_setting, topic_prefix);
        
        // Setup default sensor publishers (IMU, magnetometer, barometer) if not already configured
        setup_default_sensor_publishers(topic_prefix);
        
        // Setup additional features
        setup_instance_segmentation_publisher(topic_prefix);
        setup_object_transforms_publisher(topic_prefix);
        
        RCLCPP_INFO(this->get_logger(), "Sensor setup summary for vehicle %s:", vehicle_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "LiDAR sensors: %zu publishers created", lidar_pubs_.size());
        RCLCPP_INFO(this->get_logger(), "Camera sensors: %zu publishers created", image_pubs_.size());
        RCLCPP_INFO(this->get_logger(), "IMU sensors: %zu publishers created", imu_pubs_.size());
        RCLCPP_INFO(this->get_logger(), "Magnetometer sensors: %zu publishers created", magnetometer_pubs_.size());
        RCLCPP_INFO(this->get_logger(), "Barometer sensors: %zu publishers created", barometer_pubs_.size());
        RCLCPP_INFO(this->get_logger(), "Successfully created all publishers from settings");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create publishers from settings for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

// Placeholder implementations for complex settings-driven configuration
void VehicleNodeBase::setup_camera_publishers_from_settings(const VehicleSetting& vehicle_setting,
                                                            const std::string& topic_prefix)
{

    RCLCPP_INFO(this->get_logger(), "Creating Image Publishers: %sImage", topic_prefix.c_str());
    // Iterate over camera map std::map<std::string, CameraSetting> .cameras;
    for (const auto& camera_elem : vehicle_setting.cameras) {
        const auto& camera_setting = camera_elem.second;
        const auto& camera_name = camera_elem.first;

        set_nans_to_zeros_in_pose(vehicle_setting, const_cast<CameraSetting&>(camera_setting));
        append_static_camera_tf(camera_name, camera_setting);
        
        std::vector<ImageRequest> current_image_requests;
        current_image_requests.clear();
        
        // Iterate over capture_setting std::map<int, CaptureSetting> capture_settings
        for (const auto& capture_elem : camera_setting.capture_settings) {
            const auto& capture_setting = capture_elem.second;
            
            if (!(std::isnan(capture_setting.fov_degrees))) {
                msr::airlib::ImageCaptureBase::ImageType curr_image_type = msr::airlib::Utils::toEnum<msr::airlib::ImageCaptureBase::ImageType>(capture_setting.image_type);
                
                if(curr_image_type == msr::airlib::ImageCaptureBase::ImageType::Annotation) {
                    // Handle annotation images (multiple annotations per camera)
                    for(const auto& annotation_element : msr::airlib::AirSimSettings::singleton().annotator_settings) {
                        current_image_requests.emplace_back(camera_name, curr_image_type, false, false, annotation_element.name);
                        
                        const std::string camera_topic_prefix = topic_prefix + camera_name + "_" + 
                                                               image_type_int_to_string_map_.at(capture_setting.image_type) + "_" + 
                                                               annotation_element.name;
                        const std::string image_topic = camera_topic_prefix + "/image";
                        const std::string camera_info_topic = camera_topic_prefix + "/camera_info";
                        
                        image_pubs_.push_back(image_transport_->advertise(image_topic, 1));
                        legacy_camera_info_pubs_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10));
                        camera_info_msgs_.push_back(generate_camera_info(camera_name, camera_setting, capture_setting));
                    }
                } else {
                    // Handle regular image types
                    if (curr_image_type == msr::airlib::ImageCaptureBase::ImageType::DepthPlanar || 
                        curr_image_type == msr::airlib::ImageCaptureBase::ImageType::DepthPerspective || 
                        curr_image_type == msr::airlib::ImageCaptureBase::ImageType::DepthVis || 
                        curr_image_type == msr::airlib::ImageCaptureBase::ImageType::DisparityNormalized) {
                        current_image_requests.emplace_back(camera_name, curr_image_type, true);
                    } else {
                        current_image_requests.emplace_back(camera_name, curr_image_type, false, false);
                    }
                    
                    const std::string camera_topic_prefix = topic_prefix + camera_name + "_" + 
                                                           image_type_int_to_string_map_.at(capture_setting.image_type);
                    const std::string image_topic = camera_topic_prefix + "/image";
                    const std::string camera_info_topic = camera_topic_prefix + "/camera_info";
                    
                    image_pubs_.push_back(image_transport_->advertise(image_topic, 1));
                    legacy_camera_info_pubs_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10));
                    camera_info_msgs_.push_back(generate_camera_info(camera_name, camera_setting, capture_setting));
                }
            }
        }
        
        // Store camera requests for this vehicle
        if (!current_image_requests.empty()) {
            camera_request_vehicle_pairs_.emplace_back(current_image_requests, vehicle_name_);
            camera_sensor_names_.push_back(camera_name);
            
            RCLCPP_INFO(this->get_logger(), "Setup camera %s with %zu capture settings", 
                       camera_name.c_str(), current_image_requests.size());
        }
    }
}

// Camera info generation placeholder
sensor_msgs::msg::CameraInfo VehicleNodeBase::generate_camera_info(const std::string& camera_name,
                                                                   [[maybe_unused]] const CameraSetting& camera_setting,
                                                                   const CaptureSetting& capture_setting) const
{
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    
    float f_x = (capture_setting.width / 2.0) / tan(capture_setting.fov_degrees * M_PI / 180.0 / 2.0);
    
    cam_info_msg.k = {
        f_x, 0.0, capture_setting.width / 2.0,
        0.0, f_x, capture_setting.height / 2.0,
        0.0, 0.0, 1.0
    };
    
    cam_info_msg.p = {
        f_x, 0.0, capture_setting.width / 2.0, 0.0,
        0.0, f_x, capture_setting.height / 2.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    return cam_info_msg;
}

void VehicleNodeBase::setup_sensor_publishers_from_settings(const VehicleSetting& vehicle_setting,
                                                            const std::string& topic_prefix)
{
    RCLCPP_INFO(this->get_logger(), "Processing %zu sensors from settings.json for vehicle: %s", 
               vehicle_setting.sensors.size(), vehicle_name_.c_str());
               
    // Iterate over sensors
    for (const auto& sensor_elem : vehicle_setting.sensors) {
        const auto& sensor_name = sensor_elem.first;
        const auto& sensor_setting = sensor_elem.second;
        
        RCLCPP_DEBUG(this->get_logger(), "Found sensor '%s' of type %d, enabled: %s", 
                    sensor_name.c_str(), static_cast<int>(sensor_setting->sensor_type), 
                    sensor_setting->enabled ? "true" : "false");
        
        if (!sensor_setting->enabled) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping disabled sensor: %s", sensor_name.c_str());
            continue;
        }
        
        switch (sensor_setting->sensor_type) {
            case msr::airlib::SensorBase::SensorType::Barometer: {
                auto sensor_publisher = create_sensor_publisher<airsim_interfaces::msg::Altimeter>(
                    "Barometer sensor", sensor_name, sensor_setting->sensor_type, 
                    topic_prefix + sensor_name, 10);
                barometer_pubs_.emplace_back(sensor_publisher);
                break;
            }
            case msr::airlib::SensorBase::SensorType::Imu: {
                auto sensor_publisher = create_sensor_publisher<sensor_msgs::msg::Imu>(
                    "IMU sensor", sensor_name, sensor_setting->sensor_type, 
                    topic_prefix + sensor_name, 10);
                imu_pubs_.emplace_back(sensor_publisher);
                break;
            }
            case msr::airlib::SensorBase::SensorType::Gps: {
                // GPS is handled by base class
                RCLCPP_DEBUG(this->get_logger(), "GPS sensor %s handled by base class", sensor_name.c_str());
                break;
            }
            case msr::airlib::SensorBase::SensorType::Magnetometer: {
                auto sensor_publisher = create_sensor_publisher<sensor_msgs::msg::MagneticField>(
                    "Magnetometer sensor", sensor_name, sensor_setting->sensor_type, 
                    topic_prefix + sensor_name, 10);
                magnetometer_pubs_.emplace_back(sensor_publisher);
                break;
            }
            case msr::airlib::SensorBase::SensorType::Distance: {
                auto sensor_publisher = create_sensor_publisher<sensor_msgs::msg::Range>(
                    "Distance sensor", sensor_name, sensor_setting->sensor_type, 
                    topic_prefix + sensor_name, 10);
                distance_pubs_.emplace_back(sensor_publisher);
                break;
            }
            case msr::airlib::SensorBase::SensorType::Lidar: {
                RCLCPP_INFO(this->get_logger(), "Processing LiDAR sensor: %s", sensor_name.c_str());
                
                auto lidar_setting = *static_cast<msr::airlib::AirSimSettings::LidarSetting*>(sensor_setting.get());
                msr::airlib::LidarSimpleParams params;
                params.initializeFromSettings(lidar_setting);
                
                RCLCPP_DEBUG(this->get_logger(), "LiDAR %s parameters: channels=%d, range=%f", 
                            sensor_name.c_str(), params.number_of_channels, params.range);
                
                // Create static TF for this LiDAR
                append_static_lidar_tf(sensor_name, params);
                
                // Use settings.json naming convention: /{vehicle}/{sensor_name}/points
                std::string points_topic = topic_prefix + sensor_name + "/points";
                std::string labels_topic = topic_prefix + sensor_name + "/labels";
                
                auto sensor_publisher = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                    "LiDAR sensor", sensor_name, sensor_setting->sensor_type, 
                    points_topic, 10);
                lidar_pubs_.emplace_back(sensor_publisher);
                
                auto labels_publisher = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                    "", sensor_name, sensor_setting->sensor_type, 
                    labels_topic, 10);
                lidar_labels_pubs_.emplace_back(labels_publisher);
                
                RCLCPP_INFO(this->get_logger(), "Created LiDAR publishers for sensor %s: points=%s, labels=%s", 
                           sensor_name.c_str(), points_topic.c_str(), labels_topic.c_str());
                
                lidar_sensor_names_.push_back(sensor_name);
                break;
            }
            case msr::airlib::SensorBase::SensorType::GPULidar: {
                auto gpulidar_setting = *static_cast<msr::airlib::AirSimSettings::GPULidarSetting*>(sensor_setting.get());
                msr::airlib::GPULidarSimpleParams params;
                params.initializeFromSettings(gpulidar_setting);
                
                // Create static TF for this GPU LiDAR
                append_static_gpulidar_tf(sensor_name, params);
                
                auto sensor_publisher = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                    "GPU LiDAR sensor", sensor_name, sensor_setting->sensor_type, 
                    topic_prefix + "gpulidar/points" + sensor_name, 10);
                gpulidar_pubs_.emplace_back(sensor_publisher);
                
                gpulidar_sensor_names_.push_back(sensor_name);
                break;
            }
            case msr::airlib::SensorBase::SensorType::Echo: {
                auto echo_setting = *static_cast<msr::airlib::AirSimSettings::EchoSetting*>(sensor_setting.get());
                msr::airlib::EchoSimpleParams params;
                params.initializeFromSettings(echo_setting);
                
                // Create static TF for this Echo sensor
                append_static_echo_tf(sensor_name, params);
                
                if(params.active) {
                    auto active_publisher = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "Echo (active) sensor", sensor_name, sensor_setting->sensor_type, 
                        topic_prefix + "echo/active/points" + sensor_name, 10);
                    echo_active_pubs_.emplace_back(active_publisher);
                    
                    auto active_labels_publisher = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                        "", sensor_name, sensor_setting->sensor_type, 
                        topic_prefix + "echo/active/labels/" + sensor_name, 10);
                    echo_active_labels_pubs_.emplace_back(active_labels_publisher);
                }
                
                if(params.passive) {
                    auto passive_publisher = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "Echo (passive) sensor", sensor_name, sensor_setting->sensor_type, 
                        topic_prefix + "echo/passive/points" + sensor_name, 10);
                    echo_passive_pubs_.emplace_back(passive_publisher);
                    
                    auto passive_labels_publisher = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                        "", sensor_name, sensor_setting->sensor_type, 
                        topic_prefix + "echo/passive/labels/" + sensor_name, 10);
                    echo_passive_labels_pubs_.emplace_back(passive_labels_publisher);
                }
                
                echo_sensor_names_.push_back(sensor_name);
                break;
            }
            default: {
                RCLCPP_WARN(this->get_logger(), "Unknown sensor type %d for sensor: %s", 
                           static_cast<int>(sensor_setting->sensor_type), sensor_name.c_str());
            }
        }
    }
}

void VehicleNodeBase::setup_default_sensor_publishers(const std::string& topic_prefix)
{
    // Check if IMU publisher is already created from settings
    if (imu_pubs_.empty()) {
        auto imu_publisher = create_sensor_publisher<sensor_msgs::msg::Imu>(
            "Default IMU sensor", "Imu", msr::airlib::SensorBase::SensorType::Imu,
            topic_prefix + "imu", 10);
        imu_pubs_.emplace_back(imu_publisher);
        RCLCPP_INFO(this->get_logger(), "Setup default IMU publisher");
    }
    
    // Check if magnetometer publisher is already created from settings
    if (magnetometer_pubs_.empty()) {
        auto mag_publisher = create_sensor_publisher<sensor_msgs::msg::MagneticField>(
            "Default Magnetometer sensor", "Magnetometer", msr::airlib::SensorBase::SensorType::Magnetometer,
            topic_prefix + "magnetometer", 10);
        magnetometer_pubs_.emplace_back(mag_publisher);
        RCLCPP_INFO(this->get_logger(), "Setup default magnetometer publisher");
    }
    
    // Check if barometer publisher is already created from settings
    if (barometer_pubs_.empty()) {
        auto baro_publisher = create_sensor_publisher<airsim_interfaces::msg::Altimeter>(
            "Default Barometer sensor", "Barometer", msr::airlib::SensorBase::SensorType::Barometer,
            topic_prefix + "barometer", 10);
        barometer_pubs_.emplace_back(baro_publisher);
        RCLCPP_INFO(this->get_logger(), "Setup default barometer publisher");
    }
}

void VehicleNodeBase::setup_default_camera_publishers(const std::string& topic_prefix)
{
    // Check if any cameras were already created from settings.json
    if (!camera_request_vehicle_pairs_.empty() || !image_pubs_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Cameras already configured from settings.json, skipping defaults");
        return;
    }
    
    try {
        // AirSim multirotor vehicles have these default cameras when none are specified in settings
        std::vector<std::string> airsim_default_cameras = {
            "front_center",  // Primary forward camera
            "front_right",   // Right forward camera  
            "front_left",    // Left forward camera
            "bottom_center", // Downward camera
            "back_center"    // Rear camera
        };
        
        // Create default capture setting for camera info
        msr::airlib::AirSimSettings::CaptureSetting default_capture_setting;
        default_capture_setting.image_type = 0; // Scene (RGB)
        default_capture_setting.width = 640;
        default_capture_setting.height = 480; 
        default_capture_setting.fov_degrees = 90.0f;
        
        // Create minimal camera setting template for camera info generation
        msr::airlib::AirSimSettings::CameraSetting default_camera_setting;
        default_camera_setting.position = msr::airlib::Vector3r(0.0f, 0.0f, 0.0f);
        default_camera_setting.rotation.pitch = 0.0f;
        default_camera_setting.rotation.roll = 0.0f;
        default_camera_setting.rotation.yaw = 0.0f;
        default_camera_setting.external = false;
        
        // Only create cameras that don't already exist in settings
        for (const auto& camera_name : airsim_default_cameras) {
            
            // Check if camera already exists from settings
            bool camera_exists = std::find(camera_sensor_names_.begin(), camera_sensor_names_.end(), camera_name) != camera_sensor_names_.end();
            if (camera_exists) {
                RCLCPP_DEBUG(this->get_logger(), "Camera %s already configured from settings, skipping default", camera_name.c_str());
                continue;
            }
            
            // Create image request for Scene (RGB) capture
            std::vector<ImageRequest> camera_image_requests;
            camera_image_requests.emplace_back(camera_name, 
                                             msr::airlib::ImageCaptureBase::ImageType::Scene, 
                                             false, false);
            
            // Setup publishers with valid ROS topic naming (camera_name_scene)
            const std::string camera_topic_prefix = topic_prefix + camera_name + "_scene";
            const std::string image_topic = camera_topic_prefix + "/image";
            const std::string camera_info_topic = camera_topic_prefix + "/camera_info";
            
            image_pubs_.push_back(image_transport_->advertise(image_topic, 1));
            camera_info_pubs_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10));
            camera_info_msgs_.push_back(generate_camera_info(camera_name, default_camera_setting, default_capture_setting));
            
            // Create basic static TF for camera (without client dependency)
            append_static_camera_tf_basic(camera_name, default_camera_setting);
            
            // Store camera request and name
            camera_request_vehicle_pairs_.emplace_back(camera_image_requests, vehicle_name_);
            camera_sensor_names_.push_back(camera_name);
            
            RCLCPP_INFO(this->get_logger(), "Setup default AirSim camera '%s' with Scene (RGB) capture", camera_name.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to setup default camera publishers: %s", e.what());
    }
}

void VehicleNodeBase::setup_instance_segmentation_publisher(const std::string& topic_prefix)
{
    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    instance_segmentation_pub_ = this->create_publisher<airsim_interfaces::msg::InstanceSegmentationList>(
        topic_prefix + "instance_segmentation_labels", qos_settings);
    
    RCLCPP_INFO(this->get_logger(), "Setup instance segmentation publisher");
}

void VehicleNodeBase::setup_object_transforms_publisher(const std::string& topic_prefix)
{
    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    object_transforms_pub_ = this->create_publisher<airsim_interfaces::msg::ObjectTransformsList>(
        topic_prefix + "object_transforms", qos_settings);
    
    RCLCPP_INFO(this->get_logger(), "Setup object transforms publisher");
>>>>>>> main
}

bool VehicleNodeBase::establish_connections()
{
<<<<<<< HEAD
    try {
        // Phase 2.1: Independent connection management per vehicle, isolated failure handling
        airsim_client_images_->confirmConnection();
        airsim_client_lidar_->confirmConnection();
        airsim_client_gpulidar_->confirmConnection();
        airsim_client_echo_->confirmConnection();
=======
    RCLCPP_INFO(this->get_logger(), "Establishing connections to the clients now...");
    
    // Base class doesn't create clients - this is handled by derived classes
    // The derived class should call initialize_vehicle_client() to set up airsim_client_
    
    if (!state_client_) {
        RCLCPP_ERROR(this->get_logger(), "No AirSim client found. Derived class must call initialize_vehicle_client() first.");
        return false;
    }

    // Skip individual sensor client connections to avoid overwhelming AirSim
    try {
        // Phase 2.1: Independent connection management per vehicle, isolated failure handling
        // state_client_images_->confirmConnection();
        // state_client_lidar_->confirmConnection();
        // state_client_gpulidar_->confirmConnection();
        // state_client_echo_->confirmConnection();
>>>>>>> main
        
        RCLCPP_INFO(this->get_logger(), "RPC connections established for vehicle: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "connection establishment");
        return false;
    }
<<<<<<< HEAD
}

void VehicleNodeBase::setup_publishers()
{
    // Common publishers for all vehicle types
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>("environment", 10);
    
    // Call virtual method for vehicle-specific publishers
    setup_vehicle_publishers();
=======
    
    RCLCPP_INFO(this->get_logger(), "RPC connections established for all: %s", vehicle_name_.c_str());
>>>>>>> main
}

void VehicleNodeBase::setup_services()
{
<<<<<<< HEAD
    reset_service_ = this->create_service<airsim_interfaces::srv::Reset>(
        "reset", 
        std::bind(&VehicleNodeBase::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Call virtual method for vehicle-specific services
    setup_vehicle_services();
=======
    // Topic prefixing: vehicle_name + "/" + service
    std::string topic_prefix = vehicle_name_ + "/";
    
    reset_service_ = this->create_service<airsim_interfaces::srv::Reset>(
        topic_prefix + "reset", 
        std::bind(&VehicleNodeBase::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    
>>>>>>> main
}

// Phase 2.1: Independent Timer Management with isolated callback groups
void VehicleNodeBase::setup_timers()
{
<<<<<<< HEAD
    // Phase 2.1: Per-vehicle timer frequencies, each vehicle has independent timers
    state_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(state_timer_freq_),
        std::bind(&VehicleNodeBase::state_timer_callback, this),
        state_callback_group_);
        
    image_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(image_timer_freq_),
        std::bind(&VehicleNodeBase::image_timer_callback, this),
        sensor_callback_group_);
        
    lidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(lidar_timer_freq_),
        std::bind(&VehicleNodeBase::lidar_timer_callback, this),
        lidar_callback_group_);
        
    gpulidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(gpulidar_timer_freq_),
        std::bind(&VehicleNodeBase::gpulidar_timer_callback, this),
        gpulidar_callback_group_);
        
    echo_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(echo_timer_freq_),
        std::bind(&VehicleNodeBase::echo_timer_callback, this),
        echo_callback_group_);
=======
    // 4-TIER CALLBACK ARCHITECTURE: Assign timers to specialized callback groups
    
    // State timer uses light_sensor_group for low-latency operations (IMU, GPS, baro)
    state_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(state_timer_freq_),
        std::bind(&VehicleNodeBase::state_timer_callback, this), light_sensor_group_);
        
    // Camera operations use individual timers for parallel processing (performance optimization)
    // setup_individual_camera_timers();

    // LEGACY: Keep image_timer_ for backward compatibility (will be inactive if individual timers are used)
    if (active_camera_timers_ == 0) {
        image_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(image_timer_freq_),
            std::bind(&VehicleNodeBase::image_timer_callback, this), camera_sensor_group_);
    }
        
    // LiDAR operations use dedicated lidar_sensor_group for point cloud processing
    lidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(lidar_timer_freq_),
        std::bind(&VehicleNodeBase::lidar_timer_callback, this), lidar_sensor_group_);
            
    RCLCPP_INFO(this->get_logger(), "LiDAR timer created with frequency: %.1f Hz (%.3f sec) using dedicated LiDAR callback group", 
               1.0/lidar_timer_freq_, lidar_timer_freq_);
        
    // GPU LiDAR also uses lidar_sensor_group for consistent point cloud processing
    gpulidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(gpulidar_timer_freq_),
        std::bind(&VehicleNodeBase::gpulidar_timer_callback, this), lidar_sensor_group_);
        
    // Echo sensors use light_sensor_group as they're typically lower bandwidth  
    echo_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(echo_timer_freq_),
        std::bind(&VehicleNodeBase::echo_timer_callback, this), light_sensor_group_);
        
    RCLCPP_INFO(this->get_logger(), 
        "4-tier callback groups configured: State %.1fHz (light), LiDAR %.1fHz (lidar), Images %.1fHz (camera), Echo %.1fHz (light)",
        1.0/state_timer_freq_, 1.0/lidar_timer_freq_, 1.0/image_timer_freq_, 1.0/echo_timer_freq_);
>>>>>>> main
}

// Phase 2.1: Independent timer callbacks
void VehicleNodeBase::state_timer_callback()
{
<<<<<<< HEAD
    try {
        update_vehicle_state();
        publish_vehicle_state();
        handle_vehicle_commands();
=======
    // CRITICAL: Initialization guard - prevent execution before RPC clients/publishers ready
    if (!initialization_complete_.load()) {
        return;  // Skip callback until initialize_common() completes
    }

    // DIAGNOSTIC: Manual throttled log (5s interval) - safe alternative to RCLCPP_INFO_THROTTLE
    static auto last_state_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_state_log).count() >= 5) {
        RCLCPP_INFO(this->get_logger(), "✓ State callback active for %s (50Hz)", vehicle_name_.c_str());
        last_state_log = now;
    }

    // RAII-based mutex with try_to_lock: Prevents concurrent execution with other callbacks
    std::unique_lock<std::mutex> lock(state_callback_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        // Another high-priority operation is running - skip this iteration
        return;
    }

    try {
        update_vehicle_state();
        process_state_changes();  // NEW: Virtual hook for state change processing
        publish_vehicle_state();
        process_vehicle_commands();
>>>>>>> main
        publish_static_transforms();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "state update");
    }
<<<<<<< HEAD
=======
    // Mutex automatically released by RAII (exception-safe!)
>>>>>>> main
}

void VehicleNodeBase::image_timer_callback()
{
<<<<<<< HEAD
=======
    // CRITICAL: Initialization guard - prevent execution before RPC clients ready
    if (!initialization_complete_.load()) {
        return;  // Skip callback until initialize_common() completes
    }

>>>>>>> main
    try {
        process_images();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "image processing");
    }
}

<<<<<<< HEAD
void VehicleNodeBase::lidar_timer_callback()
{
    try {
        process_lidar();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "lidar processing");
    }
=======
// Individual camera timer callbacks for parallel processing
void VehicleNodeBase::camera_1_timer_callback()
{
    try {
        process_camera_1();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 1 processing");
    }
}

void VehicleNodeBase::camera_2_timer_callback()
{
    try {
        process_camera_2();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 2 processing");
    }
}

void VehicleNodeBase::camera_3_timer_callback()
{
    try {
        process_camera_3();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 3 processing");
    }
}

void VehicleNodeBase::camera_4_timer_callback()
{
    try {
        process_camera_4();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 4 processing");
    }
}

void VehicleNodeBase::lidar_timer_callback()
{
    // CRITICAL: Initialization guard - prevent execution before RPC clients ready
    if (!initialization_complete_.load()) {
        return;  // Skip callback until initialize_common() completes
    }

    // CRITICAL: Non-blocking guard to prevent concurrent callback execution (race condition fix)
    // RAII-based mutex with try_to_lock: Non-blocking + automatic cleanup on exception
    std::unique_lock<std::mutex> lock(lidar_callback_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        // Another callback is already running - skip this one (non-blocking behavior)
        // DISABLED THROTTLE: ROS2 Humble bug - RCLCPP_*_THROTTLE uses static vars causing thread corruption
        return;
    }

    try {
        // DIAGNOSTIC: Manual throttled log (5s interval) - safe alternative to RCLCPP_INFO_THROTTLE
        static auto last_lidar_log = std::chrono::steady_clock::now();
        auto log_now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_lidar_log).count() >= 5) {
            RCLCPP_INFO(this->get_logger(), "✓ LiDAR callback active for %s (50Hz)", vehicle_name_.c_str());
            last_lidar_log = log_now;
        }

        auto start_time = std::chrono::steady_clock::now();
        process_lidar();
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // DISABLED: RCLCPP_INFO_THROTTLE has threading issues causing SEGFAULT
        // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //     "LiDAR processing took %ld ms", duration.count());
    }
    catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR(this->get_logger(), "RPC error in LiDAR timer: %s", e.what());
        handle_rpc_error(e, "lidar processing");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in LiDAR timer: %s", e.what());
    }

    // Mutex automatically released by RAII when lock goes out of scope (exception-safe!)
>>>>>>> main
}

void VehicleNodeBase::gpulidar_timer_callback()
{
    try {
        process_gpulidar();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "gpulidar processing");
    }
}

void VehicleNodeBase::echo_timer_callback()
{
<<<<<<< HEAD
=======
    // CRITICAL: Initialization guard - prevent execution before RPC clients ready
    if (!initialization_complete_.load()) {
        return;  // Skip callback until initialize_common() completes
    }

>>>>>>> main
    try {
        process_echo();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "echo processing");
    }
}

void VehicleNodeBase::publish_odometry_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
<<<<<<< HEAD
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
=======
    odom_tf.header.stamp = odom_msg.header.stamp;
    
    // REP 105 COMPLIANT: {vehicle}/odom → {vehicle}/base_link transform (dynamic from robot odometry)
    odom_tf.header.frame_id = odom_msg.header.frame_id;      // "{vehicle}/odom" 
    odom_tf.child_frame_id = odom_msg.child_frame_id;        // "{vehicle}/base_link"
    
    // Use actual position data from AirSim (already converted in odom_msg)
>>>>>>> main
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
<<<<<<< HEAD
    tf_broadcaster_->sendTransform(odom_tf);
=======

    // DISABLED THROTTLE: ROS2 Humble bug - RCLCPP_DEBUG_THROTTLE uses static vars causing thread corruption
    // This function runs at 50Hz in state_timer_callback - logging disabled for thread safety
    // REP 105: Log standard ROS transform chain with detailed diagnostics
    // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "PUBLISHING TF: %s → %s at [%.3f, %.3f, %.3f] (ENU coordinates)",
    //     odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str(),
    //     odom_tf.transform.translation.x, odom_tf.transform.translation.y, odom_tf.transform.translation.z);
    
    // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    //     "Transform Chain Status: map → %s → %s → sensors (expecting localization for map→odom)", 
    //     (odom_frame_id_).c_str(), base_link_frame_id_.c_str());
    
    // Validate TF broadcaster before sending
    if (!tf_broadcaster_) {
        RCLCPP_ERROR(this->get_logger(), "TF broadcaster not initialized - cannot publish %s → %s transform!",
            odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
        return;
    }
    
    // Broadcast the transform
    tf_broadcaster_->sendTransform(odom_tf);

    // DIAGNOSTIC: Manual throttled log (10s interval) - verify TF publishing
    static auto last_tf_pub_log = std::chrono::steady_clock::now();
    auto log_now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_tf_pub_log).count() >= 10) {
        RCLCPP_INFO(this->get_logger(), "✓ Publishing TF %s→%s (localization needs this!)",
            odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str());
        last_tf_pub_log = log_now;
    }

    // Periodic connectivity validation
    // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 15000,
    //     "Odometry TF published: %s → %s (stamp: %ld.%ld)", 
    //     odom_tf.header.frame_id.c_str(), odom_tf.child_frame_id.c_str(),
    //     odom_tf.header.stamp.sec, odom_tf.header.stamp.nanosec);
>>>>>>> main
}

void VehicleNodeBase::publish_static_transforms()
{
    if (!static_tf_msg_vec_.empty()) {
        auto now = this->get_clock()->now();
<<<<<<< HEAD
        for (auto& static_tf_msg : static_tf_msg_vec_) {
            static_tf_msg.header.stamp = now;
            static_tf_pub_->sendTransform(static_tf_msg);
        }
=======

        // DISABLED THROTTLE: ROS2 Humble bug - static vars cause thread corruption in MultiThreadedExecutor
        // This function runs at 50Hz in state_timer_callback - logging disabled for thread safety
        // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 15000,
        //     "Publishing %zu static transforms for vehicle: %s",
        //     static_tf_msg_vec_.size(), vehicle_name_.c_str());

        int valid_transforms = 0;
        int invalid_transforms = 0;

        for (auto& static_tf_msg : static_tf_msg_vec_) {
            // CRITICAL: Validate frame IDs before publishing
            if (static_tf_msg.header.frame_id.empty()) {
                // DISABLED THROTTLE: ROS2 Humble bug - using regular ERROR for thread safety
                RCLCPP_ERROR(this->get_logger(),
                    "FAIL SKIPPING transform with empty frame_id → child_frame_id='%s' for vehicle: %s",
                    static_tf_msg.child_frame_id.c_str(), vehicle_name_.c_str());
                invalid_transforms++;
                continue;
            }
            
            if (static_tf_msg.child_frame_id.empty()) {
                // DISABLED THROTTLE: ROS2 Humble bug - using regular ERROR for thread safety
                RCLCPP_ERROR(this->get_logger(),
                    "FAIL SKIPPING transform with empty child_frame_id from frame_id='%s' for vehicle: %s",
                    static_tf_msg.header.frame_id.c_str(), vehicle_name_.c_str());
                invalid_transforms++;
                continue;
            }

            if (static_tf_msg.header.frame_id == static_tf_msg.child_frame_id) {
                // DISABLED THROTTLE: ROS2 Humble bug - using regular ERROR for thread safety
                RCLCPP_ERROR(this->get_logger(),
                    "FAIL SKIPPING self-transform '%s' → '%s' for vehicle: %s",
                    static_tf_msg.header.frame_id.c_str(), static_tf_msg.child_frame_id.c_str(), vehicle_name_.c_str());
                invalid_transforms++;
                continue;
            }
            
            // Update timestamp and publish valid transform
            static_tf_msg.header.stamp = now;
            static_tf_pub_->sendTransform(static_tf_msg);
            valid_transforms++;

            // DISABLED THROTTLE: ROS2 Humble bug - debug logging disabled for thread safety
            // Log each valid static transform being published
            // // DISABLED THROTTLE: ROS2 Humble threading bug
 // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            //     "PASS Static TF: %s → %s [%.2f, %.2f, %.2f]",
            //     static_tf_msg.header.frame_id.c_str(),
            //     static_tf_msg.child_frame_id.c_str(),
            //     static_tf_msg.transform.translation.x,
            //     static_tf_msg.transform.translation.y,
            //     static_tf_msg.transform.translation.z);
        }

        if (invalid_transforms > 0) {
            // DISABLED THROTTLE: ROS2 Humble bug - using regular ERROR for thread safety
            RCLCPP_ERROR(this->get_logger(),
                "WARNING Transform validation: %d valid, %d invalid (skipped) for vehicle: %s",
                valid_transforms, invalid_transforms, vehicle_name_.c_str());
        }
        
    } else {
        RCLCPP_DEBUG_ONCE(this->get_logger(), 
            "No static transforms to publish for vehicle: %s", vehicle_name_.c_str());
>>>>>>> main
    }
}

bool VehicleNodeBase::reset_callback(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
                                    std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
{
    (void)request;  // Suppress unused parameter warning
    
    try {
        // Need to use main client for reset (not specialized clients)
<<<<<<< HEAD
        if (airsim_client_) {
            airsim_client_->reset();
=======
        if (state_client_) {
            state_client_->reset();
>>>>>>> main
        }
        RCLCPP_INFO(this->get_logger(), "Reset successful for vehicle: %s", vehicle_name_.c_str());
        response->success = true;
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "reset");
        response->success = false;
        return false;
    }
}

tf2::Quaternion VehicleNodeBase::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr VehicleNodeBase::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z);
}

void VehicleNodeBase::handle_rpc_error(const rpc::rpc_error& e, const std::string& context) 
{
    try {
        rpc::rpc_error& non_const_e = const_cast<rpc::rpc_error&>(e);
        std::string msg = non_const_e.get_error().as<std::string>();
        RCLCPP_ERROR(this->get_logger(), "RPC error in %s for vehicle %s: %s", 
                     context.c_str(), vehicle_name_.c_str(), msg.c_str());
    }
    catch (...) {
        RCLCPP_ERROR(this->get_logger(), "RPC error in %s for vehicle %s: Unable to extract error message", 
                     context.c_str(), vehicle_name_.c_str());
    }
}

// Default implementations for virtual methods (can be overridden by derived classes)
void VehicleNodeBase::initialize_vehicle_client() 
{
<<<<<<< HEAD
    // Default implementation - create basic client
    airsim_client_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
}

void VehicleNodeBase::setup_vehicle_publishers() 
{
    // Default implementation - no additional publishers
}

void VehicleNodeBase::setup_vehicle_subscribers() 
{
    // Default implementation - no subscribers
}
=======
    // Single client architecture - create only basic client by default
    // Derived classes override this to create specialized clients (MultirotorRpcLibClient, etc.)
    sensor_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
    sensor_client_->confirmConnection();
    
    // Low-latency state client (IMU, GPS, barometer)  
    state_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
    state_client_->confirmConnection();
    
    RCLCPP_INFO(this->get_logger(), 
        "Initialized single RPC client for %s (optimized architecture - no redundant connections)", 
        vehicle_name_.c_str());

    establish_connections();
}


>>>>>>> main

void VehicleNodeBase::setup_vehicle_services() 
{
    // Default implementation - no additional services
}

<<<<<<< HEAD
void VehicleNodeBase::update_vehicle_state() 
{
    // Default implementation - basic state update
    RCLCPP_DEBUG(this->get_logger(), "Updating state for vehicle: %s", vehicle_name_.c_str());
=======
void VehicleNodeBase::setup_vehicle_subscribers()
{
    // Default implementation - no additional subscribers
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle subscribers for: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::update_vehicle_state() 
{
    // Default implementation - should be overridden by derived classes
    RCLCPP_DEBUG(this->get_logger(), "Base class update_vehicle_state() called for vehicle: %s", vehicle_name_.c_str());
    
    // Derived classes should override this method completely for vehicle-specific behavior
    // This base implementation only handles basic environment data if needed
    
    try {
        if (state_client_) {
            // Get environment data (common to all vehicle types)
            auto env_data = state_client_->simGetGroundTruthEnvironment(vehicle_name_);
            env_msg_.header.stamp = this->get_clock()->now();
            env_msg_.header.frame_id = vehicle_name_;
            env_msg_.position.x = env_data.position.x();
            env_msg_.position.y = env_data.position.y();
            env_msg_.position.z = env_data.position.z();
            env_msg_.air_pressure = env_data.air_pressure;
            env_msg_.temperature = env_data.temperature;
            env_msg_.air_density = env_data.air_density;
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "base vehicle state update");
    }
}

void VehicleNodeBase::process_state_changes()
{
    // Default implementation - empty hook for state change processing
    // Override in derived classes (e.g., MissionMultirotorNode) to implement:
    // - Mission event detection and logging
    // - State change monitoring
    // - Custom activity tracking
    RCLCPP_DEBUG(this->get_logger(), "Base class process_state_changes() - no processing implemented");
>>>>>>> main
}

void VehicleNodeBase::publish_vehicle_state() 
{
    // Default implementation - basic state publishing
    RCLCPP_DEBUG(this->get_logger(), "Publishing state for vehicle: %s", vehicle_name_.c_str());
<<<<<<< HEAD
}

void VehicleNodeBase::handle_vehicle_commands() 
=======
    // Publish odometry
    odom_pub_->publish(curr_odom_);
    publish_odometry_tf(curr_odom_);
    
    // Publish GPS
    gps_pub_->publish(gps_sensor_msg_);
    
    // Publish environment
    env_pub_->publish(env_msg_);
    
    // Publish settings-driven sensor data (unified approach)
    publish_sensor_data();
}

void VehicleNodeBase::process_vehicle_commands() 
>>>>>>> main
{
    // Default implementation - no command handling
}

<<<<<<< HEAD
void VehicleNodeBase::process_images() 
{
    // Default implementation - no image processing
}

void VehicleNodeBase::process_lidar() 
{
    // Default implementation - no lidar processing
}
=======
>>>>>>> main

void VehicleNodeBase::process_gpulidar() 
{
    // Default implementation - no GPU lidar processing
}

void VehicleNodeBase::process_echo() 
{
    // Default implementation - no echo processing
<<<<<<< HEAD
}
=======
}

// ===== UTILITY METHODS FOR SENSOR DATA CONVERSION =====

sensor_msgs::msg::Imu VehicleNodeBase::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    
    // Convert orientation (AirSim NED to ROS ENU)
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = -imu_data.orientation.y();
    imu_msg.orientation.z = -imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();
    
    // Convert angular velocity (AirSim NED to ROS ENU)
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = -imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = -imu_data.angular_velocity.z();
    
    // Convert linear acceleration (AirSim NED to ROS ENU)
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = -imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z();
    
    return imu_msg;
}

sensor_msgs::msg::MagneticField VehicleNodeBase::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField mag_msg;
    
    // Convert magnetic field (AirSim body frame to ROS ENU)
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    
    return mag_msg;
}

airsim_interfaces::msg::Altimeter VehicleNodeBase::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& baro_data) const
{
    airsim_interfaces::msg::Altimeter alt_msg;
    alt_msg.altitude = baro_data.altitude;
    alt_msg.pressure = baro_data.pressure;
    alt_msg.qnh = baro_data.qnh;
    return alt_msg;
}

sensor_msgs::msg::Range VehicleNodeBase::get_range_msg_from_airsim(const msr::airlib::DistanceSensorData& distance_data) const
{
    sensor_msgs::msg::Range range_msg;
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;  // or INFRARED depending on sensor
    range_msg.field_of_view = 0.1;  // Set based on sensor specifications
    range_msg.min_range = 0.02;     // Set based on sensor specifications  
    range_msg.max_range = 10.0;     // Set based on sensor specifications
    range_msg.range = distance_data.distance;
    return range_msg;
}

// ===== SETTINGS-DRIVEN CONFIGURATION (moved from multirotor_node) =====

// ===== SENSOR PROCESSING FUNCTIONS (moved from multirotor_node) =====

void VehicleNodeBase::publish_sensor_data()
{
    try {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Sensor publisher counts: IMU=%zu, Magnetometer=%zu, Barometer=%zu", 
            imu_pubs_.size(), magnetometer_pubs_.size(), barometer_pubs_.size());
        
        // Publish IMU data from settings-driven sensors (wrapper approach: dedicated sensor client)
        for (const auto& imu_publisher : imu_pubs_) {
            try {
                // Use empty string for default/fallback sensors (AirSim expects "" for default sensors)
                std::string sensor_name_for_airsim = imu_publisher.sensor_name;
                if (sensor_name_for_airsim == "Imu") {
                    sensor_name_for_airsim = "";  // Default IMU sensor
                }
                
                auto imu_data = sensor_client_->getImuData(sensor_name_for_airsim, vehicle_name_);
                sensor_msgs::msg::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
                std::string topic_prefix = vehicle_name_ + "/";
                // REP 105: Use {vehicle}/sensor_link naming convention
                imu_msg.header.frame_id = topic_prefix + imu_publisher.sensor_name + "_link";
                imu_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);  // Use ROS time for TF synchronization
                imu_publisher.publisher->publish(imu_msg);

                // DIAGNOSTIC: Manual throttled log (10s interval) - verify IMU publishing
                static auto last_imu_pub_log = std::chrono::steady_clock::now();
                auto log_now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_imu_pub_log).count() >= 10) {
                    RCLCPP_INFO(this->get_logger(), "✓ Publishing IMU data for sensor: %s",
                        imu_publisher.sensor_name.c_str());
                    last_imu_pub_log = log_now;
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error publishing IMU data for sensor %s: %s", 
                    imu_publisher.sensor_name.c_str(), e.what());
            }
        }
        
        // Publish magnetometer data (wrapper approach: dedicated sensor client)
        for (const auto& mag_publisher : magnetometer_pubs_) {
            try {
                // Use empty string for default/fallback sensors (AirSim expects "" for default sensors)
                std::string sensor_name_for_airsim = mag_publisher.sensor_name;
                if (sensor_name_for_airsim == "Magnetometer") {
                    sensor_name_for_airsim = "";  // Default magnetometer sensor
                }
                
                
                auto mag_data = state_client_->getMagnetometerData(sensor_name_for_airsim, vehicle_name_);
                sensor_msgs::msg::MagneticField mag_msg = get_mag_msg_from_airsim(mag_data);
                // REP 105: Use {vehicle}/sensor_link naming convention  
                mag_msg.header.frame_id = vehicle_name_ + "/" + mag_publisher.sensor_name + "_link";
                mag_msg.header.stamp = rclcpp::Time(mag_data.time_stamp);
                mag_publisher.publisher->publish(mag_msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error publishing magnetometer data for sensor %s: %s", 
                    mag_publisher.sensor_name.c_str(), e.what());
            }
        }
        
        // Publish barometer data (wrapper approach: dedicated sensor client)
        for (const auto& baro_publisher : barometer_pubs_) {
            try {
                // Use empty string for default/fallback sensors (AirSim expects "" for default sensors)
                std::string sensor_name_for_airsim = baro_publisher.sensor_name;
                if (sensor_name_for_airsim == "Barometer") {
                    sensor_name_for_airsim = "";  // Default barometer sensor
                }
                
                auto baro_data = state_client_->getBarometerData(sensor_name_for_airsim, vehicle_name_);
                airsim_interfaces::msg::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
                // REP 105: Use {vehicle}/sensor_link naming convention
                alt_msg.header.frame_id = vehicle_name_ + "/" + baro_publisher.sensor_name + "_link";
                alt_msg.header.stamp = rclcpp::Time(baro_data.time_stamp);
                baro_publisher.publisher->publish(alt_msg);
            } catch (const std::exception& e) {
                // DISABLED THROTTLE: ROS2 Humble threading bug
                RCLCPP_WARN(this->get_logger(),
                    "Error publishing barometer data for sensor %s: %s", 
                    baro_publisher.sensor_name.c_str(), e.what());
            }
        }
        
        // Publish distance sensor data (wrapper approach: dedicated sensor client)
        for (const auto& distance_publisher : distance_pubs_) {
            try {
                auto distance_data = state_client_->getDistanceSensorData(distance_publisher.sensor_name, vehicle_name_);
                sensor_msgs::msg::Range range_msg = get_range_msg_from_airsim(distance_data);
                range_msg.header.frame_id = vehicle_name_ + "/" + distance_publisher.sensor_name + "_link";
                range_msg.header.stamp = rclcpp::Time(distance_data.time_stamp);
                distance_publisher.publisher->publish(range_msg);
            } catch (const std::exception& e) {
                // DISABLED THROTTLE: ROS2 Humble threading bug
                RCLCPP_WARN(this->get_logger(),
                    "Error publishing distance sensor data for sensor %s: %s", 
                    distance_publisher.sensor_name.c_str(), e.what());
            }
        }
        
    } catch (const std::exception& e) {
        // DISABLED THROTTLE: ROS2 Humble threading bug
        RCLCPP_ERROR(this->get_logger(),
            "Error in sensor data publishing for vehicle %s: %s", vehicle_name_.c_str(), e.what());
    }
}

// Helper function to transform point cloud from AirSim to ROS2 coordinates
// Matches the working implementation in airsim_ros_wrapper.cpp
void VehicleNodeBase::fixPointCloud(std::vector<float>& data, int offset, std::vector<int> flip_indexes) {
    for (size_t i = 1; i < data.size(); i += offset) {
        data[i] = -data[i];  // Negate Y coordinate
        for (int flip_index : flip_indexes) {
            if (i + flip_index < data.size()) {
                data[i + flip_index] = -data[i + flip_index];
            }
        }
    }
}

void VehicleNodeBase::process_settings_driven_lidar()
{
    // Check if any LiDAR publishers exist
    if (lidar_pubs_.empty()) {
        // DISABLED THROTTLE: ROS2 Humble threading bug
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            // "No LiDAR publishers found for vehicle %s - check settings.json sensor configuration", 
            // vehicle_name_.c_str());
        return;
    }
    
    // DISABLED THROTTLE: ROS2 Humble threading bug
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 8000,
        // "[VEHICLE BASE NODE] LIDAR PROCESSING ACTIVE: Found %zu LiDAR publishers for vehicle %s", 
        // lidar_pubs_.size(), vehicle_name_.c_str());
    
    // DISABLED THROTTLE: ROS2 Humble threading bug
    // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
        // "[VEHICLE BASE NODE] Processing %zu LiDAR sensors for vehicle: %s", lidar_pubs_.size(), vehicle_name_.c_str());
    
    // Wrapper's superior caching approach: avoid duplicate getLidarData() calls
    // Cache all LiDAR data first for sensors that need both point cloud and labels
    // std::unordered_map<std::string, msr::airlib::LidarData> sensor_names_to_lidar_data_map;
    
    // Process LiDAR point cloud publishers (using virtual method for client access)
    for (const auto& lidar_publisher : lidar_pubs_) {
        try {            
            // Use virtual method to get LiDAR data (derived class provides client access)
            auto lidar_data = get_lidar_data_for_sensor(lidar_publisher.sensor_name, vehicle_name_);
            // sensor_names_to_lidar_data_map[lidar_publisher.sensor_name] = lidar_data;
            
            // DISABLED THROTTLE: ROS2 Humble threading bug
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                // "RECEIVED LiDAR data for %s: %zu points, timestamp: %lu", 
                // lidar_publisher.sensor_name.c_str(), lidar_data.point_cloud.size(), 
                // lidar_data.time_stamp);
            
            if (lidar_data.point_cloud.size() > 3) {
                sensor_msgs::msg::PointCloud2 lidar_msg;
                lidar_msg.header.stamp = this->get_clock()->now();
                std::string topic_prefix = vehicle_name_ + "/";
                // REP 105: Use {vehicle}/sensor_link naming convention for LiDAR (FIXED: added _link suffix)
                lidar_msg.header.frame_id = topic_prefix + lidar_publisher.sensor_name + "_link";
                
                // Configure point cloud message
                lidar_msg.width = lidar_data.point_cloud.size() / 3;
                lidar_msg.height = 1;
                lidar_msg.is_dense = true;
                lidar_msg.is_bigendian = false;
                
                // Set up fields for XYZ point cloud (wrapper's cleaner loop approach)
                lidar_msg.fields.resize(3);
                lidar_msg.fields[0].name = "x";
                lidar_msg.fields[1].name = "y";  
                lidar_msg.fields[2].name = "z";
                
                int offset = 0;
                for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
                    lidar_msg.fields[d].offset = offset;
                    lidar_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    lidar_msg.fields[d].count = 1;
                }
                
                lidar_msg.point_step = offset; // 4 * num fields (more flexible than hardcoded 12)
                lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;
                
                // Copy and transform the data (AirSim NED to ROS ENU)
                std::vector<float> data_std = lidar_data.point_cloud;
                fixPointCloud(data_std, 3, {1}); // Flip Y coordinate
                
                // Convert filtered data to byte array
                const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
                std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
                lidar_msg.data = std::move(lidar_msg_data);
                
                // Add debug info for data quality
                int valid_point_count = lidar_msg.width;  // Use the actual number of points published
                // DISABLED THROTTLE: ROS2 Humble threading bug
                // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    // "LiDAR %s: %d/%zu valid points (%.1f%% valid)", 
                    // lidar_publisher.sensor_name.c_str(), valid_point_count, lidar_data.point_cloud.size() / 3,
                    // valid_point_count > 0 ? (100.0 * valid_point_count) / (lidar_data.point_cloud.size() / 3) : 0.0);
                lidar_publisher.publisher->publish(lidar_msg);
                
                RCLCPP_INFO(this->get_logger(),
                    "PUBLISHED LiDAR data for sensor: %s (%u points) on topic: %s", 
                    lidar_publisher.sensor_name.c_str(), lidar_msg.width,
                    lidar_publisher.publisher->get_topic_name());
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "INSUFFICIENT POINTS! LiDAR sensor %s has only %zu points (need >3), not publishing", 
                    lidar_publisher.sensor_name.c_str(), lidar_data.point_cloud.size());
            }
            
        } catch (const rpc::rpc_error& e) {
            // DISABLED THROTTLE: ROS2 Humble threading bug
            RCLCPP_WARN(this->get_logger(),
                "LiDAR sensor %s not available: %s", 
                lidar_publisher.sensor_name.c_str(), e.what());
        } catch (const std::exception& e) {
            // DISABLED THROTTLE: ROS2 Humble threading bug
            RCLCPP_ERROR(this->get_logger(),
                "Error processing LiDAR sensor %s: %s", 
                lidar_publisher.sensor_name.c_str(), e.what());
        }
    }
}

void VehicleNodeBase::create_vehicle_map_to_odom_transform()
{
    // WARNING  DEPRECATED - REP 105 FRAME AUTHORITIES VIOLATION WARNING 
    // This function creates a STATIC map→odom transform, which violates REP 105 Frame Authorities
    // REP 105 requires: "localization component...uses this information to broadcast the transform from map to odom"
    // The map→odom transform MUST be published DYNAMICALLY by a localization component, not statically!
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = this->get_clock()->now();
    map_to_odom_tf.header.frame_id = map_frame_id_;                   // "map" - Standard global frame
    map_to_odom_tf.child_frame_id = odom_frame_id_;                   // "drone_1/odom" - clean namespace structure
    
    // Identity transform - vehicle starts at map origin (can be updated by localization)
    // In a real system, this transform would be managed by a localization/SLAM node
    map_to_odom_tf.transform.translation.x = 0.0;
    map_to_odom_tf.transform.translation.y = 0.0;
    map_to_odom_tf.transform.translation.z = 0.0;
    map_to_odom_tf.transform.rotation.x = 0.0;
    map_to_odom_tf.transform.rotation.y = 0.0;
    map_to_odom_tf.transform.rotation.z = 0.0;
    map_to_odom_tf.transform.rotation.w = 1.0;
    
    // Add to static transform vector for publishing
    static_tf_msg_vec_.emplace_back(map_to_odom_tf);
    
    RCLCPP_INFO(this->get_logger(), 
        "TRANSFORM: %s → %s (standard ROS structure)",
        map_to_odom_tf.header.frame_id.c_str(), 
        map_to_odom_tf.child_frame_id.c_str());
        
    RCLCPP_INFO(this->get_logger(),
        "Vehicle %s ready for multi-robot operation with standard transform tree", 
        vehicle_name_.c_str());
}

geometry_msgs::msg::Transform VehicleNodeBase::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, 
                                                                              const msr::airlib::Quaternionr& orientation)
{
    geometry_msgs::msg::Transform transform_msg;
    
    transform_msg.translation.x = position.x();
    transform_msg.translation.y = position.y();
    transform_msg.translation.z = position.z();
    
    transform_msg.rotation.w = orientation.w();
    transform_msg.rotation.x = orientation.x();
    transform_msg.rotation.y = orientation.y();
    transform_msg.rotation.z = orientation.z();
    
    return transform_msg;
}

void VehicleNodeBase::convert_tf_msg_to_ros(geometry_msgs::msg::TransformStamped& tf_msg)
{
    // Convert AirSim NED coordinate system to ROS ENU coordinate system
    tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
    tf_msg.transform.translation.y = -tf_msg.transform.translation.y;
    tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
    tf_msg.transform.rotation.y = -tf_msg.transform.rotation.y;
}

// Placeholder implementations for additional transform functions
void VehicleNodeBase::append_static_sensor_tf(const std::string& sensor_name, 
                                               const msr::airlib::Pose& sensor_pose,
                                               const std::string& frame_suffix)
{
    geometry_msgs::msg::TransformStamped sensor_tf_msg;
    sensor_tf_msg.header.stamp = this->get_clock()->now();
    sensor_tf_msg.header.frame_id = base_link_frame_id_;
    sensor_tf_msg.child_frame_id = vehicle_name_ + "_" + sensor_name + frame_suffix;
    
    sensor_tf_msg.transform = get_transform_msg_from_airsim(sensor_pose.position, sensor_pose.orientation);
    convert_tf_msg_to_ros(sensor_tf_msg);
    
    static_tf_msg_vec_.emplace_back(sensor_tf_msg);
    
    RCLCPP_INFO(this->get_logger(), 
        "Created static TF for sensor %s: %s -> %s",
        sensor_name.c_str(), 
        sensor_tf_msg.header.frame_id.c_str(), 
        sensor_tf_msg.child_frame_id.c_str());
}

void VehicleNodeBase::append_static_camera_tf(const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    std::string topic_prefix = vehicle_name_ + "/";
    if(camera_setting.external)
        static_cam_tf_body_msg.header.frame_id = map_frame_id_;  // External cameras connect to map
    else
        static_cam_tf_body_msg.header.frame_id = base_link_frame_id_;  // Use REP 105 base_link
    static_cam_tf_body_msg.child_frame_id = topic_prefix + camera_name + "_body";

    auto camera_info_data = sensor_client_->simGetCameraInfo(camera_name, vehicle_name_);
    static_cam_tf_body_msg.transform = get_transform_msg_from_airsim(camera_info_data.pose.position, camera_info_data.pose.orientation);

    convert_tf_msg_to_ros(static_cam_tf_body_msg);

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    if(camera_setting.external)
        static_cam_tf_optical_msg.header.frame_id = map_frame_id_;  // External cameras connect to map
    else
        static_cam_tf_optical_msg.header.frame_id = base_link_frame_id_;  // Use REP 105 base_link
    static_cam_tf_optical_msg.child_frame_id = topic_prefix + camera_name + "_optical";
    static_cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(static_cam_tf_body_msg.transform);

    static_tf_msg_vec_.emplace_back(static_cam_tf_body_msg);
    static_tf_msg_vec_.emplace_back(static_cam_tf_optical_msg);
}

void VehicleNodeBase::append_static_camera_tf_basic(const std::string& camera_name, const CameraSetting& camera_setting)
{
    // Basic camera TF creation without requiring AirSim client (uses default positions)
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.stamp = this->get_clock()->now();
    std::string topic_prefix = vehicle_name_ + "/";
    
    if(camera_setting.external)
        static_cam_tf_body_msg.header.frame_id = map_frame_id_;
    else
        static_cam_tf_body_msg.header.frame_id = base_link_frame_id_;  // Use REP 105 base_link
    
    static_cam_tf_body_msg.child_frame_id = topic_prefix + camera_name + "_body";
    
    // Use camera setting position and rotation (defaults for most cameras)
    static_cam_tf_body_msg.transform = get_transform_msg_from_airsim(camera_setting.position, 
        msr::airlib::VectorMath::toQuaternion(camera_setting.rotation.pitch, camera_setting.rotation.roll, camera_setting.rotation.yaw));
    
    convert_tf_msg_to_ros(static_cam_tf_body_msg);

    // Create optical frame transform (standard camera convention: x=right, y=down, z=forward)
    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = topic_prefix + camera_name + "_optical";
    static_cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(static_cam_tf_body_msg.transform);

    static_tf_msg_vec_.emplace_back(static_cam_tf_body_msg);
    static_tf_msg_vec_.emplace_back(static_cam_tf_optical_msg);
    
    RCLCPP_INFO(this->get_logger(), 
        "Created basic static TF for camera %s: %s -> %s, %s", 
        camera_name.c_str(),
        static_cam_tf_body_msg.header.frame_id.c_str(), 
        static_cam_tf_body_msg.child_frame_id.c_str(),
        static_cam_tf_optical_msg.child_frame_id.c_str());
}

void VehicleNodeBase::append_static_lidar_tf(const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_params)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.stamp = this->get_clock()->now();
    std::string topic_prefix = vehicle_name_ + "/";
    
    // REP 105: Connect to proper base_link frame 
    if(lidar_params.external)
        lidar_tf_msg.header.frame_id = "map";  // External sensors connect to global map frame
    else
        lidar_tf_msg.header.frame_id = base_link_frame_id_;  // REP 105 standard base_link
    
    // REP 105: Use namespace sensor_link convention 
    lidar_tf_msg.child_frame_id = topic_prefix + lidar_name + "_link";
    
    // Get LiDAR pose from AirSim using virtual method
    try {
        auto lidar_data = get_lidar_data_for_sensor(lidar_name, vehicle_name_);
        lidar_tf_msg.transform = get_transform_msg_from_airsim(lidar_data.pose.position, lidar_data.pose.orientation);
        convert_tf_msg_to_ros(lidar_tf_msg);
        
        static_tf_msg_vec_.emplace_back(lidar_tf_msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "Created static TF for LiDAR %s: %s -> %s",
            lidar_name.c_str(), 
            lidar_tf_msg.header.frame_id.c_str(), 
            lidar_tf_msg.child_frame_id.c_str());
            
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), 
            "Failed to get LiDAR pose from AirSim for sensor %s: %s. Using identity transform.", 
            lidar_name.c_str(), e.what());
        
        // Create basic transform with identity pose when AirSim data unavailable
        append_static_lidar_tf_basic(lidar_name, lidar_params);
    }
}

void VehicleNodeBase::append_static_lidar_tf_basic(const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_params)
{
    // Basic LiDAR TF creation without requiring AirSim client (uses identity transform)
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.stamp = this->get_clock()->now();
    std::string topic_prefix = vehicle_name_ + "/";
    // Connect to proper frame chain
    if(lidar_params.external)
        lidar_tf_msg.header.frame_id = map_frame_id_;
    else
        lidar_tf_msg.header.frame_id = base_link_frame_id_;  // Connect to base_link
    
    lidar_tf_msg.child_frame_id = topic_prefix + lidar_name + "_link";
    
    // Use identity transform as fallback (LiDAR positioned at vehicle origin)
    // Note: LidarSimpleParams contains sensor configuration, not pose information
    lidar_tf_msg.transform.translation.x = 0.0;
    lidar_tf_msg.transform.translation.y = 0.0;
    lidar_tf_msg.transform.translation.z = 0.0;
    lidar_tf_msg.transform.rotation.x = 0.0;
    lidar_tf_msg.transform.rotation.y = 0.0;
    lidar_tf_msg.transform.rotation.z = 0.0;
    lidar_tf_msg.transform.rotation.w = 1.0;
    
    static_tf_msg_vec_.emplace_back(lidar_tf_msg);
    
    RCLCPP_INFO(this->get_logger(), 
        "Created basic static TF for LiDAR %s: %s -> %s (using identity transform)", 
        lidar_name.c_str(),
        lidar_tf_msg.header.frame_id.c_str(), 
        lidar_tf_msg.child_frame_id.c_str());
}

void VehicleNodeBase::append_static_gpulidar_tf(const std::string& gpulidar_name, const msr::airlib::GPULidarSimpleParams& gpulidar_params)
{
    geometry_msgs::msg::TransformStamped gpulidar_tf_msg;
    if(gpulidar_params.external)
        gpulidar_tf_msg.header.frame_id = map_frame_id_;
    else
        gpulidar_tf_msg.header.frame_id = base_link_frame_id_;
    std::string topic_prefix = vehicle_name_ + "/";
    gpulidar_tf_msg.child_frame_id = topic_prefix + gpulidar_name + "_link";
    gpulidar_tf_msg.header.stamp = this->get_clock()->now();
    
    // Get GPU LiDAR pose from AirSim
    try {
        auto gpulidar_data = sensor_client_->getGPULidarData(gpulidar_name, vehicle_name_);
        gpulidar_tf_msg.transform = get_transform_msg_from_airsim(gpulidar_data.pose.position, gpulidar_data.pose.orientation);
        convert_tf_msg_to_ros(gpulidar_tf_msg);
        
        static_tf_msg_vec_.emplace_back(gpulidar_tf_msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "Created static TF for GPU LiDAR %s: %s -> %s",
            gpulidar_name.c_str(), 
            gpulidar_tf_msg.header.frame_id.c_str(), 
            gpulidar_tf_msg.child_frame_id.c_str());
            
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), 
            "Failed to create static TF for GPU LiDAR sensor %s: %s", 
            gpulidar_name.c_str(), e.what());
    }
}

void VehicleNodeBase::append_static_echo_tf(const std::string& echo_name, const msr::airlib::EchoSimpleParams& echo_params)
{
    geometry_msgs::msg::TransformStamped echo_tf_msg;
    if(echo_params.external)
        echo_tf_msg.header.frame_id = map_frame_id_;
    else
        echo_tf_msg.header.frame_id = base_link_frame_id_;
    std::string topic_prefix = vehicle_name_ + "/";
    echo_tf_msg.child_frame_id = topic_prefix + echo_name + "_link";
    echo_tf_msg.header.stamp = this->get_clock()->now();
    
    // Get Echo pose from AirSim
    try {
        auto echo_data = sensor_client_->getEchoData(echo_name, vehicle_name_);
        echo_tf_msg.transform = get_transform_msg_from_airsim(echo_data.pose.position, echo_data.pose.orientation);
        convert_tf_msg_to_ros(echo_tf_msg);
        
        static_tf_msg_vec_.emplace_back(echo_tf_msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "Created static TF for Echo sensor %s: %s -> %s",
            echo_name.c_str(), 
            echo_tf_msg.header.frame_id.c_str(), 
            echo_tf_msg.child_frame_id.c_str());
            
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), 
            "Failed to create static TF for Echo sensor %s: %s", 
            echo_name.c_str(), e.what());
    }
}



// Update the base class process functions to call the common implementations
void VehicleNodeBase::process_lidar()
{
    try {
        // Process settings-driven LiDAR sensors (preferred approach)
        process_settings_driven_lidar();
        
        
    } catch (const std::exception& e) {
        // DISABLED THROTTLE: ROS2 Humble threading bug
        RCLCPP_ERROR(this->get_logger(), 
            "Error in LiDAR processing for vehicle %s: %s", vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::process_images()
{
    // Skip if no cameras are configured
    if (camera_request_vehicle_pairs_.empty() || image_pubs_.empty()) {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "No cameras configured for vehicle: %s", vehicle_name_.c_str());
        return;
    }

    // PERFORMANCE INSTRUMENTATION: Start timing overall function
    auto function_start_time = std::chrono::steady_clock::now();

    try {
        // Process camera requests for this specific vehicle
        size_t image_index = 0;
        
        for (const auto& request_pair : camera_request_vehicle_pairs_) {
            const auto& image_requests = request_pair.first;
            const auto& vehicle_name = request_pair.second;
            
            // Only process requests for this vehicle
            if (vehicle_name != vehicle_name_) {
                image_index += image_requests.size();
                continue;
            }
            
            if (image_requests.empty()) {
                continue;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Processing %zu image requests for vehicle: %s",
                        image_requests.size(), vehicle_name.c_str());

            // PERFORMANCE INSTRUMENTATION: Time request preparation
            auto prep_start_time = std::chrono::steady_clock::now();

            // Convert our ImageRequest objects to AirSim ImageRequest format
            std::vector<msr::airlib::ImageCaptureBase::ImageRequest> airsim_requests;
            for (const auto& req : image_requests) {
                msr::airlib::ImageCaptureBase::ImageRequest airsim_req;
                airsim_req.camera_name = req.camera_name;
                airsim_req.image_type = req.image_type;
                airsim_req.pixels_as_float = req.pixels_as_float;
                airsim_req.compress = req.compress;
                airsim_req.annotation_name = req.annotation_name;
                airsim_requests.push_back(airsim_req);
            }

            auto prep_end_time = std::chrono::steady_clock::now();
            auto prep_duration = std::chrono::duration_cast<std::chrono::milliseconds>(prep_end_time - prep_start_time);

            // PERFORMANCE INSTRUMENTATION: Time RPC call (CRITICAL BOTTLENECK MEASUREMENT)
            auto rpc_start_time = std::chrono::steady_clock::now();

            // Request images from AirSim - THIS IS THE SUSPECTED BOTTLENECK
            // GLOBAL RPC MUTEX: Protect ALL AirSim RPC calls (GPS, LiDAR, cameras, etc.)
            decltype(sensor_client_->simGetImages(airsim_requests, vehicle_name)) responses;
            {
                std::lock_guard<std::mutex> rpc_lock(airsim_rpc_mutex_);
                responses = sensor_client_->simGetImages(airsim_requests, vehicle_name);
            }

            auto rpc_end_time = std::chrono::steady_clock::now();
            auto rpc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(rpc_end_time - rpc_start_time);

            // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s]: RPC call (%zu cameras) took %ld ms, prep took %ld ms",
            //            vehicle_name.c_str(), airsim_requests.size(), rpc_duration.count(), prep_duration.count());

            // RCLCPP_DEBUG(this->get_logger(), "Received %zu image responses from AirSim", responses.size());

            // PERFORMANCE INSTRUMENTATION: Time image processing and publishing
            // auto processing_start_time = std::chrono::steady_clock::now();
            size_t processed_images = 0;

            // Process each response and publish via modern image_transport system
            for (size_t i = 0; i < responses.size() && image_index < image_pubs_.size(); ++i, ++image_index) {
                const auto& response = responses[i];
                
                if (response.image_data_uint8.empty() && response.image_data_float.empty()) {
                    RCLCPP_DEBUG(this->get_logger(), "Empty image data for camera %s", 
                                airsim_requests[i].camera_name.c_str());
                    continue;
                }
                
                sensor_msgs::msg::Image img_msg;
                img_msg.header.stamp = rclcpp::Time(response.time_stamp);
                img_msg.header.frame_id = airsim_requests[i].camera_name + "_optical";
                img_msg.width = response.width;
                img_msg.height = response.height;
                
                // Handle different image data formats
                if (!response.pixels_as_float && !response.image_data_uint8.empty()) {
                    // Integer image data (RGB, depth visualization, etc.)
                    if (response.image_data_uint8.size() == static_cast<size_t>(response.width * response.height * 3)) {
                        // RGB image - convert to BGR for ROS
                        img_msg.encoding = "bgr8";
                        img_msg.step = response.width * 3;
                        img_msg.data.resize(response.image_data_uint8.size());
                        
                        for (size_t j = 0; j < response.image_data_uint8.size(); j += 3) {
                            img_msg.data[j] = response.image_data_uint8[j + 2];     // B
                            img_msg.data[j + 1] = response.image_data_uint8[j + 1]; // G  
                            img_msg.data[j + 2] = response.image_data_uint8[j];     // R
                        }
                    } else if (response.image_data_uint8.size() == static_cast<size_t>(response.width * response.height * 4)) {
                        // RGBA image
                        img_msg.encoding = "bgra8";
                        img_msg.step = response.width * 4;
                        img_msg.data.resize(response.image_data_uint8.size());
                        
                        for (size_t j = 0; j < response.image_data_uint8.size(); j += 4) {
                            img_msg.data[j] = response.image_data_uint8[j + 2];     // B
                            img_msg.data[j + 1] = response.image_data_uint8[j + 1]; // G  
                            img_msg.data[j + 2] = response.image_data_uint8[j];     // R
                            img_msg.data[j + 3] = response.image_data_uint8[j + 3]; // A
                        }
                    } else {
                        // Single channel or unknown format
                        img_msg.encoding = "mono8";
                        img_msg.step = response.width;
                        img_msg.data = response.image_data_uint8;
                    }
                } else if (response.pixels_as_float && !response.image_data_float.empty()) {
                    // Float image data (depth, disparity, etc.)
                    img_msg.encoding = "32FC1";
                    img_msg.step = response.width * 4;  // 4 bytes per float
                    img_msg.data.resize(response.image_data_float.size() * 4);
                    
                    std::memcpy(img_msg.data.data(), response.image_data_float.data(), 
                               response.image_data_float.size() * 4);
                }
                
                // Publish using modern image_transport system (provides automatic compression)
                image_pubs_[image_index].publish(img_msg);

                processed_images++;

                RCLCPP_DEBUG(this->get_logger(), "Published image from camera %s (%dx%d, %s)",
                            airsim_requests[i].camera_name.c_str(),
                            img_msg.width, img_msg.height, img_msg.encoding.c_str());
            }

            // PERFORMANCE INSTRUMENTATION: Complete processing timing
            // auto processing_end_time = std::chrono::steady_clock::now();
            // auto processing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(processing_end_time - processing_start_time);

            // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s]: Processing %zu images took %ld ms",
            //            vehicle_name.c_str(), processed_images, processing_duration.count());
        }

        // PERFORMANCE INSTRUMENTATION: Overall function timing
        auto function_end_time = std::chrono::steady_clock::now();
        auto function_duration = std::chrono::duration_cast<std::chrono::milliseconds>(function_end_time - function_start_time);

        // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s]: TOTAL process_images() took %ld ms (timer freq: %.1f Hz, actual: %.1f Hz)",
        //            vehicle_name_.c_str(), function_duration.count(), 1.0/image_timer_freq_,
        //            function_duration.count() > 0 ? 1000.0/function_duration.count() : 0.0);

    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "image processing");
    } catch (const std::exception& e) {
        // DISABLED THROTTLE: ROS2 Humble threading bug
        RCLCPP_ERROR(this->get_logger(),
            "Error in image processing for vehicle %s: %s", vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::setup_individual_camera_timers()
{
    // Assign cameras to individual timers for parallel processing
    size_t total_cameras = camera_request_vehicle_pairs_.size();

    if (total_cameras == 0) {
        RCLCPP_INFO(this->get_logger(), "No cameras configured - individual camera timers not needed");
        active_camera_timers_ = 0;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Setting up individual camera timers for %zu cameras", total_cameras);

    // Clear any existing assignments
    camera_1_indices_.clear();
    camera_2_indices_.clear();
    camera_3_indices_.clear();
    camera_4_indices_.clear();

    // Assign cameras to timers (round-robin distribution for optimal load balancing)
    for (size_t i = 0; i < total_cameras; ++i) {
        switch (i % 4) {
            case 0:
                camera_1_indices_.push_back(i);
                break;
            case 1:
                camera_2_indices_.push_back(i);
                break;
            case 2:
                camera_3_indices_.push_back(i);
                break;
            case 3:
                camera_4_indices_.push_back(i);
                break;
        }
    }

    // Create timers only for cameras that have assignments
    active_camera_timers_ = 0;

    if (!camera_1_indices_.empty()) {
        camera_1_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(image_timer_freq_),
            std::bind(&VehicleNodeBase::camera_1_timer_callback, this), camera_1_group_);
        active_camera_timers_++;
        RCLCPP_INFO(this->get_logger(), "Camera 1 timer created for %zu cameras", camera_1_indices_.size());
    }

    if (!camera_2_indices_.empty()) {
        camera_2_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(image_timer_freq_),
            std::bind(&VehicleNodeBase::camera_2_timer_callback, this), camera_2_group_);
        active_camera_timers_++;
        RCLCPP_INFO(this->get_logger(), "Camera 2 timer created for %zu cameras", camera_2_indices_.size());
    }

    if (!camera_3_indices_.empty()) {
        camera_3_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(image_timer_freq_),
            std::bind(&VehicleNodeBase::camera_3_timer_callback, this), camera_3_group_);
        active_camera_timers_++;
        RCLCPP_INFO(this->get_logger(), "Camera 3 timer created for %zu cameras", camera_3_indices_.size());
    }

    if (!camera_4_indices_.empty()) {
        camera_4_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(image_timer_freq_),
            std::bind(&VehicleNodeBase::camera_4_timer_callback, this), camera_4_group_);
        active_camera_timers_++;
        RCLCPP_INFO(this->get_logger(), "Camera 4 timer created for %zu cameras", camera_4_indices_.size());
    }

    RCLCPP_INFO(this->get_logger(), "Individual camera timers setup complete: %zu active timers for parallel processing",
               active_camera_timers_);
}

// Individual camera processing functions for parallel execution
void VehicleNodeBase::process_camera_1()
{
    process_specific_cameras(camera_1_indices_, "Camera1");
}

void VehicleNodeBase::process_camera_2()
{
    process_specific_cameras(camera_2_indices_, "Camera2");
}

void VehicleNodeBase::process_camera_3()
{
    process_specific_cameras(camera_3_indices_, "Camera3");
}

void VehicleNodeBase::process_camera_4()
{
    process_specific_cameras(camera_4_indices_, "Camera4");
}

void VehicleNodeBase::process_specific_cameras(const std::vector<size_t>& camera_indices, const std::string& timer_name)
{
    // Skip if no cameras assigned to this timer
    if (camera_indices.empty()) {
        return;
    }

    // PERFORMANCE INSTRUMENTATION: Start timing this specific timer's execution
    auto function_start_time = std::chrono::steady_clock::now();

    try {
        for (size_t camera_pair_index : camera_indices) {
            // Safety check: ensure index is valid
            if (camera_pair_index >= camera_request_vehicle_pairs_.size()) {
                RCLCPP_ERROR(this->get_logger(), "[%s] Invalid camera index %zu (max: %zu)",
                           timer_name.c_str(), camera_pair_index, camera_request_vehicle_pairs_.size());
                continue;
            }

            const auto& request_pair = camera_request_vehicle_pairs_[camera_pair_index];
            const auto& image_requests = request_pair.first;
            const auto& vehicle_name = request_pair.second;

            // Only process requests for this vehicle
            if (vehicle_name != vehicle_name_) {
                continue;
            }

            if (image_requests.empty()) {
                continue;
            }

            // PERFORMANCE INSTRUMENTATION: Time request preparation
            auto prep_start_time = std::chrono::steady_clock::now();

            // Convert our ImageRequest objects to AirSim ImageRequest format
            std::vector<msr::airlib::ImageCaptureBase::ImageRequest> airsim_requests;
            for (const auto& req : image_requests) {
                msr::airlib::ImageCaptureBase::ImageRequest airsim_req;
                airsim_req.camera_name = req.camera_name;
                airsim_req.image_type = req.image_type;
                airsim_req.pixels_as_float = req.pixels_as_float;
                airsim_req.compress = req.compress;
                airsim_req.annotation_name = req.annotation_name;
                airsim_requests.push_back(airsim_req);
            }

            auto prep_end_time = std::chrono::steady_clock::now();
            auto prep_duration = std::chrono::duration_cast<std::chrono::milliseconds>(prep_end_time - prep_start_time);

            // PERFORMANCE INSTRUMENTATION: Time RPC call (INDIVIDUAL CAMERA MEASUREMENT)
            auto rpc_start_time = std::chrono::steady_clock::now();

            // Request images from AirSim for this specific camera set
            // GLOBAL RPC MUTEX: Protect ALL AirSim RPC calls (GPS, LiDAR, cameras, etc.)
            decltype(sensor_client_->simGetImages(airsim_requests, vehicle_name)) responses;
            {
                std::lock_guard<std::mutex> rpc_lock(airsim_rpc_mutex_);
                responses = sensor_client_->simGetImages(airsim_requests, vehicle_name);
            }

            auto rpc_end_time = std::chrono::steady_clock::now();
            auto rpc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(rpc_end_time - rpc_start_time);

            // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s-%s]: RPC call (%zu cameras) took %ld ms, prep took %ld ms",
            //            vehicle_name.c_str(), timer_name.c_str(), airsim_requests.size(), rpc_duration.count(), prep_duration.count());

            // PERFORMANCE INSTRUMENTATION: Time image processing and publishing
            // auto processing_start_time = std::chrono::steady_clock::now();
            size_t processed_images = 0;

            // Calculate the starting index for publishers (based on camera_pair_index)
            size_t publisher_start_index = 0;
            for (size_t i = 0; i < camera_pair_index; ++i) {
                if (i < camera_request_vehicle_pairs_.size()) {
                    publisher_start_index += camera_request_vehicle_pairs_[i].first.size();
                }
            }

            // Process each response and publish
            for (size_t i = 0; i < responses.size() && (publisher_start_index + i) < image_pubs_.size(); ++i) {
                const auto& response = responses[i];
                size_t pub_index = publisher_start_index + i;

                if (response.image_data_uint8.empty() && response.image_data_float.empty()) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s] Empty image data for camera %s",
                                timer_name.c_str(), airsim_requests[i].camera_name.c_str());
                    continue;
                }

                sensor_msgs::msg::Image img_msg;
                img_msg.header.stamp = rclcpp::Time(response.time_stamp);
                img_msg.header.frame_id = airsim_requests[i].camera_name + "_optical";
                img_msg.width = response.width;
                img_msg.height = response.height;

                // Handle different image data formats (same logic as original process_images)
                if (!response.pixels_as_float && !response.image_data_uint8.empty()) {
                    // Integer image data (RGB, depth visualization, etc.)
                    if (response.image_data_uint8.size() == static_cast<size_t>(response.width * response.height * 3)) {
                        // RGB image - convert to BGR for ROS
                        img_msg.encoding = "bgr8";
                        img_msg.step = response.width * 3;
                        img_msg.data.resize(response.image_data_uint8.size());

                        for (size_t j = 0; j < response.image_data_uint8.size(); j += 3) {
                            img_msg.data[j] = response.image_data_uint8[j + 2];     // B
                            img_msg.data[j + 1] = response.image_data_uint8[j + 1]; // G
                            img_msg.data[j + 2] = response.image_data_uint8[j];     // R
                        }
                    } else if (response.image_data_uint8.size() == static_cast<size_t>(response.width * response.height * 4)) {
                        // RGBA image
                        img_msg.encoding = "bgra8";
                        img_msg.step = response.width * 4;
                        img_msg.data.resize(response.image_data_uint8.size());

                        for (size_t j = 0; j < response.image_data_uint8.size(); j += 4) {
                            img_msg.data[j] = response.image_data_uint8[j + 2];     // B
                            img_msg.data[j + 1] = response.image_data_uint8[j + 1]; // G
                            img_msg.data[j + 2] = response.image_data_uint8[j];     // R
                            img_msg.data[j + 3] = response.image_data_uint8[j + 3]; // A
                        }
                    } else {
                        // Single channel or unknown format
                        img_msg.encoding = "mono8";
                        img_msg.step = response.width;
                        img_msg.data = response.image_data_uint8;
                    }
                } else if (response.pixels_as_float && !response.image_data_float.empty()) {
                    // Float image data (depth, disparity, etc.)
                    img_msg.encoding = "32FC1";
                    img_msg.step = response.width * 4;  // 4 bytes per float
                    img_msg.data.resize(response.image_data_float.size() * 4);

                    std::memcpy(img_msg.data.data(), response.image_data_float.data(),
                               response.image_data_float.size() * 4);
                }

                // Publish using modern image_transport system
                image_pubs_[pub_index].publish(img_msg);
                processed_images++;

                RCLCPP_DEBUG(this->get_logger(), "[%s] Published image from camera %s (%dx%d, %s)",
                            timer_name.c_str(), airsim_requests[i].camera_name.c_str(),
                            img_msg.width, img_msg.height, img_msg.encoding.c_str());
            }

            // PERFORMANCE INSTRUMENTATION: Complete processing timing for this camera set
            // auto processing_end_time = std::chrono::steady_clock::now();
            // auto processing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(processing_end_time - processing_start_time);

            // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s-%s]: Processing %zu images took %ld ms",
            //            vehicle_name.c_str(), timer_name.c_str(), processed_images, processing_duration.count());
        }

        // PERFORMANCE INSTRUMENTATION: Overall timer function timing
        auto function_end_time = std::chrono::steady_clock::now();
        auto function_duration = std::chrono::duration_cast<std::chrono::milliseconds>(function_end_time - function_start_time);

        // RCLCPP_INFO(this->get_logger(), "CAMERA PERFORMANCE [%s-%s]: TOTAL process took %ld ms (timer freq: %.1f Hz, actual: %.1f Hz)",
        //            vehicle_name_.c_str(), timer_name.c_str(), function_duration.count(), 1.0/image_timer_freq_,
        //            function_duration.count() > 0 ? 1000.0/function_duration.count() : 0.0);

    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, timer_name + " processing");
    } catch (const std::exception& e) {
        // DISABLED THROTTLE: ROS2 Humble threading bug
        RCLCPP_ERROR(this->get_logger(),
            "Error in %s processing for vehicle %s: %s", timer_name.c_str(), vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void VehicleNodeBase::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

// Missing function implementation (moved from AirsimROSWrapper)
geometry_msgs::msg::Transform VehicleNodeBase::get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const
{
    geometry_msgs::msg::Transform optical_tf = body_tf;
    
    // Convert from camera body frame to optical frame (ROS convention)
    // Body frame: X forward, Y left, Z up
    // Optical frame: Z forward, X right, Y down
    tf2::Quaternion body_quat(body_tf.rotation.x, body_tf.rotation.y, body_tf.rotation.z, body_tf.rotation.w);
    tf2::Quaternion optical_rotation;
    optical_rotation.setRPY(-M_PI/2, 0, -M_PI/2);  // Rotate to optical frame
    tf2::Quaternion optical_quat = body_quat * optical_rotation;
    
    optical_tf.rotation.x = optical_quat.x();
    optical_tf.rotation.y = optical_quat.y();
    optical_tf.rotation.z = optical_quat.z();
    optical_tf.rotation.w = optical_quat.w();
    
    return optical_tf;
}

// Default implementations of virtual methods
void VehicleNodeBase::setup_vehicle_publishers()
{
    // Default implementation - derived classes can override
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle publishers for: %s", vehicle_name_.c_str());
    
    // Note: Common publishers (odom, GPS, environment) are already created by create_ros_pubs_from_settings_json()
    // This method is for vehicle-specific publishers only
}

// Helper function to find vehicle in settings with name mapping
std::string VehicleNodeBase::findVehicleInSettings(const msr::airlib::Settings& settings_json, const std::string& rpc_vehicle_name)
{
    try {
        // Try to get the Vehicles section
        msr::airlib::Settings vehicles;
        if (!settings_json.getChild("Vehicles", vehicles)) {
            // No Vehicles section found
            return "";
        }
        
        // Get list of vehicle names in settings
        std::vector<std::string> vehicle_names;
        vehicles.getChildNames(vehicle_names);
        
        // First, try direct match
        for (const std::string& settings_name : vehicle_names) {
            if (settings_name == rpc_vehicle_name) {
                return rpc_vehicle_name;
            }
        }
        
        // Create mapping from common RPC names to likely settings.json names
        std::map<std::string, std::vector<std::string>> name_mappings = {
            {"Drone1", {"Drone_1", "Drone1", "drone1", "DRONE1"}},
            {"Drone2", {"D2", "Drone2", "drone2", "DRONE2", "Drone_2"}},
            {"Drone3", {"MyDrone3", "Drone3", "drone3", "DRONE3", "Drone_3"}},
            {"Drone4", {"Drone4", "drone4", "DRONE4", "Drone_4"}},
            {"Drone5", {"Drone5", "drone5", "DRONE5", "Drone_5"}}
        };
        
        // Try mapped names
        if (name_mappings.find(rpc_vehicle_name) != name_mappings.end()) {
            for (const std::string& candidate : name_mappings[rpc_vehicle_name]) {
                for (const std::string& settings_name : vehicle_names) {
                    if (settings_name == candidate) {
                        return candidate;
                    }
                }
            }
        }
        
        // If still not found, try to find any vehicle that contains the RPC name as substring
        for (const std::string& settings_name : vehicle_names) {
            // Check if RPC name is contained in settings name or vice versa
            if (settings_name.find(rpc_vehicle_name) != std::string::npos || 
                rpc_vehicle_name.find(settings_name) != std::string::npos) {
                return settings_name;
            }
        }
        
    } catch (const std::exception& e) {
        // Settings parsing failed - return empty string
        return "";
    }
    
    // No match found
    return "";
}

>>>>>>> main
