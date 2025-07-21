#include "vehicles/VehicleNodeBase.h"
#include <chrono>

VehicleNodeBase::VehicleNodeBase(const std::string& vehicle_name,
                                 const VehicleSetting& vehicle_config,
                                 const std::string& host_ip,
                                 uint16_t host_port,
                                 const std::string& world_frame_id,
                                 bool enable_api_control)
    : Node(vehicle_name + "_node")
    , vehicle_name_(vehicle_name)
    , vehicle_config_(vehicle_config)
    , host_ip_(host_ip)
    , host_port_(host_port)
    , world_frame_id_(world_frame_id)
    , odom_frame_id_(AIRSIM_ODOM_FRAME_ID)
    , enable_api_control_(enable_api_control)
    , has_gimbal_cmd_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing vehicle node: %s", vehicle_name_.c_str());
    
    // Initialize transform broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
    // Set up callback groups for parallel processing
    setup_callback_groups();
    
    // Setup ROS interface
    setup_common_publishers();
    setup_common_services();
}

VehicleNodeBase::~VehicleNodeBase()
{
    RCLCPP_INFO(this->get_logger(), "Destroying vehicle node: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing AirSim connection for vehicle: %s", vehicle_name_.c_str());
    
    // Initialize RPC clients
    initialize_rpc_clients();
    
    // Create vehicle-specific AirSim client
    create_airsim_client();
    
    // Test connection to AirSim
    try {
        airsim_client_->confirmConnection();
        RCLCPP_INFO(this->get_logger(), "Successfully connected to AirSim for vehicle: %s", vehicle_name_.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to AirSim for vehicle %s: %s", 
                     vehicle_name_.c_str(), e.what());
        throw;
    }
    
    // Setup vehicle-specific ROS interface
    setup_vehicle_ros_interface();
    
    // Setup sensor publishers based on vehicle configuration
    setup_sensor_publishers();
    
    // Setup timers for different update rates
    setup_timers();
    
    // Publish static transforms
    update_and_publish_static_transforms();
    
    RCLCPP_INFO(this->get_logger(), "Vehicle node initialization complete: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::setup_callback_groups()
{
    // Create separate callback groups for parallel processing
    state_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    image_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gpulidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    echo_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void VehicleNodeBase::setup_common_publishers()
{
    // Create publishers for common vehicle data
    odom_local_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_local", 10);
    global_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("global_gps", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>("environment", 10);
    instance_segmentation_pub_ = this->create_publisher<airsim_interfaces::msg::InstanceSegmentationList>("instance_segmentation", 10);
    object_transforms_pub_ = this->create_publisher<airsim_interfaces::msg::ObjectTransformsList>("object_transforms", 10);
    
    RCLCPP_DEBUG(this->get_logger(), "Common publishers created for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::setup_common_services()
{
    // Create services for instance segmentation and object transforms
    instance_segmentation_refresh_srvr_ = this->create_service<airsim_interfaces::srv::RefreshInstanceSegmentation>(
        "refresh_instance_segmentation",
        std::bind(&VehicleNodeBase::instance_segmentation_refresh_cb, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    object_transforms_refresh_srvr_ = this->create_service<airsim_interfaces::srv::RefreshObjectTransforms>(
        "refresh_object_transforms",
        std::bind(&VehicleNodeBase::object_transforms_refresh_cb, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Setup gimbal control subscribers
    gimbal_angle_quat_cmd_sub_ = this->create_subscription<airsim_interfaces::msg::GimbalAngleQuatCmd>(
        "gimbal_angle_quat_cmd", 10,
        std::bind(&VehicleNodeBase::gimbal_angle_quat_cmd_cb, this, std::placeholders::_1));
    
    gimbal_angle_euler_cmd_sub_ = this->create_subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>(
        "gimbal_angle_euler_cmd", 10,
        std::bind(&VehicleNodeBase::gimbal_angle_euler_cmd_cb, this, std::placeholders::_1));
    
    RCLCPP_DEBUG(this->get_logger(), "Common services created for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::setup_sensor_publishers()
{
    RCLCPP_DEBUG(this->get_logger(), "Setting up sensor publishers for vehicle: %s", vehicle_name_.c_str());
    
    // Setup sensor publishers based on vehicle configuration
    for (const auto& sensor_setting : vehicle_config_.sensors) {
        const std::string& sensor_name = sensor_setting.first;
        const auto& sensor_config = sensor_setting.second;
        
        if (!sensor_config->enabled) {
            continue;
        }
        
        // Create publishers based on sensor type
        switch (sensor_config->sensor_type) {
            case msr::airlib::SensorBase::SensorType::Imu:
                {
                    auto pub = create_sensor_publisher<sensor_msgs::msg::Imu>(
                        "imu", sensor_name, sensor_config->sensor_type, 
                        sensor_name + "/imu", 10);
                    imu_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Magnetometer:
                {
                    auto pub = create_sensor_publisher<sensor_msgs::msg::MagneticField>(
                        "magnetometer", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/magnetometer", 10);
                    magnetometer_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Gps:
                {
                    auto pub = create_sensor_publisher<sensor_msgs::msg::NavSatFix>(
                        "gps", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/gps", 10);
                    gps_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Barometer:
                {
                    auto pub = create_sensor_publisher<airsim_interfaces::msg::Altimeter>(
                        "barometer", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/altimeter", 10);
                    barometer_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Distance:
                {
                    auto pub = create_sensor_publisher<sensor_msgs::msg::Range>(
                        "distance", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/distance", 10);
                    distance_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Lidar:
                {
                    auto lidar_pub = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "lidar", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/points", 10);
                    lidar_pubs_.push_back(lidar_pub);
                    
                    auto labels_pub = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                        "lidar_labels", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/labels", 10);
                    lidar_labels_pubs_.push_back(labels_pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::GPULidar:
                {
                    auto pub = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "gpulidar", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/points", 10);
                    gpulidar_pubs_.push_back(pub);
                }
                break;
                
            case msr::airlib::SensorBase::SensorType::Echo:
                {
                    auto active_pub = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "echo_active", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/active_points", 10);
                    echo_active_pubs_.push_back(active_pub);
                    
                    auto passive_pub = create_sensor_publisher<sensor_msgs::msg::PointCloud2>(
                        "echo_passive", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/passive_points", 10);
                    echo_passive_pubs_.push_back(passive_pub);
                    
                    auto active_labels_pub = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                        "echo_active_labels", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/active_labels", 10);
                    echo_active_labels_pubs_.push_back(active_labels_pub);
                    
                    auto passive_labels_pub = create_sensor_publisher<airsim_interfaces::msg::StringArray>(
                        "echo_passive_labels", sensor_name, sensor_config->sensor_type,
                        sensor_name + "/passive_labels", 10);
                    echo_passive_labels_pubs_.push_back(passive_labels_pub);
                }
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported sensor type for sensor: %s", sensor_name.c_str());
                break;
        }
    }
    
    // Setup camera publishers
    setup_camera_publishers();
    
    RCLCPP_INFO(this->get_logger(), "Sensor publishers setup complete for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::setup_timers()
{
    // Create timers with different callback groups for parallel processing
    state_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(STATE_TIMER_RATE),
        std::bind(&VehicleNodeBase::state_timer_cb, this),
        state_callback_group_);
    
    image_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(IMAGE_TIMER_RATE),
        std::bind(&VehicleNodeBase::image_timer_cb, this),
        image_callback_group_);
    
    lidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(LIDAR_TIMER_RATE),
        std::bind(&VehicleNodeBase::lidar_timer_cb, this),
        lidar_callback_group_);
    
    gpulidar_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(GPULIDAR_TIMER_RATE),
        std::bind(&VehicleNodeBase::gpulidar_timer_cb, this),
        gpulidar_callback_group_);
    
    echo_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(ECHO_TIMER_RATE),
        std::bind(&VehicleNodeBase::echo_timer_cb, this),
        echo_callback_group_);
    
    RCLCPP_DEBUG(this->get_logger(), "Timers created for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::initialize_rpc_clients()
{
    // Initialize specialized RPC clients for different data types
    airsim_client_images_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_lidar_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_gpulidar_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    airsim_client_echo_ = std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_, host_port_);
    
    RCLCPP_DEBUG(this->get_logger(), "RPC clients initialized for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::state_timer_cb()
{
    try {
        // Update vehicle state from AirSim
        auto current_time = update_state();
        stamp_ = current_time;
        
        // Publish vehicle-specific state
        publish_vehicle_state();
        
        // Update and execute commands
        update_commands();
        
        // Publish odometry transform
        if (odom_local_pub_->get_subscription_count() > 0) {
            publish_odom_tf(curr_odom_);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in state timer callback for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::image_timer_cb()
{
    try {
        // Process camera images for this vehicle only
        if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
            for (const auto& img_request_pair : airsim_img_request_vehicle_name_pair_vec_) {
                const std::vector<ImageResponse> img_response = airsim_client_images_->simGetImages(
                    img_request_pair.first, vehicle_name_);
                
                if (img_response.size() == img_request_pair.first.size()) {
                    process_and_publish_img_response(img_response);
                }
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in image timer callback for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::lidar_timer_cb()
{
    try {
        // Process LiDAR data for this vehicle
        if (!lidar_pubs_.empty()) {
            std::unordered_map<std::string, msr::airlib::LidarData> sensor_names_to_lidar_data_map;
            
            // Process regular lidar data
            for (auto& lidar_publisher : lidar_pubs_) {
                auto lidar_data = airsim_client_lidar_->getLidarData(
                    lidar_publisher.sensor_name, vehicle_name_);
                sensor_names_to_lidar_data_map[lidar_publisher.sensor_name] = lidar_data;
                
                sensor_msgs::msg::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data);
                lidar_msg.header.stamp = stamp_;
                lidar_msg.header.frame_id = vehicle_name_ + "/" + lidar_publisher.sensor_name;
                lidar_publisher.publisher->publish(lidar_msg);
            }
            
            // Process lidar labels
            for (auto& lidar_labels_publisher : lidar_labels_pubs_) {
                msr::airlib::LidarData lidar_data;
                auto it = sensor_names_to_lidar_data_map.find(lidar_labels_publisher.sensor_name);
                if (it != sensor_names_to_lidar_data_map.end()) {
                    lidar_data = it->second;
                } else {
                    lidar_data = airsim_client_lidar_->getLidarData(
                        lidar_labels_publisher.sensor_name, vehicle_name_);
                }
                
                airsim_interfaces::msg::StringArray lidar_label_msg = get_lidar_labels_msg_from_airsim(lidar_data);
                lidar_label_msg.header.stamp = stamp_;
                lidar_label_msg.header.frame_id = vehicle_name_ + "/" + lidar_labels_publisher.sensor_name;
                lidar_labels_publisher.publisher->publish(lidar_label_msg);
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in LiDAR timer callback for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::gpulidar_timer_cb()
{
    try {
        // Process GPU LiDAR data for this vehicle
        if (!gpulidar_pubs_.empty()) {
            for (auto& gpulidar_publisher : gpulidar_pubs_) {
                auto gpulidar_data = airsim_client_gpulidar_->getGPULidarData(
                    gpulidar_publisher.sensor_name, vehicle_name_);
                sensor_msgs::msg::PointCloud2 gpulidar_msg = get_gpulidar_msg_from_airsim(gpulidar_data);
                gpulidar_msg.header.stamp = stamp_;
                gpulidar_msg.header.frame_id = vehicle_name_ + "/" + gpulidar_publisher.sensor_name;
                gpulidar_publisher.publisher->publish(gpulidar_msg);
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in GPU LiDAR timer callback for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::echo_timer_cb()
{
    try {
        // Process Echo sensor data for this vehicle
        if (!echo_active_pubs_.empty() || !echo_passive_pubs_.empty()) {
            std::unordered_map<std::string, msr::airlib::EchoData> sensor_names_to_echo_data_map;
            
            // Process active echo data
            for (auto& active_echo_publisher : echo_active_pubs_) {
                auto echo_data = airsim_client_echo_->getEchoData(
                    active_echo_publisher.sensor_name, vehicle_name_);
                sensor_names_to_echo_data_map[active_echo_publisher.sensor_name] = echo_data;
                
                sensor_msgs::msg::PointCloud2 echo_msg = get_active_echo_msg_from_airsim(echo_data);
                echo_msg.header.stamp = stamp_;
                echo_msg.header.frame_id = vehicle_name_ + "/" + active_echo_publisher.sensor_name;
                active_echo_publisher.publisher->publish(echo_msg);
            }
            
            // Process passive echo data
            for (auto& passive_echo_publisher : echo_passive_pubs_) {
                msr::airlib::EchoData echo_data;
                auto it = sensor_names_to_echo_data_map.find(passive_echo_publisher.sensor_name);
                if (it != sensor_names_to_echo_data_map.end()) {
                    echo_data = it->second;
                } else {
                    echo_data = airsim_client_echo_->getEchoData(
                        passive_echo_publisher.sensor_name, vehicle_name_);
                    sensor_names_to_echo_data_map[passive_echo_publisher.sensor_name] = echo_data;
                }
                
                sensor_msgs::msg::PointCloud2 echo_msg = get_passive_echo_msg_from_airsim(echo_data);
                echo_msg.header.stamp = stamp_;
                echo_msg.header.frame_id = vehicle_name_ + "/" + passive_echo_publisher.sensor_name;
                passive_echo_publisher.publisher->publish(echo_msg);
            }
            
            // Process active echo labels
            for (auto& active_echo_labels_publisher : echo_active_labels_pubs_) {
                msr::airlib::EchoData echo_data;
                auto it = sensor_names_to_echo_data_map.find(active_echo_labels_publisher.sensor_name);
                if (it != sensor_names_to_echo_data_map.end()) {
                    echo_data = it->second;
                } else {
                    echo_data = airsim_client_echo_->getEchoData(
                        active_echo_labels_publisher.sensor_name, vehicle_name_);
                    sensor_names_to_echo_data_map[active_echo_labels_publisher.sensor_name] = echo_data;
                }
                
                airsim_interfaces::msg::StringArray echo_labels_msg = get_active_echo_labels_msg_from_airsim(echo_data);
                echo_labels_msg.header.stamp = stamp_;
                echo_labels_msg.header.frame_id = vehicle_name_ + "/" + active_echo_labels_publisher.sensor_name;
                active_echo_labels_publisher.publisher->publish(echo_labels_msg);
            }
            
            // Process passive echo labels
            for (auto& passive_echo_labels_publisher : echo_passive_labels_pubs_) {
                msr::airlib::EchoData echo_data;
                auto it = sensor_names_to_echo_data_map.find(passive_echo_labels_publisher.sensor_name);
                if (it != sensor_names_to_echo_data_map.end()) {
                    echo_data = it->second;
                } else {
                    echo_data = airsim_client_echo_->getEchoData(
                        passive_echo_labels_publisher.sensor_name, vehicle_name_);
                }
                
                airsim_interfaces::msg::StringArray echo_labels_msg = get_passive_echo_labels_msg_from_airsim(echo_data);
                echo_labels_msg.header.stamp = stamp_;
                echo_labels_msg.header.frame_id = vehicle_name_ + "/" + passive_echo_labels_publisher.sensor_name;
                passive_echo_labels_publisher.publisher->publish(echo_labels_msg);
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in echo timer callback for vehicle %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void VehicleNodeBase::publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = vehicle_name_;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(tf_msg);
}

void VehicleNodeBase::update_and_publish_static_transforms()
{
    // Append vehicle transform
    append_static_vehicle_tf(vehicle_config_);
    
    // Publish all static transforms
    if (!static_tf_msg_vec_.empty()) {
        static_tf_pub_->sendTransform(static_tf_msg_vec_);
        RCLCPP_DEBUG(this->get_logger(), "Published %zu static transforms for vehicle: %s", 
                     static_tf_msg_vec_.size(), vehicle_name_.c_str());
    }
}

void VehicleNodeBase::append_static_vehicle_tf(const VehicleSetting& vehicle_setting)
{
    geometry_msgs::msg::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.stamp = this->get_clock()->now();
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.child_frame_id = vehicle_name_;
    
    auto pose = vehicle_setting.position;
    auto orientation = vehicle_setting.rotation;
    
    vehicle_tf_msg.transform.translation.x = pose.x();
    vehicle_tf_msg.transform.translation.y = pose.y();
    vehicle_tf_msg.transform.translation.z = pose.z();
    
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(orientation.roll, orientation.pitch, orientation.yaw);
    vehicle_tf_msg.transform.rotation.x = tf_quat.x();
    vehicle_tf_msg.transform.rotation.y = tf_quat.y();
    vehicle_tf_msg.transform.rotation.z = tf_quat.z();
    vehicle_tf_msg.transform.rotation.w = tf_quat.w();
    
    static_tf_msg_vec_.push_back(vehicle_tf_msg);
}

bool VehicleNodeBase::instance_segmentation_refresh_cb(
    const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Response> response)
{
    try {
        if (airsim_client_) {
            airsim_client_->simListInstanceSegmentationObjects();
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Instance segmentation refreshed for vehicle: %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "AirSim client not initialized for vehicle: %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Failed to refresh instance segmentation for vehicle %s: %s", 
                     vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool VehicleNodeBase::object_transforms_refresh_cb(
    const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Response> response)
{
    try {
        if (airsim_client_) {
            airsim_client_->simListInstanceSegmentationPoses();
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Object transforms refreshed for vehicle: %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "AirSim client not initialized for vehicle: %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Failed to refresh object transforms for vehicle %s: %s", 
                     vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

void VehicleNodeBase::gimbal_angle_quat_cmd_cb(const airsim_interfaces::msg::GimbalAngleQuatCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    if (msg->vehicle_name == vehicle_name_ || msg->vehicle_name.empty()) {
        has_gimbal_cmd_ = true;
        gimbal_cmd_.vehicle_name = vehicle_name_;
        gimbal_cmd_.camera_name = msg->camera_name;
        gimbal_cmd_.target_quat = msr::airlib::Quaternionr(msg->orientation.w, msg->orientation.x, 
                                                          msg->orientation.y, msg->orientation.z);
        
        RCLCPP_DEBUG(this->get_logger(), "Received gimbal quaternion command for vehicle: %s", vehicle_name_.c_str());
    }
}

void VehicleNodeBase::gimbal_angle_euler_cmd_cb(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    if (msg->vehicle_name == vehicle_name_ || msg->vehicle_name.empty()) {
        has_gimbal_cmd_ = true;
        gimbal_cmd_.vehicle_name = vehicle_name_;
        gimbal_cmd_.camera_name = msg->camera_name;
        
        // Convert Euler angles to quaternion
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(msg->roll, msg->pitch, msg->yaw);
        gimbal_cmd_.target_quat = msr::airlib::Quaternionr(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z());
        
        RCLCPP_DEBUG(this->get_logger(), "Received gimbal Euler command for vehicle: %s", vehicle_name_.c_str());
    }
}

// Utility conversion methods
tf2::Quaternion VehicleNodeBase::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr VehicleNodeBase::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, 
                                   geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr VehicleNodeBase::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}

nav_msgs::msg::Odometry VehicleNodeBase::get_odom_msg_from_kinematic_state(const msr::airlib::Kinematics::State& kinematics_estimated) const
{
    nav_msgs::msg::Odometry odom_msg;
    
    odom_msg.header.stamp = stamp_;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = vehicle_name_;
    
    odom_msg.pose.pose.position.x = kinematics_estimated.pose.position.x();
    odom_msg.pose.pose.position.y = kinematics_estimated.pose.position.y();
    odom_msg.pose.pose.position.z = kinematics_estimated.pose.position.z();
    
    odom_msg.pose.pose.orientation.x = kinematics_estimated.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = kinematics_estimated.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = kinematics_estimated.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = kinematics_estimated.pose.orientation.w();
    
    odom_msg.twist.twist.linear.x = kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = kinematics_estimated.twist.linear.z();
    
    odom_msg.twist.twist.angular.x = kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = kinematics_estimated.twist.angular.z();
    
    return odom_msg;
}

// TODO: Implement remaining utility methods
// These need to be extracted from the original airsim_ros_wrapper.cpp file
// For now, providing stubs to make compilation successful

airsim_interfaces::msg::GPSYaw VehicleNodeBase::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    airsim_interfaces::msg::GPSYaw gps_msg;
    // TODO: Implement conversion from AirSim GeoPoint to GPSYaw message
    return gps_msg;
}

sensor_msgs::msg::NavSatFix VehicleNodeBase::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::msg::NavSatFix gps_sensor_msg;
    // TODO: Implement conversion from AirSim GeoPoint to NavSatFix message
    return gps_sensor_msg;
}

sensor_msgs::msg::Imu VehicleNodeBase::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    // TODO: Implement conversion from AirSim IMU data to ROS IMU message
    return imu_msg;
}

// Additional stub implementations for compilation
airsim_interfaces::msg::Altimeter VehicleNodeBase::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    airsim_interfaces::msg::Altimeter alt_msg;
    return alt_msg;
}

sensor_msgs::msg::Range VehicleNodeBase::get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::msg::Range range_msg;
    return range_msg;
}

airsim_interfaces::msg::InstanceSegmentationList VehicleNodeBase::get_instance_segmentation_list_msg_from_airsim() const
{
    airsim_interfaces::msg::InstanceSegmentationList instance_msg;
    return instance_msg;
}

airsim_interfaces::msg::ObjectTransformsList VehicleNodeBase::get_object_transforms_list_msg_from_airsim(rclcpp::Time timestamp) const
{
    airsim_interfaces::msg::ObjectTransformsList transforms_msg;
    return transforms_msg;
}

sensor_msgs::msg::PointCloud2 VehicleNodeBase::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    return cloud_msg;
}

airsim_interfaces::msg::StringArray VehicleNodeBase::get_lidar_labels_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const
{
    airsim_interfaces::msg::StringArray labels_msg;
    return labels_msg;
}

sensor_msgs::msg::PointCloud2 VehicleNodeBase::get_gpulidar_msg_from_airsim(const msr::airlib::GPULidarData& gpulidar_data) const
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    return cloud_msg;
}

sensor_msgs::msg::PointCloud2 VehicleNodeBase::get_active_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data) const
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    return cloud_msg;
}

airsim_interfaces::msg::StringArray VehicleNodeBase::get_active_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data) const
{
    airsim_interfaces::msg::StringArray labels_msg;
    return labels_msg;
}

sensor_msgs::msg::PointCloud2 VehicleNodeBase::get_passive_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data) const
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    return cloud_msg;
}

airsim_interfaces::msg::StringArray VehicleNodeBase::get_passive_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data) const
{
    airsim_interfaces::msg::StringArray labels_msg;
    return labels_msg;
}

sensor_msgs::msg::NavSatFix VehicleNodeBase::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    return gps_msg;
}

sensor_msgs::msg::MagneticField VehicleNodeBase::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField mag_msg;
    return mag_msg;
}

airsim_interfaces::msg::Environment VehicleNodeBase::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    airsim_interfaces::msg::Environment env_msg;
    return env_msg;
}

geometry_msgs::msg::Transform VehicleNodeBase::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
{
    geometry_msgs::msg::Transform transform_msg;
    return transform_msg;
}

geometry_msgs::msg::Transform VehicleNodeBase::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
{
    geometry_msgs::msg::Transform transform_msg;
    return transform_msg;
}

msr::airlib::Pose VehicleNodeBase::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
{
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), airlib_quat);
}

msr::airlib::GeoPoint VehicleNodeBase::get_origin_geo_point() const
{
    // TODO: Get from global configuration or parameter server
    return msr::airlib::GeoPoint(47.641468, -122.140165, 122.0);
}

// Pose utility methods
void VehicleNodeBase::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    // TODO: Implement NaN handling for vehicle poses
}

void VehicleNodeBase::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    // TODO: Implement NaN handling for camera poses
}

void VehicleNodeBase::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const
{
    // TODO: Implement NaN handling for LiDAR poses
}

void VehicleNodeBase::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, GPULidarSetting& gpulidar_setting) const
{
    // TODO: Implement NaN handling for GPU LiDAR poses
}

void VehicleNodeBase::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, EchoSetting& echo_setting) const
{
    // TODO: Implement NaN handling for echo sensor poses
}