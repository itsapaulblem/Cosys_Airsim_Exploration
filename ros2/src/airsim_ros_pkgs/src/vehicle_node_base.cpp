#include "vehicle_node_base.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

VehicleNodeBase::VehicleNodeBase(const std::string& vehicle_name, 
                                 const std::string& host_ip, 
                                 uint16_t host_port)
    : Node("airsim_" + vehicle_name)
    , vehicle_name_(vehicle_name)
    , host_ip_(host_ip)
    , host_port_(host_port)
    , world_frame_id_("world_ned")
    , odom_frame_id_("odom_local_ned")
    , state_timer_freq_(0.01)
    , image_timer_freq_(0.05)
    , lidar_timer_freq_(0.01)
    , gpulidar_timer_freq_(0.01)
    , echo_timer_freq_(0.01)
{
    // Declare parameters
    this->declare_parameter("host_ip", host_ip);
    this->declare_parameter("host_port", static_cast<int>(host_port));
    this->declare_parameter("world_frame_id", world_frame_id_);
    this->declare_parameter("odom_frame_id", odom_frame_id_);

    this->declare_parameter("state_timer_freq", state_timer_freq_);
    this->declare_parameter("image_timer_freq", image_timer_freq_);
    this->declare_parameter("lidar_timer_freq", lidar_timer_freq_);
    this->declare_parameter("gpulidar_timer_freq", gpulidar_timer_freq_);
    this->declare_parameter("echo_timer_freq", echo_timer_freq_);
    
    // Get parameters
    this->get_parameter("host_ip", host_ip_);
    int port_param = static_cast<int>(host_port_);
    this->get_parameter("host_port", port_param);
    host_port_ = static_cast<uint16_t>(port_param);
    this->get_parameter("world_frame_id", world_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    
    this->get_parameter("state_timer_freq", state_timer_freq_);
    this->get_parameter("image_timer_freq", image_timer_freq_);
    this->get_parameter("lidar_timer_freq", lidar_timer_freq_);
    this->get_parameter("gpulidar_timer_freq", gpulidar_timer_freq_);
    this->get_parameter("echo_timer_freq", echo_timer_freq_);

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

void VehicleNodeBase::initialize_common()
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
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
}

bool VehicleNodeBase::establish_connections()
{
    try {
        // Phase 2.1: Independent connection management per vehicle, isolated failure handling
        airsim_client_images_->confirmConnection();
        airsim_client_lidar_->confirmConnection();
        airsim_client_gpulidar_->confirmConnection();
        airsim_client_echo_->confirmConnection();
        
        RCLCPP_INFO(this->get_logger(), "RPC connections established for vehicle: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "connection establishment");
        return false;
    }
}

void VehicleNodeBase::setup_publishers()
{
    // Common publishers for all vehicle types
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>("environment", 10);
    
    // Call virtual method for vehicle-specific publishers
    setup_vehicle_publishers();
}

void VehicleNodeBase::setup_services()
{
    reset_service_ = this->create_service<airsim_interfaces::srv::Reset>(
        "reset", 
        std::bind(&VehicleNodeBase::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Call virtual method for vehicle-specific services
    setup_vehicle_services();
}

// Phase 2.1: Independent Timer Management with isolated callback groups
void VehicleNodeBase::setup_timers()
{
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
}

// Phase 2.1: Independent timer callbacks
void VehicleNodeBase::state_timer_callback()
{
    try {
        update_vehicle_state();
        publish_vehicle_state();
        handle_vehicle_commands();
        publish_static_transforms();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "state update");
    }
}

void VehicleNodeBase::image_timer_callback()
{
    try {
        process_images();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "image processing");
    }
}

void VehicleNodeBase::lidar_timer_callback()
{
    try {
        process_lidar();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "lidar processing");
    }
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
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
}

void VehicleNodeBase::publish_static_transforms()
{
    if (!static_tf_msg_vec_.empty()) {
        auto now = this->get_clock()->now();
        for (auto& static_tf_msg : static_tf_msg_vec_) {
            static_tf_msg.header.stamp = now;
            static_tf_pub_->sendTransform(static_tf_msg);
        }
    }
}

bool VehicleNodeBase::reset_callback(const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
                                    std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
{
    (void)request;  // Suppress unused parameter warning
    
    try {
        // Need to use main client for reset (not specialized clients)
        if (airsim_client_) {
            airsim_client_->reset();
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

void VehicleNodeBase::setup_vehicle_services() 
{
    // Default implementation - no additional services
}

void VehicleNodeBase::update_vehicle_state() 
{
    // Default implementation - basic state update
    RCLCPP_DEBUG(this->get_logger(), "Updating state for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::publish_vehicle_state() 
{
    // Default implementation - basic state publishing
    RCLCPP_DEBUG(this->get_logger(), "Publishing state for vehicle: %s", vehicle_name_.c_str());
}

void VehicleNodeBase::handle_vehicle_commands() 
{
    // Default implementation - no command handling
}

void VehicleNodeBase::process_images() 
{
    // Default implementation - no image processing
}

void VehicleNodeBase::process_lidar() 
{
    // Default implementation - no lidar processing
}

void VehicleNodeBase::process_gpulidar() 
{
    // Default implementation - no GPU lidar processing
}

void VehicleNodeBase::process_echo() 
{
    // Default implementation - no echo processing
}