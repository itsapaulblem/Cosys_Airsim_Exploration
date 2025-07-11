#include "vehicles/ComputerVisionNode.h"
#include <chrono>

ComputerVisionNode::ComputerVisionNode(const std::string& vehicle_name,
                                       const VehicleSetting& vehicle_config,
                                       const std::string& host_ip,
                                       uint16_t host_port,
                                       const std::string& world_frame_id,
                                       bool enable_api_control)
    : VehicleNodeBase(vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control)
{
    RCLCPP_INFO(this->get_logger(), "Initializing ComputerVision node: %s", vehicle_name_.c_str());
}

ComputerVisionNode::~ComputerVisionNode()
{
    RCLCPP_INFO(this->get_logger(), "Destroying ComputerVision node: %s", vehicle_name_.c_str());
}

void ComputerVisionNode::create_airsim_client()
{
    // Create ComputerVisionRpcLibClient for CV-specific communication
    airsim_client_ = std::make_unique<msr::airlib::ComputerVisionRpcLibClient>(host_ip_, host_port_);
    RCLCPP_DEBUG(this->get_logger(), "Created ComputerVisionRpcLibClient for entity: %s", vehicle_name_.c_str());
}

void ComputerVisionNode::setup_vehicle_ros_interface()
{
    // Create computer vision state publisher
    computer_vision_state_pub_ = this->create_publisher<airsim_interfaces::msg::ComputerVisionState>(
        "computer_vision_state", 10);
    
    RCLCPP_INFO(this->get_logger(), "ComputerVision ROS interface setup complete for entity: %s", vehicle_name_.c_str());
    
    // Note: Computer vision mode doesn't have control input subscribers like drones or cars
    // It's primarily focused on perception and sensor data without vehicle control capabilities
}

rclcpp::Time ComputerVisionNode::update_state()
{
    try {
        auto cv_client = get_computer_vision_client();
        curr_computer_vision_state_ = cv_client->getComputerVisionState(vehicle_name_);
        
        // Convert AirSim timestamp to ROS time
        rclcpp::Time vehicle_time(curr_computer_vision_state_.timestamp);
        
        // Update GPS sensor message (using pose position for geo-point conversion)
        gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(
            curr_computer_vision_state_.kinematics_estimated.pose.position);
        gps_sensor_msg_.header.stamp = vehicle_time;
        
        // Update odometry message
        curr_odom_ = get_odom_msg_from_computer_vision_state(curr_computer_vision_state_);
        curr_odom_.header.stamp = vehicle_time;
        curr_odom_.header.frame_id = odom_frame_id_;
        curr_odom_.child_frame_id = vehicle_name_;
        
        // Update computer vision state message
        computer_vision_state_msg_ = get_roscomputervisionstate_msg_from_computer_vision_state(curr_computer_vision_state_);
        computer_vision_state_msg_.header.frame_id = vehicle_name_;
        computer_vision_state_msg_.header.stamp = vehicle_time;
        
        return vehicle_time;
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to update computer vision state for %s: %s", 
                    vehicle_name_.c_str(), e.what());
        return this->get_clock()->now();
    }
}

void ComputerVisionNode::update_commands()
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        // Computer vision mode has limited control capabilities
        // Primary control is gimbal/camera orientation
        if (has_gimbal_cmd_) {
            auto cv_client = get_computer_vision_client();
            
            cv_client->setCameraOrientation(
                gimbal_cmd_.camera_name,
                gimbal_cmd_.target_quat,
                vehicle_name_
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Executed gimbal command for %s", vehicle_name_.c_str());
            has_gimbal_cmd_ = false;
        }
        
        // Note: Computer vision mode doesn't have velocity commands, takeoff/land, or car controls
        // It's primarily for perception tasks with stationary or scripted camera movements
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to execute commands for %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void ComputerVisionNode::publish_vehicle_state()
{
    // Publish odometry (pose and velocity information)
    if (odom_local_pub_->get_subscription_count() > 0) {
        odom_local_pub_->publish(curr_odom_);
    }
    
    // Publish GPS data
    if (global_gps_pub_->get_subscription_count() > 0) {
        gps_sensor_msg_.header.stamp = stamp_;
        gps_sensor_msg_.header.frame_id = world_frame_id_;
        global_gps_pub_->publish(gps_sensor_msg_);
    }
    
    // Publish computer vision specific state
    if (computer_vision_state_pub_->get_subscription_count() > 0) {
        computer_vision_state_pub_->publish(computer_vision_state_msg_);
    }
    
    // Publish environment data (if available)
    if (env_pub_->get_subscription_count() > 0) {
        env_msg_ = get_environment_msg_from_airsim(curr_computer_vision_state_.kinematics_estimated.pose.position);
        env_msg_.header.stamp = stamp_;
        env_msg_.header.frame_id = world_frame_id_;
        env_pub_->publish(env_msg_);
    }
}

nav_msgs::msg::Odometry ComputerVisionNode::get_odom_msg_from_computer_vision_state(
    const msr::airlib::ComputerVisionApiBase::ComputerVisionState& computer_vision_state) const
{
    return get_odom_msg_from_kinematic_state(computer_vision_state.kinematics_estimated);
}

airsim_interfaces::msg::ComputerVisionState ComputerVisionNode::get_roscomputervisionstate_msg_from_computer_vision_state(
    const msr::airlib::ComputerVisionApiBase::ComputerVisionState& computer_vision_state) const
{
    airsim_interfaces::msg::ComputerVisionState state_msg;
    
    // Get odometry data for pose and twist
    const auto odo = get_odom_msg_from_computer_vision_state(computer_vision_state);
    
    // Fill in computer vision state message
    state_msg.pose = odo.pose;
    state_msg.twist = odo.twist;
    state_msg.header.stamp = rclcpp::Time(computer_vision_state.timestamp);
    
    return state_msg;
}

msr::airlib::ComputerVisionRpcLibClient* ComputerVisionNode::get_computer_vision_client()
{
    return static_cast<msr::airlib::ComputerVisionRpcLibClient*>(airsim_client_.get());
}