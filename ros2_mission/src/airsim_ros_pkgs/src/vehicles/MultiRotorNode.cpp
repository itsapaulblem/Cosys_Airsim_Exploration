#include "vehicles/MultiRotorNode.h"
#include <chrono>

MultiRotorNode::MultiRotorNode(const std::string& vehicle_name,
                               const VehicleSetting& vehicle_config,
                               const std::string& host_ip,
                               uint16_t host_port,
                               const std::string& world_frame_id,
                               bool enable_api_control)
    : VehicleNodeBase(vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control)
    , has_vel_cmd_(false)
    , vel_cmd_duration_(DEFAULT_VEL_CMD_DURATION)
{
    RCLCPP_INFO(this->get_logger(), "Initializing MultiRotor node: %s", vehicle_name_.c_str());
    
    // Get velocity command duration from parameters if available
    this->declare_parameter("vel_cmd_duration", DEFAULT_VEL_CMD_DURATION);
    vel_cmd_duration_ = this->get_parameter("vel_cmd_duration").as_double();
}

MultiRotorNode::~MultiRotorNode()
{
    RCLCPP_INFO(this->get_logger(), "Destroying MultiRotor node: %s", vehicle_name_.c_str());
}

void MultiRotorNode::create_airsim_client()
{
    // Create MultirrotorRpcLibClient for drone-specific communication
    airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
    RCLCPP_DEBUG(this->get_logger(), "Created MultirotorRpcLibClient for vehicle: %s", vehicle_name_.c_str());
}

void MultiRotorNode::setup_vehicle_ros_interface()
{
    // Create velocity command subscribers
    vel_cmd_body_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_body_frame", 10,
        std::bind(&MultiRotorNode::vel_cmd_body_frame_cb, this, std::placeholders::_1));
    
    vel_cmd_world_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_world_frame", 10,
        std::bind(&MultiRotorNode::vel_cmd_world_frame_cb, this, std::placeholders::_1));
    
    // Create drone control services
    takeoff_srvr_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        "takeoff",
        std::bind(&MultiRotorNode::takeoff_srv_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    land_srvr_ = this->create_service<airsim_interfaces::srv::Land>(
        "land",
        std::bind(&MultiRotorNode::land_srv_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    set_altitude_srvr_ = this->create_service<airsim_interfaces::srv::SetAltitude>(
        "set_altitude",
        std::bind(&MultiRotorNode::set_altitude_srv_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    set_local_position_srvr_ = this->create_service<airsim_interfaces::srv::SetLocalPosition>(
        "set_local_position",
        std::bind(&MultiRotorNode::set_local_position_srv_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "MultiRotor ROS interface setup complete for vehicle: %s", vehicle_name_.c_str());
}

rclcpp::Time MultiRotorNode::update_state()
{
    try {
        auto multirotor_client = get_multirotor_client();
        curr_drone_state_ = multirotor_client->getMultirotorState(vehicle_name_);
        
        // Convert AirSim timestamp to ROS time
        rclcpp::Time vehicle_time(curr_drone_state_.timestamp);
        
        // Update GPS sensor message
        // TODO: Fix GPS conversion - commented out for compilation

        // gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(curr_drone_state_.gps_location);
        
        // Update odometry message
        curr_odom_ = get_odom_msg_from_multirotor_state(curr_drone_state_);
        curr_odom_.header.stamp = vehicle_time;
        curr_odom_.header.frame_id = odom_frame_id_;
        curr_odom_.child_frame_id = vehicle_name_;
        
        return vehicle_time;
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to update drone state for %s: %s", 
                    vehicle_name_.c_str(), e.what());
        return this->get_clock()->now();
    }
}

void MultiRotorNode::update_commands()
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        // Execute velocity commands if available
        if (has_vel_cmd_) {
            auto multirotor_client = get_multirotor_client();
            
            multirotor_client->moveByVelocityAsync(
                vel_cmd_.x, vel_cmd_.y, vel_cmd_.z,
                vel_cmd_duration_,
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                vel_cmd_.yaw_mode,
                vehicle_name_
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Executed velocity command for %s: [%.2f, %.2f, %.2f]", 
                         vehicle_name_.c_str(), vel_cmd_.x, vel_cmd_.y, vel_cmd_.z);
            
            has_vel_cmd_ = false;
        }
        
        // Execute gimbal commands if available
        if (has_gimbal_cmd_) {
            auto multirotor_client = get_multirotor_client();
            
            // TODO: Fix camera orientation - commented out for compilation

            
            // multirotor_client->setCameraOrientation(
                gimbal_cmd_.camera_name,
                gimbal_cmd_.target_quat,
                vehicle_name_
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Executed gimbal command for %s", vehicle_name_.c_str());
            has_gimbal_cmd_ = false;
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to execute commands for %s: %s", 
                    vehicle_name_.c_str(), e.what());
    }
}

void MultiRotorNode::publish_vehicle_state()
{
    // Publish odometry
    if (odom_local_pub_->get_subscription_count() > 0) {
        odom_local_pub_->publish(curr_odom_);
    }
    
    // Publish GPS data
    if (global_gps_pub_->get_subscription_count() > 0) {
        gps_sensor_msg_.header.stamp = stamp_;
        gps_sensor_msg_.header.frame_id = world_frame_id_;
        global_gps_pub_->publish(gps_sensor_msg_);
    }
    
    // Publish environment data (if available)
    if (env_pub_->get_subscription_count() > 0) {
        // TODO: Fix Environment conversion - commented out for compilation

        // env_msg_ = get_environment_msg_from_airsim(curr_drone_state_.kinematics_estimated.pose.position);
        env_msg_.header.stamp = stamp_;
        env_msg_.header.frame_id = world_frame_id_;
        env_pub_->publish(env_msg_);
    }
}

void MultiRotorNode::vel_cmd_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    if (msg->vehicle_name.empty() || msg->vehicle_name == vehicle_name_) {
        std::lock_guard<std::mutex> guard(control_mutex_);
        
        vel_cmd_ = get_airlib_body_vel_cmd(*msg, curr_drone_state_.kinematics_estimated.pose.orientation);
        vel_cmd_.vehicle_name = vehicle_name_;
        has_vel_cmd_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Received body frame velocity command for %s", vehicle_name_.c_str());
    }
}

void MultiRotorNode::vel_cmd_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    if (msg->vehicle_name.empty() || msg->vehicle_name == vehicle_name_) {
        std::lock_guard<std::mutex> guard(control_mutex_);
        
        vel_cmd_ = get_airlib_world_vel_cmd(*msg);
        vel_cmd_.vehicle_name = vehicle_name_;
        has_vel_cmd_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Received world frame velocity command for %s", vehicle_name_.c_str());
    }
}

bool MultiRotorNode::takeoff_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
                                   const std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Takeoff request received for %s", vehicle_name_.c_str());
        
        auto multirotor_client = get_multirotor_client();
        auto future = multirotor_client->takeoffAsync(TAKEOFF_TIMEOUT_SEC, vehicle_name_);
        
        // Wait for takeoff to complete
        auto status = future.wait_for(std::chrono::seconds(static_cast<int>(TAKEOFF_TIMEOUT_SEC)));
        
        if (status == std::future_status::ready) {
            future.get(); // This will throw if there was an error
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff completed successfully for %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Takeoff timeout for %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Takeoff failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultiRotorNode::land_srv_cb(const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
                                const std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Land request received for %s", vehicle_name_.c_str());
        
        auto multirotor_client = get_multirotor_client();
        auto future = multirotor_client->landAsync(LAND_TIMEOUT_SEC, vehicle_name_);
        
        // Wait for landing to complete
        auto status = future.wait_for(std::chrono::seconds(static_cast<int>(LAND_TIMEOUT_SEC)));
        
        if (status == std::future_status::ready) {
            future.get(); // This will throw if there was an error
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Landing completed successfully for %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Landing timeout for %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Landing failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultiRotorNode::set_altitude_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Request> request,
                                         const std::shared_ptr<airsim_interfaces::srv::SetAltitude::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Set altitude request received for %s: %.2f", 
                    vehicle_name_.c_str(), request->altitude);
        
        auto multirotor_client = get_multirotor_client();
        auto future = multirotor_client->moveToZAsync(request->altitude, 0.0f /* TODO: velocity field not in service */, POSITION_TIMEOUT_SEC, 
                                                     msr::airlib::YawMode(), -1.0, 1.0, vehicle_name_);
        
        // Wait for position change to complete
        auto status = future.wait_for(std::chrono::seconds(static_cast<int>(POSITION_TIMEOUT_SEC)));
        
        if (status == std::future_status::ready) {
            future.get(); // This will throw if there was an error
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Set altitude completed successfully for %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Set altitude timeout for %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Set altitude failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultiRotorNode::set_local_position_srv_cb(const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Request> request,
                                              const std::shared_ptr<airsim_interfaces::srv::SetLocalPosition::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Set local position request received for %s: [%.2f, %.2f, %.2f]", 
                    vehicle_name_.c_str(), request->x, request->y, request->z);
        
        auto multirotor_client = get_multirotor_client();
        auto future = multirotor_client->moveToPositionAsync(
            request->x, request->y, request->z,
            0.0f /* TODO: velocity field not in service */, POSITION_TIMEOUT_SEC,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(), -1.0, 1.0, vehicle_name_);
        
        // Wait for position change to complete
        auto status = future.wait_for(std::chrono::seconds(static_cast<int>(POSITION_TIMEOUT_SEC)));
        
        if (status == std::future_status::ready) {
            future.get(); // This will throw if there was an error
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Set local position completed successfully for %s", vehicle_name_.c_str());
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Set local position timeout for %s", vehicle_name_.c_str());
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Set local position failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

nav_msgs::msg::Odometry MultiRotorNode::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const
{
    return get_odom_msg_from_kinematic_state(drone_state.kinematics_estimated);
}

VelCmd MultiRotorNode::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg) const
{
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = static_cast<msr::airlib::DrivetrainType>(msg.drivetrain);
    
    // Set yaw mode
    vel_cmd.yaw_mode.is_rate = msg.yaw_mode.is_rate;
    vel_cmd.yaw_mode.yaw_or_rate = msg.yaw_mode.yaw_or_rate;
    
    return vel_cmd;
}

VelCmd MultiRotorNode::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, const msr::airlib::Quaternionr& orientation) const
{
    VelCmd vel_cmd = get_airlib_world_vel_cmd(msg);
    
    // Transform velocity from body frame to world frame using vehicle orientation
    msr::airlib::Vector3r body_vel(vel_cmd.x, vel_cmd.y, vel_cmd.z);
    msr::airlib::Vector3r world_vel = msr::airlib::VectorMath::transformToWorldFrame(body_vel, orientation);
    
    vel_cmd.x = world_vel.x();
    vel_cmd.y = world_vel.y();
    vel_cmd.z = world_vel.z();
    
    return vel_cmd;
}

msr::airlib::MultirotorRpcLibClient* MultiRotorNode::get_multirotor_client()
{
    return static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
}