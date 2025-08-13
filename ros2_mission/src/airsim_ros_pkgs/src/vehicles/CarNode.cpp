#include "vehicles/CarNode.h"
#include <chrono>

CarNode::CarNode(const std::string& vehicle_name,
                 const VehicleSetting& vehicle_config,
                 const std::string& host_ip,
                 uint16_t host_port,
                 const std::string& world_frame_id,
                 bool enable_api_control)
    : VehicleNodeBase(vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control)
    , has_car_cmd_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Car node: %s", vehicle_name_.c_str());
}

CarNode::~CarNode()
{
    RCLCPP_INFO(this->get_logger(), "Destroying Car node: %s", vehicle_name_.c_str());
}

void CarNode::create_airsim_client()
{
    // Create CarRpcLibClient for car-specific communication
    airsim_client_ = std::make_unique<msr::airlib::CarRpcLibClient>(host_ip_, host_port_);
    RCLCPP_DEBUG(this->get_logger(), "Created CarRpcLibClient for vehicle: %s", vehicle_name_.c_str());
}

void CarNode::setup_vehicle_ros_interface()
{
    // Create car control subscriber (only if API control is enabled)
    if (enable_api_control_) {
        car_cmd_sub_ = this->create_subscription<airsim_interfaces::msg::CarControls>(
            "car_cmd", 1,
            std::bind(&CarNode::car_cmd_cb, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Car control subscriber created for vehicle: %s", vehicle_name_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "API control disabled for vehicle: %s", vehicle_name_.c_str());
    }
    
    // Create car state publisher
    car_state_pub_ = this->create_publisher<airsim_interfaces::msg::CarState>("car_state", 10);
    
    RCLCPP_INFO(this->get_logger(), "Car ROS interface setup complete for vehicle: %s", vehicle_name_.c_str());
}

rclcpp::Time CarNode::update_state()
{
    try {
        auto car_client = get_car_client();
        curr_car_state_ = car_client->getCarState(vehicle_name_);
        
        // Convert AirSim timestamp to ROS time
        rclcpp::Time vehicle_time(curr_car_state_.timestamp);
        
        // Update GPS sensor message
        // TODO: Fix GPS conversion - position is Vector3r not GeoPoint
        // TODO: Fix GPS conversion - commented out for compilation

        // // gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(curr_car_state_.kinematics_estimated.pose.position);
        gps_sensor_msg_.header.stamp = vehicle_time;
        
        // Update odometry message
        curr_odom_ = get_odom_msg_from_car_state(curr_car_state_);
        curr_odom_.header.stamp = vehicle_time;
        curr_odom_.header.frame_id = odom_frame_id_;
        curr_odom_.child_frame_id = vehicle_name_;
        
        // Update car state message
        car_state_msg_ = get_roscarstate_msg_from_car_state(curr_car_state_);
        car_state_msg_.header.frame_id = vehicle_name_;
        car_state_msg_.header.stamp = vehicle_time;
        
        return vehicle_time;
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to update car state for %s: %s", 
                    vehicle_name_.c_str(), e.what());
        return this->get_clock()->now();
    }
}

void CarNode::update_commands()
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    try {
        // Execute car commands if available and API control is enabled
        if (enable_api_control_ && has_car_cmd_) {
            auto car_client = get_car_client();
            
            car_client->setCarControls(car_cmd_, vehicle_name_);
            
            RCLCPP_DEBUG(this->get_logger(), "Executed car command for %s: throttle=%.2f, steering=%.2f, brake=%.2f", 
                         vehicle_name_.c_str(), car_cmd_.throttle, car_cmd_.steering, car_cmd_.brake);
            
            has_car_cmd_ = false;
        }
        
        // Execute gimbal commands if available
        if (has_gimbal_cmd_) {
            auto car_client = get_car_client();
            
            // Use simSetCameraPose instead of setCameraOrientation 
            msr::airlib::Pose camera_pose;
            camera_pose.orientation = gimbal_cmd_.target_quat;
            camera_pose.position = msr::airlib::Vector3r(0, 0, 0); // Keep position unchanged
            
            car_client->simSetCameraPose(
                gimbal_cmd_.camera_name,
                camera_pose,
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

void CarNode::publish_vehicle_state()
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
    
    // Publish car state (dashboard readings: RPM, gear, speed, etc.)
    if (car_state_pub_->get_subscription_count() > 0) {
        car_state_pub_->publish(car_state_msg_);
    }
    
    // Publish environment data (if available)
    if (env_pub_->get_subscription_count() > 0) {
        // TODO: Fix Environment conversion - position is Vector3r not Environment::State  
        // TODO: Fix Environment conversion - commented out for compilation
  
        // // env_msg_ = get_environment_msg_from_airsim(curr_car_state_.kinematics_estimated.pose.position);
        env_msg_.header.stamp = stamp_;
        env_msg_.header.frame_id = world_frame_id_;
        env_pub_->publish(env_msg_);
    }
}

void CarNode::car_cmd_cb(const airsim_interfaces::msg::CarControls::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    
    // Parse car control message
    car_cmd_.throttle = msg->throttle;
    car_cmd_.steering = msg->steering;
    car_cmd_.brake = msg->brake;
    car_cmd_.handbrake = msg->handbrake;
    car_cmd_.is_manual_gear = msg->manual;
    car_cmd_.manual_gear = msg->manual_gear;
    car_cmd_.gear_immediate = msg->gear_immediate;
    
    has_car_cmd_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received car command for %s: throttle=%.2f, steering=%.2f, brake=%.2f, handbrake=%s", 
                 vehicle_name_.c_str(), car_cmd_.throttle, car_cmd_.steering, car_cmd_.brake, 
                 car_cmd_.handbrake ? "true" : "false");
}

nav_msgs::msg::Odometry CarNode::get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    return get_odom_msg_from_kinematic_state(car_state.kinematics_estimated);
}

airsim_interfaces::msg::CarState CarNode::get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    airsim_interfaces::msg::CarState state_msg;
    
    // Get odometry data
    const auto odo = get_odom_msg_from_car_state(car_state);
    
    // Fill in car state message
    state_msg.pose = odo.pose;
    state_msg.twist = odo.twist;
    state_msg.speed = car_state.speed;
    state_msg.gear = car_state.gear;
    state_msg.rpm = car_state.rpm;
    state_msg.maxrpm = car_state.maxrpm;
    state_msg.handbrake = car_state.handbrake;
    state_msg.header.stamp = rclcpp::Time(car_state.timestamp);
    
    return state_msg;
}

msr::airlib::CarRpcLibClient* CarNode::get_car_client()
{
    return static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get());
}