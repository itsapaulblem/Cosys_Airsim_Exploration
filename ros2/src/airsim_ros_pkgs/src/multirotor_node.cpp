#include "multirotor_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>

MultirotorNode::MultirotorNode(const std::string& vehicle_name, 
                               const std::string& host_ip, 
                               uint16_t host_port)
    : VehicleNodeBase(vehicle_name, host_ip, host_port)
    , stamp_(this->get_clock()->now())
{
    initialize_vehicle_client();
    setup_sensor_publishers();
    setup_vehicle_publishers();
    setup_vehicle_subscribers();
    setup_vehicle_services();
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            this->update_vehicle_state();
            this->publish_vehicle_state();
        }
    );

    // Initialize common components
    initialize_common();
    
    RCLCPP_INFO(this->get_logger(), "Multirotor node created for: %s", vehicle_name_.c_str());
}

void MultirotorNode::initialize_vehicle_client()
{
    try {
        airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        airsim_client_->confirmConnection();
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        multirotor_client->enableApiControl(true, vehicle_name_);
        multirotor_client->armDisarm(true, vehicle_name_);
        
        RCLCPP_INFO(this->get_logger(), "Multirotor client initialized for: %s", vehicle_name_.c_str());
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "multirotor client initialization");
    }
}

void MultirotorNode::setup_sensor_publishers()
{
    for (int i =  0; i < 4; ++i) {
        auto camera_pub = this->create_publisher<sensor_msgs::msg::Image>("camera" + std::to_string(i) + "/image", 10);
        camera_pubs_.push_back(camera_pub);

        auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera" + std::to_string(i) + "/camera_info", 10); 
        camera_info_pubs_.push_back(camera_info_pub); 
    }

    for (int i = 0; i < 1; ++i) {
        auto lidar_pub  = this-> create_publisher<sensor_msgs::msg::PointCloud2>("lidar" + std::to_string(i) + "/points", 10);
        lidar_pubs_.push_back(lidar_pub);
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10); 
    baro_pub_ = this->create_publisher<sensor_msgs::msg::Range>("baro", 10);
}

void MultirotorNode::setup_vehicle_publishers()
{
    // Base class already sets up odom_pub_, gps_pub_, env_pub_
    // Add any drone-specific publishers here if needed in the future
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle publishers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_subscribers()
{
    vel_cmd_body_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_body_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_body_callback, this, std::placeholders::_1));
        
    vel_cmd_world_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_world_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_world_callback, this, std::placeholders::_1));
        
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle subscribers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_services()
{
    takeoff_service_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        "takeoff",
        std::bind(&MultirotorNode::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    land_service_ = this->create_service<airsim_interfaces::srv::Land>(
        "land",
        std::bind(&MultirotorNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle services for: %s", vehicle_name_.c_str());
}

void MultirotorNode::update_vehicle_state()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        curr_drone_state_ = multirotor_client->getMultirotorState(vehicle_name_);
        
        // Update timestamp
        stamp_ = this->get_clock()->now();
        
        // Update odometry
        curr_odom_ = get_odom_from_multirotor_state(curr_drone_state_);
        curr_odom_.header.frame_id = world_frame_id_;
        curr_odom_.child_frame_id = vehicle_name_ + "/" + odom_frame_id_;
        curr_odom_.header.stamp = stamp_;
        
        // Update GPS
        auto gps_data = multirotor_client->getGpsData("", vehicle_name_);
        gps_sensor_msg_.header.stamp = stamp_;
        gps_sensor_msg_.header.frame_id = vehicle_name_;
        gps_sensor_msg_.latitude = gps_data.gnss.geo_point.latitude;
        gps_sensor_msg_.longitude = gps_data.gnss.geo_point.longitude;
        gps_sensor_msg_.altitude = gps_data.gnss.geo_point.altitude;

        // IMU
        auto imu_data = multirotor_client->getImuData("", vehicle_name_);
        imu_msg_.header.stamp = stamp_;
        imu_msg_.header.frame_id = vehicle_name_ + "_base_link"; 
        imu_msg_.orientation.x = imu_data.orientation.x();
        imu_msg_.orientation.y = - imu_data.orientation.y();
        imu_msg_.orientation.z = - imu_data.orientation.z();
        imu_msg_.orientation.w = imu_data.orientation.w();
        imu_msg_.angular_velocity.x = imu_data.angular_velocity.x();
        imu_msg_.angular_velocity.y = - imu_data.angular_velocity.y();
        imu_msg_.angular_velocity.z = - imu_data.angular_velocity.z();
        imu_msg_.linear_acceleration.x = imu_data.linear_acceleration.x();
        imu_msg_.linear_acceleration.y = - imu_data.linear_acceleration.y();
        imu_msg_.linear_acceleration.z = - imu_data.linear_acceleration.z();

        // Get environment data
        auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_name_);
        env_msg_.header.stamp = stamp_;
        env_msg_.header.frame_id = vehicle_name_;
        env_msg_.position.x = env_data.position.x();
        env_msg_.position.y = env_data.position.y();
        env_msg_.position.z = env_data.position.z();
        env_msg_.air_pressure = env_data.air_pressure;
        env_msg_.temperature = env_data.temperature;
        env_msg_.air_density = env_data.air_density;

        // Magnetometer (optional)
        auto mag_data = multirotor_client->getMagnetometerData("", vehicle_name_);
        mag_msg_.header.stamp = stamp_;
        mag_msg_.header.frame_id = vehicle_name_ + "_base_link";
        mag_msg_.magnetic_field.x = mag_data.magnetic_field_body.x();
        mag_msg_.magnetic_field.y = mag_data.magnetic_field_body.y();
        mag_msg_.magnetic_field.z = mag_data.magnetic_field_body.z();

        // Barometer (optional)
        auto baro_data = multirotor_client->getBarometerData("", vehicle_name_); 
        baro_msg_.header.stamp = stamp_;
        baro_msg_.header.frame_id = vehicle_name_ + "_base_link";
        baro_msg_.range = baro_data.altitude;

    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "multirotor state update");
    }
}

void MultirotorNode::publish_vehicle_state()
{
    // Publish odometry
    odom_pub_->publish(curr_odom_);
    publish_odometry_tf(curr_odom_);
    
    // Publish GPS
    gps_pub_->publish(gps_sensor_msg_);
    
    // Publish environment
    env_pub_->publish(env_msg_);
    mag_pub_->publish(mag_msg_);
    baro_pub_->publish(baro_msg_);
    imu_pub_->publish(imu_msg_);
}

void MultirotorNode::handle_vehicle_commands()
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    if (has_vel_cmd_) {
        try {
            auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            multirotor_client->moveByVelocityAsync(
                current_vel_cmd_.x, current_vel_cmd_.y, current_vel_cmd_.z,
                0.05,
                current_vel_cmd_.drivetrain,
                current_vel_cmd_.yaw_mode,
                vehicle_name_);
                
            has_vel_cmd_ = false;
        }
        catch (const rpc::rpc_error& e) {
            handle_rpc_error(e, "velocity command");
        }
    }
}

void MultirotorNode::vel_cmd_body_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_vel_cmd_ = get_airlib_body_vel_cmd(*msg, curr_drone_state_.kinematics_estimated.pose.orientation);
    has_vel_cmd_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received body velocity command for: %s", vehicle_name_.c_str());
}

void MultirotorNode::vel_cmd_world_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_vel_cmd_ = get_airlib_world_vel_cmd(*msg);
    has_vel_cmd_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received world velocity command for: %s", vehicle_name_.c_str());
}

bool MultirotorNode::takeoff_callback(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
                                     std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        if (request->wait_on_last_task) {
            multirotor_client->takeoffAsync(20, vehicle_name_)->waitOnLastTask();
        } else {
            multirotor_client->takeoffAsync(20, vehicle_name_);
        }
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Takeoff command sent for: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "takeoff");
        response->success = false;
        return false;
    }
}

bool MultirotorNode::land_callback(const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
                                  std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        if (request->wait_on_last_task) {
            multirotor_client->landAsync(60, vehicle_name_)->waitOnLastTask();
        } else {
            multirotor_client->landAsync(60, vehicle_name_);
        }
        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Land command sent for: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "land");
        response->success = false;
        return false;
    }
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg)
{
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = msg.twist.angular.z * 180.0 / M_PI;
    return vel_cmd;
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                                               const msr::airlib::Quaternionr& orientation)
{
    VelCmd vel_cmd;
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(orientation)).getRPY(roll, pitch, yaw);
    
    // Transform to body frame
    vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw));
    vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw));
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = msg.twist.angular.z * 180.0 / M_PI;
    
    return vel_cmd;
}

nav_msgs::msg::Odometry MultirotorNode::get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state)
{
    nav_msgs::msg::Odometry odom_msg;
    const auto& kinematics = state.kinematics_estimated;
    
    // Position conversion (AirSim NED to ROS ENU)
    odom_msg.pose.pose.position.x = kinematics.pose.position.x();
    odom_msg.pose.pose.position.y = -kinematics.pose.position.y();
    odom_msg.pose.pose.position.z = -kinematics.pose.position.z();
    
    // Orientation conversion (AirSim NED to ROS ENU)
    odom_msg.pose.pose.orientation.x = kinematics.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = -kinematics.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = -kinematics.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = kinematics.pose.orientation.w();
    
    // Linear velocity conversion (AirSim NED to ROS ENU)
    odom_msg.twist.twist.linear.x = kinematics.twist.linear.x();
    odom_msg.twist.twist.linear.y = -kinematics.twist.linear.y();
    odom_msg.twist.twist.linear.z = -kinematics.twist.linear.z();
    
    // Angular velocity conversion (AirSim NED to ROS ENU)
    odom_msg.twist.twist.angular.x = kinematics.twist.angular.x();
    odom_msg.twist.twist.angular.y = -kinematics.twist.angular.y();
    odom_msg.twist.twist.angular.z = -kinematics.twist.angular.z();
    
    return odom_msg;
}

void MultirotorNode::process_images()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_images_.get());

        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;
        
        msr::airlib::ImageCaptureBase::ImageRequest request;
        request.camera_name = "0";
        request.image_type = msr::airlib::ImageCaptureBase::ImageType::Scene;
        request.pixels_as_float = false;
        requests.push_back(request);

        auto responses = multirotor_client->simGetImages(requests, vehicle_name_);

        for (size_t i = 0; i < responses.size() && i < camera_pubs_.size(); ++i) {
            if (!responses[i].pixels_as_float && !responses[i].image_data_uint8.empty()) {
                sensor_msgs::msg::Image img_msg;
                img_msg.header.stamp = stamp_;
                img_msg.header.frame_id = vehicle_name_ + "/camera";
                img_msg.width = responses[i].width;
                img_msg.height = responses[i].height;
                img_msg.encoding = "bgr8";
                img_msg.step = responses[i].width * 3;
                img_msg.data = responses[i].image_data_uint8;

                camera_pubs_[i]->publish(img_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "Published camera image for: %s", vehicle_name_.c_str());
            }
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "image processing");
    }
}

void MultirotorNode::process_lidar()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_lidar_.get());

        auto lidar_data = multirotor_client->getLidarData("Lidar1", vehicle_name_);

        if (lidar_data.point_cloud.size() > 3) {
            sensor_msgs::msg::PointCloud2 lidar_msg;
            lidar_msg.header.stamp = stamp_;
            lidar_msg.header.frame_id = vehicle_name_ + "/lidar";
            
            // Configure point cloud message
            lidar_msg.width = lidar_data.point_cloud.size() / 3;
            lidar_msg.height = 1;
            lidar_msg.is_dense = true;
            lidar_msg.is_bigendian = false;
            
            // Set up fields for XYZ point cloud
            lidar_msg.fields.resize(3);
            lidar_msg.fields[0].name = "x";
            lidar_msg.fields[0].offset = 0;
            lidar_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[0].count = 1;
            
            lidar_msg.fields[1].name = "y";
            lidar_msg.fields[1].offset = 4;
            lidar_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[1].count = 1;
            
            lidar_msg.fields[2].name = "z";
            lidar_msg.fields[2].offset = 8;
            lidar_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[2].count = 1;
            
            lidar_msg.point_step = 12; // 3 floats * 4 bytes
            lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;
            
            // Copy point cloud data
            lidar_msg.data.resize(lidar_data.point_cloud.size() * sizeof(float));
            std::memcpy(lidar_msg.data.data(), lidar_data.point_cloud.data(), 
                       lidar_data.point_cloud.size() * sizeof(float));

            if (!lidar_pubs_.empty()) {
                lidar_pubs_[0]->publish(lidar_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published lidar data for: %s", vehicle_name_.c_str());
            }
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "lidar processing");
    }
}

void MultirotorNode::process_gpulidar()
{
    try {
        // GPU LiDAR processing - similar to regular LiDAR but with different API calls
        // auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_gpulidar_.get());
        
        // Note: This would need to be implemented based on your specific GPU LiDAR configuration
        // For now, placeholder implementation
        RCLCPP_DEBUG(this->get_logger(), "Processing GPU lidar for: %s", vehicle_name_.c_str());

        (void) airsim_client_gpulidar_; // Suppress unused variable warning
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "GPU lidar processing");
    }
}

void MultirotorNode::process_echo()
{
    try {
        // Echo/radar processing - implement based on your sensor configuration
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_echo_.get());
        
        // Note: This would need to be implemented based on your specific echo/radar configuration
        // For now, placeholder implementation
        RCLCPP_DEBUG(this->get_logger(), "Processing echo/radar for: %s", vehicle_name_.c_str());
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "echo processing");
    }
}