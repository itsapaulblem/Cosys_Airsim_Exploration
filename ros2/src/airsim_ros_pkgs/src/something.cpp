#include "multirotor_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Note: Static image type mapping moved to VehicleNodeBase

MultirotorNode::MultirotorNode(const std::string& vehicle_name, 
                               const std::string& host_ip, 
                               uint16_t host_port)
    : VehicleNodeBase(vehicle_name, host_ip, host_port)
{
    RCLCPP_INFO(this->get_logger(), "Multirotor node created for: %s", vehicle_name_.c_str());
}

void MultirotorNode::initialize_common()
{
    // CRITICAL: Initialize the main vehicle client first
    initialize_vehicle_client();
    
    // Call base class implementation (handles image_transport, publishers, services, timers, etc.)
    VehicleNodeBase::initialize_common();
    
    // Now establish connections using the initialized client
    if (!establish_connections()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish connections for vehicle: %s", vehicle_name_.c_str());
        return;
    }
    
    // Add multirotor-specific setup after base initialization is complete
    setup_vehicle_control_subscribers();
    setup_vehicle_control_services();
    
    RCLCPP_INFO(this->get_logger(), "Multirotor node fully initialized: %s", vehicle_name_.c_str());
}

void MultirotorNode::initialize_vehicle_client()
{
    try {
        airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        airsim_client_->confirmConnection();
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        // Test sensor data access immediately after connection
        try {
            auto test_imu = multirotor_client->getImuData("", vehicle_name_);
            RCLCPP_INFO(this->get_logger(), "✅ Client can read IMU data for %s (timestamp: %lu)", 
                vehicle_name_.c_str(), test_imu.time_stamp);
            
            // Check if simulation is paused (timestamp = 0 indicates paused simulation)
            if (test_imu.time_stamp == 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️  Simulation appears paused (timestamp=0), attempting to unpause...");
                try {
                    multirotor_client->simPause(false);
                    RCLCPP_INFO(this->get_logger(), "✅ Simulation unpaused for %s", vehicle_name_.c_str());
                    
                    // Test again after unpausing
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    auto test_imu2 = multirotor_client->getImuData("", vehicle_name_);
                    RCLCPP_INFO(this->get_logger(), "✅ Post-unpause IMU timestamp for %s: %lu", 
                        vehicle_name_.c_str(), test_imu2.time_stamp);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "⚠️  Could not unpause simulation for %s: %s", 
                        vehicle_name_.c_str(), e.what());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️  Client cannot read IMU data for %s: %s", 
                vehicle_name_.c_str(), e.what());
        }
        
        // Enable API control (non-critical for sensor publishing)
        try {
            multirotor_client->enableApiControl(true, vehicle_name_);
            RCLCPP_INFO(this->get_logger(), "API control enabled for: %s", vehicle_name_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not enable API control for %s: %s", vehicle_name_.c_str(), e.what());
        }
        
        // Arm the vehicle (non-critical for sensor publishing)
        try {
            multirotor_client->armDisarm(true, vehicle_name_);
            RCLCPP_INFO(this->get_logger(), "Vehicle armed: %s", vehicle_name_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not arm vehicle %s: %s", vehicle_name_.c_str(), e.what());
        }
        
        RCLCPP_INFO(this->get_logger(), "Multirotor client initialized for: %s", vehicle_name_.c_str());
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "multirotor client initialization");
        throw;  // Re-throw to prevent further initialization
    }
}

void MultirotorNode::setup_sensor_publishers()
{
    // Ultra-clean topic prefixing: vehicle_name + "/" + sensor_name + "/" + data_type
    std::string topic_prefix = vehicle_name_ + "/";

    // Camera publishers (keeping existing functionality)
    for (int i =  0; i < 4; ++i) {
        auto camera_pub = this->create_publisher<sensor_msgs::msg::Image>(topic_prefix + "camera" + std::to_string(i) + "/image", 10);
        legacy_camera_pubs_.push_back(camera_pub);

        auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic_prefix + "camera" + std::to_string(i) + "/camera_info", 10); 
        legacy_camera_info_pubs_.push_back(camera_info_pub); 
    }

    // Fallback sensor publishers (in case settings.json doesn't define sensors)
    // These will be used if the settings-driven approach doesn't find configured sensors
    
    // Check if settings-driven approach created publishers
    bool has_settings_imu = !imu_pubs_.empty();
    bool has_settings_mag = !magnetometer_pubs_.empty();
    bool has_settings_baro = !barometer_pubs_.empty();
    
    RCLCPP_INFO(this->get_logger(), "Settings-driven sensors found: IMU=%s, Mag=%s, Baro=%s", 
                has_settings_imu ? "Yes" : "No",
                has_settings_mag ? "Yes" : "No", 
                has_settings_baro ? "Yes" : "No");
    
    // Create fallback publishers if settings-driven approach didn't create them
    if (!has_settings_imu || !has_settings_mag || !has_settings_baro) {
        RCLCPP_WARN(this->get_logger(), 
                   "Some sensors not found in settings.json for vehicle %s. Creating fallback publishers.",
                   vehicle_name_.c_str());
                   
        if (!has_settings_imu) {
            auto imu_publisher = this->create_sensor_publisher<sensor_msgs::msg::Imu>(
                "IMU sensor (fallback)", "Imu", msr::airlib::SensorBase::SensorType::Imu,
                topic_prefix + "Imu", 10);
            imu_pubs_.emplace_back(imu_publisher);
            RCLCPP_INFO(this->get_logger(), "Created fallback IMU publisher: %sImu", topic_prefix.c_str());
        }
        
        if (!has_settings_mag) {
            auto mag_publisher = this->create_sensor_publisher<sensor_msgs::msg::MagneticField>(
                "Magnetometer sensor (fallback)", "Magnetometer", msr::airlib::SensorBase::SensorType::Magnetometer,
                topic_prefix + "Magnetometer", 10);
            magnetometer_pubs_.emplace_back(mag_publisher);
            RCLCPP_INFO(this->get_logger(), "Created fallback magnetometer publisher: %sMagnetometer", topic_prefix.c_str());
        }
        
        if (!has_settings_baro) {
            auto baro_publisher = this->create_sensor_publisher<airsim_interfaces::msg::Altimeter>(
                "Barometer sensor (fallback)", "Barometer", msr::airlib::SensorBase::SensorType::Barometer,
                topic_prefix + "Barometer", 10);
            barometer_pubs_.emplace_back(baro_publisher);
            RCLCPP_INFO(this->get_logger(), "Created fallback barometer publisher: %sBarometer", topic_prefix.c_str());
        }
    }
}

void MultirotorNode::setup_vehicle_publishers()
{
    // Base class already sets up odom_pub_, gps_pub_, env_pub_
    
    // Call multirotor-specific sensor publisher setup
    // setup_sensor_publishers();
    
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle publishers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_control_subscribers()
{
    // Ultra-clean topic prefixing: vehicle_name + "/" + topic
    std::string topic_prefix = vehicle_name_ + "/";
    
    vel_cmd_body_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        topic_prefix + "vel_cmd_body_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_body_callback, this, std::placeholders::_1));
        
    vel_cmd_world_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        topic_prefix + "vel_cmd_world_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_world_callback, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "Setup multirotor control subscribers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_control_services()
{
    // Ultra-clean topic prefixing: vehicle_name + "/" + service
    std::string topic_prefix = vehicle_name_ + "/";
    
    takeoff_service_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        topic_prefix + "takeoff",
        std::bind(&MultirotorNode::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    land_service_ = this->create_service<airsim_interfaces::srv::Land>(
        topic_prefix + "land",
        std::bind(&MultirotorNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    RCLCPP_INFO(this->get_logger(), "Setup multirotor control services for: %s", vehicle_name_.c_str());
}

// Implementation of required virtual method from base class
nav_msgs::msg::Odometry MultirotorNode::get_vehicle_odometry()
{
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
    curr_drone_state_ = multirotor_client->getMultirotorState(vehicle_name_);
    return get_odom_from_multirotor_state(curr_drone_state_);
}

// Implementation of LiDAR data access method (provides client access to base class)
msr::airlib::LidarData MultirotorNode::get_lidar_data_for_sensor(const std::string& sensor_name, const std::string& vehicle_name)
{
    RCLCPP_INFO(this->get_logger(), "VIRTUAL METHOD CALLED! get_lidar_data_for_sensor(%s, %s)", 
                sensor_name.c_str(), vehicle_name.c_str());
                
    try {
        if (!airsim_client_) {
            RCLCPP_ERROR(this->get_logger(), "❌ AirSim client is null!");
            return msr::airlib::LidarData{};
        }
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        if (!multirotor_client) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to cast to MultirotorRpcLibClient!");
            return msr::airlib::LidarData{};
        }
        
        RCLCPP_INFO(this->get_logger(), "🔌 Calling AirSim getLidarData(%s, %s)...", 
                    sensor_name.c_str(), vehicle_name.c_str());
        
        auto lidar_data = multirotor_client->getLidarData(sensor_name, vehicle_name);
        
        RCLCPP_INFO(this->get_logger(), "📈 AirSim returned LiDAR data: %zu points, timestamp: %lu, pose: [%.2f, %.2f, %.2f]", 
                    lidar_data.point_cloud.size(), lidar_data.time_stamp,
                    lidar_data.pose.position.x(), lidar_data.pose.position.y(), lidar_data.pose.position.z());
        
        return lidar_data;
        
    } catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ RPC error in get_lidar_data_for_sensor: %s", e.what());
        return msr::airlib::LidarData{};
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception in get_lidar_data_for_sensor: %s", e.what());
        return msr::airlib::LidarData{};
    }
}

void MultirotorNode::update_vehicle_state()
{
    try {
        // Get multirotor-specific state first
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        curr_drone_state_ = multirotor_client->getMultirotorState(vehicle_name_);
        
        // Update odometry using vehicle-specific implementation (with proper position conversion)
        curr_odom_ = get_vehicle_odometry();
        std::string topic_prefix = vehicle_name_ + "/";

        // REP 105 COMPLIANT: Use per-vehicle frames directly (base class provides proper namespacing)
        curr_odom_.header.frame_id = odom_frame_id_;        // drone_1/odom (already namespaced by base class)
        curr_odom_.child_frame_id = base_link_frame_id_;    // drone_1/base_link1 (already namespaced by base class)
        curr_odom_.header.stamp = rclcpp::Time(curr_drone_state_.timestamp);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "REP 105 Odometry: %s → %s (standard ROS structure)", 
            curr_odom_.header.frame_id.c_str(), curr_odom_.child_frame_id.c_str());
        
        // Get environment data (common functionality, but using multirotor client)
        auto env_data = multirotor_client->simGetGroundTruthEnvironment(vehicle_name_);
        env_msg_.header.stamp = this->get_clock()->now();
        env_msg_.header.frame_id = base_link_frame_id_;  // Use REP 105 base_link frame
        env_msg_.position.x = env_data.position.x();
        env_msg_.position.y = env_data.position.y();
        env_msg_.position.z = env_data.position.z();
        env_msg_.air_pressure = env_data.air_pressure;
        env_msg_.temperature = env_data.temperature;
        env_msg_.air_density = env_data.air_density;
        
        // GPS
        auto gps_data = multirotor_client->getGpsData("", vehicle_name_);
        gps_sensor_msg_.header.stamp = rclcpp::Time(gps_data.time_stamp);
        gps_sensor_msg_.header.frame_id = base_link_frame_id_;  // Use REP 105 base_link frame
        gps_sensor_msg_.latitude = gps_data.gnss.geo_point.latitude;
        gps_sensor_msg_.longitude = gps_data.gnss.geo_point.longitude;
        gps_sensor_msg_.altitude = gps_data.gnss.geo_point.altitude;

        // Note: Sensor data (IMU, magnetometer, barometer) is now handled 
        // by the settings-driven approach in publish_sensor_data()

    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "multirotor state update");
    }
}

void MultirotorNode::publish_vehicle_state()
{
    // Publish odometry (multirotor-specific)
    odom_pub_->publish(curr_odom_);
    publish_odometry_tf(curr_odom_);
    
    // Publish GPS (multirotor-specific)
    gps_pub_->publish(gps_sensor_msg_);
    
    // Publish environment (multirotor-specific)
    env_pub_->publish(env_msg_);
    
    // Use base class implementation for sensor data publishing (has better error handling and debug logging)
    VehicleNodeBase::publish_sensor_data();
}

void MultirotorNode::process_vehicle_commands()
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
    
    // DIAGNOSTIC: Log raw AirSim position data (before conversion)
    auto raw_pos = kinematics.pose.position;
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //     "Vehicle %s - Raw AirSim position (NED): [%.3f, %.3f, %.3f]", 
    //     vehicle_name_.c_str(), raw_pos.x(), raw_pos.y(), raw_pos.z());
    
    // Initialize spawn offset on first call using GPS position
    if (!spawn_offset_initialized_) {
        try {
            auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            auto gps_data = multirotor_client->getGpsData("", vehicle_name_);
            auto origin_gps = multirotor_client->getHomeGeoPoint("");
            
            // Calculate spawn offset from GPS coordinates
            double lat_diff = gps_data.gnss.geo_point.latitude - origin_gps.latitude;
            double lon_diff = gps_data.gnss.geo_point.longitude - origin_gps.longitude;
            double alt_diff = gps_data.gnss.geo_point.altitude - origin_gps.altitude;
            
            // Convert to local NED meters
            double lat_rad = origin_gps.latitude * M_PI / 180.0;
            double ned_x = lat_diff * 111320.0;
            double ned_y = lon_diff * 111320.0 * std::cos(lat_rad);
            double ned_z = -alt_diff;
            
            // Store spawn offset (already in NED, will convert to ENU when used)
            spawn_offset_x_ = ned_x;
            spawn_offset_y_ = ned_y;
            spawn_offset_z_ = ned_z;
            spawn_offset_initialized_ = true;
            
            // RCLCPP_INFO(this->get_logger(), 
            //     "Vehicle %s spawn offset initialized: NED [%.3f, %.3f, %.3f]", 
            //     vehicle_name_.c_str(), spawn_offset_x_, spawn_offset_y_, spawn_offset_z_);
                
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to initialize spawn offset for %s: %s", vehicle_name_.c_str(), e.what());
            // Leave spawn offset at zero and mark as initialized to avoid repeated attempts
            spawn_offset_initialized_ = true;
        }
    }
    
    // Check if local position is valid (non-zero for grounded vehicles)
    bool local_position_valid = (std::abs(raw_pos.x()) > 0.001 || 
                                std::abs(raw_pos.y()) > 0.001 || 
                                std::abs(raw_pos.z()) > 0.001);
    
    if (local_position_valid) {
        // Use local position data + spawn offset (for flying vehicles)
        double local_x = kinematics.pose.position.x() + spawn_offset_x_;
        double local_y = kinematics.pose.position.y() + spawn_offset_y_;
        double local_z = kinematics.pose.position.z() + spawn_offset_z_;
        
        // Convert NED to ENU
        odom_msg.pose.pose.position.x = local_x;
        odom_msg.pose.pose.position.y = -local_y;
        odom_msg.pose.pose.position.z = -local_z;
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Vehicle %s using LOCAL position + spawn offset", vehicle_name_.c_str());
    } else {
        // Fallback to GPS-derived position (for grounded vehicles)
        try {
            auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            auto gps_data = multirotor_client->getGpsData("", vehicle_name_);
            auto origin_gps = multirotor_client->getHomeGeoPoint("");
            
            // Convert GPS coordinates to local NED position relative to origin
            double lat_diff = gps_data.gnss.geo_point.latitude - origin_gps.latitude;
            double lon_diff = gps_data.gnss.geo_point.longitude - origin_gps.longitude;
            double alt_diff = gps_data.gnss.geo_point.altitude - origin_gps.altitude;
            
            // Convert lat/lon differences to meters (approximate)
            // 1 degree latitude ≈ 111,320 meters
            // 1 degree longitude ≈ 111,320 * cos(latitude) meters
            double lat_rad = origin_gps.latitude * M_PI / 180.0;
            double ned_x = lat_diff * 111320.0;  // North (positive X in NED)
            double ned_y = lon_diff * 111320.0 * std::cos(lat_rad);  // East (positive Y in NED)
            double ned_z = -alt_diff;  // Down (negative Z in NED for higher altitude)
            
            // Convert NED to ENU
            odom_msg.pose.pose.position.x = ned_x;
            odom_msg.pose.pose.position.y = -ned_y;
            odom_msg.pose.pose.position.z = -ned_z;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Vehicle %s using GPS-derived position: [%.3f, %.3f, %.3f]", 
                vehicle_name_.c_str(), odom_msg.pose.pose.position.x, 
                odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
                
        } catch (const std::exception& e) {
            // GPS fallback failed, use zero position
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Vehicle %s: GPS fallback failed (%s), using zero position", 
                vehicle_name_.c_str(), e.what());
            
            odom_msg.pose.pose.position.x = 0.0;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.position.z = 0.0;
        }
    }
    
    // DIAGNOSTIC: Log converted position data (after NED→ENU)
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //     "Vehicle %s - Converted position (ENU): [%.3f, %.3f, %.3f]", 
    //     vehicle_name_.c_str(), odom_msg.pose.pose.position.x, 
    //     odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    
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
