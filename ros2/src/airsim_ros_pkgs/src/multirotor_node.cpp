/*
* AirSim Multirotor Node
* 
* PURPOSE: Individual drone control and sensor data interface for single multirotor vehicle
*
* MAIN FUNCTIONALITY:
* - Individual Vehicle Control: Handles commands for a specific drone (takeoff, land, velocity)
* - Sensor Data Publishing: Streams camera, LiDAR, IMU, GPS, magnetometer, barometer data
* - State Management: Publishes odometry, pose and vehicle status information
* - Real-time Communication: Maintains continuous connection with AirSim for one vehicle
* - Motion-based Target Detection: Uses AI vision to detect moving objects as targets
*
*
* DIFFERENCE FROM COORDINATION NODE:
* - Coordination Node: Controls MULTIPLE drones simultaneously (fleet management)
* - Multirotor Node: Controls ONE specific drone (individual vehicle interface)
*
* TYPICAL SETUP:
* - One Multirotor Node per drone in the simulation
* - Each node manages its own sensors, commands and state publishing
* - Coordination Node can command all Multirotor Nodes simultaneously
*
* ROS INTERFACES
* Publishers: camera images, lidar points, imu data, gps, odometry, environment
* Subscribers: velocity commands (body frame and world frame), motion detection
* Services: individual takeoff, individual land, search target, track target
*/

#include "multirotor_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include "common/common_utils/Utils.hpp"
#include <airsim_interfaces/msg/target_detection.hpp>
#include <airsim_interfaces/srv/search_target.hpp>
#include <airsim_interfaces/srv/track_target.hpp>

using namespace msr::airlib;

MultirotorNode::MultirotorNode(const std::string& vehicle_name, 
                             const std::string& host_ip, 
                             uint16_t host_port)
    : VehicleNodeBase(vehicle_name, host_ip, host_port)
    , static_transforms_published_(false)
    , stamp_(this->get_clock()->now())
{
    initialize_vehicle_client();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    setup_sensor_publishers();
    setup_vehicle_publishers();
    setup_vehicle_subscribers();
    setup_vehicle_services();
    
    // Add motion detection subscription
    motion_detection_sub_ = this->create_subscription<airsim_interfaces::msg::TargetDetection>(
        "target_detection", 10,
        std::bind(&MultirotorNode::motion_detection_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            stamp_ = this->get_clock()->now();
            
            try {
                update_vehicle_state();
                publish_vehicle_state();
                publish_tf_data();
                handle_vehicle_commands();
                
                if (!static_transforms_published_) {
                    publish_static_transforms();
                    static_transforms_published_ = true;
                }
                
                // Process sensor data
                process_images();
                process_lidar();
                process_gpulidar();
                process_echo();
                
                publish_imu_data();
                publish_magnetometer_data();
                publish_barometer_data();
                publish_gps_data();
                publish_environment_data();
                publish_system_status();
                
            } catch (const rpc::rpc_error& e) {
                handle_rpc_error(e, "main timer callback");
            }
        }
    );

    initialize_common();
    
    RCLCPP_INFO(this->get_logger(), "Multirotor node created for: %s", vehicle_name_.c_str());
}

void MultirotorNode::initialize_vehicle_client()
{
    try {
        airsim_client_.reset(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_images_.reset(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        airsim_client_lidar_.reset(new msr::airlib::MultirotorRpcLibClient(host_ip_, host_port_));
        
        airsim_client_->confirmConnection();
        airsim_client_images_->confirmConnection();
        airsim_client_lidar_->confirmConnection();
        
        RCLCPP_INFO(this->get_logger(), "Connected to AirSim for vehicle: %s", vehicle_name_.c_str());
    }
    catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to AirSim: %s", e.what());
        throw;
    }
}

void MultirotorNode::setup_sensor_publishers()
{
    // Camera publishers
    for (int i = 0; i < 4; ++i) {
        auto image_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "camera" + std::to_string(i) + "/image", 10);
        image_pubs_.push_back(image_pub);
        
        auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera" + std::to_string(i) + "/camera_info", 10);
        camera_info_pubs_.push_back(camera_info_pub);
    }

    // LiDAR publisher
    for (int i = 0; i < 1; ++i) {
        auto lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar" + std::to_string(i), 10);
        lidar_pubs_.push_back(lidar_pub);
    }

    // Other sensor publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10); 
    baro_pub_ = this->create_publisher<sensor_msgs::msg::Range>("baro", 10);
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>("environment", 10);
    target_detection_pub_ = this->create_publisher<airsim_interfaces::msg::TargetDetection>(
        "target_detection", 10);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("system_status", 10);
}

void MultirotorNode::setup_vehicle_publishers()
{
    // Vehicle status and state publishers can be added here if needed
}

void MultirotorNode::setup_vehicle_subscribers()
{
    vel_cmd_body_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_body_frame", 10,
        std::bind(&MultirotorNode::vel_cmd_body_frame_callback, this, std::placeholders::_1));

    vel_cmd_world_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_world_frame", 10,
        std::bind(&MultirotorNode::vel_cmd_world_frame_callback, this, std::placeholders::_1));
}

void MultirotorNode::setup_vehicle_services()
{
    takeoff_srv_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        "takeoff", std::bind(&MultirotorNode::takeoff_callback, this,
                           std::placeholders::_1, std::placeholders::_2));
    
    land_srv_ = this->create_service<airsim_interfaces::srv::Land>(
        "land", std::bind(&MultirotorNode::land_callback, this,
                         std::placeholders::_1, std::placeholders::_2));
    
    gps_waypoint_srv_ = this->create_service<airsim_interfaces::srv::GpsWaypoint>(
        "gps_waypoint", std::bind(&MultirotorNode::gps_waypoint_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
    
    search_target_srv_ = this->create_service<airsim_interfaces::srv::SearchTarget>(
        "search_target", std::bind(&MultirotorNode::search_target_callback, this,
                                  std::placeholders::_1, std::placeholders::_2));
    
    track_target_srv_ = this->create_service<airsim_interfaces::srv::TrackTarget>(
        "track_target", std::bind(&MultirotorNode::track_target_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

bool MultirotorNode::takeoff_callback(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    (void) request;
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        // Use default altitude since Takeoff service doesn't have altitude field
        float takeoff_altitude = 3.0f; // Default 3 meters
        
        RCLCPP_INFO(this->get_logger(), "Takeoff request for %s at default altitude %.2f", 
                   vehicle_name_.c_str(), takeoff_altitude);
        
        multirotor_client->enableApiControl(true, vehicle_name_);
        multirotor_client->armDisarm(true, vehicle_name_);
        
        auto takeoff_future = multirotor_client->takeoffAsync(60.0f, vehicle_name_);
        takeoff_future->waitOnLastTask();
        
        // Move to default altitude
        auto move_future = multirotor_client->moveToZAsync(-takeoff_altitude, 2.0f, 60.0f,
                                                            msr::airlib::YawMode(false, 0), 
                                                            -1, 1, vehicle_name_);
        move_future->waitOnLastTask();
        
        response->success = true;
        response->message = "Takeoff successful for " + vehicle_name_;
        
        RCLCPP_INFO(this->get_logger(), "Takeoff completed for %s", vehicle_name_.c_str());
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Takeoff failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Takeoff failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultirotorNode::land_callback(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request>,
    std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        RCLCPP_INFO(this->get_logger(), "Land request for %s", vehicle_name_.c_str());
        
        auto land_future = multirotor_client->landAsync(60.0f, vehicle_name_);
        land_future->waitOnLastTask();
        
        multirotor_client->armDisarm(false, vehicle_name_);
        multirotor_client->enableApiControl(false, vehicle_name_);
        
        response->success = true;
        response->message = "Landing successful for " + vehicle_name_;
        
        RCLCPP_INFO(this->get_logger(), "Landing completed for %s", vehicle_name_.c_str());
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Landing failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Landing failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

void MultirotorNode::vel_cmd_body_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vel_cmd_mutex_);
    
    // VelCmd message doesn't have header, so skip header check
    vel_cmd_body_frame_ = *msg;
    has_new_vel_cmd_body_frame_ = true;
}

void MultirotorNode::vel_cmd_world_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vel_cmd_mutex_);
    
    // VelCmd message doesn't have header, so skip header check
    vel_cmd_world_frame_ = *msg;
    has_new_vel_cmd_world_frame_ = true;
}

void MultirotorNode::motion_detection_callback(const airsim_interfaces::msg::TargetDetection::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(motion_target_mutex_);
    
    // Update current motion target info
    current_motion_target_.x = msg->target_x;
    current_motion_target_.y = msg->target_y;
    current_motion_target_.z = msg->target_z;
    current_motion_target_.confidence = msg->confidence;
    current_motion_target_.last_seen = std::chrono::steady_clock::now();
    
    RCLCPP_DEBUG(this->get_logger(), "Moving target detected by %s: (%.2f, %.2f, %.2f) confidence=%.2f",
                vehicle_name_.c_str(), msg->target_x, msg->target_y, msg->target_z, msg->confidence);
    
    // Re-publish for coordination node
    target_detection_pub_->publish(*msg);
}

void MultirotorNode::update_vehicle_state()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        vehicle_state_ = multirotor_client->getMultirotorState(vehicle_name_);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "update_vehicle_state");
    }
}

void MultirotorNode::publish_static_transforms()
{
    try {
        auto transform_stamped = geometry_msgs::msg::TransformStamped();
        transform_stamped.header.stamp = stamp_;
        transform_stamped.header.frame_id = "world_ned";
        transform_stamped.child_frame_id = vehicle_name_;
        
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;
        
        static_tf_broadcaster_->sendTransform(transform_stamped);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing static transforms: %s", e.what());
    }
}

void MultirotorNode::publish_system_status()
{
    try {
        auto status_msg = std_msgs::msg::String();
        
        std::string status = "Vehicle: " + vehicle_name_ + " | ";
        status += "State: " + std::to_string(static_cast<int>(vehicle_state_.landed_state)) + " | ";
        status += "Armed: ";
        status += (vehicle_state_.kinematics_estimated.pose.position.z() < -0.1 ? "Yes" : "No");
        status += " | ";
        status += "GPS: " + std::to_string(vehicle_state_.gps_location.latitude) + ", " + 
                 std::to_string(vehicle_state_.gps_location.longitude);
        
        status_msg.data = status;
        system_status_pub_->publish(status_msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing system status: %s", e.what());
    }
}

void MultirotorNode::publish_vehicle_state()
{
    try {
        auto odom_msg = get_odom_from_multirotor_state(vehicle_state_);
        odom_msg.header.stamp = stamp_;
        odom_msg.header.frame_id = "world_ned";
        odom_msg.child_frame_id = vehicle_name_;
        
        odom_pub_->publish(odom_msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing vehicle state: %s", e.what());
    }
}

void MultirotorNode::publish_imu_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto imu_data = multirotor_client->getImuData("Imu", vehicle_name_);
        
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp_;
        imu_msg.header.frame_id = vehicle_name_ + "/imu";
        
        imu_msg.orientation.x = imu_data.orientation.x();
        imu_msg.orientation.y = imu_data.orientation.y();
        imu_msg.orientation.z = imu_data.orientation.z();
        imu_msg.orientation.w = imu_data.orientation.w();
        
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();
        
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z();
        
        imu_pub_->publish(imu_msg);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "IMU data processing");
    }
}

void MultirotorNode::publish_magnetometer_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto mag_data = multirotor_client->getMagnetometerData("Magnetometer", vehicle_name_);
        
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = stamp_;
        mag_msg.header.frame_id = vehicle_name_ + "/magnetometer";
        
        mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
        mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
        mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
        
        mag_pub_->publish(mag_msg);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "magnetometer data processing");
    }
}

void MultirotorNode::publish_barometer_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto baro_data = multirotor_client->getBarometerData("Barometer", vehicle_name_);
        
        sensor_msgs::msg::Range baro_msg;
        baro_msg.header.stamp = stamp_;
        baro_msg.header.frame_id = vehicle_name_ + "/barometer";
        baro_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        baro_msg.field_of_view = 0.0;
        baro_msg.min_range = 0.0;
        baro_msg.max_range = 1000.0;
        baro_msg.range = baro_data.altitude;
        
        baro_pub_->publish(baro_msg);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "barometer data processing");
    }
}

void MultirotorNode::publish_gps_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto gps_data = multirotor_client->getGpsData("Gps", vehicle_name_);
        
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = stamp_;
        gps_msg.header.frame_id = vehicle_name_ + "/gps";
        
        gps_msg.latitude = gps_data.gnss.geo_point.latitude;
        gps_msg.longitude = gps_data.gnss.geo_point.longitude;
        gps_msg.altitude = gps_data.gnss.geo_point.altitude;
        
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        gps_pub_->publish(gps_msg);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "GPS data processing");
    }
}

void MultirotorNode::publish_environment_data()
{
    try {
        airsim_interfaces::msg::Environment env_msg;
        env_msg.header.stamp = stamp_;
        env_msg.header.frame_id = vehicle_name_;
        
        // Get environmental data if available
        env_msg.position.x = vehicle_state_.kinematics_estimated.pose.position.x();
        env_msg.position.y = vehicle_state_.kinematics_estimated.pose.position.y();
        env_msg.position.z = vehicle_state_.kinematics_estimated.pose.position.z();
        
        env_msg.geo_point.latitude = vehicle_state_.gps_location.latitude;
        env_msg.geo_point.longitude = vehicle_state_.gps_location.longitude;
        env_msg.geo_point.altitude = vehicle_state_.gps_location.altitude;
        
        env_pub_->publish(env_msg);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "environment data processing");
    }
}

void MultirotorNode::publish_tf_data()
{
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp_;
        transform_stamped.header.frame_id = "world_ned";
        transform_stamped.child_frame_id = vehicle_name_;
        
        const auto& pos = vehicle_state_.kinematics_estimated.pose.position;
        const auto& ori = vehicle_state_.kinematics_estimated.pose.orientation;
        
        transform_stamped.transform.translation.x = pos.x();
        transform_stamped.transform.translation.y = pos.y();
        transform_stamped.transform.translation.z = pos.z();
        
        transform_stamped.transform.rotation.x = ori.x();
        transform_stamped.transform.rotation.y = ori.y();
        transform_stamped.transform.rotation.z = ori.z();
        transform_stamped.transform.rotation.w = ori.w();
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing TF data: %s", e.what());
    }
}

void MultirotorNode::handle_vehicle_commands()
{
    std::lock_guard<std::mutex> lock(vel_cmd_mutex_);
    
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        if (has_new_vel_cmd_body_frame_) {
            auto vel_cmd = get_airlib_body_vel_cmd(vel_cmd_body_frame_, 
                                                  vehicle_state_.kinematics_estimated.pose.orientation);
            
            multirotor_client->moveByVelocityBodyFrameAsync(
                vel_cmd.x, vel_cmd.y, vel_cmd.z, 
                vel_cmd.duration, 
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom, 
                msr::airlib::YawMode(false, 0), 
                vehicle_name_);
            
            has_new_vel_cmd_body_frame_ = false;
        }
        
        if (has_new_vel_cmd_world_frame_) {
            auto vel_cmd = get_airlib_world_vel_cmd(vel_cmd_world_frame_);
            
            multirotor_client->moveByVelocityAsync(
                vel_cmd.x, vel_cmd.y, vel_cmd.z,
                vel_cmd.duration,
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                msr::airlib::YawMode(false, 0),
                vehicle_name_);
            
            has_new_vel_cmd_world_frame_ = false;
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "vehicle command handling");
    }
}

bool MultirotorNode::gps_waypoint_callback(
    const std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Request> request,
    std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Response> response)
{
    try {
        if (!validate_gps_coordinates(request->latitude, request->longitude)) {
            response->success = false;
            response->message = "Invalid GPS coordinates";
            return true;
        }
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        // Get current GPS position as home
        auto current_gps = vehicle_state_.gps_location;
        
        // Convert GPS to NED coordinates
        auto ned_coords = gps_to_ned(request->latitude, request->longitude, 
                                    current_gps.latitude, current_gps.longitude);
        
        RCLCPP_INFO(this->get_logger(), "GPS waypoint for %s: (%.6f, %.6f) -> NED (%.2f, %.2f)", 
                   vehicle_name_.c_str(), request->latitude, request->longitude, 
                   ned_coords.first, ned_coords.second);
        
        float altitude = request->altitude > 0 ? -request->altitude : vehicle_state_.kinematics_estimated.pose.position.z();
        
        auto move_task = multirotor_client->moveToPositionAsync(
            ned_coords.first, ned_coords.second, altitude,
            2.0f, // Use default velocity since GpsWaypoint service doesn't have velocity field
            60.0f,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(false, 0),
            -1, 1, vehicle_name_);
        
        move_task->waitOnLastTask();
        
        response->success = true;
        response->message = "GPS waypoint reached successfully";
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "GPS waypoint failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "GPS waypoint error for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

std::pair<double, double> MultirotorNode::gps_to_ned(double lat, double lon, double home_lat, double home_lon)
{
    const double R = 6378137.0; // Earth radius in meters

    double lat1 = home_lat * M_PI / 180.0;
    double lon1 = home_lon * M_PI / 180.0;
    double lat2 = lat * M_PI / 180.0;
    double lon2 = lon * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double north = dlat * R;
    double east = dlon * R * cos(lat1);

    return {north, east};
}

bool MultirotorNode::validate_gps_coordinates(double lat, double lon)
{
    return (lat >= -90.0 && lat <= 90.0) && (lon >= -180.0 && lon <= 180.0);
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg)
{
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.duration = 1.0f; // Default duration since VelCmd doesn't have duration field
    return vel_cmd;
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                                               const msr::airlib::Quaternionr& orientation)
{
    VelCmd vel_cmd;
    
    // Transform velocity from body frame to world frame
    msr::airlib::Vector3r body_vel(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    msr::airlib::Vector3r world_vel = VectorMath::transformToWorldFrame(body_vel, orientation);
    
    vel_cmd.x = world_vel.x();
    vel_cmd.y = world_vel.y();
    vel_cmd.z = world_vel.z();
    vel_cmd.duration = 1.0f; // Default duration since VelCmd doesn't have duration field
    
    return vel_cmd;
}

nav_msgs::msg::Odometry MultirotorNode::get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state)
{
    nav_msgs::msg::Odometry odom_msg;
    
    // Position
    odom_msg.pose.pose.position.x = state.kinematics_estimated.pose.position.x();
    odom_msg.pose.pose.position.y = state.kinematics_estimated.pose.position.y();
    odom_msg.pose.pose.position.z = state.kinematics_estimated.pose.position.z();
    
    // Orientation
    odom_msg.pose.pose.orientation.x = state.kinematics_estimated.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = state.kinematics_estimated.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = state.kinematics_estimated.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = state.kinematics_estimated.pose.orientation.w();
    
    // Linear velocity
    odom_msg.twist.twist.linear.x = state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = state.kinematics_estimated.twist.linear.z();
    
    // Angular velocity
    odom_msg.twist.twist.angular.x = state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = state.kinematics_estimated.twist.angular.z();
    
    return odom_msg;
}

void MultirotorNode::process_images()
{
    try {
        auto multirotor_client_images = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_images_.get());
        
        // Only use the default camera (camera 0)
        if (!image_pubs_.empty()) {
            try {
                std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = {
                    msr::airlib::ImageCaptureBase::ImageRequest("0", 
                                                              msr::airlib::ImageCaptureBase::ImageType::Scene, 
                                                              false, false)
                };
                
                auto responses = multirotor_client_images->simGetImages(requests, vehicle_name_);
                
                if (!responses.empty() && responses[0].image_data_uint8.size() > 0) {
                    sensor_msgs::msg::Image image_msg;
                    image_msg.header.stamp = stamp_;
                    image_msg.header.frame_id = vehicle_name_ + "/camera0";
                    image_msg.height = responses[0].height;
                    image_msg.width = responses[0].width;
                    image_msg.encoding = "rgb8";
                    image_msg.step = image_msg.width * 3;
                    image_msg.data = responses[0].image_data_uint8;
                    
                    image_pubs_[0]->publish(image_msg);
                    
                    // Publish camera info
                    sensor_msgs::msg::CameraInfo camera_info;
                    camera_info.header = image_msg.header;
                    camera_info.width = image_msg.width;
                    camera_info.height = image_msg.height;
                    
                    camera_info_pubs_[0]->publish(camera_info);
                }
                
            } catch (const rpc::rpc_error& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Camera error: %s", e.what());
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Error in image processing: %s", e.what());
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
            
            lidar_msg.height = 1;
            lidar_msg.width = lidar_data.point_cloud.size() / 3;
            lidar_msg.is_bigendian = false;
            lidar_msg.point_step = 12;
            lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;
            lidar_msg.is_dense = true;
            
            sensor_msgs::msg::PointField field_x, field_y, field_z;
            field_x.name = "x"; field_x.offset = 0; field_x.datatype = sensor_msgs::msg::PointField::FLOAT32; field_x.count = 1;
            field_y.name = "y"; field_y.offset = 4; field_y.datatype = sensor_msgs::msg::PointField::FLOAT32; field_y.count = 1;
            field_z.name = "z"; field_z.offset = 8; field_z.datatype = sensor_msgs::msg::PointField::FLOAT32; field_z.count = 1;
            
            lidar_msg.fields = {field_x, field_y, field_z};
            
            lidar_msg.data.resize(lidar_msg.row_step);
            std::memcpy(lidar_msg.data.data(), lidar_data.point_cloud.data(), lidar_msg.row_step);
            
            if (!lidar_pubs_.empty()) {
                lidar_pubs_[0]->publish(lidar_msg);
            }
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "lidar processing");
    }
}

void MultirotorNode::process_gpulidar()
{
    // GPU LiDAR processing if needed
}

void MultirotorNode::process_echo()
{
    // Echo processing if needed
}

bool MultirotorNode::search_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Motion-based search target request for %s: radius=%.1f, time=%.1f, confidence=%.2f", 
               vehicle_name_.c_str(), request->search_radius, request->search_time, request->min_confidence);
    
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        // Check drone readiness
        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        bool api_control_enabled = multirotor_client->isApiControlEnabled(vehicle_name_);
        
        RCLCPP_INFO(this->get_logger(), "Drone %s state - Landed: %d, API Control: %s",
                   vehicle_name_.c_str(), 
                   static_cast<int>(current_state.landed_state),
                   api_control_enabled ? "true" : "false");
        
        if (current_state.landed_state != msr::airlib::LandedState::Flying) {
            response->success = false;
            response->message = "Drone " + vehicle_name_ + " must be airborne before searching.";
            return true;
        }
        
        if (!api_control_enabled) {
            response->success = false;
            response->message = "API control not enabled for " + vehicle_name_;
            return true;
        }
        
        std::lock_guard<std::mutex> lock(motion_target_mutex_);
        
        // Check if we have a recent moving target detection
        auto now = std::chrono::steady_clock::now();
        auto time_since_detection = std::chrono::duration_cast<std::chrono::seconds>(now - current_motion_target_.last_seen);
        
        if (current_motion_target_.confidence >= request->min_confidence && time_since_detection.count() < 5) {
            response->success = true;
            response->target_x = current_motion_target_.x;
            response->target_y = current_motion_target_.y;
            response->target_z = current_motion_target_.z;
            response->confidence = current_motion_target_.confidence;
            response->message = "Moving target already detected by " + vehicle_name_;
            return true;
        }
        
        // **CRITICAL FIX: Get initial position ONCE and use it for all waypoints**
        const auto& start_pos = current_state.kinematics_estimated.pose.position;
        float search_altitude = start_pos.z();
        if (search_altitude > -0.5f) {
            search_altitude = -2.0f;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting motion-based search pattern from position: (%.2f, %.2f, %.2f)",
                   start_pos.x(), start_pos.y(), start_pos.z());
        
        // Much smaller, safer search pattern
        float actual_radius = std::min(request->search_radius * 0.2f, 10.0f); // Even smaller!
        std::vector<std::pair<float, float>> search_offsets = {
            {actual_radius, 0.0f},                                    // East
            {actual_radius * 0.7f, actual_radius * 0.7f},           // Northeast
            {0.0f, actual_radius},                                   // North  
            {-actual_radius * 0.7f, actual_radius * 0.7f},          // Northwest
            {-actual_radius, 0.0f},                                  // West
            {-actual_radius * 0.7f, -actual_radius * 0.7f},         // Southwest
            {0.0f, -actual_radius},                                  // South
            {actual_radius * 0.7f, -actual_radius * 0.7f}           // Southeast
        };
        
        auto search_start_time = std::chrono::steady_clock::now();
        bool moving_target_found = false;
        
        for (size_t i = 0; i < search_offsets.size() && !moving_target_found; ++i) {
            // Check time limit
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - search_start_time);
            
            if (elapsed_time.count() >= request->search_time) {
                RCLCPP_WARN(this->get_logger(), "Motion search time limit exceeded for %s", vehicle_name_.c_str());
                break;
            }
            
            // **CRITICAL FIX: Calculate waypoint from INITIAL position, not current position**
            float search_x = start_pos.x() + search_offsets[i].first; 
            float search_y = start_pos.y() + search_offsets[i].second;
            
            RCLCPP_INFO(this->get_logger(), "Moving to motion search waypoint %zu: (%.2f, %.2f, %.2f)",
                       i + 1, search_x, search_y, search_altitude);
            
            try {
                // Don't cancel tasks - this can cause issues
                // multirotor_client->cancelLastTask(vehicle_name_);
                
                auto task = multirotor_client->moveToPositionAsync(
                    search_x, search_y, search_altitude,
                    1.5f,   // Reasonable speed
                    30.0f,  // Shorter timeout
                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                    msr::airlib::YawMode(false, 0.0f),
                    -1, 1, vehicle_name_);
                
                task->waitOnLastTask();
                RCLCPP_INFO(this->get_logger(), "Reached motion search waypoint %zu", i + 1);
                
                // Hover at position for motion detection
                multirotor_client->hoverAsync(vehicle_name_);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                // Check if moving target detected
                auto detection_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - current_motion_target_.last_seen);
                    
                if (current_motion_target_.confidence >= request->min_confidence && detection_time.count() < 3) {
                    RCLCPP_INFO(this->get_logger(), "Moving target detected during search at waypoint %zu", i + 1);
                    moving_target_found = true;
                    break;
                }
                
            } catch (const rpc::rpc_error& e) {
                RCLCPP_ERROR(this->get_logger(), "RPC error at motion search waypoint %zu: %s", i + 1, e.what());
                try {
                    multirotor_client->hoverAsync(vehicle_name_)->waitOnLastTask();
                    RCLCPP_WARN(this->get_logger(), "Recovered to hover after RPC error");
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to recover from RPC error");
                }
                continue;
            }
        }
        
        // Final response
        if (moving_target_found && current_motion_target_.confidence >= request->min_confidence) {
            response->success = true;
            response->target_x = current_motion_target_.x;
            response->target_y = current_motion_target_.y;
            response->target_z = current_motion_target_.z;
            response->confidence = current_motion_target_.confidence;
            response->message = "Moving target found by " + vehicle_name_;
        } else {
            response->success = false;
            response->target_x = 0.0f;
            response->target_y = 0.0f;
            response->target_z = 0.0f;
            response->confidence = 0.0f;
            response->message = "No moving target found after search pattern by " + vehicle_name_;
        }
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Motion search failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Motion search RPC error for %s: %s", vehicle_name_.c_str(), e.what());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Motion search failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Motion search error for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultirotorNode::track_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> response)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        RCLCPP_INFO(this->get_logger(), "Track target request for %s to position (%.2f, %.2f, %.2f)", 
                   vehicle_name_.c_str(), request->target_x, request->target_y, request->target_z);
        
        float altitude = request->target_z != 0.0f ? request->target_z : 
                        vehicle_state_.kinematics_estimated.pose.position.z();
        
        auto track_task = multirotor_client->moveToPositionAsync(
            request->target_x, request->target_y, altitude,
            2.0f, // Use default velocity since TrackTarget service doesn't have velocity field
            60.0f,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(false, 0),
            -1, 1, vehicle_name_);
        
        track_task->waitOnLastTask();
        
        response->success = true;
        response->message = "Target tracking completed for " + vehicle_name_;
        
        RCLCPP_INFO(this->get_logger(), "Target tracking completed for %s", vehicle_name_.c_str());
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Target tracking failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Target tracking failed for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}