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
* Subscribers: velocity commands (body frame and world frame)
* Services: individual takeoff, individual land
*/

#include "multirotor_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include "common/common_utils/Utils.hpp"

using namespace msr::airlib;

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
    // Camera publishers
    for (int i = 0; i < 4; ++i) {
        auto camera_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "camera" + std::to_string(i) + "/image", 10);
        camera_pubs_.push_back(camera_pub);

        auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera" + std::to_string(i) + "/camera_info", 10); 
        camera_info_pubs_.push_back(camera_info_pub); 
    }

    // LiDAR publisher
    for (int i = 0; i < 1; ++i) {
        auto lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar" + std::to_string(i) + "/points", 10);
        lidar_pubs_.push_back(lidar_pub);
    }

    // Other sensor publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10); 
    baro_pub_ = this->create_publisher<sensor_msgs::msg::Range>("baro", 10);
}

void MultirotorNode::setup_vehicle_publishers()
{
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

    gps_waypoint_service_ = this->create_service<airsim_interfaces::srv::GpsWaypoint>(
        "gps_waypoint",
        std::bind(&MultirotorNode::gps_waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle services for: %s", vehicle_name_.c_str());
}

bool MultirotorNode::takeoff_callback(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    (void)request;
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        multirotor_client->takeoffAsync(20, vehicle_name_)->waitOnLastTask();
        auto state = multirotor_client->getMultirotorState(vehicle_name_);
        bool in_air = state.landed_state == msr::airlib::LandedState::Flying;
        response->success = in_air;
        if (in_air) {
            RCLCPP_INFO(this->get_logger(), "Takeoff successful for: %s", vehicle_name_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Takeoff command completed but vehicle is not in air: %s", vehicle_name_.c_str());
        }
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "takeoff");
        response->success = false;
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff error for %s: %s", vehicle_name_.c_str(), e.what());
        response->success = false;
        return false;
    }
}

bool MultirotorNode::land_callback(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    (void)request;
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        multirotor_client->landAsync(60, vehicle_name_)->waitOnLastTask();
        auto state = multirotor_client->getMultirotorState(vehicle_name_);
        bool is_landed = state.landed_state == msr::airlib::LandedState::Landed;
        response->success = is_landed;
        if (is_landed) {
            RCLCPP_INFO(this->get_logger(), "Landing successful for: %s", vehicle_name_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Landing command completed but vehicle is not landed: %s", vehicle_name_.c_str());
        }
        return true;
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "land");
        response->success = false;
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Landing error for %s: %s", vehicle_name_.c_str(), e.what());
        response->success = false;
        return false;
    }
}

void MultirotorNode::vel_cmd_body_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto state = multirotor_client->getMultirotorState(vehicle_name_);
        auto vel_cmd = get_airlib_body_vel_cmd(*msg, state.kinematics_estimated.pose.orientation);
        multirotor_client->moveByVelocityAsync(
            vel_cmd.x, vel_cmd.y, vel_cmd.z, 0.1f,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            vel_cmd.yaw_mode, vehicle_name_);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "body velocity command");
    }
}

void MultirotorNode::vel_cmd_world_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto vel_cmd = get_airlib_world_vel_cmd(*msg);
        multirotor_client->moveByVelocityAsync(
            vel_cmd.x, vel_cmd.y, vel_cmd.z, 0.1f,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            vel_cmd.yaw_mode, vehicle_name_);
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "world velocity command");
    }
}

void MultirotorNode::update_vehicle_state()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        curr_drone_state_ = multirotor_client->getMultirotorState(vehicle_name_);
        stamp_ = this->get_clock()->now();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "state update");
    }
}

void MultirotorNode::publish_vehicle_state()
{
    try {
        if (airsim_client_) {
            curr_odom_ = get_odom_from_multirotor_state(curr_drone_state_);
            curr_odom_.header.stamp = stamp_;
            curr_odom_.header.frame_id = "world";
            curr_odom_.child_frame_id = vehicle_name_;

            // Process and publish sensor data
            process_images();
            process_lidar();
            process_gpulidar();
            process_echo();

            RCLCPP_DEBUG(this->get_logger(), "Published state for: %s", vehicle_name_.c_str());
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "state publishing");
    }
}

void MultirotorNode::handle_vehicle_commands()
{
    // This method handles incoming vehicle commands
    // Currently a placeholder - implement specific command handling if needed
    RCLCPP_DEBUG(this->get_logger(), "Handling commands for: %s", vehicle_name_.c_str());
}

bool MultirotorNode::gps_waypoint_callback(
    const std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Request> request,
    std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Response> response)
{
    try {
        if (!validate_gps_coordinates(request->latitude, request->longitude)) {
            response->success = false;
            response->message = "Invalid GPS coordinates provided";
            response->final_distance = -1.0;
            return true;
        }

        if (request->altitude <= 0) {
            response->success = false; 
            response->message = "Altitude must be positive (above ground level)";
            response->final_distance = -1.0;
            return true;
        }

        double speed = request->speed > 0 ? request->speed : 5.0;
        double tolerance = request->tolerance > 0 ? request->tolerance : 1.0;

        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());

        auto current_gps = multirotor_client->getGpsData("", vehicle_name_);
        double home_lat = current_gps.gnss.geo_point.latitude;
        double home_lon = current_gps.gnss.geo_point.longitude;

        auto [north_offset, east_offset] = gps_to_ned(request->latitude, request->longitude, home_lat, home_lon);

        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        const auto& current_pos = current_state.kinematics_estimated.pose.position;
        
        double target_x = current_pos.x() + north_offset;
        double target_y = current_pos.y() + east_offset;
        double target_z = -abs(request->altitude);

        RCLCPP_INFO(this->get_logger(),
            "GPS Waypoint for %s: GPS(%.6f, %.6f) -> NED(%.2f, %.2f, %.2f)",
            vehicle_name_.c_str(), request->latitude, request->longitude,
            target_x, target_y, target_z);
        
        if (request->wait_on_last_task) {
            auto task = multirotor_client->moveToPositionAsync(
                target_x, 
                target_y, 
                target_z, 
                speed,
                Utils::max<float>(),
                DrivetrainType::MaxDegreeOfFreedom,
                YawMode(),
                -1,
                1,
                vehicle_name_.c_str()
            );
            
            task->waitOnLastTask();

            auto final_state = multirotor_client->getMultirotorState(vehicle_name_);
            const auto& final_pos = final_state.kinematics_estimated.pose.position;

            double dx = final_pos.x() - target_x; 
            double dy = final_pos.y() - target_y;
            double dz = final_pos.z() - target_z; 
            double final_distance = sqrt(dx*dx + dy*dy + dz*dz);

            response->final_distance = final_distance;
            
            if (final_distance <= tolerance) {
                response->success = true;
                response->message = "Reached GPS waypoint successfully";
                RCLCPP_INFO(this->get_logger(), 
                    "GPS waypoint reached for %s. Final distance is %.2f m", 
                    vehicle_name_.c_str(), final_distance);
            } else {
                response->success = false;
                response->message = "GPS waypoint not reached within tolerance";
                RCLCPP_WARN(this->get_logger(), 
                    "GPS waypoint not reached for %s. Final distance is %.2f m (tolerance: %.2f m)", 
                    vehicle_name_.c_str(), final_distance, tolerance);
            }
        } else {
            multirotor_client->moveToPositionAsync(
                target_x,
                target_y, 
                target_z,
                speed,
                Utils::max<float>(),
                DrivetrainType::MaxDegreeOfFreedom,
                YawMode(),
                -1,
                1,
                vehicle_name_.c_str()
            );
            
            response->success = true;
            response->message = "GPS waypoint command sent (non-blocking)";
            response->final_distance = 0.0;
            RCLCPP_INFO(this->get_logger(), 
                "GPS waypoint command sent for %s (non blocking)", 
                vehicle_name_.c_str());
        }

        return true;
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "GPS waypoint");
        response->success = false; 
        response->message = "RPC error occurred during GPS waypoint mission";
        response->final_distance = -1.0;
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
            "GPS waypoint error for %s: %s", 
            vehicle_name_.c_str(), e.what());
        response->success = false;
        response->message = std::string("GPS waypoint error: ") + e.what();
        response->final_distance = -1.0;
        return false;
    }
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
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0);
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg)
{
    // Convert ROS velocity command to AirSim format in world frame
    // World frame: velocities relative to global coordinate system
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;  
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = msg.twist.angular.z * 180.0 / M_PI; // Convert rad/s to deg/s
    return vel_cmd;
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                                               const msr::airlib::Quaternionr& orientation)
{
    // Convert ROS velocity command to AirSim format (body frame)
    // Body frame: velocities relative to drone's current orientation
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
    // Convert AirSim multirotor state to ROS odometry message
    // Handles coordinate system conversion from AirSim NED to ROS ENU
    nav_msgs::msg::Odometry odom_msg;
    const auto& kinematics = state.kinematics_estimated;
    
    // Position conversion (AirSim NED to ROS ENU)
    odom_msg.pose.pose.position.x = kinematics.pose.position.x();
    odom_msg.pose.pose.position.y = -kinematics.pose.position.y(); // East -> North (flip)
    odom_msg.pose.pose.position.z = -kinematics.pose.position.z(); // Down -> Up (Flip)
    
    // Orientation conversion (AirSim NED to ROS ENU)
    odom_msg.pose.pose.orientation.x = kinematics.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = -kinematics.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = -kinematics.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = kinematics.pose.orientation.w();
    
    // Linear velocity conversion (AirSim NED to ROS ENU)
    odom_msg.twist.twist.linear.x = kinematics.twist.linear.x();
    odom_msg.twist.twist.linear.y = -kinematics.twist.linear.y(); // East -> North (flip)
    odom_msg.twist.twist.linear.z = -kinematics.twist.linear.z(); // Down -> Up (flip)

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

        (void) multirotor_client; // Suppress unused variable warning
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "echo processing");
    }
}