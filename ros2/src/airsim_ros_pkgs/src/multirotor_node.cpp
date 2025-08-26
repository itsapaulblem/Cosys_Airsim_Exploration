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
#include <airsim_interfaces/msg/target_detection.hpp>
#include <airsim_interfaces/srv/search_target.hpp>
#include <airsim_interfaces/srv/track_target.hpp> 

using namespace msr::airlib;

MultirotorNode::MultirotorNode(const std::string& vehicle_name, 
                             const std::string& host_ip, 
                             uint16_t host_port)
    : VehicleNodeBase(vehicle_name, host_ip, host_port)
    , stamp_(this->get_clock()->now())
    , static_transforms_published_(false)
{
    initialize_vehicle_client();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

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
    RCLCPP_DEBUG(this->get_logger(), "Setting up vehicle publishers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_subscribers()
{
    vel_cmd_body_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_body_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_body_frame_callback, this, std::placeholders::_1));
        
    vel_cmd_world_frame_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        "vel_cmd_world_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_world_frame_callback, this, std::placeholders::_1));
        
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

    search_target_service_ = this->create_service<airsim_interfaces::srv::SearchTarget>(
        "search_target",
        std::bind(&MultirotorNode::search_target_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    track_target_service_ = this->create_service<airsim_interfaces::srv::TrackTarget>(
        "track_target", 
        std::bind(&MultirotorNode::track_target_callback, this, std::placeholders::_1, std::placeholders::_2));
    

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
// subscriber
void MultirotorNode::vel_cmd_body_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
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
// subscriber 
void MultirotorNode::vel_cmd_world_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
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

void MultirotorNode::publish_static_transforms()
{
    if (static_transforms_published_) {
        return;
    }

     try {
        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
        
        // Static transform from world to map (identity transform)
        geometry_msgs::msg::TransformStamped world_to_map;
        world_to_map.header.stamp = this->get_clock()->now();
        world_to_map.header.frame_id = "map";
        world_to_map.child_frame_id = "world";
        world_to_map.transform.translation.x = 0.0;
        world_to_map.transform.translation.y = 0.0;
        world_to_map.transform.translation.z = 0.0;
        world_to_map.transform.rotation.x = 0.0;
        world_to_map.transform.rotation.y = 0.0;
        world_to_map.transform.rotation.z = 0.0;
        world_to_map.transform.rotation.w = 1.0;
        static_transforms.push_back(world_to_map);
        
        // Static transforms for sensor frames relative to vehicle base
        std::vector<std::string> sensor_frames = {
            "imu", "magnetometer", "barometer", "gps", "lidar", 
            "camera0", "camera1", "camera2", "camera3"
        };
        
        for (const auto& sensor : sensor_frames) {
            geometry_msgs::msg::TransformStamped sensor_transform;
            sensor_transform.header.stamp = this->get_clock()->now();
            sensor_transform.header.frame_id = vehicle_name_;
            sensor_transform.child_frame_id = vehicle_name_ + "/" + sensor;
            
            // Set sensor-specific offsets (you can adjust these based on your vehicle configuration)
            if (sensor == "imu") {
                sensor_transform.transform.translation.x = 0.0;
                sensor_transform.transform.translation.y = 0.0;
                sensor_transform.transform.translation.z = 0.0;
            } else if (sensor == "lidar") {
                sensor_transform.transform.translation.x = 0.0;
                sensor_transform.transform.translation.y = 0.0;
                sensor_transform.transform.translation.z = 0.1; // 10cm above base
            } else if (sensor.find("camera") != std::string::npos) {
                // Camera positions (adjust as needed)
                int camera_id = std::stoi(sensor.substr(6)); // Extract number from "cameraN"
                sensor_transform.transform.translation.x = 0.1; // 10cm forward
                sensor_transform.transform.translation.y = (camera_id - 1.5) * 0.05; // Spread cameras
                sensor_transform.transform.translation.z = 0.05; // 5cm up
            } else {
                // Default sensor position
                sensor_transform.transform.translation.x = 0.0;
                sensor_transform.transform.translation.y = 0.0;
                sensor_transform.transform.translation.z = 0.0;
            }
            
            // Identity rotation for all sensors (adjust if needed)
            sensor_transform.transform.rotation.x = 0.0;
            sensor_transform.transform.rotation.y = 0.0;
            sensor_transform.transform.rotation.z = 0.0;
            sensor_transform.transform.rotation.w = 1.0;
            
            static_transforms.push_back(sensor_transform);
        }
        
        // Publish all static transforms
        if (static_tf_broadcaster_) {
            static_tf_broadcaster_->sendTransform(static_transforms);
            static_transforms_published_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                "Published %zu static transforms for %s", 
                static_transforms.size(), vehicle_name_.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Static TF publishing error for %s: %s", 
                   vehicle_name_.c_str(), e.what());
    }
}

void MultirotorNode::publish_system_status()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);

        std_msgs::msg::String status_msg;
        
        // Create status string based on drone state
        std::string status = "Vehicle: " + vehicle_name_ + " | ";
        
        switch(current_state.landed_state) {
            case msr::airlib::LandedState::Landed:
                status += "Status: LANDED";
                break;
            case msr::airlib::LandedState::Flying:
                status += "Status: FLYING";
                break;
            default:
                status += "Status: UNKNOWN";
        }
        
        // Add position info
        const auto& pos = current_state.kinematics_estimated.pose.position;
        status += " | Position: (" + 
                  std::to_string(pos.x()) + ", " + 
                  std::to_string(pos.y()) + ", " + 
                  std::to_string(pos.z()) + ")";
        
        // Add battery info if available (placeholder)
        status += " | Battery: OK | Armed: " + 
                  std::string(multirotor_client->isApiControlEnabled(vehicle_name_) ? "YES" : "NO");
        
        status_msg.data = status;
        
        if (system_status_pub_) {
            system_status_pub_->publish(status_msg);
        }
        
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "system status publishing");
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

            if (odom_pub_) {
                odom_pub_->publish(curr_odom_);
            }
            // Process and publish sensor data
            process_images();
            process_lidar();
            publish_imu_data();
            publish_magnetometer_data();
            publish_barometer_data();
            process_gpulidar();
            process_echo();
            publish_gps_data();
            publish_environment_data();
            publish_tf_data(); 
            publish_system_status();
            publish_static_transforms();

            RCLCPP_DEBUG(this->get_logger(), "Published state for: %s", vehicle_name_.c_str());
        }
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "state publishing");
    }
}

void MultirotorNode::publish_imu_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto imu_data = multirotor_client->getImuData("", vehicle_name_);

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp_;
        imu_msg.header.frame_id = vehicle_name_ + "/imu";

        // Linear acceleration
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z(); 

        // Angular velocity
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

        // Orientation (from vehicle state)
        imu_msg.orientation.x = curr_drone_state_.kinematics_estimated.pose.orientation.x();
        imu_msg.orientation.y = curr_drone_state_.kinematics_estimated.pose.orientation.y();
        imu_msg.orientation.z = curr_drone_state_.kinematics_estimated.pose.orientation.z();
        imu_msg.orientation.w = curr_drone_state_.kinematics_estimated.pose.orientation.w();

        // Set covariance matrices (optional - set to unknown)
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = -1;
            imu_msg.angular_velocity_covariance[i] = -1;
            imu_msg.linear_acceleration_covariance[i] = -1;
        }

        if (imu_pub_) {
            imu_pub_->publish(imu_msg);
        }
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "IMU data publishing");
    }
}

void MultirotorNode::publish_magnetometer_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto mag_data = multirotor_client->getMagnetometerData("", vehicle_name_);

        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = stamp_;
        mag_msg.header.frame_id = vehicle_name_ + "/magnetometer";

        mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
        mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
        mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();

        // Set covariance matrices (optional - set to unknown)
        for (int i = 0; i < 9; ++i) {
            mag_msg.magnetic_field_covariance[i] = -1;
        }

        if (mag_pub_) {
            mag_pub_->publish(mag_msg);
        }
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "Magnetometer data publishing");    
    }
}

void MultirotorNode::publish_barometer_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto baro_data = multirotor_client->getBarometerData("", vehicle_name_);

        sensor_msgs::msg::Range baro_msg;
        baro_msg.header.stamp = stamp_;
        baro_msg.header.frame_id = vehicle_name_ + "/barometer";
        baro_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        baro_msg.field_of_view = 0.0;
        baro_msg.min_range = 0.0;
        baro_msg.max_range = 1000.0;
        baro_msg.range = baro_data.altitude;

        if (baro_pub_) {
            baro_pub_->publish(baro_msg);
        }
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "barometer data publishing");
    }
}


void MultirotorNode::publish_gps_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto gps_data = multirotor_client->getGpsData("", vehicle_name_);

        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = stamp_;
        gps_msg.header.frame_id = vehicle_name_ + "/gps";

        gps_msg.latitude = gps_data.gnss.geo_point.latitude;
        gps_msg.longitude = gps_data.gnss.geo_point.longitude;
        gps_msg.altitude = gps_data.gnss.geo_point.altitude; 

        // Set position covariance (optional)
        gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        for (int i = 0; i < 9; ++i) {
            gps_msg.position_covariance[i] = -1;
        }

        // Set status
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        if (gps_pub_) {
            gps_pub_->publish(gps_msg);
        }
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "GPS data publishing");
    }
}

void MultirotorNode::publish_environment_data()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());

        // Get environment state
        airsim_interfaces::msg::Environment env_msg;
        env_msg.header.stamp  = stamp_;
        env_msg.header.frame_id = "world";

        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        env_msg.position.x = current_state.kinematics_estimated.pose.position.x();
        env_msg.position.y = current_state.kinematics_estimated.pose.position.y();
        env_msg.position.z = current_state.kinematics_estimated.pose.position.z();
        

        // Get barometer data for atmospheric pressure
        auto baro_data = multirotor_client->getBarometerData("", vehicle_name_);
        env_msg.air_pressure = baro_data.pressure;
        env_msg.temperature = 20.0f; 

        if (env_pub_) {
            env_pub_->publish(env_msg);
        }
    } catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "environment data publishing");
    }
}

void MultirotorNode::publish_tf_data()
{
    try {
        tf2_msgs::msg::TFMessage tf_msg;
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = stamp_;
        transform.header.frame_id = "world";
        transform.child_frame_id = vehicle_name_;

        // Set translation from odometry
        transform.transform.translation.x = curr_odom_.pose.pose.position.x;
        transform.transform.translation.y = curr_odom_.pose.pose.position.y;
        transform.transform.translation.z = curr_odom_.pose.pose.position.z;
        
        // Set rotation from odometry
        transform.transform.rotation.x = curr_odom_.pose.pose.orientation.x;
        transform.transform.rotation.y = curr_odom_.pose.pose.orientation.y;
        transform.transform.rotation.z = curr_odom_.pose.pose.orientation.z;
        transform.transform.rotation.w = curr_odom_.pose.pose.orientation.w;
        
        tf_msg.transforms.push_back(transform);
        
        if (tf_pub_) {
            tf_pub_->publish(tf_msg);
        }

        if (tf_broadcaster_) {
            tf_broadcaster_->sendTransform(transform);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "TF publishing error for %s: %s", vehicle_name_.c_str(), e.what());
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
                target_x, target_y, target_z,  // Fixed variable names
                speed,   // velocity
                60.0f,  // timeout_sec
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                msr::airlib::YawMode(false, 0.0f),
                -1,     // lookahead
                1,      // adaptive_lookahead  
                vehicle_name_);
            
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
            } else {
                response->success = false;
                response->message = "GPS waypoint not reached within tolerance";
            }
        } else {
            multirotor_client->moveToPositionAsync(
                target_x, target_y, target_z,
                speed, 60.0f,  // Added timeout
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                msr::airlib::YawMode(),
                -1, 1, vehicle_name_
            );
            
            response->success = true;
            response->message = "GPS waypoint command sent (non-blocking)";
            response->final_distance = 0.0;
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
        request.compress = false; 
        requests.push_back(request);

        auto responses = multirotor_client->simGetImages(requests, vehicle_name_);

        for (size_t i = 0; i < responses.size() && i < camera_pubs_.size(); ++i) {
           const auto& response = responses[i];
            
            // Check if we have valid image data
            if (response.image_data_uint8.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty image data received for camera %zu on %s", i, vehicle_name_.c_str());
                continue;
            }

            // Validate image dimensions
            if (response.width == 0 || response.height == 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid image dimensions %dx%d for camera %zu on %s", 
                           response.width, response.height, i, vehicle_name_.c_str());
                continue;
            }

            sensor_msgs::msg::Image img_msg;
            img_msg.header.stamp = stamp_;
            img_msg.header.frame_id = vehicle_name_ + "/camera" + std::to_string(i);
            img_msg.width = response.width;
            img_msg.height = response.height;
            
            // Determine encoding and step based on actual data size
            size_t expected_rgb_size = response.width * response.height * 3;
            size_t expected_rgba_size = response.width * response.height * 4;
            size_t actual_size = response.image_data_uint8.size();
            
            if (actual_size == expected_rgb_size) {
                img_msg.encoding = "rgb8";
                img_msg.step = response.width * 3;
            } else if (actual_size == expected_rgba_size) {
                img_msg.encoding = "rgba8";
                img_msg.step = response.width * 4;
            } else if (actual_size == expected_rgb_size && response.image_type == msr::airlib::ImageCaptureBase::ImageType::Scene) {
                // AirSim sometimes returns BGR for scene images
                img_msg.encoding = "bgr8";
                img_msg.step = response.width * 3;
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Image size mismatch for camera %zu on %s: expected %zu or %zu bytes, got %zu bytes (dims: %dx%d)", 
                    i, vehicle_name_.c_str(), expected_rgb_size, expected_rgba_size, actual_size, 
                    response.width, response.height);
                continue;
            }
            
            img_msg.data = response.image_data_uint8;

            camera_pubs_[i]->publish(img_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published %s image %dx%d (%zu bytes) for camera %zu on %s", 
                        img_msg.encoding.c_str(), img_msg.width, img_msg.height, 
                        img_msg.data.size(), i, vehicle_name_.c_str());
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
            process_lidar_for_targets();
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

void MultirotorNode::process_lidar_for_targets()
{
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_lidar_.get());
        auto lidar_data = multirotor_client->getLidarData("Lidar1", vehicle_name_);
        
        if (lidar_data.point_cloud.size() < 12) { // Need at least 4 points (x,y,z each)
            return;
        }
        
        float center_x, center_y, radius, confidence;
        bool detected = detect_circular_spline(lidar_data.point_cloud, center_x, center_y, radius, confidence);
        
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (detected && confidence > 0.5f) {
            current_target_.x = center_x;
            current_target_.y = center_y;
            current_target_.z = 0.0f; // Ground level assumption
            current_target_.radius = radius;
            current_target_.confidence = confidence;
            current_target_.last_seen = std::chrono::steady_clock::now();
            
            // Publish target detection
            airsim_interfaces::msg::TargetDetection msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = vehicle_name_ + "/lidar";
            msg.vehicle_name = vehicle_name_;
            msg.target_x = center_x;
            msg.target_y = center_y;
            msg.target_z = 0.0f;
            msg.confidence = confidence;
            
            target_detection_pub_->publish(msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Target detected by %s: (%.2f, %.2f) radius=%.2f confidence=%.2f",
                        vehicle_name_.c_str(), center_x, center_y, radius, confidence);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Target detection error for %s: %s", vehicle_name_.c_str(), e.what());
    }
}

bool MultirotorNode::detect_circular_spline(const std::vector<float>& point_cloud, 
                                          float& center_x, float& center_y, float& radius, float& confidence)
{
    if (point_cloud.size() < 30) { // Need at least 10 points (x,y,z each)
        confidence = 0.0f;
        return false;
    }
    
    // Simple circle fitting algorithm
    float sum_x = 0.0f, sum_y = 0.0f;
    int point_count = 0;
    
    // Extract 2D points and calculate centroid
    for (size_t i = 0; i < point_cloud.size(); i += 3) {
        if (i + 2 < point_cloud.size()) {
            sum_x += point_cloud[i];     // x
            sum_y += point_cloud[i + 1]; // y
            // z is point_cloud[i + 2] but we ignore for 2D circle fitting
            point_count++;
        }
    }
    
    if (point_count < 10) {
        confidence = 0.0f;
        return false;
    }
    
    center_x = sum_x / point_count;
    center_y = sum_y / point_count;
    
    // Calculate average radius
    float sum_radius = 0.0f;
    int valid_points = 0;
    
    for (size_t i = 0; i < point_cloud.size(); i += 3) {
        if (i + 2 < point_cloud.size()) {
            float dx = point_cloud[i] - center_x;
            float dy = point_cloud[i + 1] - center_y;
            float r = std::sqrt(dx * dx + dy * dy);
            
            if (r > 1.0f && r < 100.0f) { // Reasonable radius range
                sum_radius += r;
                valid_points++;
            }
        }
    }
    
    if (valid_points < 8) {
        confidence = 0.0f;
        return false;
    }
    
    radius = sum_radius / valid_points;
    
    // Calculate confidence based on how well points fit the circle
    float deviation_sum = 0.0f;
    for (size_t i = 0; i < point_cloud.size(); i += 3) {
        if (i + 2 < point_cloud.size()) {
            float dx = point_cloud[i] - center_x;
            float dy = point_cloud[i + 1] - center_y;
            float r = std::sqrt(dx * dx + dy * dy);
            float deviation = std::abs(r - radius);
            deviation_sum += deviation;
        }
    }
    
    float avg_deviation = deviation_sum / point_count;
    confidence = std::max(0.0f, 1.0f - (avg_deviation / (radius * 0.2f))); // 20% tolerance
    
    return confidence > 0.3f; // Minimum confidence threshold
}

bool MultirotorNode::search_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Search target request for %s: radius=%.1f, time=%.1f, confidence=%.2f", 
               vehicle_name_.c_str(), request->search_radius, request->search_time, request->min_confidence);
    
    try {
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        
        // Check if drone is ready to move
        auto current_state = multirotor_client->getMultirotorState(vehicle_name_);
        
        // Check API control and armed state using separate API calls
        bool api_control_enabled = multirotor_client->isApiControlEnabled(vehicle_name_);
        
        RCLCPP_INFO(this->get_logger(), "Drone %s state - Landed: %d, API Control: %s",
                   vehicle_name_.c_str(), 
                   static_cast<int>(current_state.landed_state),
                   api_control_enabled ? "true" : "false");
        
        if (current_state.landed_state != msr::airlib::LandedState::Flying) {
            response->success = false;
            response->message = "Drone " + vehicle_name_ + " must be airborne before searching. Current state: " + 
                              std::to_string(static_cast<int>(current_state.landed_state)) + " (0=Landed, 1=Flying)";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return true;
        }
        
        if (!api_control_enabled) {
            response->success = false;
            response->message = "API control not enabled for " + vehicle_name_;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return true;
        }
        
        std::lock_guard<std::mutex> lock(target_mutex_);
        
        // Check if we have a recent target detection first
        auto now = std::chrono::steady_clock::now();
        auto time_since_detection = std::chrono::duration_cast<std::chrono::seconds>(now - current_target_.last_seen);
        
        if (current_target_.confidence >= request->min_confidence && time_since_detection.count() < request->search_time) {
            response->success = true;
            response->target_x = current_target_.x;
            response->target_y = current_target_.y;
            response->target_z = current_target_.z;
            response->confidence = current_target_.confidence;
            response->message = "Target already detected by " + vehicle_name_;
            return true;
        }
        
        // Initiate active search pattern
        const auto& pos = current_state.kinematics_estimated.pose.position;
        float search_altitude = pos.z(); // Maintain current altitude
        
        RCLCPP_INFO(this->get_logger(), "Starting search pattern from position: (%.2f, %.2f, %.2f)",
                   pos.x(), pos.y(), pos.z());
        
        int num_waypoints = 4; // Reduced for testing
        float angle_step = 2.0f * M_PI / num_waypoints;
        
        for (int i = 0; i < num_waypoints; ++i) {
            float angle = i * angle_step;
            float search_x = pos.x() + (request->search_radius * 0.3f) * std::cos(angle); // Reduced radius
            float search_y = pos.y() + (request->search_radius * 0.3f) * std::sin(angle);
            
            RCLCPP_INFO(this->get_logger(), "Moving to search waypoint %d: (%.2f, %.2f, %.2f)",
                       i + 1, search_x, search_y, search_altitude);
            
            try {
                auto task = multirotor_client->moveToPositionAsync(
                    search_x, search_y, search_altitude,
                    3.0f,   // velocity (reduced)
                    15.0f,  // timeout_sec (reduced)
                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                    msr::airlib::YawMode(false, 0.0f),
                    -1,     // lookahead
                    1,      // adaptive_lookahead
                    vehicle_name_);
                
                // Wait for movement to complete with timeout
                task->waitOnLastTask();
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", i + 1);
                
            } catch (const rpc::rpc_error& e) {
                RCLCPP_ERROR(this->get_logger(), "RPC error at waypoint %d: %s", i + 1, e.what());
                // Continue with search even if one waypoint fails
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint %d: %s", i + 1, e.what());
            }
            
            // Small delay to process sensor data
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // Check if target was detected during search
            if (current_target_.confidence >= request->min_confidence) {
                RCLCPP_INFO(this->get_logger(), "Target detected during search at waypoint %d", i + 1);
                break;
            }
        }
        
        // Final response
        if (current_target_.confidence >= request->min_confidence) {
            response->success = true;
            response->target_x = current_target_.x;
            response->target_y = current_target_.y;
            response->target_z = current_target_.z;
            response->confidence = current_target_.confidence;
            response->message = "Target found during search by " + vehicle_name_;
        } else {
            response->success = false;
            response->target_x = 0.0f;
            response->target_y = 0.0f;
            response->target_z = 0.0f;
            response->confidence = 0.0f;
            response->message = "No target found after search pattern by " + vehicle_name_;
        }
        
    } catch (const rpc::rpc_error& e) {
        response->success = false;
        response->target_x = 0.0f;
        response->target_y = 0.0f;
        response->target_z = 0.0f;
        response->confidence = 0.0f;
        response->message = "Search failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Search RPC error for %s: %s", vehicle_name_.c_str(), e.what());
    } catch (const std::exception& e) {
        response->success = false;
        response->target_x = 0.0f;
        response->target_y = 0.0f;
        response->target_z = 0.0f;
        response->confidence = 0.0f;
        response->message = "Search failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Search error for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}

bool MultirotorNode::track_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> response)
{
    std::lock_guard<std::mutex> lock(target_mutex_);
    
    if (current_target_.confidence < 0.5f) {
        response->success = false;
        response->message = "No target detected for tracking";
        return true;
    }
    
    try {
        // Calculate next waypoint along the detected circular spline
        float current_angle = std::atan2(request->target_y - current_target_.y, 
                                       request->target_x - current_target_.x);
        float next_angle = current_angle + 0.1f; // 0.1 radian step
        
        float next_x = current_target_.x + current_target_.radius * std::cos(next_angle);
        float next_y = current_target_.y + current_target_.radius * std::sin(next_angle);
        float next_z = request->target_z; // Maintain altitude
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
        auto task = multirotor_client->moveToPositionAsync(
            next_x, next_y, next_z, 
            3.0f,   // velocity
            60.0f,  // timeout_sec
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(false, 0.0f),
            -1,     // lookahead
            1,      // adaptive_lookahead  
            vehicle_name_);
        
        task->waitOnLastTask();
        
        response->success = true;
        response->message = "Tracking spline - next waypoint: (" + 
                          std::to_string(next_x) + ", " + std::to_string(next_y) + ", " + std::to_string(next_z) + ")";
        
        RCLCPP_INFO(this->get_logger(), "Tracking waypoint for %s: (%.2f, %.2f, %.2f)",
                   vehicle_name_.c_str(), next_x, next_y, next_z);
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Tracking failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Tracking error for %s: %s", vehicle_name_.c_str(), e.what());
    }
    
    return true;
}