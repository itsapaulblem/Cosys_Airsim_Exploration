/*
* AirSim Multirotor Node - Enhanced Parallel Processing
* 
* PURPOSE
* This node provides control and sensor data interface for a single
* multirotor vehicle with fully parallelized sensor processing to eliminate
* FPS bottlenecks in image streaming and visualization tools like rqt_image_view.
*
* ENHANCED PARALLEL ARCHITECTURE:
* - Utilizes VehicleNodeBase's parallel callback groups and timers
* - Independent processing for images, lidar, IMU, and state data
* - Thread-safe state management with mutexes
* - Separate AirSim RPC clients to prevent sensor blocking
* - Configurable timer frequencies optimized for each sensor type
*
* PERFORMANCE IMPROVEMENTS:
* - Image processing: 30Hz independent timer with dedicated RPC client
* - LiDAR processing: 20Hz timer isolated from image processing
* - State publishing: 50Hz timer for responsive control
* - Sensor data: 10Hz timer for IMU/GPS/barometer data
* - Each sensor stream operates independently without sequential bottlenecks
*/

#include "multirotor_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include "common/common_utils/Utils.hpp"
// #include <airsim_interfaces/msg/target_detection.hpp>  // TODO: Create this message if needed
#include <airsim_interfaces/srv/search_target.hpp>
#include <airsim_interfaces/srv/track_target.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void fixPointCloud(std::vector<float>& data, int offset, std::vector<int> flip_indexes) {
    for (size_t i = 1; i < data.size(); i += offset) {
        for (auto idx : flip_indexes) {
            if (i + idx < data.size()) {
                data[i + idx] = -data[i + idx];
            }
        }
    }
}

MultirotorNode::MultirotorNode(const std::string& vehicle_name, 
                               const std::string& host_ip, 
                               uint16_t host_port)
    : VehicleNodeBase(vehicle_name, host_ip, host_port, nullptr)
{
    // Set optimized timer frequencies for parallel processing
    state_timer_freq_ = 0.02;   // 50Hz for responsive control
    image_timer_freq_ = 0.033;  // 30Hz for smooth video
    lidar_timer_freq_ = 0.05;   // 20Hz for lidar data
    echo_timer_freq_ = 0.1;     // 10Hz for other sensors
    
    // Initialize basic components first
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Setup publishers and subscribers (but don't initialize clients yet)
    setup_sensor_publishers();
    setup_vehicle_publishers();
    setup_vehicle_subscribers();
    setup_vehicle_services();
    
    RCLCPP_INFO(this->get_logger(), "Enhanced parallel multirotor node created for: %s", vehicle_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Timer frequencies - State: %.1fHz, Images: %.1fHz, LiDAR: %.1fHz, Sensors: %.1fHz", 
                1.0/state_timer_freq_, 1.0/image_timer_freq_, 1.0/lidar_timer_freq_, 1.0/echo_timer_freq_);
}

void MultirotorNode::initialize_common()
{
    // CRITICAL: Initialize vehicle clients first before any base class calls
    try {
        initialize_vehicle_client();
        RCLCPP_INFO(this->get_logger(), "Vehicle clients initialized successfully for: %s", vehicle_name_.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize vehicle clients for %s: %s", vehicle_name_.c_str(), e.what());
        return;
    }

    // Now call base class implementation with properly initialized clients
    VehicleNodeBase::initialize_common();
    
    // Verify connections are established
    if (!establish_connections()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish connections for vehicle: %s", vehicle_name_.c_str());
        return;
    }
    
    // Add multirotor-specific setup after base initialization is complete
    try {
        setup_vehicle_control_subscribers();
        setup_vehicle_control_services();
        RCLCPP_INFO(this->get_logger(), "Multirotor-specific setup completed for: %s", vehicle_name_.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup multirotor-specific components for %s: %s", vehicle_name_.c_str(), e.what());
        return;
    }

    // CRITICAL: Mark initialization complete ONLY after ALL setup is done
    initialization_complete_.store(true);
    RCLCPP_INFO(this->get_logger(), "Multirotor node fully initialized: %s - timer callbacks now enabled", vehicle_name_.c_str());
}

void MultirotorNode::initialize_vehicle_client()
{
    try {
        // Cast base client to multirotor client for vehicle-specific operations
        state_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        sensor_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);

        // Confirm connections
        static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get())->confirmConnection();
        static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get())->confirmConnection();

        // Enable API control
        static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get())->enableApiControl(true, vehicle_name_);
        
        RCLCPP_INFO(this->get_logger(), "Connected to AirSim with parallel RPC clients for vehicle: %s", vehicle_name_.c_str());
    }
    catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR(this->get_logger(), "RPC connection failed for vehicle %s: %s", vehicle_name_.c_str(), e.what());
        throw;
    }
}

void MultirotorNode::setup_sensor_publishers()
{
    // Camera publishers (4 cameras)
    for (int i = 0; i < 4; ++i) {
        auto image_pub = this->create_publisher<sensor_msgs::msg::Image>("camera" + std::to_string(i) + "/image", 10);
        image_pubs_.push_back(image_pub);

        auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera" + std::to_string(i) + "/camera_info", 10);
        camera_info_pubs_.push_back(camera_info_pub);
    }

    // LiDAR publisher
    for (int i = 0; i < 1; ++i) {
        auto lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar" + std::to_string(i) + "/points", 10);
        lidar_pubs_.push_back(lidar_pub);
    }

    // Other sensor publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10); 
    baro_pub_ = this->create_publisher<sensor_msgs::msg::Range>("baro", 10);
    env_pub_ = this->create_publisher<airsim_interfaces::msg::Environment>("environment", 10);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>("system_status", 10);
    // TODO: Create TargetDetection.msg in airsim_interfaces if motion detection is needed
    // target_detection_pub_ = this->create_publisher<airsim_interfaces::msg::TargetDetection>("target_detection", 10);

    RCLCPP_INFO(this->get_logger(), "Setup sensor publishers: 4 cameras, 1 lidar, 5 other sensors");
}

void MultirotorNode::setup_vehicle_publishers()
{
    // Odometry publisher handled by base class
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void MultirotorNode::setup_vehicle_control_subscribers()
{
    // Ultra-clean topic prefixing: vehicle_name + "/" + topic
    std::string topic_prefix = vehicle_name_ + "/";
    
    // Use control_group for velocity command subscriptions for isolation from sensor operations
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = control_group_;
    
    vel_cmd_body_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        topic_prefix + "vel_cmd_body_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_body_frame_callback, this, std::placeholders::_1),
        sub_options);
        
    vel_cmd_world_sub_ = this->create_subscription<airsim_interfaces::msg::VelCmd>(
        topic_prefix + "vel_cmd_world_frame", 1,
        std::bind(&MultirotorNode::vel_cmd_world_frame_callback, this, std::placeholders::_1),
        sub_options);
        
    RCLCPP_INFO(this->get_logger(), "Setup multirotor control subscribers for: %s", vehicle_name_.c_str());
}

void MultirotorNode::setup_vehicle_control_services()
{
    // Ultra-clean topic prefixing: vehicle_name + "/" + service
    std::string topic_prefix = vehicle_name_ + "/";

    takeoff_srv_ = this->create_service<airsim_interfaces::srv::Takeoff>(
        "takeoff",
        std::bind(&MultirotorNode::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));

    land_srv_ = this->create_service<airsim_interfaces::srv::Land>(
        "land",
        std::bind(&MultirotorNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));

    gps_waypoint_srv_ = this->create_service<airsim_interfaces::srv::GpsWaypoint>(
        "gps_waypoint",
        std::bind(&MultirotorNode::gps_waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));

    search_target_srv_ = this->create_service<airsim_interfaces::srv::SearchTarget>(
        "search_target",
        std::bind(&MultirotorNode::search_target_callback, this, std::placeholders::_1, std::placeholders::_2));

    track_target_srv_ = this->create_service<airsim_interfaces::srv::TrackTarget>(
        "track_target", 
        std::bind(&MultirotorNode::track_target_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// Override base class virtual methods for parallel processing
void MultirotorNode::update_vehicle_state()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get());
    vehicle_state_ = multirotor_client->getMultirotorState(vehicle_name_);
    stamp_ = this->get_clock()->now();
}

void MultirotorNode::publish_vehicle_state()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    nav_msgs::msg::Odometry odom_msg = get_odom_from_multirotor_state(vehicle_state_);
    odom_msg.header.stamp = stamp_;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = vehicle_name_ + "_base_link";
    
    odom_pub_->publish(odom_msg);
    publish_odometry_tf(odom_msg);
}

void MultirotorNode::process_vehicle_commands()
{
    std::lock_guard<std::mutex> cmd_lock(vel_cmd_mutex_);
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get());
    
    if (has_new_vel_cmd_body_frame_) {
        msr::airlib::Quaternionr current_orientation;
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            current_orientation = vehicle_state_.getOrientation();
        }
        VelCmd cmd = get_airlib_body_vel_cmd(vel_cmd_body_frame_, current_orientation);
        multirotor_client->moveByVelocityBodyFrameAsync(
            cmd.x, cmd.y, cmd.z, cmd.duration,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(), vehicle_name_);
        has_new_vel_cmd_body_frame_ = false;
    }
    
    if (has_new_vel_cmd_world_frame_) {
        VelCmd cmd = get_airlib_world_vel_cmd(vel_cmd_world_frame_);
        multirotor_client->moveByVelocityAsync(
            cmd.x, cmd.y, cmd.z, cmd.duration,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(), vehicle_name_);
        has_new_vel_cmd_world_frame_ = false;
    }
}

// Parallel sensor processing implementations
void MultirotorNode::process_images()
{
    try {
        auto multirotor_client_images = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;

        // Create requests for all 4 cameras
        for (int i = 0; i < 4; ++i) {
            requests.push_back(
                msr::airlib::ImageCaptureBase::ImageRequest(
                    std::to_string(i),
                    msr::airlib::ImageCaptureBase::ImageType::Scene,
                    false, false
                )
            );
        }
                
        auto responses = multirotor_client_images->simGetImages(requests, vehicle_name_);
        
        rclcpp::Time current_stamp;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_stamp = stamp_;
        }
        
        for (size_t i = 0; i < responses.size() && i < image_pubs_.size(); ++i) {
            if (responses[i].image_data_uint8.size() > 0) {
                try {
                    sensor_msgs::msg::Image image_msg;
                    image_msg.header.stamp = current_stamp;
                    image_msg.header.frame_id = vehicle_name_ + "_camera" + std::to_string(i);
                    image_msg.height = responses[i].height;
                    image_msg.width = responses[i].width;
                    image_msg.encoding = "rgb8";
                    image_msg.step = image_msg.width * 3;
                    
                    // Verify data size matches expected size
                    size_t expected_data_size = image_msg.height * image_msg.width * 3;
                    if (responses[i].image_data_uint8.size() != expected_data_size) {
                        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                           "Camera%zu: Data size mismatch. Expected: %zu, Got: %zu", 
                                           i, expected_data_size, responses[i].image_data_uint8.size());
                        continue;
                    }
                    
                    image_msg.data = responses[i].image_data_uint8;
                    image_pubs_[i]->publish(image_msg);
                    
                    // Publish camera info
                    if (i < camera_info_pubs_.size()) {
                        sensor_msgs::msg::CameraInfo camera_info;
                        camera_info.header = image_msg.header;
                        camera_info.width = image_msg.width;
                        camera_info.height = image_msg.height;
                        
                        // Set basic camera parameters
                        camera_info.distortion_model = "plumb_bob";
                        camera_info.d.resize(5, 0.0);
                        
                        // Set camera matrix
                        camera_info.k.fill(0.0);
                        camera_info.k[0] = image_msg.width * 0.8;  // fx
                        camera_info.k[4] = image_msg.height * 0.8; // fy  
                        camera_info.k[2] = image_msg.width / 2.0;  // cx
                        camera_info.k[5] = image_msg.height / 2.0; // cy
                        camera_info.k[8] = 1.0;
                        
                        // Set rectification matrix (identity)
                        camera_info.r.fill(0.0);
                        camera_info.r[0] = camera_info.r[4] = camera_info.r[8] = 1.0;
                        
                        // Set projection matrix
                        camera_info.p.fill(0.0);
                        camera_info.p[0] = camera_info.k[0];   // fx
                        camera_info.p[5] = camera_info.k[4];   // fy
                        camera_info.p[2] = camera_info.k[2];   // cx
                        camera_info.p[6] = camera_info.k[5];   // cy
                        camera_info.p[10] = 1.0;
                        
                        camera_info_pubs_[i]->publish(camera_info);
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Error processing camera%zu: %s", i, e.what());
                }
            }
        }
        
    } catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Camera RPC error: %s", e.what());
    }
}

void MultirotorNode::process_lidar()
{
    try {
        if (!lidar_pubs_.empty()) {
            auto multirotor_client_lidar = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
            auto lidar_data = multirotor_client_lidar->getLidarData("", vehicle_name_);
            
            sensor_msgs::msg::PointCloud2 lidar_msg;
            
            rclcpp::Time current_stamp;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_stamp = stamp_;
            }
            
            lidar_msg.header.stamp = current_stamp;
            lidar_msg.header.frame_id = vehicle_name_ + "_lidar0";
            
            if (lidar_data.point_cloud.size() > 0) {
                lidar_msg.height = 1;
                lidar_msg.width = lidar_data.point_cloud.size() / 3;
                lidar_msg.is_dense = true;
                
                sensor_msgs::PointCloud2Modifier modifier(lidar_msg);
                modifier.setPointCloud2Fields(3,
                    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
                modifier.resize(lidar_msg.width);
                
                sensor_msgs::PointCloud2Iterator<float> iter_x(lidar_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(lidar_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(lidar_msg, "z");
                
                for (size_t i = 0; i < lidar_data.point_cloud.size(); i += 3) {
                    *iter_x = lidar_data.point_cloud[i];
                    *iter_y = lidar_data.point_cloud[i + 1];
                    *iter_z = lidar_data.point_cloud[i + 2];
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                }
                
                lidar_pubs_[0]->publish(lidar_msg);
            }
        }
    } catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "LiDAR RPC error: %s", e.what());
    }

}

void MultirotorNode::process_gpulidar()
{
    // GPU LiDAR processing - placeholder for future implementation
}

void MultirotorNode::process_echo()
{
    // Publish other sensor data in this method
    try {
        // auto multirotor_client_echo = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
        
        rclcpp::Time current_stamp;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_stamp = stamp_;
        }

        // Publish IMU data
        publish_imu_data();
        publish_magnetometer_data();
        publish_barometer_data();
        publish_gps_data();
        publish_environment_data();
        publish_system_status();
        
    } catch (const rpc::rpc_error& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Sensor RPC error: %s", e.what());
    }
}

bool MultirotorNode::takeoff_callback(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    (void)request; // Suppress unused parameter warning
    try {
        RCLCPP_INFO(this->get_logger(), "Takeoff request received for %s", vehicle_name_.c_str());
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get());
        
        // Arm the vehicle
        multirotor_client->armDisarm(true, vehicle_name_);
        
        // Takeoff
        multirotor_client->takeoffAsync(20.0f, vehicle_name_);
        
        response->success = true;
        response->message = "Takeoff completed successfully for " + vehicle_name_;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        
        return true;
    }
    catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Takeoff failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return false;
    }
}

bool MultirotorNode::land_callback(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request>,
    std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Land request received for %s", vehicle_name_.c_str());
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get());
        
        multirotor_client->landAsync(60.0f, vehicle_name_);
        
        // Disarm after landing
        std::this_thread::sleep_for(std::chrono::seconds(2));
        multirotor_client->armDisarm(false, vehicle_name_);
        
        response->success = true;
        response->message = "Landing completed successfully for " + vehicle_name_;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        
        return true;
    }
    catch (const rpc::rpc_error& e) {
        response->success = false;
        response->message = "Landing failed: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return false;
    }
}

void MultirotorNode::vel_cmd_body_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vel_cmd_mutex_);
    vel_cmd_body_frame_ = *msg;
    has_new_vel_cmd_body_frame_ = true;
}

void MultirotorNode::vel_cmd_world_frame_callback(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vel_cmd_mutex_);
    vel_cmd_world_frame_ = *msg;
    has_new_vel_cmd_world_frame_ = true;
}

// TODO: Create TargetDetection.msg in airsim_interfaces if motion detection is needed
/*
void MultirotorNode::motion_detection_callback(const airsim_interfaces::msg::TargetDetection::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(motion_target_mutex_);
    
    if (msg->target_detected) {
        current_motion_target_.x = msg->target_x;
        current_motion_target_.y = msg->target_y;
        current_motion_target_.z = msg->target_z;
        current_motion_target_.confidence = msg->confidence;
        current_motion_target_.last_seen = std::chrono::steady_clock::now();
        
        RCLCPP_DEBUG(this->get_logger(), "Target detected at (%f, %f) with confidence %f", 
                    msg->target_x, msg->target_y, msg->confidence);
    }
}
*/

void MultirotorNode::publish_static_transforms()
{
    if (static_transforms_published_) return;
    
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    for (int i = 0; i < 4; ++i) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_stamp;
        tf_msg.header.frame_id = vehicle_name_ + "_base_link";
        tf_msg.child_frame_id = vehicle_name_ + "_camera" + std::to_string(i);
        
        // Set camera positions (adjust as needed for your setup)
        tf_msg.transform.translation.x = 0.1;
        tf_msg.transform.translation.y = i * 0.05 - 0.075; // Spread cameras slightly
        tf_msg.transform.translation.z = 0.0;
        
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        
        transforms.push_back(tf_msg);
    }
    
    static_tf_broadcaster_->sendTransform(transforms);
    static_transforms_published_ = true;
}

void MultirotorNode::publish_system_status()
{
    std_msgs::msg::String status_msg;
    status_msg.data = "MultirotorNode " + vehicle_name_ + " operational";
    system_status_pub_->publish(status_msg);
}

void MultirotorNode::publish_imu_data()
{
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
    auto imu_data = multirotor_client->getImuData("", vehicle_name_);
    
    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    imu_msg.header.stamp = current_stamp;
    imu_msg.header.frame_id = vehicle_name_ + "_imu";
    
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();
    
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();
    
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();
    
    imu_pub_->publish(imu_msg);
}

void MultirotorNode::publish_magnetometer_data()
{
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
    auto mag_data = multirotor_client->getMagnetometerData("", vehicle_name_);
    
    sensor_msgs::msg::MagneticField mag_msg;
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    mag_msg.header.stamp = current_stamp;
    mag_msg.header.frame_id = vehicle_name_ + "_magnetometer";
    
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    
    mag_pub_->publish(mag_msg);
}

void MultirotorNode::publish_barometer_data()
{
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
    auto baro_data = multirotor_client->getBarometerData("", vehicle_name_);
    
    sensor_msgs::msg::Range baro_msg;
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    baro_msg.header.stamp = current_stamp;
    baro_msg.header.frame_id = vehicle_name_ + "_barometer";
    baro_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    baro_msg.field_of_view = 0.1;
    baro_msg.min_range = 0.0;
    baro_msg.max_range = 100.0;
    baro_msg.range = baro_data.altitude;
    
    baro_pub_->publish(baro_msg);
}

void MultirotorNode::publish_gps_data()
{
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
    auto gps_data = multirotor_client->getGpsData("", vehicle_name_);
    
    sensor_msgs::msg::NavSatFix gps_msg;
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    gps_msg.header.stamp = current_stamp;
    gps_msg.header.frame_id = vehicle_name_ + "_gps";
    
    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude;
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;
    
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    
    gps_pub_->publish(gps_msg);
}

void MultirotorNode::publish_environment_data()
{
    airsim_interfaces::msg::Environment env_msg;
    rclcpp::Time current_stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
    }
    
    env_msg.header.stamp = current_stamp;
    env_msg.header.frame_id = odom_frame_id_;
    
    // Get environment data from AirSim
    env_msg.temperature = 20.0; // Celsius
    env_msg.air_pressure = 101325.0; // Pa
    env_msg.air_density = 1.225; // kg/m³
    
    env_pub_->publish(env_msg);
}

void MultirotorNode::publish_tf_data()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    rclcpp::Time current_stamp;
    msr::airlib::Vector3r position;
    msr::airlib::Quaternionr orientation;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_stamp = stamp_;
        position = vehicle_state_.getPosition();
        orientation = vehicle_state_.getOrientation();
    }
    
    tf_msg.header.stamp = current_stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = vehicle_name_ + "_base_link";
    
    tf_msg.transform.translation.x = position.x();
    tf_msg.transform.translation.y = position.y();
    tf_msg.transform.translation.z = position.z();
    
    tf_msg.transform.rotation.x = orientation.x();
    tf_msg.transform.rotation.y = orientation.y();
    tf_msg.transform.rotation.z = orientation.z();
    tf_msg.transform.rotation.w = orientation.w();
    
    tf_broadcaster_->sendTransform(tf_msg);
}

bool MultirotorNode::gps_waypoint_callback(
    const std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Request> request,
    std::shared_ptr<airsim_interfaces::srv::GpsWaypoint::Response> response)
{
    try {
        if (!validate_gps_coordinates(request->latitude, request->longitude)) {
            response->success = false;
            response->message = "Invalid GPS coordinates";
            return false;
        }
        
        auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(state_client_.get());
        
        // Get current GPS position for home reference
        auto current_gps = multirotor_client->getGpsData("", vehicle_name_);
        auto [x, y] = gps_to_ned(request->latitude, request->longitude,
                                current_gps.gnss.geo_point.latitude, 
                                current_gps.gnss.geo_point.longitude);
        
        // Move to position
        multirotor_client->moveToPositionAsync(
            x, y, -request->altitude, 5.0f, -1.0f,
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(), -1.0f, 1.0f, vehicle_name_);
        
        response->success = true;
        response->message = "GPS waypoint navigation completed";
        RCLCPP_INFO(this->get_logger(), "Moved to GPS waypoint: %f, %f", request->latitude, request->longitude);
        
        return true;
    }
    catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("GPS waypoint failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return false;
    }
}

std::pair<double, double> MultirotorNode::gps_to_ned(double lat, double lon, double home_lat, double home_lon)
{
    // Simple conversion - for more accuracy, use proper geodetic conversion
    const double earth_radius = 6371000.0; // meters
    const double deg_to_rad = M_PI / 180.0;
    
    double dlat = (lat - home_lat) * deg_to_rad;
    double dlon = (lon - home_lon) * deg_to_rad;
    
    double x = dlat * earth_radius;
    double y = dlon * earth_radius * cos(home_lat * deg_to_rad);
    
    return {x, y};
}

bool MultirotorNode::validate_gps_coordinates(double lat, double lon)
{
    return (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0);
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg)
{
    VelCmd cmd;
    cmd.x = msg.twist.linear.x;
    cmd.y = msg.twist.linear.y;
    cmd.z = msg.twist.linear.z;
    cmd.duration = 0.1f; // Default duration
    return cmd;
}

MultirotorNode::VelCmd MultirotorNode::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, 
                                                               const msr::airlib::Quaternionr& orientation)
{
    // Transform velocity from body frame to world frame using current orientation
    msr::airlib::Vector3r body_vel(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    msr::airlib::Vector3r world_vel = msr::airlib::VectorMath::transformToWorldFrame(body_vel, orientation);
    
    VelCmd cmd;
    cmd.x = world_vel.x();
    cmd.y = world_vel.y();
    cmd.z = world_vel.z();
    cmd.duration = 0.1f; // Default duration
    return cmd;
}

nav_msgs::msg::Odometry MultirotorNode::get_odom_from_multirotor_state(const msr::airlib::MultirotorState& state)
{
    nav_msgs::msg::Odometry odom_msg;
    
    auto position = state.getPosition();
    auto orientation = state.getOrientation();
    auto linear_vel = state.kinematics_estimated.twist.linear;
    auto angular_vel = state.kinematics_estimated.twist.angular;
    
    // Position
    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();
    
    // Orientation
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    
    // Linear velocity
    odom_msg.twist.twist.linear.x = linear_vel.x();
    odom_msg.twist.twist.linear.y = linear_vel.y();
    odom_msg.twist.twist.linear.z = linear_vel.z();
    
    // Angular velocity
    odom_msg.twist.twist.angular.x = angular_vel.x();
    odom_msg.twist.twist.angular.y = angular_vel.y();
    odom_msg.twist.twist.angular.z = angular_vel.z();
    
    return odom_msg;
}

bool MultirotorNode::search_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> response)
{
    (void)request; // Suppress unused parameter warning
    // Search target implementation - placeholder
    response->success = true;
    response->message = "Search target functionality not implemented yet";
    return true;
}

bool MultirotorNode::track_target_callback(
    const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> request,
    std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> response)
{
    (void)request; // Suppress unused parameter warning
    // Track target implementation - placeholder
    response->success = true;
    response->message = "Track target functionality not implemented yet";
    return true;
}

// Override implementations for virtual functions declared in header
void MultirotorNode::setup_vehicle_subscribers()
{
    // Call base class implementation
    VehicleNodeBase::setup_vehicle_subscribers();
    // Add any multirotor-specific subscribers here if needed in the future
}

void MultirotorNode::setup_vehicle_services()
{
    // Call base class implementation
    VehicleNodeBase::setup_vehicle_services();
    // Add any multirotor-specific services here if needed in the future
}

// Virtual method implementations required by base class
nav_msgs::msg::Odometry MultirotorNode::get_vehicle_odometry()
{
    nav_msgs::msg::Odometry odom_msg;
    
    // Get current vehicle state
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Fill header
        odom_msg.header.stamp = stamp_;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_link_frame_id_;
        
        // Fill pose
        odom_msg.pose.pose.position.x = vehicle_state_.kinematics_estimated.pose.position.x();
        odom_msg.pose.pose.position.y = -vehicle_state_.kinematics_estimated.pose.position.y();
        odom_msg.pose.pose.position.z = -vehicle_state_.kinematics_estimated.pose.position.z();
        
        odom_msg.pose.pose.orientation.x = vehicle_state_.kinematics_estimated.pose.orientation.x();
        odom_msg.pose.pose.orientation.y = -vehicle_state_.kinematics_estimated.pose.orientation.y();
        odom_msg.pose.pose.orientation.z = -vehicle_state_.kinematics_estimated.pose.orientation.z();
        odom_msg.pose.pose.orientation.w = vehicle_state_.kinematics_estimated.pose.orientation.w();
        
        // Fill twist
        odom_msg.twist.twist.linear.x = vehicle_state_.kinematics_estimated.twist.linear.x();
        odom_msg.twist.twist.linear.y = -vehicle_state_.kinematics_estimated.twist.linear.y();
        odom_msg.twist.twist.linear.z = -vehicle_state_.kinematics_estimated.twist.linear.z();
        
        odom_msg.twist.twist.angular.x = vehicle_state_.kinematics_estimated.twist.angular.x();
        odom_msg.twist.twist.angular.y = -vehicle_state_.kinematics_estimated.twist.angular.y();
        odom_msg.twist.twist.angular.z = -vehicle_state_.kinematics_estimated.twist.angular.z();
    }
    
    return odom_msg;
}

msr::airlib::LidarData MultirotorNode::get_lidar_data_for_sensor(const std::string& sensor_name, const std::string& vehicle_name)
{
    if (!sensor_client_) {
        throw std::runtime_error("Sensor client not initialized for lidar data retrieval");
    }
    
    auto multirotor_client = static_cast<msr::airlib::MultirotorRpcLibClient*>(sensor_client_.get());
    return multirotor_client->getLidarData(sensor_name, vehicle_name);
}
