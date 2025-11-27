#include "localization_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>

// Forward declaration for VehicleNodeBase to access global AirSim RPC mutex
class VehicleNodeBase {
public:
    static std::mutex airsim_rpc_mutex_;  // Forward declaration only
};

LocalizationNode::LocalizationNode(const std::string& vehicle_name, const std::string& host_ip, uint16_t host_port)
    : Node("localization_" + vehicle_name), vehicle_name_(vehicle_name), host_ip_(host_ip), host_port_(host_port)
{
    // Initialize REP 105 compliant frame IDs - MUST match vehicle node frame names exactly
    map_frame_id_ = "map";
    odom_frame_id_ = vehicle_name_ + "/odom";
    base_link_frame_id_ = vehicle_name_ + "/base_link";  // Match vehicle_node_base.cpp line 53
    
    // Initialize TF2 infrastructure
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize AirSim client for ground truth localization
    initialize_airsim_client();
    
    // Initialize spawn offset for coordinate consistency with vehicle nodes
    initialize_spawn_offset();
    
    // REP 105 COMPLIANT: Timer for dynamic map→odom transform publishing
    // This is the KEY difference from static transforms - it's computed dynamically!
    localization_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20Hz localization update rate
        std::bind(&LocalizationNode::publish_localization_transform, this)
    );
    
    RCLCPP_INFO(this->get_logger(), 
        "Localization Node initialized for vehicle: %s", vehicle_name_.c_str());
    RCLCPP_INFO(this->get_logger(), 
        "Frame Authorities - Listening: %s→%s, Publishing: %s→%s", 
        odom_frame_id_.c_str(), base_link_frame_id_.c_str(),
        map_frame_id_.c_str(), odom_frame_id_.c_str());
}

void LocalizationNode::initialize_airsim_client()
{
    try {
        airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        airsim_client_->confirmConnection();
        
        RCLCPP_INFO(this->get_logger(), 
            "AirSim localization client connected for vehicle: %s at %s:%d", 
            vehicle_name_.c_str(), host_ip_.c_str(), host_port_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
            "Failed to connect AirSim localization client for %s to %s:%d - %s", 
            vehicle_name_.c_str(), host_ip_.c_str(), host_port_, e.what());
        // airsim_client_.reset();
    }
}

void LocalizationNode::publish_localization_transform()
{
    if (!airsim_client_) {
        // DIAGNOSTIC: Manual throttled log (5s interval) - AirSim client check
        static auto last_client_log = std::chrono::steady_clock::now();
        auto log_now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_client_log).count() >= 5) {
            RCLCPP_WARN(this->get_logger(), "⚠ Localization BLOCKED for %s: AirSim client not available",
                vehicle_name_.c_str());
            last_client_log = log_now;
        }
        return;
    }

    try {
        // REP 105 FRAME AUTHORITIES IMPLEMENTATION:
        // Step 1: Listen to odom→base_link from odometry source (as required by REP 105)
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        try {
            odom_to_base_link = tf_buffer_->lookupTransform(
                odom_frame_id_, base_link_frame_id_, tf2::TimePointZero,
                tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& e) {
            // DIAGNOSTIC: Manual throttled log (5s interval) - Transform lookup failure
            static auto last_tf_log = std::chrono::steady_clock::now();
            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_tf_log).count() >= 5) {
                RCLCPP_WARN(this->get_logger(), "⚠ Localization BLOCKED for %s: Cannot find transform %s→%s (reason: %s)",
                    vehicle_name_.c_str(), odom_frame_id_.c_str(), base_link_frame_id_.c_str(), e.what());
                last_tf_log = log_now;
            }
            return;
        }

        // Step 2: Get map→base_link from AirSim ground truth "localization sensor"
        geometry_msgs::msg::TransformStamped map_to_base_link = get_map_to_base_link_ground_truth();
        
        // Step 3: Compute map→odom = map→base_link * (odom→base_link)^-1
        // This is the KEY REP 105 computation that localization components must perform
        tf2::Transform map_to_base_tf, odom_to_base_tf, map_to_odom_tf;
        
        tf2::fromMsg(map_to_base_link.transform, map_to_base_tf);
        tf2::fromMsg(odom_to_base_link.transform, odom_to_base_tf);
        
        // Compute the localization correction: how much the map frame differs from odom
        map_to_odom_tf = map_to_base_tf * odom_to_base_tf.inverse();
        
        // Step 4: Publish DYNAMIC map→odom transform (REP 105 compliant!)
        geometry_msgs::msg::TransformStamped map_to_odom_msg;
        map_to_odom_msg.header.stamp = this->get_clock()->now();
        map_to_odom_msg.header.frame_id = map_frame_id_;
        map_to_odom_msg.child_frame_id = odom_frame_id_;
        
        // Convert GPS-derived spawn offset (NED) to ENU coordinates for ROS
        double enu_x = spawn_offset_x_.load();
        double enu_y = -spawn_offset_y_.load();  // NED to ENU conversion
        double enu_z = -spawn_offset_z_.load();  // NED to ENU conversion

        // CRITICAL VALIDATION: Check final transform values before publishing to TF
        if (!std::isfinite(enu_x) || !std::isfinite(enu_y) || !std::isfinite(enu_z)) {
            // DIAGNOSTIC: Manual throttled log (5s interval) - Invalid spawn offset
            static auto last_valid_log = std::chrono::steady_clock::now();
            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_valid_log).count() >= 5) {
                RCLCPP_ERROR(this->get_logger(), "⚠ Localization BLOCKED for %s: Invalid spawn offset x=%.3f, y=%.3f, z=%.3f (initialized=%s)",
                    vehicle_name_.c_str(), enu_x, enu_y, enu_z, spawn_offset_initialized_.load() ? "YES" : "NO");
                last_valid_log = log_now;
            }
            return; // Do not publish invalid transforms
        }

        map_to_odom_msg.transform.translation.x = enu_x;
        map_to_odom_msg.transform.translation.y = enu_y;
        map_to_odom_msg.transform.translation.z = enu_z;

        // Identity rotation (no rotation between map and odom frames)
        map_to_odom_msg.transform.rotation.x = 0.0;
        map_to_odom_msg.transform.rotation.y = 0.0;
        map_to_odom_msg.transform.rotation.z = 0.0;
        map_to_odom_msg.transform.rotation.w = 1.0;

        // Final safety check before TF publishing
        if (!std::isfinite(map_to_odom_msg.transform.rotation.w)) {
        // // DISABLED THROTTLE:             RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        //                 "LOCALIZATION TF ERROR: Invalid rotation quaternion for %s", vehicle_name_.c_str());
            return; // Do not publish invalid transforms
        }

        tf_broadcaster_->sendTransform(map_to_odom_msg);

        // DIAGNOSTIC: Manual throttled log (5s interval) - safe alternative to RCLCPP_INFO_THROTTLE
        static auto last_localization_log = std::chrono::steady_clock::now();
        auto log_now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(log_now - last_localization_log).count() >= 5) {
            RCLCPP_INFO(this->get_logger(), "✓ Localization active for %s - Publishing map→odom transform",
                vehicle_name_.c_str());
            last_localization_log = log_now;
        }
            
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 12000,
        //     "Chain: %s → %s (localization) + %s → %s (odometry) = Complete TF tree",
        //     map_frame_id_.c_str(), odom_frame_id_.c_str(),
        //     odom_frame_id_.c_str(), base_link_frame_id_.c_str());
            
    } catch (const std::exception& e) {
        // // DISABLED THROTTLE:         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        //             "Localization computation failed: %s", e.what());
    }
}

void LocalizationNode::initialize_spawn_offset()
{
    if (!airsim_client_ || spawn_offset_initialized_.load()) {
        return;
    }

    try {
        // GLOBAL RPC MUTEX: Serialize ALL AirSim RPC calls across ALL nodes (GPS, LiDAR, cameras, etc.)
        decltype(airsim_client_->getGpsData("", vehicle_name_)) gps_data;
        decltype(airsim_client_->getHomeGeoPoint("")) origin_gps;
        {
            std::lock_guard<std::mutex> rpc_lock(VehicleNodeBase::airsim_rpc_mutex_);
            gps_data = airsim_client_->getGpsData("", vehicle_name_);
            origin_gps = airsim_client_->getHomeGeoPoint("");
        }
        
        // Calculate GPS coordinate differences (same logic as vehicle nodes)
        double lat_diff = gps_data.gnss.geo_point.latitude - origin_gps.latitude;
        double lon_diff = gps_data.gnss.geo_point.longitude - origin_gps.longitude;
        double alt_diff = gps_data.gnss.geo_point.altitude - origin_gps.altitude;
        
        // Convert to NED meters for spawn offset
        double lat_rad = origin_gps.latitude * M_PI / 180.0;
        spawn_offset_x_.store(lat_diff * 111320.0);
        spawn_offset_y_.store(lon_diff * 111320.0 * std::cos(lat_rad));
        spawn_offset_z_.store(-alt_diff);
        spawn_offset_initialized_.store(true);
        
        RCLCPP_INFO(this->get_logger(),
            "🌐 Localization %s spawn offset: NED [%.3f, %.3f, %.3f] (GPS-derived for coordinate consistency)",
            vehicle_name_.c_str(), spawn_offset_x_.load(), spawn_offset_y_.load(), spawn_offset_z_.load());
        RCLCPP_INFO(this->get_logger(),
            "📡 Localization GPS: lat=%.6f, lon=%.6f, alt=%.3f (diff: lat=%.6f, lon=%.6f, alt=%.3f)",
            gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude,
            lat_diff, lon_diff, alt_diff);
            
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
            "Failed to initialize spawn offset for localization %s: %s",
            vehicle_name_.c_str(), e.what());
        spawn_offset_initialized_.store(true); // Mark as initialized to avoid repeated attempts
    }
}

geometry_msgs::msg::TransformStamped LocalizationNode::get_map_to_base_link_ground_truth()
{
    // Initialize spawn offset if not done yet (for diagnostic logging only)
    if (!spawn_offset_initialized_) {
        const_cast<LocalizationNode*>(this)->initialize_spawn_offset();
    }
    
    // Use AirSim as perfect "localization sensor" - in real systems this would be
    // replaced by SLAM, particle filters, EKF, etc.
    auto multirotor_state = airsim_client_->getMultirotorState(vehicle_name_);
    auto pose = multirotor_state.kinematics_estimated.pose;
    
    // REP 105 CRITICAL: Use RAW AirSim position for map→base_link (global coordinates)
    // DO NOT apply spawn offset here - that's only for odom→base_link calculations!
    // The localization computation map→odom = map→base_link * (odom→base_link)^-1 
    // requires map→base_link to be in global coordinates
    
    // // DISABLED THROTTLE:     RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
    //         "Localization %s: Using RAW AirSim position for map→base_link: NED [%.3f, %.3f, %.3f]",
    //         vehicle_name_.c_str(), pose.position.x(), pose.position.y(), pose.position.z());
    // // DISABLED THROTTLE:     RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
    //         "Vehicle spawn offset: [%.3f, %.3f, %.3f] (used by odom→base_link, not by map→base_link)",
    //         spawn_offset_x_.load(), spawn_offset_y_.load(), spawn_offset_z_.load());
    
    return convert_airsim_pose_to_transform(
        pose, map_frame_id_, base_link_frame_id_
    );
}

geometry_msgs::msg::TransformStamped LocalizationNode::convert_airsim_pose_to_transform(
    const msr::airlib::Pose& pose, const std::string& frame_id, const std::string& child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    
    // Convert AirSim NED coordinates to ROS ENU coordinates
    // AirSim: X=forward, Y=right, Z=down
    // ROS: X=forward, Y=left, Z=up  
    transform.transform.translation.x = pose.position.x();
    transform.transform.translation.y = -pose.position.y();  // Flip Y for ENU
    transform.transform.translation.z = -pose.position.z();  // Flip Z for ENU
    
    // Convert quaternion from AirSim NED to ROS ENU
    auto orientation = pose.orientation;
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = -orientation.y();  // Flip Y
    transform.transform.rotation.z = -orientation.z();  // Flip Z  
    transform.transform.rotation.w = orientation.w();
    
    return transform;
}