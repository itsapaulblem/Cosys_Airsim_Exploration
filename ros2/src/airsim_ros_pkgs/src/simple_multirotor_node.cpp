#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <airsim_interfaces/srv/takeoff.hpp>
#include <airsim_interfaces/srv/land.hpp>

class SimpleMultirotorNode : public rclcpp::Node
{
public:
    SimpleMultirotorNode(const std::string& vehicle_name) 
        : Node("airsim_" + vehicle_name)
        , vehicle_name_(vehicle_name)
        , host_ip_("localhost")
        , host_port_(41451)
    {
        // Declare parameters
        this->declare_parameter("vehicle_name", vehicle_name);
        this->declare_parameter("host_ip", host_ip_);
        this->declare_parameter("host_port", static_cast<int>(host_port_));
        
        // Get parameters
        this->get_parameter("vehicle_name", vehicle_name_);
        this->get_parameter("host_ip", host_ip_);
        int port_param = static_cast<int>(host_port_);
        this->get_parameter("host_port", port_param);
        host_port_ = static_cast<uint16_t>(port_param);
        
        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_local_ned", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("global_gps", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        
        // Create subscribers
        vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vel_cmd_body_frame", 10,
            std::bind(&SimpleMultirotorNode::vel_cmd_callback, this, std::placeholders::_1));
        
        // Create services
        takeoff_service_ = this->create_service<airsim_interfaces::srv::Takeoff>(
            "takeoff",
            std::bind(&SimpleMultirotorNode::takeoff_callback, this, std::placeholders::_1, std::placeholders::_2));
            
        land_service_ = this->create_service<airsim_interfaces::srv::Land>(
            "land",
            std::bind(&SimpleMultirotorNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Connect to AirSim
        connect_to_airsim();
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&SimpleMultirotorNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple multirotor node created for: %s", vehicle_name_.c_str());
    }

private:
    void connect_to_airsim()
    {
        try {
            airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
            airsim_client_->confirmConnection();
            airsim_client_->enableApiControl(true, vehicle_name_);
            
            RCLCPP_INFO(this->get_logger(), "Connected to AirSim for vehicle: %s", vehicle_name_.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to AirSim: %s", e.what());
        }
    }
    
    void timer_callback()
    {
        if (!airsim_client_) {
            return;
        }
        
        try {
            // Get vehicle state
            auto state = airsim_client_->getMultirotorState(vehicle_name_);
            auto timestamp = this->get_clock()->now();
            
            // Publish odometry
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = timestamp;
            odom_msg.header.frame_id = "world_ned";
            odom_msg.child_frame_id = vehicle_name_ + "_base_link";
            
            // Position (NED to ENU conversion)
            auto pos = state.getPosition();
            odom_msg.pose.pose.position.x = pos.x();
            odom_msg.pose.pose.position.y = -pos.y();
            odom_msg.pose.pose.position.z = -pos.z();
            
            // Orientation
            auto orientation = state.getOrientation();
            odom_msg.pose.pose.orientation.x = orientation.x();
            odom_msg.pose.pose.orientation.y = -orientation.y();
            odom_msg.pose.pose.orientation.z = -orientation.z();
            odom_msg.pose.pose.orientation.w = orientation.w();
            
            // Use velocity structure
            auto lin_vel = state.kinematics_estimated.twist.linear;
            odom_msg.twist.twist.linear.x = lin_vel.x();
            odom_msg.twist.twist.linear.y = -lin_vel.y();
            odom_msg.twist.twist.linear.z = -lin_vel.z();
            
            // Use angular velocity structure
            auto ang_vel = state.kinematics_estimated.twist.angular;
            odom_msg.twist.twist.angular.x = ang_vel.x();
            odom_msg.twist.twist.angular.y = -ang_vel.y();
            odom_msg.twist.twist.angular.z = -ang_vel.z();
            
            odom_pub_->publish(odom_msg);
            
            // Publish TF
            geometry_msgs::msg::TransformStamped transform;
            transform.header = odom_msg.header;
            transform.child_frame_id = odom_msg.child_frame_id;
            transform.transform.translation.x = odom_msg.pose.pose.position.x;
            transform.transform.translation.y = odom_msg.pose.pose.position.y;
            transform.transform.translation.z = odom_msg.pose.pose.position.z;
            transform.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(transform);
            
            // Publish GPS
            auto gps_data = airsim_client_->getGpsData("", vehicle_name_);
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = timestamp;
            gps_msg.header.frame_id = vehicle_name_ + "_base_link";
            gps_msg.latitude = gps_data.gnss.geo_point.latitude;
            gps_msg.longitude = gps_data.gnss.geo_point.longitude;
            gps_msg.altitude = gps_data.gnss.geo_point.altitude;
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            gps_pub_->publish(gps_msg);
            
            // Publish IMU
            auto imu_data = airsim_client_->getImuData("", vehicle_name_);
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = timestamp;
            imu_msg.header.frame_id = vehicle_name_ + "_base_link";
            
            imu_msg.orientation.x = imu_data.orientation.x();
            imu_msg.orientation.y = -imu_data.orientation.y();
            imu_msg.orientation.z = -imu_data.orientation.z();
            imu_msg.orientation.w = imu_data.orientation.w();
            
            imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
            imu_msg.angular_velocity.y = -imu_data.angular_velocity.y();
            imu_msg.angular_velocity.z = -imu_data.angular_velocity.z();
            
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
            imu_msg.linear_acceleration.y = -imu_data.linear_acceleration.y();
            imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z();
            
            imu_pub_->publish(imu_msg);
            
        }
        catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to get data for %s: %s", vehicle_name_.c_str(), e.what());
        }
    }
    
    void vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!airsim_client_) {
            return;
        }
        
        try {
            airsim_client_->moveByVelocityBodyFrameAsync(
                msg->linear.x, msg->linear.y, msg->linear.z,
                1.0f, // duration
                msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                msr::airlib::YawMode(false, 0),
                vehicle_name_
            );
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send velocity command: %s", e.what());
        }
    }
    
    bool takeoff_callback(const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
                         std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
    {
        try {
            if (request->wait_on_last_task) {
                airsim_client_->takeoffAsync(20.0f, vehicle_name_)->waitOnLastTask();
            } else {
                airsim_client_->takeoffAsync(20.0f, vehicle_name_);
            }
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff successful for: %s", vehicle_name_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %s", e.what());
            response->success = false;
            return false;
        }
    }
    
    bool land_callback(const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
                      std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
    {
        try {
            if (request->wait_on_last_task) {
                airsim_client_->landAsync(60.0f, vehicle_name_)->waitOnLastTask();
            } else {
                airsim_client_->landAsync(60.0f, vehicle_name_);
            }
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Land successful for: %s", vehicle_name_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Land failed: %s", e.what());
            response->success = false;
            return false;
        }
    }
    
    std::string vehicle_name_;
    std::string host_ip_;
    uint16_t host_port_;
    
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
    
    rclcpp::Service<airsim_interfaces::srv::Takeoff>::SharedPtr takeoff_service_;
    rclcpp::Service<airsim_interfaces::srv::Land>::SharedPtr land_service_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::string vehicle_name = "Drone1";
    if (argc > 1) {
        vehicle_name = std::string(argv[1]);
    }
    
    try {
        auto node = std::make_shared<SimpleMultirotorNode>(vehicle_name);
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("simple_multirotor_main"), 
                     "Failed to create simple multirotor node %s: %s", vehicle_name.c_str(), e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}