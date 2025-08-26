#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <airsim_ros_pkgs_msg/target_detection.hpp>
#include <airsim_ros_pkgs/srv/search_target.hpp>
#include <airsim_ros_pkgs/srv/track_target.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <mutex>

struct Point3D {
    float x, y, z;
};

class LidarSearchNode : public rclcpp::Node {
public:
    LidarSearchNode() : Node("lidar_search_node") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar0/points", 10,
            std::bind(&LidarSearchNode::lidar_callback, this, std::placeholders::_1));
        
        detection_pub_ = this->create_publisher<airsim_ros_pkgs::msg::TargetDetection>(
            "target_detection", 10);

        search_service_ = this->create_service<airsim_ros_pkgs::srv::SearchTarget>(
            "search_target",
            std::bind(&LidarSearchNode::search_service_cb, this, std::placeholders::_1, std::placeholders::_2));
        
        track_service_ = this->create_service<airsim_ros_pkgs::srv::TrackTarget>(
            "track_target",
            std::bind(&LidarSearchNode::track_service_cb, this, std::placeholders::_1, std::placeholders::_2));
        

        // AirSim client
        airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>("localhost", 41451);
        airsim_client_->confirmConnection();
        vehicle_name_ = "Drone1";
        RCLCPP_INFO(this->get_logger(), "Lidar Search Node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<airsim_ros_pkgs::msg::TargetDetection>::SharedPtr detection_pub_;
    rclcpp::Service<airsim_ros_pkgs::srv::SearchTarget>::SharedPtr search_service_;
    rclcpp::Service<airsim_ros_pkgs::srv::TrackTarget>::SharedPtr track_service_;

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    std::string vehicle_name_;

    // Spline parameters
    float spline_cx = 0.0, spline_cy = 0.0, spline_radius = 0.0;
    float last_confidence_ = 0.0; 
    std::mutex spline_mutex_; 

    // Helper: Extract points from PointCloud2
    std::vector<Point3D> extract_points(const sensor_msgs::PointCloud2::SharedPtr& msg) {
        std::vector<Point3D> points;
        size_t n = msg->width;
        for (size_t i = 0; i < n; ++i) {
            float x, y, z;
            memcpy(&x, &msg->data[i * msg->point_step + 0], sizeof(float));
            memcpy(&y, &msg->data[i * msg->point_step + 4], sizeof(float));
            memcpy(&z, &msg->data[i * msg->point_step + 8], sizeof(float));
            points.push_back({x, y, z});
        }
        return points;
    }

    // Helper: Fit circle 
    bool fit_circle(const std::vector<Point3D>& points, float& cx, float& cy, float& radius) {
        if (points.size() < 10) {
            return false;
        }

        float sum_x = 0;
        float sum_y = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        cx = sum_x / points.size();
        cy = sum_y / points.size();
        float sum_r = 0;
        for (const auto& p : points) {
            sum_r += std::hypot(p.x - cx, p.y - cy);
        }
        radius = sum_r / points.size();
        return true;
    }

    // Helper: Cluster points (naive, by distance to center)
    std::vector<Point3D> cluster_points(const std::vector<Point3D>& points, float cx, float cy, float radius, float tol = 0.5) {
        std::vector<Point3D> cluster;
        for (const auto& p : points) {
            float r = std::hypot(p.x - cx, p.y - cy);
            if (std::abs(r - radius) < tol) {
                cluster.push_back(p);
            }
        }
        return cluster;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto points = extract_points(msg);
        float cx, cy, radius;
        bool found = fit_circle(points, cx, cy, radius);

        std::lock_guard<std::mutex> lock(spline_mutex_);
        if (found) {
            // Cluster points near the circle
            auto cluster = cluster_points(points, cx, cy, radius, 0.5);
            last_confidence_ = cluster.size() > 10 ? 0.95f : 0.0f;
            spline_cx_ = cx;
            spline_cy_ = cy;
            spline_radius_ = radius;
        } else {
            last_confidence_ = 0.0f;
        }

        airsim_interfaces::msg::TargetDetection det;
        det.header.stamp = this->now();
        det.vehicle_name = vehicle_name_;
        det.target.x = spline_cx_;
        det.target.y = spline_cy_;
        det.target.z = 0.0; // Assume this is ground level
        det.confidence = last_confidence_;
        detection_pub_->publish(det);
    }

    void search_service_cb(
        const std::shared_ptr<airsim_interfaces::srv::SearchTarget::Request> req, 
        std::shared_ptr<airsim_interfaces::srv::SearchTarget::Response> res) {
            (void) req;
            std::lock_guard<std::mutex> lock(spline_mutex_);
            res->success = last_confidence_ > 0.5;
            res->target_x = spline_cx_;
            res->target_y  = spline_cy_;
            res->target_z  = 0.0;
            res->confidence = last_confidence_;
            res->message = res->success ? "Spline found" : "Spline not found";
        }

    void track_service_cb(
        const std::shared_ptr<airsim_interfaces::srv::TrackTarget::Request> req,
        std::shared_ptr<airsim_interfaces::srv::TrackTarget::Response> res) {
            std::lock_guard<std::mutex> lock(spline_mutex_);
            if (last_confidence_ < 0.5) {
                res->success = false;
                res->message = "No spline detected";
                return;
            }

            // Compute next waypoint along the spline (circle)
            float theta = std::atan2(req->target_y - spline_cy_, req->target_x - spline_cx_);
            float next_theta = theta + 0.1f;
            float next_x = spline_cx_ + spline_radius_ * std::cos(next_theta);
            float next_y = spline_cy_ + spline_radius_ * std::sin(next_theta);
            float next_z = req->target_z; // Maintain altitude

            try {
                airsim_client_->moveToPositionAsync(
                    next_x, next_y, next_z, 3.0f, 
                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                    msr::airlib::YawMode(false, 0.0f),
                    -1, 1, vehicle_name_
                )->waitOnLastTask();

                res->success = true;
                res->message = "Tracking spline, next waypoint: (" +
                    std::to_string(next_x) + ", " + std::to_string(next_y) + ", " + std::to_string(next_z) + ")";
                
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
            } catch (const std::exception& e) {
                res->success = false;
                res->message = std::string("AirSim command failed: ") + e.what();
                RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            }
        }
    };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSearchNode>());
    rclcpp::shutdown();
    return 0;
}
