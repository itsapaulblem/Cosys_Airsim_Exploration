// Point Cloud Merger Node
// Merges LiDAR point clouds from 4 drones into a single unified point cloud
// for OctoMap 3D mapping and visualization
//
// Subscribes to:
//   /Drone1/Lidar/points
//   /Drone2/Lidar/points
//   /Drone3/Lidar/points
//   /Drone4/Lidar/points
//
// Publishes to:
//   /merged_pointcloud (sensor_msgs::PointCloud2)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <algorithm>
#include <array>
#include <memory>

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : Node("pointcloud_merger")
    {
        // Declare parameters
        this->declare_parameter<int>("num_drones", 4);
        this->declare_parameter<double>("merge_rate_hz", 10.0);
        this->declare_parameter<bool>("enable_downsampling", false);  // Disabled by default for safety
        this->declare_parameter<double>("voxel_leaf_size", 0.05);  // 5cm voxels
        this->declare_parameter<bool>("use_tf_transform", true);   // Enable TF transformation
        this->declare_parameter<std::string>("target_frame", "map");  // Target frame for transformation

        // Get parameters
        num_drones_ = this->get_parameter("num_drones").as_int();
        double merge_rate = this->get_parameter("merge_rate_hz").as_double();
        enable_downsampling_ = this->get_parameter("enable_downsampling").as_bool();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        use_tf_transform_ = this->get_parameter("use_tf_transform").as_bool();
        target_frame_ = this->get_parameter("target_frame").as_string();

        RCLCPP_INFO(this->get_logger(), "Point Cloud Merger Node Starting...");
        RCLCPP_INFO(this->get_logger(), "  - Number of drones: %d", num_drones_);
        RCLCPP_INFO(this->get_logger(), "  - Merge rate: %.1f Hz", merge_rate);
        RCLCPP_INFO(this->get_logger(), "  - TF transformation: %s", use_tf_transform_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "  - Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Downsampling: %s", enable_downsampling_ ? "enabled" : "disabled");
        if (enable_downsampling_) {
            RCLCPP_INFO(this->get_logger(), "  - Voxel leaf size: %.3f m", voxel_leaf_size_);
        }

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "  - TF2 listener initialized");

        // Initialize cloud storage
        latest_clouds_.resize(num_drones_);
        cloud_received_.resize(num_drones_, false);

        // Create subscribers for each drone's LiDAR
        for (int i = 0; i < num_drones_; ++i) {
            std::string topic = "/Drone" + std::to_string(i + 1) + "/Lidar/points";
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic,
                rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
                [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->lidar_callback(msg, i);
                }
            );
            lidar_subscribers_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "  - Subscribed to: %s", topic.c_str());
        }

        // Create publisher for merged point cloud
        merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/merged_pointcloud", 10);
        RCLCPP_INFO(this->get_logger(), "  - Publishing to: /merged_pointcloud");

        // Create timer for periodic merging (reduces CPU load)
        double period = 1.0 / merge_rate;
        merge_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&PointCloudMerger::merge_and_publish, this));

        RCLCPP_INFO(this->get_logger(), "Point Cloud Merger Node Ready!");
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int drone_idx)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_clouds_[drone_idx] = msg;
        cloud_received_[drone_idx] = true;
    }

    void merge_and_publish()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        // Count how many clouds we have
        int available_clouds = 0;
        for (bool received : cloud_received_) {
            if (received) available_clouds++;
        }

        if (available_clouds == 0) {
            return;  // No data yet
        }

        // Create PCL point cloud for merging
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        merged_cloud->header.frame_id = target_frame_;

        // Merge all available clouds (with optional TF transformation)
        int clouds_merged = 0;
        int clouds_skipped = 0;

        for (int i = 0; i < num_drones_; ++i) {
            if (!cloud_received_[i] || !latest_clouds_[i]) {
                continue;
            }

            sensor_msgs::msg::PointCloud2 cloud_to_merge;

            // Transform cloud to target frame if TF transformation is enabled
            if (use_tf_transform_) {
                try {
                    // Lookup transform from source frame to target frame
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                        target_frame_,
                        latest_clouds_[i]->header.frame_id,
                        tf2::TimePointZero,  // Get latest available transform
                        tf2::durationFromSec(0.1)  // 100ms timeout
                    );

                    // Transform the point cloud
                    tf2::doTransform(*latest_clouds_[i], cloud_to_merge, transform);

                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        5000,  // Log every 5 seconds max
                        "Transform failed for cloud %d (%s → %s): %s. Skipping this cloud.",
                        i + 1,
                        latest_clouds_[i]->header.frame_id.c_str(),
                        target_frame_.c_str(),
                        ex.what()
                    );
                    clouds_skipped++;
                    continue;
                }
            } else {
                // No transformation - use cloud as-is (for debugging/testing)
                cloud_to_merge = *latest_clouds_[i];
            }

            // Convert ROS PointCloud2 to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr drone_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cloud_to_merge, *drone_cloud);

            // Concatenate transformed cloud
            *merged_cloud += *drone_cloud;
            clouds_merged++;
        }

        // Check if no clouds were successfully merged
        if (clouds_merged == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,  // Log every 5 seconds max
                "No clouds merged! Possible causes: "
                "(1) No LiDAR data received yet, "
                "(2) TF transforms missing (check: ros2 run tf2_ros tf2_echo map Drone1/Lidar_link), "
                "(3) enable_localization:=false in launch command"
            );
            return;  // Don't publish empty/invalid cloud
        }

        if (merged_cloud->empty()) {
            return;
        }

        // CRITICAL: Check spatial bounds before downsampling to detect TF issues early
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*merged_cloud, min_pt, max_pt);

        float span_x = max_pt.x - min_pt.x;
        float span_y = max_pt.y - min_pt.y;
        float span_z = max_pt.z - min_pt.z;
        float max_span = std::max({span_x, span_y, span_z});

        // Calculate safe maximum span based on voxel leaf size to prevent VoxelGrid overflow
        // PCL VoxelGrid uses int32 indices: max = 2^31 - 1
        // Max voxels per axis: cbrt(2^31) ≈ 1290
        // Safe limit with margin: 1000 voxels per axis
        float safe_max_span = 1000.0f * voxel_leaf_size_;  // For 0.05m: 50m, for 0.1m: 100m

        // Additionally check for obviously wrong TF (> 500m regardless of voxel size)
        bool obviously_wrong_tf = max_span > 500.0f;
        bool would_overflow_voxelgrid = enable_downsampling_ && (max_span > safe_max_span);

        if (obviously_wrong_tf || would_overflow_voxelgrid) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Merged cloud has spatial extent: [%.1f x %.1f x %.1f]m (max: %.1f m)",
                span_x, span_y, span_z, max_span
            );

            if (would_overflow_voxelgrid) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "This will cause VoxelGrid OVERFLOW! With voxel_leaf_size=%.3fm, safe max span is %.1fm",
                    voxel_leaf_size_, safe_max_span
                );
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "FIX: Increase voxel_leaf_size (e.g., 0.2m) OR fix TF transformation"
                );
            }

            if (obviously_wrong_tf) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Cloud spans >500m - This indicates TF transformation FAILED. Clouds are in different coordinate frames!"
                );
            }

            if (use_tf_transform_) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "TF transformation is ENABLED but failing. Verify 'map' frame exists and transforms are published:"
                );
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "  1. Check launch: enable_localization:=true"
                );
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "  2. Test TF: ros2 run tf2_ros tf2_echo map Drone1/Lidar_link"
                );
            } else {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "TF transformation is DISABLED (use_tf_transform:=false). This WILL cause coordinate issues!"
                );
            }

            return;  // Don't publish garbage data or attempt VoxelGrid (would overflow)
        }

        // Optional downsampling using VoxelGrid filter
        if (enable_downsampling_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(merged_cloud);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*filtered_cloud);
            merged_cloud = filtered_cloud;
        }

        // Convert back to ROS PointCloud2 and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*merged_cloud, output_msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = target_frame_;

        merged_pub_->publish(output_msg);

        // Log statistics periodically
        static int count = 0;
        if (++count % 100 == 0) {  // Every 10 seconds at 10 Hz
            if (clouds_skipped > 0) {
                RCLCPP_INFO(this->get_logger(),
                           "Merged %d/%d clouds (%d skipped due to TF), %zu points total",
                           clouds_merged, num_drones_, clouds_skipped, merged_cloud->size());
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "Merged %d/%d clouds, %zu points total",
                           clouds_merged, num_drones_, merged_cloud->size());
            }
        }
    }

    // Parameters
    int num_drones_;
    bool enable_downsampling_;
    double voxel_leaf_size_;
    bool use_tf_transform_;
    std::string target_frame_;

    // TF2 components
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers and publisher
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
    rclcpp::TimerBase::SharedPtr merge_timer_;

    // Cloud storage
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
    std::vector<bool> cloud_received_;
    std::mutex cloud_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
