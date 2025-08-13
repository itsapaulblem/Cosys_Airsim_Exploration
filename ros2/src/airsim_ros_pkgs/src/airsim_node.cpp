#include <rclcpp/rclcpp.hpp>
#include "airsim_ros_wrapper.h"
// The old ROS 2 wrapper uses a single monolithic node that manages ALL vehicles in the simulation 

// Current limitations & bottlenecks
// 1) Single point of failure 
//     - One vehicle issue (RPC timeout, exception) affects ALL vehicles
//     - Node crash brings down entire multi vehicle operation 
//     - No isolation between vehicle processing

// 2) Performance bottkenecks
// - Shared processing: All vehicles processed sequentially in timer callbacks
// - Resource contention: high frequnecy sensors compete for CPU time
// - Memory sharing: Large point clouds from multiple vehicles in same process
// - RPC queuing: single connection handles all vehicle requests

// 3) Scalability issues
// - Linear perfomance degration: processing time increases with time count 
// - Timer synchronisation: all vehicles must complete processing wihtin timer period
// - Memory growth: unbounded growth with additional vehicles/sensors

// 4) Development & Debugging Challenges
// - Mixed logs: all vehicle logs intermixed in single node output
// - Complex state: hard to isolate issues to specific vehicles 
// - Restart impact: restarting node affects all vehicles simultaneously 

// 5) Resource management
// - CPU binding: all processing on single core/thread
// - Memory pooling: no per vehicle memory limits
// - Network sharing: isngle tcp connection for all vehicles


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node", node_options);
    std::shared_ptr<rclcpp::Node> nh_img = nh->create_sub_node("img");
    std::shared_ptr<rclcpp::Node> nh_lidar = nh->create_sub_node("lidar");
    std::shared_ptr<rclcpp::Node> nh_gpulidar = nh->create_sub_node("gpulidar");
    std::shared_ptr<rclcpp::Node> nh_echo = nh->create_sub_node("echo");
    std::string host_ip;
    uint16_t host_port = 41451;
    bool enable_api_control = false;
    bool enable_object_transforms_list = true;
    nh->get_parameter("host_ip", host_ip);
    nh->get_parameter("host_port", host_port);
    nh->get_parameter("enable_api_control", enable_api_control);
    nh->get_parameter("enable_object_transforms_list", enable_object_transforms_list);
    auto callbackGroup = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh_img, nh_lidar, nh_gpulidar, nh_echo, host_ip, callbackGroup, enable_api_control, enable_object_transforms_list, host_port);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();

    return 0;
}