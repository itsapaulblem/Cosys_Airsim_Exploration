/* LEGACY_PURPOSE
* This file represents the original monolithic approach to AirSim ROS 2 integration, 
* where a single node managed ALL vehicles in the simulation simultaneously. 
* It is maintained for backward compatibility and reference purposes only.
*
* WHY DEPRECATED
* The old ROS 2 wrapper uses a single monolithic node that manages ALL vehicles
* in the simulation, which creates several critical limitations and bottlenecks:
*
* Current limitations & bottlenecks
* 1) Single point of failure 
*     - One vehicle issue (RPC timeout, exception) affects ALL vehicles
*     - Node crash brings down entire multi vehicle operation 
*     - No isolation between vehicle processing

* 2) Performance bottlenecks
* - Shared processing: All vehicles processed sequentially in timer callbacks
* - Resource contention: high frequency sensors compete for CPU time
* - Memory sharing: Large point clouds from multiple vehicles in same process
* - RPC queuing: single connection handles all vehicle requests

* 3) Scalability issues
* - Linear performance degradation: Processing time increases with time count
* - Timer synchronisation: All vehicles must complete processing within timer period
* - Memory growth: Unbounded growth with additional vehicles/sensors

* 4) Development & Debugging Challenges
* - Mixed logs: All vehicle logs intermixed in single node output
* - Complex state: Hard to isolate issues to specific vehicles
* - Restart impact: Restarting node affects all vehicles simultaneously

* 5) Resource management
* - CPU binding: All processing on single core/thread
* - Memory pooling: No per vehicle memory limits
* - Network sharing: Single TCP connection for all vehicles

 * MIGRATION_PATH
 * To migrate from legacy to modern architecture:
 * 1. Replace airsim_node.launch.py with single_drone.launch.py or multi_drone.launch.py
 * 2. Update topic names to include vehicle namespaces (/drone1/, /drone2/, etc.)
 * 3. Use coordination_node services for fleet operations (/takeoff_all, /land_all)
 * 4. Leverage per-vehicle fault isolation and debugging capabilities
 *
 * @warning This file should not be used for new deployments. Use the modular
 *          architecture (multirotor_node + coordination_node) instead.
*/

#include <rclcpp/rclcpp.hpp>
#include "airsim_ros_wrapper.h"

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