#include "VehicleNodeFactory.h"
#include "common/AirSimSettings.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>

/**
 * @brief Test program to verify concurrent RPC connections to AirSim server
 * 
 * This program creates multiple vehicle nodes and tests their ability to
 * connect to AirSim simultaneously, verifying that the multi-node architecture
 * can handle concurrent RPC communications without conflicts.
 */

class ConcurrentRpcTester
{
public:
    ConcurrentRpcTester(const std::string& host_ip = "localhost", uint16_t host_port = 41451)
        : host_ip_(host_ip), host_port_(host_port), success_count_(0), failure_count_(0)
    {
        rclcpp::init(0, nullptr);
    }

    ~ConcurrentRpcTester()
    {
        rclcpp::shutdown();
    }

    /**
     * @brief Create test vehicle configurations
     */
    void createTestConfigurations()
    {
        using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
        
        // Create test configurations for different vehicle types
        
        // Drone 1 - MultiRotor
        auto drone1_config = std::make_shared<VehicleSetting>();
        drone1_config->vehicle_type = "SimpleFlight";
        drone1_config->position = msr::airlib::Vector3r(0, 0, 0);
        drone1_config->rotation = msr::airlib::AirSimSettings::Rotation();
        test_settings_.vehicles["drone1"] = drone1_config;

        // Drone 2 - MultiRotor
        auto drone2_config = std::make_shared<VehicleSetting>();
        drone2_config->vehicle_type = "SimpleFlight";
        drone2_config->position = msr::airlib::Vector3r(10, 0, 0);
        drone2_config->rotation = msr::airlib::AirSimSettings::Rotation();
        test_settings_.vehicles["drone2"] = drone2_config;

        // Car 1 - Ground Vehicle
        auto car1_config = std::make_shared<VehicleSetting>();
        car1_config->vehicle_type = "PhysXCar";
        car1_config->position = msr::airlib::Vector3r(0, 10, 0);
        car1_config->rotation = msr::airlib::AirSimSettings::Rotation();
        test_settings_.vehicles["car1"] = car1_config;

        // Computer Vision entity
        auto cv1_config = std::make_shared<VehicleSetting>();
        cv1_config->vehicle_type = "ComputerVision";
        cv1_config->position = msr::airlib::Vector3r(-10, 0, 0);
        cv1_config->rotation = msr::airlib::AirSimSettings::Rotation();
        test_settings_.vehicles["cv1"] = cv1_config;

        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Created %zu test vehicle configurations", test_settings_.vehicles.size());
    }

    /**
     * @brief Test individual vehicle node initialization
     */
    bool testVehicleNodeInitialization(const std::string& vehicle_name, 
                                      const msr::airlib::AirSimSettings::VehicleSetting& config)
    {
        try {
            RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                        "Testing initialization for vehicle: %s", vehicle_name.c_str());

            // Create vehicle node
            auto vehicle_node = VehicleNodeFactory::createVehicleNode(
                vehicle_name, config, host_ip_, host_port_, "world", true);

            if (!vehicle_node) {
                RCLCPP_ERROR(rclcpp::get_logger("ConcurrentRpcTester"), 
                             "Failed to create vehicle node: %s", vehicle_name.c_str());
                return false;
            }

            // Initialize the vehicle node (this creates RPC connections)
            vehicle_node->initialize();

            // Test basic RPC operation - confirmConnection
            // This is handled internally by the initialize() method
            
            RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                        "Successfully initialized vehicle: %s", vehicle_name.c_str());

            // Store the node for later testing
            {
                std::lock_guard<std::mutex> lock(nodes_mutex_);
                vehicle_nodes_.push_back(vehicle_node);
            }

            success_count_++;
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ConcurrentRpcTester"), 
                         "Exception during initialization of %s: %s", vehicle_name.c_str(), e.what());
            failure_count_++;
            return false;
        }
    }

    /**
     * @brief Test concurrent vehicle node initialization
     */
    void testConcurrentInitialization()
    {
        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Starting concurrent initialization test...");

        std::vector<std::thread> threads;
        
        // Create threads for each vehicle
        for (const auto& vehicle_pair : test_settings_.vehicles) {
            const std::string& vehicle_name = vehicle_pair.first;
            const auto& vehicle_config = *vehicle_pair.second;
            
            threads.emplace_back([this, vehicle_name, vehicle_config]() {
                testVehicleNodeInitialization(vehicle_name, vehicle_config);
            });
        }

        // Wait for all threads to complete
        for (auto& thread : threads) {
            thread.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Concurrent initialization test completed. Success: %d, Failures: %d", 
                    success_count_.load(), failure_count_.load());
    }

    /**
     * @brief Test sequential vehicle node initialization
     */
    void testSequentialInitialization()
    {
        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Starting sequential initialization test...");

        success_count_ = 0;
        failure_count_ = 0;
        vehicle_nodes_.clear();

        for (const auto& vehicle_pair : test_settings_.vehicles) {
            const std::string& vehicle_name = vehicle_pair.first;
            const auto& vehicle_config = *vehicle_pair.second;
            
            testVehicleNodeInitialization(vehicle_name, vehicle_config);
            
            // Small delay between initializations
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Sequential initialization test completed. Success: %d, Failures: %d", 
                    success_count_.load(), failure_count_.load());
    }

    /**
     * @brief Test factory batch creation
     */
    void testFactoryBatchCreation()
    {
        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Testing factory batch creation...");

        try {
            auto nodes = VehicleNodeFactory::createVehicleNodesFromSettings(
                test_settings_, host_ip_, host_port_, "world", true);
            
            RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                        "Factory created %zu vehicle nodes", nodes.size());

            // Initialize all nodes
            for (auto& node : nodes) {
                try {
                    node->initialize();
                    RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                                "Initialized node: %s", node->get_name());
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("ConcurrentRpcTester"), 
                                 "Failed to initialize node %s: %s", node->get_name(), e.what());
                }
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ConcurrentRpcTester"), 
                         "Factory batch creation failed: %s", e.what());
        }
    }

    /**
     * @brief Run all tests
     */
    void runTests()
    {
        createTestConfigurations();

        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "Starting RPC connection tests with AirSim server at %s:%d", 
                    host_ip_.c_str(), host_port_);

        // Test 1: Sequential initialization
        testSequentialInitialization();
        
        // Small delay between tests
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test 2: Concurrent initialization
        testConcurrentInitialization();
        
        // Small delay between tests
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test 3: Factory batch creation
        testFactoryBatchCreation();

        // Summary
        RCLCPP_INFO(rclcpp::get_logger("ConcurrentRpcTester"), 
                    "All tests completed. Check logs for detailed results.");
    }

private:
    std::string host_ip_;
    uint16_t host_port_;
    msr::airlib::AirSimSettings test_settings_;
    std::vector<std::shared_ptr<VehicleNodeBase>> vehicle_nodes_;
    std::mutex nodes_mutex_;
    std::atomic<int> success_count_;
    std::atomic<int> failure_count_;
};

int main(int argc, char** argv)
{
    std::string host_ip = "localhost";
    uint16_t host_port = 41451;

    // Parse command line arguments
    if (argc >= 2) {
        host_ip = argv[1];
    }
    if (argc >= 3) {
        host_port = static_cast<uint16_t>(std::stoi(argv[2]));
    }

    std::cout << "=== AirSim Concurrent RPC Connection Test ===" << std::endl;
    std::cout << "Testing connections to AirSim server at " << host_ip << ":" << host_port << std::endl;
    std::cout << "Make sure AirSim is running before starting this test." << std::endl;
    std::cout << "==========================================" << std::endl;

    try {
        ConcurrentRpcTester tester(host_ip, host_port);
        tester.runTests();
        
        std::cout << "Test completed successfully!" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}