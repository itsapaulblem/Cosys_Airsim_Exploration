#pragma once

#include "vehicles/VehicleNodeBase.h"
#include "vehicles/MultiRotorNode.h"
#include "vehicles/CarNode.h"
#include "vehicles/ComputerVisionNode.h"
#include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include <memory>
#include <string>

/**
 * @brief Factory class for creating vehicle nodes based on configuration
 * 
 * This factory handles the instantiation of the appropriate vehicle node type
 * based on the vehicle configuration from settings.json. It provides a unified
 * interface for creating vehicles in the multi-node architecture.
 */
class VehicleNodeFactory
{
    using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;

public:
    /**
     * @brief Vehicle types supported by the factory
     */
    enum class VehicleType
    {
        MULTIROTOR,
        CAR,
        COMPUTER_VISION,
        UNKNOWN
    };

    /**
     * @brief Create a vehicle node based on configuration
     * @param vehicle_name Unique name of the vehicle
     * @param vehicle_config Vehicle configuration from settings.json
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control for this vehicle
     * @return Shared pointer to the created vehicle node
     */
    static std::shared_ptr<VehicleNodeBase> createVehicleNode(
        const std::string& vehicle_name,
        const VehicleSetting& vehicle_config,
        const std::string& host_ip = "localhost",
        uint16_t host_port = 41451,
        const std::string& world_frame_id = "world",
        bool enable_api_control = true);

    /**
     * @brief Determine vehicle type from configuration
     * @param vehicle_config Vehicle configuration from settings.json
     * @return VehicleType enum value
     */
    static VehicleType getVehicleType(const VehicleSetting& vehicle_config);

    /**
     * @brief Get vehicle type from string
     * @param vehicle_type_str Vehicle type string from configuration
     * @return VehicleType enum value
     */
    static VehicleType getVehicleTypeFromString(const std::string& vehicle_type_str);

    /**
     * @brief Get string representation of vehicle type
     * @param vehicle_type VehicleType enum value
     * @return String representation
     */
    static std::string getVehicleTypeString(VehicleType vehicle_type);

    /**
     * @brief Validate vehicle configuration
     * @param vehicle_name Vehicle name
     * @param vehicle_config Vehicle configuration
     * @return True if configuration is valid
     */
    static bool validateVehicleConfig(const std::string& vehicle_name, const VehicleSetting& vehicle_config);

    /**
     * @brief Create multiple vehicle nodes from AirSim settings
     * @param airsim_settings AirSim settings containing vehicle configurations
     * @param host_ip AirSim server host IP
     * @param host_port AirSim server port
     * @param world_frame_id World coordinate frame ID
     * @param enable_api_control Whether to enable API control
     * @return Vector of created vehicle nodes
     */
    static std::vector<std::shared_ptr<VehicleNodeBase>> createVehicleNodesFromSettings(
        const msr::airlib::AirSimSettings& airsim_settings,
        const std::string& host_ip = "localhost",
        uint16_t host_port = 41451,
        const std::string& world_frame_id = "world",
        bool enable_api_control = true);

private:
    /**
     * @brief Private constructor to prevent instantiation
     */
    VehicleNodeFactory() = delete;
};