#include "VehicleNodeFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>

std::shared_ptr<VehicleNodeBase> VehicleNodeFactory::createVehicleNode(
    const std::string& vehicle_name,
    const VehicleSetting& vehicle_config,
    const std::string& host_ip,
    uint16_t host_port,
    const std::string& world_frame_id,
    bool enable_api_control)
{
    // Validate configuration
    if (!validateVehicleConfig(vehicle_name, vehicle_config)) {
        RCLCPP_ERROR(rclcpp::get_logger("VehicleNodeFactory"), 
                     "Invalid vehicle configuration for: %s", vehicle_name.c_str());
        return nullptr;
    }

    // Determine vehicle type
    VehicleType vehicle_type = getVehicleType(vehicle_config);
    
    RCLCPP_INFO(rclcpp::get_logger("VehicleNodeFactory"), 
                "Creating %s node for vehicle: %s", 
                getVehicleTypeString(vehicle_type).c_str(), vehicle_name.c_str());

    // Create appropriate vehicle node based on type
    switch (vehicle_type) {
        case VehicleType::MULTIROTOR:
            return std::make_shared<MultiRotorNode>(
                vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control);
        
        case VehicleType::CAR:
            return std::make_shared<CarNode>(
                vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control);
        
        case VehicleType::COMPUTER_VISION:
            return std::make_shared<ComputerVisionNode>(
                vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control);
        
        case VehicleType::UNKNOWN:
        default:
            RCLCPP_ERROR(rclcpp::get_logger("VehicleNodeFactory"), 
                         "Unknown or unsupported vehicle type for: %s", vehicle_name.c_str());
            return nullptr;
    }
}

VehicleNodeFactory::VehicleType VehicleNodeFactory::getVehicleType(const VehicleSetting& vehicle_config)
{
    // Check if vehicle_type is explicitly set in the configuration
    if (!vehicle_config.vehicle_type.empty()) {
        return getVehicleTypeFromString(vehicle_config.vehicle_type);
    }
    
    // Fallback to default based on common naming patterns
    // This is a safety fallback if vehicle_type is not set
    RCLCPP_WARN(rclcpp::get_logger("VehicleNodeFactory"), 
                "Vehicle type not specified in configuration, using default fallback logic");
    
    return VehicleType::MULTIROTOR; // Default to multirotor
}

VehicleNodeFactory::VehicleType VehicleNodeFactory::getVehicleTypeFromString(const std::string& vehicle_type_str)
{
    // Convert to lowercase for case-insensitive comparison
    std::string type_lower = vehicle_type_str;
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);
    
    // Check for multirotor types
    if (type_lower == "multirotor" || type_lower == "simpleflight" || 
        type_lower == "px4" || type_lower == "arducopter" || 
        type_lower == "drone" || type_lower == "quadcopter") {
        return VehicleType::MULTIROTOR;
    }
    
    // Check for car types
    if (type_lower == "car" || type_lower == "physxcar" || 
        type_lower == "skidsteer" || type_lower == "ground") {
        return VehicleType::CAR;
    }
    
    // Check for computer vision types
    if (type_lower == "computervision" || type_lower == "computer_vision" || 
        type_lower == "cv" || type_lower == "camera") {
        return VehicleType::COMPUTER_VISION;
    }
    
    RCLCPP_WARN(rclcpp::get_logger("VehicleNodeFactory"), 
                "Unknown vehicle type string: %s", vehicle_type_str.c_str());
    return VehicleType::UNKNOWN;
}

std::string VehicleNodeFactory::getVehicleTypeString(VehicleType vehicle_type)
{
    switch (vehicle_type) {
        case VehicleType::MULTIROTOR:
            return "MultiRotor";
        case VehicleType::CAR:
            return "Car";
        case VehicleType::COMPUTER_VISION:
            return "ComputerVision";
        case VehicleType::UNKNOWN:
        default:
            return "Unknown";
    }
}

bool VehicleNodeFactory::validateVehicleConfig(const std::string& vehicle_name, const VehicleSetting& vehicle_config)
{
    // Check for valid vehicle name
    if (vehicle_name.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("VehicleNodeFactory"), "Vehicle name cannot be empty");
        return false;
    }
    
    // Check for valid vehicle type
    VehicleType vehicle_type = getVehicleType(vehicle_config);
    if (vehicle_type == VehicleType::UNKNOWN) {
        RCLCPP_ERROR(rclcpp::get_logger("VehicleNodeFactory"), 
                     "Unknown vehicle type for vehicle: %s", vehicle_name.c_str());
        return false;
    }
    
    // Additional validation could be added here for:
    // - Required sensors for specific vehicle types
    // - Valid pose configurations
    // - Camera settings validation
    // - etc.
    
    RCLCPP_DEBUG(rclcpp::get_logger("VehicleNodeFactory"), 
                 "Vehicle configuration validated for: %s (%s)", 
                 vehicle_name.c_str(), getVehicleTypeString(vehicle_type).c_str());
    
    return true;
}

std::vector<std::shared_ptr<VehicleNodeBase>> VehicleNodeFactory::createVehicleNodesFromSettings(
    const msr::airlib::AirSimSettings& airsim_settings,
    const std::string& host_ip,
    uint16_t host_port,
    const std::string& world_frame_id,
    bool enable_api_control)
{
    std::vector<std::shared_ptr<VehicleNodeBase>> vehicle_nodes;
    
    RCLCPP_INFO(rclcpp::get_logger("VehicleNodeFactory"), 
                "Creating vehicle nodes from AirSim settings...");
    
    // Iterate through all vehicles in the settings
    for (const auto& vehicle_setting_pair : airsim_settings.vehicles) {
        const std::string& vehicle_name = vehicle_setting_pair.first;
        const VehicleSetting& vehicle_config = *vehicle_setting_pair.second;
        
        RCLCPP_INFO(rclcpp::get_logger("VehicleNodeFactory"), 
                    "Processing vehicle: %s", vehicle_name.c_str());
        
        // Create vehicle node
        auto vehicle_node = createVehicleNode(
            vehicle_name, vehicle_config, host_ip, host_port, world_frame_id, enable_api_control);
        
        if (vehicle_node) {
            vehicle_nodes.push_back(vehicle_node);
            RCLCPP_INFO(rclcpp::get_logger("VehicleNodeFactory"), 
                        "Successfully created node for vehicle: %s", vehicle_name.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("VehicleNodeFactory"), 
                         "Failed to create node for vehicle: %s", vehicle_name.c_str());
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("VehicleNodeFactory"), 
                "Created %zu vehicle nodes from settings", vehicle_nodes.size());
    
    return vehicle_nodes;
}