#include "vehicle_settings_parser.hpp"
#include "common/AirSimSettings.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>

VehicleSettingsParser::VehicleSettingsParser(const std::string& settings_file_path)
    : settings_file_path_(settings_file_path)
{
    parse_settings();
}

void VehicleSettingsParser::parse_settings()
{
    // Load AirSim settings
    if (!settings_file_path_.empty() && std::filesystem::exists(settings_file_path_)) {
        try {
            // Initialize AirSim settings from file
            msr::airlib::AirSimSettings::initializeSettings(settings_file_path_);
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("VehicleSettingsParser"), 
                       "Failed to load settings from %s: %s", settings_file_path_.c_str(), e.what());
            // Fall back to default settings
            create_default_vehicle_config();
            return;
        }
    }
    else {
        // Create simple default configuration
        create_default_vehicle_config();
        return;
    }

    // Parse vehicle configurations from AirSim settings
    try {
        const auto& vehicles = msr::airlib::AirSimSettings::singleton().vehicles;
        
        for (const auto& vehicle_pair : vehicles) {
            const std::string& vehicle_name = vehicle_pair.first;
            const auto& vehicle_setting = vehicle_pair.second.get();
            
            VehicleConfig config;
            config.name = vehicle_name;
            config.vehicle_type = vehicle_setting->vehicle_type;
            config.auto_create = vehicle_setting->auto_create;
            config.allow_api_always = false; // Default value
            
            // Parse position if specified
            if (!std::isnan(vehicle_setting->position.x())) {
                config.initial_position.x = vehicle_setting->position.x();
                config.initial_position.y = vehicle_setting->position.y();
                config.initial_position.z = vehicle_setting->position.z();
            }
            
            // Parse orientation if specified
            if (!std::isnan(vehicle_setting->rotation.yaw)) {
                config.initial_orientation.roll = vehicle_setting->rotation.roll;
                config.initial_orientation.pitch = vehicle_setting->rotation.pitch;
                config.initial_orientation.yaw = vehicle_setting->rotation.yaw;
            }
            
            // Parse sensors
            for (const auto& sensor_pair : vehicle_setting->sensors) {
                const std::string& sensor_name = sensor_pair.first;
                const auto& sensor_setting = sensor_pair.second.get();
                
                SensorConfig sensor_config;
                sensor_config.name = sensor_name;
                sensor_config.type = static_cast<int>(sensor_setting->sensor_type);
                sensor_config.enabled = sensor_setting->enabled;
                
                config.sensors.push_back(sensor_config);
            }
            
            // Parse cameras - simplified version
            for (const auto& camera_pair : vehicle_setting->cameras) {
                const std::string& camera_name = camera_pair.first;
                
                CameraConfig camera_config;
                camera_config.name = camera_name;
                // Use default values since the exact member names vary by AirSim version
                camera_config.width = 640;   // Default width
                camera_config.height = 480;  // Default height
                camera_config.fov_degrees = 90.0; // Default FOV
                
                config.cameras.push_back(camera_config);
            }
            
            vehicle_configs_[vehicle_name] = config;
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("VehicleSettingsParser"), 
                   "Error parsing AirSim settings: %s. Using default configuration.", e.what());
        create_default_vehicle_config();
    }
    
    // Set global settings
    global_config_.host_ip = "localhost";
    global_config_.host_port = 41451;
    global_config_.world_frame_id = "world_ned";
    global_config_.enable_api_control = false;
}

void VehicleSettingsParser::create_default_vehicle_config()
{
    // Create a simple default configuration with 2 drones
    VehicleConfig drone1;
    drone1.name = "Drone1";
    drone1.vehicle_type = "PX4Multirotor";
    drone1.auto_create = true;
    drone1.allow_api_always = false;
    drone1.initial_position = {0.0, 0.0, -2.0};
    drone1.initial_orientation = {0.0, 0.0, 0.0};
    
    VehicleConfig drone2;
    drone2.name = "Drone2";
    drone2.vehicle_type = "PX4Multirotor";
    drone2.auto_create = true;
    drone2.allow_api_always = false;
    drone2.initial_position = {5.0, 0.0, -2.0};
    drone2.initial_orientation = {0.0, 0.0, 0.0};
    
    vehicle_configs_["Drone1"] = drone1;
    vehicle_configs_["Drone2"] = drone2;
    
    RCLCPP_INFO(rclcpp::get_logger("VehicleSettingsParser"), 
               "Using default vehicle configuration: Drone1, Drone2");
}

std::vector<VehicleConfig> VehicleSettingsParser::get_vehicle_configs() const
{
    std::vector<VehicleConfig> configs;
    for (const auto& pair : vehicle_configs_) {
        configs.push_back(pair.second);
    }
    return configs;
}

VehicleConfig VehicleSettingsParser::get_vehicle_config(const std::string& vehicle_name) const
{
    auto it = vehicle_configs_.find(vehicle_name);
    if (it != vehicle_configs_.end()) {
        return it->second;
    }
    
    // Return default config if not found
    VehicleConfig default_config;
    default_config.name = vehicle_name;
    default_config.vehicle_type = "PX4Multirotor";
    return default_config;
}

GlobalConfig VehicleSettingsParser::get_global_config() const
{
    return global_config_;
}