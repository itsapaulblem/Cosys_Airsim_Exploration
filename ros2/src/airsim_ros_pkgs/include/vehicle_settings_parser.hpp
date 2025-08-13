
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <filesystem>

struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Orientation {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct SensorConfig {
    std::string name;
    int type;
    bool enabled = true;
};

struct CameraConfig {
    std::string name;
    int width = 640;
    int height = 480;
    double fov_degrees = 90.0;
};

struct VehicleConfig {
    std::string name;
    std::string vehicle_type = "SimpleFlight";
    bool auto_create = true;
    bool allow_api_always = false;
    Position initial_position;
    Orientation initial_orientation;
    std::vector<SensorConfig> sensors;
    std::vector<CameraConfig> cameras;
};

struct GlobalConfig {
    std::string host_ip = "localhost";
    uint16_t host_port = 41451;
    std::string world_frame_id = "world_ned";
    bool enable_api_control = false;
};

class VehicleSettingsParser {
public: 
    explicit VehicleSettingsParser(const std::string& settings_file_path = "");

    std::vector<VehicleConfig> get_vehicle_configs() const; 
    VehicleConfig get_vehicle_config(const std::string& vehicle_name) const; 
    GlobalConfig get_global_config() const;

    bool has_vehicle(const std::string& vehicle_name) const {
        return vehicle_configs_.find(vehicle_name) != vehicle_configs_.end();
    }

    size_t get_vehicle_count() const {
        return vehicle_configs_.size();
    }

private:
    void parse_settings();
    void create_default_vehicle_config();

    std::string settings_file_path_;
    std::unordered_map<std::string, VehicleConfig> vehicle_configs_;
    GlobalConfig global_config_;
};
