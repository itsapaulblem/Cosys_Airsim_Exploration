# Vehicle capabilities message
# Describes what each vehicle can do for mission planning

# Vehicle identification
string vehicle_name
string vehicle_type         # "multirotor", "fixed_wing", "car", "boat"

# Flight/movement capabilities
float32 max_speed_ms        # Maximum speed (m/s)
float32 max_altitude_m      # Maximum operational altitude (meters)
float32 min_altitude_m      # Minimum safe altitude (meters)
builtin_interfaces/Duration max_flight_time  # Maximum continuous operation time

# Battery/fuel information
float32 battery_percentage  # Current battery level (0.0 to 100.0)
builtin_interfaces/Duration remaining_flight_time  # Estimated remaining flight time

# Sensor capabilities
string[] available_cameras   # List of camera names
bool has_thermal_camera     # Thermal imaging capability
bool has_night_vision       # Night vision capability
bool has_lidar             # LiDAR capability
bool has_zoom_camera       # Optical zoom capability

# Detection capabilities
string[] detection_types    # Types this vehicle can detect: "person", "vehicle", etc.
float32 detection_range_m   # Maximum detection range (meters)
float32 detection_accuracy  # Detection accuracy rating (0.0 to 1.0)

# Communication capabilities
bool has_radio_relay       # Can act as communication relay
float32 communication_range_m  # Communication range (meters)

# Special capabilities
bool can_carry_payload     # Can carry rescue equipment, supplies, etc.
float32 max_payload_kg     # Maximum payload weight (kg)
bool can_land_anywhere     # Can land in rough terrain
bool weather_resistant     # Can operate in adverse weather

# Current status
bool available_for_mission # Currently available for new missions
string current_mission_id   # ID of current mission (empty if available)
geometry_msgs/Point current_position  # Current location