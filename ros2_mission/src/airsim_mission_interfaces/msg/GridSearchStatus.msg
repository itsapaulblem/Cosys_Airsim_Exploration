# Real-time grid search status
std_msgs/Header header
string mission_id
MissionProgress progress
CurrentWaypoint current_waypoint
SearchArea search_area
GridParams grid_params
SafetyParams safety_params

# Real-time metrics
geometry_msgs/Point current_position
geometry_msgs/Twist current_velocity
float32 current_battery_level
float32 signal_strength
float32 gps_accuracy

# Environmental conditions
float32 wind_speed_ms
float32 wind_direction_rad
float32 temperature_celsius
float32 visibility_m

# System health
bool motors_healthy
bool navigation_healthy
bool communication_healthy
bool sensors_healthy
string[] active_warnings
string[] active_errors