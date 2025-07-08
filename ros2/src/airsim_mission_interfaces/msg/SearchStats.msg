# Mission completion statistics
std_msgs/Header header
string mission_id
builtin_interfaces/Time mission_start_time
builtin_interfaces/Time mission_end_time
builtin_interfaces/Duration total_flight_time

# Coverage statistics
float32 area_searched_m2       # Total area searched (square meters)
float32 total_area_m2          # Total planned area (square meters)
float32 coverage_percentage    # Percentage of area covered (0.0-1.0)

# Flight statistics
float32 total_distance_m       # Total distance flown (meters)
float32 average_speed_ms       # Average flight speed (m/s)
uint32 total_waypoints         # Total number of waypoints
uint32 waypoints_completed     # Number of waypoints successfully completed

# Data collection statistics
uint32 photos_captured         # Number of photos captured
uint32 sensor_recordings       # Number of sensor data recordings
float32 data_size_mb          # Total data collected (megabytes)

# Performance metrics
uint32 navigation_errors       # Number of navigation errors encountered
uint32 communication_losses    # Number of communication losses
uint32 safety_violations       # Number of safety parameter violations
float32 battery_consumed       # Battery consumed (percentage)

# Quality metrics
float32 average_gps_accuracy   # Average GPS accuracy (meters)
float32 average_altitude_hold  # Average altitude holding accuracy (meters)
float32 path_following_error   # Average path following error (meters)