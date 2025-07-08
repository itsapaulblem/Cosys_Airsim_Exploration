# Array of waypoints for mission planning
std_msgs/Header header
Waypoint[] waypoints
uint32 total_waypoints
float32 estimated_flight_time    # Estimated total flight time (seconds)
float32 total_distance          # Total mission distance (meters)
string mission_id               # Unique mission identifier