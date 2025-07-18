# Current waypoint being executed
std_msgs/Header header
Waypoint current_waypoint
uint32 waypoint_index          # Current waypoint index in mission
uint32 total_waypoints         # Total number of waypoints
float32 distance_to_waypoint   # Distance to current waypoint (meters)
float32 time_to_waypoint       # Estimated time to reach waypoint (seconds)
float32 progress_percentage    # Progress to current waypoint (0.0-1.0)