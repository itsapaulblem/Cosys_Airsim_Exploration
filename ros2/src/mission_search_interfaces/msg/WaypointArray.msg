# Array of waypoints for mission execution
# Used to send planned flight paths to vehicles

# Header and identification
std_msgs/Header header
string vehicle_name         # Target vehicle for these waypoints

# Waypoint sequence
geometry_msgs/Point[] waypoints        # Sequence of 3D waypoints
float32[] waypoint_altitudes          # Altitude for each waypoint (meters)
float32[] waypoint_speeds             # Speed to use at each waypoint (m/s)
builtin_interfaces/Duration[] waypoint_durations  # Time to spend at each waypoint

# Waypoint types and actions
string[] waypoint_actions    # Action at each waypoint: "search", "hover", "photo", "land"
float32[] waypoint_headings  # Desired heading at each waypoint (degrees, 0-360)

# Path execution parameters
bool use_smooth_path        # Use smooth curves between waypoints
float32 path_tolerance      # Acceptable distance from waypoint (meters)
bool allow_altitude_changes # Allow altitude changes during path following

# Safety parameters
geometry_msgs/Point emergency_return_point  # Return here if emergency
float32 minimum_safe_altitude              # Never go below this altitude
geometry_msgs/Polygon[] no_fly_zones       # Areas to avoid during path execution

# Path metadata
float32 total_path_distance    # Total distance of all waypoints
builtin_interfaces/Duration estimated_execution_time  # Time to complete path
string path_generation_method  # How path was generated: "manual", "spiral", "grid"