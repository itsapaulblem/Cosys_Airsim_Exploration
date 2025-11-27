# Search progress feedback message
# Used for real-time progress updates during search operations

# Header and identification
std_msgs/Header header
string mission_id
string vehicle_name

# Progress metrics
float32 progress_percentage      # Overall mission progress (0.0 to 100.0)
float32 area_covered_sq_m       # Area searched so far
float32 total_area_sq_m         # Total area to be searched

# Current search state
geometry_msgs/Point current_position
float32 current_altitude
float32 current_speed
string current_search_pattern   # Active search pattern

# Detection summary
uint32 targets_detected_total   # Total targets found
uint32 targets_detected_recent  # Targets found in last update interval
TargetDetection[] recent_detections  # Recent detection details

# Path information
uint32 waypoints_completed     # Number of waypoints visited
uint32 total_waypoints        # Total planned waypoints
geometry_msgs/Point next_waypoint  # Next planned position

# Performance metrics
float32 search_efficiency      # Coverage rate (sq_m per second)
builtin_interfaces/Duration elapsed_time      # Time since search started
builtin_interfaces/Duration estimated_remaining  # Estimated time to completion

# Status indicators
bool search_active            # Currently actively searching
bool returning_home           # Returning to base/home position
string status_message         # Human-readable status update