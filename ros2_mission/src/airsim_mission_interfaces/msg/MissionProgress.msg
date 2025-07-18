# Overall mission progress information
std_msgs/Header header
string mission_id
uint8 mission_state

# Mission states
uint8 STATE_IDLE = 0
uint8 STATE_PLANNING = 1
uint8 STATE_EXECUTING = 2
uint8 STATE_PAUSED = 3
uint8 STATE_COMPLETED = 4
uint8 STATE_ABORTED = 5
uint8 STATE_EMERGENCY = 6
uint8 STATE_RETURNING_HOME = 7

# Progress metrics
float32 completion_percentage   # Overall mission completion (0.0-1.0)
uint32 waypoints_completed     # Number of waypoints completed
uint32 total_waypoints         # Total number of waypoints
float32 distance_traveled      # Distance traveled so far (meters)
float32 total_distance         # Total planned distance (meters)

# Time metrics
builtin_interfaces/Time mission_start_time
builtin_interfaces/Duration elapsed_time
builtin_interfaces/Duration estimated_remaining_time

# Current position and status
geometry_msgs/Point current_position
geometry_msgs/Vector3 current_velocity
float32 current_battery_percentage
float32 current_altitude

# Error and warning information
string[] warnings              # Current warnings
string[] errors               # Current errors
string status_message         # Human-readable status message