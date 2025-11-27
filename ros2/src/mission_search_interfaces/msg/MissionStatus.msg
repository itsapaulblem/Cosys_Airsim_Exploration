# Mission status message for ultra-clean multi-node architecture
# Published by coordination node and individual vehicles

# Header
std_msgs/Header header

# Mission identification
string mission_id
string vehicle_name          # Empty for coordination node, vehicle name for individual nodes

# Mission state
uint8 IDLE = 0
uint8 PLANNING = 1
uint8 ACTIVE = 2
uint8 PAUSED = 3
uint8 COMPLETED = 4
uint8 FAILED = 5
uint8 CANCELLED = 6

uint8 status

# Progress information
float32 progress_percentage   # 0.0 to 100.0
float32 area_covered_sq_m    # Area covered so far
uint32 targets_detected      # Number of targets found
uint32 waypoints_completed   # Progress through waypoints

# Current location and activity
geometry_msgs/Point current_position
string current_activity      # Human-readable activity description

# Timing information
builtin_interfaces/Duration estimated_remaining_time
builtin_interfaces/Time mission_start_time

# Error information
bool has_error
string error_message