# Mission Event message for tracking all vehicle activities
# Published on /VehicleName/mission/events

# Header with timestamp  
std_msgs/Header header

# Event identification
string event_id                    # Unique identifier for this event
string vehicle_name               # Vehicle that generated the event
uint64 sequence_number            # Incremental sequence for event ordering

# Event classification
string event_type                 # "MOVEMENT", "TAKEOFF", "LANDING", "MODE_CHANGE", "COMMAND_START", "COMMAND_END"
string event_source              # "DIRECT_CLIENT", "ROS2_ACTION", "AUTONOMOUS", "MANUAL", "UNKNOWN"
string event_description         # Human-readable event description

# State information
geometry_msgs/Point previous_position     # Position before event
geometry_msgs/Point current_position      # Position after event  
geometry_msgs/Vector3 previous_velocity   # Velocity before event
geometry_msgs/Vector3 current_velocity    # Velocity after event
float32 previous_altitude                 # Altitude before event
float32 current_altitude                  # Altitude after event

# Movement analysis
float32 distance_moved                    # Distance traveled since last event (meters)
float32 altitude_change                   # Altitude change since last event (meters) 
float32 speed_change                      # Speed change magnitude (m/s)
builtin_interfaces/Duration time_since_last_event  # Time between events

# Mission context (if available)
string active_mission_id                  # Current mission ID if within mission context
string mission_phase                      # "PREPARATION", "EXECUTION", "INVESTIGATION", "RETURNING", "IDLE"
float32 mission_progress_percentage       # Current mission progress (0.0-100.0)

# Detection triggers
bool significant_position_change          # Position moved beyond threshold
bool significant_velocity_change          # Velocity changed beyond threshold
bool significant_altitude_change          # Altitude changed beyond threshold
bool mode_transition                      # Flight mode or state changed
bool command_execution_detected           # New command execution detected

# Additional metadata
string[] tags                             # Custom tags for event categorization
float32 confidence_score                  # Confidence in event detection (0.0-1.0)