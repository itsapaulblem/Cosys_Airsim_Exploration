# Individual waypoint definition
uint32 waypoint_id
uint8 waypoint_type

# Waypoint types
uint8 TYPE_NAVIGATION = 0       # Standard navigation waypoint
uint8 TYPE_PHOTO = 1           # Photo capture waypoint
uint8 TYPE_HOVER = 2           # Hovering waypoint
uint8 TYPE_LAND = 3            # Landing waypoint
uint8 TYPE_TAKEOFF = 4         # Takeoff waypoint
uint8 TYPE_RETURN_HOME = 5     # Return to home waypoint

# Position (local NED coordinates)
geometry_msgs/Point position
float32 yaw                    # Desired yaw angle (radians)

# Flight parameters
float32 speed_ms               # Desired speed to this waypoint (m/s)
float32 acceptance_radius      # Waypoint acceptance radius (meters)
float32 hover_time             # Time to hover at waypoint (seconds)

# Data collection parameters
bool capture_photo             # Capture photo at this waypoint
bool record_sensors            # Record sensor data at this waypoint
string data_tag                # Tag for data collection

# Mission flow control
bool wait_for_approval         # Wait for manual approval before proceeding
float32 max_time_at_waypoint   # Maximum time to spend at waypoint (seconds)