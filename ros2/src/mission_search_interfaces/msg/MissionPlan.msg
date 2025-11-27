# Mission plan message
# Contains complete mission plan with vehicle assignments and zones

# Mission identification
string mission_id
string mission_name
string mission_type         # "search_and_rescue", "surveillance", "inspection"

# Mission parameters
geometry_msgs/Polygon mission_area      # Overall mission boundary
float32 search_altitude                 # Default search altitude
builtin_interfaces/Duration max_mission_time  # Maximum mission duration

# Vehicle assignments
string[] assigned_vehicles              # List of vehicle names
SearchZone[] search_zones              # Individual search zones per vehicle

# Mission coordination
geometry_msgs/Point rally_point        # Coordination/staging point
geometry_msgs/Point emergency_landing  # Emergency landing location
string communication_frequency         # Radio frequency for coordination

# Mission priorities and rules
uint8 priority_level       # 1=highest, 5=lowest
bool allow_zone_sharing    # Multiple vehicles can search same zone
bool require_confirmation  # Detections must be confirmed by second vehicle
float32 minimum_detection_confidence  # Minimum confidence for valid detection

# Weather and environmental constraints
float32 max_wind_speed     # Maximum allowable wind speed (m/s)
float32 min_visibility     # Minimum visibility requirements (meters)
bool daylight_only         # Restrict to daylight hours only

# Success criteria
uint32 target_detection_goal     # Number of targets expected to find
float32 coverage_requirement     # Percentage of area that must be covered
builtin_interfaces/Duration timeout_abort  # Auto-abort if takes longer than this