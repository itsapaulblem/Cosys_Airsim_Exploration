# Search zone definition for area assignment
# Used by coordination node to assign search areas to individual vehicles

# Zone identification
string zone_id
string assigned_vehicle      # Vehicle name assigned to this zone

# Geographic boundary
geometry_msgs/Polygon boundary_polygon  # Search area boundary in world coordinates
float32 min_altitude        # Minimum search altitude (meters)
float32 max_altitude        # Maximum search altitude (meters)

# Search parameters
string search_pattern       # "spiral", "lawnmower", "random", "waypoint"
float32 search_spacing      # Distance between search paths (meters)  
float32 search_speed        # Recommended search speed (m/s)

# Priority and timing
uint8 priority              # 1=highest, 5=lowest priority
builtin_interfaces/Duration max_search_time  # Maximum time to spend in zone
bool exclusive_assignment   # True if only one vehicle should search this zone

# Zone characteristics
float32 area_sq_m          # Total area of search zone
uint8 terrain_difficulty   # 1=easy, 5=difficult terrain
bool requires_special_sensors  # Requires thermal, night vision, etc.

# Progress tracking
float32 coverage_percentage   # Percentage of zone already covered
uint32 previous_detections   # Number of targets found in this zone previously