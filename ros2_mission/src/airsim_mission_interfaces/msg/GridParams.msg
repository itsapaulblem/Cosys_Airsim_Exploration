# Grid search pattern parameters
uint8 PATTERN_BOUSTROPHEDON = 0     # Standard lawnmower pattern
uint8 PATTERN_SPIRAL = 1            # Spiral pattern (future)
uint8 PATTERN_ADAPTIVE = 2          # Adaptive pattern (future)

uint8 pattern_type
float32 lane_spacing        # Distance between parallel flight lines (meters)
float32 waypoint_spacing    # Distance between waypoints on each line (meters)
float32 overlap_percentage  # Sensor overlap percentage (0.0-1.0)
float32 turn_radius         # Minimum turn radius for direction changes (meters)

# Flight direction and orientation
float32 initial_heading     # Initial flight direction (radians, 0=north, pi/2=east)
bool reverse_direction      # Whether to reverse direction on alternate lanes

# Advanced parameters
bool optimize_path          # Enable path optimization algorithms
bool minimize_turns         # Prioritize turn minimization over distance
float32 speed_ms            # Desired flight speed (m/s)
float32 max_acceleration    # Maximum acceleration (m/s²)