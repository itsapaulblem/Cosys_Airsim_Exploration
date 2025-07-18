# Safety parameters for mission execution
float32 max_flight_time         # Maximum mission duration (seconds)
float32 min_battery_percentage  # Minimum battery level to continue (0.0-1.0)
float32 return_home_battery     # Battery level to trigger return home (0.0-1.0)
float32 max_wind_speed          # Maximum allowable wind speed (m/s)
float32 max_distance_from_home  # Maximum distance from takeoff point (meters)

# Geofence parameters
bool enable_geofence            # Enable geofence checking
geometry_msgs/Polygon geofence_polygon  # Geofence boundary (GPS or local)
float32 geofence_buffer         # Safety buffer inside geofence (meters)

# Emergency procedures
bool enable_return_home         # Enable automatic return home on emergency
geometry_msgs/Point home_position      # Home/takeoff position for return
float32 emergency_altitude      # Altitude to climb to in emergency (meters)
float32 failsafe_timeout        # Time to wait for manual override (seconds)

# Communication parameters
float32 max_comm_loss_time      # Maximum acceptable communication loss (seconds)
bool enable_offline_mode        # Allow mission continuation without comms