# Search area definition using GPS coordinates or local coordinates
uint8 COORDINATE_TYPE_GPS = 0
uint8 COORDINATE_TYPE_LOCAL = 1

uint8 coordinate_type
string area_name
string description

# GPS coordinates (if coordinate_type == GPS)
float64 north_lat      # North boundary latitude
float64 south_lat      # South boundary latitude  
float64 east_lon       # East boundary longitude
float64 west_lon       # West boundary longitude

# Local NED coordinates (if coordinate_type == LOCAL)
float32 north_m        # North boundary in meters
float32 south_m        # South boundary in meters
float32 east_m         # East boundary in meters
float32 west_m         # West boundary in meters

# Common parameters
float32 min_altitude   # Minimum search altitude (meters)
float32 max_altitude   # Maximum search altitude (meters)
float32 default_altitude # Default search altitude (meters)