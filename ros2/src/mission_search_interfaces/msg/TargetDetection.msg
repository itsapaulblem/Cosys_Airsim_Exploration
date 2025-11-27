# Target detection message for search and rescue missions
# Published when any detection algorithm identifies a potential target

# Header with timestamp and frame
std_msgs/Header header

# Detection identification
string detection_id          # Unique identifier for this detection
string vehicle_name          # Vehicle that made the detection
string camera_name           # Camera/sensor used for detection

# Target information
geometry_msgs/Point world_position     # Target position in world coordinates
geometry_msgs/Point gps_coordinates    # GPS coordinates if available
float32 confidence_score               # Detection confidence (0.0 to 1.0)

# Classification
string target_type           # "person", "vehicle", "object", "unknown"
string target_description    # Human-readable description

# Detection context
float32 detection_altitude   # Vehicle altitude when detected
float32 detection_distance   # Estimated distance to target
float32 detection_bearing    # Bearing from vehicle to target (degrees)

# Image information
sensor_msgs/Image detection_image      # Image containing the detection
geometry_msgs/Point image_pixel_location  # Pixel coordinates in image

# Validation status
bool verified               # Has detection been verified by human/secondary sensor
bool false_positive         # Marked as false positive
string notes               # Additional notes about detection