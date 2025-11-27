# Single YOLOv10 + DeepSORT object detection
# Used by detection service microservice for object tracking

std_msgs/Header header

# Bounding box in image coordinates (pixels)
float32 x           # Top-left x coordinate
float32 y           # Top-left y coordinate
float32 width       # Bounding box width
float32 height      # Bounding box height

# Detection metadata
int32 class_id      # COCO class ID (0-79)
string class_name   # Human-readable class name (e.g., "person", "car")
float32 confidence  # Detection confidence (0.0-1.0)

# Tracking information (from DeepSORT)
int32 track_id      # Unique tracking ID across frames (-1 if not tracked)
