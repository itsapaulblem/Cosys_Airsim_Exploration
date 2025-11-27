# Array of YOLOv10 + DeepSORT object detections
# Published by detection service microservice

std_msgs/Header header

# Camera/sensor identification
string camera_id    # Camera identifier (e.g., "front_center", "camera_0")

# Detection array
ObjectDetection[] detections
