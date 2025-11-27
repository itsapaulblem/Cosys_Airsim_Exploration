#!/bin/bash
# YOLOv10 Detection Service Entrypoint
# Microservices Architecture - Isolated ML Container

set -e

# Color output for logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  YOLOv10 Detection Service Starting${NC}"
echo -e "${GREEN}========================================${NC}"

# Source ROS2 environment
echo -e "${YELLOW}[1/5] Sourcing ROS2 Humble environment...${NC}"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
    echo -e "${RED}✗ ROS2 setup.bash not found!${NC}"
    exit 1
fi

# Source custom airsim_interfaces workspace (shared via named volume ros2_x11_install)
echo -e "${YELLOW}[2/6] Waiting for airsim_interfaces workspace to be ready...${NC}"

# Wait for the install directory to be populated by ros2-x11-node
MAX_WAIT=120  # Maximum wait time in seconds
WAIT_COUNT=0
WAIT_INTERVAL=2

while [ ! -f /airsim_ros2_ws/install/setup.bash ] && [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    echo -e "${YELLOW}  Waiting for ros2-x11-node to build airsim_interfaces... ($WAIT_COUNT/${MAX_WAIT}s)${NC}"
    sleep $WAIT_INTERVAL
    WAIT_COUNT=$((WAIT_COUNT + WAIT_INTERVAL))
done

if [ -f /airsim_ros2_ws/install/setup.bash ]; then
    source /airsim_ros2_ws/install/setup.bash

    # Verify airsim_interfaces is actually importable
    if python3 -c "from airsim_interfaces.msg import ObjectDetection, ObjectDetectionArray" 2>/dev/null; then
        echo -e "${GREEN}✓ Custom interfaces sourced and verified (ObjectDetection, ObjectDetectionArray)${NC}"
    else
        echo -e "${YELLOW}⚠ WARNING: airsim_interfaces sourced but not importable yet${NC}"
        echo -e "${YELLOW}  Waiting additional 10 seconds for package to be fully ready...${NC}"
        sleep 10

        # Try sourcing again
        source /airsim_ros2_ws/install/setup.bash

        if python3 -c "from airsim_interfaces.msg import ObjectDetection, ObjectDetectionArray" 2>/dev/null; then
            echo -e "${GREEN}✓ Custom interfaces now importable${NC}"
        else
            echo -e "${RED}✗ ERROR: airsim_interfaces still not importable after waiting${NC}"
            echo -e "${RED}  Detection service will fail without custom message types!${NC}"
            echo -e "${YELLOW}  Troubleshooting:${NC}"
            echo -e "${YELLOW}    1. Check ros2-x11-node logs: docker logs ros2-x11-node${NC}"
            echo -e "${YELLOW}    2. Verify AUTO_BUILD=true is set for ros2-x11-node${NC}"
            echo -e "${YELLOW}    3. Manually build: docker exec ros2-x11-node colcon build --packages-select airsim_interfaces${NC}"
        fi
    fi
else
    echo -e "${RED}✗ ERROR: airsim_interfaces not found at /airsim_ros2_ws/install after ${MAX_WAIT}s${NC}"
    echo -e "${RED}  Detection service will fail without custom message types!${NC}"
    echo -e "${YELLOW}  Troubleshooting:${NC}"
    echo -e "${YELLOW}    1. Ensure ros2-x11-node container is running${NC}"
    echo -e "${YELLOW}    2. Check ros2-x11-node logs for build errors${NC}"
    echo -e "${YELLOW}    3. Verify ros2_x11_install volume is properly mounted${NC}"
fi

# Add YOLOv10 directory to PYTHONPATH for objectTracking.py imports
echo -e "${YELLOW}[3/6] Configuring PYTHONPATH for YOLOv10...${NC}"
export PYTHONPATH="/detection_ws/yolov10_tracking/yolov10:${PYTHONPATH}"
echo -e "${GREEN}✓ PYTHONPATH=${PYTHONPATH}${NC}"

# Set ROS2 environment variables for inter-container communication
echo -e "${YELLOW}[4/6] Configuring ROS2 DDS discovery...${NC}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-}
echo -e "${GREEN}✓ ROS_DOMAIN_ID=${ROS_DOMAIN_ID}${NC}"
echo -e "${GREEN}✓ ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}${NC}"

# Verify YOLOv10 installation
echo -e "${YELLOW}[5/6] Verifying YOLOv10 installation...${NC}"
if python3 -c "from ultralytics import YOLOv10; import rclpy" 2>/dev/null; then
    echo -e "${GREEN}✓ YOLOv10 + ROS2 imports successful${NC}"
else
    echo -e "${RED}✗ YOLOv10 import failed! Check installation.${NC}"
    exit 1
fi

# Check for DeepSORT checkpoint
echo -e "${YELLOW}[5.5/6] Checking DeepSORT checkpoint...${NC}"
if [ -d "/tmp/deep_sort_pytorch" ]; then
    echo -e "${GREEN}✓ DeepSORT checkpoint found at /tmp/deep_sort_pytorch${NC}"
elif [ -d "/detection_ws/yolov10_tracking/yolov10/deep_sort_pytorch" ]; then
    echo -e "${GREEN}✓ DeepSORT checkpoint found at /detection_ws/yolov10_tracking/yolov10/deep_sort_pytorch${NC}"
else
    echo -e "${YELLOW}⚠ WARNING: DeepSORT checkpoint not found${NC}"
    echo -e "${YELLOW}  Tracking may fail. Checkpoint should be at /tmp/deep_sort_pytorch${NC}"
fi

# Display detection service configuration
echo -e "${YELLOW}[6/6] Detection Service Configuration:${NC}"
echo -e "  Workspace: /detection_ws"
echo -e "  YOLOv10 Tracking Repo: /detection_ws/yolov10_tracking"
echo -e "  Model: YOLOv10 (from THU-MIG)"
echo -e "  Tracker: DeepSORT"
echo -e "  ROS2 Topics:"
echo -e "    - Subscribe: /camera/image_raw (sensor_msgs/Image)"
echo -e "    - Publish: /detections (custom ObjectDetectionArray)"
echo -e "${GREEN}✓ Configuration validated${NC}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Starting Detection Service Node${NC}"
echo -e "${GREEN}========================================${NC}"

# Execute the main detection service node
# Use exec to replace shell process (proper signal handling)
exec "$@"
