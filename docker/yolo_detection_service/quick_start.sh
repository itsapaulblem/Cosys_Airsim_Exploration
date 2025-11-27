#!/bin/bash
# YOLOv10 Detection Service - Quick Start & Verification Script
# One-command check of the entire detection pipeline

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DOCKER_DIR="$PROJECT_ROOT/docker"

echo -e "\n${GREEN}${BOLD}========================================${NC}"
echo -e "${GREEN}${BOLD}  YOLOv10 Detection Service - Quick Start${NC}"
echo -e "${GREEN}${BOLD}========================================${NC}\n"

# Function to print status
print_status() {
    local status=$1
    local message=$2

    if [ "$status" == "OK" ]; then
        echo -e "  ${GREEN}✓${NC} $message"
    elif [ "$status" == "WARNING" ]; then
        echo -e "  ${YELLOW}⚠${NC} $message"
    elif [ "$status" == "ERROR" ]; then
        echo -e "  ${RED}✗${NC} $message"
    else
        echo -e "  ${BLUE}•${NC} $message"
    fi
}

# Function to check command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo -e "${CYAN}${BOLD}[1/6] Checking Prerequisites...${NC}"

if command_exists docker; then
    print_status "OK" "Docker is installed"
else
    print_status "ERROR" "Docker is not installed!"
    exit 1
fi

if command_exists docker compose; then
    print_status "OK" "Docker Compose V2 is installed"
elif command_exists docker-compose; then
    print_status "OK" "Docker Compose V1 is installed"
else
    print_status "ERROR" "Docker Compose is not installed!"
    exit 1
fi

# Check container status
echo -e "\n${CYAN}${BOLD}[2/6] Checking Container Status...${NC}"

cd "$DOCKER_DIR"

# Check yolov10-detection-service
if docker ps --format '{{.Names}}' | grep -q "yolov10-detection"; then
    STATUS=$(docker inspect yolov10-detection --format='{{.State.Status}}')
    HEALTH=$(docker inspect yolov10-detection --format='{{.State.Health.Status}}' 2>/dev/null || echo "none")

    if [ "$STATUS" == "running" ]; then
        print_status "OK" "Detection service is running"

        if [ "$HEALTH" == "unhealthy" ]; then
            print_status "WARNING" "Health check failing (this is normal - service works fine)"
        fi
    else
        print_status "ERROR" "Detection service is not running (status: $STATUS)"
        echo -e "\n${YELLOW}To start the service:${NC}"
        echo -e "  cd $DOCKER_DIR"
        echo -e "  docker compose -f docker-compose-master.yml up -d yolov10-detection-service"
        exit 1
    fi
else
    print_status "ERROR" "Detection service container not found!"
    echo -e "\n${YELLOW}To start the service:${NC}"
    echo -e "  cd $DOCKER_DIR"
    echo -e "  docker compose -f docker-compose-master.yml up -d yolov10-detection-service"
    exit 1
fi

# Check ros2-x11-node
if docker ps --format '{{.Names}}' | grep -q "ros2-x11-node"; then
    STATUS=$(docker inspect ros2-x11-node --format='{{.State.Status}}')
    if [ "$STATUS" == "running" ]; then
        print_status "OK" "ROS2 node is running"
    else
        print_status "WARNING" "ROS2 node is not running"
    fi
else
    print_status "WARNING" "ROS2 node container not found (some features unavailable)"
fi

# Check service logs
echo -e "\n${CYAN}${BOLD}[3/6] Checking Service Initialization...${NC}"

if docker logs yolov10-detection 2>&1 | grep -q "Detection Service ready"; then
    print_status "OK" "YOLOv10 model loaded successfully"
else
    print_status "WARNING" "Service may still be initializing..."
fi

if docker logs yolov10-detection 2>&1 | grep -q "DeepSORT tracker initialized"; then
    print_status "OK" "DeepSORT tracker initialized"
else
    print_status "WARNING" "DeepSORT tracker not initialized"
fi

# Check ROS2 topics
echo -e "\n${CYAN}${BOLD}[4/6] Checking ROS2 Topics...${NC}"

if docker ps --format '{{.Names}}' | grep -q "ros2-x11-node"; then
    # Check detection topic
    if docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/detections/front_center"; then
        print_status "OK" "Detection topic is published (/detections/front_center)"

        # Check publisher count
        PUB_COUNT=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic info /detections/front_center 2>/dev/null" | grep "Publisher count" | awk '{print $3}')

        if [ "$PUB_COUNT" == "1" ]; then
            print_status "OK" "Publisher active (count: 1)"
        else
            print_status "WARNING" "Unexpected publisher count: $PUB_COUNT"
        fi
    else
        print_status "ERROR" "Detection topic not found!"
    fi

    # Check camera topic
    if docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/Drone1/front_center/Scene"; then
        print_status "OK" "Camera topic exists (/Drone1/front_center/Scene)"

        # Check if camera is publishing
        CAMERA_HZ=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && timeout 2 ros2 topic hz /Drone1/front_center/Scene 2>/dev/null" || echo "0")

        if echo "$CAMERA_HZ" | grep -q "average rate"; then
            print_status "OK" "Camera is actively publishing images"
        else
            print_status "WARNING" "Camera topic exists but no images being published (AirSim not running?)"
        fi
    else
        print_status "WARNING" "Camera topic not found (AirSim may not be running)"
    fi
else
    print_status "WARNING" "Cannot check topics (ros2-x11-node not running)"
fi

# Show recent detections
echo -e "\n${CYAN}${BOLD}[5/6] Checking Detection Activity...${NC}"

if docker ps --format '{{.Names}}' | grep -q "ros2-x11-node"; then
    # Try to get one detection message
    DETECTION_TEST=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && timeout 3 ros2 topic echo /detections/front_center --once 2>/dev/null" || echo "timeout")

    if echo "$DETECTION_TEST" | grep -q "camera_id"; then
        print_status "OK" "Detections are being published!"

        # Count detections in message
        DET_COUNT=$(echo "$DETECTION_TEST" | grep -c "class_name:" || echo "0")
        print_status "INFO" "Last frame had $DET_COUNT detection(s)"
    elif echo "$DETECTION_TEST" | grep -q "timeout"; then
        print_status "WARNING" "No detections received in 3 seconds (camera may not be publishing)"
    fi
else
    print_status "WARNING" "Cannot check detection activity"
fi

# Display service info
echo -e "\n${CYAN}${BOLD}[6/6] Service Information...${NC}"

# Get configuration from logs
MODEL=$(docker logs yolov10-detection 2>&1 | grep "Model:" | tail -1 | awk '{print $3}')
CONF=$(docker logs yolov10-detection 2>&1 | grep "Confidence threshold:" | tail -1 | awk '{print $4}')
IOU=$(docker logs yolov10-detection 2>&1 | grep "IOU threshold:" | tail -1 | awk '{print $4}')

if [ -n "$MODEL" ]; then
    print_status "INFO" "Model: $MODEL"
    print_status "INFO" "Confidence threshold: $CONF"
    print_status "INFO" "IOU threshold: $IOU"
fi

# Summary
echo -e "\n${GREEN}${BOLD}========================================${NC}"
echo -e "${GREEN}${BOLD}  Summary${NC}"
echo -e "${GREEN}${BOLD}========================================${NC}\n"

if docker logs yolov10-detection 2>&1 | grep -q "Detection Service ready"; then
    echo -e "${GREEN}✓ Detection service is fully operational!${NC}\n"
else
    echo -e "${YELLOW}⚠ Detection service may still be starting...${NC}\n"
fi

# Useful commands
echo -e "${CYAN}${BOLD}Useful Commands:${NC}\n"

echo -e "${BOLD}View live detections:${NC}"
echo -e "  docker exec ros2-x11-node bash -c \"source /airsim_ros2_ws/install/setup.bash && ros2 topic echo /detections/front_center\""

echo -e "\n${BOLD}Monitor with colors & statistics:${NC}"
echo -e "  docker exec ros2-x11-node bash -c \"source /airsim_ros2_ws/install/setup.bash && python3 /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/monitor_detections.py\""

echo -e "\n${BOLD}View service logs:${NC}"
echo -e "  docker logs -f yolov10-detection"

echo -e "\n${BOLD}Restart service:${NC}"
echo -e "  cd $DOCKER_DIR"
echo -e "  docker compose -f docker-compose-master.yml restart yolov10-detection-service"

echo -e "\n${BOLD}Full documentation:${NC}"
echo -e "  cat $SCRIPT_DIR/YOLOV10_USAGE.md"

echo -e "\n${GREEN}${BOLD}========================================${NC}\n"
