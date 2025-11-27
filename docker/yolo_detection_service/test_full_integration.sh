#!/bin/bash
# Complete Integration Test for YOLOv10 Detection → Feedback Loop
# Tests the entire pipeline from camera images to drone velocity commands

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  YOLOv10 Complete Integration Test${NC}"
echo -e "${GREEN}========================================${NC}\n"

# Change to docker directory
cd "$(dirname "$0")/.."

# Test counter
PASS=0
FAIL=0

# Function to print test result
test_result() {
    local test_name=$1
    local result=$2

    if [ "$result" -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $test_name"
        ((PASS++))
    else
        echo -e "${RED}✗${NC} $test_name"
        ((FAIL++))
    fi
}

# Test 1: Check all containers running
echo -e "${BLUE}[1/8] Checking Docker containers...${NC}"
docker compose -f docker-compose-master.yml ps --format json > /tmp/docker_status.json 2>&1
if docker compose -f docker-compose-master.yml ps | grep -q "yolov10-detection.*Up"; then
    test_result "YOLOv10 detection service running" 0
else
    test_result "YOLOv10 detection service running" 1
fi

if docker compose -f docker-compose-master.yml ps | grep -q "ros2-x11-node.*Up"; then
    test_result "ROS2 node running" 0
else
    test_result "ROS2 node running" 1
fi

if docker compose -f docker-compose-master.yml ps | grep -q "airsim-container.*Up"; then
    test_result "AirSim container running" 0
else
    test_result "AirSim container running" 1
fi

# Test 2: Verify camera topics exist
echo -e "\n${BLUE}[2/8] Verifying camera topics...${NC}"
CAMERA_TOPICS=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic list 2>/dev/null | grep Camera_.*Scene/image" 2>/dev/null)

if echo "$CAMERA_TOPICS" | grep -q "Camera_0_Scene/image"; then
    test_result "Camera_0_Scene/image topic exists" 0
else
    test_result "Camera_0_Scene/image topic exists" 1
fi

if echo "$CAMERA_TOPICS" | grep -q "Camera_1_Scene/image"; then
    test_result "Camera_1_Scene/image topic exists" 0
else
    test_result "Camera_1_Scene/image topic exists" 1
fi

# Test 3: Check YOLOv10 subscriptions
echo -e "\n${BLUE}[3/8] Checking YOLOv10 subscriptions...${NC}"
TOPIC_INFO=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic info /Drone1/Camera_0_Scene/image 2>/dev/null" 2>/dev/null)

if echo "$TOPIC_INFO" | grep -q "Subscription count:"; then
    SUB_COUNT=$(echo "$TOPIC_INFO" | grep "Subscription count:" | awk '{print $3}')
    if [ "$SUB_COUNT" -ge 1 ]; then
        test_result "YOLOv10 subscribed to Camera_0" 0
    else
        test_result "YOLOv10 subscribed to Camera_0" 1
    fi
else
    test_result "YOLOv10 subscribed to Camera_0" 1
fi

# Test 4: Check detection topics published
echo -e "\n${BLUE}[4/8] Checking detection publication topics...${NC}"
DETECTION_TOPICS=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic list 2>/dev/null | grep /detections/" 2>/dev/null)

if echo "$DETECTION_TOPICS" | grep -q "/detections/Camera_0_Scene"; then
    test_result "Detection topic /detections/Camera_0_Scene exists" 0
else
    test_result "Detection topic /detections/Camera_0_Scene exists" 1
fi

# Test 5: Monitor detection flow (5 second test)
echo -e "\n${BLUE}[5/8] Testing detection message flow...${NC}"
echo -e "${YELLOW}  (Waiting 5 seconds for detection messages...)${NC}"
DETECTION_MSG=$(timeout 5 docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic echo /detections/Camera_0_Scene --once 2>/dev/null" 2>/dev/null || echo "timeout")

if echo "$DETECTION_MSG" | grep -q "camera_id"; then
    test_result "Detection messages being published" 0
    echo -e "${YELLOW}  Sample detection: $(echo "$DETECTION_MSG" | head -5)${NC}"
elif [ "$DETECTION_MSG" == "timeout" ]; then
    echo -e "${YELLOW}  ⚠ No detections in 5 seconds (camera may not be publishing or no objects in view)${NC}"
    test_result "Detection messages being published" 1
else
    test_result "Detection messages being published" 1
fi

# Test 6: Check velocity command topics
echo -e "\n${BLUE}[6/8] Checking feedback loop (velocity commands)...${NC}"
VEL_TOPIC_INFO=$(docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && ros2 topic info /Drone1/vel_cmd_body_frame 2>/dev/null" 2>/dev/null)

if echo "$VEL_TOPIC_INFO" | grep -q "Publisher count:"; then
    PUB_COUNT=$(echo "$VEL_TOPIC_INFO" | grep "Publisher count:" | awk '{print $3}')
    if [ "$PUB_COUNT" -ge 1 ]; then
        test_result "Velocity command publisher active" 0
    else
        test_result "Velocity command publisher active" 1
    fi
else
    test_result "Velocity command publisher active" 1
fi

# Test 7: Check YOLOv10 service logs
echo -e "\n${BLUE}[7/8] Checking YOLOv10 service status...${NC}"
SERVICE_LOGS=$(docker logs yolov10-detection --tail 50 2>&1)

if echo "$SERVICE_LOGS" | grep -q "Detection Service ready"; then
    test_result "YOLOv10 service initialized successfully" 0
else
    test_result "YOLOv10 service initialized successfully" 1
fi

if echo "$SERVICE_LOGS" | grep -q "DeepSORT tracker initialized"; then
    test_result "DeepSORT tracker initialized" 0
else
    test_result "DeepSORT tracker initialized" 1
fi

# Test 8: Complete pipeline verification
echo -e "\n${BLUE}[8/8] Complete pipeline summary...${NC}"

# Count active topics
CAMERA_COUNT=$(echo "$CAMERA_TOPICS" | grep -c Camera_.*Scene/image)
DETECTION_COUNT=$(echo "$DETECTION_TOPICS" | grep -c /detections/)

echo -e "  ${GREEN}Active Components:${NC}"
echo -e "    • Containers running: $(docker compose -f docker-compose-master.yml ps | grep -c Up)"
echo -e "    • Camera topics: $CAMERA_COUNT"
echo -e "    • Detection topics: $DETECTION_COUNT"
echo -e "    • Drone nodes: $(docker exec ros2-x11-node bash -c 'source /airsim_ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | grep -c Drone' 2>/dev/null)"

# Final summary
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  Integration Test Summary${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  ${GREEN}Passed:${NC} $PASS tests"
echo -e "  ${RED}Failed:${NC} $FAIL tests"

if [ $FAIL -eq 0 ]; then
    echo -e "\n${GREEN}✓ All tests passed! Integration is working correctly.${NC}\n"

    echo -e "${BLUE}Complete Data Flow:${NC}"
    echo -e "  ${GREEN}[AirSim]${NC} → /Drone1/Camera_0_Scene/image"
    echo -e "      ↓"
    echo -e "  ${GREEN}[YOLOv10 Service]${NC} → /detections/Camera_0_Scene"
    echo -e "      ↓"
    echo -e "  ${GREEN}[motion_detection_node]${NC} → /Drone1/vel_cmd_body_frame"
    echo -e "      ↓"
    echo -e "  ${GREEN}[Drone Movement]${NC}\n"

    exit 0
else
    echo -e "\n${YELLOW}⚠ Some tests failed. Check the output above.${NC}\n"

    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo -e "  1. Ensure AirSim is running and publishing camera images"
    echo -e "  2. Check YOLOv10 service logs: ${BLUE}docker logs yolov10-detection${NC}"
    echo -e "  3. Verify ROS2 topic connections: ${BLUE}ros2 topic list${NC}"
    echo -e "  4. Check motion_detection_node logs for errors\n"

    exit 1
fi
