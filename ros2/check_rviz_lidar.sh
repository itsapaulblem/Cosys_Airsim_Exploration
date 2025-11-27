#!/bin/bash
# Quick diagnostic script for RViz2 LiDAR visualization issues

echo "======================================"
echo "RViz2 LiDAR Diagnostic"
echo "======================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Vehicle and sensor names
VEHICLE="Drone_1"
SENSOR="LidarSensor1"
TOPIC="/${VEHICLE}/${SENSOR}/points"

echo -e "\nChecking for vehicle: ${GREEN}${VEHICLE}${NC}"
echo -e "Checking for sensor: ${GREEN}${SENSOR}${NC}"
echo -e "Topic: ${GREEN}${TOPIC}${NC}\n"

# 1. Check if topic exists
echo "1. Checking if topic exists..."
if ros2 topic list | grep -q "${TOPIC}"; then
    echo -e "   ${GREEN}✓${NC} Topic found"
    
    # Get topic info
    echo -e "\n2. Topic info:"
    ros2 topic info ${TOPIC}
    
    # Check publishing rate
    echo -e "\n3. Publishing rate:"
    timeout 2 ros2 topic hz ${TOPIC} 2>/dev/null | head -3
    
else
    echo -e "   ${RED}✗${NC} Topic not found!"
    echo "   Available topics with 'Drone' in name:"
    ros2 topic list | grep -i drone
fi

# 4. Check TF frames
echo -e "\n4. Checking TF frames..."

# Check if world_ned exists
if timeout 1 ros2 run tf2_ros tf2_echo world_ned ${VEHICLE}_base_link 2>/dev/null | head -1 | grep -q "At time"; then
    echo -e "   ${GREEN}✓${NC} Transform world_ned → ${VEHICLE}_base_link exists"
    FIXED_FRAME="world_ned"
else
    echo -e "   ${YELLOW}⚠${NC} No transform from world_ned to ${VEHICLE}_base_link"
    
    # Try other frames
    if timeout 1 ros2 run tf2_ros tf2_echo ${VEHICLE}/odom_local_ned ${VEHICLE}_base_link 2>/dev/null | head -1 | grep -q "At time"; then
        echo -e "   ${GREEN}✓${NC} Transform ${VEHICLE}/odom_local_ned → ${VEHICLE}_base_link exists"
        FIXED_FRAME="${VEHICLE}/odom_local_ned"
    else
        echo -e "   ${YELLOW}⚠${NC} No transform from ${VEHICLE}/odom_local_ned"
        FIXED_FRAME="${VEHICLE}/${SENSOR}"
        echo -e "   ${YELLOW}→${NC} Try using sensor frame directly: ${FIXED_FRAME}"
    fi
fi

# 5. Check for sensor transform
echo -e "\n5. Checking sensor transform..."
if ros2 topic echo /tf_static --once 2>/dev/null | grep -q "${SENSOR}"; then
    echo -e "   ${GREEN}✓${NC} Static transform for ${SENSOR} found"
else
    echo -e "   ${YELLOW}⚠${NC} No static transform for ${SENSOR}"
fi

# 6. Sample the point cloud data
echo -e "\n6. Sampling point cloud data..."
SAMPLE=$(timeout 1 ros2 topic echo ${TOPIC} --once 2>/dev/null)

if [ ! -z "$SAMPLE" ]; then
    WIDTH=$(echo "$SAMPLE" | grep "width:" | awk '{print $2}')
    echo -e "   ${GREEN}✓${NC} Point cloud width: ${WIDTH} points"
    
    # Check for valid data (not all zeros)
    if echo "$SAMPLE" | grep -q "data:" && echo "$SAMPLE" | grep -A 50 "data:" | grep -q -v "^- 0$"; then
        echo -e "   ${GREEN}✓${NC} Point cloud contains non-zero data"
    else
        echo -e "   ${YELLOW}⚠${NC} Point cloud might be all zeros"
    fi
else
    echo -e "   ${RED}✗${NC} Could not sample point cloud"
fi

# 7. Check running nodes
echo -e "\n7. Checking ROS2 nodes..."
NODES=$(ros2 node list 2>/dev/null)

if echo "$NODES" | grep -q "coordination"; then
    echo -e "   ${GREEN}✓${NC} Coordination node is running"
else
    echo -e "   ${YELLOW}⚠${NC} No coordination node found"
    echo "   Consider running: ros2 run airsim_ros_pkgs airsim_coordination_node"
fi

if echo "$NODES" | grep -q "${VEHICLE}"; then
    echo -e "   ${GREEN}✓${NC} ${VEHICLE} node is running"
else
    echo -e "   ${RED}✗${NC} No ${VEHICLE} node found"
fi

# 8. Generate RViz2 launch command
echo -e "\n${GREEN}======================================"
echo "Recommended RViz2 Settings"
echo -e "======================================${NC}"

echo -e "\n${YELLOW}Fixed Frame:${NC} ${FIXED_FRAME}"
echo -e "${YELLOW}Topic:${NC} ${TOPIC}"

echo -e "\n${YELLOW}Quick test command:${NC}"
echo "rviz2 -d <(cat <<EOF"
echo "Panels:"
echo "  - Class: rviz_common/Displays"
echo "    Name: Displays"
echo "Visualization Manager:"
echo "  Displays:"
echo "    - Class: rviz_default_plugins/Grid"
echo "      Name: Grid"
echo "      Value: true"
echo "    - Class: rviz_default_plugins/TF"
echo "      Name: TF" 
echo "      Value: true"
echo "      Show Names: true"
echo "    - Class: rviz_default_plugins/PointCloud2"
echo "      Name: ${VEHICLE} LiDAR"
echo "      Topic: ${TOPIC}"
echo "      Reliability Policy: Best Effort"
echo "      Style: Points"
echo "      Size (m): 0.05"
echo "      Color Transformer: FlatColor"
echo "      Color: [255, 255, 0]"
echo "  Global Options:"
echo "    Fixed Frame: ${FIXED_FRAME}"
echo "    Frame Rate: 30"
echo "EOF"
echo ")"

echo -e "\n${YELLOW}Manual RViz2 settings:${NC}"
echo "1. Set Fixed Frame to: ${FIXED_FRAME}"
echo "2. Add PointCloud2 display"
echo "3. Set Topic to: ${TOPIC}"
echo "4. Set Reliability Policy to: Best Effort"
echo "5. Set Size to: 0.05"
echo "6. Set Style to: Points or Flat Squares"

# 9. Final diagnosis
echo -e "\n${GREEN}======================================"
echo "Diagnosis Summary"
echo -e "======================================${NC}"

if [ ! -z "$WIDTH" ] && [ "$WIDTH" -gt 0 ]; then
    echo -e "${GREEN}✓${NC} Point cloud is being published (${WIDTH} points)"
else
    echo -e "${RED}✗${NC} No point cloud data detected"
fi

if [ ! -z "$FIXED_FRAME" ]; then
    echo -e "${GREEN}✓${NC} TF transforms available (use Fixed Frame: ${FIXED_FRAME})"
else
    echo -e "${RED}✗${NC} TF transform issues detected"
fi

echo -e "\n${YELLOW}Most likely issue:${NC}"
if [ ! -z "$WIDTH" ] && [ "$WIDTH" -gt 0 ] && [ ! -z "$FIXED_FRAME" ]; then
    echo "RViz2 Fixed Frame is not set correctly."
    echo "Solution: Set Fixed Frame to '${FIXED_FRAME}' in RViz2"
else
    echo "Missing TF transforms or point cloud data."
    echo "Solution: Ensure all ROS2 nodes are running correctly"
fi