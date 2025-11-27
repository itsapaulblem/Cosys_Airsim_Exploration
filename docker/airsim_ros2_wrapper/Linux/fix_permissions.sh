#!/bin/bash
# Emergency Permission Fix Script for ros2-x11-node container
# Run this inside the container if you're experiencing permission issues

set -e

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         ROS2 X11 Container - Permission Fix Script          ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if running inside container
if [ ! -f /.dockerenv ] && [ ! -f /run/.containerenv ]; then
    echo -e "${RED}[ERROR]${NC} This script should be run inside the container!"
    echo "Usage: docker exec ros2-x11-node /airsim_ros2_ws/fix_permissions.sh"
    exit 1
fi

echo -e "${YELLOW}[1/5]${NC} Checking current user and permissions..."
echo "  Current user: $(whoami) (UID=$(id -u), GID=$(id -g))"
echo "  Groups: $(groups)"
echo ""

echo -e "${YELLOW}[2/5]${NC} Fixing workspace ownership..."
sudo chown -R $(id -u):$(id -g) /airsim_ros2_ws/{log,build,install,src} 2>/dev/null || {
    echo -e "${RED}[ERROR]${NC} Failed to change ownership. Trying alternative method..."
    sudo chown -R $(id -u):$(id -g) /airsim_ros2_ws/
}
echo -e "${GREEN}✅${NC} Workspace ownership fixed"
echo ""

echo -e "${YELLOW}[3/5]${NC} Setting correct permissions..."
sudo chmod -R u+w /airsim_ros2_ws/{log,build,install}
sudo chmod -R u+w /airsim_ros2_ws/src 2>/dev/null || true
echo -e "${GREEN}✅${NC} Permissions updated"
echo ""

echo -e "${YELLOW}[4/5]${NC} Adding user to video group (for GPU access)..."
if groups | grep -q video; then
    echo -e "${GREEN}✅${NC} Already in video group"
else
    sudo usermod -aG video $(whoami)
    echo -e "${YELLOW}⚠${NC}  Added to video group - you must exit and re-enter container for this to take effect"
fi
echo ""

echo -e "${YELLOW}[5/5]${NC} Setting up development environment..."
if [ -f /home/dockeruser/.bashrc ] && [ ! -f ~/.bashrc ]; then
    echo "  Copying development aliases from dockeruser..."
    cp /home/dockeruser/.bashrc ~/.bashrc 2>/dev/null || \
    sudo cp /home/dockeruser/.bashrc ~/.bashrc
    sudo chown $(id -u):$(id -g) ~/.bashrc 2>/dev/null || true
    echo -e "${GREEN}✅${NC} Development aliases configured"
elif [ -f ~/.bashrc ]; then
    echo -e "${GREEN}✅${NC} ~/.bashrc already exists"
else
    echo -e "${YELLOW}⚠${NC}  Could not find source .bashrc"
fi
echo ""

echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    Fix Complete!                             ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo "Next steps:"
echo "  1. ${YELLOW}Exit and re-enter container:${NC}"
echo "     exit"
echo "     docker exec -it ros2-x11-node bash"
echo ""
echo "  2. ${YELLOW}Source development environment:${NC}"
echo "     source ~/.bashrc"
echo ""
echo "  3. ${YELLOW}Build workspace:${NC}"
echo "     colcon build"
echo ""
echo "  4. ${YELLOW}Source workspace:${NC}"
echo "     source install/setup.bash"
echo ""
echo "  5. ${YELLOW}Test RViz2:${NC}"
echo "     rviz2"
echo ""

# Test if colcon can write to log directory
echo -e "${YELLOW}Testing colcon permissions...${NC}"
if touch /airsim_ros2_ws/log/.test_write 2>/dev/null; then
    rm -f /airsim_ros2_ws/log/.test_write
    echo -e "${GREEN}✅${NC} colcon should work now!"
else
    echo -e "${RED}❌${NC} Still cannot write to log directory"
    echo "   You may need to rebuild the container with the updated configuration"
fi
