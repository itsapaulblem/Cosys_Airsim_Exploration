#!/bin/bash

# 🎯 Ultra-Clean Multi-Node ROS2 System Launcher
# Advanced orchestration for AirSim multi-vehicle simulation

set -e

# Color codes for enhanced output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default configuration
DEFAULT_LAUNCH_MODE="multi"
DEFAULT_AIRSIM_HOST="host.docker.internal"
DEFAULT_AIRSIM_PORT="41451"
DEFAULT_ENABLE_COORDINATION="true"
DEFAULT_RPC_TIMEOUT="10.0"
DEFAULT_LAUNCH_RVIZ="false"

# Parse command line arguments
LAUNCH_MODE="${1:-$DEFAULT_LAUNCH_MODE}"
LAUNCH_RVIZ="${2:-$DEFAULT_LAUNCH_RVIZ}"

show_help() {
    echo -e "${CYAN}🎯 Ultra-Clean Multi-Node ROS2 System Launcher${NC}"
    echo -e "${CYAN}===============================================${NC}"
    echo ""
    echo "Usage: $0 [MODE] [RVIZ]"
    echo ""
    echo -e "${YELLOW}Launch Modes:${NC}"
    echo "  multi    - 🎯 Ultra-clean multi-node (default, recommended)"
    echo "  legacy   - 🔄 Legacy monolithic (backward compatibility)"
    echo "  custom   - 🛠️  Manual control with container shell access"
    echo ""
    echo -e "${YELLOW}RViz Options:${NC}"
    echo "  true     - 📊 Auto-launch RViz2 for visualization"
    echo "  false    - Terminal-only mode (default)"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo "  $0                    # Launch multi-node without RViz"
    echo "  $0 multi true         # Launch multi-node with RViz"
    echo "  $0 legacy false       # Launch legacy mode"
    echo "  $0 custom             # Custom shell access"
    echo ""
    echo -e "${YELLOW}Environment Variables (override via export):${NC}"
    echo "  AIRSIM_HOST_IP        - AirSim server IP ($DEFAULT_AIRSIM_HOST)"
    echo "  AIRSIM_HOST_PORT      - AirSim API port ($DEFAULT_AIRSIM_PORT)"
    echo "  ENABLE_COORDINATION   - Global coordination node ($DEFAULT_ENABLE_COORDINATION)"
    echo "  RPC_TIMEOUT          - Discovery timeout ($DEFAULT_RPC_TIMEOUT)"
    echo ""
}

# Handle help requests
if [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
    show_help
    exit 0
fi

# Validate launch mode
case "$LAUNCH_MODE" in
    "multi"|"legacy"|"custom")
        # Valid modes
        ;;
    *)
        echo -e "${RED}❌ Invalid launch mode: $LAUNCH_MODE${NC}"
        echo -e "${YELLOW}Valid options: multi, legacy, custom${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac

echo -e "${CYAN}🎯 Ultra-Clean Multi-Node ROS2 System Launcher${NC}"
echo -e "${CYAN}===============================================${NC}"
echo ""

# Docker availability check
echo -e "${BLUE}🔍 Checking Docker availability...${NC}"
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}❌ Docker is not running. Please start Docker Desktop.${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Docker is running${NC}"

# Navigate to script directory
cd "$(dirname "$0")"
echo -e "${BLUE}📁 Working directory: $(pwd)${NC}"

# Verify docker-compose.yml exists
if [ ! -f "docker-compose.yml" ]; then
    echo -e "${RED}❌ docker-compose.yml not found!${NC}"
    exit 1
fi

# Set environment variables for docker-compose
export LAUNCH_MODE="$LAUNCH_MODE"
export LAUNCH_RVIZ="$LAUNCH_RVIZ"
export AIRSIM_HOST_IP="${AIRSIM_HOST_IP:-$DEFAULT_AIRSIM_HOST}"
export AIRSIM_HOST_PORT="${AIRSIM_HOST_PORT:-$DEFAULT_AIRSIM_PORT}"
export ENABLE_COORDINATION="${ENABLE_COORDINATION:-$DEFAULT_ENABLE_COORDINATION}"
export RPC_TIMEOUT="${RPC_TIMEOUT:-$DEFAULT_RPC_TIMEOUT}"

echo -e "${YELLOW}🎯 Configuration Summary:${NC}"
echo "   Launch Mode: $LAUNCH_MODE"
echo "   RViz2: $LAUNCH_RVIZ"
echo "   AirSim Server: $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
echo "   Coordination Node: $ENABLE_COORDINATION"
echo "   RPC Timeout: ${RPC_TIMEOUT}s"
echo ""

# Enhanced AirSim connectivity check
echo -e "${BLUE}🔍 Testing AirSim connectivity...${NC}"
if timeout 5 bash -c "echo >/dev/tcp/$AIRSIM_HOST_IP/$AIRSIM_HOST_PORT" 2>/dev/null; then
    echo -e "${GREEN}✓ AirSim API server accessible at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT${NC}"
else
    echo -e "${YELLOW}⚠️  Cannot connect to AirSim at $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT${NC}"
    echo -e "${YELLOW}💡 This is normal if AirSim is not running yet${NC}"
    echo -e "${YELLOW}   Container will attempt to connect after startup${NC}"
fi
echo ""

# Launch mode-specific information
case "$LAUNCH_MODE" in
    "multi")
        echo -e "${GREEN}🚀 Launching Ultra-Clean Multi-Node Architecture...${NC}"
        echo -e "${CYAN}📋 Expected Result:${NC}"
        echo "   • Nodes: /airsim_coordination_node + individual vehicle nodes"
        echo "   • Topics: /Droan1/odom_local_ned, /PX4_Drone2/imu, etc."
        echo "   • Services: /Droan1/takeoff, /airsim_coordination_node/takeoff_all"
        echo "   • TF Structure: map → world_ned → {vehicle_name}_base_link"
        ;;
    "legacy")
        echo -e "${YELLOW}🔄 Launching Legacy Monolithic Architecture...${NC}"
        echo -e "${CYAN}📋 Expected Result:${NC}"
        echo "   • Single node: /airsim_node handles all vehicles"
        echo "   • Traditional topic structure"
        ;;
    "custom")
        echo -e "${BLUE}🛠️  Launching Custom Manual Control Mode...${NC}"
        echo -e "${CYAN}📋 Available Commands in Container:${NC}"
        echo "   • launch_multi  - Start ultra-clean multi-node"
        echo "   • launch_legacy - Start legacy monolithic"
        echo "   • launch_rviz   - Open RViz2 visualization"
        echo "   • build         - Rebuild workspace"
        ;;
esac
echo ""

# Start the container
echo -e "${BLUE}🔧 Starting ROS2 multi-node container...${NC}"
if docker-compose up -d; then
    echo ""
    echo -e "${GREEN}✅ Container started successfully!${NC}"
    
    # Wait for container to be ready
    echo -e "${BLUE}⏳ Waiting for container initialization...${NC}"
    sleep 3
    
    # Check container health
    if docker-compose ps | grep -q "Up"; then
        echo -e "${GREEN}✓ Container is healthy and running${NC}"
        
        echo ""
        echo -e "${CYAN}🖥️  Access Information:${NC}"
        echo "   VNC URL:      localhost:5901"
        echo "   VNC Password: ubuntu"
        echo "   Container:    ros2-multi-node"
        echo ""
        
        echo -e "${CYAN}📊 Container Status:${NC}"
        docker-compose ps
        echo ""
        
        echo -e "${CYAN}💡 Useful Commands:${NC}"
        echo "   View logs:        docker-compose logs -f"
        echo "   View ROS logs:    docker-compose logs -f | grep 'ros2-multi-node'"
        echo "   Enter container:  docker exec -it ros2-multi-node bash"
        echo "   Stop system:      docker-compose stop"
        echo "   Full cleanup:     docker-compose down -v"
        echo ""
        
        if [[ "$LAUNCH_MODE" == "multi" ]]; then
            echo -e "${CYAN}🎯 Multi-Node Verification Commands:${NC}"
            echo "   Check nodes:      docker exec -it ros2-multi-node ros2 node list"
            echo "   Check topics:     docker exec -it ros2-multi-node ros2 topic list"
            echo "   Check TF tree:    docker exec -it ros2-multi-node ros2 run tf2_tools view_frames"
            echo "   Test vehicle:     docker exec -it ros2-multi-node ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff \"{wait_on_last_task: true}\""
            echo ""
        fi
        
        if [[ "$LAUNCH_RVIZ" == "true" ]]; then
            echo -e "${YELLOW}📊 RViz2 Configuration Tips:${NC}"
            echo "   • Fixed Frame: Set to 'world_ned' for proper multi-vehicle view"
            echo "   • Add TF Display: See individual vehicle coordinate frames"
            echo "   • Vehicle Topics: Add /Droan1/odom_local_ned, /PX4_Drone2/imu"
            echo ""
        fi
        
        echo -e "${GREEN}🚁 Ultra-clean multi-node ROS2 system ready!${NC}"
        
        # Show follow-up actions based on mode
        if [[ "$LAUNCH_MODE" == "custom" ]]; then
            echo ""
            echo -e "${BLUE}🛠️  Enter the container to run manual commands:${NC}"
            echo "   docker exec -it ros2-multi-node bash"
        else
            echo ""
            echo -e "${BLUE}📋 Monitor the system:${NC}"
            echo "   docker-compose logs -f"
        fi
        
    else
        echo -e "${RED}⚠️  Container started but may not be fully healthy${NC}"
        echo -e "${YELLOW}📋 Check logs for details: docker-compose logs${NC}"
    fi
    
else
    echo ""
    echo -e "${RED}❌ Failed to start container${NC}"
    echo -e "${YELLOW}📋 Troubleshooting:${NC}"
    echo "   • Check logs: docker-compose logs"
    echo "   • Verify AirSim is running if using multi/legacy modes"
    echo "   • Ensure ports 5901, 7400, 7401 are available"
    echo "   • For Windows: Check Docker Desktop WSL2 integration"
    exit 1
fi