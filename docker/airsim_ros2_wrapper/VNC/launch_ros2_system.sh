#!/bin/bash

# 🎯 Ultra-Clean Multi-Node ROS2 System Launcher (Container Version)
# Comprehensive launch system for AirSim multi-vehicle simulation

set -e

# Color codes for enhanced output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# System identification
SCRIPT_NAME="Ultra-Clean ROS2 System"
SCRIPT_VERSION="2.0.0"

# Parse command line arguments
COMMAND="${1:-help}"
MODE="${2:-multi}"
EXTRA_ARGS="${3:-}"

show_header() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                  🎯 ${SCRIPT_NAME}                     ║${NC}"
    echo -e "${CYAN}║                    Version ${SCRIPT_VERSION}                           ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}Commands:${NC}"
    echo "  start [MODE]         - Start ROS2 system (multi/legacy/custom)"
    echo "  stop                 - Stop all ROS2 processes"
    echo "  restart [MODE]       - Restart ROS2 system"
    echo "  status               - Show system status"
    echo "  build                - Build workspace"
    echo "  clean                - Clean build artifacts"
    echo "  test                 - Run system tests"
    echo "  monitor              - Monitor system health"
    echo "  demo                 - Run interactive demo"
    echo "  help                 - Show this help"
    echo ""
    echo -e "${YELLOW}Modes:${NC}"
    echo "  multi               - 🎯 Ultra-clean multi-node (recommended)"
    echo "  legacy              - 🔄 Legacy monolithic"
    echo "  custom              - 🛠️  Manual control"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo "  $0 start multi       # Start ultra-clean multi-node"
    echo "  $0 start legacy      # Start legacy mode"
    echo "  $0 demo              # Interactive demo"
    echo "  $0 monitor           # Monitor system"
    echo ""
}

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_debug() {
    if [[ "${DEBUG:-false}" == "true" ]]; then
        echo -e "${PURPLE}[DEBUG]${NC} $1"
    fi
}

# Check system prerequisites
check_system() {
    print_status "Checking system prerequisites..."
    
    # Check ROS2 installation
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found! Please source ROS2 environment."
        return 1
    fi
    
    # Check workspace
    if [[ ! -f "/airsim_ros2_ws/install/setup.bash" ]]; then
        print_warning "Workspace not built. Building now..."
        build_workspace
    fi
    
    # Source workspace
    if [[ -f "/airsim_ros2_ws/install/setup.bash" ]]; then
        source /airsim_ros2_ws/install/setup.bash
        print_debug "Workspace sourced successfully"
    fi
    
    print_success "System prerequisites satisfied"
    return 0
}

# Build workspace
build_workspace() {
    print_status "Building ROS2 workspace..."
    
    cd /airsim_ros2_ws
    
    # Fix permissions
    sudo chown -R $USER:$USER /airsim_ros2_ws/log /airsim_ros2_ws/build /airsim_ros2_ws/install /airsim_ros2_ws/src 2>/dev/null || true
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    # Build interfaces first
    print_status "Building airsim_interfaces..."
    colcon build --packages-select airsim_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    # Build main packages
    print_status "Building airsim_ros_pkgs..."
    colcon build --packages-select airsim_ros_pkgs --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    print_success "Workspace built successfully!"
}

# Clean build artifacts
clean_workspace() {
    print_status "Cleaning workspace..."
    cd /airsim_ros2_ws
    rm -rf build install log
    print_success "Workspace cleaned!"
}

# Start ROS2 system
start_system() {
    local mode="${1:-multi}"
    
    show_header
    print_status "Starting ROS2 system in $mode mode..."
    
    # Check and prepare system
    check_system || return 1
    
    # Stop any existing processes
    stop_system_quiet
    
    case "$mode" in
        "multi")
            print_status "🎯 Launching Ultra-Clean Multi-Node Architecture..."
            print_status "Expected nodes: /airsim_coordination_node + individual vehicle nodes"
            print_status "Expected topics: /Droan1/odom_local_ned, /PX4_Drone2/imu, etc."
            print_status "TF Structure: map → world_ned → {vehicle_name}_base_link"
            echo ""
            
            ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py \
                host_ip:=${AIRSIM_HOST_IP:-host.docker.internal} \
                host_port:=${AIRSIM_HOST_PORT:-41451} \
                enable_coordination:=${ENABLE_COORDINATION:-true} \
                rpc_timeout:=${RPC_TIMEOUT:-10.0}
            ;;
            
        "legacy")
            print_status "🔄 Launching Legacy Monolithic Architecture..."
            print_status "Single node: /airsim_node handles all vehicles"
            echo ""
            
            ros2 launch airsim_ros_pkgs airsim_node.launch.py \
                host_ip:=${AIRSIM_HOST_IP:-host.docker.internal} \
                host_port:=${AIRSIM_HOST_PORT:-41451} \
                enable_api_control:=true \
                output:=screen
            ;;
            
        "custom")
            print_status "🛠️  Custom mode - manual control"
            print_status "Available commands:"
            echo "  launch_multi  - Start ultra-clean multi-node"
            echo "  launch_legacy - Start legacy monolithic" 
            echo "  launch_rviz   - Open RViz2 visualization"
            echo "  show_nodes    - List active nodes"
            echo "  show_topics   - List available topics"
            echo "  test_drone    - Test drone takeoff"
            echo ""
            exec /bin/bash
            ;;
            
        *)
            print_error "Invalid mode: $mode"
            print_status "Valid modes: multi, legacy, custom"
            return 1
            ;;
    esac
}

# Stop ROS2 system
stop_system() {
    print_status "Stopping ROS2 system..."
    stop_system_quiet
    print_success "ROS2 system stopped!"
}

stop_system_quiet() {
    # Kill ROS2 launch processes
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "airsim_node" 2>/dev/null || true
    pkill -f "coordination_node" 2>/dev/null || true
    pkill -f "simple_multirotor_node" 2>/dev/null || true
    pkill -f "multirotor_node" 2>/dev/null || true
    
    # Wait a moment for graceful shutdown
    sleep 2
}

# Restart system
restart_system() {
    local mode="${1:-multi}"
    print_status "Restarting ROS2 system..."
    stop_system_quiet
    sleep 1
    start_system "$mode"
}

# Show system status
show_status() {
    show_header
    print_status "System Status Report"
    echo ""
    
    # ROS2 environment
    echo -e "${CYAN}🔧 ROS2 Environment:${NC}"
    echo "  ROS_DISTRO: ${ROS_DISTRO:-not set}"
    echo "  ROS_VERSION: ${ROS_VERSION:-not set}"
    echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
    echo ""
    
    # Workspace status
    echo -e "${CYAN}📁 Workspace Status:${NC}"
    if [[ -f "/airsim_ros2_ws/install/setup.bash" ]]; then
        echo "  Status: ✅ Built and ready"
    else
        echo "  Status: ❌ Not built"
    fi
    echo "  Path: /airsim_ros2_ws"
    echo ""
    
    # Active nodes
    echo -e "${CYAN}🔍 Active ROS2 Nodes:${NC}"
    if command -v ros2 &> /dev/null; then
        local nodes=$(ros2 node list 2>/dev/null || echo "  No nodes active or ROS2 not running")
        if [[ "$nodes" == *"No nodes active"* ]] || [[ -z "$nodes" ]]; then
            echo "  ❌ No ROS2 nodes active"
        else
            echo "$nodes" | sed 's/^/  ✅ /'
        fi
    else
        echo "  ❌ ROS2 not available"
    fi
    echo ""
    
    # System resources
    echo -e "${CYAN}💾 System Resources:${NC}"
    echo "  Memory: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
    echo "  Disk: $(df -h /airsim_ros2_ws | awk 'NR==2 {print $3 "/" $2 " (" $5 " used)"}')"
    echo ""
}

# Run system tests
run_tests() {
    show_header
    print_status "Running system tests..."
    
    check_system || return 1
    
    # Test 1: Node discovery
    print_status "Test 1: ROS2 node discovery..."
    if ros2 node list &>/dev/null; then
        print_success "✅ ROS2 node system operational"
    else
        print_error "❌ ROS2 node system not responding"
        return 1
    fi
    
    # Test 2: Topic listing
    print_status "Test 2: Topic discovery..."
    if ros2 topic list &>/dev/null; then
        print_success "✅ ROS2 topic system operational"
    else
        print_error "❌ ROS2 topic system not responding"
        return 1
    fi
    
    # Test 3: Service discovery
    print_status "Test 3: Service discovery..."
    if ros2 service list &>/dev/null; then
        print_success "✅ ROS2 service system operational"
    else
        print_error "❌ ROS2 service system not responding"
        return 1
    fi
    
    print_success "All tests passed! 🎉"
}

# Monitor system health
monitor_system() {
    show_header
    print_status "Starting system health monitor..."
    print_status "Press Ctrl+C to stop monitoring"
    echo ""
    
    while true; do
        clear
        echo -e "${CYAN}📊 Live System Monitor - $(date)${NC}"
        echo ""
        
        # Node count
        local node_count=$(ros2 node list 2>/dev/null | wc -l)
        echo -e "${YELLOW}Nodes:${NC} $node_count active"
        
        # Topic count  
        local topic_count=$(ros2 topic list 2>/dev/null | wc -l)
        echo -e "${YELLOW}Topics:${NC} $topic_count available"
        
        # Memory usage
        local memory=$(free | awk '/^Mem:/ {printf "%.1f%%", ($3/$2)*100}')
        echo -e "${YELLOW}Memory:${NC} $memory used"
        
        # Recent nodes
        echo ""
        echo -e "${CYAN}Recent Nodes:${NC}"
        ros2 node list 2>/dev/null | tail -5 | sed 's/^/  /'
        
        sleep 5
    done
}

# Interactive demo
run_demo() {
    show_header
    print_status "Starting interactive demo..."
    
    check_system || return 1
    
    echo -e "${CYAN}🎮 Interactive Demo Menu${NC}"
    echo ""
    echo "1. Start ultra-clean multi-node system"
    echo "2. List active nodes and topics"
    echo "3. Launch RViz2 visualization"
    echo "4. Test drone takeoff"
    echo "5. Monitor system health"
    echo "6. Exit demo"
    echo ""
    
    while true; do
        echo -n -e "${YELLOW}Choose option (1-6): ${NC}"
        read -r choice
        
        case "$choice" in
            1)
                print_status "Starting multi-node system in background..."
                start_system multi &
                sleep 5
                ;;
            2)
                echo -e "${CYAN}Active Nodes:${NC}"
                ros2 node list 2>/dev/null || echo "No nodes active"
                echo ""
                echo -e "${CYAN}Available Topics:${NC}"
                ros2 topic list 2>/dev/null | head -10 || echo "No topics available"
                echo ""
                ;;
            3)
                print_status "Launching RViz2..."
                ros2 launch airsim_ros_pkgs rviz.launch.py &
                ;;
            4)
                print_status "Testing drone takeoff..."
                ros2 service call /Droan1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}" 2>/dev/null || \
                echo "Takeoff service not available - ensure AirSim is running with vehicles"
                ;;
            5)
                monitor_system
                ;;
            6)
                print_status "Exiting demo..."
                break
                ;;
            *)
                print_warning "Invalid choice. Please select 1-6."
                ;;
        esac
        echo ""
    done
}

# Main execution
main() {
    case "$COMMAND" in
        "start")
            start_system "$MODE"
            ;;
        "stop")
            stop_system
            ;;
        "restart")
            restart_system "$MODE"
            ;;
        "status")
            show_status
            ;;
        "build")
            build_workspace
            ;;
        "clean")
            clean_workspace
            ;;
        "test")
            run_tests
            ;;
        "monitor")
            monitor_system
            ;;
        "demo")
            run_demo
            ;;
        "help"|"")
            show_help
            ;;
        *)
            print_error "Unknown command: $COMMAND"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Set up signal handlers
trap 'print_warning "Interrupted by user"; stop_system_quiet; exit 1' INT TERM

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Run main function
main "$@"