#!/bin/bash

# Bridge Network PX4 Swarm Management Script
# This script manages Docker containers using bridge networking
# Compatible with Docker Desktop limitations on Windows/WSL2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$PROJECT_DIR/docker-compose-bridge.yml"

# Default container list (9 drones) - Bridge network names
DEFAULT_CONTAINERS=(
    "px4-bridge-drone-1"
    "px4-bridge-drone-2" 
    "px4-bridge-drone-3"
    "px4-bridge-drone-4"
    "px4-bridge-drone-5"
    "px4-bridge-drone-6"
    "px4-bridge-drone-7"
    "px4-bridge-drone-8"
    "px4-bridge-drone-9"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[BRIDGE SWARM]${NC} $1"
}

# Function to test connectivity to AirSim
test_connectivity() {
    local host="$1"
    local port="${2:-41451}"
    local timeout="${3:-3}"
    
    if command -v nc >/dev/null 2>&1; then
        if timeout "$timeout" nc -z "$host" "$port" 2>/dev/null; then
            return 0
        fi
    elif command -v telnet >/dev/null 2>&1; then
        if timeout "$timeout" bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

# Check Bridge Network environment
detect_bridge_network() {
    print_status "Bridge Network Mode - Docker containers will use host.docker.internal"
    
    # Test connectivity to AirSim via host.docker.internal
    print_status "Testing connectivity to host.docker.internal..."
    if test_connectivity "host.docker.internal" 41451; then
        print_status "✅ AirSim API accessible via host.docker.internal:41451"
    else
        print_warning "⚠️  AirSim API not accessible via host.docker.internal:41451"
        print_warning "   Make sure AirSim is running and configured for bridge network access"
    fi
    
    # Also test direct WSL2 IP if in WSL2
    if grep -qi "microsoft" /proc/version 2>/dev/null; then
        WSL2_IP=$(ip route show | grep "src" | awk '{print $NF}' | head -1)
        if [[ -n "$WSL2_IP" ]]; then
            print_status "WSL2 detected - also testing direct IP: $WSL2_IP"
            if test_connectivity "$WSL2_IP" 41451; then
                print_status "✅ AirSim also accessible via WSL2 IP: $WSL2_IP:41451"
            else
                print_warning "⚠️  AirSim not accessible via WSL2 IP: $WSL2_IP:41451"
            fi
        fi
    fi
}

# Function to start containers in delayed mode
start_containers() {
    local num_drones=${1:-9}
    
    print_header "Starting $num_drones containers in bridge network mode..."
    detect_bridge_network
    
    # Select containers to start
    local containers_to_start=()
    for ((i=0; i<num_drones && i<${#DEFAULT_CONTAINERS[@]}; i++)); do
        containers_to_start+=("${DEFAULT_CONTAINERS[$i]}")
    done
    
    print_status "Starting containers: ${containers_to_start[*]}"
    print_status "Using compose file: $COMPOSE_FILE"
    
    # Start the selected containers
    docker-compose -f "$COMPOSE_FILE" up -d "${containers_to_start[@]}"
    
    print_status "Waiting for containers to be ready..."
    sleep 5
    
    # Check status
    echo ""
    print_header "Container Status:"
    for container in "${containers_to_start[@]}"; do
        if docker ps | grep -q "$container"; then
            # Get the container's bridge IP
            CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$container" 2>/dev/null || echo "N/A")
            print_status "✅ $container - RUNNING (IP: $CONTAINER_IP, delayed mode)"
        else
            print_error "❌ $container - FAILED TO START"
        fi
    done
    
    echo ""
    print_header "Port Mappings:"
    local port_base=4561
    for ((i=0; i<num_drones; i++)); do
        local mapped_port=$((port_base + i))
        print_status "Drone $((i+1)): localhost:$mapped_port -> container:$mapped_port"
    done
    
    echo ""
    print_header "Next Steps:"
    echo "• Containers are running in bridge network mode but PX4 SITL is not started yet"
    echo "• Containers use host.docker.internal to reach AirSim on the host"
    echo "• Use '$0 run-px4 <container_name>' to start PX4 in specific container"
    echo "• Use '$0 run-all-px4' to start PX4 in all running containers"
    echo "• Use '$0 status' to check container status"
    echo "• PythonClient can connect to localhost:4561-456X ports"
}

# Function to start PX4 in a specific container
run_px4_in_container() {
    local container_name="$1"
    
    if [[ -z "$container_name" ]]; then
        print_error "Container name required. Usage: $0 run-px4 <container_name>"
        return 1
    fi
    
    print_header "Starting PX4 SITL in bridge container: $container_name"
    
    # Check if container is running
    if ! docker ps | grep -q "$container_name"; then
        print_error "Container '$container_name' is not running"
        return 1
    fi
    
    # Start PX4 SITL in the container using run_airsim_sitl.sh
    print_status "Executing PX4 SITL startup script..."
    docker exec -d "$container_name" bash -c '
        cd /px4_workspace/PX4-Autopilot
        
        # Check if the script is available (should be copied during container startup)
        if [ -f "./Scripts/run_airsim_sitl.sh" ]; then
            echo "🚀 Starting PX4 SITL with instance $PX4_INSTANCE (bridge network)..."
            
            # Set environment variables for this instance
            export PX4_SIM_HOSTNAME=${PX4_SIM_HOSTNAME:-host.docker.internal}
            export PX4_SIM_MODEL=${PX4_SIM_MODEL:-iris}
            
            # Show which hostname is being used for debugging
            echo "🔗 Using PX4_SIM_HOSTNAME: $PX4_SIM_HOSTNAME"
            echo "🌐 Network mode: Bridge (host.docker.internal)"
            
            # Start PX4 SITL using the script
            ./Scripts/run_airsim_sitl.sh $PX4_INSTANCE
        else
            echo "❌ run_airsim_sitl.sh not found in ./Scripts/"
            echo "Container may not have started properly or script mounting failed"
            exit 1
        fi
    '
    
    sleep 2
    print_status "✅ PX4 SITL started in $container_name (bridge network)"
    print_status "Check logs with: $0 logs $container_name"
}

# Function to start PX4 in all running containers
run_px4_in_all_containers() {
    print_header "Starting PX4 SITL in all running bridge containers..."
    
    local running_containers=()
    for container in "${DEFAULT_CONTAINERS[@]}"; do
        if docker ps | grep -q "$container"; then
            running_containers+=("$container")
        fi
    done
    
    if [[ ${#running_containers[@]} -eq 0 ]]; then
        print_error "No bridge swarm containers are currently running"
        return 1
    fi
    
    print_status "Found ${#running_containers[@]} running containers"
    
    for container in "${running_containers[@]}"; do
        print_status "Starting PX4 in $container..."
        run_px4_in_container "$container"
        sleep 2  # Small delay between starts
    done
    
    print_status "✅ PX4 SITL started in all bridge containers"
}

# Function to show container status
show_status() {
    print_header "Bridge Swarm Container Status:"
    
    local running_count=0
    local px4_running_count=0
    
    for container in "${DEFAULT_CONTAINERS[@]}"; do
        if docker ps | grep -q "$container"; then
            running_count=$((running_count + 1))
            
            # Get container IP and port mapping
            CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$container" 2>/dev/null || echo "N/A")
            PORT_MAP=$(docker port "$container" 2>/dev/null | grep "4561\|4562\|4563\|4564\|4565\|4566\|4567\|4568\|4569" | head -1 | cut -d'-' -f1 || echo "N/A")
            
            # Check if PX4 is running in the container
            if docker exec "$container" pgrep -f "px4" > /dev/null 2>&1; then
                print_status "✅ $container - CONTAINER + PX4 RUNNING (IP: $CONTAINER_IP, Port: $PORT_MAP)"
                px4_running_count=$((px4_running_count + 1))
            else
                print_warning "🟡 $container - CONTAINER ONLY (IP: $CONTAINER_IP, Port: $PORT_MAP, PX4 not started)"
            fi
        else
            print_error "❌ $container - NOT RUNNING"
        fi
    done
    
    echo ""
    print_header "Summary:"
    echo "• Running containers: $running_count/${#DEFAULT_CONTAINERS[@]}"
    echo "• PX4 instances: $px4_running_count/${#DEFAULT_CONTAINERS[@]}"
    echo "• Network mode: Bridge (host.docker.internal)"
}

# Function to show logs for a container
show_logs() {
    local container_name="$1"
    local follow="${2:-false}"
    
    if [[ -z "$container_name" ]]; then
        print_error "Container name required. Usage: $0 logs <container_name> [follow]"
        return 1
    fi
    
    if [[ "$follow" == "follow" || "$follow" == "-f" ]]; then
        docker logs -f "$container_name"
    else
        docker logs --tail 50 "$container_name"
    fi
}

# Function to stop all containers
stop_containers() {
    print_header "Stopping all bridge swarm containers..."
    docker-compose -f "$COMPOSE_FILE" down
    print_status "✅ All bridge containers stopped"
}

# Function to restart containers in delayed mode
restart_containers() {
    local num_drones=${1:-9}
    print_header "Restarting bridge containers in delayed mode..."
    stop_containers
    sleep 2
    start_containers "$num_drones"
}

# Function to test network connectivity
test_network_connectivity() {
    print_header "Bridge Network Connectivity Test"
    
    # Test host.docker.internal connectivity
    print_status "Testing host.docker.internal connectivity..."
    
    if test_connectivity "host.docker.internal" 41451; then
        print_status "✅ AirSim API (41451): ACCESSIBLE via host.docker.internal"
    else
        print_error "❌ AirSim API (41451): NOT ACCESSIBLE via host.docker.internal"
    fi
    
    # Test PX4 SITL ports (4561-4563)
    for port in 4561 4562 4563; do
        if test_connectivity "localhost" "$port"; then
            print_status "✅ PX4 SITL ($port): ACCESSIBLE via localhost (port mapped)"
        else
            print_warning "⚠️  PX4 SITL ($port): NOT ACCESSIBLE (may be normal if PX4 not started)"
        fi
    done
    
    # Additional WSL2 test if available
    if grep -qi "microsoft" /proc/version 2>/dev/null; then
        WSL2_IP=$(ip route show | grep "src" | awk '{print $NF}' | head -1)
        if [[ -n "$WSL2_IP" ]]; then
            print_status "Testing WSL2 direct IP: $WSL2_IP"
            if test_connectivity "$WSL2_IP" 41451; then
                print_status "✅ AirSim also accessible via WSL2 IP: $WSL2_IP:41451"
            else
                print_warning "⚠️  AirSim not accessible via WSL2 IP: $WSL2_IP:41451"
            fi
        fi
    fi
}

# Function to show current configuration
show_config() {
    print_header "Bridge Network Configuration"
    
    # Environment detection
    if grep -qi "microsoft" /proc/version 2>/dev/null; then
        print_status "Environment: WSL2 + Docker Desktop (Bridge Network)"
        WSL2_IP=$(ip route show | grep "src" | awk '{print $NF}' | head -1)
        print_status "WSL2 IP: ${WSL2_IP:-'Not detected'}"
    else
        print_status "Environment: Native Linux/Docker (Bridge Network)"
    fi
    
    # Compose file settings
    if [[ -f "$COMPOSE_FILE" ]]; then
        print_status "Compose file: $COMPOSE_FILE"
        print_status "PX4_SIM_HOSTNAME: host.docker.internal (Docker host alias for bridge mode)"
        print_status "Network: Custom bridge network (172.20.0.0/16)"
    else
        print_error "Compose file not found: $COMPOSE_FILE"
    fi
    
    # Container status
    echo ""
    show_status
}

# Function to access container shell
shell_access() {
    local container_name="$1"
    
    if [[ -z "$container_name" ]]; then
        print_error "Container name required. Usage: $0 shell <container_name>"
        return 1
    fi
    
    if ! docker ps | grep -q "$container_name"; then
        print_error "Container '$container_name' is not running"
        return 1
    fi
    
    print_status "Opening shell in $container_name (bridge network)..."
    docker exec -it "$container_name" /bin/bash
}

# Function to show help
show_help() {
    echo "Bridge Network PX4 Swarm Management Script"
    echo "Compatible with Docker Desktop limitations on Windows/WSL2"
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  start [num]          Start containers in delayed mode (default: 9 drones)"
    echo "  stop                 Stop all containers"
    echo "  restart [num]        Restart containers in delayed mode"
    echo "  status               Show container and PX4 status"
    echo "  run-px4 <container>  Start PX4 SITL in specific container"
    echo "  run-all-px4          Start PX4 SITL in all running containers"
    echo "  logs <container> [follow]  Show container logs"
    echo "  shell <container>    Open shell in container"
    echo "  test-connection      Test network connectivity to AirSim"
    echo "  show-config          Show bridge network configuration"
    echo "  help                 Show this help"
    echo ""
    echo "Bridge Network Features:"
    echo "  • Uses host.docker.internal to reach AirSim"
    echo "  • Port mapping: localhost:4561-4569 -> containers"
    echo "  • Compatible with Docker Desktop Enhanced Container Isolation"
    echo "  • Custom bridge network (172.20.0.0/16) for better isolation"
    echo ""
    echo "Examples:"
    echo "  $0 start 3                            # Start 3 containers"
    echo "  $0 run-px4 px4-bridge-drone-1        # Start PX4 in drone 1"
    echo "  $0 logs px4-bridge-drone-1 follow    # Follow logs for drone 1"
    echo "  $0 status                             # Check all container status"
    echo "  $0 show-config                        # Show bridge configuration"
    echo "  $0 test-connection                    # Test host.docker.internal connectivity"
    echo ""
    echo "Notes:"
    echo "  • AirSim should be configured with ApiServerEndpoint: '0.0.0.0:41451'"
    echo "  • PythonClient connects to localhost:4561-4569 (port mapped)"
    echo "  • No need to modify AirSim settings.json for LocalHostIp"
}

# Main script logic
case "${1:-help}" in
    "start")
        start_containers "${2:-9}"
        ;;
    "stop")
        stop_containers
        ;;
    "restart")
        restart_containers "${2:-9}"
        ;;
    "status")
        show_status
        ;;
    "run-px4")
        run_px4_in_container "$2"
        ;;
    "run-all-px4")
        run_px4_in_all_containers
        ;;
    "logs")
        show_logs "$2" "$3"
        ;;
    "shell")
        shell_access "$2"
        ;;
    "test-connection")
        test_network_connectivity
        ;;
    "show-config")
        show_config
        ;;
    "help"|*)
        show_help
        ;;
esac