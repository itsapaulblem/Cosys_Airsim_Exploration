#!/bin/bash

# Optimized Bridge Network PX4 Swarm Management Script
# Reduced redundancy and improved efficiency
# Compatible with Docker Desktop limitations on Windows/WSL2

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$PROJECT_DIR/docker-compose-slim.yml"

# Default PX4 simulation hostname (can be overridden)
DEFAULT_PX4_SIM_HOSTNAME="172.28.240.1"
CUSTOM_PX4_SIM_HOSTNAME=""

# Container configuration
readonly DEFAULT_CONTAINERS=(
    "px4-bridge-drone-1" "px4-bridge-drone-2" "px4-bridge-drone-3"
    "px4-bridge-drone-4" "px4-bridge-drone-5" "px4-bridge-drone-6"
    "px4-bridge-drone-7" "px4-bridge-drone-8" "px4-bridge-drone-9"
)

# Colors
readonly RED='\033[0;31m' GREEN='\033[0;32m' YELLOW='\033[1;33m' BLUE='\033[0;34m' NC='\033[0m'

# Cached environment detection
ENVIRONMENT_CACHE=""
WSL2_IP_CACHE=""

# Logging functions
log() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }
header() { echo -e "${BLUE}[BRIDGE SWARM]${NC} $1"; }


# Function to get the effective PX4_SIM_HOSTNAME
get_px4_sim_hostname() {
    if [[ -n "$CUSTOM_PX4_SIM_HOSTNAME" ]]; then
        echo "$CUSTOM_PX4_SIM_HOSTNAME"
    else
        echo "$DEFAULT_PX4_SIM_HOSTNAME"
    fi
}

# Optimized connectivity test with caching
test_connectivity() {
    local host="$1" port="${2:-41451}" timeout="${3:-3}"
    
    if command -v nc >/dev/null 2>&1; then
        timeout "$timeout" nc -z "$host" "$port" 2>/dev/null
    elif command -v telnet >/dev/null 2>&1; then
        timeout "$timeout" bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null
    else
        return 1
    fi
}

# Cached environment detection
detect_environment() {
    if [[ -n "$ENVIRONMENT_CACHE" ]]; then
        return 0
    fi
    
    ENVIRONMENT_CACHE="bridge"
    if grep -qi "microsoft" /proc/version 2>/dev/null; then
        ENVIRONMENT_CACHE="wsl2"
        WSL2_IP_CACHE=$(ip route show | grep "src" | awk '{print $NF}' | head -1)
    fi
}

# Network connectivity check with environment awareness
check_network_connectivity() {
    detect_environment
    log "Bridge Network Mode - Testing connectivity..."
    
    local connectivity_ok=false
    local px4_sim_host=$(get_px4_sim_hostname)
    
    # Test configured PX4_SIM_HOSTNAME
    if test_connectivity "$px4_sim_host" 41451; then
        log "✅ AirSim API accessible via $px4_sim_host:41451"
        connectivity_ok=true
    else
        warn "⚠️  AirSim API not accessible via $px4_sim_host:41451"
    fi
    
    # Test WSL2 IP if available
    if [[ "$ENVIRONMENT_CACHE" == "wsl2" && -n "$WSL2_IP_CACHE" ]]; then
        if test_connectivity "$WSL2_IP_CACHE" 41451; then
            log "✅ AirSim also accessible via WSL2 IP: $WSL2_IP_CACHE:41451"
            connectivity_ok=true
        else
            warn "⚠️  AirSim not accessible via WSL2 IP: $WSL2_IP_CACHE:41451"
        fi
    fi
    
    if [[ "$connectivity_ok" == "false" ]]; then
        warn "Make sure AirSim is running and configured for bridge network access"
    fi
}

# Container operations
get_running_containers() {
    local running=()
    for container in "${DEFAULT_CONTAINERS[@]}"; do
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            running+=("$container")
        fi
    done
    printf '%s\n' "${running[@]}"
}

get_container_info() {
    local container="$1"
    local ip status px4_status
    
    if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
        ip=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$container" 2>/dev/null || echo "N/A")
        
        if docker exec "$container" pgrep -f "px4" >/dev/null 2>&1; then
            px4_status="PX4_RUNNING"
        else
            px4_status="CONTAINER_ONLY"
        fi
        
        echo "$container|RUNNING|$ip|$px4_status"
    else
        echo "$container|STOPPED|N/A|NOT_RUNNING"
    fi
}

# Main operations
start_containers() {
    local num_drones=${1:-9}
    local px4_sim_host=$(get_px4_sim_hostname)
    
    header "Starting $num_drones containers in bridge network mode..."
    log "Using PX4_SIM_HOSTNAME: $px4_sim_host"
    check_network_connectivity
    
    local containers_to_start=("${DEFAULT_CONTAINERS[@]:0:$num_drones}")
    
    log "Starting containers: ${containers_to_start[*]}"
    
    # Set PX4_SIM_HOSTNAME environment variable for docker-compose
    export PX4_SIM_HOSTNAME="$px4_sim_host"
    docker-compose -f "$COMPOSE_FILE" up -d "${containers_to_start[@]}"
    
    log "Waiting for containers to be ready..."
    sleep 5
    
    # Show status
    show_status
    
    echo ""
    header "Port Mappings:"
    for ((i=0; i<num_drones; i++)); do
        log "Drone $((i+1)): localhost:$((4561 + i)) -> container:$((4561 + i))"
    done
    
    echo ""
    header "Next Steps:"
    echo "• Use '$0 run-px4 <container_name>' to start PX4 in specific container (with MAVLink broadcast)"
    echo "• Use '$0 run-all-px4' to start PX4 in all running containers"
    echo "• Use '$0 status' to check container status"
    echo "• Manual start: docker exec -it <container_name> Scripts/run_airsim_sitl_broadcast.sh <drone_num>"
}

run_px4_in_container() {
    local container_name="$1"
    
    if [[ -z "$container_name" ]]; then
        error "Container name required. Usage: $0 run-px4 <container_name>"
        return 1
    fi
    
    if ! docker ps --format "{{.Names}}" | grep -q "^$container_name$"; then
        error "Container '$container_name' is not running"
        return 1
    fi
    
    # Extract drone number from container name
    local drone_num=$(echo "$container_name" | sed 's/px4-bridge-drone-//')
    
    header "Starting PX4 SITL in bridge container: $container_name (drone $drone_num)"
    
    docker exec -d "$container_name" bash -c "
        cd /px4_workspace/PX4-Autopilot
        if [ -f \"Scripts/run_airsim_sitl_broadcast.sh\" ]; then
            echo \"🚀 Starting PX4 SITL with AirSim configuration and MAVLink broadcast...\"
            echo \"🔗 Instance: $drone_num\"
            echo \"🌐 Network mode: Bridge (Windows Host IP: $PX4_SIM_HOSTNAME)\"
            echo \"📡 MAVLink broadcast: ENABLED for QGroundControl\"
            Scripts/run_airsim_sitl_broadcast.sh $drone_num
        elif [ -f \"Scripts/run_airsim_sitl.sh\" ]; then
            echo \"🚀 Starting PX4 SITL with AirSim configuration...\"
            echo \"🔗 Instance: $drone_num\"
            echo \"🌐 Network mode: Bridge (Windows Host IP: $PX4_SIM_HOSTNAME)\"
            echo \"⚠️  MAVLink broadcast: DISABLED (use run_airsim_sitl_broadcast.sh for QGroundControl)\"
            Scripts/run_airsim_sitl.sh $drone_num
        else
            echo \"❌ No AirSim SITL scripts found in Scripts/\"
            echo \"Available scripts:\"
            ls -la Scripts/
            exit 1
        fi
    "
    
    sleep 2
    log "✅ PX4 SITL started in $container_name (drone $drone_num)"
}

run_px4_in_all_containers() {
    header "Starting PX4 SITL in all running bridge containers..."
    
    local running_containers
    mapfile -t running_containers < <(get_running_containers)
    
    if [[ ${#running_containers[@]} -eq 0 ]]; then
        error "No bridge swarm containers are currently running"
        return 1
    fi
    
    log "Found ${#running_containers[@]} running containers"
    
    for container in "${running_containers[@]}"; do
        log "Starting PX4 in $container..."
        run_px4_in_container "$container"
        sleep 2
    done
    
    log "✅ PX4 SITL started in all bridge containers"
}

show_status() {
    header "Bridge Swarm Container Status:"
    
    local running_count=0 px4_running_count=0
    
    for container in "${DEFAULT_CONTAINERS[@]}"; do
        local info
        info=$(get_container_info "$container")
        IFS='|' read -r name status ip px4_status <<< "$info"
        
        case "$status" in
            "RUNNING")
                running_count=$((running_count + 1))
                case "$px4_status" in
                    "PX4_RUNNING")
                        log "✅ $name - CONTAINER + PX4 RUNNING (IP: $ip)"
                        px4_running_count=$((px4_running_count + 1))
                        ;;
                    *)
                        warn "🟡 $name - CONTAINER ONLY (IP: $ip, PX4 not started)"
                        ;;
                esac
                ;;
            *)
                error "❌ $name - NOT RUNNING"
                ;;
        esac
    done
    
    echo ""
    header "Summary:"
    echo "• Running containers: $running_count/${#DEFAULT_CONTAINERS[@]}"
    echo "• PX4 instances: $px4_running_count/${#DEFAULT_CONTAINERS[@]}"
    echo "• Network mode: Bridge (Windows Host IP)"
}

show_logs() {
    local container_name="$1" follow="${2:-false}"
    
    if [[ -z "$container_name" ]]; then
        error "Container name required. Usage: $0 logs <container_name> [follow]"
        return 1
    fi
    
    if [[ "$follow" == "follow" || "$follow" == "-f" ]]; then
        docker logs -f "$container_name"
    else
        docker logs --tail 50 "$container_name"
    fi
}

stop_containers() {
    header "Stopping all bridge swarm containers..."
    docker-compose -f "$COMPOSE_FILE" down
    log "✅ All bridge containers stopped"
}

delete_containers() {
    header "Removing all bridge swarm containers, networks, and volumes..."
    
    # Stop and remove containers with volumes
    docker-compose -f "$COMPOSE_FILE" down --volumes --remove-orphans
    
    # Clean up any remaining px4-bridge containers
    local containers_to_remove
    containers_to_remove=$(docker ps -aq --filter "name=px4-bridge-drone" 2>/dev/null || true)
    if [[ -n "$containers_to_remove" ]]; then
        log "Removing remaining px4-bridge containers..."
        docker rm -f $containers_to_remove
    fi
    
    # Remove px4_network if it exists
    if docker network ls | grep -q "px4_network"; then
        log "Removing px4_network..."
        docker network rm px4_network 2>/dev/null || true
    fi
    
    # Clean up shared volumes
    if docker volume ls | grep -q "px4-shared-data"; then
        log "Removing px4-shared-data volume..."
        docker volume rm px4-shared-data 2>/dev/null || true
    fi
    
    log "✅ All bridge containers, networks, and volumes removed"
}

test_network_connectivity() {
    header "Bridge Network Connectivity Test"
    check_network_connectivity
    
    # Test PX4 SITL ports
    for port in 4561 4562 4563; do
        if test_connectivity "localhost" "$port"; then
            log "✅ PX4 SITL ($port): ACCESSIBLE via localhost"
        else
            warn "⚠️  PX4 SITL ($port): NOT ACCESSIBLE (may be normal if PX4 not started)"
        fi
    done
}

show_config() {
    header "Bridge Network Configuration"
    detect_environment
    
    case "$ENVIRONMENT_CACHE" in
        "wsl2")
            log "Environment: WSL2 + Docker Desktop (Bridge Network)"
            log "WSL2 IP: ${WSL2_IP_CACHE:-'Not detected'}"
            ;;
        *)
            log "Environment: Native Linux/Docker (Bridge Network)"
            ;;
    esac
    
    if [[ -f "$COMPOSE_FILE" ]]; then
        log "Compose file: $COMPOSE_FILE (slim configuration)"
        log "Network: Custom bridge network (172.20.0.0/16)"
        log "Image: px4-airsim:slim (optimized multi-stage build)"
        log "Scripts: Ultra-swarm configuration with GPS"
        log "PX4_SIM_HOSTNAME: $(get_px4_sim_hostname)"
    else
        error "Compose file not found: $COMPOSE_FILE"
    fi
    
    echo ""
    show_status
}

shell_access() {
    local container_name="$1"
    
    if [[ -z "$container_name" ]]; then
        error "Container name required. Usage: $0 shell <container_name>"
        return 1
    fi
    
    if ! docker ps --format "{{.Names}}" | grep -q "^$container_name$"; then
        error "Container '$container_name' is not running"
        return 1
    fi
    
    log "Opening shell in $container_name (bridge network)..."
    docker exec -it "$container_name" /bin/bash
}

show_help() {
    cat << EOF
Optimized Bridge Network PX4 Swarm Management Script (Slim Configuration)
Compatible with Docker Desktop limitations on Windows/WSL2

Usage: $0 [--PX4-Sim-IP <ip>] <command> [options]

Global Options:
  --PX4-Sim-IP <ip>    Override PX4_SIM_HOSTNAME (default: 172.28.240.1)

Container Management:
  start [num]          Start N containers (1-9, default: 9 drones)
  stop                 Stop all containers
  delete               Remove all containers, networks, and volumes
  restart [num]        Restart containers

PX4 Control:
  run-px4 <container>  Start PX4 SITL in specific container (uses AirSim script with MAVLink broadcast)
  run-all-px4          Start PX4 SITL in all running containers

Monitoring:
  status               Show container and PX4 status
  logs <container> [follow]  Show container logs
  shell <container>    Open shell in container

Network:
  test-connection      Test network connectivity to AirSim
  show-config          Show bridge network configuration
  help                 Show this help

Configuration:
  • Uses docker-compose-slim.yml with optimized images
  • Bridge network: px4_network (172.20.0.0/16)
  • Port mapping: localhost:4561-4569 → containers
  • AirSim SITL scripts with MAVLink broadcast for QGroundControl
  • Default PX4_SIM_HOSTNAME: 172.28.240.1

Examples:
  $0 start 5                                    # Start 5 containers (default IP)
  $0 --PX4-Sim-IP 192.168.1.100 start 5        # Start 5 containers (custom IP)
  $0 --PX4-Sim-IP 10.0.0.50 run-all-px4        # Start PX4 with custom AirSim IP
  $0 show-config                                # Show current configuration
  $0 delete                                     # Clean up everything

IP Override Examples:
  --PX4-Sim-IP 192.168.1.100    # LAN AirSim instance
  --PX4-Sim-IP 10.0.0.50         # Different subnet
  --PX4-Sim-IP 127.0.0.1         # Local testing
  --PX4-Sim-IP 172.28.240.1      # Default WSL2 gateway
EOF
}

# Main command dispatcher
# Process arguments for --PX4-Sim-IP
while [[ $# -gt 0 ]]; do
    case $1 in
        --PX4-Sim-IP)
            if [[ -n "$2" && "$2" != --* ]]; then
                CUSTOM_PX4_SIM_HOSTNAME="$2"
                log "Custom PX4_SIM_HOSTNAME set to: $CUSTOM_PX4_SIM_HOSTNAME"
                shift 2
            else
                error "--PX4-Sim-IP requires an IP address"
                exit 1
            fi
            ;;
        *)
            break
            ;;
    esac
done

case "${1:-help}" in
    "start") start_containers "${2:-9}" ;;
    "stop") stop_containers ;;
    "delete") delete_containers ;;
    "restart") stop_containers; sleep 2; start_containers "${2:-9}" ;;
    "status") show_status ;;
    "run-px4") run_px4_in_container "$2" ;;
    "run-all-px4") run_px4_in_all_containers ;;
    "logs") show_logs "$2" "$3" ;;
    "shell") shell_access "$2" ;;
    "test-connection") test_network_connectivity ;;
    "show-config") show_config ;;
    "help"|*) show_help ;;
esac