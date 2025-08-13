#!/bin/bash
# AirSim Container Build and Run Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"

# Default values
PROFILE="full-stack"
BUILD_CACHE="--build"
DISPLAY_MODE="vnc"
DETACHED="-d"
CLEANUP="false"

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

show_help() {
    cat << EOF
AirSim Container Management Script

Usage: $0 [OPTIONS] [COMMAND]

Commands:
    build           Build the AirSim container images
    run             Run the full stack
    stop            Stop all containers
    clean           Remove all containers and images
    logs            Show container logs
    shell           Open shell in AirSim container
    test            Run connectivity tests
    status          Show container status

Options:
    -p, --profile   Docker Compose profile (default: full-stack)
                    Options: full-stack, airsim-only, px4-only, multi-drone, debug
    -d, --display   Display mode (default: vnc)
                    Options: vnc, headless, host
    -f, --foreground Run in foreground (default: background)
    --no-cache      Build without cache
    --cleanup       Remove existing containers before running
    -h, --help      Show this help message

Examples:
    $0 build                        # Build all images
    $0 run                          # Run full stack with VNC
    $0 run -d headless             # Run headless mode
    $0 run -p airsim-only          # Run only AirSim
    $0 test                         # Test connections
    $0 shell                        # Open AirSim shell
    $0 logs airsim                  # Show AirSim logs

VNC Access:
    URL: http://localhost:5900
    Or use VNC client: localhost:5900

API Access:
    AirSim API: localhost:41451
    Python: airsim.MultirotorClient("localhost")

EOF
}

check_prerequisites() {
    print_header "Checking Prerequisites"
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Check Docker Compose
    if ! docker compose version &> /dev/null; then
        print_error "Docker Compose is not available"
        exit 1
    fi
    
    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        print_error "Docker daemon is not running"
        exit 1
    fi
    
    print_status "Prerequisites check passed"
}

build_images() {
    print_header "Building AirSim Container Images"
    
    cd "$DOCKER_DIR"
    
    if [ "$BUILD_CACHE" = "--no-cache" ]; then
        print_warning "Building without cache (this will take longer)"
        docker compose build --no-cache
    else
        docker compose build
    fi
    
    print_status "Build completed successfully"
}

run_stack() {
    print_header "Running AirSim Stack"
    
    cd "$DOCKER_DIR"
    
    # Cleanup if requested
    if [ "$CLEANUP" = "true" ]; then
        print_warning "Cleaning up existing containers..."
        docker compose --profile "$PROFILE" down --remove-orphans 2>/dev/null || true
    fi
    
    # Set environment variables
    export DISPLAY_MODE="$DISPLAY_MODE"
    
    # Handle display mode specific setup
    if [ "$DISPLAY_MODE" = "host" ]; then
        print_status "Setting up host display forwarding..."
        if [ -n "$DISPLAY" ]; then
            export DISPLAY="$DISPLAY"
            # Allow X11 forwarding (Linux only)
            xhost +local:docker 2>/dev/null || print_warning "Could not set xhost permissions"
        else
            print_warning "DISPLAY not set, falling back to VNC mode"
            export DISPLAY_MODE="vnc"
        fi
    fi
    
    print_status "Starting stack with profile: $PROFILE"
    print_status "Display mode: $DISPLAY_MODE"
    
    if [ "$DETACHED" = "-d" ]; then
        docker compose --profile "$PROFILE" up $DETACHED
        print_status "Stack started in background"
        print_status "Access VNC at: http://localhost:5900"
        print_status "AirSim API at: localhost:41451"
        
        # Show status
        sleep 3
        show_status
    else
        print_status "Starting in foreground mode (Ctrl+C to stop)"
        docker compose --profile "$PROFILE" up
    fi
}

stop_stack() {
    print_header "Stopping AirSim Stack"
    
    cd "$DOCKER_DIR"
    docker compose --profile all down
    
    print_status "Stack stopped"
}

clean_all() {
    print_header "Cleaning Up All Resources"
    
    cd "$DOCKER_DIR"
    
    print_warning "This will remove all containers, networks, and images"
    read -p "Are you sure? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Stop and remove containers
        docker compose --profile all down --volumes --remove-orphans
        
        # Remove images
        docker rmi $(docker images -q "*airsim*" "*px4*") 2>/dev/null || true
        
        # Clean up dangling images
        docker image prune -f
        
        print_status "Cleanup completed"
    else
        print_status "Cleanup cancelled"
    fi
}

show_logs() {
    cd "$DOCKER_DIR"
    
    if [ -n "$1" ]; then
        docker compose logs -f "$1"
    else
        docker compose logs -f
    fi
}

open_shell() {
    print_header "Opening Shell in AirSim Container"
    
    if ! docker ps | grep -q airsim-container; then
        print_error "AirSim container is not running"
        print_status "Start the stack first: $0 run"
        exit 1
    fi
    
    docker exec -it airsim-container /bin/bash
}

run_tests() {
    print_header "Running Connectivity Tests"
    
    cd "$DOCKER_DIR"
    
    # Start stack if not running
    if ! docker ps | grep -q airsim-container; then
        print_status "Starting test environment..."
        DISPLAY_MODE="headless" DETACHED="-d" run_stack
        sleep 15  # Wait for services to start
    fi
    
    # Run tests in AirSim container
    docker exec airsim-container /home/airsim_user/Scripts/entrypoint.sh test
}

show_status() {
    print_header "Container Status"
    
    cd "$DOCKER_DIR"
    docker compose ps
    
    echo
    print_header "Network Information"
    docker network inspect airsim-network --format '{{range .Containers}}{{.Name}}: {{.IPv4Address}}{{"\n"}}{{end}}' 2>/dev/null || echo "Network not found"
    
    echo
    print_header "Port Mappings"
    docker compose port airsim 41451 2>/dev/null && echo "AirSim API: $(docker compose port airsim 41451)" || echo "AirSim API: Not available"
    docker compose port airsim 5900 2>/dev/null && echo "VNC Access: $(docker compose port airsim 5900)" || echo "VNC Access: Not available"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--profile)
            PROFILE="$2"
            shift 2
            ;;
        -d|--display)
            DISPLAY_MODE="$2"
            shift 2
            ;;
        -f|--foreground)
            DETACHED=""
            shift
            ;;
        --no-cache)
            BUILD_CACHE="--no-cache"
            shift
            ;;
        --cleanup)
            CLEANUP="true"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        build|run|stop|clean|logs|shell|test|status)
            COMMAND="$1"
            shift
            break
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Set default command if none provided
COMMAND=${COMMAND:-run}

# Main execution
print_header "AirSim Container Management"
check_prerequisites

case $COMMAND in
    build)
        build_images
        ;;
    run)
        build_images
        run_stack
        ;;
    stop)
        stop_stack
        ;;
    clean)
        clean_all
        ;;
    logs)
        show_logs "$1"
        ;;
    shell)
        open_shell
        ;;
    test)
        run_tests
        ;;
    status)
        show_status
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        show_help
        exit 1
        ;;
esac 