#!/bin/bash

# AirSim ROS2 Docker Management Script
# This script helps build and run the AirSim ROS2 Docker container

set -e  # Exit on any error

# Default configuration
CONTAINER_NAME="airsim-ros2-wrapper"
IMAGE_NAME="airsim-ros2-wrapper"
IMAGE_TAG="latest"
DOCKERFILE_PATH="./Dockerfile"
BUILD_CONTEXT="."
VNC_PORT="6080"
ROS_DOMAIN_ID="0"
AIRSIM_HOST_IP="host.docker.internal"
AIRSIM_HOST_PORT="41451"
LAUNCH_RVIZ="false"

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
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [COMMAND] [OPTIONS]

Commands:
    build       Build the Docker image
    run         Run the Docker container
    stop        Stop the running container
    restart     Restart the container
    logs        Show container logs
    shell       Open a shell in the running container
    clean       Remove container and image
    status      Show container status
    help        Show this help message

Options:
    --name NAME             Container name (default: $CONTAINER_NAME)
    --image IMAGE           Image name (default: $IMAGE_NAME)
    --tag TAG               Image tag (default: $IMAGE_TAG)
    --dockerfile PATH       Dockerfile path (default: $DOCKERFILE_PATH)
    --context PATH          Build context path (default: $BUILD_CONTEXT)
    --vnc-port PORT         VNC port (default: $VNC_PORT)
    --ros-domain-id ID      ROS Domain ID (default: $ROS_DOMAIN_ID)
    --airsim-host HOST      AirSim host IP (default: $AIRSIM_HOST_IP)
    --airsim-port PORT      AirSim host port (default: $AIRSIM_HOST_PORT)
    --launch-rviz           Launch RViz2 with AirSim node
    --no-cache              Build without cache
    --detach                Run container in detached mode
    --privileged            Run container in privileged mode

Examples:
    $0 build --no-cache
    $0 run --launch-rviz --vnc-port 8080
    $0 run --airsim-host 192.168.1.100 --airsim-port 41451
    $0 shell
    $0 logs --follow

EOF
}

# Function to parse arguments
parse_args() {
    COMMAND=""
    NO_CACHE=""
    DETACHED=""
    PRIVILEGED=""
    FOLLOW_LOGS=""
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            build|run|stop|restart|logs|shell|clean|status|help)
                COMMAND="$1"
                shift
                ;;
            --name)
                CONTAINER_NAME="$2"
                shift 2
                ;;
            --image)
                IMAGE_NAME="$2"
                shift 2
                ;;
            --tag)
                IMAGE_TAG="$2"
                shift 2
                ;;
            --dockerfile)
                DOCKERFILE_PATH="$2"
                shift 2
                ;;
            --context)
                BUILD_CONTEXT="$2"
                shift 2
                ;;
            --vnc-port)
                VNC_PORT="$2"
                shift 2
                ;;
            --ros-domain-id)
                ROS_DOMAIN_ID="$2"
                shift 2
                ;;
            --airsim-host)
                AIRSIM_HOST_IP="$2"
                shift 2
                ;;
            --airsim-port)
                AIRSIM_HOST_PORT="$2"
                shift 2
                ;;
            --launch-rviz)
                LAUNCH_RVIZ="true"
                shift
                ;;
            --no-cache)
                NO_CACHE="--no-cache"
                shift
                ;;
            --detach)
                DETACHED="-d"
                shift
                ;;
            --privileged)
                PRIVILEGED="--privileged"
                shift
                ;;
            --follow)
                FOLLOW_LOGS="-f"
                shift
                ;;
            *)
                print_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done
}

# Function to check if Docker is running
check_docker() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker is not running. Please start Docker and try again."
        exit 1
    fi
}

# Function to build the Docker image
build_image() {
    print_header "Building AirSim ROS2 Docker Image"
    
    if [[ ! -f "$DOCKERFILE_PATH" ]]; then
        print_error "Dockerfile not found at: $DOCKERFILE_PATH"
        exit 1
    fi
    
    print_status "Building image: ${IMAGE_NAME}:${IMAGE_TAG}"
    print_status "Using Dockerfile: $DOCKERFILE_PATH"
    print_status "Build context: $BUILD_CONTEXT"
    
    docker build $NO_CACHE \
        -f "$DOCKERFILE_PATH" \
        -t "${IMAGE_NAME}:${IMAGE_TAG}" \
        "$BUILD_CONTEXT"
    
    print_status "Build completed successfully!"
}

# Function to run the container
run_container() {
    print_header "Running AirSim ROS2 Container"
    
    # Check if container already exists
    if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_warning "Container '$CONTAINER_NAME' already exists."
        read -p "Do you want to remove it and create a new one? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
        else
            print_status "Starting existing container..."
            docker start "$CONTAINER_NAME"
            return
        fi
    fi
    
    print_status "Starting new container: $CONTAINER_NAME"
    print_status "VNC will be available at: http://localhost:$VNC_PORT"
    print_status "ROS Domain ID: $ROS_DOMAIN_ID"
    print_status "AirSim Host: $AIRSIM_HOST_IP:$AIRSIM_HOST_PORT"
    print_status "Launch RViz2: $LAUNCH_RVIZ"
    
    # Create and run the container
    docker run $DETACHED $PRIVILEGED \
        --name "$CONTAINER_NAME" \
        -p "${VNC_PORT}:6080" \
        -p 7400:7400/udp \
        -p 7401:7401/udp \
        -p 7402:7402/udp \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        -e AIRSIM_HOST_IP="$AIRSIM_HOST_IP" \
        -e AIRSIM_HOST_PORT="$AIRSIM_HOST_PORT" \
        -e LAUNCH_RVIZ="$LAUNCH_RVIZ" \
        -e DISPLAY=:1 \
        --shm-size=1g \
        "${IMAGE_NAME}:${IMAGE_TAG}"
    
    if [[ -z "$DETACHED" ]]; then
        print_status "Container is running in foreground mode."
    else
        print_status "Container started in detached mode."
        print_status "Access VNC at: http://localhost:$VNC_PORT"
        print_status "Default VNC password: ubuntu"
    fi
}

# Function to stop the container
stop_container() {
    print_header "Stopping Container"
    
    if docker ps --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_status "Stopping container: $CONTAINER_NAME"
        docker stop "$CONTAINER_NAME"
        print_status "Container stopped successfully!"
    else
        print_warning "Container '$CONTAINER_NAME' is not running."
    fi
}

# Function to restart the container
restart_container() {
    print_header "Restarting Container"
    
    if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_status "Restarting container: $CONTAINER_NAME"
        docker restart "$CONTAINER_NAME"
        print_status "Container restarted successfully!"
        print_status "Access VNC at: http://localhost:$VNC_PORT"
    else
        print_warning "Container '$CONTAINER_NAME' does not exist."
        print_status "Use '$0 run' to create and start a new container."
    fi
}

# Function to show logs
show_logs() {
    print_header "Container Logs"
    
    if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker logs $FOLLOW_LOGS "$CONTAINER_NAME"
    else
        print_warning "Container '$CONTAINER_NAME' does not exist."
    fi
}

# Function to open shell
open_shell() {
    print_header "Opening Shell in Container"
    
    if docker ps --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_status "Opening bash shell in container: $CONTAINER_NAME"
        docker exec -it "$CONTAINER_NAME" /bin/bash
    else
        print_warning "Container '$CONTAINER_NAME' is not running."
        print_status "Use '$0 run' to start the container first."
    fi
}

# Function to clean up
clean_up() {
    print_header "Cleaning Up"
    
    # Stop and remove container
    if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_status "Removing container: $CONTAINER_NAME"
        docker rm -f "$CONTAINER_NAME"
    fi
    
    # Remove image
    if docker images --format 'table {{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}:${IMAGE_TAG}$"; then
        read -p "Do you want to remove the image '${IMAGE_NAME}:${IMAGE_TAG}'? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_status "Removing image: ${IMAGE_NAME}:${IMAGE_TAG}"
            docker rmi "${IMAGE_NAME}:${IMAGE_TAG}"
        fi
    fi
    
    print_status "Cleanup completed!"
}

# Function to show status
show_status() {
    print_header "Container Status"
    
    # Check if container exists
    if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_status "Container '$CONTAINER_NAME' exists"
        
        # Check if running
        if docker ps --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            print_status "Status: RUNNING"
            print_status "VNC Access: http://localhost:$VNC_PORT"
            
            # Show container details
            echo
            docker ps --filter "name=${CONTAINER_NAME}" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
        else
            print_warning "Status: STOPPED"
        fi
    else
        print_warning "Container '$CONTAINER_NAME' does not exist"
    fi
    
    # Check if image exists
    if docker images --format 'table {{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}:${IMAGE_TAG}$"; then
        print_status "Image '${IMAGE_NAME}:${IMAGE_TAG}' exists"
    else
        print_warning "Image '${IMAGE_NAME}:${IMAGE_TAG}' does not exist"
    fi
}

# Main execution
main() {
    check_docker
    
    if [[ -z "$COMMAND" ]] || [[ "$COMMAND" == "help" ]]; then
        show_usage
        exit 0
    fi
    
    case "$COMMAND" in
        build)
            build_image
            ;;
        run)
            run_container
            ;;
        stop)
            stop_container
            ;;
        restart)
            restart_container
            ;;
        logs)
            show_logs
            ;;
        shell)
            open_shell
            ;;
        clean)
            clean_up
            ;;
        status)
            show_status
            ;;
        *)
            print_error "Unknown command: $COMMAND"
            show_usage
            exit 1
            ;;
    esac
}

# Parse arguments and run main function
parse_args "$@"
main