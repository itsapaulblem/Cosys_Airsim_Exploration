#!/bin/bash
# Multi-Swarm PX4 Startup Helper Script
# Makes it easy to start different swarm configurations

set -e

COMPOSE_FILE="docker-compose.multi-swarm-fixed.yml"

show_usage() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  single-drone     Start 1 drone in swarm 1 (AirSim port 4561)"
    echo "  swarm1-2         Start 2 drones in swarm 1 (AirSim ports 4561-4562)"
    echo "  swarm1-3         Start 3 drones in swarm 1 (AirSim ports 4561-4563)"
    echo "  swarm2-2         Start 2 drones in swarm 2 (AirSim ports 4571-4572)"
    echo "  both-swarms      Start 3 drones in swarm 1 + 2 drones in swarm 2"
    echo "  status           Show status of all containers"
    echo "  logs [drone]     Show logs for specific drone (e.g., logs px4-swarm-1-drone-1)"
    echo "  stop             Stop all swarm containers"
    echo "  clean            Stop and remove all swarm containers and networks"
    echo ""
    echo "Port Allocation:"
    echo "  Swarm 1: AirSim TCP 4561-4570, QGC UDP 14550-14559, MAVLink UDP 18570-18579"
    echo "  Swarm 2: AirSim TCP 4571-4580, QGC UDP 14560-14569, MAVLink UDP 18580-18589"
    echo ""
    echo "Examples:"
    echo "  $0 single-drone                    # Start 1 drone"
    echo "  $0 swarm1-3                        # Start 3 drones in swarm 1"
    echo "  $0 both-swarms                     # Start both swarms"
    echo "  $0 logs px4-swarm-1-drone-1        # Show logs for drone 1"
    echo "  $0 status                          # Show container status"
    echo "  $0 stop                            # Stop all containers"
}

start_single_drone() {
    echo "🚁 Starting single drone (AirSim port 4561)..."
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 -d
}

start_swarm1_2() {
    echo "🚁 Starting Swarm 1 with 2 drones (AirSim ports 4561-4562)..."
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 px4-swarm-1-drone-2 -d
}

start_swarm1_3() {
    echo "🚁 Starting Swarm 1 with 3 drones (AirSim ports 4561-4563)..."
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 px4-swarm-1-drone-2 px4-swarm-1-drone-3 -d
}

start_swarm2_2() {
    echo "🚁 Starting Swarm 2 with 2 drones (AirSim ports 4571-4572)..."
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 up px4-swarm-2-drone-1 px4-swarm-2-drone-2 -d
}

start_both_swarms() {
    echo "🚁 Starting both swarms: 3 drones in swarm 1 + 2 drones in swarm 2..."
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 px4-swarm-1-drone-2 px4-swarm-1-drone-3 -d
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 up px4-swarm-2-drone-1 px4-swarm-2-drone-2 -d
}

show_status() {
    echo "📊 Container Status:"
    docker-compose -f "$COMPOSE_FILE" ps
}

show_logs() {
    if [ -z "$2" ]; then
        echo "❌ Please specify a drone name (e.g., px4-swarm-1-drone-1)"
        return 1
    fi
    echo "📋 Logs for $2:"
    docker logs "$2"
}

stop_all() {
    echo "🛑 Stopping all swarm containers..."
    docker-compose -f "$COMPOSE_FILE" down
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 down
}

clean_all() {
    echo "🧹 Cleaning up all swarm containers and networks..."
    docker-compose -f "$COMPOSE_FILE" down --volumes --remove-orphans
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 down --volumes --remove-orphans
}

# Main command dispatcher
case "$1" in
    single-drone)
        start_single_drone
        ;;
    swarm1-2)
        start_swarm1_2
        ;;
    swarm1-3)
        start_swarm1_3
        ;;
    swarm2-2)
        start_swarm2_2
        ;;
    both-swarms)
        start_both_swarms
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs "$@"
        ;;
    stop)
        stop_all
        ;;
    clean)
        clean_all
        ;;
    *)
        show_usage
        exit 1
        ;;
esac

if [ "$1" != "logs" ] && [ "$1" != "status" ] && [ "$1" != "stop" ] && [ "$1" != "clean" ]; then
    echo ""
    echo "✅ Started! Use the following commands:"
    echo "  ./start-swarms.sh status          # Check container status"
    echo "  ./start-swarms.sh logs [drone]    # View logs"
    echo "  ./start-swarms.sh stop            # Stop all containers"
    echo ""
    echo "🔗 AirSim Connection Ports:"
    if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "px4-swarm-1-drone-1.*Up"; then
        echo "  Drone 1 (Swarm 1): 4561"
    fi
    if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "px4-swarm-1-drone-2.*Up"; then
        echo "  Drone 2 (Swarm 1): 4562"
    fi
    if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "px4-swarm-1-drone-3.*Up"; then
        echo "  Drone 3 (Swarm 1): 4563"
    fi
    if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "px4-swarm-2-drone-1.*Up"; then
        echo "  Drone 1 (Swarm 2): 4571"
    fi
    if docker ps --format "table {{.Names}}\t{{.Status}}" | grep -q "px4-swarm-2-drone-2.*Up"; then
        echo "  Drone 2 (Swarm 2): 4572"
    fi
fi