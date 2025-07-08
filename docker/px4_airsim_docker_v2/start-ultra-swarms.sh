#!/bin/bash
# Ultra-Swarm PX4 Startup Helper Script
# Supports up to 27 drones: 9 drones per swarm across 3 swarms
# Enhanced version with GPS configuration and incremental testing support

set -e

COMPOSE_FILE="docker-compose.ultra-swarm.yml"

show_usage() {
    echo "🚁 Ultra-Swarm Management Script"
    echo "Supports up to 27 drones (9 per swarm × 3 swarms) with GPS configuration"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "🔧 Basic Commands:"
    echo "  single           Start 1 drone for testing (Swarm 1, Drone 1)"
    echo "  test-3           Start 3 drones in Swarm 1 (incremental test)"
    echo "  swarm1-full      Start all 9 drones in Swarm 1 (Blue Team)"
    echo "  swarm2-full      Start all 9 drones in Swarm 2 (Red Team)" 
    echo "  swarm3-full      Start all 9 drones in Swarm 3 (Green Team)"
    echo ""
    echo "🌟 Advanced Commands:"
    echo "  dual-swarms      Start Swarm 1 + Swarm 2 (18 drones total)"
    echo "  tri-swarms       Start Swarm 1 + Swarm 2 + Swarm 3 (27 drones total)"
    echo "  ultra-max        Start ALL 27 drones (maximum configuration)"
    echo ""
    echo "📊 Management Commands:"
    echo "  status           Show status of all containers"
    echo "  health           Run health checks on all active drones"
    echo "  logs [drone]     Show logs for specific drone"
    echo "  ports            Show active port allocations"
    echo "  gps-info         Show GPS configuration for all swarms"
    echo ""
    echo "🛑 Control Commands:"
    echo "  stop             Stop all running containers"
    echo "  stop-swarm1      Stop only Swarm 1 containers"
    echo "  stop-swarm2      Stop only Swarm 2 containers"
    echo "  stop-swarm3      Stop only Swarm 3 containers"
    echo "  clean            Stop and remove all containers/networks"
    echo ""
    echo "🔗 Port Allocation:"
    echo "  Swarm 1 (Blue):  AirSim TCP 4561-4569, QGC UDP 14550-14558, MAVLink UDP 18570-18578"
    echo "  Swarm 2 (Red):   AirSim TCP 4571-4579, QGC UDP 14560-14568, MAVLink UDP 18580-18588"
    echo "  Swarm 3 (Green): AirSim TCP 4581-4589, QGC UDP 14570-14578, MAVLink UDP 18590-18598"
    echo ""
    echo "📍 GPS Locations:"
    echo "  Swarm 1: Seattle area     (47.641468, -122.140165)"
    echo "  Swarm 2: Bellevue area    (47.642468, -122.139165)"
    echo "  Swarm 3: Redmond area     (47.643468, -122.138165)"
    echo ""
    echo "🚀 Examples:"
    echo "  $0 single                          # Start 1 drone for basic testing"
    echo "  $0 test-3                          # Start 3 drones for swarm testing"
    echo "  $0 swarm1-full                     # Start full Blue Team (9 drones)"
    echo "  $0 dual-swarms                     # Start Blue + Red Teams (18 drones)"
    echo "  $0 ultra-max                       # Start all 27 drones"
    echo "  $0 logs px4-swarm-1-drone-1        # View logs for specific drone"
    echo "  $0 status                          # Check all container status"
    echo ""
}

start_single() {
    echo "🚁 Starting single drone for testing..."
    echo "   Drone: Swarm 1, Drone 1 (AirSim port 4561)"
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 -d
    show_connection_info
}

start_test_3() {
    echo "🚁 Starting 3-drone test configuration..."
    echo "   Drones: Swarm 1, Drones 1-3 (AirSim ports 4561-4563)"
    docker-compose -f "$COMPOSE_FILE" up px4-swarm-1-drone-1 px4-swarm-1-drone-2 px4-swarm-1-drone-3 -d
    show_connection_info
}

start_swarm1_full() {
    echo "🔵 Starting Swarm 1 (Blue Team) - Full 9 drones..."
    echo "   Drones: Swarm 1, Drones 1-9 (AirSim ports 4561-4569)"
    docker-compose -f "$COMPOSE_FILE" up \
        px4-swarm-1-drone-1 px4-swarm-1-drone-2 px4-swarm-1-drone-3 \
        px4-swarm-1-drone-4 px4-swarm-1-drone-5 px4-swarm-1-drone-6 \
        px4-swarm-1-drone-7 px4-swarm-1-drone-8 px4-swarm-1-drone-9 -d
    show_connection_info
}

start_swarm2_full() {
    echo "🔴 Starting Swarm 2 (Red Team) - Full 9 drones..."
    echo "   Drones: Swarm 2, Drones 1-9 (AirSim ports 4571-4579)"
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 up \
        px4-swarm-2-drone-1 px4-swarm-2-drone-2 px4-swarm-2-drone-3 \
        px4-swarm-2-drone-4 px4-swarm-2-drone-5 px4-swarm-2-drone-6 \
        px4-swarm-2-drone-7 px4-swarm-2-drone-8 px4-swarm-2-drone-9 -d
    show_connection_info
}

start_swarm3_full() {
    echo "🟢 Starting Swarm 3 (Green Team) - Full 9 drones..."
    echo "   Drones: Swarm 3, Drones 1-9 (AirSim ports 4581-4589)"
    docker-compose -f "$COMPOSE_FILE" --profile swarm3 up \
        px4-swarm-3-drone-1 px4-swarm-3-drone-2 px4-swarm-3-drone-3 \
        px4-swarm-3-drone-4 px4-swarm-3-drone-5 px4-swarm-3-drone-6 \
        px4-swarm-3-drone-7 px4-swarm-3-drone-8 px4-swarm-3-drone-9 -d
    show_connection_info
}

start_dual_swarms() {
    echo "🔵🔴 Starting Dual-Swarms (Blue + Red Teams) - 18 drones total..."
    echo "   Swarm 1: AirSim ports 4561-4569"
    echo "   Swarm 2: AirSim ports 4571-4579"
    start_swarm1_full
    start_swarm2_full
}

start_tri_swarms() {
    echo "🔵🔴🟢 Starting Tri-Swarms (Blue + Red + Green Teams) - 27 drones total..."
    echo "   Swarm 1: AirSim ports 4561-4569"
    echo "   Swarm 2: AirSim ports 4571-4579" 
    echo "   Swarm 3: AirSim ports 4581-4589"
    start_swarm1_full
    start_swarm2_full
    start_swarm3_full
}

start_ultra_max() {
    echo "🚀 ULTRA-MAX: Starting ALL 27 drones across 3 swarms!"
    echo "   This is the maximum configuration supported."
    echo "   Please ensure your system has sufficient resources."
    echo ""
    read -p "Continue with ultra-max configuration? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        start_tri_swarms
    else
        echo "❌ Ultra-max startup cancelled."
        exit 0
    fi
}

show_status() {
    echo "📊 Ultra-Swarm Container Status:"
    echo ""
    docker-compose -f "$COMPOSE_FILE" ps
    echo ""
    
    # Count active drones per swarm
    local swarm1_count=$(docker-compose -f "$COMPOSE_FILE" ps | grep "px4-swarm-1" | grep "Up" | wc -l)
    local swarm2_count=$(docker-compose -f "$COMPOSE_FILE" ps | grep "px4-swarm-2" | grep "Up" | wc -l)
    local swarm3_count=$(docker-compose -f "$COMPOSE_FILE" ps | grep "px4-swarm-3" | grep "Up" | wc -l)
    local total_count=$((swarm1_count + swarm2_count + swarm3_count))
    
    echo "📈 Active Drone Count:"
    echo "   🔵 Swarm 1 (Blue):  $swarm1_count/9 drones"
    echo "   🔴 Swarm 2 (Red):   $swarm2_count/9 drones"
    echo "   🟢 Swarm 3 (Green): $swarm3_count/9 drones"
    echo "   🚁 Total Active:    $total_count/27 drones"
}

run_health_checks() {
    echo "🔍 Running health checks on all active drones..."
    echo ""
    
    for container in $(docker ps --format "{{.Names}}" | grep "px4-swarm"); do
        echo "Checking $container..."
        if docker exec "$container" /Scripts/ultra_swarm_health_check.sh >/dev/null 2>&1; then
            echo "   ✅ $container: Healthy"
        else
            echo "   ❌ $container: Health check failed"
        fi
    done
}

show_logs() {
    if [ -z "$2" ]; then
        echo "❌ Please specify a drone name."
        echo "Examples:"
        echo "  $0 logs px4-swarm-1-drone-1"
        echo "  $0 logs px4-swarm-2-drone-5"
        echo "  $0 logs px4-swarm-3-drone-9"
        return 1
    fi
    echo "📋 Logs for $2:"
    docker logs "$2"
}

show_ports() {
    echo "🔗 Active Port Allocations:"
    echo ""
    
    for i in {1..9}; do
        container="px4-swarm-1-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4560 + i))
            echo "   🔵 Swarm 1, Drone $i: AirSim TCP $port"
        fi
    done
    
    for i in {1..9}; do
        container="px4-swarm-2-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4570 + i))
            echo "   🔴 Swarm 2, Drone $i: AirSim TCP $port"
        fi
    done
    
    for i in {1..9}; do
        container="px4-swarm-3-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4580 + i))
            echo "   🟢 Swarm 3, Drone $i: AirSim TCP $port"
        fi
    done
}

show_gps_info() {
    echo "📍 GPS Configuration Information:"
    echo ""
    if [ -f "config/gps_locations.json" ]; then
        echo "🔵 Swarm 1 (Blue Team): Seattle area"
        echo "   Home: 47.641468, -122.140165"
        echo ""
        echo "🔴 Swarm 2 (Red Team): Bellevue area"
        echo "   Home: 47.642468, -122.139165"
        echo ""
        echo "🟢 Swarm 3 (Green Team): Redmond area"
        echo "   Home: 47.643468, -122.138165"
        echo ""
        echo "ℹ️  Each drone is offset by ~11m within its swarm formation"
        echo "ℹ️  GPS parameters are automatically configured to prevent arming errors"
    else
        echo "❌ GPS configuration file not found at config/gps_locations.json"
    fi
}

stop_all() {
    echo "🛑 Stopping all ultra-swarm containers..."
    docker-compose -f "$COMPOSE_FILE" down
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 down
    docker-compose -f "$COMPOSE_FILE" --profile swarm3 down
}

stop_swarm1() {
    echo "🛑 Stopping Swarm 1 (Blue Team) containers..."
    docker-compose -f "$COMPOSE_FILE" stop $(docker-compose -f "$COMPOSE_FILE" config --services | grep "px4-swarm-1")
}

stop_swarm2() {
    echo "🛑 Stopping Swarm 2 (Red Team) containers..."
    docker-compose -f "$COMPOSE_FILE" --profile swarm2 stop $(docker-compose -f "$COMPOSE_FILE" --profile swarm2 config --services | grep "px4-swarm-2")
}

stop_swarm3() {
    echo "🛑 Stopping Swarm 3 (Green Team) containers..."
    docker-compose -f "$COMPOSE_FILE" --profile swarm3 stop $(docker-compose -f "$COMPOSE_FILE" --profile swarm3 config --services | grep "px4-swarm-3")
}

clean_all() {
    echo "🧹 Cleaning up all ultra-swarm containers and networks..."
    echo "   This will remove all containers, volumes, and networks."
    echo ""
    read -p "Continue with cleanup? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker-compose -f "$COMPOSE_FILE" down --volumes --remove-orphans
        docker-compose -f "$COMPOSE_FILE" --profile swarm2 down --volumes --remove-orphans
        docker-compose -f "$COMPOSE_FILE" --profile swarm3 down --volumes --remove-orphans
        echo "✅ Cleanup completed."
    else
        echo "❌ Cleanup cancelled."
    fi
}

show_connection_info() {
    echo ""
    echo "✅ Ultra-swarm containers started successfully!"
    echo ""
    echo "🔗 AirSim Connection Ports:"
    
    # Show active connections
    for i in {1..9}; do
        container="px4-swarm-1-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4560 + i))
            echo "   🔵 Swarm 1, Drone $i: localhost:$port"
        fi
    done
    
    for i in {1..9}; do
        container="px4-swarm-2-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4570 + i))
            echo "   🔴 Swarm 2, Drone $i: localhost:$port"
        fi
    done
    
    for i in {1..9}; do
        container="px4-swarm-3-drone-$i"
        if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
            port=$((4580 + i))
            echo "   🟢 Swarm 3, Drone $i: localhost:$port"
        fi
    done
    
    echo ""
    echo "📋 Management Commands:"
    echo "   $0 status     # Check container status"
    echo "   $0 health     # Run health checks"
    echo "   $0 ports      # Show port allocations"
    echo "   $0 stop       # Stop all containers"
}

# Main command dispatcher
case "$1" in
    single)
        start_single
        ;;
    test-3)
        start_test_3
        ;;
    swarm1-full)
        start_swarm1_full
        ;;
    swarm2-full)
        start_swarm2_full
        ;;
    swarm3-full)
        start_swarm3_full
        ;;
    dual-swarms)
        start_dual_swarms
        ;;
    tri-swarms)
        start_tri_swarms
        ;;
    ultra-max)
        start_ultra_max
        ;;
    status)
        show_status
        ;;
    health)
        run_health_checks
        ;;
    logs)
        show_logs "$@"
        ;;
    ports)
        show_ports
        ;;
    gps-info)
        show_gps_info
        ;;
    stop)
        stop_all
        ;;
    stop-swarm1)
        stop_swarm1
        ;;
    stop-swarm2)
        stop_swarm2
        ;;
    stop-swarm3)
        stop_swarm3
        ;;
    clean)
        clean_all
        ;;
    *)
        show_usage
        exit 1
        ;;
esac

echo ""