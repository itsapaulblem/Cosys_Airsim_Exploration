#!/bin/bash

# Test script for AirSim-PX4 bridge network configuration
# This script verifies that the bridge network setup works correctly

echo "🔧 AirSim-PX4 Bridge Network Test Script"
echo "======================================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test configuration
TEST_DRONE_COUNT=3
COMPOSE_FILE="docker-compose-bridge.yml"
AIRSIM_HOST="host.docker.internal"
AIRSIM_PORT="41451"

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a service is responding
check_service() {
    local host=$1
    local port=$2
    local protocol=${3:-tcp}
    local timeout=${4:-5}
    
    if [ "$protocol" == "tcp" ]; then
        timeout $timeout bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null
    elif [ "$protocol" == "udp" ]; then
        timeout $timeout bash -c "echo | nc -u -w1 $host $port" 2>/dev/null
    fi
    
    return $?
}

# Function to test TCP HIL connection (sensor data)
test_tcp_hil_connection() {
    print_status "Testing TCP HIL connections (sensor data)..."
    
    local success_count=0
    local total_count=$TEST_DRONE_COUNT
    
    for i in $(seq 1 $TEST_DRONE_COUNT); do
        local tcp_port=$((4560 + i))
        print_status "Testing TCP HIL for drone $i on port $tcp_port..."
        
        if check_service "127.0.0.1" $tcp_port "tcp" 3; then
            print_success "✅ TCP HIL connection for drone $i: WORKING"
            ((success_count++))
        else
            print_error "❌ TCP HIL connection for drone $i: FAILED"
        fi
    done
    
    echo ""
    print_status "TCP HIL Test Results: $success_count/$total_count connections successful"
    return $((total_count - success_count))
}

# Function to test UDP GCS connection (control commands)
test_udp_gcs_connection() {
    print_status "Testing UDP GCS connections (control commands)..."
    
    local success_count=0
    local total_count=$((TEST_DRONE_COUNT * 2))  # Local and remote ports
    
    for i in $(seq 1 $TEST_DRONE_COUNT); do
        local control_port_local=$((14540 + i))
        local control_port_remote=$((14580 + i))
        
        print_status "Testing UDP GCS for drone $i..."
        
        # Test local control port
        if check_service "127.0.0.1" $control_port_local "udp" 3; then
            print_success "✅ UDP Control Local (port $control_port_local): WORKING"
            ((success_count++))
        else
            print_error "❌ UDP Control Local (port $control_port_local): FAILED"
        fi
        
        # Test remote control port
        if check_service "127.0.0.1" $control_port_remote "udp" 3; then
            print_success "✅ UDP Control Remote (port $control_port_remote): WORKING"
            ((success_count++))
        else
            print_error "❌ UDP Control Remote (port $control_port_remote): FAILED"
        fi
    done
    
    echo ""
    print_status "UDP GCS Test Results: $success_count/$total_count connections successful"
    return $((total_count - success_count))
}

# Function to check Docker containers
check_containers() {
    print_status "Checking Docker container status..."
    
    local running_containers=$(docker ps --format "table {{.Names}}\t{{.Status}}" | grep "px4-bridge-drone")
    
    if [ -z "$running_containers" ]; then
        print_error "❌ No PX4 bridge containers are running"
        return 1
    fi
    
    print_success "✅ Running PX4 bridge containers:"
    echo "$running_containers"
    echo ""
    return 0
}

# Function to check AirSim connectivity
check_airsim_connection() {
    print_status "Testing AirSim API connection..."
    
    if check_service "127.0.0.1" $AIRSIM_PORT "tcp" 5; then
        print_success "✅ AirSim API connection: WORKING"
        return 0
    else
        print_error "❌ AirSim API connection: FAILED"
        print_warning "Make sure AirSim is running with the updated settings.json"
        return 1
    fi
}

# Function to display network configuration
show_network_config() {
    print_status "Bridge Network Configuration:"
    echo ""
    echo "Docker Network: px4_network (172.20.0.0/16)"
    echo "AirSim Connection: host.docker.internal"
    echo ""
    echo "Port Mappings:"
    for i in $(seq 1 $TEST_DRONE_COUNT); do
        local tcp_port=$((4560 + i))
        local control_port_local=$((14540 + i))
        local control_port_remote=$((14580 + i))
        echo "  Drone $i:"
        echo "    TCP HIL (sensor data): 127.0.0.1:$tcp_port -> container:$tcp_port"
        echo "    UDP Control Local: 127.0.0.1:$control_port_local -> container:$control_port_local"
        echo "    UDP Control Remote: 127.0.0.1:$control_port_remote -> container:$control_port_remote"
    done
    echo ""
}

# Function to run full test suite
run_full_test() {
    print_status "Starting comprehensive bridge network test..."
    echo ""
    
    # Show configuration
    show_network_config
    
    # Check prerequisites
    print_status "Checking prerequisites..."
    
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose is not installed or not in PATH"
        exit 1
    fi
    
    if [ ! -f "$COMPOSE_FILE" ]; then
        print_error "Docker Compose file not found: $COMPOSE_FILE"
        exit 1
    fi
    
    print_success "✅ Prerequisites check passed"
    echo ""
    
    # Check container status
    if ! check_containers; then
        print_warning "Starting containers first..."
        docker-compose -f $COMPOSE_FILE up -d px4-bridge-drone-{1..3}
        sleep 10
        check_containers
    fi
    
    # Test connections
    local test_results=0
    
    # Test AirSim connection
    if ! check_airsim_connection; then
        ((test_results++))
    fi
    echo ""
    
    # Test TCP HIL connections
    if ! test_tcp_hil_connection; then
        ((test_results++))
    fi
    echo ""
    
    # Test UDP GCS connections  
    if ! test_udp_gcs_connection; then
        ((test_results++))
    fi
    echo ""
    
    # Summary
    print_status "Test Summary:"
    if [ $test_results -eq 0 ]; then
        print_success "🎉 ALL TESTS PASSED! Bridge network is working correctly."
    else
        print_error "❌ Some tests failed. Check the output above for details."
    fi
    
    return $test_results
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  --tcp-test      Test only TCP HIL connections"
    echo "  --udp-test      Test only UDP GCS connections"
    echo "  --config        Show network configuration"
    echo "  --containers    Check container status"
    echo "  --airsim        Test AirSim connection"
    echo "  --help          Show this help message"
    echo ""
    echo "Default: Run full test suite"
}

# Main execution
case "${1:-full}" in
    --tcp-test)
        test_tcp_hil_connection
        ;;
    --udp-test)
        test_udp_gcs_connection
        ;;
    --config)
        show_network_config
        ;;
    --containers)
        check_containers
        ;;
    --airsim)
        check_airsim_connection
        ;;
    --help)
        show_usage
        ;;
    full)
        run_full_test
        ;;
    *)
        print_error "Unknown option: $1"
        show_usage
        exit 1
        ;;
esac

exit $?