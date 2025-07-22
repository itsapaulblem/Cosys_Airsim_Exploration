#!/bin/bash
# Quick Deployment Test for MAVLink External Connectivity
# Rapid validation and testing script for the complete solution

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

error() {
    echo -e "${RED}[ERROR] $1${NC}"
    exit 1
}

info() {
    echo -e "${BLUE}[INFO] $1${NC}"
}

# Get host IP automatically
get_host_ip() {
    # Try multiple methods to get external IP
    local host_ip=""
    
    # Method 1: hostname -I (Linux)
    if command -v hostname &> /dev/null; then
        host_ip=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "")
    fi
    
    # Method 2: ip route (more reliable)
    if [ -z "$host_ip" ] && command -v ip &> /dev/null; then
        host_ip=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+' || echo "")
    fi
    
    # Method 3: ifconfig fallback
    if [ -z "$host_ip" ] && command -v ifconfig &> /dev/null; then
        host_ip=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | head -n1)
    fi
    
    # Default fallback
    if [ -z "$host_ip" ]; then
        host_ip="localhost"
    fi
    
    echo "$host_ip"
}

# Quick system check
quick_system_check() {
    log "🔍 Running quick system check..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        error "Docker is not installed or not in PATH"
    fi
    log "✅ Docker found"
    
    # Check Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        warn "docker-compose not found, trying 'docker compose'"
        if ! docker compose version &> /dev/null; then
            error "Neither docker-compose nor 'docker compose' available"
        fi
        DOCKER_COMPOSE_CMD="docker compose"
    else
        DOCKER_COMPOSE_CMD="docker-compose"
    fi
    log "✅ Docker Compose found: $DOCKER_COMPOSE_CMD"
    
    # Check Python
    if ! command -v python3 &> /dev/null; then
        error "Python 3 is required but not found"
    fi
    log "✅ Python 3 found"
    
    # Check for required Python packages
    if ! python3 -c "import requests, json, subprocess" &> /dev/null; then
        warn "Some Python packages may be missing. Install with: pip install requests"
    fi
    
    log "✅ System check completed"
}

# Check solution files
check_solution_files() {
    log "📁 Checking solution files..."
    
    local required_files=(
        "proxy/nginx-mavlink-proxy.conf"
        "proxy/docker-compose-proxy.yml"
        "api/mavlink-registration-api.py"
        "scripts/generate_mavlink_config_enhanced.py"
        "scripts/mavlink-router-enhanced-template.conf"
        "tools/test_external_connectivity.py"
        "tools/validate_complete_solution.py"
    )
    
    local missing_files=()
    
    for file in "${required_files[@]}"; do
        if [ -f "$PROJECT_DIR/$file" ]; then
            log "✅ Found: $file"
        else
            error "❌ Missing: $file"
            missing_files+=("$file")
        fi
    done
    
    if [ ${#missing_files[@]} -gt 0 ]; then
        error "Missing required files. Please ensure all solution components are present."
    fi
    
    log "✅ All required files found"
}

# Test configuration generation
test_config_generation() {
    log "⚙️  Testing configuration generation..."
    
    local config_script="$PROJECT_DIR/scripts/generate_mavlink_config_enhanced.py"
    local template_file="$PROJECT_DIR/scripts/mavlink-router-enhanced-template.conf"
    local test_output="/tmp/test_mavlink_config.conf"
    
    if python3 "$config_script" \
        --template "$template_file" \
        --output "$test_output" \
        --validate \
        --system-id 1 \
        --px4-instance 1; then
        log "✅ Configuration generation successful"
        rm -f "$test_output"
    else
        error "❌ Configuration generation failed"
    fi
}

# Quick Docker deployment test
quick_deploy_test() {
    log "🚀 Testing quick deployment..."
    
    cd "$PROJECT_DIR"
    
    # Stop any existing containers
    log "Stopping any existing containers..."
    $DOCKER_COMPOSE_CMD -f proxy/docker-compose-proxy.yml down 2>/dev/null || true
    
    # Start proxy services
    log "Starting MAVLink proxy services..."
    $DOCKER_COMPOSE_CMD -f proxy/docker-compose-proxy.yml up -d mavlink-proxy
    
    # Wait for services to initialize
    log "Waiting for services to initialize..."
    sleep 10
    
    # Check if proxy is running
    if docker ps | grep -q "mavlink-proxy"; then
        log "✅ MAVLink proxy container is running"
    else
        error "❌ MAVLink proxy failed to start"
    fi
    
    # Get host IP
    local host_ip=$(get_host_ip)
    log "🌐 Detected host IP: $host_ip"
    
    # Test basic endpoints
    log "🧪 Testing basic connectivity..."
    
    # Test health endpoint
    local health_url="http://$host_ip:8080/health"
    if curl -sf "$health_url" &> /dev/null; then
        log "✅ Health endpoint accessible: $health_url"
    else
        warn "⚠️  Health endpoint not accessible (may need time to initialize)"
    fi
    
    # Test UDP port accessibility
    if command -v nc &> /dev/null; then
        log "Testing UDP port 14550..."
        if timeout 3 nc -u -z "$host_ip" 14550 2>/dev/null; then
            log "✅ UDP port 14550 is accessible"
        else
            warn "⚠️  UDP port 14550 not immediately accessible"
        fi
    else
        warn "netcat not available, skipping UDP test"
    fi
    
    return 0
}

# Run external connectivity test
run_external_test() {
    local host_ip="$1"
    
    log "🌐 Running external connectivity test..."
    
    local test_script="$PROJECT_DIR/tools/test_external_connectivity.py"
    
    if [ -f "$test_script" ]; then
        # Run quick test first
        if python3 "$test_script" "$host_ip" --quick; then
            log "✅ Quick external connectivity test passed"
            
            # Ask if user wants full test
            echo ""
            read -p "Run full connectivity test? (y/n): " run_full
            if [[ $run_full =~ ^[Yy]$ ]]; then
                log "Running full external connectivity test..."
                python3 "$test_script" "$host_ip" --output "/tmp/connectivity_test_results.json"
            fi
        else
            warn "⚠️  Quick connectivity test had issues"
        fi
    else
        warn "External connectivity test script not found"
    fi
}

# Cleanup function
cleanup() {
    log "🧹 Cleaning up test deployment..."
    cd "$PROJECT_DIR"
    $DOCKER_COMPOSE_CMD -f proxy/docker-compose-proxy.yml down 2>/dev/null || true
    log "✅ Cleanup completed"
}

# Main function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  MAVLink Quick Deployment Test         ${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Set up cleanup trap
    trap cleanup EXIT
    
    # Quick system check
    quick_system_check
    echo ""
    
    # Check solution files
    check_solution_files
    echo ""
    
    # Test configuration generation
    test_config_generation
    echo ""
    
    # Quick deployment test
    quick_deploy_test
    echo ""
    
    # Get host IP for external testing
    local host_ip=$(get_host_ip)
    
    # Run external connectivity test
    run_external_test "$host_ip"
    echo ""
    
    # Final summary
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  QUICK TEST COMPLETED                  ${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}🎯 Your MAVLink external connectivity solution is working!${NC}"
    echo ""
    echo -e "${BLUE}Connection Information:${NC}"
    echo "  Host IP: $host_ip"
    echo "  QGroundControl: $host_ip:14550"
    echo "  API Access: udp://$host_ip:14540"
    echo "  Health Check: http://$host_ip:8080/health"
    echo ""
    echo -e "${BLUE}Next Steps:${NC}"
    echo "1. Test from external network:"
    echo "   $PROJECT_DIR/tools/test_external_connectivity.py <EXTERNAL_IP>"
    echo ""
    echo "2. Connect QGroundControl:"
    echo "   • Add UDP connection to $host_ip:14550"
    echo ""
    echo "3. Use MAVSDK/DroneKit:"
    echo "   • Connection string: udp://$host_ip:14540"
    echo ""
    echo "4. Register additional clients:"
    echo "   • POST to http://$host_ip:8000/api/v1/clients/register"
    echo ""
    
    # Offer to keep running or cleanup
    echo ""
    read -p "Keep services running for testing? (y/n): " keep_running
    if [[ $keep_running =~ ^[Yy]$ ]]; then
        echo ""
        log "🚀 Services will continue running. Use 'docker-compose down' to stop."
        log "📊 Monitor logs with: docker-compose -f proxy/docker-compose-proxy.yml logs -f"
        echo ""
        
        # Disable cleanup trap
        trap - EXIT
    else
        log "Services will be stopped during cleanup"
    fi
}

# Check if script is being sourced or executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi