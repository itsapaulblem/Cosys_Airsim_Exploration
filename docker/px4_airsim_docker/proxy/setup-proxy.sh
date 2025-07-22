#!/bin/bash
# MAVLink Proxy Setup Script
# Configures host-level Nginx proxy for external MAVLink connectivity

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

# Check system requirements
check_requirements() {
    log "Checking system requirements for MAVLink proxy..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        error "Docker is required but not installed"
    fi
    
    # Check Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        error "Docker Compose is required but not installed"
    fi
    
    # Check if running on Docker host (not inside container)
    if [ -f /.dockerenv ]; then
        warn "Running inside Docker container. This script should run on the Docker host."
        read -p "Continue anyway? (y/n): " continue_anyway
        if [[ ! $continue_anyway =~ ^[Yy]$ ]]; then
            exit 0
        fi
    fi
    
    # Check for required ports
    for port in 14550 14540 8080; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            warn "Port $port is already in use. Proxy may fail to start."
        fi
    done
    
    log "Requirements check completed"
}

# Install Nginx with stream module (if needed)
install_nginx_stream() {
    log "Checking Nginx stream module availability..."
    
    # Check if system has nginx with stream module
    if command -v nginx &> /dev/null; then
        if nginx -V 2>&1 | grep -q "stream"; then
            log "Nginx with stream module is available"
            return 0
        else
            warn "Nginx installed but missing stream module"
        fi
    fi
    
    # Offer to install nginx with stream module
    echo ""
    echo -e "${BLUE}The MAVLink proxy requires Nginx with stream module.${NC}"
    echo -e "${BLUE}You can either:${NC}"
    echo "1. Use Docker-based proxy (recommended)"
    echo "2. Install system Nginx with stream module"
    echo ""
    
    read -p "Use Docker-based proxy? (y/n): " use_docker
    if [[ $use_docker =~ ^[Yy]$ ]]; then
        log "Will use Docker-based Nginx proxy"
        return 0
    fi
    
    # Try to install nginx with stream module
    if command -v apt-get &> /dev/null; then
        log "Installing Nginx with stream module (Ubuntu/Debian)..."
        sudo apt-get update
        sudo apt-get install -y nginx nginx-module-stream
    elif command -v yum &> /dev/null; then
        log "Installing Nginx with stream module (CentOS/RHEL)..."
        sudo yum install -y nginx nginx-mod-stream
    else
        error "Cannot automatically install Nginx. Please install nginx with stream module manually."
    fi
}

# Configure host firewall
configure_firewall() {
    log "Configuring firewall for MAVLink proxy..."
    
    # UFW configuration
    if command -v ufw &> /dev/null; then
        log "Configuring UFW firewall..."
        
        # Allow MAVLink UDP ports
        sudo ufw allow 14540:14570/udp comment "MAVLink UDP Proxy" || true
        
        # Allow management interface
        sudo ufw allow 8080/tcp comment "MAVLink Proxy Management" || true
        
        log "UFW firewall configured"
        
    # iptables configuration
    elif command -v iptables &> /dev/null; then
        log "Configuring iptables firewall..."
        
        # Allow MAVLink UDP ports
        sudo iptables -A INPUT -p udp --dport 14540:14570 -j ACCEPT || true
        
        # Allow management interface
        sudo iptables -A INPUT -p tcp --dport 8080 -j ACCEPT || true
        
        # Save iptables rules (if possible)
        if command -v iptables-save &> /dev/null; then
            sudo iptables-save > /tmp/iptables-proxy.rules 2>/dev/null || true
        fi
        
        log "iptables firewall configured"
    else
        warn "No firewall utility found. Please configure firewall manually:"
        echo "  - Allow UDP ports 14540-14570"
        echo "  - Allow TCP port 8080"
    fi
}

# Create proxy configuration directories
setup_directories() {
    log "Setting up proxy directories..."
    
    # Create necessary directories
    mkdir -p "$SCRIPT_DIR/dynamic-config"
    mkdir -p "$SCRIPT_DIR/logs"
    mkdir -p "$SCRIPT_DIR/scripts"
    
    # Set proper permissions
    chmod 755 "$SCRIPT_DIR/dynamic-config"
    chmod 755 "$SCRIPT_DIR/logs"
    chmod 755 "$SCRIPT_DIR/scripts"
    
    log "Proxy directories created"
}

# Generate enhanced MAVLink Router configuration script
create_enhanced_config_generator() {
    log "Creating enhanced MAVLink Router configuration generator..."
    
    cat > "$SCRIPT_DIR/scripts/generate_mavlink_config_enhanced.py" << 'EOF'
#!/usr/bin/env python3
"""
Enhanced MAVLink Router Configuration Generator
Generates configurations for external IP connectivity via proxy
"""

import argparse
import os
import sys
from typing import List, Dict

def generate_enhanced_config(
    container_ip: str,
    proxy_ip: str, 
    enable_external: bool = True,
    drone_instance: int = 1
) -> str:
    """Generate enhanced MAVLink Router configuration"""
    
    config = f"""# Enhanced MAVLink Router Configuration
# Generated for external IP connectivity via proxy
# Container IP: {container_ip}, Proxy IP: {proxy_ip}

[General]
TcpServerPort = 0
ReportStats = true
DebugLogLevel = info

# ================================
# INTERNAL ENDPOINTS (Container Network)
# ================================

# PX4 SITL connection (container internal)
[UdpEndpoint PX4_SITL]
Mode = Normal
Address = 127.0.0.1
Port = {14580 + drone_instance}
Description = "PX4 SITL local connection"

# ================================
# PROXY ENDPOINTS (Host Network via Proxy)
# ================================

# Primary QGroundControl endpoint (via proxy)
[UdpEndpoint QGC_Proxy]
Mode = Normal
Address = {proxy_ip}
Port = {14550 + drone_instance - 1}
Description = "QGroundControl via proxy"

# API endpoint (via proxy)
[UdpEndpoint API_Proxy]
Mode = Normal
Address = {proxy_ip}
Port = 14540
Description = "API access via proxy"

"""

    if enable_external:
        config += f"""
# ================================
# EXTERNAL CLIENT ENDPOINTS (Active Connections)
# ================================

# Note: These endpoints actively connect TO external clients
# They will be populated dynamically via registration API

# Template for external client connections
# [UdpEndpoint External_Client_1]
# Mode = Normal  
# Address = EXTERNAL_CLIENT_IP
# Port = EXTERNAL_CLIENT_PORT
# Description = "External client connection"

"""

    return config

def main():
    parser = argparse.ArgumentParser(description="Enhanced MAVLink Router Config Generator")
    parser.add_argument("--container-ip", default="172.20.0.11", help="Container IP address")
    parser.add_argument("--proxy-ip", default="172.20.0.200", help="Proxy IP address")
    parser.add_argument("--drone-instance", type=int, default=1, help="Drone instance number")
    parser.add_argument("--external-endpoints", action="store_true", help="Enable external endpoints")
    parser.add_argument("--output", default="/px4_workspace/mavlink-router-config/enhanced-router.conf", 
                       help="Output configuration file")
    
    args = parser.parse_args()
    
    # Generate configuration
    config = generate_enhanced_config(
        args.container_ip,
        args.proxy_ip,
        args.external_endpoints,
        args.drone_instance
    )
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    
    # Write configuration
    with open(args.output, 'w') as f:
        f.write(config)
    
    print(f"Enhanced MAVLink Router configuration generated: {args.output}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
EOF

    chmod +x "$SCRIPT_DIR/scripts/generate_mavlink_config_enhanced.py"
    
    log "Enhanced configuration generator created"
}

# Create deployment script
create_deployment_script() {
    log "Creating proxy deployment script..."
    
    cat > "$SCRIPT_DIR/deploy-proxy.sh" << 'EOF'
#!/bin/bash
# Deploy MAVLink Proxy for External Connectivity

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

log "🌐 Deploying MAVLink Proxy for external connectivity..."

# Check if PX4 containers are running
if ! docker ps | grep -q "px4-"; then
    log "⚠️  No PX4 containers found. Starting basic containers first..."
    cd .. && docker-compose -f docker-compose-slim.yml up -d
    sleep 10
fi

# Start proxy services
log "Starting MAVLink proxy services..."
docker-compose -f docker-compose-proxy.yml up -d

# Wait for services to be ready
log "Waiting for proxy to initialize..."
sleep 15

# Verify proxy health
log "Verifying proxy health..."
if curl -sf http://localhost:8080/health | grep -q "Healthy"; then
    log "✅ MAVLink proxy is healthy and running"
else
    log "❌ MAVLink proxy health check failed"
    exit 1
fi

# Test UDP endpoints
log "Testing UDP endpoint accessibility..."
if nc -u -z -w 3 localhost 14550; then
    log "✅ UDP port 14550 (QGC) is accessible"
else
    log "❌ UDP port 14550 is not accessible"
fi

if nc -u -z -w 3 localhost 14540; then
    log "✅ UDP port 14540 (API) is accessible"
else
    log "❌ UDP port 14540 is not accessible"
fi

# Display connection information
log ""
log "🎯 MAVLink Proxy deployed successfully!"
log ""
log "External Connection Endpoints:"
log "  QGroundControl: YOUR_HOST_IP:14550"
log "  API Access:     YOUR_HOST_IP:14540"
log "  Drone 2 QGC:    YOUR_HOST_IP:14551"
log "  Drone 3 QGC:    YOUR_HOST_IP:14552"
log ""
log "Management Interface:"
log "  Health:         YOUR_HOST_IP:8080/health"
log "  Statistics:     YOUR_HOST_IP:8080/stats"
log ""
log "Next Steps:"
log "1. Test from external client: nc -u YOUR_HOST_IP 14550"
log "2. Connect QGroundControl to YOUR_HOST_IP:14550"
log "3. Use MAVSDK with udp://YOUR_HOST_IP:14540"
log ""

# Get host IP for display
HOST_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "YOUR_HOST_IP")
log "Your host IP appears to be: $HOST_IP"
log "Replace YOUR_HOST_IP with: $HOST_IP"
EOF

    chmod +x "$SCRIPT_DIR/deploy-proxy.sh"
    
    log "Deployment script created"
}

# Create test script for external connectivity
create_test_script() {
    log "Creating external connectivity test script..."
    
    cat > "$SCRIPT_DIR/test-external-connectivity.sh" << 'EOF'
#!/bin/bash
# Test External MAVLink Connectivity

set -e

HOST_IP="${1:-localhost}"
TIMEOUT=5

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

test_endpoint() {
    local desc="$1"
    local host="$2" 
    local port="$3"
    local protocol="$4"
    
    echo -n "Testing $desc ($host:$port)... "
    
    if [ "$protocol" = "udp" ]; then
        if timeout $TIMEOUT nc -u -z "$host" "$port" 2>/dev/null; then
            echo "✅ PASS"
            return 0
        else
            echo "❌ FAIL"
            return 1
        fi
    else
        if timeout $TIMEOUT nc -z "$host" "$port" 2>/dev/null; then
            echo "✅ PASS"
            return 0
        else
            echo "❌ FAIL"
            return 1
        fi
    fi
}

log "🧪 Testing MAVLink External Connectivity"
log "Host: $HOST_IP"
log "Timeout: ${TIMEOUT}s"
log ""

# Test basic connectivity
test_endpoint "Host Reachability" "$HOST_IP" "22" "tcp" || true

# Test proxy management
test_endpoint "Proxy Health Check" "$HOST_IP" "8080" "tcp"

# Test MAVLink UDP endpoints
test_endpoint "QGroundControl Port" "$HOST_IP" "14550" "udp"
test_endpoint "API Access Port" "$HOST_IP" "14540" "udp"
test_endpoint "Drone 2 QGC Port" "$HOST_IP" "14551" "udp"
test_endpoint "Drone 3 QGC Port" "$HOST_IP" "14552" "udp"

# Test health endpoint content
log ""
log "Testing proxy health endpoint content..."
if curl -sf "http://$HOST_IP:8080/health" 2>/dev/null; then
    echo "✅ Health endpoint returns valid response"
else
    echo "❌ Health endpoint failed or returned invalid response"
fi

# Test statistics endpoint
log "Testing proxy statistics endpoint..."
if curl -sf "http://$HOST_IP:8080/stats" 2>/dev/null | grep -q "status"; then
    echo "✅ Statistics endpoint working"
else
    echo "❌ Statistics endpoint failed"
fi

log ""
log "🎯 External connectivity test completed"
log ""
log "If tests pass, you can now:"
log "1. Connect QGroundControl to $HOST_IP:14550"
log "2. Use MAVSDK: udp://$HOST_IP:14540"
log "3. Access multiple drones via ports 14551, 14552, etc."
EOF

    chmod +x "$SCRIPT_DIR/test-external-connectivity.sh"
    
    log "Test script created"
}

# Main setup function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  MAVLink Proxy Setup                   ${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    read -p "Set up MAVLink proxy for external connectivity? (y/n): " confirm
    if [[ ! $confirm =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 0
    fi
    
    # Run setup steps
    check_requirements
    install_nginx_stream
    configure_firewall
    setup_directories
    create_enhanced_config_generator
    create_deployment_script
    create_test_script
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  Proxy setup completed!                ${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    echo "1. Deploy proxy: cd $SCRIPT_DIR && ./deploy-proxy.sh"
    echo "2. Test connectivity: ./test-external-connectivity.sh YOUR_HOST_IP"
    echo "3. Connect external clients to YOUR_HOST_IP:14550"
    echo ""
    echo -e "${YELLOW}Important:${NC}"
    echo "- Replace YOUR_HOST_IP with your actual public IP address"
    echo "- Ensure firewall allows UDP ports 14540-14570"
    echo "- Test from a different network to verify external access"
    echo ""
}

main "$@"