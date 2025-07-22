#!/bin/bash
# PX4 Remote Server Security Setup Script
# Configures WireGuard VPN and HTTP authentication for secure remote access

set -e

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
VPN_DIR="$SCRIPT_DIR/wireguard"
AUTH_DIR="$SCRIPT_DIR/auth"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
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

# Check requirements
check_requirements() {
    log "Checking system requirements..."
    
    # Check if running as root
    if [[ $EUID -eq 0 ]]; then
        warn "Running as root. Consider using a non-root user with sudo privileges."
    fi
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        error "Docker is required but not installed. Please install Docker first."
    fi
    
    # Check Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        error "Docker Compose is required but not installed. Please install Docker Compose first."
    fi
    
    # Check OpenSSL
    if ! command -v openssl &> /dev/null; then
        error "OpenSSL is required but not installed. Please install OpenSSL first."
    fi
    
    # Check htpasswd (Apache utils)
    if ! command -v htpasswd &> /dev/null; then
        warn "htpasswd not found. Installing apache2-utils..."
        if command -v apt-get &> /dev/null; then
            sudo apt-get update && sudo apt-get install -y apache2-utils
        elif command -v yum &> /dev/null; then
            sudo yum install -y httpd-tools
        else
            error "Cannot install htpasswd. Please install apache2-utils or httpd-tools manually."
        fi
    fi
    
    log "Requirements check completed successfully."
}

# Generate SSL certificates
generate_ssl_certificates() {
    log "Generating SSL certificates for authentication service..."
    
    mkdir -p "$AUTH_DIR/ssl"
    
    # Generate private key
    openssl genpkey -algorithm RSA -out "$AUTH_DIR/ssl/server.key" -pkcs8 -pass pass:temp 2>/dev/null
    openssl rsa -in "$AUTH_DIR/ssl/server.key" -out "$AUTH_DIR/ssl/server.key" -passin pass:temp 2>/dev/null
    
    # Generate certificate signing request
    openssl req -new -key "$AUTH_DIR/ssl/server.key" -out "$AUTH_DIR/ssl/server.csr" -subj "/C=US/ST=State/L=City/O=PX4-Server/CN=px4-auth-server" 2>/dev/null
    
    # Generate self-signed certificate
    openssl x509 -req -days 365 -in "$AUTH_DIR/ssl/server.csr" -signkey "$AUTH_DIR/ssl/server.key" -out "$AUTH_DIR/ssl/server.crt" 2>/dev/null
    
    # Set proper permissions
    chmod 600 "$AUTH_DIR/ssl/server.key"
    chmod 644 "$AUTH_DIR/ssl/server.crt"
    
    # Clean up CSR
    rm -f "$AUTH_DIR/ssl/server.csr"
    
    log "SSL certificates generated successfully."
}

# Create authentication users
create_auth_users() {
    log "Setting up authentication users..."
    
    mkdir -p "$AUTH_DIR"
    
    # Create main user authentication file
    if [ ! -f "$AUTH_DIR/.htpasswd" ]; then
        echo ""
        echo -e "${BLUE}Creating user accounts for PX4 remote access${NC}"
        echo -e "${BLUE}You'll be prompted to create users and passwords${NC}"
        echo ""
        
        # Create at least one user
        read -p "Enter username for PX4 remote access: " username
        if [ -z "$username" ]; then
            username="px4user"
            log "Using default username: px4user"
        fi
        
        htpasswd -c "$AUTH_DIR/.htpasswd" "$username"
        
        # Ask for additional users
        while true; do
            echo ""
            read -p "Add another user? (y/n): " add_user
            if [[ $add_user =~ ^[Yy]$ ]]; then
                read -p "Enter username: " additional_user
                if [ ! -z "$additional_user" ]; then
                    htpasswd "$AUTH_DIR/.htpasswd" "$additional_user"
                fi
            else
                break
            fi
        done
    else
        log "Authentication file already exists. Skipping user creation."
    fi
    
    # Create admin authentication file
    if [ ! -f "$AUTH_DIR/.htpasswd-admin" ]; then
        echo ""
        echo -e "${BLUE}Creating admin account for PX4 server management${NC}"
        read -p "Enter admin username: " admin_username
        if [ -z "$admin_username" ]; then
            admin_username="admin"
            log "Using default admin username: admin"
        fi
        
        htpasswd -c "$AUTH_DIR/.htpasswd-admin" "$admin_username"
    else
        log "Admin authentication file already exists. Skipping admin creation."
    fi
    
    # Set proper permissions
    chmod 600 "$AUTH_DIR/.htpasswd"
    chmod 600 "$AUTH_DIR/.htpasswd-admin"
    
    log "Authentication users created successfully."
}

# Configure firewall
configure_firewall() {
    log "Configuring firewall rules..."
    
    # Check if UFW is available
    if command -v ufw &> /dev/null; then
        log "Configuring UFW firewall..."
        
        # Allow SSH
        sudo ufw allow ssh || true
        
        # Allow WireGuard VPN
        sudo ufw allow 51820/udp comment "WireGuard VPN" || true
        
        # Allow HTTPS for authentication
        sudo ufw allow 443/tcp comment "HTTPS Authentication" || true
        
        # Block direct MAVLink access (force VPN usage)
        sudo ufw deny 14540:14560/udp comment "Block direct MAVLink" || true
        
        # Enable UFW if not already enabled
        echo "y" | sudo ufw enable 2>/dev/null || true
        
        log "UFW firewall configured."
        
    elif command -v iptables &> /dev/null; then
        log "Configuring iptables firewall..."
        
        # Allow SSH
        sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT || true
        
        # Allow WireGuard VPN
        sudo iptables -A INPUT -p udp --dport 51820 -j ACCEPT || true
        
        # Allow HTTPS for authentication
        sudo iptables -A INPUT -p tcp --dport 443 -j ACCEPT || true
        
        # Block direct MAVLink access
        sudo iptables -A INPUT -p udp --dport 14540:14560 -j DROP || true
        
        # Save iptables rules (if possible)
        if command -v iptables-save &> /dev/null; then
            sudo iptables-save > /tmp/iptables.rules 2>/dev/null || true
        fi
        
        log "iptables firewall configured."
    else
        warn "No firewall utility found. Please configure firewall manually."
    fi
}

# Create environment configuration
create_environment_config() {
    log "Creating environment configuration..."
    
    # Create .env file for VPN setup
    cat > "$VPN_DIR/.env" << EOF
# PX4 VPN Server Configuration
# Generated by setup script on $(date)

# VPN Configuration
VPN_SERVER_URL=auto
VPN_PEERS=5
TZ=$(cat /etc/timezone 2>/dev/null || echo "America/New_York")

# Security Settings
SECURITY_MODE=wireguard
REMOTE_ACCESS_MODE=vpn

# PX4 Configuration
PX4_HOME_LAT=47.641468
PX4_HOME_LON=-122.140165
PX4_HOME_ALT=0.0
SWARM_SIZE=3

# Network Configuration
VPN_NETWORK=10.10.0.0/24
VPN_SERVER_IP=10.10.0.1
EOF

    # Create deployment script
    cat > "$VPN_DIR/deploy-vpn.sh" << 'EOF'
#!/bin/bash
# Deploy PX4 VPN Server

set -e

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1"
}

log "Starting PX4 VPN server deployment..."

# Build PX4 image if needed
if ! docker images | grep -q "px4-airsim.*slim"; then
    log "Building PX4 Docker image..."
    cd .. && docker build -t px4-airsim:slim -f Dockerfile.slim .
fi

# Start VPN services
log "Starting VPN and authentication services..."
docker-compose -f docker-compose-vpn.yml up -d

# Wait for services to be ready
log "Waiting for services to initialize..."
sleep 30

# Check WireGuard status
if docker exec px4-wireguard-server wg show | grep -q "interface"; then
    log "WireGuard VPN server is running."
else
    log "Warning: WireGuard server may not be properly configured."
fi

# Display client configurations
log "VPN client configurations are available in:"
docker exec px4-wireguard-server ls -la /config/peer_*/ 2>/dev/null || true

log "VPN server deployment completed!"
log ""
log "Next steps:"
log "1. Download client configs: docker exec px4-wireguard-server cat /config/peer_1/peer_1.conf"
log "2. Install WireGuard client on remote devices"
log "3. Import configuration and connect"
log "4. Access PX4 via VPN: 10.10.0.11:14550"
EOF

    chmod +x "$VPN_DIR/deploy-vpn.sh"
    
    log "Environment configuration created."
}

# Create client connection guide
create_client_guide() {
    log "Creating client connection guide..."
    
    cat > "$SCRIPT_DIR/CLIENT_CONNECTION_GUIDE.md" << 'EOF'
# PX4 Remote Server VPN Client Setup Guide

## Overview
This guide explains how to connect to your PX4 server securely using WireGuard VPN.

## Step 1: Install WireGuard Client

### Windows
1. Download WireGuard from: https://www.wireguard.com/install/
2. Install the application
3. Run WireGuard

### macOS
1. Install via App Store: "WireGuard"
2. Or via Homebrew: `brew install --cask wireguard-tools`

### Linux
```bash
# Ubuntu/Debian
sudo apt update && sudo apt install wireguard

# CentOS/RHEL
sudo yum install wireguard-tools
```

### Android/iOS
Install "WireGuard" app from respective app stores.

## Step 2: Get Client Configuration

On your PX4 server, run:
```bash
# List available client configs
docker exec px4-wireguard-server ls /config/

# Display client config (replace peer_1 with your client number)
docker exec px4-wireguard-server cat /config/peer_1/peer_1.conf
```

Copy the entire configuration file content.

## Step 3: Import Configuration

### Desktop (Windows/macOS/Linux)
1. Open WireGuard application
2. Click "Add Tunnel" → "Add empty tunnel" or "Import from file"
3. Paste the configuration or select the downloaded .conf file
4. Click "Save"

### Mobile (Android/iOS)
1. Open WireGuard app
2. Tap "+" → "Create from QR code" or "Create from file or archive"
3. Scan QR code or import file
4. Save the tunnel

## Step 4: Connect to VPN

1. In WireGuard client, select your PX4 tunnel
2. Click "Activate" or toggle the switch
3. Verify connection status shows "Active"

## Step 5: Connect to PX4

Once VPN is connected, you can access PX4 services:

### QGroundControl
1. Open QGroundControl
2. Go to Application Settings → Comm Links
3. Add new UDP connection:
   - Name: PX4 VPN Server
   - Type: UDP
   - Listening Port: 14550
   - Target Hosts: 10.10.0.11 (or 10.10.0.12 for drone 2)
   - Port: 14550

### MAVSDK/Programming APIs
```python
# Python example
from mavsdk import System

drone = System()
await drone.connect(system_address="udp://10.10.0.11:14540")
```

### Custom Applications
Connect to:
- Drone 1: `10.10.0.11:14550` (QGC) or `10.10.0.11:14540` (API)
- Drone 2: `10.10.0.12:14551` (QGC) or `10.10.0.12:14540` (API)
- Drone 3: `10.10.0.13:14552` (QGC) or `10.10.0.13:14540` (API)

## Troubleshooting

### VPN Connection Issues
```bash
# Check VPN server status
docker exec px4-wireguard-server wg show

# Check container logs
docker logs px4-wireguard-server

# Test VPN connectivity
ping 10.10.0.1
```

### PX4 Connection Issues
```bash
# Check PX4 container status
docker ps | grep px4-vpn

# Test MAVLink connectivity
nc -u 10.10.0.11 14550

# Check MAVLink Router status
docker exec px4-mavlink-router-vpn pgrep mavlink-router
```

### Firewall Issues
Ensure these ports are open on the server:
- 51820/udp (WireGuard)
- 443/tcp (Authentication - optional)

## Security Notes

1. **Keep configurations secure**: VPN configs grant access to your PX4 server
2. **Regular key rotation**: Consider regenerating client configs periodically
3. **Monitor connections**: Check VPN logs for unauthorized access attempts
4. **Firewall protection**: VPN blocks direct internet access to MAVLink ports

## Multiple Clients

Each client needs a unique configuration:
```bash
# Generate additional client configs
docker exec px4-wireguard-server wg-quick down wg0
# Edit peer count in docker-compose-vpn.yml (VPN_PEERS=10)
docker-compose -f docker-compose-vpn.yml up -d wireguard
```

For support, check server logs or contact your PX4 server administrator.
EOF

    log "Client connection guide created."
}

# Main setup function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  PX4 Remote Server Security Setup     ${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check if user wants to continue
    read -p "This script will set up VPN and authentication for secure PX4 remote access. Continue? (y/n): " confirm
    if [[ ! $confirm =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 0
    fi
    
    # Run setup steps
    check_requirements
    generate_ssl_certificates
    create_auth_users
    configure_firewall
    create_environment_config
    create_client_guide
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  Setup completed successfully!        ${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    echo "1. Deploy VPN server: cd $VPN_DIR && ./deploy-vpn.sh"
    echo "2. Get client configs: docker exec px4-wireguard-server cat /config/peer_1/peer_1.conf"
    echo "3. Follow client setup guide: $SCRIPT_DIR/CLIENT_CONNECTION_GUIDE.md"
    echo ""
    echo -e "${YELLOW}Security reminders:${NC}"
    echo "- Keep VPN client configurations secure"
    echo "- Monitor VPN connection logs regularly"
    echo "- Consider enabling authentication for additional security"
    echo ""
}

# Run main function
main "$@"