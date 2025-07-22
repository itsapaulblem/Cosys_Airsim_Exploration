# Cloud Deployment Configurations for PX4 Remote Server Access

This directory contains cloud-specific Docker Compose configurations that enable remote server connectivity for PX4 simulation in the Cosys-AirSim environment.

## Overview

Each configuration extends the base local setup to support:
- **Internet-accessible MAVLink endpoints**
- **Cloud-specific networking and security**
- **Provider-optimized resource allocation**
- **Monitoring and logging integration**
- **Scalable multi-drone deployments**

## Available Configurations

### 1. AWS EC2 (`docker-compose-aws.yml`)
**Best for**: Enterprise deployments, high availability, AWS ecosystem integration

**Features**:
- Elastic Load Balancer support
- CloudWatch monitoring integration
- VPC networking with security groups
- Auto Scaling Group compatibility
- Route 53 DNS integration

**Recommended Instance**: `t3.medium` (2 vCPU, 4 GB RAM)
**Cost**: ~$30-40/month

```bash
# Deploy to AWS
export AWS_PUBLIC_IP=$(curl -s http://169.254.169.254/latest/meta-data/public-ipv4)
export AWS_REGION=us-east-1
docker-compose -f docker-compose-aws.yml up -d
```

### 2. Google Cloud Platform (`docker-compose-gcp.yml`)
**Best for**: AI/ML workloads, Google ecosystem, global load balancing

**Features**:
- Cloud Load Balancer integration
- Stackdriver monitoring
- Cloud SQL proxy for telemetry storage
- Cloud NAT for outbound connectivity
- IAM and service account integration

**Recommended Instance**: `e2-standard-2` (2 vCPU, 8 GB RAM)
**Cost**: ~$25-35/month

```bash
# Deploy to GCP
export GCP_PROJECT_ID=your-project-id
export GCP_EXTERNAL_IP=$(curl -s "http://metadata.google.internal/computeMetadata/v1/instance/network-interfaces/0/external-ip" -H "Metadata-Flavor: Google")
docker-compose -f docker-compose-gcp.yml up -d
```

### 3. Microsoft Azure (`docker-compose-azure.yml`)
**Best for**: Enterprise Windows environments, Microsoft ecosystem integration

**Features**:
- Azure Load Balancer support
- Application Insights monitoring
- Network Security Groups
- Azure Key Vault integration
- Log Analytics workspace

**Recommended Instance**: `Standard_B2s` (2 vCPU, 4 GB RAM)
**Cost**: ~$30-40/month

```bash
# Deploy to Azure
export AZURE_PUBLIC_IP=$(curl -s -H Metadata:true "http://169.254.169.254/metadata/instance/network/interface/0/ipv4/ipAddress/0/publicIpAddress?api-version=2017-08-01&format=text")
export AZURE_RESOURCE_GROUP=px4-rg
docker-compose -f docker-compose-azure.yml up -d
```

### 4. Generic VPS (`docker-compose-vps.yml`)
**Best for**: Cost-effective hosting, development, small-scale deployments

**Features**:
- Lightweight resource usage
- Provider-agnostic configuration
- Cost optimization settings
- Simple iptables firewall
- Optional SSL with Let's Encrypt

**Supported Providers**: DigitalOcean, Linode, Vultr, Hetzner, AWS Lightsail
**Recommended Size**: 2 vCPU, 4 GB RAM
**Cost**: ~$20-25/month

```bash
# Deploy to VPS
export VPS_PUBLIC_IP=$(curl -s ifconfig.me)
export VPS_PROVIDER=digitalocean
docker-compose -f docker-compose-vps.yml up -d
```

## Quick Start Guide

### 1. Choose Your Cloud Provider
Select the configuration that matches your needs:
- **AWS**: Enterprise, existing AWS infrastructure
- **GCP**: AI/ML workloads, global reach
- **Azure**: Microsoft ecosystem integration
- **VPS**: Cost-effective, simple deployment

### 2. Prepare Your Environment
```bash
# Clone and navigate to deployment directory
cd /path/to/cosys-airsim/docker/px4_airsim_docker/cloud-deployments

# Copy environment template
cp .env.template .env

# Edit configuration variables
nano .env
```

### 3. Configure Cloud Resources
Each provider requires specific setup steps:

#### AWS Setup
```bash
# Create security group
aws ec2 create-security-group --group-name px4-server --description "PX4 Server Access"
aws ec2 authorize-security-group-ingress --group-name px4-server --protocol udp --port 14540-14560 --cidr 0.0.0.0/0

# Launch instance with user data script
aws ec2 run-instances --image-id ami-0abcdef1234567890 --instance-type t3.medium --security-groups px4-server
```

#### GCP Setup
```bash
# Create firewall rules
gcloud compute firewall-rules create px4-mavlink-udp --allow udp:14540-14560 --source-ranges 0.0.0.0/0

# Create instance
gcloud compute instances create px4-server --machine-type e2-standard-2 --tags px4-server
```

#### Azure Setup
```bash
# Create resource group
az group create --name px4-rg --location "East US"

# Create virtual machine
az vm create --resource-group px4-rg --name px4-server --image UbuntuLTS --size Standard_B2s
```

### 4. Deploy PX4 Server
```bash
# Build the image (if not using registry)
docker build -t px4-airsim:slim -f Dockerfile.slim .

# Deploy with chosen configuration
docker-compose -f docker-compose-[provider].yml up -d

# Verify deployment
docker-compose ps
```

### 5. Test Remote Connectivity
```bash
# Test MAVLink connectivity
nc -u [SERVER_IP] 14550

# Use provided testing tools
cd ../tools
python3 unified_mavlink_tester.py --remote-server [SERVER_IP]
```

## Client Connection Examples

### QGroundControl
1. Open QGroundControl
2. Go to **Application Settings** > **Comm Links**
3. Create new connection:
   - **Type**: UDP
   - **Listening Port**: 14550
   - **Target Host**: `[YOUR_SERVER_IP]`
   - **Port**: 14550

### MAVSDK (Python)
```python
import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udp://[SERVER_IP]:14540")
    
    async for state in drone.core.connection_state():
        print(f"Drone connection state: {state}")
        if state.is_connected:
            break

if __name__ == "__main__":
    asyncio.run(main())
```

### Custom UDP Client
```python
import socket

# Connect to remote PX4 server
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('[SERVER_IP]', 14540)

# Send MAVLink heartbeat
heartbeat = b'\\xfe\\x09\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00'
sock.sendto(heartbeat, server_address)
```

## Security Considerations

### Firewall Configuration
Each deployment includes provider-specific firewall rules:
- **UDP 14540-14560**: MAVLink communication
- **TCP 22**: SSH access (from your IP only)
- **TCP 80/443**: Web interface (optional)

### VPN Setup (Recommended)
For production deployments, consider VPN access:
```bash
# Install WireGuard
apt install wireguard

# Configure VPN tunnel (see security/ directory for full setup)
wg-quick up wg0
```

### Authentication
Consider implementing:
- **Client certificates** for MAVLink authentication
- **API tokens** for programmatic access
- **Rate limiting** to prevent abuse

## Monitoring and Logging

### Health Checks
All configurations include health check endpoints:
```bash
# Check container health
curl http://[SERVER_IP]:8080/health

# View health status
docker-compose ps
```

### Monitoring Integration
- **AWS**: CloudWatch metrics and logs
- **GCP**: Stackdriver monitoring
- **Azure**: Application Insights and Log Analytics
- **VPS**: Optional Prometheus/Grafana

### Log Access
```bash
# View container logs
docker-compose logs -f px4-cloud-drone-1

# Monitor MAVLink traffic
docker exec px4-cloud-drone-1 tail -f /var/log/mavlink.log
```

## Troubleshooting

### Common Issues

#### Connection Refused
```bash
# Check firewall rules
iptables -L
ufw status

# Verify port bindings
netstat -tulpn | grep 14550
```

#### High Latency
```bash
# Test network latency
ping [SERVER_IP]
traceroute [SERVER_IP]

# Check container resources
docker stats
```

#### MAVLink Errors
```bash
# Verify MAVLink Router status
docker exec px4-cloud-drone-1 pgrep mavlink-router

# Check MAVLink configuration
docker exec px4-cloud-drone-1 cat /px4_workspace/mavlink-router-config/mavlink-router-instance-1.conf
```

### Debug Mode
Enable debug logging:
```bash
# Set debug environment
export PX4_DEBUG=1
export MAVLINK_DEBUG=1

# Restart with debug
docker-compose down && docker-compose up -d
```

## Cost Optimization

### Resource Limits
```yaml
# Add to service configuration
deploy:
  resources:
    limits:
      memory: 512M
      cpus: '0.5'
```

### Scheduled Shutdown
```bash
# Auto-shutdown during off-hours
echo "0 22 * * * docker-compose stop" | crontab -
echo "0 8 * * * docker-compose start" | crontab -
```

### Spot Instances
Consider using spot/preemptible instances for development:
- **AWS**: Spot Instances (up to 90% discount)
- **GCP**: Preemptible VMs (up to 80% discount)
- **Azure**: Spot VMs (up to 90% discount)

## Next Steps

1. **Choose deployment configuration** based on your needs
2. **Set up cloud resources** using provider-specific instructions
3. **Deploy PX4 server** with appropriate configuration
4. **Test connectivity** using provided tools
5. **Implement security measures** (VPN, authentication)
6. **Set up monitoring** for production use

For additional support and advanced configurations, see:
- `../docs/PX4_REMOTE_CONNECTIVITY_ARCHITECTURE.md`
- `../security/` directory for VPN and authentication setup
- `../tools/` directory for connectivity testing tools