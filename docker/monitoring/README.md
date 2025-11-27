# AirSim Ecosystem Monitoring Integration

**Comprehensive Prometheus/Grafana monitoring for AirSim multi-drone simulation**

## 📊 What This Provides

### Infrastructure Monitoring
- **Container Resources**: CPU, memory, network per drone container (cAdvisor)
- **Host Metrics**: System-level CPU, disk, network (Node Exporter)
- **Service Health**: Connection status for all components

### Application Metrics

#### ROS2 Metrics Exporter (Port 9200)
- Active node count
- Topic publish rates (per topic)
- Message throughput
- TF tree health status
- ROS2 DDS connection health

#### AirSim API Metrics Exporter (Port 9201)
- Vehicle position/velocity (NED coordinates)
- Vehicle altitude and flight state (armed/landed)
- Collision detection and impact force
- API call latency per endpoint
- Simulation FPS and clock speed

#### PX4 MAVLink Metrics Exporter (Port 9202)
- GPS position, altitude, velocity for all 4 drones
- Battery voltage, current, remaining percentage
- Attitude (roll, pitch, yaw)
- Flight mode and armed status
- MAVLink connection health
- GPS fix type and satellite count

### Smart Alerts
- Container crashes or high resource usage (>80%)
- ROS2 topic timeouts (no messages >5s)
- PX4 low battery (<20%) and critical battery (<10%)
- MAVLink disconnections
- GPS fix loss
- AirSim API timeouts
- Collision events

---

## 🚀 Quick Start

### Prerequisites
1. AirSim ecosystem running (see `docker/docker-compose-master.yml`)
2. Docker and Docker Compose installed
3. At least 4GB free RAM for monitoring stack

### Step 1: Build Custom Exporters
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring

# Build the multi-exporter image
cd exporters
docker build -t airsim-metrics-exporters:latest -f Dockerfile.exporters .
```

### Step 2: Start Monitoring Stack
```bash
# Return to monitoring directory
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring

# Start integrated monitoring (connects to AirSim networks)
docker-compose -f docker-compose-integrated.yml up -d

# Check all services are running
docker-compose -f docker-compose-integrated.yml ps
```

### Step 3: Verify Metrics Collection
```bash
# Check Prometheus targets (should all be UP)
curl http://localhost:9090/api/v1/targets | jq '.data.activeTargets[] | {job: .labels.job, health: .health}'

# Test individual exporters
curl http://localhost:9200/metrics  # ROS2 metrics
curl http://localhost:9201/metrics  # AirSim metrics
curl http://localhost:9202/metrics  # PX4 metrics
```

### Step 4: Access Dashboards
- **Grafana**: http://localhost:3000 (admin / P@ssw0rd)
- **Prometheus**: http://localhost:9090
- **cAdvisor**: http://localhost:8080

---

## 📈 Monitoring Architecture

```
┌─────────────────────── AirSim Ecosystem ──────────────────────┐
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │ AirSim       │  │ PX4 Drone 1-4│  │ ROS2 Nodes   │        │
│  │ Container    │  │ Containers   │  │ Container    │        │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘        │
│         │                  │                  │                 │
│         │ RPC (41451)      │ MAVLink (14550+) │ DDS            │
│         │                  │                  │                 │
└─────────┼──────────────────┼──────────────────┼─────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
    ┌──────────┐       ┌──────────┐      ┌──────────┐
    │ AirSim   │       │ PX4      │      │ ROS2     │
    │ Exporter │       │ Exporter │      │ Exporter │
    │ :9201    │       │ :9202    │      │ :9200    │
    └────┬─────┘       └────┬─────┘      └────┬─────┘
         │                  │                  │
         │  HTTP /metrics   │  HTTP /metrics   │  HTTP /metrics
         │                  │                  │
         └──────────────────┼──────────────────┘
                            │
                       ┌────▼─────┐
                       │Prometheus│  ← Alert Rules
                       │  :9090   │
                       └────┬─────┘
                            │
                       ┌────▼─────┐
                       │ Grafana  │  ← Dashboards
                       │  :3000   │
                       └──────────┘
```

---

## 🔧 Configuration

### Environment Variables

#### AirSim Exporter
```bash
EXPORTER_TYPE=airsim
AIRSIM_HOST_IP=airsim-container      # AirSim container hostname
AIRSIM_HOST_PORT=41451                # AirSim RPC port
METRICS_PORT=9201                     # Prometheus metrics port
POLL_INTERVAL=1.0                     # Polling interval (seconds)
```

#### PX4 Exporter
```bash
EXPORTER_TYPE=px4
PX4_NUM_DRONES=4                      # Number of PX4 drones to monitor
PX4_BASE_PORT=14550                   # Base MAVLink port
METRICS_PORT=9202
```

#### ROS2 Exporter
```bash
EXPORTER_TYPE=ros2
ROS_DOMAIN_ID=0                       # ROS2 domain ID
METRICS_PORT=9200
```

### Prometheus Scrape Intervals
Edit `prometheus-integrated.yml`:
```yaml
global:
  scrape_interval: 2s     # Fast scraping for simulation
  evaluation_interval: 5s  # Alert evaluation frequency
```

---

## 📊 Example Prometheus Queries

### Container Resource Usage
```promql
# CPU usage per drone container
rate(container_cpu_usage_seconds_total{name=~"px4-drone-.*"}[1m])

# Memory usage percentage
(container_memory_usage_bytes{name=~"px4-drone-.*"} / container_spec_memory_limit_bytes) * 100
```

### ROS2 Metrics
```promql
# Topic publish rate
ros2_topic_rate_hz{topic_name="/Drone1/odom_local_ned"}

# Total active nodes
ros2_node_count
```

### PX4 Telemetry
```promql
# Battery levels for all drones
px4_battery_remaining_percent

# Altitude above ground
px4_position_relative_altitude_meters

# Ground speed
px4_ground_speed_ms
```

### AirSim Performance
```promql
# API latency (95th percentile)
histogram_quantile(0.95, rate(airsim_api_latency_seconds_bucket[1m]))

# Collision rate
rate(airsim_collisions_total[1m])
```

---

## 🚨 Alert Management

### View Active Alerts
```bash
# Prometheus alerts page
open http://localhost:9090/alerts

# Check alert rules
curl http://localhost:9090/api/v1/rules | jq '.data.groups[].rules[] | select(.alerts | length > 0)'
```

### Modify Alert Thresholds
Edit `alerts/airsim_alerts.yml`:
```yaml
- alert: PX4LowBattery
  expr: px4_battery_remaining_percent < 20  # Change threshold here
  for: 10s
```

Reload Prometheus config:
```bash
curl -X POST http://localhost:9090/-/reload
```

---

## 🐛 Troubleshooting

### All Prometheus Targets Show "DOWN"
**Cause**: Network isolation

**Fix**:
```bash
# Verify exporters can reach services
docker exec prometheus ping -c 2 airsim-container
docker exec prometheus ping -c 2 px4-drone-1
docker exec prometheus ping -c 2 ros2-x11-node

# Check exporter logs
docker logs ros2-metrics-exporter
docker logs airsim-api-exporter
docker logs px4-mavlink-exporter
```

### ROS2 Exporter Shows No Nodes
**Cause**: ROS2 DDS domain mismatch

**Fix**:
```bash
# Check ROS_DOMAIN_ID matches
docker exec ros2-metrics-exporter env | grep ROS_DOMAIN_ID
docker exec ros2-x11-node env | grep ROS_DOMAIN_ID
```

### PX4 Exporter Can't Connect to Drones
**Cause**: MAVLink port mismatch or PX4 not broadcasting

**Fix**:
```bash
# Check PX4 MAVLink configuration
docker exec px4-drone-1 cat /px4_workspace/PX4-Autopilot/build/px4_sitl_default/etc/extras.txt

# Verify MAVLink traffic
docker exec px4-exporter tcpdump -i any port 14550 -c 10
```

### AirSim Exporter Connection Timeout
**Cause**: AirSim container not ready or firewall

**Fix**:
```bash
# Check AirSim is accepting connections
docker exec airsim-container netstat -tuln | grep 41451

# Test RPC connection manually
docker exec airsim-exporter python3 -c "
import cosysairsim as airsim
client = airsim.MultirotorClient(ip='airsim-container', port=41451)
print(client.confirmConnection())
print(client.listVehicles())
"
```

---

## 📦 Files Created

### Metrics Exporters
- `exporters/ros2_metrics_exporter.py` - ROS2 node/topic monitoring
- `exporters/airsim_metrics_exporter.py` - AirSim API telemetry
- `exporters/px4_mavlink_exporter.py` - PX4 MAVLink telemetry
- `exporters/Dockerfile.exporters` - Multi-exporter container image

### Configuration
- `docker-compose-integrated.yml` - Integrated monitoring stack
- `prometheus-integrated.yml` - Prometheus scrape configuration
- `alerts/airsim_alerts.yml` - Alert rules (30+ alerts)

### Dashboards (Coming Soon)
- `dashboards/container-resources.json` - Container CPU/memory/network
- `dashboards/ros2-telemetry.json` - ROS2 node/topic health
- `dashboards/px4-flight.json` - PX4 flight telemetry
- `dashboards/airsim-performance.json` - AirSim API performance

---

## 🔄 Stopping and Cleanup

```bash
# Stop monitoring stack
docker-compose -f docker-compose-integrated.yml down

# Remove all data (WARNING: deletes metrics history)
docker-compose -f docker-compose-integrated.yml down -v

# Remove exporter images
docker rmi airsim-metrics-exporters:latest
```

---

## 📚 Next Steps

1. **Create Custom Grafana Dashboards**
   - Import pre-built dashboards from `dashboards/` directory
   - Customize visualizations for your specific metrics

2. **Configure Alertmanager** (Optional)
   - Set up email/Slack notifications for critical alerts
   - Add Alertmanager service to docker-compose

3. **Optimize Scrape Intervals**
   - Adjust based on your simulation requirements
   - Balance between data granularity and resource usage

4. **Add Custom Metrics**
   - Extend exporters with domain-specific telemetry
   - Add application-level metrics to ROS2 nodes

---

## 💡 Tips

- **Performance**: Monitoring adds ~5-10% CPU overhead - acceptable for simulation
- **Storage**: Prometheus retention is 200h - adjust based on disk space
- **Network**: All monitoring traffic stays within Docker networks (no external latency)
- **Security**: Change Grafana password in production deployments

---

## 🤝 Support

For issues or questions:
1. Check Prometheus targets: http://localhost:9090/targets
2. Review exporter logs: `docker logs <exporter-name>`
3. Verify network connectivity between containers
4. Consult alert rules in `alerts/airsim_alerts.yml`

---

**Status**: ✅ Core monitoring infrastructure complete
**TODO**: Create Grafana dashboard JSON files (templates provided in queries above)
