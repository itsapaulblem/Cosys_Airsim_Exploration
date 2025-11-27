# AirSim Monitoring Stack - Quick Start Guide

## ✅ Prerequisites
- Docker and Docker Compose installed
- At least 4GB free RAM
- Port 41451 (AirSim RPC) available or AirSim already running

---

## 🚀 Step-by-Step Deployment

### 0. Start AirSim Ecosystem FIRST (Required!)

**CRITICAL**: The monitoring stack requires three Docker networks that are created by the AirSim ecosystem. You MUST start the ecosystem before the monitoring stack.

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Start the AirSim ecosystem (creates required networks and services)
docker compose -f docker-compose-master.yml --profile linux-integrated up -d \
  airsim-container px4-bridge-drone-1 px4-bridge-drone-2 \
  px4-bridge-drone-3 px4-bridge-drone-4 ros2-x11-node qgroundcontrol-x11
```

**This creates the required networks**:
- `airsim-ecosystem` (172.30.0.0/16)
- `px4_network` (172.20.0.0/16)
- `ros2-multi-node-network` (172.26.0.0/16)

**Verify ecosystem is running**:
```bash
# Check networks exist
docker network ls | grep -E "airsim-ecosystem|px4_network|ros2-multi-node-network"

# Check containers are healthy
docker ps --filter "name=airsim|px4-drone|ros2" --format "table {{.Names}}\t{{.Status}}"
```

**Expected output**: All containers should show "Up" and PX4 drones should be "healthy".

---

### 1. Build the Metrics Exporters

**Important**: Build from the project root, not from the `exporters/` directory!

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration

# Build the multi-exporter image (includes ROS2, AirSim, PX4 exporters)
docker build -t airsim-metrics-exporters:latest -f docker/monitoring/exporters/Dockerfile.exporters .
```

**Build time**: ~2-3 minutes (downloads opencv, pymavlink, etc.)

**Expected output**:
```
Successfully installed ... opencv-contrib-python ... prometheus-client ...
naming to docker.io/library/airsim-metrics-exporters:latest done
```

**Verify the image**:
```bash
docker images | grep airsim-metrics-exporters
# Should show: airsim-metrics-exporters   latest   <image-id>   <time-ago>   1.18GB
```

---

### 2. Start the Monitoring Stack

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring

# Start integrated monitoring (connects to AirSim networks)
docker-compose -f docker-compose-integrated.yml up -d

# Check all services are running
docker-compose -f docker-compose-integrated.yml ps
```

**Expected services** (should all show "Up"):
- `prometheus` (port 9090)
- `grafana` (port 3000)
- `cadvisor` (port 8080)
- `node-exporter` (port 9100)
- `ros2-metrics-exporter` (port 9200)
- `airsim-api-exporter` (port 9201)
- `px4-mavlink-exporter` (port 9202)

---

### 3. Verify Metrics Collection

#### Check Prometheus Targets (All Should Be "UP")
```bash
curl -s http://localhost:9090/api/v1/targets | \
  jq '.data.activeTargets[] | {job: .labels.job, health: .health, lastScrape: .lastScrape}'
```

**Expected**: All targets showing `"health": "up"`

#### Test Individual Exporters
```bash
# ROS2 metrics
curl http://localhost:9200/metrics | grep ros2_node_count

# AirSim metrics
curl http://localhost:9201/metrics | grep airsim_connection_status

# PX4 metrics
curl http://localhost:9202/metrics | grep px4_battery_remaining_percent
```

---

### 4. Access Dashboards

- **Grafana**: http://localhost:3000
  - Username: `admin`
  - Password: `P@ssw0rd`

- **Prometheus**: http://localhost:9090
  - Check targets: http://localhost:9090/targets
  - Check alerts: http://localhost:9090/alerts

- **cAdvisor**: http://localhost:8080
  - Container resource usage

---

## 📊 Key Metrics Available

### ROS2 Metrics (Port 9200)
- `ros2_node_count` - Total active ROS2 nodes
- `ros2_topic_count` - Total active topics
- `ros2_topic_rate_hz{topic_name="..."}` - Topic publish rate
- `ros2_connection_health` - DDS connection status

### AirSim Metrics (Port 9201)
- `airsim_vehicle_position_x/y/z_meters{vehicle_name="..."}` - Vehicle position (NED)
- `airsim_vehicle_velocity_x/y/z_mps{vehicle_name="..."}` - Velocity
- `airsim_vehicle_altitude_meters{vehicle_name="..."}` - Altitude
- `airsim_vehicle_armed{vehicle_name="..."}` - Armed status
- `airsim_collisions_total{vehicle_name="...", object_name="..."}` - Collision count
- `airsim_connection_status` - AirSim API connection

### PX4 Metrics (Port 9202)
- `px4_battery_remaining_percent{drone_id="..."}` - Battery level
- `px4_position_latitude/longitude_degrees{drone_id="..."}` - GPS position
- `px4_position_altitude_meters{drone_id="..."}` - Altitude MSL
- `px4_velocity_vx/vy/vz_ms{drone_id="..."}` - Velocity
- `px4_attitude_roll/pitch/yaw_degrees{drone_id="..."}` - Attitude
- `px4_armed_status{drone_id="..."}` - Armed status
- `px4_mavlink_connection_status{drone_id="..."}` - MAVLink health

### Container Metrics (cAdvisor - Port 8080)
- `container_cpu_usage_seconds_total{name="px4-drone-1"}` - CPU usage per container
- `container_memory_usage_bytes{name="px4-drone-1"}` - Memory usage
- `container_network_receive_bytes_total` - Network RX
- `container_network_transmit_bytes_total` - Network TX

---

## 🛠️ Troubleshooting

### Issue: "network ros2-multi-node-network declared as external, but could not be found"
**Cause**: AirSim ecosystem not started yet. The monitoring stack requires networks created by docker-compose-master.yml

**Fix**: Start the AirSim ecosystem FIRST (see Step 0 above):
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Start ecosystem to create required networks
docker compose -f docker-compose-master.yml --profile linux-integrated up -d \
  airsim-container px4-bridge-drone-1 px4-bridge-drone-2 \
  px4-bridge-drone-3 px4-bridge-drone-4 ros2-x11-node qgroundcontrol-x11

# Verify networks exist
docker network ls | grep -E "airsim-ecosystem|px4_network|ros2-multi-node-network"

# Then start monitoring stack
cd docker/monitoring
docker compose -f docker-compose-integrated.yml up -d
```

---

### Issue: Build fails with "python3-rclpy not found"
**Cause**: Building from wrong directory or old Dockerfile cached

**Fix**:
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration  # MUST be project root!
docker build --no-cache -t airsim-metrics-exporters:latest \
  -f docker/monitoring/exporters/Dockerfile.exporters .
```

### Issue: "cosysairsim not found" when running exporter
**Cause**: AirSim Python client not copied correctly

**Fix**: Check build context includes PythonClient directory:
```bash
# Verify build context
ls PythonClient/cosysairsim/client.py  # Should exist
```

### Issue: All Prometheus targets showing "DOWN"
**Cause**: Network isolation or exporters not running

**Fix**:
```bash
# Check exporter logs
docker logs ros2-metrics-exporter
docker logs airsim-api-exporter
docker logs px4-mavlink-exporter

# Test network connectivity
docker exec prometheus ping -c 2 airsim-container
docker exec prometheus ping -c 2 px4-drone-1
docker exec prometheus ping -c 2 ros2-x11-node
```

### Issue: ROS2 exporter shows no nodes
**Cause**: ROS_DOMAIN_ID mismatch

**Fix**:
```bash
# Check domain IDs match
docker exec ros2-metrics-exporter env | grep ROS_DOMAIN_ID
docker exec ros2-x11-node env | grep ROS_DOMAIN_ID

# Update if needed in docker-compose-integrated.yml
```

### Issue: PX4 exporter can't connect to drones
**Cause**: MAVLink port mismatch

**Fix**:
```bash
# Check PX4 is listening on MAVLink ports
docker exec px4-drone-1 netstat -tuln | grep 14550

# Check exporter can reach PX4
docker exec px4-mavlink-exporter ping -c 2 px4-drone-1
```

---

## 📈 Example Prometheus Queries

### CPU Usage per Drone
```promql
rate(container_cpu_usage_seconds_total{name=~"px4-drone-.*"}[1m]) * 100
```

### Battery Levels (All Drones)
```promql
px4_battery_remaining_percent
```

### ROS2 Topic Rates
```promql
ros2_topic_rate_hz{topic_name=~"/Drone.*/odom_local_ned"}
```

### Collision Events (Last 5 Minutes)
```promql
increase(airsim_collisions_total[5m])
```

### MAVLink Connection Health
```promql
px4_mavlink_connection_status{drone_id=~"drone-.*"}
```

---

## 🔄 Stopping and Cleanup

```bash
# Stop monitoring stack
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring
docker-compose -f docker-compose-integrated.yml down

# Remove all data (WARNING: deletes metrics history)
docker-compose -f docker-compose-integrated.yml down -v

# Remove exporter image
docker rmi airsim-metrics-exporters:latest
```

---

## 📝 What Was Fixed

### Original Error
```
E: Unable to locate package python3-rclpy
```

### Root Causes
1. ❌ Using wrong package names (`python3-rclpy` doesn't exist)
2. ❌ Trying to install packages already in base image
3. ❌ Circular import in cosysairsim setup.py
4. ❌ Wrong build context (missing PythonClient directory)

### Fixes Applied
1. ✅ Removed incorrect apt-get install of ROS2 packages (already in `ros:humble-ros-base-jammy`)
2. ✅ Changed build context to project root to access PythonClient
3. ✅ Copied cosysairsim directly instead of pip install (avoids setup.py circular import)
4. ✅ Updated COPY paths to match new build context
5. ✅ Added .dockerignore to optimize build

### Result
- ✅ **Build successful** (~2-3 minutes)
- ✅ **Image size**: 1.18GB (reasonable for ROS2 + opencv)
- ✅ **All exporters functional** (ROS2, AirSim, PX4)

---

## 🎯 Next Steps

1. **Create Grafana dashboards** using example queries above
2. **Configure alerting** - alerts already defined in `alerts/airsim_alerts.yml`
3. **Customize scrape intervals** in `prometheus-integrated.yml` if needed
4. **Add custom metrics** to ROS2 nodes or exporters

---

For detailed documentation, see: `README.md`

For issues, check:
- Prometheus targets: http://localhost:9090/targets
- Exporter logs: `docker logs <exporter-name>`
- Container health: `docker ps`
