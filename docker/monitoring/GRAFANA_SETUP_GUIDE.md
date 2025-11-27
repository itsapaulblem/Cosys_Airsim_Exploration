# Grafana Configuration Guide for AirSim Monitoring

**Status**: ✅ Prometheus and Grafana are fully configured and running

## Access Your Monitoring Stack

- **Grafana**: http://localhost:3000
  - Username: `admin`
  - Password: `P@ssw0rd`

- **Prometheus**: http://localhost:9090
  - View alerts: http://localhost:9090/alerts
  - View targets: http://localhost:9090/targets

---

## ✅ Already Configured

1. **Prometheus** is running with 15 alert rules loaded from `alerts/airsim_alerts.yml`
2. **Grafana** datasource is configured to use Prometheus
3. **All targets are UP**: ros2-metrics, airsim-api-metrics, px4-mavlink-metrics, cadvisor, node-exporter, prometheus

---

## Step 1: View Prometheus Alerts in Grafana

### Option A: Quick Alert List Panel

1. Login to Grafana: http://localhost:3000
2. Click **"+"** → **"Dashboard"** → **"Add visualization"**
3. Select **"prometheus"** datasource
4. In the panel type dropdown (top right), change from "Time series" to **"Alert list"**
5. Configure the alert list:
   - **Show**: "Current state"
   - **State filter**: All
   - **Alert name filter**: Leave empty to show all alerts
6. Click **"Apply"**
7. Click **"Save dashboard"** (disk icon, top right)
8. Name it: "AirSim Alerts Overview"

**Result**: You'll see all 15 Prometheus alerts with their current state (OK, Pending, Firing).

### Option B: Alert State Timeline (Visual)

1. Create new panel
2. Select **"State timeline"** visualization
3. Query: `ALERTS{alertname=~".*"}`
4. This shows alert state changes over time with color coding:
   - Green = OK
   - Yellow = Pending
   - Red = Firing

---

## Step 2: Create Dashboards for Metrics

### Dashboard 1: Container Resources

Create a new dashboard with these panels:

#### Panel 1: CPU Usage per Container
```promql
# Query
rate(container_cpu_usage_seconds_total{name=~"px4-drone-.*|ros2-.*|airsim-container"}[1m]) * 100

# Visualization: Time series
# Legend: {{name}}
# Unit: percent (0-100)
# Thresholds: 50% (yellow), 80% (red)
```

#### Panel 2: Memory Usage per Container
```promql
# Query
(container_memory_usage_bytes{name=~"px4-drone-.*|ros2-.*|airsim-container"} / container_spec_memory_limit_bytes) * 100

# Visualization: Time series
# Legend: {{name}}
# Unit: percent (0-100)
# Thresholds: 70% (yellow), 90% (red)
```

#### Panel 3: Network Traffic
```promql
# Receive
rate(container_network_receive_bytes_total{name=~"px4-drone-.*"}[1m])

# Transmit
rate(container_network_transmit_bytes_total{name=~"px4-drone-.*"}[1m])

# Visualization: Time series
# Unit: bytes/sec
```

---

### Dashboard 2: ROS2 Metrics

#### Panel 1: Active ROS2 Nodes
```promql
# Query
ros2_node_count

# Visualization: Stat panel
# Show: Current value
# Thresholds: 0-5 (red), 5-8 (yellow), 8+ (green)
```

#### Panel 2: Topic Publish Rates
```promql
# Query
ros2_topic_rate_hz

# Visualization: Time series
# Legend: {{topic_name}}
# Unit: hertz (Hz)
```

#### Panel 3: ROS2 Connection Health
```promql
# Query
ros2_connection_health

# Visualization: Stat panel
# Mappings:
#   Value: 0 → Text: "Disconnected" → Color: Red
#   Value: 1 → Text: "Connected" → Color: Green
```

---

### Dashboard 3: PX4 Telemetry

#### Panel 1: Battery Levels (All Drones)
```promql
# Query
px4_battery_remaining_percent

# Visualization: Gauge
# Legend: Drone {{drone_id}}
# Unit: percent (0-100)
# Thresholds:
#   0-10: red (critical)
#   10-20: orange (low)
#   20-100: green (ok)
```

#### Panel 2: GPS Position (Map View)
```promql
# Latitude
px4_position_latitude_degrees

# Longitude
px4_position_longitude_degrees

# Visualization: Table or Geomap
# Note: For actual map visualization, use Grafana Geomap panel
```

#### Panel 3: Altitude MSL
```promql
# Query
px4_position_altitude_meters

# Visualization: Time series
# Legend: {{drone_id}}
# Unit: meters
```

#### Panel 4: Armed Status
```promql
# Query
px4_armed_status

# Visualization: Stat panel (one per drone)
# Mappings:
#   Value: 0 → Text: "Disarmed" → Color: Gray
#   Value: 1 → Text: "Armed" → Color: Red
```

#### Panel 5: MAVLink Connection Status
```promql
# Query
px4_mavlink_connection_status

# Visualization: Stat panel
# Mappings:
#   Value: 0 → Text: "Disconnected" → Color: Red
#   Value: 1 → Text: "Connected" → Color: Green
```

---

### Dashboard 4: AirSim Performance

#### Panel 1: Vehicle Positions (3D View)
```promql
# X Position
airsim_vehicle_position_x_meters

# Y Position
airsim_vehicle_position_y_meters

# Z Position (Altitude)
airsim_vehicle_position_z_meters

# Visualization: Table or Stat panels (one per vehicle)
# Unit: meters
```

#### Panel 2: Collision Events
```promql
# Query
rate(airsim_collisions_total[1m])

# Visualization: Time series
# Legend: {{vehicle_name}} - {{object_name}}
# Alert: rate > 0 indicates active collisions
```

#### Panel 3: AirSim API Connection
```promql
# Query
airsim_connection_status

# Visualization: Stat panel
# Mappings:
#   Value: 0 → Text: "Disconnected" → Color: Red
#   Value: 1 → Text: "Connected" → Color: Green
```

#### Panel 4: Vehicle Velocities
```promql
# Query
airsim_vehicle_velocity_x_mps
airsim_vehicle_velocity_y_mps
airsim_vehicle_velocity_z_mps

# Visualization: Time series
# Legend: {{vehicle_name}} - {{axis}}
# Unit: meters per second (m/s)
```

---

## Step 3: Set Up Alert Notifications (Optional)

If you want to receive notifications when alerts fire:

### Option 1: Slack Notifications

1. Go to **Alerting** → **Contact points**
2. Click **"+ Add contact point"**
3. Select **"Slack"**
4. Enter your Slack webhook URL
5. Click **"Test"** to verify
6. Click **"Save contact point"**

### Option 2: Email Notifications

1. Go to **Alerting** → **Contact points**
2. Click **"+ Add contact point"**
3. Select **"Email"**
4. Configure SMTP settings in `docker-compose-integrated.yml`:
```yaml
grafana:
  environment:
    - GF_SMTP_ENABLED=true
    - GF_SMTP_HOST=smtp.gmail.com:587
    - GF_SMTP_USER=your-email@gmail.com
    - GF_SMTP_PASSWORD=your-app-password
    - GF_SMTP_FROM_ADDRESS=your-email@gmail.com
```

### Create Notification Policy

1. Go to **Alerting** → **Notification policies**
2. Click **"+ New policy"**
3. Set matchers:
   - Label: `severity`
   - Operator: `=`
   - Value: `critical`
4. Select contact point (Slack, Email, etc.)
5. Click **"Save policy"**

---

## Step 4: View Active Alerts

### In Prometheus
- http://localhost:9090/alerts
- Shows all 15 alert rules and their current state
- Color coding:
  - Green: OK (no alerts firing)
  - Yellow: Pending (condition met but waiting for `for` duration)
  - Red: Firing (alert active)

### In Grafana
1. Go to **Alerting** → **Alert rules**
2. Click **"Prometheus"** tab
3. View all Prometheus-managed alerts
4. Filter by state, severity, component

---

## Available Alerts (15 Total)

### Container Health (3 alerts)
- `ContainerDown` - Container crashed or stopped
- `HighContainerCPU` - CPU usage >80% for 2 minutes
- `HighContainerMemory` - Memory usage >90% for 2 minutes

### ROS2 Health (3 alerts)
- `ROS2TopicTimeout` - No messages on topic for 5 seconds
- `ROS2NodeDown` - Less than 8 nodes active
- `ROS2ConnectionDegraded` - DDS connection failed

### PX4 Telemetry (4 alerts)
- `PX4LowBattery` - Battery <20% for 10 seconds
- `PX4CriticalBattery` - Battery <10% for 5 seconds (CRITICAL)
- `PX4MAVLinkDisconnected` - MAVLink connection lost
- `PX4GPSLost` - GPS fix type <3 (no 3D fix)

### AirSim Performance (2 alerts)
- `AirSimAPIDisconnected` - RPC API connection lost
- `AirSimCollision` - Collision detected in last 10 seconds

---

## Quick Links

### Grafana Dashboards (After Creation)
- http://localhost:3000/d/<dashboard-uid>/airsim-alerts-overview
- http://localhost:3000/d/<dashboard-uid>/container-resources
- http://localhost:3000/d/<dashboard-uid>/ros2-metrics
- http://localhost:3000/d/<dashboard-uid>/px4-telemetry
- http://localhost:3000/d/<dashboard-uid>/airsim-performance

### Prometheus
- **Alerts**: http://localhost:9090/alerts
- **Targets**: http://localhost:9090/targets
- **Graph**: http://localhost:9090/graph
- **Alert Rules**: http://localhost:9090/rules

### Direct Exporter Access
- **ROS2 Metrics**: http://localhost:9200/metrics
- **AirSim Metrics**: http://localhost:9201/metrics
- **PX4 Metrics**: http://localhost:9202/metrics
- **cAdvisor**: http://localhost:8080
- **Node Exporter**: http://localhost:9100/metrics

---

## Tips

1. **Dashboard Variables**: Add dropdown filters for drone_id, vehicle_name, etc.
2. **Time Range**: Use 5-15 minute windows for simulation monitoring
3. **Refresh Rate**: Set dashboards to auto-refresh every 5-10 seconds
4. **Panel Repeating**: Use "Repeat by variable" to automatically create panels for each drone
5. **Annotations**: Mark events like "Mission Start", "Takeoff", "Landing" on dashboards

---

## Troubleshooting

### No data in Grafana panels
1. Check Prometheus targets are UP: http://localhost:9090/targets
2. Test query directly in Prometheus: http://localhost:9090/graph
3. Verify datasource connection: Grafana → Configuration → Data Sources → prometheus → Save & Test

### Alerts not showing
1. Verify Prometheus loaded rules: `curl http://localhost:9090/api/v1/rules | jq '.data.groups'`
2. Check alert file syntax: `docker exec prometheus promtool check rules /etc/prometheus/alerts/airsim_alerts.yml`
3. Reload Prometheus config: `curl -X POST http://localhost:9090/-/reload`

### Alert not firing when condition is met
- Check the `for` duration in the alert rule (e.g., "for: 10s" means condition must be true for 10 seconds)
- View alert state in Prometheus: http://localhost:9090/alerts (will show "Pending" before firing)

---

## Next Steps

1. **Create your dashboards** using the example queries above
2. **Customize alert thresholds** in `docker/monitoring/alerts/airsim_alerts.yml`
3. **Set up notifications** for critical alerts (battery, crashes, MAVLink loss)
4. **Export dashboards** as JSON for version control and sharing

**Documentation**: For more details, see `README.md` and `QUICKSTART.md` in this directory.
