# Monitoring Stack Migration Guide

## ✅ Problem Solved

**Issue**: Monitoring stack (Grafana/Prometheus) breaks when you `docker compose up`/`down` the AirSim ecosystem.

**Root Cause**: The separate monitoring stack in `docker/monitoring/docker-compose-integrated.yml` declares ecosystem networks as `external: true`. When the ecosystem goes down, those networks get removed, breaking the monitoring exporters' connectivity.

**Solution**: Monitoring services have been integrated into `docker-compose-master.yml` under the `monitoring` profile, ensuring proper startup/shutdown order and network management.

---

## Migration Steps

### Step 1: Stop Current Monitoring Stack

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring

# Stop the separate monitoring stack
docker compose -f docker-compose-integrated.yml down
```

**Result**: All monitoring containers stopped, exporters removed from ecosystem networks.

---

### Step 2: Use Unified Command

From now on, use a single command to start/stop everything:

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Start ecosystem WITH monitoring
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d

# Or stop everything together
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down
```

---

## New Workflow

### Starting the Full Stack

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Single command starts:
# - AirSim container
# - 4 PX4 drones
# - ROS2 X11 node
# - QGroundControl
# - Prometheus + Grafana + Alerts
# - All exporters (ROS2, AirSim, PX4)
# - cAdvisor + node-exporter
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d
```

**Access your monitoring**:
- Grafana: http://localhost:3000 (admin/P@ssw0rd)
- Prometheus: http://localhost:9090
- Alerts: http://localhost:9090/alerts

### Stopping the Full Stack

```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Single command stops everything in correct order
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down
```

**Benefits**:
- ✅ Monitoring stops BEFORE ecosystem (releases networks first)
- ✅ Ecosystem stops AFTER monitoring (networks still available)
- ✅ No manual coordination needed
- ✅ No "network still in use" errors

---

## Profile Options

You can mix and match profiles based on what you need:

### Ecosystem Only (No Monitoring)
```bash
docker compose -f docker-compose-master.yml --profile linux-integrated up -d
```
Starts AirSim, PX4, ROS2, QGroundControl (no Prometheus/Grafana).

### Monitoring Only (Debugging)
```bash
# Requires ecosystem to be running already!
docker compose -f docker-compose-master.yml --profile monitoring up -d
```
Starts only Prometheus, Grafana, exporters (assumes ecosystem networks exist).

### Full Stack (Recommended)
```bash
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d
```
Starts everything with proper dependencies.

---

## Differences from Old Setup

### Old Way (Separate Stacks)
```bash
# Start ecosystem
cd docker
docker compose -f docker-compose-master.yml --profile linux-integrated up -d

# Start monitoring (in separate terminal/step)
cd monitoring
docker compose -f docker-compose-integrated.yml up -d

# Problem: Stopping ecosystem breaks monitoring!
docker compose -f docker-compose-master.yml down  # ❌ Networks removed
```

### New Way (Unified)
```bash
# Start everything
cd docker
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d

# Stop everything (correct order automatic)
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down  # ✅ Clean shutdown
```

---

## What Changed

### In `docker-compose-master.yml`

Added 7 new services under `monitoring` profile:
1. `prometheus` - Metrics aggregation, alert evaluation
2. `grafana` - Dashboard visualization
3. `cadvisor` - Container resource monitoring
4. `node-exporter` - Host system metrics
5. `ros2-exporter` - ROS2 node/topic metrics
6. `airsim-exporter` - AirSim API telemetry
7. `px4-exporter` - PX4 MAVLink telemetry

**Profiles**:
- All monitoring services have `monitoring`, `integrated`, AND `linux-integrated` profiles
- This means:
  - `--profile monitoring` → starts monitoring only
  - `--profile integrated` OR `--profile linux-integrated` → starts ecosystem AND monitoring
  - No profile needed for monitoring when using `integrated`/`linux-integrated`

**Dependencies**:
- Prometheus depends on `airsim-container` (waits for it to be healthy)
- Grafana depends on Prometheus
- Exporters depend on Prometheus
- Proper startup order enforced automatically

---

## Troubleshooting

### Issue: "service 'ros2-exporter' didn't match any service"

**Cause**: Old monitoring stack containers still running.

**Fix**:
```bash
# Stop old monitoring stack completely
cd /home/mnsuser/Cosys_Airsim_Exploration/docker/monitoring
docker compose -f docker-compose-integrated.yml down

# Remove any orphan containers
docker rm -f ros2-metrics-exporter airsim-api-exporter px4-mavlink-exporter prometheus grafana cadvisor node-exporter 2>/dev/null

# Start with unified command
cd ..
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d
```

---

### Issue: Networks already exist with different subnet

**Cause**: Old networks from previous runs.

**Fix**:
```bash
# Remove old networks (ensure no containers using them)
docker network rm airsim-ecosystem px4_network ros2-multi-node-network 2>/dev/null

# Start fresh
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d
```

---

### Issue: Port conflicts (3000, 9090, 8080, 9100, 9200-9202)

**Cause**: Old monitoring containers still running from separate stack.

**Fix**:
```bash
# Check what's using the ports
ss -tlnp | grep -E "3000|9090|8080|9100|9200|9201|9202"

# Stop old monitoring stack
cd docker/monitoring
docker compose -f docker-compose-integrated.yml down

# Verify no containers using ports
docker ps --format "table {{.Names}}\t{{.Ports}}" | grep -E "3000|9090"

# Start unified stack
cd ..
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d
```

---

## Verifying Migration

### 1. Check All Services Running
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

docker compose -f docker-compose-master.yml ps --filter "name=prom|grafana|exporter|cadvisor|node-exporter"
```

**Expected**: 7 monitoring containers with status "Up"

---

### 2. Check Networks
```bash
docker network ls | grep -E "airsim-ecosystem|px4_network|ros2"
```

**Expected**: Three networks exist and are managed by `tevv-airstack` compose project

---

### 3. Check Prometheus Targets
```bash
curl -s http://localhost:9090/api/v1/targets | jq '.data.activeTargets[] | {job: .labels.job, health: .health}'
```

**Expected**: All 6 targets showing `"health": "up"`

---

### 4. Check Grafana Access
```bash
curl -s http://localhost:3000/api/health | jq '.'
```

**Expected**: `{"database": "ok", "version": "..."}`

---

### 5. Test Stop/Start Cycle
```bash
cd /home/mnsuser/Cosys_Airsim_Exploration/docker

# Stop everything
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down

# Wait 5 seconds
sleep 5

# Start everything
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d

# Check Prometheus targets after restart
sleep 10
curl -s http://localhost:9090/api/v1/targets | jq '.data.activeTargets[].health' | sort | uniq -c
```

**Expected**: All targets showing "up" after restart cycle

---

## Rollback (If Needed)

If you need to go back to the separate monitoring stack:

```bash
# Stop unified stack
cd /home/mnsuser/Cosys_Airsim_Exploration/docker
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down

# Start ecosystem only
docker compose -f docker-compose-master.yml --profile linux-integrated up -d

# Start separate monitoring
cd monitoring
docker compose -f docker-compose-integrated.yml up -d
```

**Note**: Remember the manual shutdown order issue still exists in the old setup.

---

## Benefits of Unified Stack

1. **Automatic Dependency Management**
   - Monitoring waits for ecosystem to be ready
   - Shutdown happens in reverse order automatically

2. **Single Command Operation**
   - One command to start everything
   - One command to stop everything
   - No manual coordination needed

3. **No Network Issues**
   - Networks managed by single compose project
   - No "network still in use" errors
   - Clean startup/shutdown cycles

4. **Profile Flexibility**
   - Can enable/disable monitoring with a flag
   - Can start monitoring separately if needed
   - Can use integrated or linux-integrated profiles

5. **Consistent Configuration**
   - All environment variables in one place
   - Centralized network definitions
   - Unified volume management

---

## Quick Reference

### Essential Commands

```bash
# Start everything
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring up -d

# Stop everything
docker compose -f docker-compose-master.yml --profile linux-integrated --profile monitoring down

# Restart monitoring only
docker compose -f docker-compose-master.yml restart prometheus grafana

# View monitoring logs
docker compose -f docker-compose-master.yml logs -f prometheus grafana

# Check status
docker compose -f docker-compose-master.yml ps
```

### Monitoring Access

- **Grafana**: http://localhost:3000 (admin/P@ssw0rd)
- **Prometheus**: http://localhost:9090
- **Alerts**: http://localhost:9090/alerts
- **Targets**: http://localhost:9090/targets
- **cAdvisor**: http://localhost:8080

---

## Support

For issues or questions:
1. Check Prometheus targets: http://localhost:9090/targets
2. Check Grafana datasource: Configuration → Data Sources → prometheus → Save & Test
3. Review container logs: `docker compose -f docker-compose-master.yml logs <service-name>`
4. See `GRAFANA_SETUP_GUIDE.md` for dashboard creation
5. See `QUICKSTART.md` for general monitoring setup

---

**Status**: ✅ Migration complete - unified monitoring stack operational
