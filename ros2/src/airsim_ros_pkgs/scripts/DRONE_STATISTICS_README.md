# Comprehensive Drone Statistics Tracking System

## Table of Contents
- [System Overview](#system-overview)
- [ROS2 Topics Being Tracked](#ros2-topics-being-tracked)
- [PostgreSQL Database Schema](#postgresql-database-schema)
- [Data Storage Architecture](#data-storage-architecture)
- [Data Retrieval and Analysis](#data-retrieval-and-analysis)
- [Practical Usage Examples](#practical-usage-examples)
- [Performance and Optimization](#performance-and-optimization)
- [Configuration and Setup](#configuration-and-setup)
- [Troubleshooting](#troubleshooting)

## System Overview

The Drone Statistics Tracking System is a comprehensive data collection and analysis pipeline designed for multi-vehicle ROS2 drone missions. It automatically discovers active vehicles in the system and tracks detailed movement statistics, mission progress, target detections, and flight patterns.

### Key Features
- **SEARCH Auto-Discovery**: Automatically detects and subscribes to new vehicles joining the network
- **STATS Real-Time Collection**: Continuous data collection from multiple ROS2 topics
- **[EMOJI]️ Robust Storage**: High-performance PostgreSQL with TimescaleDB optimization
- **GRAPH Advanced Analytics**: Built-in analysis tools and visualization capabilities
- **[EMOJI] Performance Optimized**: Efficient sampling and batching for high-frequency data
- **REFRESH Cross-Platform**: Supports SQLite (development) and PostgreSQL (production)

### Architecture Flow
```
ROS2 Topics → drone_stats_logger.py → PostgreSQL/TimescaleDB → stats_analyzer.py → Reports/Visualizations
```

---

## ROS2 Topics Being Tracked

The system monitors 4 key topic types per vehicle, providing comprehensive mission coverage:

### 1. Mission Events (`/{vehicle_name}/mission/events`)
**Message Type**: `mission_search_interfaces/MissionEvent`

Tracks all significant vehicle activities and state changes.

**Key Data Fields**:
- **Event Classification**:
  - `event_type`: MOVEMENT, TAKEOFF, LANDING, MODE_CHANGE, COMMAND_START, COMMAND_END
  - `event_source`: DIRECT_CLIENT, ROS2_ACTION, AUTONOMOUS, MANUAL, UNKNOWN
  - `sequence_number`: Incremental ordering for event correlation

- **Position Tracking**:
  - `previous_position`, `current_position`: 3D position before/after event
  - `previous_velocity`, `current_velocity`: Velocity vectors
  - `previous_altitude`, `current_altitude`: Altitude measurements

- **Movement Analysis**:
  - `distance_moved`: Distance traveled since last event (meters)
  - `altitude_change`: Altitude change since last event (meters)
  - `speed_change`: Speed change magnitude (m/s)
  - `time_since_last_event`: Duration between events

- **Mission Context**:
  - `active_mission_id`: Current mission identifier
  - `mission_phase`: PREPARATION, EXECUTION, INVESTIGATION, RETURNING, IDLE
  - `mission_progress_percentage`: Mission completion (0.0-100.0)

- **Detection Triggers**:
  - `significant_position_change`: Position moved beyond threshold
  - `significant_velocity_change`: Velocity changed beyond threshold
  - `significant_altitude_change`: Altitude changed beyond threshold
  - `mode_transition`: Flight mode or state changed

### 2. Mission Status (`/{vehicle_name}/mission/status`)
**Message Type**: `mission_search_interfaces/MissionStatus`

Provides periodic mission progress updates and current status.

**Key Data Fields**:
- **Mission State**:
  - `status`: IDLE(0), PLANNING(1), ACTIVE(2), PAUSED(3), COMPLETED(4), FAILED(5), CANCELLED(6)
  - `mission_id`: Unique mission identifier
  - `current_activity`: Human-readable activity description

- **Progress Metrics**:
  - `progress_percentage`: Overall mission progress (0.0-100.0)
  - `area_covered_sq_m`: Area covered so far (square meters)
  - `targets_detected`: Number of targets found
  - `waypoints_completed`: Progress through waypoints

- **Timing Information**:
  - `mission_start_time`: Mission start timestamp
  - `estimated_remaining_time`: Estimated time to completion
  - `current_position`: Current 3D position

- **Error Handling**:
  - `has_error`: Error state flag
  - `error_message`: Detailed error description

### 3. Target Detections (`/{vehicle_name}/detections/target`)
**Message Type**: `mission_search_interfaces/TargetDetection`

Captures search and rescue target identification events.

**Key Data Fields**:
- **Detection Identification**:
  - `detection_id`: Unique identifier for this detection
  - `vehicle_name`: Vehicle that made the detection
  - `camera_name`: Camera/sensor used for detection

- **Target Information**:
  - `world_position`: Target position in world coordinates
  - `gps_coordinates`: GPS coordinates if available
  - `confidence_score`: Detection confidence (0.0-1.0)
  - `target_type`: person, vehicle, object, unknown
  - `target_description`: Human-readable description

- **Detection Context**:
  - `detection_altitude`: Vehicle altitude when detected
  - `detection_distance`: Estimated distance to target
  - `detection_bearing`: Bearing from vehicle to target (degrees)

- **Validation Status**:
  - `verified`: Has detection been verified by human/secondary sensor
  - `false_positive`: Marked as false positive
  - `notes`: Additional notes about detection

### 4. Odometry (`/{vehicle_name}/odom_local_ned`)
**Message Type**: `nav_msgs/Odometry`

High-frequency position and velocity data for precise tracking.

**Key Data Fields**:
- **Position**: 3D position (x, y, z) in local NED coordinates
- **Velocity**: Linear velocity vector (vx, vy, vz)
- **Orientation**: Quaternion orientation (w, x, y, z)
- **Sampling**: Intelligently sampled at 1:5 ratio to prevent database overflow
- **Usage**: Precise distance calculations, flight time tracking, path analysis

---

## PostgreSQL Database Schema

The system uses 5 optimized tables with TimescaleDB hypertables for time-series performance:

### 1. `vehicle_statistics` - Vehicle Summary Table
**Purpose**: Aggregated statistics per vehicle for quick fleet overviews.

```sql
CREATE TABLE drone_stats.vehicle_statistics (
    vehicle_name TEXT PRIMARY KEY,
    total_distance_m REAL DEFAULT 0.0,          -- Total distance traveled
    flight_time_s REAL DEFAULT 0.0,             -- Total flight time
    events_count INTEGER DEFAULT 0,             -- Total events recorded
    targets_detected INTEGER DEFAULT 0,         -- Total targets found
    waypoints_completed INTEGER DEFAULT 0,      -- Total waypoints completed
    area_covered_m2 REAL DEFAULT 0.0,          -- Total area covered
    last_updated TIMESTAMPTZ DEFAULT NOW(),     -- Last update timestamp
    created_at TIMESTAMPTZ DEFAULT NOW()        -- First seen timestamp
);
```

### 2. `movement_events` - Mission Events (TimescaleDB Hypertable)
**Purpose**: Detailed mission events from ROS2 with full context.

```sql
CREATE TABLE drone_stats.movement_events (
    vehicle_name TEXT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    event_type TEXT NOT NULL,                    -- TAKEOFF, LANDING, MOVEMENT, etc.
    event_source TEXT,                           -- DIRECT_CLIENT, ROS2_ACTION, etc.
    sequence_number BIGINT,                      -- Event ordering
    
    -- Movement metrics
    distance_moved REAL,                         -- Distance since last event (m)
    altitude_change REAL,                        -- Altitude change (m)
    speed_change REAL,                           -- Speed change (m/s)
    
    -- Position data
    position_x REAL,                             -- Current position
    position_y REAL,
    position_z REAL,
    previous_x REAL,                             -- Previous position
    previous_y REAL,
    previous_z REAL,
    
    -- Velocity data
    velocity_x REAL,                             -- Current velocity
    velocity_y REAL,
    velocity_z REAL,
    
    -- Mission context
    mission_id TEXT,                             -- Associated mission
    mission_phase TEXT,                          -- Mission phase
    mission_progress REAL,                       -- Progress percentage
    
    -- Metadata
    confidence_score REAL,                       -- Event confidence
    tags JSONB,                                  -- Custom tags
    
    PRIMARY KEY (vehicle_name, timestamp, sequence_number)
);
SELECT create_hypertable('drone_stats.movement_events', 'timestamp');
```

### 3. `mission_events` - Mission Status Updates (TimescaleDB Hypertable)
**Purpose**: Mission progress tracking and status updates.

```sql
CREATE TABLE drone_stats.mission_events (
    vehicle_name TEXT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    mission_id TEXT,                             -- Mission identifier
    progress_percentage REAL,                    -- Mission progress (0-100)
    current_activity TEXT,                       -- Current activity description
    waypoints_completed INTEGER,                 -- Waypoints completed
    area_covered REAL,                          -- Area covered (sq meters)
    targets_detected INTEGER,                    -- Targets found
    mission_start_time TIMESTAMPTZ,             -- Mission start time
    estimated_remaining_time INTERVAL,          -- Time remaining estimate
    mission_status INTEGER,                      -- Status code (0-6)
    has_error BOOLEAN DEFAULT FALSE,             -- Error flag
    error_message TEXT,                          -- Error description
    
    PRIMARY KEY (vehicle_name, timestamp)
);
SELECT create_hypertable('drone_stats.mission_events', 'timestamp');
```

### 4. `target_detections` - Target Detection Events (TimescaleDB Hypertable)
**Purpose**: Search and rescue target identification records.

```sql
CREATE TABLE drone_stats.target_detections (
    vehicle_name TEXT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    detection_id TEXT NOT NULL,                  -- Unique detection ID
    target_type TEXT,                            -- person, vehicle, object, unknown
    confidence_score REAL,                       -- Detection confidence (0-1)
    
    -- World coordinates
    world_position_x REAL,                       -- Target world position
    world_position_y REAL,
    world_position_z REAL,
    
    -- GPS coordinates
    gps_lat REAL,                               -- GPS latitude
    gps_lon REAL,                               -- GPS longitude
    gps_alt REAL,                               -- GPS altitude
    
    -- Detection context
    detection_altitude REAL,                     -- Vehicle altitude at detection
    detection_distance REAL,                     -- Distance to target
    detection_bearing REAL,                      -- Bearing to target (degrees)
    
    -- Validation
    verified BOOLEAN DEFAULT FALSE,              -- Human verified
    false_positive BOOLEAN DEFAULT FALSE,       -- Marked as false positive
    notes TEXT,                                 -- Additional notes
    
    PRIMARY KEY (vehicle_name, timestamp, detection_id)
);
SELECT create_hypertable('drone_stats.target_detections', 'timestamp');
```

### 5. `position_history` - High-Frequency Odometry (TimescaleDB Hypertable)
**Purpose**: Detailed position and velocity history for path analysis.

```sql
CREATE TABLE drone_stats.position_history (
    vehicle_name TEXT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    position_x REAL,                            -- Position coordinates
    position_y REAL,
    position_z REAL,
    velocity_x REAL,                            -- Velocity components
    velocity_y REAL,
    velocity_z REAL,
    orientation_w REAL,                         -- Quaternion orientation
    orientation_x REAL,
    orientation_y REAL,
    orientation_z REAL,
    
    PRIMARY KEY (vehicle_name, timestamp)
);
SELECT create_hypertable('drone_stats.position_history', 'timestamp');
```

---

## Data Storage Architecture

### TimescaleDB Optimization
The system leverages TimescaleDB's time-series capabilities:

- **Hypertables**: 4 tables optimized for time-series queries
- **Automatic Partitioning**: Data partitioned by time for optimal performance
- **Compression**: Automatic compression for older data
- **Indexing**: Optimized indexes for common query patterns

### Performance Indexes
```sql
-- Optimized indexes for common queries
CREATE INDEX idx_movement_vehicle_time ON movement_events(vehicle_name, timestamp DESC);
CREATE INDEX idx_movement_event_type ON movement_events(event_type);
CREATE INDEX idx_mission_vehicle_time ON mission_events(vehicle_name, timestamp DESC);
CREATE INDEX idx_detection_vehicle_time ON target_detections(vehicle_name, timestamp DESC);
CREATE INDEX idx_detection_type ON target_detections(target_type);
CREATE INDEX idx_position_vehicle_time ON position_history(vehicle_name, timestamp DESC);
```

### Data Sampling Strategy
- **Movement Events**: Full capture of all significant events
- **Mission Status**: Periodic updates (configurable interval)
- **Target Detections**: Full capture of all detections
- **Odometry**: Intelligently sampled at 1:5 ratio to prevent overflow

### Cross-Platform Support
- **Development**: SQLite with simplified schema
- **Production**: PostgreSQL with TimescaleDB optimization
- **Automatic Detection**: Environment-based database selection

---

## Data Retrieval and Analysis

### 1. Direct SQL Queries

#### Fleet Summary Query
```sql
-- Get fleet-wide statistics
SELECT 
    COUNT(*) as total_vehicles,
    SUM(total_distance_m) as fleet_distance,
    SUM(flight_time_s) as fleet_flight_time,
    SUM(targets_detected) as fleet_targets,
    AVG(total_distance_m) as avg_vehicle_distance
FROM drone_stats.vehicle_statistics;
```

#### Vehicle Performance Analysis
```sql
-- Analyze vehicle performance over time
SELECT 
    DATE_TRUNC('hour', timestamp) as hour,
    vehicle_name,
    COUNT(*) as events,
    SUM(distance_moved) as distance_per_hour,
    AVG(confidence_score) as avg_confidence
FROM drone_stats.movement_events
WHERE timestamp >= NOW() - INTERVAL '24 hours'
GROUP BY hour, vehicle_name
ORDER BY hour DESC, vehicle_name;
```

#### Mission Efficiency Metrics
```sql
-- Calculate mission efficiency
SELECT 
    mission_id,
    vehicle_name,
    MAX(progress_percentage) as max_progress,
    MAX(area_covered) as area_covered,
    COUNT(DISTINCT targets_detected) as unique_targets,
    MAX(waypoints_completed) as waypoints_done,
    EXTRACT(EPOCH FROM (MAX(timestamp) - MIN(timestamp)))/3600 as mission_hours
FROM drone_stats.mission_events
WHERE mission_id IS NOT NULL
GROUP BY mission_id, vehicle_name
ORDER BY mission_hours DESC;
```

#### Target Detection Success Rate
```sql
-- Analyze target detection patterns
SELECT 
    target_type,
    COUNT(*) as total_detections,
    COUNT(*) FILTER (WHERE verified = true) as verified_detections,
    COUNT(*) FILTER (WHERE false_positive = true) as false_positives,
    AVG(confidence_score) as avg_confidence,
    ROUND(100.0 * COUNT(*) FILTER (WHERE verified = true) / COUNT(*), 2) as success_rate
FROM drone_stats.target_detections
WHERE timestamp >= NOW() - INTERVAL '7 days'
GROUP BY target_type
ORDER BY total_detections DESC;
```

### 2. Stats Analyzer CLI Tool

The `stats_analyzer.py` provides comprehensive analysis capabilities:

#### Basic Usage
```bash
# Get help and available commands
python3 stats_analyzer.py --help

# Vehicle summary
python3 stats_analyzer.py summary --vehicle Droan1

# Get events for specific time range
python3 stats_analyzer.py events --last-hours 24 --export csv

# Fleet-wide statistics
python3 stats_analyzer.py fleet-summary --export json

# Real-time monitoring
python3 stats_analyzer.py monitor --live --refresh-rate 5
```

#### Advanced Analysis
```bash
# Plot distance over time
python3 stats_analyzer.py plot distance --vehicle Droan1 --hours 12

# Analyze mission patterns
python3 stats_analyzer.py missions --mission-id "search_mission_001" --detailed

# Target detection analysis
python3 stats_analyzer.py targets --confidence-threshold 0.8 --verified-only

# Export comprehensive report
python3 stats_analyzer.py export --format json --include-all --output fleet_report.json
```

#### Programmatic Access
```python
from database_interface import DatabaseInterface
from stats_analyzer import DroneStatsAnalyzer

# Initialize analyzer
db = DatabaseInterface(db_type='postgresql')
analyzer = DroneStatsAnalyzer(db)

# Get all active vehicles
vehicles = analyzer.get_all_vehicles()
print(f"Active vehicles: {vehicles}")

# Get vehicle summary
for vehicle in vehicles:
    summary = analyzer.get_vehicle_summary(vehicle)
    if summary:
        print(f"{vehicle}: {summary.total_distance:.1f}m, {summary.flight_time:.1f}s")

# Get recent events
events = analyzer.get_events(vehicle_name="Droan1", hours_back=24)
print(f"Recent events: {len(events)}")

# Export data
analyzer.export_data("fleet_data.csv", format="csv")
```

---

## Practical Usage Examples

### Mission Performance Analysis

#### 1. Flight Efficiency Analysis
```sql
-- Calculate flight efficiency metrics
WITH flight_segments AS (
    SELECT 
        vehicle_name,
        timestamp,
        distance_moved,
        LAG(timestamp) OVER (PARTITION BY vehicle_name ORDER BY timestamp) as prev_time
    FROM drone_stats.movement_events
    WHERE event_type = 'MOVEMENT' AND distance_moved > 0
)
SELECT 
    vehicle_name,
    COUNT(*) as segments,
    SUM(distance_moved) as total_distance,
    AVG(distance_moved / EXTRACT(EPOCH FROM (timestamp - prev_time))) as avg_speed_mps,
    MAX(distance_moved / EXTRACT(EPOCH FROM (timestamp - prev_time))) as max_speed_mps
FROM flight_segments
WHERE prev_time IS NOT NULL
GROUP BY vehicle_name
ORDER BY total_distance DESC;
```

#### 2. Search Pattern Analysis
```sql
-- Analyze search coverage patterns
SELECT 
    vehicle_name,
    COUNT(*) as detection_attempts,
    COUNT(DISTINCT CONCAT(ROUND(world_position_x/10), '_', ROUND(world_position_y/10))) as grid_cells_searched,
    AVG(confidence_score) as avg_confidence,
    COUNT(*) FILTER (WHERE verified = true) as successful_finds
FROM drone_stats.target_detections
WHERE timestamp >= NOW() - INTERVAL '1 day'
GROUP BY vehicle_name
ORDER BY grid_cells_searched DESC;
```

### Fleet Coordination Analysis

#### 3. Multi-Vehicle Mission Coordination
```sql
-- Analyze fleet coordination during missions
WITH mission_timeline AS (
    SELECT 
        mission_id,
        vehicle_name,
        MIN(timestamp) as start_time,
        MAX(timestamp) as end_time,
        MAX(progress_percentage) as final_progress
    FROM drone_stats.mission_events
    WHERE mission_id IS NOT NULL
    GROUP BY mission_id, vehicle_name
)
SELECT 
    mission_id,
    COUNT(*) as vehicles_involved,
    MIN(start_time) as mission_start,
    MAX(end_time) as mission_end,
    AVG(final_progress) as avg_progress,
    EXTRACT(EPOCH FROM (MAX(end_time) - MIN(start_time)))/60 as duration_minutes
FROM mission_timeline
GROUP BY mission_id
ORDER BY duration_minutes DESC;
```

### Real-Time Monitoring Queries

#### 4. Live Fleet Dashboard
```sql
-- Real-time fleet status
SELECT 
    vs.vehicle_name,
    vs.total_distance_m,
    vs.flight_time_s,
    vs.targets_detected,
    CASE 
        WHEN vs.last_updated > NOW() - INTERVAL '5 minutes' THEN 'ACTIVE'
        WHEN vs.last_updated > NOW() - INTERVAL '1 hour' THEN 'RECENT'
        ELSE 'INACTIVE'
    END as status,
    me.current_activity,
    me.progress_percentage
FROM drone_stats.vehicle_statistics vs
LEFT JOIN LATERAL (
    SELECT current_activity, progress_percentage
    FROM drone_stats.mission_events 
    WHERE vehicle_name = vs.vehicle_name
    ORDER BY timestamp DESC
    LIMIT 1
) me ON true
ORDER BY vs.last_updated DESC;
```

---

## Performance and Optimization

### Database Performance Tuning

#### TimescaleDB Configuration
```sql
-- Optimize TimescaleDB settings
ALTER SYSTEM SET shared_preload_libraries = 'timescaledb';
ALTER SYSTEM SET max_connections = 200;
ALTER SYSTEM SET shared_buffers = '256MB';
ALTER SYSTEM SET effective_cache_size = '1GB';
ALTER SYSTEM SET work_mem = '4MB';
ALTER SYSTEM SET maintenance_work_mem = '64MB';
```

#### Data Retention Policy
```sql
-- Set up data retention for older data
SELECT add_retention_policy('drone_stats.position_history', INTERVAL '30 days');
SELECT add_retention_policy('drone_stats.movement_events', INTERVAL '90 days');
SELECT add_compression_policy('drone_stats.position_history', INTERVAL '7 days');
```

### Application Performance

#### Connection Pooling
The system uses connection pooling for optimal performance:
- **Minimum Connections**: 1
- **Maximum Connections**: 10
- **Connection Timeout**: 30 seconds
- **Automatic Retry**: Built-in retry logic for transient failures

#### Sampling Strategy
- **Odometry**: 1:5 sampling ratio (configurable)
- **Batch Processing**: Events batched for database efficiency
- **Background Processing**: Non-blocking database operations

---

## Configuration and Setup

### Environment Variables

#### Database Configuration
```bash
# PostgreSQL Configuration
export DATABASE_TYPE=postgresql
export POSTGRES_HOST=postgres-drone-stats
export POSTGRES_PORT=5432
export POSTGRES_DB=drone_statistics
export POSTGRES_USER=airsim_user
export POSTGRES_PASSWORD=your_secure_postgres_password_here
export POSTGRES_SCHEMA=drone_stats

# SQLite Configuration (Development)
export DATABASE_TYPE=sqlite
export SQLITE_DATABASE_PATH=./drone_movement_stats.db

# Logger Configuration
export STATS_UPDATE_RATE=10.0          # Statistics update interval (seconds)
export DISCOVERY_RATE=30.0             # Vehicle discovery interval (seconds)
export EXPORT_ON_SHUTDOWN=true         # Export data on shutdown
```

### Docker Integration

#### Using with PostgreSQL Container
```yaml
version: '3.8'
services:
  postgres-stats:
    image: timescale/timescaledb:latest-pg15
    environment:
      POSTGRES_DB: drone_statistics
      POSTGRES_USER: airsim_user
      POSTGRES_PASSWORD: ${POSTGRES_PASSWORD}  # Set via environment variable
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  ros2-stats:
    image: tevv-airstack-ros2-multi-node
    depends_on:
      - postgres-stats
    environment:
      DATABASE_TYPE: postgresql
      POSTGRES_HOST: postgres-stats
    command: ros2 run airsim_ros_pkgs drone_stats_logger.py
```

### ROS2 Launch Integration
```xml
<launch>
  <!-- Drone Statistics Logger -->
  <node pkg="airsim_ros_pkgs" exec="drone_stats_logger.py" name="drone_stats_logger">
    <param name="database_type" value="postgresql"/>
    <param name="stats_update_rate" value="10.0"/>
    <param name="discovery_rate" value="30.0"/>
    <param name="export_on_shutdown" value="true"/>
  </node>
</launch>
```

---

## Troubleshooting

### Common Issues and Solutions

#### 1. Database Connection Failures
**Symptoms**: "Database connectivity: FAILED" in health checks
**Solutions**:
```bash
# Check database container status
docker ps | grep postgres

# Test network connectivity
docker exec ros2-node ping postgres-drone-stats

# Verify database credentials
docker exec postgres-stats psql -U airsim_user -d drone_statistics -c "SELECT 1;"

# Check health status
python3 health_utils.py --quick
```

#### 2. Missing Tables Error
**Symptoms**: "Schema: error - No required tables found"
**Solutions**:
```bash
# Force table creation
python3 -c "from database_interface import DatabaseInterface, create_tables_if_not_exist; db = DatabaseInterface(); create_tables_if_not_exist(db)"

# Verify tables exist
docker exec postgres-stats psql -U airsim_user -d drone_statistics -c "\dt drone_stats.*"
```

#### 3. Vehicle Discovery Issues
**Symptoms**: No vehicles detected in logs
**Solutions**:
```bash
# Check ROS2 node discovery
ros2 node list | grep -i drone

# Verify topic publications
ros2 topic list | grep -E "(mission|detection|odom)"

# Monitor topic activity
ros2 topic hz /Droan1/mission/events
```

#### 4. High Memory Usage
**Symptoms**: System running out of memory
**Solutions**:
```bash
# Adjust odometry sampling rate
export ODOM_SAMPLE_RATIO=10  # Sample 1 in 10 messages

# Enable data compression
psql -U airsim_user -d drone_statistics -c "SELECT add_compression_policy('drone_stats.position_history', INTERVAL '1 hour');"

# Set up data retention
psql -U airsim_user -d drone_statistics -c "SELECT add_retention_policy('drone_stats.position_history', INTERVAL '7 days');"
```

### Health Monitoring

#### System Health Check
```bash
# Comprehensive health check
python3 health_utils.py

# Quick connectivity test
python3 health_utils.py --quick

# PostgreSQL-specific status
python3 -m mnstevv postgres status
```

#### Performance Monitoring
```sql
-- Monitor database performance
SELECT 
    schemaname,
    tablename,
    attname,
    n_distinct,
    correlation
FROM pg_stats 
WHERE schemaname = 'drone_stats'
ORDER BY tablename, attname;

-- Check TimescaleDB hypertable status
SELECT 
    hypertable_name,
    num_chunks,
    total_size_bytes,
    compressed_total_size_bytes
FROM timescaledb_information.hypertables;
```

---

## Conclusion

The Drone Statistics Tracking System provides comprehensive data collection, storage, and analysis capabilities for multi-vehicle drone operations. With its robust PostgreSQL/TimescaleDB backend, intelligent sampling strategies, and comprehensive analysis tools, it enables detailed mission performance analysis, fleet coordination insights, and real-time operational monitoring.

For additional support or feature requests, please refer to the system logs and health monitoring tools described above.