# Drone Statistics System - Complete Guide

This comprehensive guide covers the entire drone statistics collection, storage, and analysis system integrated with the AirSim ROS2 ecosystem.

## Table of Contents

1. [System Architecture Overview](#system-architecture-overview)
2. [Database Integration](#database-integration)
3. [Data Collection Process](#data-collection-process)  
4. [Data Analysis & Querying](#data-analysis--querying)
5. [Practical Usage Examples](#practical-usage-examples)
6. [Database Schema Reference](#database-schema-reference)
7. [Integration with Mission System](#integration-with-mission-system)
8. [Troubleshooting Guide](#troubleshooting-guide)
9. [Performance Optimization](#performance-optimization)

## System Architecture Overview

### Data Flow Diagram
```
AirSim Vehicles (Windows) 
    ↓ RPC Communication
ROS2 Multi-Node System (Docker)
    ↓ Topic Subscriptions (/vehicle_name/odom, /vehicle_name/mission/events, etc.)
drone_stats_logger.py
    ↓ Real-time Processing & Aggregation
Database Layer (SQLite/PostgreSQL)
    ↓ Queries & Analysis
stats_analyzer.py
    ↓ Reports & Visualizations
CSV/JSON/Charts Output
```

### Core Components

#### 1. Data Collection Layer
- **drone_stats_logger.py**: ROS2 node for real-time data collection
- **Topic Subscribers**: Automatic vehicle discovery and subscription
- **Event Processing**: Mission events, odometry, IMU, GPS data
- **Real-time Aggregation**: Statistics calculation and storage

#### 2. Database Abstraction Layer  
- **database_interface.py**: Unified SQLite/PostgreSQL interface
- **Auto-Detection**: Environment-based database selection
- **Connection Pooling**: PostgreSQL connection management
- **Schema Migration**: Seamless database upgrades

#### 3. Analysis & Reporting Layer
- **stats_analyzer.py**: CLI tool for data analysis and reporting
- **Export Formats**: CSV, JSON, plotting capabilities
- **Real-time Monitoring**: Live fleet status monitoring
- **Performance Analytics**: Flight metrics and efficiency analysis

#### 4. Health Monitoring Layer
- **health_utils.py**: Comprehensive database health monitoring
- **Connection Testing**: Real-time connectivity validation
- **Performance Metrics**: Database size, query performance, etc.
- **CLI Integration**: Built into MNSTEVV postgres status command

## Database Integration

### Multi-Database Support

The system supports both SQLite (development) and PostgreSQL with TimescaleDB (production) through a unified interface.

#### Database Selection Logic
```python
# Automatic detection based on environment
db = DatabaseInterface()  # Auto-detects from environment variables

# Manual specification
db = DatabaseInterface(db_type='postgresql')
db = DatabaseInterface(db_type='sqlite')
```

#### Environment Variables
```bash
# PostgreSQL Configuration
DATABASE_TYPE=postgresql
POSTGRES_HOST=localhost
POSTGRES_PORT=5432
POSTGRES_DB=drone_statistics
POSTGRES_USER=airsim_user
POSTGRES_PASSWORD=airsim_stats_2024
POSTGRES_SCHEMA=drone_stats

# SQLite Configuration (default)
DATABASE_TYPE=sqlite
SQLITE_PATH=drone_movement_stats.db
```

### PostgreSQL with TimescaleDB Setup

#### Docker Container Start
```bash
# Start PostgreSQL with TimescaleDB
docker-compose -f docker-compose-master.yml --profile integrated up postgres-stats

# Or using CLI
cd docker/cli && python -m mnstevv postgres up
```

#### Database Initialization
The PostgreSQL database is automatically configured with:
- **TimescaleDB Extension**: For time-series optimization
- **Custom Schemas**: `drone_stats`, `monitoring`, `audit`
- **Custom Types**: `event_type`, `vehicle_status`, `mission_status`
- **Helper Functions**: Distance calculations, statistics updates
- **Hypertables**: Automatic time-series partitioning

#### Access Information
- **PostgreSQL**: `localhost:5432` (user: `airsim_user`)
- **pgAdmin**: `http://localhost:8080` (admin@example.com)

## Data Collection Process

### ROS2 Topic Integration

The drone_stats_logger.py automatically discovers and subscribes to vehicle topics:

```python
# Vehicle Discovery Process
vehicles = self.discover_vehicles()  # Scan for /vehicle_name/* topics

# Subscribe to relevant topics for each vehicle
for vehicle_name in vehicles:
    # Odometry for position/movement tracking
    self.subscribe_to_topic(f'/{vehicle_name}/odom', Odometry, self.odom_callback)
    
    # Mission events for task tracking  
    self.subscribe_to_topic(f'/{vehicle_name}/mission/events', MissionEvent, self.mission_event_callback)
    
    # Mission status for state tracking
    self.subscribe_to_topic(f'/{vehicle_name}/mission/status', MissionStatus, self.mission_status_callback)
    
    # Target detections for mission analytics
    self.subscribe_to_topic(f'/{vehicle_name}/detections/target', TargetDetection, self.detection_callback)
```

### Event Processing Pipeline

#### 1. Odometry Processing
```python
def odom_callback(self, msg, vehicle_name):
    # Extract position and orientation data
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    # Calculate movement statistics
    distance_moved = self.calculate_distance(position, self.last_positions[vehicle_name])
    speed = self.calculate_speed(msg.twist.twist.linear)
    
    # Store in database
    self.db.insert_movement_event(
        vehicle_name=vehicle_name,
        timestamp=self.get_timestamp(msg.header),
        position=(position.x, position.y, position.z),
        distance_m=distance_moved,
        speed_ms=speed
    )
```

#### 2. Mission Event Processing  
```python
def mission_event_callback(self, msg, vehicle_name):
    # Process mission events (takeoff, landing, waypoint reached, etc.)
    event_data = {
        'vehicle_name': vehicle_name,
        'event_type': msg.event_type,
        'timestamp': self.get_timestamp(msg.header),
        'details': {
            'mission_id': msg.mission_id,
            'waypoint_id': msg.waypoint_id,
            'success': msg.success,
            'additional_data': msg.data
        }
    }
    
    self.db.insert_mission_event(**event_data)
    self.update_vehicle_statistics(vehicle_name, msg.event_type)
```

### Real-time Statistics Aggregation

The logger maintains real-time statistics for each vehicle:

```python
class VehicleStatistics:
    def __init__(self):
        self.total_distance_m = 0.0
        self.flight_time_s = 0.0
        self.events_count = 0
        self.targets_detected = 0
        self.waypoints_completed = 0
        self.area_covered_m2 = 0.0
        self.last_updated = datetime.now(timezone.utc)
    
    def update_from_movement(self, distance_m, time_delta_s):
        self.total_distance_m += distance_m
        self.flight_time_s += time_delta_s
        self.last_updated = datetime.now(timezone.utc)
    
    def update_from_event(self, event_type):
        self.events_count += 1
        if event_type == 'TARGET_DETECTED':
            self.targets_detected += 1
        elif event_type == 'WAYPOINT_REACHED':
            self.waypoints_completed += 1
        self.last_updated = datetime.now(timezone.utc)
```

## Data Analysis & Querying

### CLI Usage Examples

#### Basic Statistics Summary
```bash
# Get summary for all vehicles
python3 stats_analyzer.py summary

# Get summary for specific vehicle
python3 stats_analyzer.py summary --vehicle Droan1

# Get summary for vehicle pattern
python3 stats_analyzer.py summary --vehicle-pattern "Drone_*"
```

#### Event Analysis
```bash
# Show events from last 24 hours
python3 stats_analyzer.py events --last-hours 24

# Show mission events only
python3 stats_analyzer.py events --event-type mission --last-days 7

# Export events to CSV
python3 stats_analyzer.py events --last-hours 24 --export csv --output flight_events.csv
```

#### Performance Analytics
```bash
# Show distance analytics
python3 stats_analyzer.py performance distance --vehicle Droan1

# Show flight time analytics
python3 stats_analyzer.py performance flight-time --vehicle-pattern "PX4_*"

# Show efficiency metrics
python3 stats_analyzer.py performance efficiency --compare-vehicles
```

#### Real-time Monitoring
```bash
# Live fleet monitoring
python3 stats_analyzer.py monitor --live --update-interval 5

# Monitor specific vehicles
python3 stats_analyzer.py monitor --vehicles Droan1,PX4_Drone2 --live
```

### Programmatic API Usage

```python
from stats_analyzer import DroneStatsAnalyzer

# Initialize analyzer
analyzer = DroneStatsAnalyzer()

# Get vehicle statistics
stats = analyzer.get_vehicle_statistics('Droan1')
print(f"Total distance: {stats['total_distance_m']:.1f}m")
print(f"Flight time: {stats['flight_time_s']:.1f}s")

# Get events in time range
events = analyzer.get_events(
    start_time=datetime.now() - timedelta(hours=24),
    end_time=datetime.now(),
    vehicle_name='Droan1'
)

# Export data
analyzer.export_to_csv('flight_data.csv', events)
```

### Query Examples

#### Complex Analytics Queries
```sql
-- Vehicle efficiency comparison
SELECT 
    vehicle_name,
    total_distance_m,
    flight_time_s,
    (total_distance_m / NULLIF(flight_time_s, 0)) as efficiency_m_per_s,
    targets_detected,
    waypoints_completed
FROM drone_stats.vehicle_statistics 
ORDER BY efficiency_m_per_s DESC;

-- Mission success rate analysis
SELECT 
    vehicle_name,
    COUNT(*) as total_missions,
    COUNT(CASE WHEN event_type = 'MISSION_COMPLETE' THEN 1 END) as completed_missions,
    ROUND(
        COUNT(CASE WHEN event_type = 'MISSION_COMPLETE' THEN 1 END) * 100.0 / COUNT(*), 
        2
    ) as success_rate_percent
FROM drone_stats.mission_events 
WHERE event_type IN ('MISSION_START', 'MISSION_COMPLETE', 'MISSION_FAILED')
GROUP BY vehicle_name;

-- Hourly activity analysis
SELECT 
    DATE_TRUNC('hour', timestamp) as hour,
    vehicle_name,
    COUNT(*) as events_count,
    SUM(COALESCE((details->>'distance_m')::DOUBLE PRECISION, 0)) as total_distance_m
FROM drone_stats.movement_events
WHERE timestamp >= NOW() - INTERVAL '24 hours'
GROUP BY hour, vehicle_name
ORDER BY hour DESC;
```

## Practical Usage Examples

### Complete Workflow: From Mission to Analysis

#### Step 1: Start the Statistics System
```bash
# Start PostgreSQL database
cd docker/cli && python -m mnstevv postgres up

# Start ROS2 system with mission capabilities
cd /airsim_ros2_ws
source install/setup.bash
ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true

# Start statistics logger
ros2 run airsim_ros_pkgs drone_stats_logger.py
```

#### Step 2: Execute a Mission
```bash
# Send search area mission to Droan1
ros2 action send_goal /Droan1/actions/search_area mission_search_interfaces/action/SearchArea "{
  search_boundary: {
    points: [
      {x: 0.0, y: 0.0, z: 0.0},
      {x: 50.0, y: 0.0, z: 0.0}, 
      {x: 50.0, y: 50.0, z: 0.0},
      {x: 0.0, y: 50.0, z: 0.0}
    ]
  },
  search_pattern: 'spiral',
  search_altitude: 25.0,
  search_speed: 5.0,
  pattern_spacing: 10.0
}"
```

#### Step 3: Monitor Real-time Statistics
```bash
# Monitor live statistics
python3 stats_analyzer.py monitor --live --vehicles Droan1

# Check database health
cd docker/cli && python -m mnstevv postgres status
```

#### Step 4: Post-Mission Analysis
```bash
# Get mission summary
python3 stats_analyzer.py summary --vehicle Droan1

# Analyze mission events
python3 stats_analyzer.py events --vehicle Droan1 --last-hours 1 --event-type mission

# Export detailed data
python3 stats_analyzer.py export --vehicle Droan1 --format csv --output droan1_mission_data.csv

# Generate performance report
python3 stats_analyzer.py performance efficiency --vehicle Droan1 --plot
```

### Integration with Multiple Vehicles

#### Fleet Coordination Analysis
```bash
# Compare vehicle performance
python3 stats_analyzer.py performance compare --vehicles Droan1,PX4_Drone2,MyDrone3

# Fleet-wide statistics
python3 stats_analyzer.py fleet-status --real-time

# Mission coordination analysis
python3 stats_analyzer.py coordination --mission-id SEARCH_001 --show-timeline
```

### Data Export and Reporting

#### Export Options
```bash
# CSV export with custom fields
python3 stats_analyzer.py export --format csv --fields "timestamp,vehicle_name,event_type,distance_m" --output custom_data.csv

# JSON export for API integration
python3 stats_analyzer.py export --format json --vehicle-pattern "Drone_*" --last-days 7 --output weekly_stats.json

# Visualization export
python3 stats_analyzer.py plot distance-over-time --vehicle Droan1 --save flight_path.png
```

## Database Schema Reference

### Core Tables

#### vehicle_statistics
Primary table for aggregated vehicle statistics:
```sql
CREATE TABLE drone_stats.vehicle_statistics (
    vehicle_name VARCHAR(100) PRIMARY KEY,
    total_distance_m DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    flight_time_s DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    events_count INTEGER DEFAULT 0 NOT NULL,
    targets_detected INTEGER DEFAULT 0 NOT NULL,
    waypoints_completed INTEGER DEFAULT 0 NOT NULL,
    area_covered_m2 DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    average_speed_ms DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    max_altitude_m DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    mission_success_rate DOUBLE PRECISION DEFAULT 0.0 NOT NULL,
    last_updated TIMESTAMPTZ DEFAULT NOW() NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW() NOT NULL
);
```

#### movement_events (TimescaleDB Hypertable)
Time-series table for position and movement tracking:
```sql
CREATE TABLE drone_stats.movement_events (
    event_id UUID DEFAULT drone_stats.generate_event_id() PRIMARY KEY,
    vehicle_name VARCHAR(100) NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    position_x DOUBLE PRECISION NOT NULL,
    position_y DOUBLE PRECISION NOT NULL, 
    position_z DOUBLE PRECISION NOT NULL,
    orientation_x DOUBLE PRECISION DEFAULT 0.0,
    orientation_y DOUBLE PRECISION DEFAULT 0.0,
    orientation_z DOUBLE PRECISION DEFAULT 0.0,
    orientation_w DOUBLE PRECISION DEFAULT 1.0,
    linear_velocity_x DOUBLE PRECISION DEFAULT 0.0,
    linear_velocity_y DOUBLE PRECISION DEFAULT 0.0,
    linear_velocity_z DOUBLE PRECISION DEFAULT 0.0,
    angular_velocity_x DOUBLE PRECISION DEFAULT 0.0,
    angular_velocity_y DOUBLE PRECISION DEFAULT 0.0,
    angular_velocity_z DOUBLE PRECISION DEFAULT 0.0,
    distance_from_previous_m DOUBLE PRECISION DEFAULT 0.0,
    cumulative_distance_m DOUBLE PRECISION DEFAULT 0.0,
    speed_ms DOUBLE PRECISION DEFAULT 0.0,
    details JSONB DEFAULT '{}',
    INDEX idx_movement_events_vehicle_time (vehicle_name, timestamp DESC)
);

-- Convert to TimescaleDB hypertable
SELECT create_hypertable('drone_stats.movement_events', 'timestamp');
```

#### mission_events (TimescaleDB Hypertable)
Mission-related events and state changes:
```sql
CREATE TABLE drone_stats.mission_events (
    event_id UUID DEFAULT drone_stats.generate_event_id() PRIMARY KEY,
    vehicle_name VARCHAR(100) NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    event_type drone_stats.event_type NOT NULL,
    mission_id VARCHAR(100),
    waypoint_id VARCHAR(100),
    success BOOLEAN DEFAULT true,
    error_message TEXT,
    details JSONB DEFAULT '{}',
    processing_time_ms INTEGER DEFAULT 0,
    INDEX idx_mission_events_vehicle_time (vehicle_name, timestamp DESC),
    INDEX idx_mission_events_type (event_type),
    INDEX idx_mission_events_mission (mission_id)
);

-- Convert to TimescaleDB hypertable  
SELECT create_hypertable('drone_stats.mission_events', 'timestamp');
```

#### position_history (TimescaleDB Hypertable)
Simplified position tracking for path analysis:
```sql
CREATE TABLE drone_stats.position_history (
    id BIGSERIAL,
    vehicle_name VARCHAR(100) NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    latitude DOUBLE PRECISION,
    longitude DOUBLE PRECISION,
    altitude_m DOUBLE PRECISION NOT NULL,
    heading_deg DOUBLE PRECISION DEFAULT 0.0,
    ground_speed_ms DOUBLE PRECISION DEFAULT 0.0,
    gps_accuracy_m DOUBLE PRECISION,
    INDEX idx_position_history_vehicle_time (vehicle_name, timestamp DESC)
);

-- Convert to TimescaleDB hypertable
SELECT create_hypertable('drone_stats.position_history', 'timestamp');
```

### Custom Types

#### event_type
```sql
CREATE TYPE drone_stats.event_type AS ENUM (
    'TAKEOFF',
    'LANDING',
    'MOVEMENT', 
    'COMMAND_START',
    'COMMAND_COMPLETE',
    'MISSION_START',
    'MISSION_COMPLETE',
    'WAYPOINT_REACHED',
    'TARGET_DETECTED',
    'COLLISION_DETECTED',
    'EMERGENCY_STOP',
    'CONNECTION_LOST',
    'CONNECTION_RESTORED'
);
```

#### vehicle_status  
```sql
CREATE TYPE drone_stats.vehicle_status AS ENUM (
    'IDLE',
    'ARMED',
    'FLYING', 
    'LANDING',
    'MISSION',
    'ERROR',
    'OFFLINE'
);
```

### Helper Functions

#### Distance Calculation
```sql
CREATE OR REPLACE FUNCTION drone_stats.calculate_distance_m(
    lat1 DOUBLE PRECISION,
    lon1 DOUBLE PRECISION, 
    lat2 DOUBLE PRECISION,
    lon2 DOUBLE PRECISION
) RETURNS DOUBLE PRECISION AS $$
DECLARE
    earth_radius CONSTANT DOUBLE PRECISION := 6371000; -- Earth radius in meters
    dlat DOUBLE PRECISION;
    dlon DOUBLE PRECISION;
    a DOUBLE PRECISION;
    c DOUBLE PRECISION;
BEGIN
    dlat := radians(lat2 - lat1);
    dlon := radians(lon2 - lon1);
    
    a := sin(dlat/2) * sin(dlat/2) + 
         cos(radians(lat1)) * cos(radians(lat2)) * 
         sin(dlon/2) * sin(dlon/2);
    
    c := 2 * atan2(sqrt(a), sqrt(1-a));
    
    RETURN earth_radius * c;
END;
$$ LANGUAGE plpgsql IMMUTABLE;
```

### Monitoring Views

#### Fleet Status View
```sql
CREATE OR REPLACE VIEW drone_stats.fleet_status AS
SELECT 
    vehicle_name,
    total_distance_m,
    flight_time_s / 60.0 AS flight_time_minutes,
    events_count,
    targets_detected,
    waypoints_completed,
    area_covered_m2,
    last_updated,
    CASE 
        WHEN last_updated > NOW() - INTERVAL '30 seconds' THEN 'ACTIVE'
        WHEN last_updated > NOW() - INTERVAL '5 minutes' THEN 'RECENT'
        ELSE 'INACTIVE'
    END AS activity_status,
    EXTRACT(EPOCH FROM (NOW() - last_updated)) AS seconds_since_update
FROM drone_stats.vehicle_statistics
ORDER BY last_updated DESC;
```

## Integration with Mission System

### ROS2 Topic Integration

The statistics system seamlessly integrates with the mission coordination system by subscribing to mission-related topics:

```bash
# Mission system topics monitored by stats logger
/PX4_Drone1/mission/status           # MissionStatus messages
/PX4_Drone1/mission/events           # MissionEvent messages  
/PX4_Drone1/detections/target        # TargetDetection messages
/mission_coordinator/zone_assignments # Zone assignment events
/mission_coordinator/mission_status   # Global mission status
```

### Mission Analytics

The system provides detailed analytics for mission performance:

#### Mission Success Rate Analysis
```python
# Get mission success rates by vehicle
success_rates = analyzer.get_mission_success_rates()
for vehicle, rate in success_rates.items():
    print(f"{vehicle}: {rate:.1f}% success rate")

# Get detailed mission timeline
timeline = analyzer.get_mission_timeline('SEARCH_MISSION_001')
for event in timeline:
    print(f"{event.timestamp}: {event.vehicle_name} - {event.event_type}")
```

#### Search Pattern Efficiency
```python
# Analyze search pattern efficiency
efficiency = analyzer.get_search_efficiency(
    mission_id='SEARCH_001',
    metrics=['coverage_percentage', 'time_to_complete', 'distance_efficiency']
)
```

### Coordination Analytics

```python
# Multi-vehicle coordination analysis
coordination_stats = analyzer.get_coordination_statistics(
    mission_id='MULTI_SEARCH_001',
    include_timeline=True
)

print(f"Vehicles involved: {coordination_stats['vehicle_count']}")
print(f"Total area covered: {coordination_stats['total_area_m2']:.1f} m²")
print(f"Mission duration: {coordination_stats['duration_minutes']:.1f} minutes")
print(f"Coordination efficiency: {coordination_stats['efficiency_score']:.2f}")
```

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. Database Connection Issues

**Problem**: `ERROR: Error connecting to database`

**Diagnosis**:
```bash
# Check database health
cd docker/cli && python -m mnstevv postgres status

# Quick connectivity test
cd ros2/src/airsim_ros_pkgs/scripts
python3 health_utils.py --quick
```

**Solutions**:
```bash
# Restart PostgreSQL services
cd docker/cli && python -m mnstevv postgres down && python -m mnstevv postgres up

# Check container logs
docker logs postgres-drone-stats

# Verify network connectivity
docker exec -it postgres-drone-stats pg_isready -U airsim_user -d drone_statistics
```

#### 2. No Data Being Logged

**Problem**: Statistics logger running but no data appears in database

**Diagnosis**:
```bash
# Check ROS2 topic availability
ros2 topic list | grep -E "(odom|mission|detections)"

# Check logger subscriptions
ros2 node info /drone_stats_logger

# Check database tables
cd docker/cli && python -m mnstevv postgres logs --service postgres --tail 50
```

**Solutions**:
```bash
# Restart statistics logger
ros2 run airsim_ros_pkgs drone_stats_logger.py --verbose

# Verify vehicle discovery
ros2 topic echo /drone_stats_logger/discovered_vehicles

# Check database schema
cd ros2/src/airsim_ros_pkgs/scripts
python3 -c "from database_interface import create_tables_if_not_exist, DatabaseInterface; create_tables_if_not_exist(DatabaseInterface())"
```

#### 3. Performance Issues

**Problem**: Slow database queries or high CPU usage

**Diagnosis**:
```bash
# Check database performance
cd docker/cli && python -m mnstevv postgres status

# Check query performance
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT query, mean_time, calls FROM pg_stat_statements 
ORDER BY mean_time DESC LIMIT 10;"
```

**Solutions**:
```bash
# Optimize database (PostgreSQL)
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
VACUUM ANALYZE;
REINDEX DATABASE drone_statistics;"

# Adjust TimescaleDB settings
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT set_config('timescaledb.max_background_workers', '8', false);
SELECT set_config('work_mem', '256MB', false);"
```

#### 4. Missing TimescaleDB Features

**Problem**: TimescaleDB extension not available

**Diagnosis**:
```bash
# Check TimescaleDB installation
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT * FROM pg_extension WHERE extname = 'timescaledb';"
```

**Solutions**:
```bash
# Install TimescaleDB extension
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
CREATE EXTENSION IF NOT EXISTS timescaledb;"

# Convert tables to hypertables
docker exec -it postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT create_hypertable('drone_stats.movement_events', 'timestamp', if_not_exists => TRUE);
SELECT create_hypertable('drone_stats.mission_events', 'timestamp', if_not_exists => TRUE);"
```

### Health Check Commands

#### Comprehensive System Check
```bash
# Full health check
cd ros2/src/airsim_ros_pkgs/scripts && python3 health_utils.py

# Quick connectivity test
cd ros2/src/airsim_ros_pkgs/scripts && python3 health_utils.py --quick

# Database-specific health check
cd docker/cli && python -m mnstevv postgres status
```

#### Performance Monitoring
```bash
# Monitor database performance
watch -n 5 'docker exec postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT 
    count(*) as active_connections,
    pg_size_pretty(pg_database_size(current_database())) as db_size,
    (SELECT count(*) FROM drone_stats.vehicle_statistics) as vehicles,
    (SELECT count(*) FROM drone_stats.movement_events WHERE timestamp > NOW() - INTERVAL '"'"'1 hour'"'"') as recent_events
"'

# Monitor ROS2 performance
ros2 topic hz /drone_stats_logger/statistics_update
```

### Log Analysis

#### Database Logs
```bash
# View PostgreSQL logs
cd docker/cli && python -m mnstevv postgres logs --follow

# View specific service logs
docker logs postgres-drone-stats --tail 100 --follow
```

#### Application Logs
```bash
# View stats logger logs
ros2 run airsim_ros_pkgs drone_stats_logger.py --verbose

# View analyzer logs
python3 stats_analyzer.py monitor --debug
```

## Performance Optimization

### Database Optimization

#### TimescaleDB Configuration
```sql
-- Optimize TimescaleDB for drone data
SELECT set_config('timescaledb.max_background_workers', '4', false);
SELECT set_config('timescaledb.enable_transparent_decompression', 'on', false);

-- Configure compression for older data
ALTER TABLE drone_stats.movement_events SET (
    timescaledb.compress,
    timescaledb.compress_segmentby = 'vehicle_name',
    timescaledb.compress_orderby = 'timestamp DESC'
);

-- Enable automatic compression for data older than 7 days
SELECT add_compression_policy('drone_stats.movement_events', INTERVAL '7 days');
```

#### Indexing Strategy
```sql
-- Optimize common query patterns
CREATE INDEX CONCURRENTLY idx_movement_events_vehicle_recent 
ON drone_stats.movement_events (vehicle_name, timestamp DESC) 
WHERE timestamp > NOW() - INTERVAL '24 hours';

CREATE INDEX CONCURRENTLY idx_mission_events_success 
ON drone_stats.mission_events (vehicle_name, success, timestamp DESC)
WHERE event_type IN ('MISSION_START', 'MISSION_COMPLETE');

-- Partial indexes for active data
CREATE INDEX CONCURRENTLY idx_vehicle_stats_active
ON drone_stats.vehicle_statistics (last_updated DESC)
WHERE last_updated > NOW() - INTERVAL '1 hour';
```

### Application Optimization

#### Data Collection Tuning
```python
# Configure logger for high-frequency data
class OptimizedStatsLogger(DroneMovementStatsLogger):
    def __init__(self):
        super().__init__()
        
        # Batch processing for better performance
        self.batch_size = 100
        self.batch_timeout = 5.0  # seconds
        
        # Sampling for high-frequency topics
        self.odometry_sample_rate = 0.1  # 10% sampling
        self.position_update_threshold = 0.5  # meters
```

#### Memory Management
```python
# Configure connection pooling
config = DatabaseConfig(
    db_type=DatabaseType.POSTGRESQL,
    min_connections=2,
    max_connections=10,
    connection_timeout=30
)

db = DatabaseInterface(config=config)
```

### Monitoring and Maintenance

#### Automated Maintenance
```bash
# Daily maintenance script
#!/bin/bash
# maintenance.sh

# Vacuum and analyze tables
docker exec postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
VACUUM ANALYZE drone_stats.movement_events;
VACUUM ANALYZE drone_stats.mission_events;
VACUUM ANALYZE drone_stats.vehicle_statistics;
"

# Update statistics
docker exec postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
ANALYZE;
"

# Check compression status
docker exec postgres-drone-stats psql -U airsim_user -d drone_statistics -c "
SELECT 
    schema_name,
    table_name,
    compression_stats
FROM timescaledb_information.compressed_chunk_stats
ORDER BY schema_name, table_name;
"
```

#### Performance Monitoring
```sql
-- Monitor query performance
CREATE OR REPLACE VIEW monitoring.query_performance AS
SELECT 
    query,
    calls,
    total_time,
    mean_time,
    min_time,
    max_time,
    stddev_time
FROM pg_stat_statements 
WHERE query LIKE '%drone_stats%'
ORDER BY mean_time DESC;

-- Monitor table sizes
CREATE OR REPLACE VIEW monitoring.table_sizes AS
SELECT 
    schemaname,
    tablename,
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size,
    pg_total_relation_size(schemaname||'.'||tablename) as size_bytes
FROM pg_tables 
WHERE schemaname = 'drone_stats'
ORDER BY size_bytes DESC;
```

---

## Getting Started Checklist

1. **✅ Start PostgreSQL Database**
   ```bash
   cd docker/cli && python -m mnstevv postgres up
   ```

2. **✅ Verify Database Health**
   ```bash
   cd docker/cli && python -m mnstevv postgres status
   ```

3. **✅ Start ROS2 Mission System**
   ```bash
   ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py mission_mode:=true
   ```

4. **✅ Start Statistics Logger**
   ```bash
   ros2 run airsim_ros_pkgs drone_stats_logger.py
   ```

5. **✅ Execute a Test Mission**
   ```bash
   ros2 action send_goal /Droan1/actions/search_area [mission_parameters]
   ```

6. **✅ Analyze Results**
   ```bash
   python3 stats_analyzer.py summary --vehicle Droan1
   ```

For questions or issues, refer to the [Troubleshooting Guide](#troubleshooting-guide) or check the system health with the comprehensive monitoring tools provided.