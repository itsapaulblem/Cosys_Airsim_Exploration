#!/usr/bin/env python3
"""
Database Abstraction Layer for Drone Movement Statistics

Provides unified interface for SQLite and PostgreSQL databases,
allowing seamless switching between development (SQLite) and
production (PostgreSQL with TimescaleDB) environments.

Features:
- Automatic database type detection
- Connection pooling for PostgreSQL
- Schema migration support
- Thread-safe operations
- Environment-based configuration

Usage:
    from database_interface import DatabaseInterface
    
    # Auto-detect database type from environment
    db = DatabaseInterface()
    
    # Or specify explicitly
    db = DatabaseInterface(db_type='postgresql')
    
    # Use unified interface
    with db.get_connection() as conn:
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM vehicle_statistics")
"""

import os
import json
import sqlite3
import threading
from typing import Dict, List, Any, Optional, Union, ContextManager
from datetime import datetime, timezone
from dataclasses import dataclass
from enum import Enum
import logging

# PostgreSQL imports (optional)
try:
    import psycopg2
    import psycopg2.pool
    from psycopg2.extras import RealDictCursor, Json
    HAS_POSTGRESQL = True
except ImportError:
    HAS_POSTGRESQL = False
    psycopg2 = None
    RealDictCursor = None
    Json = None

class DatabaseType(Enum):
    """Supported database types"""
    SQLITE = "sqlite"
    POSTGRESQL = "postgresql"

@dataclass
class DatabaseConfig:
    """Database configuration parameters"""
    db_type: DatabaseType
    
    # SQLite configuration
    sqlite_path: str = "drone_movement_stats.db"
    
    # PostgreSQL configuration
    pg_host: str = "localhost"
    pg_port: int = 5432
    pg_database: str = "drone_statistics"
    pg_user: str = "airsim_user"
    pg_password: str = ""  # Must be set via POSTGRES_PASSWORD environment variable
    pg_schema: str = "drone_stats"
    
    # Connection pooling
    min_connections: int = 1
    max_connections: int = 10
    connection_timeout: int = 30
    
    # Performance settings
    autocommit: bool = False
    isolation_level: str = "READ_COMMITTED"

class DatabaseInterface:
    """
    Unified database interface supporting SQLite and PostgreSQL
    """
    
    def __init__(self, db_type: Optional[str] = None, config: Optional[DatabaseConfig] = None):
        """
        Initialize database interface
        
        Args:
            db_type: Database type ('sqlite' or 'postgresql'). Auto-detected if None.
            config: Custom database configuration. Uses environment if None.
        """
        self.logger = logging.getLogger(__name__)
        
        # Determine database type
        if db_type:
            self.db_type = DatabaseType(db_type.lower())
        else:
            self.db_type = self._detect_database_type()
        
        # Load configuration
        self.config = config or self._load_config()
        self.config.db_type = self.db_type
        
        # Initialize database-specific components
        self._connection_pool = None
        self._local_storage = threading.local()
        self._lock = threading.RLock()
        
        # Validate and initialize
        self._validate_configuration()
        self._initialize_database()
        
        self.logger.info(f"Database interface initialized: {self.db_type.value}")
    
    def _detect_database_type(self) -> DatabaseType:
        """Auto-detect database type from environment"""
        # Check environment variables
        if os.getenv('DATABASE_TYPE'):
            return DatabaseType(os.getenv('DATABASE_TYPE').lower())
        
        # Check for PostgreSQL connection parameters
        if (os.getenv('POSTGRES_HOST') or os.getenv('POSTGRES_DB') or 
            os.getenv('AIRSIM_DB_HOST')):
            return DatabaseType.POSTGRESQL
        
        # Check if running in Docker container
        if os.path.exists('/.dockerenv'):
            # In Docker, prefer PostgreSQL if available
            return DatabaseType.POSTGRESQL if HAS_POSTGRESQL else DatabaseType.SQLITE
        
        # Default to SQLite for development
        return DatabaseType.SQLITE
    
    def _load_config(self) -> DatabaseConfig:
        """Load configuration from environment variables"""
        config = DatabaseConfig(db_type=self.db_type)
        
        # SQLite configuration
        config.sqlite_path = os.getenv('SQLITE_DATABASE_PATH', config.sqlite_path)
        
        # PostgreSQL configuration
        config.pg_host = os.getenv('POSTGRES_HOST', 
                                   os.getenv('AIRSIM_DB_HOST', config.pg_host))
        config.pg_port = int(os.getenv('POSTGRES_PORT', 
                                       os.getenv('AIRSIM_DB_PORT', config.pg_port)))
        config.pg_database = os.getenv('POSTGRES_DB', 
                                       os.getenv('AIRSIM_DB_NAME', config.pg_database))
        config.pg_user = os.getenv('POSTGRES_USER', 
                                   os.getenv('AIRSIM_DB_USER', config.pg_user))
        config.pg_password = os.getenv('POSTGRES_PASSWORD',
                                       os.getenv('AIRSIM_DB_PASSWORD', ""))
        config.pg_schema = os.getenv('POSTGRES_SCHEMA', config.pg_schema)
        
        # Connection pooling
        config.max_connections = int(os.getenv('DB_MAX_CONNECTIONS', config.max_connections))
        config.connection_timeout = int(os.getenv('DB_CONNECTION_TIMEOUT', config.connection_timeout))
        
        return config
    
    def _validate_configuration(self):
        """Validate database configuration"""
        if self.db_type == DatabaseType.POSTGRESQL and not HAS_POSTGRESQL:
            raise ImportError(
                "PostgreSQL support requires psycopg2. Install with: pip install psycopg2-binary"
            )

        if self.db_type == DatabaseType.POSTGRESQL and not self.config.pg_password:
            raise ValueError(
                "PostgreSQL password is required but not configured. "
                "Set the POSTGRES_PASSWORD environment variable with a secure password."
            )
        
        if self.db_type == DatabaseType.POSTGRESQL:
            # Test PostgreSQL connection
            try:
                test_conn = psycopg2.connect(
                    host=self.config.pg_host,
                    port=self.config.pg_port,
                    database=self.config.pg_database,
                    user=self.config.pg_user,
                    password=self.config.pg_password,
                    connect_timeout=5
                )
                test_conn.close()
                self.logger.info("PostgreSQL connection validated")
            except Exception as e:
                self.logger.warning(f"PostgreSQL connection failed: {e}")
                self.logger.info("Falling back to SQLite")
                self.db_type = DatabaseType.SQLITE
                self.config.db_type = DatabaseType.SQLITE
    
    def _initialize_database(self):
        """Initialize database connections and pools"""
        if self.db_type == DatabaseType.POSTGRESQL:
            self._initialize_postgresql()
        else:
            self._initialize_sqlite()
    
    def _initialize_postgresql(self):
        """Initialize PostgreSQL connection pool"""
        try:
            self._connection_pool = psycopg2.pool.ThreadedConnectionPool(
                minconn=self.config.min_connections,
                maxconn=self.config.max_connections,
                host=self.config.pg_host,
                port=self.config.pg_port,
                database=self.config.pg_database,
                user=self.config.pg_user,
                password=self.config.pg_password,
                cursor_factory=RealDictCursor
            )
            self.logger.info("PostgreSQL connection pool initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize PostgreSQL pool: {e}")
            raise
    
    def _initialize_sqlite(self):
        """Initialize SQLite database"""
        # SQLite connections are created per-thread
        self.logger.info(f"SQLite database: {self.config.sqlite_path}")
    
    def get_connection(self) -> ContextManager:
        """Get database connection (context manager)"""
        if self.db_type == DatabaseType.POSTGRESQL:
            return PostgreSQLConnection(self._connection_pool, self.config)
        else:
            return SQLiteConnection(self.config.sqlite_path)
    
    def execute_query(self, query: str, params: Optional[tuple] = None) -> List[Dict[str, Any]]:
        """Execute query and return results"""
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query, params or ())
            
            if cursor.description:
                columns = [desc[0] for desc in cursor.description]
                return [dict(zip(columns, row)) for row in cursor.fetchall()]
            return []
    
    def execute_many(self, query: str, params_list: List[tuple]) -> int:
        """Execute query multiple times with different parameters"""
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.executemany(query, params_list)
            return cursor.rowcount
    
    def insert_event(self, event_data: Dict[str, Any]) -> Optional[int]:
        """Insert movement event into database"""
        if self.db_type == DatabaseType.POSTGRESQL:
            return self._insert_event_postgresql(event_data)
        else:
            return self._insert_event_sqlite(event_data)
    
    def _insert_event_postgresql(self, event_data: Dict[str, Any]) -> Optional[int]:
        """Insert event into PostgreSQL with proper schema"""
        query = f"""
        INSERT INTO {self.config.pg_schema}.movement_events (
            timestamp, vehicle_name, event_type, 
            position_x, position_y, position_z,
            velocity_x, velocity_y, velocity_z, velocity_magnitude,
            details, sequence_number, source_topic, ros_timestamp
        ) VALUES (
            %(timestamp)s, %(vehicle_name)s, %(event_type)s,
            %(position_x)s, %(position_y)s, %(position_z)s,
            %(velocity_x)s, %(velocity_y)s, %(velocity_z)s, %(velocity_magnitude)s,
            %(details)s, %(sequence_number)s, %(source_topic)s, %(ros_timestamp)s
        ) RETURNING id
        """
        
        # Convert details to JSON
        if 'details' in event_data and isinstance(event_data['details'], dict):
            event_data['details'] = Json(event_data['details'])
        
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query, event_data)
            result = cursor.fetchone()
            return result['id'] if result else None
    
    def _insert_event_sqlite(self, event_data: Dict[str, Any]) -> Optional[int]:
        """Insert event into SQLite database"""
        query = """
        INSERT INTO movement_events (
            timestamp, vehicle_name, event_type,
            distance_moved, altitude_change, speed_change,
            position_x, position_y, position_z,
            velocity_x, velocity_y, velocity_z,
            mission_id, mission_phase, mission_progress,
            confidence_score, tags
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        
        # Convert details to JSON string for SQLite
        tags = json.dumps(event_data.get('details', {}))
        
        params = (
            event_data.get('timestamp', datetime.now().timestamp()),
            event_data.get('vehicle_name'),
            event_data.get('event_type'),
            event_data.get('distance_moved', 0.0),
            event_data.get('altitude_change', 0.0),
            event_data.get('speed_change', 0.0),
            event_data.get('position_x'),
            event_data.get('position_y'),
            event_data.get('position_z'),
            event_data.get('velocity_x'),
            event_data.get('velocity_y'),
            event_data.get('velocity_z'),
            event_data.get('mission_id'),
            event_data.get('mission_phase'),
            event_data.get('mission_progress'),
            event_data.get('confidence_score'),
            tags
        )
        
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query, params)
            return cursor.lastrowid
    
    def get_vehicle_statistics(self, vehicle_name: Optional[str] = None) -> List[Dict[str, Any]]:
        """Get vehicle statistics"""
        if self.db_type == DatabaseType.POSTGRESQL:
            schema = self.config.pg_schema
            query = f"SELECT * FROM {schema}.vehicle_statistics"
            if vehicle_name:
                query += " WHERE vehicle_name = %s"
                params = (vehicle_name,)
            else:
                params = None
        else:
            query = "SELECT * FROM vehicle_statistics"
            if vehicle_name:
                query += " WHERE vehicle_name = ?"
                params = (vehicle_name,)
            else:
                params = None
        
        return self.execute_query(query, params)
    
    def close(self):
        """Close database connections"""
        if self._connection_pool:
            self._connection_pool.closeall()
            self.logger.info("Database connections closed")

class SQLiteConnection:
    """SQLite connection context manager"""
    
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = None
    
    def __enter__(self):
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        return self.conn
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            self.conn.commit()
        else:
            self.conn.rollback()
        self.conn.close()

class PostgreSQLConnection:
    """PostgreSQL connection context manager"""
    
    def __init__(self, pool, config: DatabaseConfig):
        self.pool = pool
        self.config = config
        self.conn = None
    
    def __enter__(self):
        self.conn = self.pool.getconn()
        # Set search path to include our schema
        with self.conn.cursor() as cursor:
            cursor.execute(f"SET search_path TO {self.config.pg_schema}, public")
        return self.conn
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            self.conn.commit()
        else:
            self.conn.rollback()
        self.pool.putconn(self.conn)

# Convenience functions for common operations
def get_database_interface(**kwargs) -> DatabaseInterface:
    """Factory function to create database interface"""
    return DatabaseInterface(**kwargs)

def create_tables_if_not_exist(db: DatabaseInterface):
    """Create comprehensive database tables for drone statistics system"""
    if db.db_type == DatabaseType.POSTGRESQL:
        _create_postgresql_tables(db)
    else:
        _create_sqlite_tables(db)

def _create_postgresql_tables(db: DatabaseInterface):
    """Create PostgreSQL tables with proper schema and TimescaleDB optimization"""
    try:
        with db.get_connection() as conn:
            cursor = conn.cursor()
            schema = db.config.pg_schema
            
            # Enable TimescaleDB extension if not already enabled
            cursor.execute("CREATE EXTENSION IF NOT EXISTS timescaledb CASCADE;")
            
            # Create schema if it doesn't exist
            cursor.execute(f"CREATE SCHEMA IF NOT EXISTS {schema};")
            cursor.execute(f"SET search_path TO {schema}, public;")
            
            # 1. Vehicle statistics table (summary data)
            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {schema}.vehicle_statistics (
                    vehicle_name TEXT PRIMARY KEY,
                    total_distance_m REAL DEFAULT 0.0,
                    flight_time_s REAL DEFAULT 0.0,
                    events_count INTEGER DEFAULT 0,
                    targets_detected INTEGER DEFAULT 0,
                    waypoints_completed INTEGER DEFAULT 0,
                    area_covered_m2 REAL DEFAULT 0.0,
                    last_updated TIMESTAMPTZ DEFAULT NOW(),
                    created_at TIMESTAMPTZ DEFAULT NOW()
                );
            ''')
            
            # 2. Movement events table (mission events from ROS2)
            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {schema}.movement_events (
                    vehicle_name TEXT NOT NULL,
                    timestamp TIMESTAMPTZ NOT NULL,
                    event_type TEXT NOT NULL,
                    event_source TEXT,
                    sequence_number BIGINT,
                    
                    distance_moved REAL,
                    altitude_change REAL,
                    speed_change REAL,
                    
                    position_x REAL,
                    position_y REAL,
                    position_z REAL,
                    previous_x REAL,
                    previous_y REAL,
                    previous_z REAL,
                    
                    velocity_x REAL,
                    velocity_y REAL,
                    velocity_z REAL,
                    
                    mission_id TEXT,
                    mission_phase TEXT,
                    mission_progress REAL,
                    
                    confidence_score REAL,
                    tags JSONB,
                    
                    PRIMARY KEY (vehicle_name, timestamp, sequence_number)
                );
            ''')
            
            # 3. Mission status events table  
            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {schema}.mission_events (
                    vehicle_name TEXT NOT NULL,
                    timestamp TIMESTAMPTZ NOT NULL,
                    mission_id TEXT,
                    progress_percentage REAL,
                    current_activity TEXT,
                    waypoints_completed INTEGER,
                    area_covered REAL,
                    targets_detected INTEGER,
                    mission_start_time TIMESTAMPTZ,
                    estimated_remaining_time INTERVAL,
                    mission_status INTEGER,
                    has_error BOOLEAN DEFAULT FALSE,
                    error_message TEXT,
                    
                    PRIMARY KEY (vehicle_name, timestamp)
                );
            ''')
            
            # 4. Target detections table
            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {schema}.target_detections (
                    vehicle_name TEXT NOT NULL,
                    timestamp TIMESTAMPTZ NOT NULL,
                    detection_id TEXT NOT NULL,
                    target_type TEXT,
                    confidence_score REAL,
                    
                    world_position_x REAL,
                    world_position_y REAL,  
                    world_position_z REAL,
                    
                    gps_lat REAL,
                    gps_lon REAL,
                    gps_alt REAL,
                    
                    detection_altitude REAL,
                    detection_distance REAL,
                    detection_bearing REAL,
                    
                    verified BOOLEAN DEFAULT FALSE,
                    false_positive BOOLEAN DEFAULT FALSE,
                    notes TEXT,
                    
                    PRIMARY KEY (vehicle_name, timestamp, detection_id)
                );
            ''')
            
            # 5. Position history table (high-frequency odometry data)
            cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {schema}.position_history (
                    vehicle_name TEXT NOT NULL,
                    timestamp TIMESTAMPTZ NOT NULL,
                    position_x REAL,
                    position_y REAL,
                    position_z REAL,
                    velocity_x REAL,
                    velocity_y REAL,
                    velocity_z REAL,
                    orientation_w REAL,
                    orientation_x REAL,
                    orientation_y REAL,
                    orientation_z REAL,
                    
                    PRIMARY KEY (vehicle_name, timestamp)
                );
            ''')
            
            # Convert to TimescaleDB hypertables for time-series optimization
            cursor.execute(f"SELECT create_hypertable('{schema}.movement_events', 'timestamp', if_not_exists => TRUE);")
            cursor.execute(f"SELECT create_hypertable('{schema}.mission_events', 'timestamp', if_not_exists => TRUE);")
            cursor.execute(f"SELECT create_hypertable('{schema}.target_detections', 'timestamp', if_not_exists => TRUE);")
            cursor.execute(f"SELECT create_hypertable('{schema}.position_history', 'timestamp', if_not_exists => TRUE);")
            
            # Create indexes for optimal query performance
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_movement_vehicle_time ON {schema}.movement_events(vehicle_name, timestamp DESC);')
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_movement_event_type ON {schema}.movement_events(event_type);')
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_mission_vehicle_time ON {schema}.mission_events(vehicle_name, timestamp DESC);')
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_detection_vehicle_time ON {schema}.target_detections(vehicle_name, timestamp DESC);')
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_detection_type ON {schema}.target_detections(target_type);')
            cursor.execute(f'CREATE INDEX IF NOT EXISTS idx_position_vehicle_time ON {schema}.position_history(vehicle_name, timestamp DESC);')
            
            db.logger.info(f"PostgreSQL tables created successfully in schema '{schema}' with TimescaleDB optimization")
            
    except Exception as e:
        db.logger.error(f"Failed to create PostgreSQL tables: {e}")
        raise

def _create_sqlite_tables(db: DatabaseInterface):
    """Create SQLite tables with comprehensive schema"""
    try:
        with db.get_connection() as conn:
            cursor = conn.cursor()
            
            # 1. Vehicle statistics table (summary data)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS vehicle_statistics (
                    vehicle_name TEXT PRIMARY KEY,
                    total_distance_m REAL DEFAULT 0.0,
                    flight_time_s REAL DEFAULT 0.0,
                    events_count INTEGER DEFAULT 0,
                    targets_detected INTEGER DEFAULT 0,
                    waypoints_completed INTEGER DEFAULT 0,
                    area_covered_m2 REAL DEFAULT 0.0,
                    last_updated REAL DEFAULT (strftime('%s', 'now')),
                    created_at REAL DEFAULT (strftime('%s', 'now'))
                );
            ''')
            
            # 2. Movement events table (mission events from ROS2)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS movement_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    vehicle_name TEXT NOT NULL,
                    timestamp REAL NOT NULL,
                    event_type TEXT NOT NULL,
                    event_source TEXT,
                    sequence_number INTEGER,
                    
                    distance_moved REAL,
                    altitude_change REAL,
                    speed_change REAL,
                    
                    position_x REAL,
                    position_y REAL,
                    position_z REAL,
                    previous_x REAL,
                    previous_y REAL,
                    previous_z REAL,
                    
                    velocity_x REAL,
                    velocity_y REAL,
                    velocity_z REAL,
                    
                    mission_id TEXT,
                    mission_phase TEXT,
                    mission_progress REAL,
                    
                    confidence_score REAL,
                    tags TEXT
                );
            ''')
            
            # 3. Mission status events table
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS mission_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    vehicle_name TEXT NOT NULL,
                    timestamp REAL NOT NULL,
                    mission_id TEXT,
                    progress_percentage REAL,
                    current_activity TEXT,
                    waypoints_completed INTEGER,
                    area_covered REAL,
                    targets_detected INTEGER,
                    mission_start_time REAL,
                    estimated_remaining_time REAL,
                    mission_status INTEGER,
                    has_error INTEGER DEFAULT 0,
                    error_message TEXT
                );
            ''')
            
            # 4. Target detections table
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS target_detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    vehicle_name TEXT NOT NULL,
                    timestamp REAL NOT NULL,
                    detection_id TEXT NOT NULL,
                    target_type TEXT,
                    confidence_score REAL,
                    
                    world_position_x REAL,
                    world_position_y REAL,
                    world_position_z REAL,
                    
                    gps_lat REAL,
                    gps_lon REAL,
                    gps_alt REAL,
                    
                    detection_altitude REAL,
                    detection_distance REAL,
                    detection_bearing REAL,
                    
                    verified INTEGER DEFAULT 0,
                    false_positive INTEGER DEFAULT 0,
                    notes TEXT,
                    
                    UNIQUE(detection_id, vehicle_name)
                );
            ''')
            
            # 5. Position history table (high-frequency odometry - simplified for SQLite)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS position_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    vehicle_name TEXT NOT NULL,
                    timestamp REAL NOT NULL,
                    position_x REAL,
                    position_y REAL,
                    position_z REAL,
                    velocity_x REAL,
                    velocity_y REAL,
                    velocity_z REAL,
                    orientation_w REAL,
                    orientation_x REAL,
                    orientation_y REAL,
                    orientation_z REAL
                );
            ''')
            
            # Create indexes for better performance
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_movement_vehicle_name ON movement_events(vehicle_name);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_movement_timestamp ON movement_events(timestamp);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_movement_event_type ON movement_events(event_type);')
            
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_mission_vehicle_name ON mission_events(vehicle_name);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_mission_timestamp ON mission_events(timestamp);')
            
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_detection_vehicle_name ON target_detections(vehicle_name);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_detection_timestamp ON target_detections(timestamp);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_detection_type ON target_detections(target_type);')
            
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_position_vehicle_name ON position_history(vehicle_name);')
            cursor.execute('CREATE INDEX IF NOT EXISTS idx_position_timestamp ON position_history(timestamp);')
            
            db.logger.info("SQLite tables created successfully with comprehensive schema")
            
    except Exception as e:
        db.logger.error(f"Failed to create SQLite tables: {e}")
        raise

if __name__ == "__main__":
    # Test the database interface
    logging.basicConfig(level=logging.INFO)
    
    # Test auto-detection
    db = DatabaseInterface()
    print(f"Auto-detected database type: {db.db_type.value}")
    
    # Create tables
    create_tables_if_not_exist(db)
    
    # Test connection
    try:
        with db.get_connection() as conn:
            cursor = conn.cursor()
            if db.db_type == DatabaseType.POSTGRESQL:
                cursor.execute("SELECT version()")
            else:
                cursor.execute("SELECT sqlite_version()")
            result = cursor.fetchone()
            print(f"Database version: {result}")
    except Exception as e:
        print(f"Connection test failed: {e}")
    finally:
        db.close()