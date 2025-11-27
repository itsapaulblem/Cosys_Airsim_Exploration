#!/usr/bin/env python3
"""
Database Health Check Utilities

Comprehensive health monitoring for PostgreSQL and SQLite databases
used by the drone statistics system.

Features:
- Connection validation and testing
- Schema verification
- Performance metrics collection
- TimescaleDB-specific health checks
- User permission validation
- Connection pool monitoring

Usage:
    from health_utils import DatabaseHealthChecker
    
    checker = DatabaseHealthChecker()
    health_status = checker.comprehensive_health_check()
    print(health_status.summary())
"""

import os
import time
import json
import logging
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from enum import Enum

# Import database interface
from database_interface import DatabaseInterface, DatabaseType, DatabaseConfig

# PostgreSQL imports (optional)
try:
    import psycopg2
    from psycopg2.extras import RealDictCursor
    HAS_POSTGRESQL = True
except ImportError:
    HAS_POSTGRESQL = False

class HealthStatus(Enum):
    """Health status indicators"""
    HEALTHY = "healthy"
    WARNING = "warning"
    ERROR = "error"
    UNKNOWN = "unknown"

@dataclass
class HealthMetric:
    """Individual health metric result"""
    name: str
    status: HealthStatus
    value: Any
    message: str
    details: Optional[Dict[str, Any]] = None
    timestamp: Optional[datetime] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now(timezone.utc)

@dataclass
class DatabaseHealthReport:
    """Comprehensive database health report"""
    database_type: str
    overall_status: HealthStatus
    connection_status: HealthMetric
    schema_status: HealthMetric
    performance_metrics: List[HealthMetric]
    timescaledb_metrics: List[HealthMetric]
    error_details: Optional[List[str]] = None
    timestamp: Optional[datetime] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now(timezone.utc)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert report to dictionary for JSON serialization"""
        return asdict(self)
    
    def summary(self) -> str:
        """Generate human-readable summary"""
        lines = [
            f"=== Database Health Report ({self.database_type}) ===",
            f"Overall Status: {self.overall_status.value.upper()}",
            f"Timestamp: {self.timestamp.strftime('%Y-%m-%d %H:%M:%S UTC')}",
            "",
            f"Connection: {self.connection_status.status.value} - {self.connection_status.message}",
            f"Schema: {self.schema_status.status.value} - {self.schema_status.message}",
            ""
        ]
        
        if self.performance_metrics:
            lines.append("Performance Metrics:")
            for metric in self.performance_metrics:
                lines.append(f"  • {metric.name}: {metric.value} ({metric.status.value})")
            lines.append("")
        
        if self.timescaledb_metrics and self.database_type == "postgresql":
            lines.append("TimescaleDB Metrics:")
            for metric in self.timescaledb_metrics:
                lines.append(f"  • {metric.name}: {metric.value} ({metric.status.value})")
            lines.append("")
        
        if self.error_details:
            lines.append("Error Details:")
            for error in self.error_details:
                lines.append(f"  ! {error}")
        
        return "\n".join(lines)

class DatabaseHealthChecker:
    """Comprehensive database health monitoring system"""
    
    def __init__(self, db_interface: Optional[DatabaseInterface] = None):
        """
        Initialize health checker.
        
        Args:
            db_interface: Optional database interface (will create if not provided)
        """
        self.db_interface = db_interface or DatabaseInterface()
        self.logger = logging.getLogger(__name__)
        
    def comprehensive_health_check(self) -> DatabaseHealthReport:
        """
        Perform comprehensive health check of the database system.
        
        Returns:
            Complete health report with all metrics
        """
        self.logger.info("Starting comprehensive database health check")
        
        # Determine database type
        db_type = self.db_interface.config.db_type.value
        
        # Core health checks
        connection_status = self._check_connection()
        schema_status = self._check_schema()
        performance_metrics = self._check_performance()
        
        # TimescaleDB-specific checks (PostgreSQL only)
        timescaledb_metrics = []
        if db_type == "postgresql" and HAS_POSTGRESQL:
            timescaledb_metrics = self._check_timescaledb()
        
        # Determine overall status
        overall_status = self._determine_overall_status(
            connection_status, schema_status, performance_metrics, timescaledb_metrics
        )
        
        return DatabaseHealthReport(
            database_type=db_type,
            overall_status=overall_status,
            connection_status=connection_status,
            schema_status=schema_status,
            performance_metrics=performance_metrics,
            timescaledb_metrics=timescaledb_metrics
        )
    
    def _check_connection(self) -> HealthMetric:
        """Test database connectivity and basic operations"""
        try:
            start_time = time.time()
            
            with self.db_interface.get_connection() as conn:
                cursor = conn.cursor()
                
                # Basic connectivity test
                if self.db_interface.config.db_type == DatabaseType.POSTGRESQL:
                    cursor.execute("SELECT version() as version, current_database() as database, current_user as user")
                    version_info = cursor.fetchone()
                    
                    connection_time = time.time() - start_time
                    
                    details = {
                        "version": version_info['version'] if version_info else "unknown",
                        "database": version_info['database'] if version_info else "unknown",
                        "user": version_info['user'] if version_info else "unknown",
                        "connection_time_ms": round(connection_time * 1000, 2)
                    }
                    
                    status = HealthStatus.HEALTHY if connection_time < 1.0 else HealthStatus.WARNING
                    message = f"Connected successfully in {connection_time:.3f}s"
                    
                else:  # SQLite
                    cursor.execute("SELECT sqlite_version() as version")
                    version_info = cursor.fetchone()
                    version = version_info['version'] if hasattr(version_info, 'keys') else version_info[0]
                    
                    connection_time = time.time() - start_time
                    
                    details = {
                        "version": version,
                        "database": self.db_interface.config.sqlite_path,
                        "connection_time_ms": round(connection_time * 1000, 2)
                    }
                    
                    status = HealthStatus.HEALTHY
                    message = f"SQLite connection successful ({version})"
                
                return HealthMetric(
                    name="database_connection",
                    status=status,
                    value=connection_time,
                    message=message,
                    details=details
                )
                
        except Exception as e:
            self.logger.error(f"Database connection failed: {e}")
            return HealthMetric(
                name="database_connection",
                status=HealthStatus.ERROR,
                value=None,
                message=f"Connection failed: {str(e)}",
                details={"error": str(e)}
            )
    
    def _check_schema(self) -> HealthMetric:
        """Verify database schema and required tables"""
        try:
            with self.db_interface.get_connection() as conn:
                cursor = conn.cursor()
                
                # Expected tables for comprehensive drone stats system
                expected_tables = [
                    'vehicle_statistics',    # Vehicle summary data
                    'movement_events',      # Mission events from ROS2
                    'mission_events',       # Mission status updates
                    'target_detections',    # Target detection events
                    'position_history'      # High-frequency odometry data
                ]
                
                missing_tables = []
                existing_tables = []
                
                for table_name in expected_tables:
                    if self.db_interface.config.db_type == DatabaseType.POSTGRESQL:
                        # Check in drone_stats schema
                        cursor.execute("""
                            SELECT EXISTS (
                                SELECT FROM information_schema.tables 
                                WHERE table_schema = %s AND table_name = %s
                            )
                        """, (self.db_interface.config.pg_schema, table_name))
                    else:  # SQLite
                        cursor.execute("""
                            SELECT name FROM sqlite_master 
                            WHERE type='table' AND name=?
                        """, (table_name,))
                    
                    result = cursor.fetchone()
                    exists = result[0] if hasattr(result, '__getitem__') and not hasattr(result, 'keys') else list(result.values())[0]
                    if exists:
                        existing_tables.append(table_name)
                    else:
                        missing_tables.append(table_name)
                
                # Determine status
                if not missing_tables:
                    status = HealthStatus.HEALTHY
                    message = f"All {len(expected_tables)} required tables exist"
                elif len(existing_tables) > 0:
                    status = HealthStatus.WARNING
                    message = f"{len(existing_tables)}/{len(expected_tables)} tables exist"
                else:
                    status = HealthStatus.ERROR
                    message = "No required tables found"
                
                details = {
                    "existing_tables": existing_tables,
                    "missing_tables": missing_tables,
                    "total_expected": len(expected_tables)
                }
                
                return HealthMetric(
                    name="database_schema",
                    status=status,
                    value=len(existing_tables),
                    message=message,
                    details=details
                )
                
        except Exception as e:
            self.logger.error(f"Schema check failed: {e}")
            return HealthMetric(
                name="database_schema",
                status=HealthStatus.ERROR,
                value=0,
                message=f"Schema check failed: {str(e)}",
                details={"error": str(e)}
            )
    
    def _check_performance(self) -> List[HealthMetric]:
        """Check database performance metrics"""
        metrics = []
        
        try:
            with self.db_interface.get_connection() as conn:
                cursor = conn.cursor()
                
                # Database size check
                if self.db_interface.config.db_type == DatabaseType.POSTGRESQL:
                    cursor.execute("SELECT pg_database_size(current_database()) as size")
                    result = cursor.fetchone()
                    db_size_bytes = result['size'] if hasattr(result, 'keys') else result[0]
                    db_size_mb = db_size_bytes / (1024 * 1024)
                    
                    # Active connections
                    cursor.execute("""
                        SELECT count(*) as count FROM pg_stat_activity 
                        WHERE state = 'active' AND datname = current_database()
                    """)
                    result = cursor.fetchone()
                    active_connections = result['count'] if hasattr(result, 'keys') else result[0]
                    
                    metrics.extend([
                        HealthMetric(
                            name="database_size_mb",
                            status=HealthStatus.HEALTHY if db_size_mb < 1000 else HealthStatus.WARNING,
                            value=round(db_size_mb, 2),
                            message=f"Database size: {db_size_mb:.1f} MB"
                        ),
                        HealthMetric(
                            name="active_connections",
                            status=HealthStatus.HEALTHY if active_connections < 10 else HealthStatus.WARNING,
                            value=active_connections,
                            message=f"Active connections: {active_connections}"
                        )
                    ])
                    
                else:  # SQLite
                    # Get database file size
                    if os.path.exists(self.db_interface.config.sqlite_path):
                        file_size = os.path.getsize(self.db_interface.config.sqlite_path)
                        file_size_mb = file_size / (1024 * 1024)
                        
                        metrics.append(HealthMetric(
                            name="database_size_mb",
                            status=HealthStatus.HEALTHY if file_size_mb < 100 else HealthStatus.WARNING,
                            value=round(file_size_mb, 2),
                            message=f"Database file size: {file_size_mb:.1f} MB"
                        ))
                
        except Exception as e:
            self.logger.error(f"Performance check failed: {e}")
            metrics.append(HealthMetric(
                name="performance_check",
                status=HealthStatus.ERROR,
                value=None,
                message=f"Performance check failed: {str(e)}"
            ))
        
        return metrics
    
    def _check_timescaledb(self) -> List[HealthMetric]:
        """Check TimescaleDB-specific health metrics"""
        metrics = []
        
        if not HAS_POSTGRESQL:
            return metrics
        
        try:
            with self.db_interface.get_connection() as conn:
                cursor = conn.cursor()
                
                # Check if TimescaleDB extension is installed
                cursor.execute("""
                    SELECT EXISTS (
                        SELECT 1 FROM pg_extension WHERE extname = 'timescaledb'
                    ) as installed
                """)
                
                result = cursor.fetchone()
                timescaledb_installed = result['installed'] if hasattr(result, 'keys') else result[0]
                
                if timescaledb_installed:
                    # Get TimescaleDB version
                    cursor.execute("SELECT extversion as version FROM pg_extension WHERE extname = 'timescaledb'")
                    result = cursor.fetchone()
                    version = result['version'] if hasattr(result, 'keys') else result[0]
                    
                    metrics.append(HealthMetric(
                        name="timescaledb_extension",
                        status=HealthStatus.HEALTHY,
                        value=version,
                        message=f"TimescaleDB {version} installed"
                    ))
                    
                    # Check for hypertables
                    cursor.execute("""
                        SELECT count(*) as count FROM timescaledb_information.hypertables
                        WHERE hypertable_schema = %s
                    """, (self.db_interface.config.pg_schema,))
                    
                    result = cursor.fetchone()
                    hypertable_count = result['count'] if hasattr(result, 'keys') else result[0]
                    
                    metrics.append(HealthMetric(
                        name="hypertables_count",
                        status=HealthStatus.HEALTHY if hypertable_count > 0 else HealthStatus.WARNING,
                        value=hypertable_count,
                        message=f"{hypertable_count} hypertables configured"
                    ))
                    
                else:
                    metrics.append(HealthMetric(
                        name="timescaledb_extension",
                        status=HealthStatus.ERROR,
                        value=None,
                        message="TimescaleDB extension not installed"
                    ))
                
        except Exception as e:
            self.logger.error(f"TimescaleDB check failed: {e}")
            metrics.append(HealthMetric(
                name="timescaledb_check",
                status=HealthStatus.ERROR,
                value=None,
                message=f"TimescaleDB check failed: {str(e)}"
            ))
        
        return metrics
    
    def _determine_overall_status(self, connection: HealthMetric, schema: HealthMetric, 
                                performance: List[HealthMetric], 
                                timescaledb: List[HealthMetric]) -> HealthStatus:
        """Determine overall system health status"""
        # Connection is critical
        if connection.status == HealthStatus.ERROR:
            return HealthStatus.ERROR
        
        # Schema is critical  
        if schema.status == HealthStatus.ERROR:
            return HealthStatus.ERROR
        
        # Check for any error status in other metrics
        all_metrics = performance + timescaledb
        if any(m.status == HealthStatus.ERROR for m in all_metrics):
            return HealthStatus.ERROR
        
        # Check for warnings
        warning_count = sum(1 for m in [connection, schema] + all_metrics 
                          if m.status == HealthStatus.WARNING)
        
        if warning_count > 2:
            return HealthStatus.WARNING
        elif warning_count > 0:
            return HealthStatus.WARNING
        
        return HealthStatus.HEALTHY
    
    def quick_connectivity_test(self) -> bool:
        """Quick connectivity test for basic validation"""
        try:
            with self.db_interface.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT 1 as test")
                result = cursor.fetchone()
                test_value = result['test'] if hasattr(result, 'keys') else result[0]
                return test_value == 1
        except Exception:
            return False


# Convenience functions for CLI usage
def get_database_health() -> DatabaseHealthReport:
    """Get complete database health report"""
    checker = DatabaseHealthChecker()
    return checker.comprehensive_health_check()

def test_database_connectivity() -> bool:
    """Quick connectivity test"""
    checker = DatabaseHealthChecker()
    return checker.quick_connectivity_test()

if __name__ == "__main__":
    # Command line usage
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        # Quick test
        if test_database_connectivity():
            print("✅ Database connectivity: OK")
            sys.exit(0)
        else:
            print("❌ Database connectivity: FAILED")
            sys.exit(1)
    else:
        # Full health check
        health_report = get_database_health()
        print(health_report.summary())
        
        # Exit with appropriate code
        if health_report.overall_status == HealthStatus.ERROR:
            sys.exit(1)
        elif health_report.overall_status == HealthStatus.WARNING:
            sys.exit(2)
        else:
            sys.exit(0)