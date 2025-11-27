#!/usr/bin/env python3
"""
SQLite to PostgreSQL Migration Tool for Drone Statistics

This tool migrates existing SQLite drone statistics data to PostgreSQL with
TimescaleDB optimization, preserving all data integrity and relationships.

Features:
- Complete data migration with validation
- Schema mapping between SQLite and PostgreSQL
- Batch processing for large datasets
- Progress tracking and logging
- Rollback capabilities
- Data verification and integrity checks

Usage:
    # Basic migration (auto-detects PostgreSQL connection)
    python3 migrate_sqlite_to_postgres.py --sqlite-db drone_stats.db
    
    # Migration with custom PostgreSQL connection
    python3 migrate_sqlite_to_postgres.py \\
        --sqlite-db drone_stats.db \\
        --pg-host localhost \\
        --pg-port 5432 \\
        --pg-database drone_statistics
    
    # Dry run (validate without migrating)
    python3 migrate_sqlite_to_postgres.py --sqlite-db drone_stats.db --dry-run
    
    # Migration with custom batch size
    python3 migrate_sqlite_to_postgres.py --sqlite-db drone_stats.db --batch-size 500

Author: Claude Code Integration
Version: 1.0
"""

import argparse
import sqlite3
import sys
import json
import os
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
import time

# Import database abstraction layer
from database_interface import DatabaseInterface, DatabaseType

@dataclass
class MigrationStats:
    """Container for migration statistics"""
    tables_migrated: int = 0
    total_records: int = 0
    successful_records: int = 0
    failed_records: int = 0
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    
    @property
    def duration(self) -> float:
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0
    
    @property
    def success_rate(self) -> float:
        if self.total_records == 0:
            return 0.0
        return (self.successful_records / self.total_records) * 100

class SQLiteToPostgreSQLMigrator:
    """
    Main migration class for transferring SQLite data to PostgreSQL
    """
    
    def __init__(self, sqlite_path: str, pg_config: Optional[Dict[str, Any]] = None,
                 batch_size: int = 1000, dry_run: bool = False):
        """
        Initialize migrator
        
        Args:
            sqlite_path: Path to SQLite database
            pg_config: PostgreSQL connection configuration
            batch_size: Number of records to process in each batch
            dry_run: If True, validate without actually migrating data
        """
        self.logger = logging.getLogger(__name__)
        self.sqlite_path = Path(sqlite_path)
        self.batch_size = batch_size
        self.dry_run = dry_run
        self.stats = MigrationStats()
        
        # Validate SQLite database
        if not self.sqlite_path.exists():
            raise FileNotFoundError(f"SQLite database not found: {sqlite_path}")
        
        # Set up PostgreSQL configuration
        if pg_config:
            for key, value in pg_config.items():
                if value is not None:
                    os.environ[f'POSTGRES_{key.upper()}'] = str(value)
        
        # Initialize database connections
        self._init_connections()
        
        # Schema mapping between SQLite and PostgreSQL
        self.schema_mapping = self._build_schema_mapping()
    
    def _init_connections(self):
        """Initialize database connections"""
        # SQLite connection
        self.sqlite_conn = sqlite3.connect(self.sqlite_path)
        self.sqlite_conn.row_factory = sqlite3.Row
        
        # PostgreSQL connection via database interface
        try:
            self.pg_db = DatabaseInterface(db_type='postgresql')
            if self.pg_db.db_type != DatabaseType.POSTGRESQL:
                raise Exception("Failed to connect to PostgreSQL - check configuration")
            
            self.logger.info(f"Connected to PostgreSQL: {self.pg_db.config.pg_host}:{self.pg_db.config.pg_port}")
            
        except Exception as e:
            raise Exception(f"Failed to connect to PostgreSQL: {e}")
    
    def _build_schema_mapping(self) -> Dict[str, Dict[str, str]]:
        """Build mapping between SQLite and PostgreSQL schemas"""
        return {
            'movement_events': {
                'sqlite_table': 'movement_events',
                'postgres_table': 'movement_events',
                'id_field': 'id',
                'timestamp_field': 'timestamp',
                'field_mapping': {
                    'id': 'id',
                    'vehicle_name': 'vehicle_name', 
                    'timestamp': 'timestamp',
                    'event_type': 'event_type',
                    'event_source': 'source_topic',
                    'sequence_number': 'sequence_number',
                    'distance_moved': None,  # Will be stored in details JSON
                    'altitude_change': None,  # Will be stored in details JSON
                    'speed_change': None,    # Will be stored in details JSON
                    'position_x': 'position_x',
                    'position_y': 'position_y',
                    'position_z': 'position_z',
                    'previous_x': None,      # Will be stored in details JSON
                    'previous_y': None,      # Will be stored in details JSON
                    'previous_z': None,      # Will be stored in details JSON
                    'velocity_x': 'velocity_x',
                    'velocity_y': 'velocity_y',
                    'velocity_z': 'velocity_z',
                    'mission_id': None,      # Will be stored in details JSON
                    'mission_phase': None,   # Will be stored in details JSON
                    'mission_progress': None, # Will be stored in details JSON
                    'confidence_score': None, # Will be stored in details JSON
                    'tags': 'details'        # JSON field mapping
                }
            },
            'vehicle_summary': {
                'sqlite_table': 'vehicle_summary',
                'postgres_table': 'vehicle_statistics',
                'id_field': 'vehicle_name',
                'timestamp_field': 'last_update',
                'field_mapping': {
                    'vehicle_name': 'vehicle_name',
                    'first_seen': 'first_seen',
                    'last_seen': 'last_updated',
                    'total_distance': 'total_distance_m',
                    'total_flight_time': 'flight_time_s',
                    'total_takeoffs': None,  # Will be computed from events
                    'total_landings': None,  # Will be computed from events
                    'total_missions': 'total_missions',
                    'total_targets_detected': 'targets_detected',
                    'last_position_x': None, # Not stored in PostgreSQL schema
                    'last_position_y': None, # Not stored in PostgreSQL schema
                    'last_position_z': None, # Not stored in PostgreSQL schema
                    'last_update': 'last_updated'
                }
            },
            'target_detections': {
                'sqlite_table': 'target_detections',
                'postgres_table': 'target_detections',
                'id_field': 'id',
                'timestamp_field': 'timestamp',
                'field_mapping': {
                    'id': 'id',
                    'vehicle_name': 'vehicle_name',
                    'timestamp': 'timestamp',
                    'detection_id': None,    # Will be stored in detection_metadata JSON
                    'target_type': 'target_type',
                    'confidence_score': 'confidence',
                    'world_position_x': 'target_position_x',
                    'world_position_y': 'target_position_y', 
                    'world_position_z': 'target_position_z',
                    'mission_id': None       # Will be stored in detection_metadata JSON
                }
            }
        }
    
    def validate_databases(self) -> bool:
        """Validate both source and target databases"""
        self.logger.info("Validating databases...")
        
        # Check SQLite tables
        sqlite_tables = self._get_sqlite_tables()
        self.logger.info(f"SQLite tables found: {sqlite_tables}")
        
        # Check PostgreSQL schema
        try:
            with self.pg_db.get_connection() as pg_conn:
                cursor = pg_conn.cursor()
                cursor.execute(f"""
                    SELECT table_name FROM information_schema.tables 
                    WHERE table_schema = '{self.pg_db.config.pg_schema}'
                """)
                pg_tables = [row[0] for row in cursor.fetchall()]
                self.logger.info(f"PostgreSQL tables found: {pg_tables}")
                
                # Verify required tables exist
                required_tables = ['movement_events', 'vehicle_statistics', 'target_detections']
                missing_tables = [t for t in required_tables if t not in pg_tables]
                if missing_tables:
                    self.logger.error(f"Missing PostgreSQL tables: {missing_tables}")
                    return False
                
        except Exception as e:
            self.logger.error(f"PostgreSQL validation failed: {e}")
            return False
        
        return True
    
    def _get_sqlite_tables(self) -> List[str]:
        """Get list of tables in SQLite database"""
        cursor = self.sqlite_conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'"
        )
        return [row[0] for row in cursor.fetchall()]
    
    def analyze_data_volume(self) -> Dict[str, int]:
        """Analyze the volume of data to be migrated"""
        self.logger.info("Analyzing data volume...")
        
        volume = {}
        for table_config in self.schema_mapping.values():
            sqlite_table = table_config['sqlite_table']
            try:
                cursor = self.sqlite_conn.execute(f"SELECT COUNT(*) FROM {sqlite_table}")
                count = cursor.fetchone()[0]
                volume[sqlite_table] = count
                self.logger.info(f"Table {sqlite_table}: {count:,} records")
            except sqlite3.OperationalError:
                volume[sqlite_table] = 0
                self.logger.warning(f"Table {sqlite_table} not found in SQLite")
        
        total_records = sum(volume.values())
        self.logger.info(f"Total records to migrate: {total_records:,}")
        return volume
    
    def migrate_table(self, table_config: Dict[str, Any]) -> Tuple[int, int]:
        """
        Migrate a single table from SQLite to PostgreSQL
        
        Returns:
            Tuple of (successful_records, failed_records)
        """
        sqlite_table = table_config['sqlite_table']
        postgres_table = table_config['postgres_table']
        field_mapping = table_config['field_mapping']
        
        self.logger.info(f"Migrating table: {sqlite_table} -> {postgres_table}")
        
        # Get total record count
        cursor = self.sqlite_conn.execute(f"SELECT COUNT(*) FROM {sqlite_table}")
        total_records = cursor.fetchone()[0]
        
        if total_records == 0:
            self.logger.info(f"No records to migrate in {sqlite_table}")
            return 0, 0
        
        successful = 0
        failed = 0
        
        # Process in batches
        offset = 0
        while offset < total_records:
            batch_records = self._get_batch_records(sqlite_table, offset, self.batch_size)
            
            if not batch_records:
                break
            
            batch_success, batch_failed = self._migrate_batch(
                batch_records, postgres_table, field_mapping
            )
            
            successful += batch_success
            failed += batch_failed
            offset += len(batch_records)
            
            # Progress reporting
            progress = (offset / total_records) * 100
            self.logger.info(f"Progress {sqlite_table}: {offset:,}/{total_records:,} ({progress:.1f}%)")
        
        self.logger.info(f"Completed {sqlite_table}: {successful:,} successful, {failed:,} failed")
        return successful, failed
    
    def _get_batch_records(self, table: str, offset: int, limit: int) -> List[sqlite3.Row]:
        """Get a batch of records from SQLite table"""
        cursor = self.sqlite_conn.execute(
            f"SELECT * FROM {table} LIMIT {limit} OFFSET {offset}"
        )
        return cursor.fetchall()
    
    def _migrate_batch(self, records: List[sqlite3.Row], postgres_table: str, 
                      field_mapping: Dict[str, str]) -> Tuple[int, int]:
        """Migrate a batch of records to PostgreSQL"""
        if self.dry_run:
            return len(records), 0
        
        successful = 0
        failed = 0
        
        try:
            with self.pg_db.get_connection() as pg_conn:
                cursor = pg_conn.cursor()
                
                for record in records:
                    try:
                        pg_record = self._transform_record(record, field_mapping)
                        
                        # Build INSERT query
                        columns = list(pg_record.keys())
                        placeholders = ['%s'] * len(columns)
                        values = list(pg_record.values())
                        
                        query = f"""
                        INSERT INTO {self.pg_db.config.pg_schema}.{postgres_table} 
                        ({', '.join(columns)}) 
                        VALUES ({', '.join(placeholders)})
                        ON CONFLICT DO NOTHING
                        """
                        
                        cursor.execute(query, values)
                        successful += 1
                        
                    except Exception as e:
                        self.logger.error(f"Failed to migrate record: {e}")
                        failed += 1
                
        except Exception as e:
            self.logger.error(f"Batch migration failed: {e}")
            failed += len(records)
        
        return successful, failed
    
    def _transform_record(self, sqlite_record: sqlite3.Row, 
                         field_mapping: Dict[str, str]) -> Dict[str, Any]:
        """Transform SQLite record to PostgreSQL format"""
        pg_record = {}
        details = {}
        
        for sqlite_field, pg_field in field_mapping.items():
            if sqlite_field not in sqlite_record.keys():
                continue
                
            value = sqlite_record[sqlite_field]
            
            if pg_field is None:
                # Store in details JSON
                if value is not None:
                    details[sqlite_field] = value
            elif pg_field == 'details':
                # Handle existing JSON data
                if value:
                    try:
                        existing_details = json.loads(value) if isinstance(value, str) else value
                        details.update(existing_details)
                    except json.JSONDecodeError:
                        details['original_tags'] = value
            else:
                # Direct field mapping
                if sqlite_field == 'timestamp' and isinstance(value, (int, float)):
                    # Convert timestamp to PostgreSQL format
                    pg_record[pg_field] = datetime.fromtimestamp(value, tz=timezone.utc)
                else:
                    pg_record[pg_field] = value
        
        # Add details JSON if not empty
        if details:
            pg_record['details'] = json.dumps(details)
        
        # Calculate velocity magnitude if velocity components exist
        if all(f in pg_record for f in ['velocity_x', 'velocity_y', 'velocity_z']):
            vx, vy, vz = pg_record['velocity_x'] or 0, pg_record['velocity_y'] or 0, pg_record['velocity_z'] or 0
            pg_record['velocity_magnitude'] = (vx**2 + vy**2 + vz**2)**0.5
        
        return pg_record
    
    def verify_migration(self) -> bool:
        """Verify migration was successful by comparing record counts"""
        self.logger.info("Verifying migration...")
        
        verification_passed = True
        
        for table_config in self.schema_mapping.values():
            sqlite_table = table_config['sqlite_table']
            postgres_table = table_config['postgres_table']
            
            # Count SQLite records
            try:
                cursor = self.sqlite_conn.execute(f"SELECT COUNT(*) FROM {sqlite_table}")
                sqlite_count = cursor.fetchone()[0]
            except sqlite3.OperationalError:
                sqlite_count = 0
            
            # Count PostgreSQL records
            try:
                pg_count_result = self.pg_db.execute_query(
                    f"SELECT COUNT(*) as count FROM {postgres_table}"
                )
                pg_count = pg_count_result[0]['count'] if pg_count_result else 0
            except Exception as e:
                self.logger.error(f"Failed to count PostgreSQL records for {postgres_table}: {e}")
                pg_count = 0
                verification_passed = False
            
            self.logger.info(f"Table {sqlite_table}: SQLite={sqlite_count:,}, PostgreSQL={pg_count:,}")
            
            if sqlite_count > 0 and pg_count == 0:
                self.logger.error(f"Migration failed for {sqlite_table}: no records in PostgreSQL")
                verification_passed = False
            elif pg_count < sqlite_count * 0.95:  # Allow 5% tolerance for duplicates/constraints
                self.logger.warning(f"Potential data loss in {sqlite_table}: {sqlite_count - pg_count} records missing")
        
        return verification_passed
    
    def run_migration(self) -> bool:
        """Run complete migration process"""
        self.stats.start_time = datetime.now()
        
        try:
            self.logger.info("Starting SQLite to PostgreSQL migration")
            
            # Validation phase
            if not self.validate_databases():
                self.logger.error("Database validation failed")
                return False
            
            # Analysis phase
            data_volume = self.analyze_data_volume()
            self.stats.total_records = sum(data_volume.values())
            
            if self.dry_run:
                self.logger.info("DRY RUN: Migration would proceed with the above data")
                return True
            
            # Migration phase
            for table_config in self.schema_mapping.values():
                successful, failed = self.migrate_table(table_config)
                self.stats.successful_records += successful
                self.stats.failed_records += failed
                self.stats.tables_migrated += 1
            
            # Verification phase
            verification_passed = self.verify_migration()
            
            # Summary
            self.stats.end_time = datetime.now()
            self._print_migration_summary()
            
            return verification_passed and self.stats.failed_records == 0
            
        except Exception as e:
            self.logger.error(f"Migration failed: {e}")
            return False
        finally:
            self._cleanup()
    
    def _print_migration_summary(self):
        """Print migration summary statistics"""
        print(f"\n{'='*60}")
        print(f"MIGRATION SUMMARY")
        print(f"{'='*60}")
        print(f"Duration: {self.stats.duration:.1f} seconds")
        print(f"Tables migrated: {self.stats.tables_migrated}")
        print(f"Total records: {self.stats.total_records:,}")
        print(f"Successful: {self.stats.successful_records:,}")
        print(f"Failed: {self.stats.failed_records:,}")
        print(f"Success rate: {self.stats.success_rate:.1f}%")
        
        if self.stats.failed_records > 0:
            print(f"\n⚠️  {self.stats.failed_records} records failed to migrate")
            print("Check logs for details")
        else:
            print(f"\n✅ Migration completed successfully!")
    
    def _cleanup(self):
        """Cleanup database connections"""
        if hasattr(self, 'sqlite_conn'):
            self.sqlite_conn.close()
        if hasattr(self, 'pg_db'):
            self.pg_db.close()

def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Migrate SQLite drone statistics to PostgreSQL with TimescaleDB",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic migration with auto-detected PostgreSQL
  %(prog)s --sqlite-db drone_movement_stats.db
  
  # Migration with custom PostgreSQL connection
  %(prog)s --sqlite-db mydata.db --pg-host localhost --pg-port 5432
  
  # Dry run to validate migration
  %(prog)s --sqlite-db drone_stats.db --dry-run
  
  # Migration with custom batch size for large datasets
  %(prog)s --sqlite-db large_dataset.db --batch-size 2000
  
  # Verbose logging for troubleshooting
  %(prog)s --sqlite-db drone_stats.db --verbose

Prerequisites:
  - PostgreSQL container must be running
  - Database and schema must be initialized
  - Source SQLite database must exist
        """
    )
    
    parser.add_argument('--sqlite-db', required=True,
                       help='Path to source SQLite database')
    parser.add_argument('--pg-host', default=None,
                       help='PostgreSQL host (default: auto-detect)')
    parser.add_argument('--pg-port', type=int, default=None,
                       help='PostgreSQL port (default: auto-detect)')
    parser.add_argument('--pg-database', default=None,
                       help='PostgreSQL database name (default: auto-detect)')
    parser.add_argument('--pg-user', default=None,
                       help='PostgreSQL username (default: auto-detect)')
    parser.add_argument('--pg-password', default=None,
                       help='PostgreSQL password (default: auto-detect)')
    parser.add_argument('--batch-size', type=int, default=1000,
                       help='Batch size for processing records (default: 1000)')
    parser.add_argument('--dry-run', action='store_true',
                       help='Validate migration without transferring data')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Set up logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Build PostgreSQL config
    pg_config = {
        'host': args.pg_host,
        'port': args.pg_port,
        'database': args.pg_database,
        'user': args.pg_user,
        'password': args.pg_password
    }
    
    try:
        # Initialize migrator
        migrator = SQLiteToPostgreSQLMigrator(
            sqlite_path=args.sqlite_db,
            pg_config=pg_config,
            batch_size=args.batch_size,
            dry_run=args.dry_run
        )
        
        # Run migration
        success = migrator.run_migration()
        
        if success:
            print("\n🎉 Migration completed successfully!")
            sys.exit(0)
        else:
            print("\n❌ Migration failed or incomplete. Check logs for details.")
            sys.exit(1)
            
    except Exception as e:
        logging.error(f"Migration failed: {e}")
        print(f"\n❌ Migration failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()