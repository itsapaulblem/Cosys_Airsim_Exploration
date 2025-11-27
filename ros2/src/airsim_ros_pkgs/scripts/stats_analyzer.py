#!/usr/bin/env python3
"""
Drone Movement Statistics Analyzer and CLI Tool

This tool provides analysis, querying, and visualization capabilities for
drone movement statistics collected by drone_stats_logger.py.

Features:
- Query statistics by vehicle, time range, or event type
- Generate reports and export data in multiple formats
- Real-time fleet monitoring and analysis
- Interactive data visualization and plotting
- Aggregate statistics across multiple vehicles

Usage:
    python3 stats_analyzer.py --help
    python3 stats_analyzer.py summary --vehicle Droan1
    python3 stats_analyzer.py events --last-hours 24 --export csv
    python3 stats_analyzer.py plot distance --vehicle-pattern "Drone_*"
    python3 stats_analyzer.py monitor --live
"""

import argparse
import sys
import json
import csv
import time
import os
import logging
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass

# Import database abstraction layer
from database_interface import DatabaseInterface, DatabaseType

try:
    import matplotlib.pyplot as plt
    import pandas as pd
    HAS_PLOTTING = True
except ImportError:
    HAS_PLOTTING = False
    print("WARNING: matplotlib/pandas not available. Plotting features disabled.")

try:
    import rclpy
    from rclpy.node import Node
    from mission_search_interfaces.msg import MissionEvent, MissionStatus, DetectionTarget
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("WARNING: ROS2 not available. Live monitoring disabled.")

@dataclass
class VehicleStats:
    """Container for vehicle statistics summary"""
    name: str
    total_distance: float
    flight_time: float
    events_count: int
    targets_detected: int
    waypoints_completed: int
    area_covered: float
    last_active: datetime

class DroneStatsAnalyzer:
    """Main analyzer class for drone movement statistics with PostgreSQL/SQLite support"""
    
    def __init__(self, database_type: Optional[str] = None, database_path: str = "drone_movement_stats.db"):
        """
        Initialize analyzer with unified database interface
        
        Args:
            database_type: 'postgresql', 'sqlite', or None for auto-detection
            database_path: Path to SQLite database (used if database_type is 'sqlite')
        """
        self.logger = logging.getLogger(__name__)
        
        # Set SQLite path in environment if specified
        if database_type == 'sqlite' or database_type is None:
            os.environ['SQLITE_DATABASE_PATH'] = database_path
        
        try:
            # Initialize database interface with auto-detection or specified type
            self.db = DatabaseInterface(db_type=database_type)
            self.logger.info(f"Connected to {self.db.db_type.value} database")
            
        except Exception as e:
            print(f"ERROR: Failed to connect to database: {e}")
            if database_type == 'postgresql':
                print("PostgreSQL connection failed. Check if containers are running:")
                print("  docker-compose -f docker-compose-master.yml up postgres-stats")
            elif database_type == 'sqlite':
                print(f"SQLite database not found: {database_path}")
                print("Make sure drone_stats_logger.py has been running to collect data.")
            sys.exit(1)
    
    def get_connection(self):
        """Get database connection using unified interface"""
        return self.db.get_connection()
    
    def get_all_vehicles(self) -> List[str]:
        """Get list of all vehicles in database"""
        try:
            results = self.db.execute_query(
                "SELECT DISTINCT vehicle_name FROM vehicle_statistics ORDER BY vehicle_name"
            )
            return [row['vehicle_name'] for row in results]
        except Exception as e:
            self.logger.error(f"Failed to get vehicles: {e}")
            return []
    
    def get_vehicle_summary(self, vehicle_name: str) -> Optional[VehicleStats]:
        """Get comprehensive statistics summary for a vehicle"""
        try:
            # Use the database interface's built-in method for vehicle statistics
            results = self.db.get_vehicle_statistics(vehicle_name)
            
            if not results:
                return None
            
            row = results[0]  # Get first result
            
            # Handle different timestamp formats between SQLite and PostgreSQL
            last_updated = row['last_updated']
            if isinstance(last_updated, str):
                last_active = datetime.fromisoformat(last_updated)
            elif isinstance(last_updated, (int, float)):
                last_active = datetime.fromtimestamp(last_updated)
            else:
                last_active = last_updated
                
            return VehicleStats(
                name=row['vehicle_name'],
                total_distance=row['total_distance_m'],
                flight_time=row['flight_time_s'],
                events_count=row['events_count'],
                targets_detected=row['targets_detected'],
                waypoints_completed=row['waypoints_completed'],
                area_covered=row['area_covered_m2'],
                last_active=last_active
            )
        except Exception as e:
            self.logger.error(f"Failed to get vehicle summary for {vehicle_name}: {e}")
            return None
    
    def get_events(self, vehicle_name: str = None, hours_back: int = 24, 
                   event_type: str = None) -> List[Dict[str, Any]]:
        """Get events with optional filtering"""
        try:
            # Build query based on database type
            if self.db.db_type == DatabaseType.POSTGRESQL:
                query = "SELECT * FROM movement_events WHERE timestamp >= %s"
                params = [datetime.now() - timedelta(hours=hours_back)]
                
                if vehicle_name:
                    query += " AND vehicle_name = %s"
                    params.append(vehicle_name)
                
                if event_type:
                    query += " AND event_type = %s"
                    params.append(event_type)
            else:
                # SQLite uses different timestamp comparison
                query = "SELECT * FROM movement_events WHERE timestamp >= ?"
                cutoff_timestamp = (datetime.now() - timedelta(hours=hours_back)).timestamp()
                params = [cutoff_timestamp]
                
                if vehicle_name:
                    query += " AND vehicle_name = ?"
                    params.append(vehicle_name)
                
                if event_type:
                    query += " AND event_type = ?"
                    params.append(event_type)
                    
            query += " ORDER BY timestamp DESC"
            
            return self.db.execute_query(query, tuple(params))
            
        except Exception as e:
            self.logger.error(f"Failed to get events: {e}")
            return []
    
    def get_distance_over_time(self, vehicle_name: str, 
                              hours_back: int = 24) -> List[Tuple[datetime, float]]:
        """Get distance traveled over time for plotting"""
        events = self.get_events(vehicle_name, hours_back, 'MOVEMENT')
        
        distance_points = []
        cumulative_distance = 0.0
        
        for event in reversed(events):  # Process chronologically
            # Handle different timestamp formats
            timestamp_val = event['timestamp']
            if isinstance(timestamp_val, str):
                timestamp = datetime.fromisoformat(timestamp_val)
            elif isinstance(timestamp_val, (int, float)):
                timestamp = datetime.fromtimestamp(timestamp_val)
            else:
                timestamp = timestamp_val
            
            # Handle details field (could be JSON string in SQLite or dict in PostgreSQL)
            details = event.get('details') or event.get('tags')
            if details:
                if isinstance(details, str):
                    try:
                        details = json.loads(details)
                    except json.JSONDecodeError:
                        details = {}
                
                if isinstance(details, dict) and 'distance_m' in details:
                    cumulative_distance += details['distance_m']
                elif 'distance_moved' in event:
                    cumulative_distance += event['distance_moved'] or 0.0
            
            distance_points.append((timestamp, cumulative_distance))
        
        return distance_points
    
    def print_summary_report(self, vehicle_pattern: str = None):
        """Print comprehensive summary report"""
        vehicles = self.get_all_vehicles()
        
        if vehicle_pattern:
            import fnmatch
            vehicles = [v for v in vehicles if fnmatch.fnmatch(v, vehicle_pattern)]
        
        if not vehicles:
            print("No vehicles found matching pattern" + (f": {vehicle_pattern}" if vehicle_pattern else ""))
            return
        
        print(f"\n{'='*60}")
        print(f"DRONE FLEET STATISTICS SUMMARY")
        print(f"{'='*60}")
        print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Show database information based on type
        if self.db.db_type == DatabaseType.POSTGRESQL:
            print(f"Database: PostgreSQL ({self.db.config.pg_host}:{self.db.config.pg_port}/{self.db.config.pg_database})")
            print(f"Schema: {self.db.config.pg_schema}")
        else:
            print(f"Database: SQLite ({self.db.config.sqlite_path})")
        
        print(f"Vehicles: {len(vehicles)}")
        print(f"{'='*60}")
        
        total_distance = 0.0
        total_flight_time = 0.0
        total_targets = 0
        total_waypoints = 0
        
        for vehicle_name in vehicles:
            stats = self.get_vehicle_summary(vehicle_name)
            if not stats:
                continue
                
            print(f"\nVehicle: {stats.name}")
            print(f"  Distance Traveled: {stats.total_distance:.2f} m")
            print(f"  Flight Time: {stats.flight_time:.1f} s ({stats.flight_time/60:.1f} min)")
            print(f"  Events Recorded: {stats.events_count}")
            print(f"  Targets Detected: {stats.targets_detected}")
            print(f"  Waypoints Completed: {stats.waypoints_completed}")
            print(f"  Area Covered: {stats.area_covered:.2f} m²")
            print(f"  Last Active: {stats.last_active.strftime('%Y-%m-%d %H:%M:%S')}")
            
            total_distance += stats.total_distance
            total_flight_time += stats.flight_time
            total_targets += stats.targets_detected
            total_waypoints += stats.waypoints_completed
        
        print(f"\n{'='*60}")
        print(f"FLEET TOTALS")
        print(f"{'='*60}")
        print(f"Total Distance: {total_distance:.2f} m")
        print(f"Total Flight Time: {total_flight_time:.1f} s ({total_flight_time/60:.1f} min)")
        print(f"Total Targets: {total_targets}")
        print(f"Total Waypoints: {total_waypoints}")
        print(f"Average Distance per Vehicle: {total_distance/len(vehicles):.2f} m")
        print(f"Average Flight Time per Vehicle: {total_flight_time/len(vehicles):.1f} s")
    
    def print_events_report(self, vehicle_name: str = None, hours_back: int = 24, 
                           event_type: str = None, limit: int = 50):
        """Print detailed events report"""
        events = self.get_events(vehicle_name, hours_back, event_type)
        
        if not events:
            print("No events found matching criteria")
            return
        
        print(f"\n{'='*80}")
        print(f"MOVEMENT EVENTS REPORT")
        print(f"{'='*80}")
        print(f"Filter: Vehicle={vehicle_name or 'All'}, Hours={hours_back}, Type={event_type or 'All'}")
        print(f"Events Found: {len(events)} (showing first {min(limit, len(events))})")
        print(f"{'='*80}")
        
        for i, event in enumerate(events[:limit]):
            timestamp = datetime.fromisoformat(event['timestamp'])
            print(f"\n[{i+1:3d}] {timestamp.strftime('%H:%M:%S')} - {event['vehicle_name']}")
            print(f"      Event: {event['event_type']}")
            if event['details']:
                try:
                    details = json.loads(event['details']) if isinstance(event['details'], str) else event['details']
                    for key, value in details.items():
                        if isinstance(value, float):
                            print(f"      {key}: {value:.3f}")
                        else:
                            print(f"      {key}: {value}")
                except:
                    print(f"      Details: {event['details']}")
    
    def export_data(self, format_type: str, output_path: str, 
                   vehicle_name: str = None, hours_back: int = 24):
        """Export data in specified format (csv, json)"""
        events = self.get_events(vehicle_name, hours_back)
        
        if format_type.lower() == 'csv':
            self._export_csv(events, output_path)
        elif format_type.lower() == 'json':
            self._export_json(events, output_path)
        else:
            print(f"ERROR: Unsupported format '{format_type}'. Use 'csv' or 'json'")
            return
            
        print(f"Exported {len(events)} events to {output_path}")
    
    def _export_csv(self, events: List[Dict], output_path: str):
        """Export events to CSV format"""
        with open(output_path, 'w', newline='') as csvfile:
            if not events:
                return
                
            fieldnames = ['timestamp', 'vehicle_name', 'event_type', 'details']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for event in events:
                # Flatten details for CSV
                row = {
                    'timestamp': event['timestamp'],
                    'vehicle_name': event['vehicle_name'],
                    'event_type': event['event_type'],
                    'details': json.dumps(event['details']) if event['details'] else ''
                }
                writer.writerow(row)
    
    def _export_json(self, events: List[Dict], output_path: str):
        """Export events to JSON format"""
        with open(output_path, 'w') as jsonfile:
            json.dump(events, jsonfile, indent=2, default=str)
    
    def plot_distance_over_time(self, vehicle_pattern: str = None, hours_back: int = 24):
        """Plot distance traveled over time"""
        if not HAS_PLOTTING:
            print("ERROR: matplotlib not available. Install with: pip install matplotlib pandas")
            return
        
        vehicles = self.get_all_vehicles()
        if vehicle_pattern:
            import fnmatch
            vehicles = [v for v in vehicles if fnmatch.fnmatch(v, vehicle_pattern)]
        
        plt.figure(figsize=(12, 8))
        
        for vehicle_name in vehicles:
            distance_points = self.get_distance_over_time(vehicle_name, hours_back)
            if not distance_points:
                continue
                
            times, distances = zip(*distance_points)
            plt.plot(times, distances, label=vehicle_name, marker='o', markersize=3)
        
        plt.title(f'Drone Distance Traveled Over Time (Last {hours_back}h)')
        plt.xlabel('Time')
        plt.ylabel('Cumulative Distance (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xticks(rotation=45)
        plt.tight_layout()
        
        output_path = f"drone_distance_plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
        plt.show()

class LiveMonitor:
    """Real-time monitoring of drone statistics"""
    
    def __init__(self, analyzer: DroneStatsAnalyzer):
        self.analyzer = analyzer
        self.running = False
    
    def start_monitoring(self, refresh_interval: int = 5):
        """Start live monitoring display"""
        if not HAS_ROS2:
            print("ERROR: ROS2 not available. Install ROS2 for live monitoring.")
            return
        
        print("Starting live monitoring... Press Ctrl+C to stop")
        self.running = True
        
        try:
            while self.running:
                self._clear_screen()
                self._print_live_status()
                time.sleep(refresh_interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")
    
    def _clear_screen(self):
        """Clear terminal screen"""
        import os
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def _print_live_status(self):
        """Print current fleet status"""
        vehicles = self.analyzer.get_all_vehicles()
        
        print(f"{'='*60}")
        print(f"LIVE DRONE FLEET MONITOR - {datetime.now().strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        
        for vehicle_name in vehicles:
            stats = self.analyzer.get_vehicle_summary(vehicle_name)
            if not stats:
                continue
                
            time_since_update = datetime.now() - stats.last_active
            status = "ACTIVE" if time_since_update.seconds < 30 else "INACTIVE"
            
            print(f"{vehicle_name:15} | {status:8} | "
                  f"Dist: {stats.total_distance:8.1f}m | "
                  f"Time: {stats.flight_time:6.0f}s | "
                  f"Events: {stats.events_count:4d}")
        
        print(f"{'='*60}")
        print("Refreshing every 5 seconds... Press Ctrl+C to stop")

def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Analyze drone movement statistics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage (auto-detects database type)
  %(prog)s summary                                 # Show all vehicles summary
  %(prog)s summary --vehicle Droan1               # Show specific vehicle
  %(prog)s events --last-hours 12 --limit 20     # Show recent events
  
  # Database type selection
  %(prog)s summary --db-type postgresql           # Use PostgreSQL (container must be running)
  %(prog)s summary --db-type sqlite -d mydata.db # Use specific SQLite file
  
  # PostgreSQL with custom connection
  %(prog)s summary --db-type postgresql --pg-host localhost --pg-port 5432
  
  # Advanced filtering and export
  %(prog)s events --vehicle "Drone*" --type TAKEOFF  # Filter events
  %(prog)s events --export csv --output drone_data.csv    # Export to CSV
  %(prog)s plot distance --vehicle-pattern "*"   # Plot all vehicles
  %(prog)s monitor --live                         # Real-time monitoring
        """
    )
    
    parser.add_argument('--database', '-d', default='drone_movement_stats.db',
                       help='Path to SQLite database (used if --db-type is sqlite)')
    parser.add_argument('--db-type', choices=['auto', 'sqlite', 'postgresql'], 
                       default='auto', help='Database type (default: auto-detect)')
    parser.add_argument('--pg-host', default=None, 
                       help='PostgreSQL host (overrides environment)')
    parser.add_argument('--pg-port', type=int, default=None,
                       help='PostgreSQL port (overrides environment)')
    parser.add_argument('--pg-database', default=None,
                       help='PostgreSQL database name (overrides environment)')
    parser.add_argument('--pg-user', default=None,
                       help='PostgreSQL username (overrides environment)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Summary command
    summary_parser = subparsers.add_parser('summary', help='Show vehicle statistics summary')
    summary_parser.add_argument('--vehicle', help='Specific vehicle name to analyze')
    summary_parser.add_argument('--vehicle-pattern', help='Vehicle name pattern (e.g., "Drone*")')
    
    # Events command
    events_parser = subparsers.add_parser('events', help='Show movement events')
    events_parser.add_argument('--vehicle', help='Filter by vehicle name')
    events_parser.add_argument('--last-hours', type=int, default=24, help='Hours of history to show')
    events_parser.add_argument('--type', help='Filter by event type (TAKEOFF, LANDING, etc.)')
    events_parser.add_argument('--limit', type=int, default=50, help='Maximum events to show')
    events_parser.add_argument('--export', choices=['csv', 'json'], help='Export format')
    events_parser.add_argument('--output', help='Output file path for export')
    
    # Plot command
    if HAS_PLOTTING:
        plot_parser = subparsers.add_parser('plot', help='Generate plots and visualizations')
        plot_parser.add_argument('type', choices=['distance'], help='Type of plot to generate')
        plot_parser.add_argument('--vehicle-pattern', help='Vehicle name pattern')
        plot_parser.add_argument('--last-hours', type=int, default=24, help='Hours of data to plot')
    
    # Monitor command
    if HAS_ROS2:
        monitor_parser = subparsers.add_parser('monitor', help='Real-time fleet monitoring')
        monitor_parser.add_argument('--live', action='store_true', help='Start live monitoring')
        monitor_parser.add_argument('--refresh', type=int, default=5, help='Refresh interval in seconds')
    
    args = parser.parse_args()
    
    # Set up logging
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format='%(levelname)s: %(message)s')
    else:
        logging.basicConfig(level=logging.WARNING)
    
    if not args.command:
        parser.print_help()
        return
    
    # Set PostgreSQL environment variables if provided
    if args.pg_host:
        os.environ['POSTGRES_HOST'] = args.pg_host
    if args.pg_port:
        os.environ['POSTGRES_PORT'] = str(args.pg_port)
    if args.pg_database:
        os.environ['POSTGRES_DB'] = args.pg_database
    if args.pg_user:
        os.environ['POSTGRES_USER'] = args.pg_user
    
    # Initialize analyzer
    try:
        db_type = None if args.db_type == 'auto' else args.db_type
        analyzer = DroneStatsAnalyzer(database_type=db_type, database_path=args.database)
        
        print(f"Connected to {analyzer.db.db_type.value} database")
        if args.verbose:
            if analyzer.db.db_type == DatabaseType.POSTGRESQL:
                print(f"PostgreSQL: {analyzer.db.config.pg_host}:{analyzer.db.config.pg_port}/{analyzer.db.config.pg_database}")
            else:
                print(f"SQLite: {analyzer.db.config.sqlite_path}")
        
    except Exception as e:
        print(f"ERROR: Failed to initialize analyzer: {e}")
        sys.exit(1)
    
    # Execute commands
    try:
        if args.command == 'summary':
            if args.vehicle:
                stats = analyzer.get_vehicle_summary(args.vehicle)
                if stats:
                    analyzer.print_summary_report(args.vehicle)
                else:
                    print(f"Vehicle '{args.vehicle}' not found in database")
            else:
                analyzer.print_summary_report(args.vehicle_pattern)
        
        elif args.command == 'events':
            if args.export:
                output_path = args.output or f"drone_events_{datetime.now().strftime('%Y%m%d_%H%M%S')}.{args.export}"
                analyzer.export_data(args.export, output_path, args.vehicle, args.last_hours)
            else:
                analyzer.print_events_report(args.vehicle, args.last_hours, args.type, args.limit)
        
        elif args.command == 'plot' and HAS_PLOTTING:
            if args.type == 'distance':
                analyzer.plot_distance_over_time(args.vehicle_pattern, args.last_hours)
        
        elif args.command == 'monitor' and HAS_ROS2:
            if args.live:
                monitor = LiveMonitor(analyzer)
                monitor.start_monitoring(args.refresh)
    
    except Exception as e:
        print(f"ERROR: Command failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()