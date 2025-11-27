#!/usr/bin/env python3
"""
Drone Movement Statistics - Complete Workflow Example

This script demonstrates how to use the drone statistics system:
1. drone_stats_logger.py - Collects movement data from ROS2 topics
2. stats_analyzer.py - Analyzes and visualizes the collected data

Usage:
    # Terminal 1: Start the statistics logger
    python3 drone_stats_logger.py
    
    # Terminal 2: Run mission and fly drones
    ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
    
    # Terminal 3: Analyze collected statistics
    python3 stats_analyzer.py summary
    python3 stats_analyzer.py events --last-hours 1
    
    # Or run this example script to see sample analysis
    python3 drone_stats_example.py
"""

import os
import sys
import time
import subprocess
from pathlib import Path
from datetime import datetime

def print_header(title: str):
    """Print formatted section header"""
    print(f"\n{'='*60}")
    print(f"{title.upper()}")
    print(f"{'='*60}")

def run_analyzer_command(command: str, description: str):
    """Run stats analyzer command and display output"""
    print(f"\n{description}")
    print(f"Command: python3 stats_analyzer.py {command}")
    print(f"{'-'*50}")
    
    try:
        result = subprocess.run(
            [sys.executable, "stats_analyzer.py"] + command.split(),
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent
        )
        
        if result.returncode == 0:
            print(result.stdout)
        else:
            print(f"ERROR: {result.stderr}")
            
    except Exception as e:
        print(f"ERROR: Failed to run command: {e}")

def check_database_exists():
    """Check if statistics database exists"""
    db_path = Path("drone_movement_stats.db")
    if not db_path.exists():
        print(f"Database not found: {db_path}")
        print("\nTo collect statistics data:")
        print("1. Start the logger: python3 drone_stats_logger.py")
        print("2. Run your drone missions with ROS2")
        print("3. Let it collect data for a few minutes")
        print("4. Then run this example script again")
        return False
    else:
        print(f"Database found: {db_path}")
        stat = db_path.stat()
        print(f"   Size: {stat.st_size} bytes")
        print(f"   Modified: {datetime.fromtimestamp(stat.st_mtime)}")
        return True

def main():
    """Demonstrate complete drone statistics workflow"""
    
    print_header("Drone Movement Statistics - Example Workflow")
    print("This example demonstrates the complete drone statistics analysis system.")
    
    # Check if we're in the right directory
    scripts_dir = Path(__file__).parent
    os.chdir(scripts_dir)
    
    print(f"\nWorking directory: {scripts_dir}")
    
    # Check required files
    print("\nChecking required files...")
    logger_exists = (scripts_dir / "drone_stats_logger.py").exists()
    analyzer_exists = (scripts_dir / "stats_analyzer.py").exists()
    
    print(f"   drone_stats_logger.py: {'Ok' if logger_exists else 'Not Found'}")
    print(f"   stats_analyzer.py: {'Ok' if analyzer_exists else 'Not Found'}")
    
    if not (logger_exists and analyzer_exists):
        print("\nERROR: Required scripts not found in current directory.")
        return
    
    # Check database
    print("\nChecking statistics database...")
    if not check_database_exists():
        return
    
    # Example 1: Fleet Summary
    print_header("Example 1: Fleet Statistics Summary")
    run_analyzer_command("summary", "Show overall fleet statistics")
    
    # Example 2: Recent Events  
    print_header("Example 2: Recent Movement Events")
    run_analyzer_command("events --last-hours 1 --limit 10", "Show last 10 events from past hour")
    
    # Example 3: Specific Vehicle Analysis
    print_header("Example 3: Vehicle-Specific Analysis")
    run_analyzer_command("summary --vehicle-pattern 'Drone*'", "Analyze all vehicles with 'Drone' pattern")
    
    # Example 4: Event Filtering
    print_header("Example 4: Event Type Filtering")
    run_analyzer_command("events --type TAKEOFF --last-hours 24 --limit 5", "Show recent takeoff events")
    
    # Example 5: Data Export
    print_header("Example 5: Data Export")
    output_file = f"drone_stats_example_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    run_analyzer_command(f"events --export csv --output {output_file} --last-hours 24", 
                        f"Export last 24 hours to CSV: {output_file}")
    
    # Show usage instructions
    print_header("Next Steps and Usage Instructions")
    print("""
COMPLETE WORKFLOW:

1. START DATA COLLECTION:
   Terminal 1: python3 drone_stats_logger.py
   (This runs continuously, collecting data from ROS2 topics)

2. RUN YOUR DRONE MISSIONS:
   Terminal 2: ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py
   (Or any other mission launch that creates drone movement)

3. ANALYZE COLLECTED DATA:
   Terminal 3: python3 stats_analyzer.py summary
               python3 stats_analyzer.py events --last-hours 2
               python3 stats_analyzer.py plot distance  # Requires matplotlib

4. LIVE MONITORING:
   python3 stats_analyzer.py monitor --live  # Real-time fleet status

AVAILABLE ANALYSIS COMMANDS:

Basic Analysis:
   python3 stats_analyzer.py summary                    # Fleet overview
   python3 stats_analyzer.py summary --vehicle Droan1   # Single vehicle
   python3 stats_analyzer.py events --last-hours 6      # Recent events

Filtering and Export:
   python3 stats_analyzer.py events --type LANDING --limit 20
   python3 stats_analyzer.py events --export csv --output my_data.csv
   python3 stats_analyzer.py events --vehicle-pattern "PX4_*"

Visualization (requires matplotlib):
   python3 stats_analyzer.py plot distance              # Distance over time
   python3 stats_analyzer.py plot distance --vehicle-pattern "Drone*"

Real-time Monitoring (requires ROS2):
   python3 stats_analyzer.py monitor --live             # Live fleet status
   python3 stats_analyzer.py monitor --live --refresh 10  # 10-second refresh

💡 TIP: Leave drone_stats_logger.py running in the background during all
       drone operations to continuously collect movement statistics!
    """)

if __name__ == '__main__':
    main()