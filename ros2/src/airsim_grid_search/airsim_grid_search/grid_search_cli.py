#!/usr/bin/env python3

"""
Grid Search CLI Tool

Command-line interface for testing and operating the grid search mission system.
Provides easy access to grid generation, mission execution, and monitoring.

Author: Cosys-Lab
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import argparse
import sys
import time
import asyncio
from typing import Optional

from geometry_msgs.msg import Point
from airsim_mission_interfaces.action import GridSearch
from airsim_mission_interfaces.msg import SearchArea, GridParams, SafetyParams
from airsim_mission_interfaces.srv import GenerateGrid, GetMissionStatus


class GridSearchCLI(Node):
    """Command-line interface for grid search operations"""
    
    def __init__(self):
        super().__init__('grid_search_cli')
        
        # Action client
        self.action_client = ActionClient(self, GridSearch, 'grid_search')
        
        # Service clients
        self.generate_grid_client = self.create_client(GenerateGrid, 'generate_grid')
        self.status_client = self.create_client(GetMissionStatus, 'get_mission_status')
        
        self.get_logger().info("Grid Search CLI initialized")
    
    async def generate_grid_command(self, args):
        """Generate and display a waypoint grid"""
        print("Generating waypoint grid...")
        
        # Create search area
        search_area = SearchArea()
        if args.gps_coords:
            search_area.coordinate_type = SearchArea.COORDINATE_TYPE_GPS
            search_area.north_lat = args.north
            search_area.south_lat = args.south
            search_area.east_lon = args.east
            search_area.west_lon = args.west
        else:
            search_area.coordinate_type = SearchArea.COORDINATE_TYPE_LOCAL
            search_area.north_m = args.north
            search_area.south_m = args.south
            search_area.east_m = args.east
            search_area.west_m = args.west
        
        search_area.default_altitude = args.altitude
        search_area.area_name = args.area_name or "CLI_Generated_Area"
        
        # Create grid parameters
        grid_params = GridParams()
        grid_params.pattern_type = GridParams.PATTERN_BOUSTROPHEDON
        grid_params.lane_spacing = args.lane_spacing
        grid_params.waypoint_spacing = args.waypoint_spacing
        grid_params.speed_ms = args.speed
        grid_params.initial_heading = args.heading
        grid_params.reverse_direction = args.reverse_direction
        grid_params.optimize_path = args.optimize
        
        # Call service
        if not self.generate_grid_client.wait_for_service(timeout_sec=5.0):
            print("❌ Grid generation service not available")
            return False
        
        request = GenerateGrid.Request()
        request.search_area = search_area
        request.grid_params = grid_params
        request.mission_id = f"cli_mission_{int(time.time())}"
        
        try:
            response = await self.generate_grid_client.call_async(request)
            
            if response.success:
                waypoints = response.generated_waypoints
                print(f"✅ Grid generated successfully!")
                print(f"   Waypoints: {len(waypoints.waypoints)}")
                print(f"   Total distance: {waypoints.total_distance:.1f} m")
                print(f"   Estimated time: {waypoints.estimated_flight_time:.1f} s")
                print(f"   Coverage area: {response.total_coverage_area:.1f} m²")
                
                if args.show_waypoints:
                    print("\n📍 Waypoints:")
                    for i, wp in enumerate(waypoints.waypoints):
                        print(f"   {i+1:3d}: ({wp.position.x:6.1f}, {wp.position.y:6.1f}, {wp.position.z:6.1f}) "
                              f"speed: {wp.speed_ms:.1f} m/s")
                
                return True
            else:
                print(f"❌ Grid generation failed: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Service call failed: {str(e)}")
            return False
    
    async def execute_mission_command(self, args):
        """Execute a grid search mission"""
        print(f"Starting grid search mission for vehicle: {args.vehicle}")
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            print("❌ Grid search action server not available")
            return False
        
        # Create mission goal
        goal = GridSearch.Goal()
        
        # Create search area
        search_area = SearchArea()
        if args.gps_coords:
            search_area.coordinate_type = SearchArea.COORDINATE_TYPE_GPS
            search_area.north_lat = args.north
            search_area.south_lat = args.south
            search_area.east_lon = args.east
            search_area.west_lon = args.west
        else:
            search_area.coordinate_type = SearchArea.COORDINATE_TYPE_LOCAL
            search_area.north_m = args.north
            search_area.south_m = args.south
            search_area.east_m = args.east
            search_area.west_m = args.west
        
        search_area.default_altitude = args.altitude
        search_area.area_name = args.area_name or "CLI_Mission_Area"
        
        # Create grid parameters
        grid_params = GridParams()
        grid_params.pattern_type = GridParams.PATTERN_BOUSTROPHEDON
        grid_params.lane_spacing = args.lane_spacing
        grid_params.waypoint_spacing = args.waypoint_spacing
        grid_params.speed_ms = args.speed
        grid_params.initial_heading = args.heading
        grid_params.reverse_direction = args.reverse_direction
        grid_params.optimize_path = args.optimize
        
        # Create safety parameters
        safety_params = SafetyParams()
        safety_params.max_flight_time = args.max_flight_time
        safety_params.min_battery_percentage = args.min_battery / 100.0
        safety_params.return_home_battery = args.return_home_battery / 100.0
        safety_params.max_distance_from_home = args.max_distance
        safety_params.enable_geofence = args.enable_geofence
        safety_params.enable_return_home = args.return_home
        
        # Set home position if provided
        if args.home_x is not None and args.home_y is not None:
            safety_params.home_position = Point(x=args.home_x, y=args.home_y, z=args.home_z or 0.0)
        
        # Fill goal
        goal.search_area = search_area
        goal.grid_params = grid_params
        goal.safety_params = safety_params
        goal.mission_id = args.mission_id or f"cli_mission_{int(time.time())}"
        goal.vehicle_name = args.vehicle
        goal.auto_start = args.auto_start
        goal.return_home_on_completion = args.return_home
        
        # Send goal
        print("🚁 Sending mission goal...")
        send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        
        try:
            goal_handle = await send_goal_future
            
            if not goal_handle.accepted:
                print("❌ Mission goal rejected")
                return False
            
            print("✅ Mission goal accepted, executing...")
            
            # Wait for result
            result = await goal_handle.get_result_async()
            
            if result.result.success:
                print("🎉 Mission completed successfully!")
                print(f"   Total waypoints: {result.result.mission_stats.total_waypoints}")
                print(f"   Waypoints completed: {result.result.mission_stats.waypoints_completed}")
                print(f"   Total distance: {result.result.mission_stats.total_distance_m:.1f} m")
                print(f"   Flight time: {result.result.total_execution_time.sec}s")
                print(f"   Final battery: {result.result.final_battery_percentage:.1f}%")
            else:
                print(f"❌ Mission failed: {result.result.result_message}")
            
            return result.result.success
            
        except Exception as e:
            print(f"❌ Mission execution failed: {str(e)}")
            return False
    
    def feedback_callback(self, feedback_msg):
        """Handle mission feedback"""
        feedback = feedback_msg.feedback
        progress = feedback.progress
        
        print(f"📊 Progress: {feedback.completion_percentage:.1f}% - {feedback.status_message}")
        
        if hasattr(feedback, 'current_waypoint') and feedback.current_waypoint:
            wp = feedback.current_waypoint
            print(f"   Current waypoint: {wp.waypoint_index + 1}/{wp.total_waypoints}")
            print(f"   Distance to target: {wp.distance_to_waypoint:.1f} m")
    
    async def status_command(self, args):
        """Get mission status"""
        if not self.status_client.wait_for_service(timeout_sec=5.0):
            print("❌ Status service not available")
            return False
        
        request = GetMissionStatus.Request()
        request.mission_id = args.mission_id or ""
        
        try:
            response = await self.status_client.call_async(request)
            
            if response.success:
                print("📊 Mission Status:")
                print(f"   Active: {'Yes' if response.mission_active else 'No'}")
                
                if response.mission_active and response.mission_status:
                    status = response.mission_status
                    progress = status.progress
                    
                    states = {
                        0: "IDLE", 1: "PLANNING", 2: "EXECUTING", 
                        3: "PAUSED", 4: "COMPLETED", 5: "ABORTED",
                        6: "EMERGENCY", 7: "RETURNING_HOME"
                    }
                    
                    print(f"   State: {states.get(progress.mission_state, 'UNKNOWN')}")
                    print(f"   Progress: {progress.completion_percentage:.1f}%")
                    print(f"   Waypoints: {progress.waypoints_completed}/{progress.total_waypoints}")
                    
                    if hasattr(status, 'current_battery_level'):
                        print(f"   Battery: {status.current_battery_level:.1f}%")
                    
                    if hasattr(status, 'current_position'):
                        pos = status.current_position
                        print(f"   Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                
                return True
            else:
                print(f"❌ Status request failed: {response.message}")
                return False
                
        except Exception as e:
            print(f"❌ Status request failed: {str(e)}")
            return False


def create_parser():
    """Create argument parser"""
    parser = argparse.ArgumentParser(description="Grid Search Mission CLI")
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Generate grid command
    gen_parser = subparsers.add_parser('generate', help='Generate waypoint grid')
    add_area_args(gen_parser)
    add_grid_args(gen_parser)
    gen_parser.add_argument('--show-waypoints', action='store_true',
                           help='Show individual waypoints')
    
    # Execute mission command
    exec_parser = subparsers.add_parser('execute', help='Execute grid search mission')
    add_area_args(exec_parser)
    add_grid_args(exec_parser)
    add_safety_args(exec_parser)
    exec_parser.add_argument('--vehicle', default='drone_1',
                            help='Vehicle name (default: drone_1)')
    exec_parser.add_argument('--mission-id',
                            help='Mission ID (auto-generated if not provided)')
    exec_parser.add_argument('--auto-start', action='store_true',
                            help='Automatically start mission after planning')
    
    # Status command
    status_parser = subparsers.add_parser('status', help='Get mission status')
    status_parser.add_argument('--mission-id',
                              help='Mission ID (current mission if not provided)')
    
    return parser


def add_area_args(parser):
    """Add search area arguments"""
    parser.add_argument('--area-name', help='Search area name')
    parser.add_argument('--gps-coords', action='store_true',
                       help='Use GPS coordinates instead of local')
    parser.add_argument('--north', type=float, required=True,
                       help='North boundary (lat or meters)')
    parser.add_argument('--south', type=float, required=True,
                       help='South boundary (lat or meters)')
    parser.add_argument('--east', type=float, required=True,
                       help='East boundary (lon or meters)')
    parser.add_argument('--west', type=float, required=True,
                       help='West boundary (lon or meters)')
    parser.add_argument('--altitude', type=float, default=20.0,
                       help='Flight altitude (meters, default: 20)')


def add_grid_args(parser):
    """Add grid parameters arguments"""
    parser.add_argument('--lane-spacing', type=float, default=20.0,
                       help='Lane spacing (meters, default: 20)')
    parser.add_argument('--waypoint-spacing', type=float, default=10.0,
                       help='Waypoint spacing (meters, default: 10)')
    parser.add_argument('--speed', type=float, default=5.0,
                       help='Flight speed (m/s, default: 5)')
    parser.add_argument('--heading', type=float, default=0.0,
                       help='Initial heading (radians, default: 0 = north)')
    parser.add_argument('--reverse-direction', action='store_true',
                       help='Reverse direction on alternate lanes')
    parser.add_argument('--optimize', action='store_true',
                       help='Enable path optimization')


def add_safety_args(parser):
    """Add safety parameters arguments"""
    parser.add_argument('--max-flight-time', type=float, default=1800.0,
                       help='Maximum flight time (seconds, default: 1800)')
    parser.add_argument('--min-battery', type=float, default=20.0,
                       help='Minimum battery percentage (default: 20)')
    parser.add_argument('--return-home-battery', type=float, default=25.0,
                       help='Return home battery percentage (default: 25)')
    parser.add_argument('--max-distance', type=float, default=1000.0,
                       help='Maximum distance from home (meters, default: 1000)')
    parser.add_argument('--enable-geofence', action='store_true',
                       help='Enable geofence checking')
    parser.add_argument('--return-home', action='store_true',
                       help='Return home after mission completion')
    parser.add_argument('--home-x', type=float, help='Home position X')
    parser.add_argument('--home-y', type=float, help='Home position Y')
    parser.add_argument('--home-z', type=float, help='Home position Z')


async def main():
    """Main CLI entry point"""
    parser = create_parser()
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    rclpy.init()
    
    try:
        cli = GridSearchCLI()
        
        success = False
        
        if args.command == 'generate':
            success = await cli.generate_grid_command(args)
        elif args.command == 'execute':
            success = await cli.execute_mission_command(args)
        elif args.command == 'status':
            success = await cli.status_command(args)
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n🛑 Operation cancelled by user")
        return 1
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        return 1
    finally:
        try:
            cli.destroy_node()
        except:
            pass
        rclpy.shutdown()


def sync_main():
    """Synchronous wrapper for async main"""
    try:
        return asyncio.run(main())
    except KeyboardInterrupt:
        return 1


if __name__ == '__main__':
    sys.exit(sync_main())