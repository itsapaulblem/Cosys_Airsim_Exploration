#!/usr/bin/env python3
"""
ROS2 Adaptation of spiral_search_mission.py
Converts spiral search mission to use ROS2 action servers instead of direct AirSim control

Original: Uses GenericMission class with direct AirSim API calls
ROS2 Version: Uses mission action servers for spiral search patterns

Ultra-Clean Architecture: /VehicleName/actions/search_area with spiral pattern
"""

import math
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point32, Polygon
from mission_search_interfaces.action import SearchArea
from mission_search_interfaces.srv import SetSearchPattern
from mission_search_interfaces.msg import MissionStatus, TargetDetection
from typing import List, Tuple, Optional

class SpiralSearchROS2(Node):
    """ROS2 Spiral Search Mission Client using ultra-clean architecture"""
    
    def __init__(self, vehicle_name: str = "Droan1"):
        super().__init__('spiral_search_ros2')
        
        self.vehicle_name = vehicle_name
        self.mission_complete = False
        self.target_found = False
        self.mission_status = None
        self.total_detections = 0
        
        # Action client for search area (spiral pattern)
        self.search_action_client = ActionClient(
            self, SearchArea, f'/{vehicle_name}/actions/search_area'
        )
        
        # Service client for pattern configuration
        self.set_pattern_client = self.create_client(
            SetSearchPattern, f'/{vehicle_name}/services/set_search_pattern'
        )
        
        # Status and detection subscribers
        self.status_subscription = self.create_subscription(
            MissionStatus, f'/{vehicle_name}/mission/status',
            self.mission_status_callback, 10
        )
        self.detection_subscription = self.create_subscription(
            TargetDetection, f'/{vehicle_name}/detections/target',
            self.target_detection_callback, 10
        )
        
        self.get_logger().info(f"Spiral Search ROS2 Client initialized for {vehicle_name}")
        self.get_logger().info("Pattern: Archimedes spiral with configurable spacing and direction")
        
    def mission_status_callback(self, msg: MissionStatus):
        """Handle mission status updates"""
        self.mission_status = msg
        status_names = {0: "IDLE", 1: "PLANNING", 2: "ACTIVE", 3: "PAUSED",
                       4: "COMPLETED", 5: "FAILED", 6: "CANCELLED"}
        status_name = status_names.get(msg.status, "UNKNOWN")
        
        if msg.status in [4, 5, 6]:  # COMPLETED, FAILED, CANCELLED
            self.mission_complete = True
            
        # Log progress updates
        self.get_logger().info(
            f"[EMOJI] Spiral Progress: {msg.progress_percent:.1f}% | "
            f"Status: {status_name} | "
            f"Targets: {msg.targets_detected} | "
            f"Time: {msg.elapsed_time_seconds:.1f}s"
        )
        
    def target_detection_callback(self, msg: TargetDetection):
        """Handle target detection events"""
        self.target_found = True
        self.total_detections += 1
        
        self.get_logger().info("TARGET TARGET DETECTED IN SPIRAL SEARCH!")
        self.get_logger().info(f"Location: {msg.local_position.x:.1f}, {msg.local_position.y:.1f}")
        self.get_logger().info(f"GPS: {msg.gps_location.latitude:.6f}, {msg.gps_location.longitude:.6f}")
        self.get_logger().info(f"Confidence: {msg.confidence:.2f}")
        self.get_logger().info(f"Detection #{self.total_detections}")
        
    def create_spiral_search_area(self, center_x: float = 0.0, center_y: float = 0.0,
                                max_radius: float = 100.0) -> Polygon:
        """Create circular search area for spiral pattern"""
        polygon = Polygon()
        
        # Create circular boundary (approximated as octagon for better coverage)
        num_points = 8
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + max_radius * math.cos(angle)
            y = center_y + max_radius * math.sin(angle)
            
            point = Point32()
            point.x = x
            point.y = y
            point.z = 0.0
            polygon.points.append(point)
            
        return polygon
        
    def print_spiral_parameters(self, max_radius: float, spacing: float, 
                               altitude: float, speed: float, pattern: str):
        """Print spiral mission parameters (similar to original)"""
        print("\n[EMOJI] SPIRAL SEARCH MISSION PARAMETERS")
        print(f"Search Pattern: {pattern.title()} Spiral")
        print(f"Maximum Radius: {max_radius}m")
        print(f"Spiral Spacing: {spacing}m")
        print(f"Flight Altitude: {altitude}m")
        print(f"Flight Speed: {speed}m/s")
        
        # Calculate spiral characteristics
        a = spacing / (2 * math.pi)  # Spiral constant
        max_angle = max_radius / a
        total_turns = max_angle / (2 * math.pi)
        
        # Approximate spiral length using Archimedes formula
        spiral_length = (a / 2) * (max_angle * math.sqrt(1 + max_angle**2) + 
                                  math.log(max_angle + math.sqrt(1 + max_angle**2)))
        coverage_area = math.pi * max_radius ** 2
        estimated_time = spiral_length / speed
        
        print(f"\n[EMOJI] SPIRAL CHARACTERISTICS")
        print(f"Total Spiral Turns: {total_turns:.1f}")
        print(f"Spiral Length: {spiral_length:.0f}m")
        print(f"Coverage Area: {coverage_area/10000:.1f} hectares")
        print(f"Estimated Time: {estimated_time/60:.1f} minutes")
        
    def configure_spiral_pattern(self, pattern_type: str = "spiral",
                                spacing: float = 25.0, speed: float = 5.0,
                                altitude: float = 25.0) -> bool:
        """Configure spiral search pattern"""
        
        if not self.set_pattern_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Pattern configuration service not available")
            return False
            
        request = SetSearchPattern.Request()
        request.pattern_type = pattern_type
        request.pattern_spacing = spacing
        request.search_speed = speed
        request.altitude = altitude
        request.enable_detection = True
        request.confidence_threshold = 0.6  # Lower threshold for better detection coverage
        
        try:
            future = self.set_pattern_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"PASS Spiral pattern configured successfully")
                    self.get_logger().info(f"Coverage estimate: {response.estimated_coverage_percent:.1f}%")
                    if response.warnings:
                        for warning in response.warnings:
                            self.get_logger().warn(f"WARNING {warning}")
                    return True
                else:
                    self.get_logger().error(f"FAIL Pattern configuration failed: {response.error_message}")
                    return False
            else:
                self.get_logger().error("No response from pattern configuration service")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error configuring spiral pattern: {str(e)}")
            return False
            
    def execute_spiral_search(self, center_x: float = 0.0, center_y: float = 0.0,
                            max_radius: float = 100.0, altitude: float = 25.0,
                            speed: float = 5.0, spacing: float = 25.0,
                            mission_id: str = "spiral_001") -> bool:
        """Execute spiral search mission using ROS2 action"""
        
        # Wait for action server
        if not self.search_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Search action server not available")
            return False
            
        # Create search area (circular boundary for spiral)
        search_polygon = self.create_spiral_search_area(center_x, center_y, max_radius)
        
        # Create search goal
        goal_msg = SearchArea.Goal()
        goal_msg.search_boundary = search_polygon
        goal_msg.search_pattern = "spiral"  # Mission nodes will implement spiral pattern
        goal_msg.search_altitude = altitude
        goal_msg.search_speed = speed
        goal_msg.pattern_spacing = spacing
        goal_msg.enable_detection = True
        goal_msg.detection_confidence_threshold = 0.6
        goal_msg.mission_id = mission_id
        
        self.get_logger().info("[EMOJI] Starting spiral search mission...")
        self.get_logger().info(f"Center: ({center_x:.1f}, {center_y:.1f})")
        self.get_logger().info(f"Radius: {max_radius}m | Spacing: {spacing}m")
        
        # Send goal with feedback
        future = self.search_action_client.send_goal_async(
            goal_msg, feedback_callback=self.spiral_feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error("FAIL Spiral search goal was rejected or timed out")
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FAIL Spiral search goal was rejected by action server")
            return False
            
        self.get_logger().info("PASS Spiral search goal accepted - mission starting")
        
        # Monitor mission execution
        result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        last_status_time = start_time
        
        while not self.mission_complete:
            rclpy.spin_once(self, timeout_sec=1.0)
            
            # Periodic status updates
            current_time = time.time()
            if current_time - last_status_time > 10.0:  # Every 10 seconds
                elapsed = current_time - start_time
                self.get_logger().info(f"[EMOJI] Spiral mission running... {elapsed:.0f}s elapsed")
                if self.total_detections > 0:
                    self.get_logger().info(f"TARGET {self.total_detections} targets detected so far")
                last_status_time = current_time
                
        # Get final result
        if result_future.done():
            result = result_future.result()
            if result:
                final_result = result.result
                self.get_logger().info("PASS Spiral search mission completed successfully!")
                self.get_logger().info(f"TARGET Total targets detected: {len(final_result.targets_detected)}")
                self.get_logger().info(f"⏱️ Mission time: {final_result.total_mission_time_seconds:.1f}s")
                self.get_logger().info(f"[EMOJI] Coverage achieved: {final_result.coverage_percent:.1f}%")
                self.get_logger().info(f"[EMOJI] Distance flown: {final_result.distance_flown_meters:.0f}m")
                return True
            else:
                self.get_logger().error("FAIL Spiral search mission failed")
                return False
        else:
            self.get_logger().warn("WARNING Spiral search result not available")
            return False
            
    def spiral_feedback_callback(self, feedback_msg):
        """Handle spiral search feedback"""
        feedback = feedback_msg.feedback
        
        # Calculate approximate spiral progress
        progress = feedback.progress_percent
        pattern = feedback.current_search_pattern
        waypoints = f"{feedback.waypoints_completed}/{feedback.total_waypoints}"
        
        if feedback.current_detection_events > 0:
            self.get_logger().info(
                f"[EMOJI] Spiral: {progress:.1f}% | Pattern: {pattern} | "
                f"Waypoints: {waypoints} | TARGET Detections: {feedback.current_detection_events}"
            )
        else:
            # Less frequent updates when no detections
            if int(progress) % 10 == 0:  # Every 10% progress
                self.get_logger().info(
                    f"[EMOJI] Spiral Progress: {progress:.1f}% | Waypoints: {waypoints}"
                )
                
    def execute_full_spiral_mission(self, center_x: float = 0.0, center_y: float = 0.0,
                                  max_radius: float = 100.0, altitude: float = 25.0,
                                  speed: float = 5.0, spacing: float = 25.0,
                                  pattern: str = "outward") -> bool:
        """Execute complete spiral search mission"""
        
        try:
            # Print mission parameters
            self.print_spiral_parameters(max_radius, spacing, altitude, speed, pattern)
            
            # Configure spiral pattern
            if not self.configure_spiral_pattern("spiral", spacing, speed, altitude):
                return False
                
            # Execute spiral search
            mission_id = f"spiral_{pattern}_{int(time.time())}"
            success = self.execute_spiral_search(
                center_x, center_y, max_radius, altitude, speed, spacing, mission_id
            )
            
            if success:
                self.get_logger().info("SUCCESS Spiral search mission completed successfully!")
                if self.total_detections > 0:
                    self.get_logger().info(f"TARGET Final detection count: {self.total_detections}")
                else:
                    self.get_logger().info("ℹ️ No targets detected during spiral search")
                return True
            else:
                self.get_logger().error("FAIL Spiral search mission failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Spiral mission error: {str(e)}")
            return False

def main():
    """Main function - ROS2 adaptation of spiral_search_mission.py"""
    
    parser = argparse.ArgumentParser(description="ROS2 Spiral Search Mission")
    parser.add_argument("--vehicle", type=str, default="Droan1",
                        help="Vehicle name (node name in ultra-clean architecture)")
    parser.add_argument("--center_x", type=float, default=0.0,
                        help="Spiral center X coordinate (meters)")
    parser.add_argument("--center_y", type=float, default=0.0,
                        help="Spiral center Y coordinate (meters)")
    parser.add_argument("--max_radius", type=float, default=100.0,
                        help="Maximum spiral radius in meters")
    parser.add_argument("--altitude", type=float, default=25.0,
                        help="Flight altitude in meters")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Flight speed in m/s")
    parser.add_argument("--spacing", type=float, default=25.0,
                        help="Distance between spiral arms in meters")
    parser.add_argument("--pattern", type=str, choices=["outward", "inward"],
                        default="outward",
                        help="Spiral direction: outward (center to edge) or inward (edge to center)")
    parser.add_argument("--preview", action="store_true",
                        help="Preview mission parameters without executing")
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create spiral search client
        spiral_client = SpiralSearchROS2(args.vehicle)
        
        # Preview mode
        if args.preview:
            print("[EMOJI] SPIRAL SEARCH MISSION PREVIEW")
            spiral_client.print_spiral_parameters(
                args.max_radius, args.spacing, args.altitude, args.speed, args.pattern
            )
            print("\nTo execute the mission, run without --preview flag")
            return 0
            
        print("=" * 60)
        print("[EMOJI] ROS2 SPIRAL SEARCH MISSION")
        print("Ultra-Clean Architecture: Vehicle-Name-As-Node-Name")
        print(f"Vehicle: /{args.vehicle}")
        print(f"Pattern: {args.pattern.title()} Spiral")
        print("=" * 60)
        
        # Execute spiral mission
        success = spiral_client.execute_full_spiral_mission(
            args.center_x, args.center_y, args.max_radius,
            args.altitude, args.speed, args.spacing, args.pattern
        )
        
        if success:
            print("SUCCESS Spiral search mission completed successfully!")
            return_code = 0
        else:
            print("FAIL Spiral search mission failed!")
            return_code = 1
            
        # Keep node alive briefly for final status
        time.sleep(2.0)
        
        return return_code
        
    except KeyboardInterrupt:
        print("Spiral search mission interrupted by user")
        return 1
    except Exception as e:
        print(f"Spiral search mission error: {str(e)}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    import sys
    sys.exit(main())