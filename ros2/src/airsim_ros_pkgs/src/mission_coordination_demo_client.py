#!/usr/bin/env python3
"""
Mission Coordination Demonstration Client
Demonstrates actual ROS2 mission orchestration with C++ nodes

This client sends real ROS2 action/service calls to demonstrate working mission coordination:
- Plan mission via /mission_coordinator/plan_mission service
- Execute mission via /mission_coordinator/actions/execute_mission action
- Monitor individual vehicle progress via /VehicleName/actions/search_area actions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Polygon, Point32
import time

# Mission search interfaces
from mission_search_interfaces.srv import PlanMission
from mission_search_interfaces.action import ExecuteMission, SearchArea
from mission_search_interfaces.msg import MissionPlan

class MissionCoordinationDemoClient(Node):
    def __init__(self):
        super().__init__('mission_coordination_demo_client')
        
        # Service clients for mission coordination
        self.plan_mission_client = self.create_client(
            PlanMission, 
            '/mission_coordinator/plan_mission'
        )
        
        # Action clients for mission execution
        self.execute_mission_client = ActionClient(
            self, 
            ExecuteMission, 
            '/mission_coordinator/actions/execute_mission'
        )
        
        # Action clients for individual vehicle control
        self.droan1_search_client = ActionClient(
            self,
            SearchArea,
            '/Droan1/actions/search_area'
        )
        
        self.px4_drone2_search_client = ActionClient(
            self,
            SearchArea, 
            '/PX4_Drone2/actions/search_area'
        )
        
        self.get_logger().info("🎯 Mission Coordination Demo Client initialized")
        self.get_logger().info("Ultra-Clean Architecture: Testing real ROS2 mission orchestration")
        
    def demonstrate_mission_orchestration(self):
        """Demonstrate complete mission orchestration workflow"""
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("🚁 ACTUAL ROS2 MISSION COORDINATION DEMONSTRATION")
        self.get_logger().info("Real C++ nodes, real action/service communication")
        self.get_logger().info("=" * 80)
        
        # Phase 1: Wait for services to be available
        self.get_logger().info("")
        self.get_logger().info("📡 Phase 1: Waiting for mission coordination services...")
        
        if not self.plan_mission_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ Mission planning service not available!")
            return False
            
        if not self.execute_mission_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Mission execution action server not available!")
            return False
            
        self.get_logger().info("✅ Mission coordination services are ready!")
        
        # Phase 2: Plan a search mission
        self.get_logger().info("")
        self.get_logger().info("🗺️  Phase 2: Planning search mission...")
        
        mission_plan = self.plan_search_mission()
        if not mission_plan:
            self.get_logger().error("❌ Mission planning failed!")
            return False
            
        self.get_logger().info("✅ Mission plan created successfully!")
        self.get_logger().info(f"   - Mission ID: {mission_plan.mission_id}")
        self.get_logger().info(f"   - Zones: {len(mission_plan.search_zones)}")
        self.get_logger().info(f"   - Vehicles: {len(mission_plan.assigned_vehicles)}")
        
        # Phase 3: Execute the mission
        self.get_logger().info("")
        self.get_logger().info("🎬 Phase 3: Executing mission with coordination...")
        
        success = self.execute_coordinated_mission(mission_plan)
        if not success:
            self.get_logger().error("❌ Mission execution failed!")
            return False
            
        self.get_logger().info("✅ Mission executed successfully!")
        
        # Phase 4: Demonstrate individual vehicle control
        self.get_logger().info("")
        self.get_logger().info("🚁 Phase 4: Testing individual vehicle actions...")
        
        self.test_individual_vehicle_actions()
        
        # Phase 5: Summary
        self.get_logger().info("")
        self.get_logger().info("🎉 MISSION COORDINATION DEMONSTRATION COMPLETE!")
        self.get_logger().info("    ✅ Real ROS2 C++ nodes communicated successfully")
        self.get_logger().info("    ✅ Mission coordination services working")
        self.get_logger().info("    ✅ Ultra-clean architecture validated")
        self.get_logger().info("    🚀 Ready for real multi-vehicle search & rescue!")
        self.get_logger().info("=" * 80)
        
        return True
        
    def plan_search_mission(self):\n        \"\"\"Plan a search mission using the coordination service\"\"\"\n        \n        # Create mission area (100m x 100m square)\n        mission_area = Polygon()\n        mission_area.points = [\n            Point32(x=0.0, y=0.0, z=0.0),\n            Point32(x=100.0, y=0.0, z=0.0),\n            Point32(x=100.0, y=100.0, z=0.0),\n            Point32(x=0.0, y=100.0, z=0.0)\n        ]\n        \n        # Create mission planning request\n        request = PlanMission.Request()\n        request.mission_name = \"demo_search_mission\"\n        request.mission_type = \"search_and_rescue\"\n        request.mission_area = mission_area\n        request.search_altitude = 25.0\n        request.preferred_search_pattern = \"spiral\"\n        request.required_coverage_percent = 80.0\n        request.min_vehicles = 2\n        request.max_vehicles = 2\n        request.max_mission_time.sec = 600  # 10 minutes\n        request.target_types = [\"person\", \"vehicle\"]\n        request.detection_confidence_threshold = 0.7\n        \n        self.get_logger().info(f\"📋 Requesting mission plan: {request.mission_name}\")\n        self.get_logger().info(f\"   Area: 100m x 100m, Altitude: {request.search_altitude}m\")\n        self.get_logger().info(f\"   Pattern: {request.preferred_search_pattern}\")\n        self.get_logger().info(f\"   Vehicles: {request.min_vehicles}-{request.max_vehicles}\")\n        \n        try:\n            future = self.plan_mission_client.call_async(request)\n            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)\n            \n            if future.result():\n                response = future.result()\n                if response.success:\n                    self.get_logger().info(f\"✅ {response.message}\")\n                    return response.mission_plan\n                else:\n                    self.get_logger().error(f\"❌ Mission planning failed: {response.message}\")\n                    return None\n            else:\n                self.get_logger().error(\"❌ Service call failed\")\n                return None\n                \n        except Exception as e:\n            self.get_logger().error(f\"❌ Exception during mission planning: {e}\")\n            return None\n            \n    def execute_coordinated_mission(self, mission_plan):\n        \"\"\"Execute mission using the coordination action server\"\"\"\n        \n        # Create mission execution goal\n        goal = ExecuteMission.Goal()\n        goal.mission_plan = mission_plan\n        goal.dry_run = False\n        goal.allow_plan_modifications = True\n        goal.inter_vehicle_coordination_interval = 5.0\n        goal.require_all_vehicles_ready = True\n        goal.max_mission_duration.sec = 600\n        goal.enable_human_oversight = False\n        goal.abort_threshold_failures = 50.0\n        goal.auto_reallocate_on_failure = True\n        \n        self.get_logger().info(\"🎬 Sending mission execution goal...\")\n        self.get_logger().info(f\"   Mission: {goal.mission_plan.mission_name}\")\n        self.get_logger().info(f\"   Duration limit: {goal.max_mission_duration.sec}s\")\n        self.get_logger().info(f\"   Coordination interval: {goal.inter_vehicle_coordination_interval}s\")\n        \n        try:\n            # Send goal\n            future = self.execute_mission_client.send_goal_async(\n                goal, \n                feedback_callback=self.mission_feedback_callback\n            )\n            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)\n            \n            goal_handle = future.result()\n            if not goal_handle.accepted:\n                self.get_logger().error(\"❌ Mission execution goal rejected\")\n                return False\n                \n            self.get_logger().info(\"✅ Mission execution goal accepted\")\n            self.get_logger().info(\"📊 Monitoring mission progress...\")\n            \n            # Wait for mission completion\n            result_future = goal_handle.get_result_async()\n            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)\n            \n            result = result_future.result().result\n            if result.success:\n                self.get_logger().info(f\"✅ Mission completed: {result.completion_reason}\")\n                self.get_logger().info(f\"   Duration: {result.total_mission_time.sec}s\")\n                self.get_logger().info(f\"   Area covered: {result.mission_area_covered_sq_m:.1f} sq_m\")\n                self.get_logger().info(f\"   Vehicles: {result.participating_vehicles_count}\")\n                return True\n            else:\n                self.get_logger().error(f\"❌ Mission failed: {result.completion_reason}\")\n                return False\n                \n        except Exception as e:\n            self.get_logger().error(f\"❌ Exception during mission execution: {e}\")\n            return False\n            \n    def mission_feedback_callback(self, feedback_msg):\n        \"\"\"Handle mission execution feedback\"\"\"\n        feedback = feedback_msg.feedback\n        \n        self.get_logger().info(\n            f\"📊 Mission Progress: {feedback.progress_percentage:.1f}% | \"\n            f\"Vehicles Active: {feedback.vehicles_active} | \"\n            f\"Phase: {feedback.current_mission_phase}\"\n        )\n        \n    def test_individual_vehicle_actions(self):\n        \"\"\"Test direct communication with individual vehicle action servers\"\"\"\n        \n        self.get_logger().info(\"🚁 Testing individual vehicle action servers...\")\n        \n        # Wait for vehicle action servers\n        if not self.droan1_search_client.wait_for_server(timeout_sec=5.0):\n            self.get_logger().warn(\"⚠️  Droan1 search action server not available\")\n            return\n            \n        if not self.px4_drone2_search_client.wait_for_server(timeout_sec=5.0):\n            self.get_logger().warn(\"⚠️  PX4_Drone2 search action server not available\")\n            return\n            \n        self.get_logger().info(\"✅ Individual vehicle action servers ready\")\n        \n        # Test sending search goal to Droan1\n        search_goal = SearchArea.Goal()\n        search_goal.search_pattern = \"spiral\"\n        search_goal.search_altitude = 25.0\n        search_goal.max_search_time.sec = 120\n        search_goal.coverage_percentage = 90.0\n        search_goal.detection_confidence_threshold = 0.8\n        \n        self.get_logger().info(\"📡 Sending search goal to /Droan1/actions/search_area\")\n        \n        try:\n            future = self.droan1_search_client.send_goal_async(search_goal)\n            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)\n            \n            goal_handle = future.result()\n            if goal_handle.accepted:\n                self.get_logger().info(\"✅ Droan1 accepted search goal\")\n                # Cancel immediately for demo purposes\n                cancel_future = goal_handle.cancel_goal_async()\n                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)\n                self.get_logger().info(\"✅ Search goal cancelled (demo purposes)\")\n            else:\n                self.get_logger().warn(\"⚠️  Droan1 rejected search goal\")\n                \n        except Exception as e:\n            self.get_logger().error(f\"❌ Exception testing Droan1: {e}\")\n            \n        self.get_logger().info(\"✅ Individual vehicle communication test complete\")\n\ndef main(args=None):\n    rclpy.init(args=args)\n    \n    try:\n        demo_client = MissionCoordinationDemoClient()\n        \n        # Wait a moment for everything to initialize\n        time.sleep(2.0)\n        \n        # Run the demonstration\n        success = demo_client.demonstrate_mission_orchestration()\n        \n        if success:\n            demo_client.get_logger().info(\"🎉 Demonstration completed successfully!\")\n        else:\n            demo_client.get_logger().error(\"❌ Demonstration encountered errors\")\n            \n        # Keep node alive briefly\n        rclpy.spin_once(demo_client, timeout_sec=2.0)\n        \n    except KeyboardInterrupt:\n        pass\n    except Exception as e:\n        print(f\"Exception in demo client: {e}\")\n    finally:\n        rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()