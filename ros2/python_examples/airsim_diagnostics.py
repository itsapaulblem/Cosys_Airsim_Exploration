#!/usr/bin/env python3
"""
AirSim ROS2 Diagnostic Tool

This script helps diagnose common issues with AirSim ROS2 setup,
particularly for velocity command problems.

Usage:
    python3 airsim_diagnostics.py --vehicle PX4_Drone1
    python3 airsim_diagnostics.py --vehicle PX4_Drone1 --check-topics
    python3 airsim_diagnostics.py --vehicle PX4_Drone1 --full-check

Author: Cosys-AirSim ROS2 Integration
"""

import argparse
import subprocess
import sys
import time
from typing import List, Dict, Optional


class AirSimDiagnostics:
    """
    Diagnostic tool for AirSim ROS2 integration issues.
    """
    
    def __init__(self, vehicle_name: str = "PX4_Drone1"):
        self.vehicle_name = vehicle_name
        self.issues_found = []
        self.recommendations = []
        
    def run_ros_command(self, cmd: List[str], timeout: int = 5) -> tuple:
        """Run a ROS2 command and return (success, output, error)."""
        try:
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                timeout=timeout
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)
    
    def check_ros2_installation(self) -> bool:
        """Check if ROS2 is properly installed and sourced."""
        print("🔍 Checking ROS2 installation...")
        
        success, output, error = self.run_ros_command(["ros2", "--version"])
        if success:
            print(f"✅ ROS2 is installed: {output.strip()}")
            return True
        else:
            print("❌ ROS2 not found or not sourced")
            self.issues_found.append("ROS2 not properly installed or sourced")
            self.recommendations.append("Source ROS2: source /opt/ros/humble/setup.bash")
            return False
    
    def check_airsim_packages(self) -> bool:
        """Check if AirSim ROS2 packages are built and sourced."""
        print("🔍 Checking AirSim ROS2 packages...")
        
        success, output, error = self.run_ros_command(
            ["ros2", "pkg", "list", "|", "grep", "airsim"], 
            shell=True
        )
        
        # Alternative approach - check for specific packages
        packages_to_check = ["airsim_ros_pkgs", "airsim_interfaces"]
        found_packages = []
        
        for package in packages_to_check:
            success, _, _ = self.run_ros_command(["ros2", "pkg", "prefix", package])
            if success:
                found_packages.append(package)
                print(f"✅ Found package: {package}")
            else:
                print(f"❌ Missing package: {package}")
        
        if len(found_packages) == len(packages_to_check):
            return True
        else:
            self.issues_found.append("AirSim ROS2 packages not built or not sourced")
            self.recommendations.append("Build packages: cd ros2 && colcon build && source install/setup.bash")
            return False
    
    def check_launch_method(self) -> str:
        """Determine which launch method is being used."""
        print("🔍 Checking active launch method...")
        
        success, output, error = self.run_ros_command(["ros2", "node", "list"])
        if not success:
            print("❌ Cannot get node list - ROS2 not running?")
            self.issues_found.append("ROS2 nodes not accessible")
            self.recommendations.append("Start ROS2 launch file first")
            return "unknown"
        
        nodes = output.strip().split('\n')
        
        # Check for RPC dynamic launch
        if f"/{self.vehicle_name}" in nodes and "/airsim_coordination_node" in nodes:
            print("✅ RPC Dynamic Launch detected (recommended)")
            return "rpc_dynamic"
        
        # Check for legacy launch
        elif "/airsim_node" in nodes:
            print("⚠️  Legacy Launch detected")
            return "legacy"
        
        else:
            print("❌ No AirSim launch detected")
            self.issues_found.append("No AirSim ROS2 nodes running")
            self.recommendations.append("Start launch file: ros2 launch airsim_ros_pkgs rpc_dynamic_vehicles.launch.py")
            return "none"
    
    def check_vehicle_topics(self, launch_method: str) -> bool:
        """Check if vehicle topics exist."""
        print(f"🔍 Checking topics for {self.vehicle_name}...")
        
        success, output, error = self.run_ros_command(["ros2", "topic", "list"])
        if not success:
            print("❌ Cannot get topic list")
            return False
        
        topics = output.strip().split('\n')
        
        # Expected topics based on launch method
        if launch_method == "rpc_dynamic":
            expected_topics = [
                f"/{self.vehicle_name}/vel_cmd_body_frame",
                f"/{self.vehicle_name}/vel_cmd_world_frame",
                f"/{self.vehicle_name}/odom_local_ned"
            ]
        elif launch_method == "legacy":
            expected_topics = [
                f"/airsim_node/{self.vehicle_name}/vel_cmd_body_frame",
                f"/airsim_node/{self.vehicle_name}/vel_cmd_world_frame",
                f"/airsim_node/{self.vehicle_name}/odom_local_ned"
            ]
        else:
            print("❌ Cannot check topics - launch method unknown")
            return False
        
        found_topics = []
        missing_topics = []
        
        for topic in expected_topics:
            if topic in topics:
                found_topics.append(topic)
                print(f"✅ Found topic: {topic}")
            else:
                missing_topics.append(topic)
                print(f"❌ Missing topic: {topic}")
        
        if missing_topics:
            self.issues_found.append(f"Missing vehicle topics: {missing_topics}")
            if launch_method == "legacy":
                self.recommendations.append("Try enabling API control: ros2 launch airsim_ros_pkgs airsim_node.launch.py enable_api_control:=true")
            return False
        
        return True
    
    def check_vehicle_services(self, launch_method: str) -> bool:
        """Check if vehicle services exist."""
        print(f"🔍 Checking services for {self.vehicle_name}...")
        
        success, output, error = self.run_ros_command(["ros2", "service", "list"])
        if not success:
            print("❌ Cannot get service list")
            return False
        
        services = output.strip().split('\n')
        
        # Expected services based on launch method
        if launch_method == "rpc_dynamic":
            expected_services = [
                f"/{self.vehicle_name}/takeoff",
                f"/{self.vehicle_name}/land"
            ]
        elif launch_method == "legacy":
            expected_services = [
                f"/airsim_node/{self.vehicle_name}/takeoff",
                f"/airsim_node/{self.vehicle_name}/land"
            ]
        else:
            print("❌ Cannot check services - launch method unknown")
            return False
        
        found_services = []
        missing_services = []
        
        for service in expected_services:
            if service in services:
                found_services.append(service)
                print(f"✅ Found service: {service}")
            else:
                missing_services.append(service)
                print(f"❌ Missing service: {service}")
        
        if missing_services:
            self.issues_found.append(f"Missing vehicle services: {missing_services}")
            return False
        
        return True
    
    def check_airsim_connection(self) -> bool:
        """Check if AirSim simulation is running and accessible."""
        print("🔍 Checking AirSim connection...")
        
        # Try to get vehicle state via topic
        success, output, error = self.run_ros_command([
            "ros2", "topic", "echo", f"/{self.vehicle_name}/odom_local_ned", 
            "--once"
        ], timeout=3)
        
        if success and output.strip():
            print("✅ AirSim connection working - receiving vehicle data")
            return True
        else:
            print("❌ No data from AirSim - simulation not running or connection failed")
            self.issues_found.append("AirSim simulation not responding")
            self.recommendations.append("Start AirSim simulation (Blocks.exe or similar)")
            return False
    
    def test_takeoff_service(self) -> bool:
        """Test if takeoff service is accessible."""
        print("🔍 Testing takeoff service...")
        
        # Just check if service exists and is callable (don't actually takeoff)
        launch_method = self.check_launch_method()
        if launch_method == "rpc_dynamic":
            service_name = f"/{self.vehicle_name}/takeoff"
        elif launch_method == "legacy":
            service_name = f"/airsim_node/{self.vehicle_name}/takeoff"
        else:
            return False
        
        success, output, error = self.run_ros_command([
            "ros2", "service", "type", service_name
        ])
        
        if success:
            print(f"✅ Takeoff service accessible: {service_name}")
            return True
        else:
            print(f"❌ Takeoff service not accessible: {service_name}")
            return False
    
    def generate_solution(self, launch_method: str) -> None:
        """Generate specific solution based on detected setup."""
        print("\n" + "="*60)
        print("🚀 SOLUTION FOR YOUR SETUP")
        print("="*60)
        
        if launch_method == "rpc_dynamic":
            print("Launch Method: RPC Dynamic (Recommended)")
            print("\n📋 Complete Working Sequence:")
            print(f"""
# 1. Takeoff first (REQUIRED for PX4)
ros2 service call /{self.vehicle_name}/takeoff airsim_interfaces/srv/Takeoff '{{}}'

# 2. Wait for takeoff (3-5 seconds)
sleep 5

# 3. Send continuous velocity commands
ros2 topic pub --rate 10 /{self.vehicle_name}/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{{
  twist: {{
    linear: {{x: 1.0, y: 0.0, z: 0.0}},
    angular: {{x: 0.0, y: 0.0, z: 0.0}}
  }},
  drivetrain: 0,
  yaw_mode: {{
    is_rate: true,
    yaw_or_rate: 0.0
  }}
}}'

# 4. Stop with Ctrl+C, then hover
python3 velocity_command_helper.py --vehicle {self.vehicle_name} --hover --continuous

# 5. Land when done
ros2 service call /{self.vehicle_name}/land airsim_interfaces/srv/Land '{{}}'
""")
        
        elif launch_method == "legacy":
            print("Launch Method: Legacy")
            print("\n📋 Complete Working Sequence:")
            print(f"""
# 1. Takeoff first
ros2 service call /airsim_node/{self.vehicle_name}/takeoff airsim_interfaces/srv/Takeoff '{{}}'

# 2. Wait for takeoff  
sleep 5

# 3. Send continuous velocity commands
ros2 topic pub --rate 10 /airsim_node/{self.vehicle_name}/vel_cmd_body_frame airsim_interfaces/msg/VelCmd '{{
  twist: {{
    linear: {{x: 1.0, y: 0.0, z: 0.0}},
    angular: {{x: 0.0, y: 0.0, z: 0.0}}
  }},
  drivetrain: 0,
  yaw_mode: {{
    is_rate: true,
    yaw_or_rate: 0.0
  }}
}}'

# 4. Land when done
ros2 service call /airsim_node/{self.vehicle_name}/land airsim_interfaces/srv/Land '{{}}'
""")
        
        print("\n🔑 Key Points:")
        print("• MUST takeoff before velocity commands work")
        print("• MUST use --rate 10 for continuous commands")
        print("• Single commands only last 0.05 seconds")
        print("• Press Ctrl+C to stop movement")
    
    def run_full_diagnostics(self) -> None:
        """Run complete diagnostic sequence."""
        print("🔧 AirSim ROS2 Diagnostics")
        print("="*60)
        
        # Basic checks
        if not self.check_ros2_installation():
            return
        
        if not self.check_airsim_packages():
            return
        
        # Launch method detection
        launch_method = self.check_launch_method()
        if launch_method == "none":
            return
        
        # Vehicle-specific checks
        self.check_vehicle_topics(launch_method)
        self.check_vehicle_services(launch_method)
        self.check_airsim_connection()
        self.test_takeoff_service()
        
        # Summary
        print("\n" + "="*60)
        print("📊 DIAGNOSTIC SUMMARY")
        print("="*60)
        
        if self.issues_found:
            print("❌ Issues Found:")
            for i, issue in enumerate(self.issues_found, 1):
                print(f"   {i}. {issue}")
            
            print("\n💡 Recommendations:")
            for i, rec in enumerate(self.recommendations, 1):
                print(f"   {i}. {rec}")
        else:
            print("✅ All checks passed!")
        
        # Generate solution
        self.generate_solution(launch_method)
        
        print(f"\n📚 For detailed troubleshooting, see:")
        print("   • PX4_VELOCITY_TROUBLESHOOTING.md")
        print("   • VELOCITY_COMMAND_EXAMPLES.md")


def main():
    parser = argparse.ArgumentParser(
        description='Diagnose AirSim ROS2 velocity command issues'
    )
    
    parser.add_argument(
        '--vehicle', '-v',
        default='PX4_Drone1',
        help='Vehicle name to check (default: PX4_Drone1)'
    )
    
    parser.add_argument(
        '--check-topics',
        action='store_true',
        help='Only check topics and services'
    )
    
    parser.add_argument(
        '--full-check',
        action='store_true',
        help='Run complete diagnostic sequence (default)'
    )
    
    args = parser.parse_args()
    
    diagnostics = AirSimDiagnostics(args.vehicle)
    
    if args.check_topics:
        launch_method = diagnostics.check_launch_method()
        diagnostics.check_vehicle_topics(launch_method)
        diagnostics.check_vehicle_services(launch_method)
    else:
        diagnostics.run_full_diagnostics()


if __name__ == '__main__':
    main()