#!/usr/bin/env python3
"""
Unified MAVLink Connection Tester
Orchestrates all existing MAVLink testing tools for comprehensive validation
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

class UnifiedMAVLinkTester:
    def __init__(self, compose_file: str = "docker-compose-slim.yml"):
        self.compose_file = compose_file
        self.tools_dir = Path(__file__).parent
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'compose_file': compose_file,
            'tests': {},
            'summary': {},
            'recommendations': []
        }
        
        # Port mappings from docker-compose-slim.yml
        self.drone_configs = [
            {'drone': 1, 'container': 'px4-bridge-drone-1', 'tcp_hil': 4561, 'udp_control': 14581, 'qgc_port': 14541, 'container_ip': '172.20.0.11'},
            {'drone': 2, 'container': 'px4-bridge-drone-2', 'tcp_hil': 4562, 'udp_control': 14582, 'qgc_port': 14542, 'container_ip': '172.20.0.12'},
            {'drone': 3, 'container': 'px4-bridge-drone-3', 'tcp_hil': 4563, 'udp_control': 14583, 'qgc_port': 14543, 'container_ip': '172.20.0.13'},
            {'drone': 4, 'container': 'px4-bridge-drone-4', 'tcp_hil': 4564, 'udp_control': 14584, 'qgc_port': 14544, 'container_ip': '172.20.0.14'},
            {'drone': 5, 'container': 'px4-bridge-drone-5', 'tcp_hil': 4565, 'udp_control': 14585, 'qgc_port': 14545, 'container_ip': '172.20.0.15'},
            {'drone': 6, 'container': 'px4-bridge-drone-6', 'tcp_hil': 4566, 'udp_control': 14586, 'qgc_port': 14546, 'container_ip': '172.20.0.16'},
            {'drone': 7, 'container': 'px4-bridge-drone-7', 'tcp_hil': 4567, 'udp_control': 14587, 'qgc_port': 14547, 'container_ip': '172.20.0.17'},
            {'drone': 8, 'container': 'px4-bridge-drone-8', 'tcp_hil': 4568, 'udp_control': 14588, 'qgc_port': 14548, 'container_ip': '172.20.0.18'},
            {'drone': 9, 'container': 'px4-bridge-drone-9', 'tcp_hil': 4569, 'udp_control': 14589, 'qgc_port': 14549, 'container_ip': '172.20.0.19'},
        ]

    def run_tool(self, tool_script: str, args: List[str] = None, capture_output: bool = True) -> Tuple[bool, str, str]:
        """Run a testing tool and capture output"""
        tool_path = self.tools_dir / tool_script
        if not tool_path.exists():
            return False, f"Tool {tool_script} not found", ""
        
        cmd = [sys.executable, str(tool_path)]
        if args:
            cmd.extend(args)
        
        try:
            result = subprocess.run(
                cmd, 
                capture_output=capture_output, 
                text=True, 
                timeout=30,
                cwd=self.tools_dir.parent
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Tool execution timed out"
        except Exception as e:
            return False, "", str(e)

    def test_environment_detection(self) -> Dict[str, Any]:
        """Test environment detection using network_tester.py"""
        print("🔍 Phase 1: Environment Detection")
        print("=" * 50)
        
        # Try network_tester from docker_clean first
        network_tester_path = Path("../../docker_clean/config_generator/tools/network_tester.py")
        if (self.tools_dir / network_tester_path).exists():
            success, stdout, stderr = self.run_tool(str(network_tester_path), ["--json"])
            if success:
                try:
                    env_data = json.loads(stdout)
                    print(f"✅ Environment: WSL2={env_data.get('environment', {}).get('is_wsl2', False)}")
                    print(f"✅ Gateway: {env_data.get('gateway', 'Unknown')}")
                    return env_data
                except json.JSONDecodeError:
                    pass
        
        # Fallback to basic detection
        print("⚠️  Using basic environment detection")
        return {
            'environment': {'is_wsl2': False, 'is_docker': False},
            'gateway': None,
            'hosts': {}
        }

    def test_container_health(self) -> Dict[str, bool]:
        """Test container health and availability"""
        print("\n🐳 Phase 2: Container Health Check")
        print("=" * 50)
        
        container_status = {}
        for config in self.drone_configs:
            container = config['container']
            try:
                # Check if container is running
                result = subprocess.run(
                    ['docker', 'ps', '--filter', f'name={container}', '--format', '{{.Status}}'],
                    capture_output=True, text=True
                )
                
                is_running = 'Up' in result.stdout
                container_status[container] = is_running
                status_icon = '✅' if is_running else '❌'
                print(f"  {status_icon} {container}: {'Running' if is_running else 'Not running'}")
                
            except Exception as e:
                container_status[container] = False
                print(f"  ❌ {container}: Error checking status - {e}")
        
        return container_status

    def test_port_mappings(self) -> Dict[str, Dict[str, bool]]:
        """Test Docker port mappings for all drones"""
        print("\n🔗 Phase 3: Port Mapping Validation")
        print("=" * 50)
        
        port_results = {}
        for config in self.drone_configs:
            container = config['container']
            drone_num = config['drone']
            port_results[container] = {}
            
            print(f"\n🚁 Testing Drone {drone_num} ({container})")
            
            # Test TCP HIL port
            tcp_port = config['tcp_hil']
            try:
                result = subprocess.run(
                    ['docker', 'port', container, f'{tcp_port}/tcp'],
                    capture_output=True, text=True
                )
                tcp_mapped = result.returncode == 0 and result.stdout.strip()
                port_results[container]['tcp_hil'] = tcp_mapped
                print(f"  TCP HIL {tcp_port}: {'✅' if tcp_mapped else '❌'}")
                if tcp_mapped:
                    print(f"    Mapped to: {result.stdout.strip()}")
            except Exception as e:
                port_results[container]['tcp_hil'] = False
                print(f"  TCP HIL {tcp_port}: ❌ Error - {e}")
            
            # Test UDP Control port
            udp_port = config['udp_control']
            try:
                result = subprocess.run(
                    ['docker', 'port', container, f'{udp_port}/udp'],
                    capture_output=True, text=True
                )
                udp_mapped = result.returncode == 0 and result.stdout.strip()
                port_results[container]['udp_control'] = udp_mapped
                print(f"  UDP Control {udp_port}: {'✅' if udp_mapped else '❌'}")
                if udp_mapped:
                    print(f"    Mapped to: {result.stdout.strip()}")
            except Exception as e:
                port_results[container]['udp_control'] = False
                print(f"  UDP Control {udp_port}: ❌ Error - {e}")
            
            # Test QGC port (mapped to localhost)
            qgc_port = config['qgc_port']
            try:
                result = subprocess.run(
                    ['docker', 'port', container, '14550/udp'],  # Internal port is 14550
                    capture_output=True, text=True
                )
                qgc_mapped = result.returncode == 0 and f"127.0.0.1:{qgc_port}" in result.stdout
                port_results[container]['qgc_port'] = qgc_mapped
                print(f"  QGC Port {qgc_port}: {'✅' if qgc_mapped else '❌'}")
                if qgc_mapped:
                    print(f"    Mapped to: 127.0.0.1:{qgc_port}")
            except Exception as e:
                port_results[container]['qgc_port'] = False
                print(f"  QGC Port {qgc_port}: ❌ Error - {e}")
        
        return port_results

    def test_mavlink_connectivity(self) -> Dict[str, Any]:
        """Test MAVLink connectivity using existing tools"""
        print("\n📡 Phase 4: MAVLink Connectivity")
        print("=" * 50)
        
        mavlink_results = {}
        
        # Test using mavlink_diagnostic.py
        print("\n🔧 Running comprehensive MAVLink diagnostic...")
        success, stdout, stderr = self.run_tool("mavlink_diagnostic.py", capture_output=True)
        mavlink_results['diagnostic'] = {
            'success': success,
            'output': stdout,
            'errors': stderr
        }
        
        if success:
            print("✅ MAVLink diagnostic completed")
            # Parse key results from output
            if "All tests passed" in stdout:
                print("✅ All MAVLink tests passed!")
            elif "Issues found" in stdout:
                print("⚠️  Issues found in MAVLink connectivity")
        else:
            print(f"❌ MAVLink diagnostic failed: {stderr}")
        
        # Test specific drone connections
        print("\n🎯 Testing specific drone connections...")
        success, stdout, stderr = self.run_tool("test_specific_drone.py", capture_output=True)
        mavlink_results['specific_drone'] = {
            'success': success,
            'output': stdout,
            'errors': stderr
        }
        
        # Test MAVLink heartbeat
        print("\n💓 Testing MAVLink heartbeat...")
        success, stdout, stderr = self.run_tool("test_mavlink_connection.py", capture_output=True)
        mavlink_results['heartbeat'] = {
            'success': success,
            'output': stdout,
            'errors': stderr
        }
        
        return mavlink_results

    def test_qgc_connectivity(self) -> Dict[str, Any]:
        """Test QGroundControl connectivity"""
        print("\n🎮 Phase 5: QGroundControl Connectivity")
        print("=" * 50)
        
        qgc_results = {}
        
        # Test simple QGC connection
        print("🔧 Testing QGroundControl compatibility...")
        success, stdout, stderr = self.run_tool("simple_qgc_test.py", capture_output=True)
        qgc_results['simple_test'] = {
            'success': success,
            'output': stdout,
            'errors': stderr
        }
        
        if success:
            print("✅ QGroundControl test completed")
        else:
            print(f"❌ QGroundControl test failed")
        
        # Test QGC fix utility
        print("\n🔧 Running QGroundControl diagnostic & fix...")
        success, stdout, stderr = self.run_tool("qgc_fix.py", capture_output=True)
        qgc_results['qgc_fix'] = {
            'success': success,
            'output': stdout,
            'errors': stderr
        }
        
        return qgc_results

    def generate_summary_and_recommendations(self) -> None:
        """Generate test summary and recommendations"""
        print("\n📊 Test Summary & Recommendations")
        print("=" * 60)
        
        # Analyze container status
        container_results = self.results['tests'].get('container_health', {})
        running_containers = sum(1 for status in container_results.values() if status)
        total_containers = len(container_results)
        
        print(f"🐳 Containers: {running_containers}/{total_containers} running")
        
        # Analyze port mappings
        port_results = self.results['tests'].get('port_mappings', {})
        total_ports = sum(len(ports) for ports in port_results.values())
        working_ports = sum(sum(1 for working in ports.values() if working) for ports in port_results.values())
        
        print(f"🔗 Port Mappings: {working_ports}/{total_ports} working")
        
        # MAVLink status
        mavlink_results = self.results['tests'].get('mavlink_connectivity', {})
        mavlink_tests = len([test for test in mavlink_results.values() if test.get('success')])
        total_mavlink_tests = len(mavlink_results)
        
        print(f"📡 MAVLink Tests: {mavlink_tests}/{total_mavlink_tests} passed")
        
        # Generate recommendations
        recommendations = []
        
        if running_containers < total_containers:
            recommendations.append("🔧 Start missing containers: docker-compose -f docker-compose-slim.yml up -d")
        
        if working_ports < total_ports:
            recommendations.append("🔗 Check Docker port mappings in docker-compose-slim.yml")
            recommendations.append("🔧 Restart containers to refresh port mappings")
        
        if mavlink_tests < total_mavlink_tests:
            recommendations.append("📡 Check MAVLink configuration in PX4 containers")
            recommendations.append("🔧 Run: docker exec px4-bridge-drone-1 mavlink status")
        
        # Environment-specific recommendations
        env_data = self.results['tests'].get('environment_detection', {})
        if env_data.get('environment', {}).get('is_wsl2'):
            recommendations.append("🔧 WSL2 detected: Ensure Windows firewall allows UDP ports 14541-14549")
            recommendations.append("🔧 Consider using host networking mode for better connectivity")
        
        # QGC-specific recommendations
        qgc_results = self.results['tests'].get('qgc_connectivity', {})
        if not any(test.get('success') for test in qgc_results.values()):
            recommendations.append("🎮 QGroundControl: Use UDP connection type")
            recommendations.append("🎮 QGroundControl: Listen on localhost ports 14541-14549")
            recommendations.append("🎮 QGroundControl: Disable TCP, enable UDP auto-connect")
        
        self.results['summary'] = {
            'containers_running': f"{running_containers}/{total_containers}",
            'ports_working': f"{working_ports}/{total_ports}",
            'mavlink_tests_passed': f"{mavlink_tests}/{total_mavlink_tests}",
            'overall_status': 'PASS' if (running_containers > 0 and working_ports > 0 and mavlink_tests > 0) else 'FAIL'
        }
        
        self.results['recommendations'] = recommendations
        
        # Print recommendations
        if recommendations:
            print(f"\n💡 Recommendations:")
            for i, rec in enumerate(recommendations, 1):
                print(f"  {i}. {rec}")
        else:
            print(f"\n✅ All tests passed! Your MAVLink setup is working correctly.")

    def run_comprehensive_test(self, drone_range: Optional[Tuple[int, int]] = None) -> Dict[str, Any]:
        """Run comprehensive MAVLink testing"""
        print("🚁 Unified MAVLink Connection Tester")
        print("=" * 60)
        print(f"Compose file: {self.compose_file}")
        print(f"Testing {len(self.drone_configs)} drone configurations")
        
        if drone_range:
            start, end = drone_range
            self.drone_configs = [config for config in self.drone_configs if start <= config['drone'] <= end]
            print(f"Filtered to drones {start}-{end}")
        
        print()
        
        # Phase 1: Environment Detection
        self.results['tests']['environment_detection'] = self.test_environment_detection()
        
        # Phase 2: Container Health
        self.results['tests']['container_health'] = self.test_container_health()
        
        # Phase 3: Port Mappings
        self.results['tests']['port_mappings'] = self.test_port_mappings()
        
        # Phase 4: MAVLink Connectivity
        self.results['tests']['mavlink_connectivity'] = self.test_mavlink_connectivity()
        
        # Phase 5: QGroundControl
        self.results['tests']['qgc_connectivity'] = self.test_qgc_connectivity()
        
        # Generate Summary
        self.generate_summary_and_recommendations()
        
        return self.results

    def save_results(self, output_file: str) -> None:
        """Save test results to JSON file"""
        with open(output_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\n💾 Results saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description="Unified MAVLink Connection Tester")
    parser.add_argument('--compose-file', '-f', default='docker-compose-slim.yml',
                        help='Docker Compose file to test (default: docker-compose-slim.yml)')
    parser.add_argument('--drone-range', '-r', type=str,
                        help='Drone range to test (e.g., "1-3" for drones 1,2,3)')
    parser.add_argument('--output', '-o', type=str,
                        help='Output JSON file for results')
    parser.add_argument('--json-only', action='store_true',
                        help='Output only JSON results')
    parser.add_argument('--quick', action='store_true',
                        help='Quick test (skip some time-consuming tests)')
    
    args = parser.parse_args()
    
    # Parse drone range
    drone_range = None
    if args.drone_range:
        try:
            start, end = map(int, args.drone_range.split('-'))
            drone_range = (start, end)
        except ValueError:
            print(f"❌ Invalid drone range format. Use format like '1-3'")
            sys.exit(1)
    
    # Initialize tester
    tester = UnifiedMAVLinkTester(args.compose_file)
    
    # Run tests
    if not args.json_only:
        results = tester.run_comprehensive_test(drone_range)
    else:
        # For JSON-only mode, run quietly
        import io
        import contextlib
        
        f = io.StringIO()
        with contextlib.redirect_stdout(f):
            results = tester.run_comprehensive_test(drone_range)
    
    # Output results
    if args.output:
        tester.save_results(args.output)
    
    if args.json_only:
        print(json.dumps(results, indent=2))
    
    # Exit with appropriate code
    overall_status = results.get('summary', {}).get('overall_status', 'FAIL')
    sys.exit(0 if overall_status == 'PASS' else 1)


if __name__ == "__main__":
    main()