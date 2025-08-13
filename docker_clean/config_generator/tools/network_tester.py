#!/usr/bin/env python3
"""
Network Connectivity Tester for AirSim and PX4 MAVLink
Tests connectivity between Docker containers, WSL2, and AirSim instances
"""

import os
import socket
import subprocess
import sys
import time
from typing import List, Tuple, Dict, Optional


class NetworkTester:
    def __init__(self):
        self.test_results = {}
        
    def test_port(self, host: str, port: int, timeout: float = 2.0) -> bool:
        """Test if a TCP port is accessible"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False
    
    def test_udp_port(self, host: str, port: int, timeout: float = 2.0) -> bool:
        """Test if a UDP port is accessible (basic check)"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(timeout)
            # Try to send a small packet
            sock.sendto(b'test', (host, port))
            sock.close()
            return True
        except Exception:
            return False
    
    def ping_host(self, host: str, count: int = 3) -> bool:
        """Test if host is reachable via ping"""
        try:
            cmd = ['ping', '-c', str(count), host]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            return result.returncode == 0
        except Exception:
            return False
    
    def test_airsim_connectivity(self, host: str) -> Dict[str, bool]:
        """Test AirSim-specific connectivity"""
        results = {}
        
        print(f"🔄 Testing AirSim connectivity to {host}...")
        
        # Test AirSim API port
        results['api_port'] = self.test_port(host, 41451)
        print(f"  API Port (41451): {'✅' if results['api_port'] else '❌'}")
        
        # Test PX4 TCP ports (common range)
        px4_ports = [4560, 4561, 4562, 4563, 4564]
        results['px4_tcp_ports'] = {}
        
        for port in px4_ports:
            results['px4_tcp_ports'][port] = self.test_port(host, port)
            status = '✅' if results['px4_tcp_ports'][port] else '❌'
            print(f"  PX4 TCP {port}: {status}")
        
        # Test MAVLink UDP ports
        mavlink_ports = [14550, 14551, 14552, 14553]
        results['mavlink_udp_ports'] = {}
        
        for port in mavlink_ports:
            results['mavlink_udp_ports'][port] = self.test_udp_port(host, port)
            status = '✅' if results['mavlink_udp_ports'][port] else '❌'
            print(f"  MAVLink UDP {port}: {status}")
        
        return results
    
    def detect_environment(self) -> Dict[str, bool]:
        """Detect current environment (WSL2, Docker, Linux)"""
        env = {
            'is_wsl2': False,
            'is_docker': False,
            'is_linux': True
        }
        
        # Check for WSL2
        try:
            with open('/proc/version', 'r') as f:
                version_info = f.read().lower()
                if 'microsoft' in version_info and 'wsl2' in version_info:
                    env['is_wsl2'] = True
        except:
            pass
        
        # Check for Docker
        if os.path.exists('/.dockerenv'):
            env['is_docker'] = True
            
        return env
    
    def get_default_gateway(self) -> Optional[str]:
        """Get default gateway IP"""
        try:
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'default' in line:
                    parts = line.split()
                    if len(parts) >= 3:
                        return parts[2]
        except:
            pass
        return None
    
    def comprehensive_test(self, target_hosts: List[str] = None) -> Dict:
        """Run comprehensive network connectivity tests"""
        if target_hosts is None:
            # Auto-detect target hosts
            gateway = self.get_default_gateway()
            target_hosts = ['localhost', '127.0.0.1']
            if gateway:
                target_hosts.append(gateway)
        
        print("🚀 Comprehensive Network Connectivity Test")
        print("=" * 50)
        
        # Environment detection
        env = self.detect_environment()
        print(f"Environment: WSL2={env['is_wsl2']}, Docker={env['is_docker']}")
        
        # Gateway info
        gateway = self.get_default_gateway()
        print(f"Default Gateway: {gateway or 'Not detected'}")
        print()
        
        results = {'environment': env, 'gateway': gateway, 'hosts': {}}
        
        for host in target_hosts:
            print(f"🎯 Testing host: {host}")
            print("-" * 30)
            
            host_results = {}
            
            # Basic connectivity
            host_results['ping'] = self.ping_host(host)
            print(f"  Ping: {'✅' if host_results['ping'] else '❌'}")
            
            # AirSim specific tests
            host_results['airsim'] = self.test_airsim_connectivity(host)
            
            results['hosts'][host] = host_results
            print()
        
        return results
    
    def print_summary(self, results: Dict):
        """Print test summary"""
        print("📊 Test Summary")
        print("=" * 30)
        
        for host, host_results in results['hosts'].items():
            print(f"🖥️ Host: {host}")
            
            # Overall connectivity
            ping_ok = host_results.get('ping', False)
            api_ok = host_results.get('airsim', {}).get('api_port', False)
            
            if ping_ok and api_ok:
                print(f"  Status: ✅ Fully operational")
            elif ping_ok:
                print(f"  Status: ⚠️ Reachable but AirSim not responding")
            else:
                print(f"  Status: ❌ Not reachable")
            
            # PX4 TCP ports summary
            tcp_ports = host_results.get('airsim', {}).get('px4_tcp_ports', {})
            available_ports = [port for port, status in tcp_ports.items() if status]
            print(f"  Available PX4 Ports: {available_ports}")
            
            print()
    
    def generate_recommendations(self, results: Dict) -> List[str]:
        """Generate recommendations based on test results"""
        recommendations = []
        
        env = results.get('environment', {})
        
        # Check if any host has working AirSim
        has_working_airsim = False
        for host_results in results['hosts'].values():
            if host_results.get('airsim', {}).get('api_port', False):
                has_working_airsim = True
                break
        
        if not has_working_airsim:
            recommendations.append("🔧 Start AirSim and ensure it's listening on port 41451")
            recommendations.append("🔧 Check firewall settings (Windows Defender, iptables)")
            
        if env.get('is_docker'):
            recommendations.append("🐳 Use 'network_mode: host' in docker-compose.yml")
            recommendations.append("🐳 Set PX4_SIM_HOSTNAME to host IP address")
            
        if env.get('is_wsl2'):
            recommendations.append("🔧 Export PX4_SIM_HOSTNAME=<windows_ip> in WSL2")
            recommendations.append("🔧 Configure Windows Firewall to allow AirSim connections")
            
        return recommendations


def main():
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='Network connectivity tester for AirSim/PX4')
    parser.add_argument('--hosts', nargs='+', help='Specific hosts to test')
    parser.add_argument('--quick', action='store_true', help='Quick test (skip some checks)')
    parser.add_argument('--port', type=int, help='Test specific port')
    parser.add_argument('--json', action='store_true', help='Output results as JSON')
    
    args = parser.parse_args()
    
    tester = NetworkTester()
    
    if args.port and args.hosts:
        # Single port test
        host = args.hosts[0]
        result = tester.test_port(host, args.port)
        print(f"Port {args.port} on {host}: {'✅ Open' if result else '❌ Closed'}")
        sys.exit(0 if result else 1)
    
    # Comprehensive test
    results = tester.comprehensive_test(args.hosts)
    
    if args.json:
        import json
        print(json.dumps(results, indent=2))
    else:
        tester.print_summary(results)
        
        recommendations = tester.generate_recommendations(results)
        if recommendations:
            print("💡 Recommendations:")
            for rec in recommendations:
                print(f"  {rec}")


if __name__ == '__main__':
    main()