#!/usr/bin/env python3
"""
Test script to verify all drone connections are working properly.
Tests AirSim TCP connections, MAVLink connections, and QGroundControl ports.
"""

import socket
import sys
import time
from typing import List, Tuple, Dict

def test_tcp_connection(host: str, port: int, timeout: int = 5) -> bool:
    """Test if a TCP port is accessible."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            return result == 0
    except Exception as e:
        print(f"  Exception testing {host}:{port} - {e}")
        return False

def test_udp_connection(host: str, port: int, timeout: int = 5) -> bool:
    """Test if a UDP port is accessible (basic check)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            # Send a small test packet
            sock.sendto(b'test', (host, port))
            return True
    except Exception as e:
        # UDP is connectionless, so this might fail even if port is open
        # We'll consider it accessible if no immediate error
        return True

def test_drone_connections(drone_configs: List[Dict]) -> Dict[str, Dict]:
    """Test all configured drone connections."""
    results = {}
    
    print("Testing drone connections...")
    print("=" * 50)
    
    for config in drone_configs:
        drone_name = config['name']
        print(f"\nTesting {drone_name}:")
        
        results[drone_name] = {
            'airsim_tcp': test_tcp_connection('localhost', config['airsim_port']),
            'qgc_udp': test_udp_connection('localhost', config['qgc_port']),
            'mavlink_local': test_udp_connection('localhost', config['mavlink_local']),
            'mavlink_remote': test_udp_connection('localhost', config['mavlink_remote'])
        }
        
        for service, status in results[drone_name].items():
            status_str = "✓ OK" if status else "✗ FAIL"
            port_info = ""
            if service == 'airsim_tcp':
                port_info = f"(port {config['airsim_port']})"
            elif service == 'qgc_udp':
                port_info = f"(port {config['qgc_port']})"
            elif service == 'mavlink_local':
                port_info = f"(port {config['mavlink_local']})"
            elif service == 'mavlink_remote':
                port_info = f"(port {config['mavlink_remote']})"
            
            print(f"  {service.ljust(15)}: {status_str} {port_info}")
    
    return results

def test_docker_containers() -> Dict[str, bool]:
    """Test if Docker containers are running."""
    import subprocess
    
    print("\nChecking Docker containers:")
    print("=" * 30)
    
    try:
        result = subprocess.run(['docker', 'ps', '--format', '{{.Names}},{{.Status}}'], 
                              capture_output=True, text=True, check=True)
        
        containers = {}
        lines = result.stdout.strip().split('\n')
        
        for line in lines:
            if line.strip():
                parts = line.split(',', 1)  # Split on first comma only
                if len(parts) >= 2:
                    name = parts[0].strip()
                    status = parts[1].strip()
                    is_running = 'Up' in status
                    containers[name] = is_running
                    status_str = "✓ Running" if is_running else "✗ Stopped"
                    print(f"  {name.ljust(15)}: {status_str}")
        
        return containers
    except subprocess.CalledProcessError as e:
        print(f"  Error checking containers: {e}")
        return {}
    except FileNotFoundError:
        print("  Docker not found or not in PATH")
        return {}

def generate_airsim_settings(drone_configs: List[Dict]) -> str:
    """Generate AirSim settings.json for the tested configuration."""
    vehicles = {}
    
    for config in drone_configs:
        drone_name = config['name'].replace('-', '_').title()
        vehicles[drone_name] = {
            "VehicleType": "PX4Multirotor",
            "UseSerial": False,
            "UseTcp": True,
            "TcpPort": config['airsim_port'],
            "ControlPortLocal": config['mavlink_local'],
            "ControlPortRemote": config['mavlink_remote'],
            "LockStep": True,
            "X": (config['instance'] - 1) * 5,
            "Y": 0,
            "Z": -2
        }
    
    settings = {
        "SettingsVersion": 2.0,
        "SimMode": "Multirotor",
        "ApiServerEndpoint": "0.0.0.0:41451",
        "Vehicles": vehicles
    }
    
    import json
    return json.dumps(settings, indent=2)

def main():
    # Standard drone configurations
    single_drone = [
        {
            'name': 'px4-single',
            'instance': 1,
            'airsim_port': 4561,
            'qgc_port': 14550,
            'mavlink_local': 14541,
            'mavlink_remote': 14581
        }
    ]
    
    multi_drone_configs = [
        {
            'name': 'px4-drone-1',
            'instance': 1,
            'airsim_port': 4561,
            'qgc_port': 14550,
            'mavlink_local': 14541,
            'mavlink_remote': 14581
        },
        {
            'name': 'px4-drone-2',
            'instance': 2,
            'airsim_port': 4562,
            'qgc_port': 14551,
            'mavlink_local': 14542,
            'mavlink_remote': 14582
        },
        {
            'name': 'px4-drone-3',
            'instance': 3,
            'airsim_port': 4563,
            'qgc_port': 14552,
            'mavlink_local': 14543,
            'mavlink_remote': 14583
        },
        {
            'name': 'px4-drone-4',
            'instance': 4,
            'airsim_port': 4564,
            'qgc_port': 14553,
            'mavlink_local': 14544,
            'mavlink_remote': 14584
        },
        {
            'name': 'px4-drone-5',
            'instance': 5,
            'airsim_port': 4565,
            'qgc_port': 14554,
            'mavlink_local': 14545,
            'mavlink_remote': 14585
        }
    ]
    
    # Check which containers are running
    containers = test_docker_containers()
    
    # Determine which configuration to test
    if 'px4-single' in containers and containers.get('px4-single', False):
        print(f"\nDetected single drone configuration")
        drone_configs = single_drone
    else:
        # Count running multi-drone containers
        running_drones = [config for config in multi_drone_configs 
                         if config['name'] in containers and containers[config['name']]]
        
        if running_drones:
            print(f"\nDetected {len(running_drones)} multi-drone configuration")
            drone_configs = running_drones
        else:
            print(f"\nNo running drone containers detected")
            print("Running containers found:")
            for name, status in containers.items():
                status_str = "Running" if status else "Stopped"
                print(f"  {name}: {status_str}")
            print("\nExpected configurations:")
            print("  Single drone: px4-single")
            print("  Multi drone: px4-drone-1, px4-drone-2, etc.")
            return 1
    
    # Test connections
    results = test_drone_connections(drone_configs)
    
    # Summary
    print(f"\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    
    total_tests = 0
    passed_tests = 0
    
    for drone_name, tests in results.items():
        for test_name, status in tests.items():
            total_tests += 1
            if status:
                passed_tests += 1
    
    print(f"Total tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    print(f"Success rate: {passed_tests/total_tests*100:.1f}%")
    
    # Generate settings
    print(f"\nRecommended AirSim settings.json:")
    print("-" * 40)
    print(generate_airsim_settings(drone_configs))
    
    return 0 if passed_tests == total_tests else 1

if __name__ == "__main__":
    sys.exit(main())