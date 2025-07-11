#!/usr/bin/env python3
"""
WSL2 Environment Detection and Configuration Helper
Automatically detects WSL2 environment and configures IP addresses for AirSim
"""

import os
import subprocess
import json
import socket
from pathlib import Path
from typing import Optional, Tuple, Dict

class WSL2Detector:
    def __init__(self):
        self.is_wsl2 = self._detect_wsl2()
        self.is_docker = self._detect_docker_context()
        self.windows_ip = None
        self.wsl_ip = None
        self.docker_host_ip = None
        
        if self.is_wsl2:
            self.windows_ip = self._get_windows_ip()
            self.wsl_ip = self._get_wsl_ip()
            
        if self.is_docker:
            self.docker_host_ip = self._get_docker_host_ip()
    
    def _detect_wsl2(self) -> bool:
        """Detect if running in WSL2 environment"""
        try:
            # Check for WSL2 specific indicators
            if os.path.exists('/proc/version'):
                with open('/proc/version', 'r') as f:
                    version_info = f.read().lower()
                    if 'microsoft' in version_info and 'wsl2' in version_info:
                        return True
            
            # Check for WSL environment variable
            if os.environ.get('WSL_DISTRO_NAME'):
                return True
                
            # Check uname output
            result = subprocess.run(['uname', '-r'], capture_output=True, text=True)
            if result.returncode == 0 and 'microsoft' in result.stdout.lower():
                return True
                
        except Exception:
            pass
        
        return False
    
    def _detect_docker_context(self) -> bool:
        """Detect if running inside a Docker container"""
        try:
            # Method 1: Check for .dockerenv file
            if os.path.exists('/.dockerenv'):
                return True
            
            # Method 2: Check cgroup for docker
            if os.path.exists('/proc/1/cgroup'):
                with open('/proc/1/cgroup', 'r') as f:
                    content = f.read()
                    if 'docker' in content or 'containerd' in content:
                        return True
            
            # Method 3: Check for container environment variables
            container_vars = ['CONTAINER', 'DOCKER_CONTAINER', 'KUBERNETES_SERVICE_HOST']
            for var in container_vars:
                if os.environ.get(var):
                    return True
                    
        except Exception:
            pass
        
        return False
    
    def _get_docker_host_ip(self) -> Optional[str]:
        """Get Docker host IP from container context"""
        try:
            # Method 1: Check default route (works for host networking)
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'default' in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            return parts[2]
            
            # Method 2: Try host.docker.internal (Docker Desktop)
            try:
                host_ip = socket.gethostbyname('host.docker.internal')
                return host_ip
            except socket.gaierror:
                pass
            
            # Method 3: Check docker bridge gateway
            try:
                result = subprocess.run(['ip', 'route', 'show', 'default'], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if 'via' in line:
                            parts = line.split()
                            via_index = parts.index('via')
                            if via_index + 1 < len(parts):
                                return parts[via_index + 1]
            except Exception:
                pass
                
        except Exception:
            pass
        
        return None
    
    def _get_windows_ip(self) -> Optional[str]:
        """Get Windows host IP from WSL2"""
        try:
            # Method 1: Check default route
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'default' in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            return parts[2]
            
            # Method 2: Check /etc/resolv.conf
            if os.path.exists('/etc/resolv.conf'):
                with open('/etc/resolv.conf', 'r') as f:
                    for line in f:
                        if line.startswith('nameserver'):
                            ip = line.split()[1]
                            if ip != '127.0.0.1':
                                return ip
        except Exception:
            pass
        
        return None
    
    def _get_wsl_ip(self) -> Optional[str]:
        """Get WSL2 internal IP"""
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            if result.returncode == 0:
                ips = result.stdout.strip().split()
                if ips:
                    return ips[0]
        except Exception:
            pass
        
        return None
    
    def test_connection(self, host: str, port: int, timeout: float = 2.0) -> bool:
        """Test if a TCP connection can be established"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False
    
    def get_recommended_settings(self) -> Dict[str, str]:
        """Get recommended IP settings based on environment"""
        base_settings = {
            'airsim_endpoint': '0.0.0.0:41451',
        }
        
        if self.is_docker:
            # Docker container context
            host_ip = self.docker_host_ip or 'host.docker.internal'
            return {
                **base_settings,
                'px4_sim_hostname': host_ip,
                'local_host_ip': host_ip,
                'docker_host_ip': self.docker_host_ip,
                'environment': 'docker'
            }
        
        elif self.is_wsl2:
            # WSL2 context
            return {
                **base_settings,
                'px4_sim_hostname': self.windows_ip or 'host.docker.internal',
                'local_host_ip': self.windows_ip or 'host.docker.internal',
                'windows_ip': self.windows_ip,
                'wsl_ip': self.wsl_ip,
                'environment': 'wsl2'
            }
        
        else:
            # Native Linux
            return {
                **base_settings,
                'px4_sim_hostname': 'localhost',
                'local_host_ip': '127.0.0.1',
                'environment': 'linux'
            }
    
    def update_settings_json(self, settings_path: Path, backup: bool = True) -> bool:
        """Update settings.json with correct IP addresses for WSL2"""
        if not settings_path.exists():
            print(f"Settings file not found: {settings_path}")
            return False
        
        if backup:
            backup_path = settings_path.with_suffix('.json.backup')
            import shutil
            shutil.copy2(settings_path, backup_path)
            print(f"Backup created: {backup_path}")
        
        try:
            with open(settings_path, 'r') as f:
                settings = json.load(f)
            
            recommended = self.get_recommended_settings()
            
            # Update ApiServerEndpoint
            settings['ApiServerEndpoint'] = recommended['airsim_endpoint']
            
            # Update LocalHostIp for all vehicles if WSL2
            if self.is_wsl2 and 'Vehicles' in settings:
                for vehicle_name, vehicle_config in settings['Vehicles'].items():
                    if 'LocalHostIp' in vehicle_config:
                        vehicle_config['LocalHostIp'] = recommended['local_host_ip']
            
            with open(settings_path, 'w') as f:
                json.dump(settings, f, indent=2)
            
            return True
            
        except Exception as e:
            print(f"Error updating settings.json: {e}")
            return False
    
    def update_docker_compose(self, compose_path: Path, backup: bool = True) -> bool:
        """Update docker-compose.yml with correct hostname"""
        if not compose_path.exists():
            print(f"Docker compose file not found: {compose_path}")
            return False
        
        if backup:
            backup_path = compose_path.with_suffix('.yml.backup')
            import shutil
            shutil.copy2(compose_path, backup_path)
            print(f"Backup created: {backup_path}")
        
        try:
            with open(compose_path, 'r') as f:
                content = f.read()
            
            recommended = self.get_recommended_settings()
            
            # Replace PX4_SIM_HOSTNAME values
            import re
            pattern = r'PX4_SIM_HOSTNAME:\s*\S+'
            replacement = f"PX4_SIM_HOSTNAME: {recommended['px4_sim_hostname']}"
            
            updated_content = re.sub(pattern, replacement, content)
            
            with open(compose_path, 'w') as f:
                f.write(updated_content)
            
            return True
            
        except Exception as e:
            print(f"Error updating docker-compose.yml: {e}")
            return False
    
    def print_status(self):
        """Print current environment status"""
        print("🔍 Environment Detection")
        print("=" * 40)
        print(f"WSL2 Environment: {'✓ Yes' if self.is_wsl2 else '✗ No'}")
        print(f"Docker Container: {'✓ Yes' if self.is_docker else '✗ No'}")
        
        if self.is_docker:
            print(f"Docker Host IP: {self.docker_host_ip or 'Not detected'}")
            print(f"Network Mode: {'Host' if self.docker_host_ip else 'Bridge (may need host mode)'}")
        
        if self.is_wsl2:
            print(f"Windows Host IP: {self.windows_ip or 'Not detected'}")
            print(f"WSL2 IP: {self.wsl_ip or 'Not detected'}")
            
        # Test connectivity based on environment
        test_ip = None
        if self.is_docker and self.docker_host_ip:
            test_ip = self.docker_host_ip
        elif self.is_wsl2 and self.windows_ip:
            test_ip = self.windows_ip
            
        if test_ip:
            print(f"\n🔗 Connectivity Tests (Target: {test_ip})")
            print("-" * 50)
            print(f"AirSim API (41451): {'✓' if self.test_connection(test_ip, 41451) else '✗'}")
            print(f"TCP 4561: {'✓' if self.test_connection(test_ip, 4561) else '✗'}")
            print(f"TCP 4562: {'✓' if self.test_connection(test_ip, 4562) else '✗'}")
            print(f"TCP 4563: {'✓' if self.test_connection(test_ip, 4563) else '✗'}")
        
        print(f"\n📋 Recommended Settings")
        print("-" * 25)
        recommended = self.get_recommended_settings()
        for key, value in recommended.items():
            print(f"{key}: {value}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='WSL2/Docker Environment Detection and Configuration')
    parser.add_argument('--status', action='store_true', help='Show environment status')
    parser.add_argument('--update-settings', type=str, help='Update settings.json file')
    parser.add_argument('--update-compose', type=str, help='Update docker-compose.yml file')
    parser.add_argument('--auto-update', type=str, help='Auto-update files in directory')
    parser.add_argument('--get-settings', action='store_true', help='Output recommended settings in key: value format')
    parser.add_argument('--get-environment', action='store_true', help='Output detected environment type')
    
    args = parser.parse_args()
    
    detector = WSL2Detector()
    
    if args.status:
        detector.print_status()
    elif args.get_settings:
        settings = detector.get_recommended_settings()
        for key, value in settings.items():
            print(f"{key}: {value}")
    elif args.get_environment:
        settings = detector.get_recommended_settings()
        print(settings.get('environment', 'unknown'))
    
    if args.update_settings:
        settings_path = Path(args.update_settings)
        if detector.update_settings_json(settings_path):
            print(f"✓ Updated {settings_path}")
        else:
            print(f"✗ Failed to update {settings_path}")
    
    if args.update_compose:
        compose_path = Path(args.update_compose)
        if detector.update_docker_compose(compose_path):
            print(f"✓ Updated {compose_path}")
        else:
            print(f"✗ Failed to update {compose_path}")
    
    if args.auto_update:
        base_dir = Path(args.auto_update)
        
        # Look for settings.json
        settings_candidates = [
            base_dir / "settings.json",
            Path.home() / "Documents" / "AirSim" / "settings.json"
        ]
        
        for settings_path in settings_candidates:
            if settings_path.exists():
                if detector.update_settings_json(settings_path):
                    print(f"✓ Updated {settings_path}")
                break
        
        # Look for docker-compose files
        compose_candidates = [
            base_dir / "docker-compose.yml",
            base_dir / "docker-compose.ultra-swarm.yml"
        ]
        
        for compose_path in compose_candidates:
            if compose_path.exists():
                if detector.update_docker_compose(compose_path):
                    print(f"✓ Updated {compose_path}")

if __name__ == '__main__':
    main()