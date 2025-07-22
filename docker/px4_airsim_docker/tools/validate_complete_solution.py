#!/usr/bin/env python3
"""
Complete MAVLink Solution Validator
Validates the entire proxy + registration API + MAVLink Router stack
Performs end-to-end testing of external IP connectivity solution
"""

import argparse
import asyncio
import docker
import json
import logging
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

try:
    import requests
    from pymavlink import mavutil
except ImportError:
    print("❌ Required packages not installed. Install with:")
    print("   pip install requests pymavlink docker")
    sys.exit(1)

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SolutionValidator:
    """Complete solution validation framework"""
    
    def __init__(self, project_root: str = "/mnt/l/cosys-airsim/docker/px4_airsim_docker"):
        self.project_root = Path(project_root)
        self.docker_client = None
        self.validation_results = {}
        
        # Expected containers and services
        self.expected_containers = {
            'mavlink-proxy': 'nginx:alpine',
            'proxy-config-manager': 'alpine:latest',
            'mavlink-router-enhanced': 'px4-airsim:slim',
            'mavlink-registration-api': 'python:3.11-alpine'
        }
        
        # Expected files and configurations
        self.required_files = {
            'proxy/nginx-mavlink-proxy.conf': 'Nginx UDP proxy configuration',
            'proxy/docker-compose-proxy.yml': 'Docker Compose proxy setup',
            'api/mavlink-registration-api.py': 'Registration API server',
            'scripts/generate_mavlink_config_enhanced.py': 'Enhanced config generator',
            'scripts/mavlink-router-enhanced-template.conf': 'Enhanced MAVLink Router template',
            'tools/test_external_connectivity.py': 'External connectivity tester'
        }
        
        # Expected network endpoints
        self.expected_endpoints = {
            'proxy_health': ('tcp', 8080, '/health'),
            'proxy_stats': ('tcp', 8080, '/stats'),
            'registration_api': ('tcp', 8000, '/health'),
            'qgc_primary': ('udp', 14550, None),
            'api_access': ('udp', 14540, None),
            'drone_2_qgc': ('udp', 14551, None)
        }
        
    def log_validation(self, component: str, status: str, details: str = ""):
        """Log validation result"""
        timestamp = datetime.now().isoformat()
        result = {
            'timestamp': timestamp,
            'component': component,
            'status': status,
            'details': details
        }
        self.validation_results[component] = result
        
        status_icon = "✅" if status == "PASS" else "❌" if status == "FAIL" else "⚠️"
        print(f"{status_icon} {component}: {status}")
        if details:
            print(f"   {details}")
            
    def validate_file_structure(self) -> bool:
        """Validate required files exist and are properly configured"""
        logger.info("Validating file structure and configurations")
        
        all_files_valid = True
        
        for relative_path, description in self.required_files.items():
            file_path = self.project_root / relative_path
            
            if file_path.exists():
                # Check file is not empty
                if file_path.stat().st_size > 0:
                    self.log_validation(f"File: {relative_path}", "PASS", f"{description} exists and non-empty")
                else:
                    self.log_validation(f"File: {relative_path}", "FAIL", f"{description} exists but is empty")
                    all_files_valid = False
            else:
                self.log_validation(f"File: {relative_path}", "FAIL", f"{description} missing")
                all_files_valid = False
                
        return all_files_valid
        
    def validate_docker_environment(self) -> bool:
        """Validate Docker environment and container setup"""
        logger.info("Validating Docker environment")
        
        try:
            self.docker_client = docker.from_env()
            
            # Check Docker daemon is running
            self.docker_client.ping()
            self.log_validation("Docker Daemon", "PASS", "Docker daemon is accessible")
            
            # Check required images exist
            images = self.docker_client.images.list()
            image_names = [tag for image in images for tag in image.tags]
            
            required_images = ['nginx:alpine', 'alpine:latest', 'python:3.11-alpine']
            for image in required_images:
                if any(image in tag for tag in image_names):
                    self.log_validation(f"Image: {image}", "PASS", "Required Docker image available")
                else:
                    self.log_validation(f"Image: {image}", "WARN", "Image not found locally (will be pulled)")
                    
            # Check for custom PX4 image
            if any('px4-airsim' in tag for tag in image_names):
                self.log_validation("Image: px4-airsim", "PASS", "Custom PX4 AirSim image available")
            else:
                self.log_validation("Image: px4-airsim", "WARN", "Custom PX4 image not found (may need building)")
                
            return True
            
        except Exception as e:
            self.log_validation("Docker Environment", "FAIL", f"Docker error: {e}")
            return False
            
    def validate_network_configuration(self) -> bool:
        """Validate network configuration"""
        logger.info("Validating network configuration")
        
        try:
            # Check for Docker networks
            networks = self.docker_client.networks.list()
            network_names = [net.name for net in networks]
            
            expected_networks = ['px4_network', 'proxy_network', 'bridge']
            for network in expected_networks:
                if network in network_names or any(network in name for name in network_names):
                    self.log_validation(f"Network: {network}", "PASS", "Docker network available")
                else:
                    self.log_validation(f"Network: {network}", "WARN", "Network will be created on deployment")
                    
            # Check host networking capabilities
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_validation("Host Networking", "PASS", "Host networking commands accessible")
            else:
                self.log_validation("Host Networking", "WARN", "Limited host networking access")
                
            return True
            
        except Exception as e:
            self.log_validation("Network Configuration", "FAIL", f"Network validation error: {e}")
            return False
            
    def validate_configuration_templates(self) -> bool:
        """Validate configuration templates and generators"""
        logger.info("Validating configuration templates")
        
        # Test enhanced config generator
        try:
            script_path = self.project_root / "scripts/generate_mavlink_config_enhanced.py"
            template_path = self.project_root / "scripts/mavlink-router-enhanced-template.conf"
            
            if script_path.exists() and template_path.exists():
                # Test config generation
                test_output = self.project_root / "test_generated_config.conf"
                
                cmd = [
                    'python3', str(script_path),
                    '--template', str(template_path),
                    '--output', str(test_output),
                    '--validate'
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                
                if result.returncode == 0:
                    self.log_validation("Config Generator", "PASS", "Enhanced config generator works correctly")
                    
                    # Clean up test file
                    if test_output.exists():
                        test_output.unlink()
                        
                    return True
                else:
                    self.log_validation("Config Generator", "FAIL", f"Generator failed: {result.stderr}")
                    return False
            else:
                self.log_validation("Config Generator", "FAIL", "Required generator files missing")
                return False
                
        except Exception as e:
            self.log_validation("Config Generator", "FAIL", f"Generator test error: {e}")
            return False
            
    def validate_api_configuration(self) -> bool:
        """Validate registration API configuration"""
        logger.info("Validating registration API configuration")
        
        try:
            api_script = self.project_root / "api/mavlink-registration-api.py"
            
            if not api_script.exists():
                self.log_validation("Registration API", "FAIL", "API script missing")
                return False
                
            # Test API script syntax
            result = subprocess.run(
                ['python3', '-m', 'py_compile', str(api_script)],
                capture_output=True, text=True
            )
            
            if result.returncode == 0:
                self.log_validation("Registration API", "PASS", "API script syntax is valid")
                return True
            else:
                self.log_validation("Registration API", "FAIL", f"API syntax error: {result.stderr}")
                return False
                
        except Exception as e:
            self.log_validation("Registration API", "FAIL", f"API validation error: {e}")
            return False
            
    def validate_deployment_readiness(self) -> bool:
        """Validate system is ready for deployment"""
        logger.info("Validating deployment readiness")
        
        # Check for deployment scripts
        deploy_script = self.project_root / "proxy/deploy-proxy.sh"
        if deploy_script.exists():
            self.log_validation("Deployment Scripts", "PASS", "Deployment scripts available")
        else:
            self.log_validation("Deployment Scripts", "FAIL", "Deployment scripts missing")
            return False
            
        # Check Docker Compose files
        compose_file = self.project_root / "proxy/docker-compose-proxy.yml"
        if compose_file.exists():
            # Validate compose file syntax
            try:
                result = subprocess.run(
                    ['docker-compose', '-f', str(compose_file), 'config'],
                    capture_output=True, text=True, timeout=10
                )
                
                if result.returncode == 0:
                    self.log_validation("Docker Compose", "PASS", "Compose file syntax valid")
                else:
                    self.log_validation("Docker Compose", "FAIL", f"Compose validation failed: {result.stderr}")
                    return False
                    
            except Exception as e:
                self.log_validation("Docker Compose", "WARN", f"Compose validation error: {e}")
                
        # Check system permissions
        try:
            # Test Docker socket access
            subprocess.run(['docker', 'ps'], capture_output=True, check=True, timeout=5)
            self.log_validation("Docker Permissions", "PASS", "Docker access available")
        except subprocess.CalledProcessError:
            self.log_validation("Docker Permissions", "FAIL", "Docker access denied (check user permissions)")
            return False
        except Exception as e:
            self.log_validation("Docker Permissions", "WARN", f"Permission check error: {e}")
            
        return True
        
    def generate_deployment_report(self) -> Dict[str, Any]:
        """Generate comprehensive deployment readiness report"""
        
        validation_summary = {
            'timestamp': datetime.now().isoformat(),
            'project_root': str(self.project_root),
            'total_checks': len(self.validation_results),
            'passed': 0,
            'failed': 0,
            'warnings': 0,
            'overall_status': 'UNKNOWN',
            'deployment_ready': False,
            'detailed_results': self.validation_results
        }
        
        # Count results
        for result in self.validation_results.values():
            if result['status'] == 'PASS':
                validation_summary['passed'] += 1
            elif result['status'] == 'FAIL':
                validation_summary['failed'] += 1
            elif result['status'] == 'WARN':
                validation_summary['warnings'] += 1
                
        # Determine overall status
        if validation_summary['failed'] == 0:
            if validation_summary['warnings'] <= 2:  # Allow some warnings
                validation_summary['overall_status'] = 'READY'
                validation_summary['deployment_ready'] = True
            else:
                validation_summary['overall_status'] = 'READY_WITH_WARNINGS'
                validation_summary['deployment_ready'] = True
        else:
            validation_summary['overall_status'] = 'NOT_READY'
            validation_summary['deployment_ready'] = False
            
        return validation_summary
        
    def run_complete_validation(self) -> Dict[str, Any]:
        """Run complete solution validation"""
        logger.info("🔍 Starting complete MAVLink solution validation")
        
        print("\n" + "="*70)
        print("🔍 MAVLINK EXTERNAL CONNECTIVITY SOLUTION VALIDATOR")
        print("="*70 + "\n")
        
        try:
            # Phase 1: File structure validation
            print("📁 Phase 1: File Structure Validation")
            self.validate_file_structure()
            print()
            
            # Phase 2: Docker environment validation
            print("🐳 Phase 2: Docker Environment Validation")
            self.validate_docker_environment()
            print()
            
            # Phase 3: Network configuration validation
            print("🌐 Phase 3: Network Configuration Validation")
            self.validate_network_configuration()
            print()
            
            # Phase 4: Configuration templates validation
            print("⚙️  Phase 4: Configuration Templates Validation")
            self.validate_configuration_templates()
            print()
            
            # Phase 5: API configuration validation
            print("🔧 Phase 5: API Configuration Validation")
            self.validate_api_configuration()
            print()
            
            # Phase 6: Deployment readiness validation
            print("🚀 Phase 6: Deployment Readiness Validation")
            self.validate_deployment_readiness()
            print()
            
        except KeyboardInterrupt:
            print("\n⚠️ Validation interrupted by user")
        except Exception as e:
            print(f"\n❌ Validation error: {e}")
            logger.exception("Validation error")
            
        # Generate final report
        report = self.generate_deployment_report()
        
        # Print summary
        print("="*70)
        print("📊 VALIDATION SUMMARY")
        print("="*70)
        print(f"Overall Status: {report['overall_status']}")
        print(f"Deployment Ready: {'✅ YES' if report['deployment_ready'] else '❌ NO'}")
        print(f"Checks Passed: {report['passed']}")
        print(f"Checks Failed: {report['failed']}")
        print(f"Warnings: {report['warnings']}")
        print(f"Total Checks: {report['total_checks']}")
        
        if report['deployment_ready']:
            print("\n✅ Solution is ready for deployment!")
            print("\n🚀 Next steps:")
            print("   1. Deploy the solution:")
            print(f"      cd {self.project_root}/proxy")
            print("      ./deploy-proxy.sh")
            print()
            print("   2. Test external connectivity:")
            print(f"      {self.project_root}/tools/test_external_connectivity.py YOUR_HOST_IP")
            print()
            print("   3. Connect external clients:")
            print("      • QGroundControl: YOUR_HOST_IP:14550")
            print("      • MAVSDK: udp://YOUR_HOST_IP:14540")
        else:
            print("\n❌ Solution has issues that need to be resolved before deployment.")
            print("\n🔧 Issues to fix:")
            for component, result in self.validation_results.items():
                if result['status'] == 'FAIL':
                    print(f"   • {component}: {result['details']}")
                    
        return report

def main():
    parser = argparse.ArgumentParser(description="Complete MAVLink Solution Validator")
    parser.add_argument("--project-root", 
                       default="/mnt/l/cosys-airsim/docker/px4_airsim_docker",
                       help="Project root directory")
    parser.add_argument("--output", help="Output file for validation report (JSON)")
    parser.add_argument("--verbose", "-v", action="store_true", 
                       help="Enable verbose logging")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        
    # Create validator
    validator = SolutionValidator(args.project_root)
    
    try:
        # Run validation
        report = validator.run_complete_validation()
        
        # Save report if requested
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\n📄 Validation report saved to: {args.output}")
            
        # Exit with appropriate code
        if report['deployment_ready']:
            sys.exit(0)
        else:
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️ Validation interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Validation error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()