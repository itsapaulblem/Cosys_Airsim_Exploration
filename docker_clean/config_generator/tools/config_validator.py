#!/usr/bin/env python3
"""
Configuration Validator for AirSim and Docker Compose Files
Validates generated configurations for common issues and conflicts
"""

import json
import yaml
import sys
import socket
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set
from dataclasses import dataclass
from enum import Enum

class ValidationLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING" 
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"

@dataclass
class ValidationIssue:
    level: ValidationLevel
    component: str
    message: str
    suggestion: Optional[str] = None

class ConfigValidator:
    def __init__(self):
        self.issues: List[ValidationIssue] = []
        
    def validate_settings_json(self, settings_file: str) -> List[ValidationIssue]:
        """Validate AirSim settings.json file"""
        self.issues.clear()
        
        try:
            with open(settings_file, 'r') as f:
                settings = json.load(f)
        except FileNotFoundError:
            self.issues.append(ValidationIssue(
                ValidationLevel.CRITICAL,
                "Settings File",
                f"Settings file not found: {settings_file}",
                "Generate settings file using unified_generator.py"
            ))
            return self.issues
        except json.JSONDecodeError as e:
            self.issues.append(ValidationIssue(
                ValidationLevel.CRITICAL,
                "Settings File",
                f"Invalid JSON format: {e}",
                "Check JSON syntax and fix formatting errors"
            ))
            return self.issues
        
        # Validate basic structure
        self._validate_settings_structure(settings)
        
        # Validate vehicles
        if "Vehicles" in settings:
            self._validate_vehicles(settings["Vehicles"])
        
        # Validate network configuration
        self._validate_network_settings(settings)
        
        return self.issues
    
    def validate_docker_compose(self, compose_file: str) -> List[ValidationIssue]:
        """Validate Docker Compose file"""
        compose_issues = []
        
        try:
            with open(compose_file, 'r') as f:
                compose = yaml.safe_load(f)
        except FileNotFoundError:
            compose_issues.append(ValidationIssue(
                ValidationLevel.CRITICAL,
                "Docker Compose",
                f"Docker compose file not found: {compose_file}",
                "Generate docker-compose.yml using unified_generator.py"
            ))
            return compose_issues
        except yaml.YAMLError as e:
            compose_issues.append(ValidationIssue(
                ValidationLevel.CRITICAL,
                "Docker Compose",
                f"Invalid YAML format: {e}",
                "Check YAML syntax and fix formatting errors"
            ))
            return compose_issues
        
        # Validate Docker compose structure
        self._validate_compose_structure(compose, compose_issues)
        
        # Validate services
        if "services" in compose:
            self._validate_compose_services(compose["services"], compose_issues)
        
        # Validate networks
        if "networks" in compose:
            self._validate_compose_networks(compose["networks"], compose_issues)
        
        return compose_issues
    
    def validate_compatibility(self, settings_file: str, compose_file: str) -> List[ValidationIssue]:
        """Validate compatibility between settings.json and docker-compose.yml"""
        compatibility_issues = []
        
        try:
            with open(settings_file, 'r') as f:
                settings = json.load(f)
            with open(compose_file, 'r') as f:
                compose = yaml.safe_load(f)
        except (FileNotFoundError, json.JSONDecodeError, yaml.YAMLError):
            compatibility_issues.append(ValidationIssue(
                ValidationLevel.ERROR,
                "Compatibility",
                "Cannot validate compatibility - files are missing or invalid",
                "Fix individual file issues first"
            ))
            return compatibility_issues
        
        # Check port consistency
        self._validate_port_consistency(settings, compose, compatibility_issues)
        
        # Check service-vehicle mapping
        self._validate_service_vehicle_mapping(settings, compose, compatibility_issues)
        
        return compatibility_issues
    
    def check_system_prerequisites(self) -> List[ValidationIssue]:
        """Check system prerequisites for running the configuration"""
        prereq_issues = []
        
        # Check Docker availability
        self._check_docker_availability(prereq_issues)
        
        # Check port availability
        self._check_port_availability(prereq_issues)
        
        # Check file permissions
        self._check_file_permissions(prereq_issues)
        
        return prereq_issues
    
    def _validate_settings_structure(self, settings: Dict):
        """Validate basic settings.json structure"""
        required_fields = ["SettingsVersion", "SimMode", "Vehicles"]
        
        for field in required_fields:
            if field not in settings:
                self.issues.append(ValidationIssue(
                    ValidationLevel.ERROR,
                    "Settings Structure",
                    f"Missing required field: {field}",
                    f"Add {field} to settings.json"
                ))
        
        # Check version compatibility
        if settings.get("SettingsVersion", 0) < 2.0:
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Settings Version",
                f"Settings version {settings.get('SettingsVersion')} may be outdated",
                "Consider updating to version 2.0 or later"
            ))
    
    def _validate_vehicles(self, vehicles: Dict):
        """Validate vehicle configurations"""
        if not vehicles:
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Vehicles",
                "No vehicles configured",
                "Add at least one vehicle to the configuration"
            ))
            return
        
        used_ports = set()
        used_positions = set()
        
        for vehicle_name, vehicle_config in vehicles.items():
            # Validate vehicle structure
            if "VehicleType" not in vehicle_config:
                self.issues.append(ValidationIssue(
                    ValidationLevel.ERROR,
                    f"Vehicle {vehicle_name}",
                    "Missing VehicleType",
                    "Specify vehicle type (e.g., PX4Multirotor)"
                ))
            
            # Check for port conflicts
            self._check_vehicle_ports(vehicle_name, vehicle_config, used_ports)
            
            # Check for position conflicts
            self._check_vehicle_position(vehicle_name, vehicle_config, used_positions)
            
            # Validate PX4-specific settings
            if vehicle_config.get("VehicleType") == "PX4Multirotor":
                self._validate_px4_settings(vehicle_name, vehicle_config)
    
    def _check_vehicle_ports(self, vehicle_name: str, config: Dict, used_ports: Set[int]):
        """Check for port conflicts between vehicles"""
        port_fields = ["TcpPort", "ControlPortLocal", "ControlPortRemote"]
        
        for port_field in port_fields:
            if port_field in config:
                port = config[port_field]
                if port in used_ports:
                    self.issues.append(ValidationIssue(
                        ValidationLevel.ERROR,
                        f"Vehicle {vehicle_name}",
                        f"Port conflict: {port_field} {port} already in use",
                        "Use unique ports for each vehicle"
                    ))
                else:
                    used_ports.add(port)
    
    def _check_vehicle_position(self, vehicle_name: str, config: Dict, used_positions: Set[Tuple]):
        """Check for position conflicts between vehicles"""
        x = config.get("X", 0)
        y = config.get("Y", 0)
        z = config.get("Z", 0)
        
        position = (round(x, 2), round(y, 2), round(z, 2))
        
        if position in used_positions:
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                f"Vehicle {vehicle_name}",
                f"Position conflict: Multiple vehicles at {position}",
                "Ensure vehicles have unique positions to avoid collisions"
            ))
        else:
            used_positions.add(position)
    
    def _validate_px4_settings(self, vehicle_name: str, config: Dict):
        """Validate PX4-specific settings"""
        px4_required = ["TcpPort", "ControlPortLocal", "ControlPortRemote"]
        
        for field in px4_required:
            if field not in config:
                self.issues.append(ValidationIssue(
                    ValidationLevel.ERROR,
                    f"PX4 Vehicle {vehicle_name}",
                    f"Missing required PX4 field: {field}",
                    f"Add {field} to vehicle configuration"
                ))
        
        # Check LockStep setting
        if not config.get("LockStep", False):
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                f"PX4 Vehicle {vehicle_name}",
                "LockStep not enabled",
                "Enable LockStep for deterministic simulation"
            ))
        
        # Validate GPS sensor
        sensors = config.get("Sensors", {})
        if "Gps" not in sensors:
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                f"PX4 Vehicle {vehicle_name}",
                "GPS sensor not configured",
                "Add GPS sensor for proper PX4 operation"
            ))
    
    def _validate_network_settings(self, settings: Dict):
        """Validate network-related settings"""
        api_endpoint = settings.get("ApiServerEndpoint", "")
        
        if not api_endpoint:
            self.issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Network",
                "ApiServerEndpoint not specified",
                "Set ApiServerEndpoint for external connections"
            ))
        elif api_endpoint.startswith("127.0.0.1") or api_endpoint.startswith("localhost"):
            self.issues.append(ValidationIssue(
                ValidationLevel.INFO,
                "Network",
                "API server bound to localhost only",
                "Use 0.0.0.0 to allow external connections"
            ))
    
    def _validate_compose_structure(self, compose: Dict, issues: List[ValidationIssue]):
        """Validate Docker Compose structure"""
        if "services" not in compose:
            issues.append(ValidationIssue(
                ValidationLevel.ERROR,
                "Docker Compose",
                "No services defined",
                "Add service definitions to docker-compose.yml"
            ))
        
        if "networks" not in compose:
            issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Docker Compose",
                "No networks defined",
                "Define custom networks for better isolation"
            ))
    
    def _validate_compose_services(self, services: Dict, issues: List[ValidationIssue]):
        """Validate Docker Compose services"""
        used_ports = set()
        used_container_names = set()
        
        for service_name, service_config in services.items():
            # Check container names
            container_name = service_config.get("container_name", service_name)
            if container_name in used_container_names:
                issues.append(ValidationIssue(
                    ValidationLevel.ERROR,
                    f"Service {service_name}",
                    f"Duplicate container name: {container_name}",
                    "Use unique container names"
                ))
            else:
                used_container_names.add(container_name)
            
            # Check port mappings
            ports = service_config.get("ports", [])
            for port_mapping in ports:
                if isinstance(port_mapping, str):
                    host_port = port_mapping.split(":")[0]
                    try:
                        host_port_num = int(host_port)
                        if host_port_num in used_ports:
                            issues.append(ValidationIssue(
                                ValidationLevel.ERROR,
                                f"Service {service_name}",
                                f"Port conflict: {host_port_num} already mapped",
                                "Use unique host ports for each service"
                            ))
                        else:
                            used_ports.add(host_port_num)
                    except ValueError:
                        pass  # Skip invalid port formats
    
    def _validate_compose_networks(self, networks: Dict, issues: List[ValidationIssue]):
        """Validate Docker Compose networks"""
        for network_name, network_config in networks.items():
            if "ipam" in network_config:
                ipam = network_config["ipam"]
                if "config" in ipam:
                    for config_item in ipam["config"]:
                        subnet = config_item.get("subnet")
                        if subnet and not self._is_valid_subnet(subnet):
                            issues.append(ValidationIssue(
                                ValidationLevel.WARNING,
                                f"Network {network_name}",
                                f"Invalid or problematic subnet: {subnet}",
                                "Use standard private network ranges"
                            ))
    
    def _validate_port_consistency(self, settings: Dict, compose: Dict, issues: List[ValidationIssue]):
        """Validate port consistency between settings and compose"""
        settings_ports = set()
        compose_ports = set()
        
        # Extract ports from settings
        vehicles = settings.get("Vehicles", {})
        for vehicle_name, vehicle_config in vehicles.items():
            for port_field in ["TcpPort", "ControlPortLocal", "ControlPortRemote"]:
                if port_field in vehicle_config:
                    settings_ports.add(vehicle_config[port_field])
        
        # Extract ports from compose
        services = compose.get("services", {})
        for service_name, service_config in services.items():
            ports = service_config.get("ports", [])
            for port_mapping in ports:
                if isinstance(port_mapping, str):
                    try:
                        host_port = int(port_mapping.split(":")[0])
                        compose_ports.add(host_port)
                    except (ValueError, IndexError):
                        pass
        
        # Check for mismatches
        settings_only = settings_ports - compose_ports
        compose_only = compose_ports - settings_ports
        
        if settings_only:
            issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Port Consistency",
                f"Ports in settings but not in compose: {sorted(settings_only)}",
                "Ensure all settings ports are mapped in docker-compose"
            ))
        
        if compose_only:
            issues.append(ValidationIssue(
                ValidationLevel.INFO,
                "Port Consistency",
                f"Extra ports in compose: {sorted(compose_only)}",
                "These ports may be for additional services"
            ))
    
    def _validate_service_vehicle_mapping(self, settings: Dict, compose: Dict, issues: List[ValidationIssue]):
        """Validate service-vehicle mapping"""
        vehicles = set(settings.get("Vehicles", {}).keys())
        services = set(compose.get("services", {}).keys())
        
        # Convert to comparable formats (lowercase, replace underscores)
        normalized_vehicles = {v.lower().replace("_", "-") for v in vehicles}
        normalized_services = {s.lower().replace("_", "-") for s in services}
        
        vehicles_without_services = normalized_vehicles - normalized_services
        services_without_vehicles = normalized_services - normalized_vehicles
        
        if vehicles_without_services:
            issues.append(ValidationIssue(
                ValidationLevel.WARNING,
                "Service Mapping",
                f"Vehicles without Docker services: {vehicles_without_services}",
                "Add corresponding Docker services or disable docker_enabled"
            ))
        
        if services_without_vehicles:
            issues.append(ValidationIssue(
                ValidationLevel.INFO,
                "Service Mapping",
                f"Services without vehicles: {services_without_vehicles}",
                "These may be additional infrastructure services"
            ))
    
    def _check_docker_availability(self, issues: List[ValidationIssue]):
        """Check if Docker is available"""
        import subprocess
        
        try:
            result = subprocess.run(
                ["docker", "--version"], 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            if result.returncode != 0:
                issues.append(ValidationIssue(
                    ValidationLevel.CRITICAL,
                    "Docker",
                    "Docker is not available or not working",
                    "Install Docker and ensure it's running"
                ))
        except (subprocess.TimeoutExpired, FileNotFoundError):
            issues.append(ValidationIssue(
                ValidationLevel.CRITICAL,
                "Docker",
                "Docker command not found",
                "Install Docker and add it to PATH"
            ))
    
    def _check_port_availability(self, issues: List[ValidationIssue]):
        """Check if commonly used ports are available"""
        common_ports = [4561, 4562, 14550, 14541, 14581, 41451]
        
        for port in common_ports:
            if self._is_port_in_use(port):
                issues.append(ValidationIssue(
                    ValidationLevel.WARNING,
                    "Port Availability",
                    f"Port {port} is already in use",
                    f"Stop services using port {port} or use different ports"
                ))
    
    def _check_file_permissions(self, issues: List[ValidationIssue]):
        """Check file permissions for common issues"""
        current_dir = Path(".")
        
        # Check if we can write to current directory
        try:
            test_file = current_dir / ".write_test"
            test_file.write_text("test")
            test_file.unlink()
        except PermissionError:
            issues.append(ValidationIssue(
                ValidationLevel.ERROR,
                "File Permissions",
                "Cannot write to current directory",
                "Run with appropriate permissions or change directory"
            ))
    
    def _is_port_in_use(self, port: int) -> bool:
        """Check if a port is currently in use"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            try:
                s.bind(('127.0.0.1', port))
                return False
            except socket.error:
                return True
    
    def _is_valid_subnet(self, subnet: str) -> bool:
        """Check if subnet is in valid private range"""
        import ipaddress
        
        try:
            network = ipaddress.ip_network(subnet, strict=False)
            # Check if it's in private ranges
            private_ranges = [
                ipaddress.ip_network('10.0.0.0/8'),
                ipaddress.ip_network('172.16.0.0/12'),
                ipaddress.ip_network('192.168.0.0/16')
            ]
            
            return any(network.subnet_of(private_range) for private_range in private_ranges)
        except ValueError:
            return False

def print_validation_results(issues: List[ValidationIssue], title: str):
    """Print validation results in a formatted way"""
    print(f"\n{title}")
    print("=" * len(title))
    
    if not issues:
        print("✅ No issues found!")
        return
    
    # Group issues by level
    by_level = {}
    for issue in issues:
        if issue.level not in by_level:
            by_level[issue.level] = []
        by_level[issue.level].append(issue)
    
    # Print issues by level
    level_icons = {
        ValidationLevel.INFO: "ℹ️",
        ValidationLevel.WARNING: "⚠️",
        ValidationLevel.ERROR: "❌",
        ValidationLevel.CRITICAL: "🚨"
    }
    
    for level in [ValidationLevel.CRITICAL, ValidationLevel.ERROR, ValidationLevel.WARNING, ValidationLevel.INFO]:
        if level in by_level:
            print(f"\n{level_icons[level]} {level.value} ({len(by_level[level])})")
            for issue in by_level[level]:
                print(f"  [{issue.component}] {issue.message}")
                if issue.suggestion:
                    print(f"    💡 {issue.suggestion}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Validate AirSim configurations")
    parser.add_argument("--settings", default="settings.json", help="Settings file to validate")
    parser.add_argument("--compose", default="docker-compose.yml", help="Docker compose file to validate")
    parser.add_argument("--compatibility", action="store_true", help="Check compatibility between files")
    parser.add_argument("--system", action="store_true", help="Check system prerequisites")
    parser.add_argument("--all", action="store_true", help="Run all validations")
    
    args = parser.parse_args()
    
    validator = ConfigValidator()
    
    print("🔍 Configuration Validator")
    print("=" * 50)
    
    if args.all or not any([args.compatibility, args.system]):
        # Validate settings.json
        if Path(args.settings).exists():
            settings_issues = validator.validate_settings_json(args.settings)
            print_validation_results(settings_issues, f"Settings Validation ({args.settings})")
        
        # Validate docker-compose.yml
        if Path(args.compose).exists():
            compose_issues = validator.validate_docker_compose(args.compose)
            print_validation_results(compose_issues, f"Docker Compose Validation ({args.compose})")
    
    if args.compatibility or args.all:
        # Check compatibility
        if Path(args.settings).exists() and Path(args.compose).exists():
            compatibility_issues = validator.validate_compatibility(args.settings, args.compose)
            print_validation_results(compatibility_issues, "Compatibility Check")
    
    if args.system or args.all:
        # Check system prerequisites
        prereq_issues = validator.check_system_prerequisites()
        print_validation_results(prereq_issues, "System Prerequisites")
    
    print("\n" + "=" * 50)
    print("Validation complete!")

if __name__ == "__main__":
    main()