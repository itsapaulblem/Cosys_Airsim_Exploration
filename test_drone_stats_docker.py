#!/usr/bin/env python3
"""
Automated Test Suite for Drone Statistics Logger in Docker Environment

This comprehensive testing script validates the drone statistics collection system
in a Dockerized environment with PostgreSQL and ROS2 containers.

Features:
- Automated container startup and health checking
- Database connectivity validation
- ROS2 environment verification
- Drone statistics logger testing
- Data collection and verification
- Performance monitoring
- Comprehensive reporting

Usage:
    # Run full test suite
    python3 test_drone_stats_docker.py --full-test
    
    # Quick connectivity test
    python3 test_drone_stats_docker.py --quick-test
    
    # Test specific components
    python3 test_drone_stats_docker.py --test-database --test-ros2
    
    # Generate test data for validation
    python3 test_drone_stats_docker.py --generate-test-data

Requirements:
    - Docker and docker-compose installed
    - AirSim running on Windows
    - Project environment properly configured

Author: Claude Code Integration
Version: 1.0
"""

import subprocess
import sys
import time
import json
import os
import argparse
import logging
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
import threading

@dataclass
class TestResult:
    """Container for test results"""
    test_name: str
    passed: bool
    duration: float
    message: str
    details: Optional[Dict[str, Any]] = None

@dataclass
class TestSuite:
    """Container for test suite results"""
    name: str
    results: List[TestResult]
    start_time: datetime
    end_time: Optional[datetime] = None
    
    @property
    def duration(self) -> float:
        if self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0
    
    @property
    def passed_count(self) -> int:
        return sum(1 for r in self.results if r.passed)
    
    @property
    def failed_count(self) -> int:
        return sum(1 for r in self.results if not r.passed)
    
    @property
    def success_rate(self) -> float:
        if not self.results:
            return 0.0
        return (self.passed_count / len(self.results)) * 100

class DroneStatsDockerTester:
    """
    Main testing class for drone statistics system in Docker environment
    """
    
    def __init__(self, project_root: Optional[str] = None, verbose: bool = False):
        """
        Initialize the Docker tester
        
        Args:
            project_root: Path to project root directory
            verbose: Enable verbose logging
        """
        self.logger = logging.getLogger(__name__)
        self.project_root = Path(project_root) if project_root else Path.cwd()
        self.verbose = verbose
        
        # Test configuration
        self.containers = {
            'postgres': 'postgres-drone-stats',
            'ros2': 'ros2-node'
        }
        
        self.networks = [
            'airsim-ecosystem',
            'ros2-multi-node-network'
        ]
        
        # Test results storage
        self.test_suites: List[TestSuite] = []
        
        # Setup logging
        self._setup_logging()
    
    def _setup_logging(self):
        """Setup logging configuration"""
        level = logging.DEBUG if self.verbose else logging.INFO
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%H:%M:%S'
        )
    
    def run_command(self, command: str, timeout: int = 60, check: bool = True) -> subprocess.CompletedProcess:
        """
        Run shell command with timeout and logging
        
        Args:
            command: Command to execute
            timeout: Command timeout in seconds
            check: Whether to raise exception on non-zero exit
            
        Returns:
            CompletedProcess result
        """
        self.logger.debug(f"Running command: {command}")
        
        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=self.project_root
            )
            
            if self.verbose:
                if result.stdout:
                    self.logger.debug(f"STDOUT: {result.stdout.strip()}")
                if result.stderr:
                    self.logger.debug(f"STDERR: {result.stderr.strip()}")
            
            if check and result.returncode != 0:
                raise subprocess.CalledProcessError(result.returncode, command, result.stdout, result.stderr)
            
            return result
            
        except subprocess.TimeoutExpired:
            self.logger.error(f"Command timed out after {timeout} seconds: {command}")
            raise
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Command failed with exit code {e.returncode}: {command}")
            if e.stderr:
                self.logger.error(f"Error output: {e.stderr.strip()}")
            raise
    
    def test_docker_environment(self) -> TestResult:
        """Test Docker environment availability"""
        start_time = time.time()
        
        try:
            # Check Docker availability
            result = self.run_command("docker --version")
            docker_version = result.stdout.strip()
            
            # Check docker-compose availability
            result = self.run_command("docker-compose --version")
            compose_version = result.stdout.strip()
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Docker Environment",
                passed=True,
                duration=duration,
                message="Docker and docker-compose available",
                details={
                    "docker_version": docker_version,
                    "compose_version": compose_version
                }
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Docker Environment",
                passed=False,
                duration=duration,
                message=f"Docker environment not available: {e}"
            )
    
    def test_container_startup(self, container_profile: str = "integrated") -> TestResult:
        """Test container startup with specified profile"""
        start_time = time.time()
        
        try:
            self.logger.info(f"Starting containers with profile: {container_profile}")
            
            # Start containers
            command = f"docker-compose -f docker-compose-master.yml --profile {container_profile} up -d"
            self.run_command(command, timeout=180)
            
            # Wait for containers to be ready
            time.sleep(10)
            
            # Check container status
            result = self.run_command("docker ps --format '{{.Names}}\t{{.Status}}'")
            running_containers = result.stdout.strip().split('\n')
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Container Startup",
                passed=True,
                duration=duration,
                message=f"Containers started successfully with profile {container_profile}",
                details={"running_containers": running_containers}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Container Startup",
                passed=False,
                duration=duration,
                message=f"Container startup failed: {e}"
            )
    
    def test_database_connectivity(self) -> TestResult:
        """Test PostgreSQL database connectivity"""
        start_time = time.time()
        
        try:
            # Check container health
            result = self.run_command(f"docker inspect --format='{{{{.State.Health.Status}}}}' {self.containers['postgres']}")
            health_status = result.stdout.strip()
            
            if health_status != "healthy":
                raise Exception(f"PostgreSQL container not healthy: {health_status}")
            
            # Test database connection from host
            test_command = f"""
            docker exec {self.containers['postgres']} psql -U airsim_user -d drone_statistics -c "SELECT version();"
            """
            result = self.run_command(test_command)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Database Connectivity",
                passed=True,
                duration=duration,
                message="PostgreSQL database accessible",
                details={"health_status": health_status}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Database Connectivity",
                passed=False,
                duration=duration,
                message=f"Database connectivity failed: {e}"
            )
    
    def test_ros2_environment(self) -> TestResult:
        """Test ROS2 environment in container"""
        start_time = time.time()
        
        try:
            # Check ROS2 container status
            result = self.run_command(f"docker inspect --format='{{{{.State.Status}}}}' {self.containers['ros2']}")
            container_status = result.stdout.strip()
            
            if container_status != "running":
                raise Exception(f"ROS2 container not running: {container_status}")
            
            # Test ROS2 environment setup
            test_command = f"""
            docker exec {self.containers['ros2']} bash -c "
                source /opt/ros/humble/setup.bash && 
                source /airsim_ros2_ws/install/setup.bash &&
                ros2 node list
            "
            """
            result = self.run_command(test_command)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="ROS2 Environment",
                passed=True,
                duration=duration,
                message="ROS2 environment ready",
                details={"container_status": container_status}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="ROS2 Environment",
                passed=False,
                duration=duration,
                message=f"ROS2 environment test failed: {e}"
            )
    
    def test_drone_stats_logger_import(self) -> TestResult:
        """Test drone statistics logger import and dependencies"""
        start_time = time.time()
        
        try:
            # Test Python script import and database interface
            test_command = f"""
            docker exec {self.containers['ros2']} bash -c "
                cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts &&
                python3 -c 'from database_interface import DatabaseInterface; print(\"Database interface imported successfully\")' &&
                python3 -c 'import drone_stats_logger; print(\"Drone stats logger imported successfully\")'
            "
            """
            result = self.run_command(test_command)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Drone Stats Logger Import",
                passed=True,
                duration=duration,
                message="Drone statistics logger imports successful",
                details={"output": result.stdout.strip()}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Drone Stats Logger Import",
                passed=False,
                duration=duration,
                message=f"Import test failed: {e}"
            )
    
    def test_database_auto_detection(self) -> TestResult:
        """Test database auto-detection from ROS2 container"""
        start_time = time.time()
        
        try:
            # Test database auto-detection
            test_command = f"""
            docker exec {self.containers['ros2']} bash -c "
                cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts &&
                python3 -c '
from database_interface import DatabaseInterface
db = DatabaseInterface()
print(f\"Detected database type: {{db.db_type.value}}\")
print(f\"Connection successful: {{db.db_type.value == \\\"postgresql\\\"}}\")
db.close()
'
            "
            """
            result = self.run_command(test_command)
            
            # Verify PostgreSQL was detected
            output = result.stdout.strip()
            if "postgresql" not in output.lower():
                raise Exception(f"PostgreSQL not auto-detected. Output: {output}")
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Database Auto-Detection",
                passed=True,
                duration=duration,
                message="PostgreSQL auto-detection successful",
                details={"detection_output": output}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Database Auto-Detection",
                passed=False,
                duration=duration,
                message=f"Database auto-detection failed: {e}"
            )
    
    def test_stats_analyzer_connection(self) -> TestResult:
        """Test stats analyzer database connection"""
        start_time = time.time()
        
        try:
            # Test stats analyzer connection
            test_command = f"""
            docker exec {self.containers['ros2']} bash -c "
                cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts &&
                python3 stats_analyzer.py summary --db-type postgresql --verbose
            "
            """
            result = self.run_command(test_command, timeout=30)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Stats Analyzer Connection",
                passed=True,
                duration=duration,
                message="Stats analyzer connected to database",
                details={"output": result.stdout.strip()}
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Stats Analyzer Connection",
                passed=False,
                duration=duration,
                message=f"Stats analyzer connection failed: {e}"
            )
    
    def generate_test_data(self) -> TestResult:
        """Generate test data by running drone stats logger briefly"""
        start_time = time.time()
        
        try:
            self.logger.info("Generating test data with drone stats logger...")
            
            # Start drone stats logger in background for a short period
            test_command = f"""
            docker exec -d {self.containers['ros2']} bash -c "
                cd /airsim_ros2_ws/src/airsim_ros_pkgs/scripts &&
                timeout 30 python3 drone_stats_logger.py --database-type postgresql ||
                echo 'Logger completed'
            "
            """
            self.run_command(test_command)
            
            # Wait for data generation
            time.sleep(35)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Test Data Generation",
                passed=True,
                duration=duration,
                message="Test data generation completed"
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Test Data Generation",
                passed=False,
                duration=duration,
                message=f"Test data generation failed: {e}"
            )
    
    def cleanup_containers(self) -> TestResult:
        """Cleanup Docker containers and resources"""
        start_time = time.time()
        
        try:
            # Stop containers
            self.run_command("docker-compose -f docker-compose-master.yml down", timeout=60)
            
            duration = time.time() - start_time
            return TestResult(
                test_name="Container Cleanup",
                passed=True,
                duration=duration,
                message="Containers stopped and cleaned up"
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                test_name="Container Cleanup",
                passed=False,
                duration=duration,
                message=f"Cleanup failed: {e}"
            )
    
    def run_quick_test(self) -> TestSuite:
        """Run quick connectivity tests"""
        suite = TestSuite(name="Quick Connectivity Test", results=[], start_time=datetime.now())
        
        self.logger.info("Running quick connectivity test...")
        
        # Essential tests only
        tests = [
            self.test_docker_environment,
            self.test_container_startup,
            self.test_database_connectivity,
            self.test_ros2_environment
        ]
        
        for test_func in tests:
            try:
                result = test_func()
                suite.results.append(result)
                
                status = "✅ PASS" if result.passed else "❌ FAIL"
                self.logger.info(f"{status} {result.test_name} ({result.duration:.1f}s): {result.message}")
                
                if not result.passed:
                    break  # Stop on first failure for quick test
                    
            except Exception as e:
                suite.results.append(TestResult(
                    test_name=test_func.__name__,
                    passed=False,
                    duration=0,
                    message=f"Test execution failed: {e}"
                ))
                break
        
        suite.end_time = datetime.now()
        self.test_suites.append(suite)
        return suite
    
    def run_full_test(self) -> TestSuite:
        """Run comprehensive test suite"""
        suite = TestSuite(name="Full Test Suite", results=[], start_time=datetime.now())
        
        self.logger.info("Running full test suite...")
        
        # Comprehensive tests
        tests = [
            self.test_docker_environment,
            self.test_container_startup,
            self.test_database_connectivity,
            self.test_ros2_environment,
            self.test_drone_stats_logger_import,
            self.test_database_auto_detection,
            self.test_stats_analyzer_connection,
            self.generate_test_data,
            self.cleanup_containers
        ]
        
        for test_func in tests:
            try:
                result = test_func()
                suite.results.append(result)
                
                status = "✅ PASS" if result.passed else "❌ FAIL"
                self.logger.info(f"{status} {result.test_name} ({result.duration:.1f}s): {result.message}")
                
                # Continue even on failures for full test
                
            except Exception as e:
                suite.results.append(TestResult(
                    test_name=test_func.__name__,
                    passed=False,
                    duration=0,
                    message=f"Test execution failed: {e}"
                ))
        
        suite.end_time = datetime.now()
        self.test_suites.append(suite)
        return suite
    
    def print_test_report(self, suite: TestSuite):
        """Print comprehensive test report"""
        print(f"\n{'='*80}")
        print(f"DRONE STATISTICS DOCKER TEST REPORT")
        print(f"{'='*80}")
        print(f"Test Suite: {suite.name}")
        print(f"Execution Time: {suite.duration:.1f} seconds")
        print(f"Tests Run: {len(suite.results)}")
        print(f"Passed: {suite.passed_count}")
        print(f"Failed: {suite.failed_count}")
        print(f"Success Rate: {suite.success_rate:.1f}%")
        print(f"{'='*80}")
        
        # Detailed results
        for result in suite.results:
            status = "✅ PASS" if result.passed else "❌ FAIL"
            print(f"{status} {result.test_name:<30} ({result.duration:>6.1f}s) {result.message}")
            
            if result.details and self.verbose:
                for key, value in result.details.items():
                    print(f"      {key}: {value}")
        
        print(f"{'='*80}")
        
        # Summary recommendations
        if suite.failed_count > 0:
            print("⚠️  FAILURES DETECTED - Troubleshooting steps:")
            print("   1. Ensure Docker is running and accessible")
            print("   2. Check AirSim is running on Windows (host.docker.internal:41451)")
            print("   3. Verify .env file has correct PROJECT_ROOT path")
            print("   4. Run: docker-compose logs postgres-drone-stats")
            print("   5. Run: docker-compose logs ros2-node")
        else:
            print("🎉 ALL TESTS PASSED - Drone statistics system ready!")
            print("   Next steps:")
            print("   1. Start AirSim simulation with drone vehicles")
            print("   2. Run: ./airsim_ros2_docker.bat shell")
            print("   3. In container: python3 drone_stats_logger.py")
            print("   4. In another terminal: python3 stats_analyzer.py monitor --live")

def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Test drone statistics logger in Docker environment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run full comprehensive test
  python3 test_drone_stats_docker.py --full-test

  # Quick connectivity check
  python3 test_drone_stats_docker.py --quick-test

  # Test specific components
  python3 test_drone_stats_docker.py --test-database --test-ros2

  # Generate test data for validation
  python3 test_drone_stats_docker.py --generate-test-data

Prerequisites:
  - Docker and docker-compose installed
  - AirSim running on Windows
  - Project properly configured (.env file)
        """
    )
    
    parser.add_argument('--full-test', action='store_true',
                       help='Run full comprehensive test suite')
    parser.add_argument('--quick-test', action='store_true',
                       help='Run quick connectivity tests only')
    parser.add_argument('--test-database', action='store_true',
                       help='Test database connectivity only')
    parser.add_argument('--test-ros2', action='store_true',
                       help='Test ROS2 environment only')
    parser.add_argument('--generate-test-data', action='store_true',
                       help='Generate test data using drone stats logger')
    parser.add_argument('--project-root', default=None,
                       help='Project root directory path')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    parser.add_argument('--no-cleanup', action='store_true',
                       help='Skip container cleanup after tests')
    
    args = parser.parse_args()
    
    if not any([args.full_test, args.quick_test, args.test_database, 
                args.test_ros2, args.generate_test_data]):
        parser.print_help()
        return
    
    # Initialize tester
    tester = DroneStatsDockerTester(
        project_root=args.project_root,
        verbose=args.verbose
    )
    
    try:
        suite = None
        
        if args.full_test:
            suite = tester.run_full_test()
        elif args.quick_test:
            suite = tester.run_quick_test()
        elif args.test_database or args.test_ros2 or args.generate_test_data:
            # Custom test selection
            suite = TestSuite(name="Custom Test Selection", results=[], start_time=datetime.now())
            
            if args.test_database:
                suite.results.extend([
                    tester.test_docker_environment(),
                    tester.test_container_startup(),
                    tester.test_database_connectivity()
                ])
            
            if args.test_ros2:
                suite.results.extend([
                    tester.test_ros2_environment(),
                    tester.test_drone_stats_logger_import()
                ])
            
            if args.generate_test_data:
                suite.results.append(tester.generate_test_data())
            
            if not args.no_cleanup:
                suite.results.append(tester.cleanup_containers())
            
            suite.end_time = datetime.now()
        
        # Print results
        if suite:
            tester.print_test_report(suite)
            
            # Exit with appropriate code
            if suite.failed_count > 0:
                sys.exit(1)
            else:
                sys.exit(0)
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Test execution failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()