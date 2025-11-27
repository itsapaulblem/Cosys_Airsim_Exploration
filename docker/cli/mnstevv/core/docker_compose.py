"""
Docker Compose Wrapper for MNSTEVV CLI

Provides a Python interface to docker-compose commands with intelligent
environment variable management and error handling.
"""
import subprocess
import os
import sys
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import json
import yaml
import shutil

class DockerComposeWrapper:
    """Wrapper for docker-compose commands with MNSTEVV-specific logic"""

    # Class-level cache for docker compose command
    _docker_compose_cmd = None

    @classmethod
    def _get_docker_compose_command(cls) -> List[str]:
        """
        Detect and return the correct docker compose command.

        Returns:
            List of command parts (e.g., ['docker', 'compose'] or ['docker-compose'])
        """
        if cls._docker_compose_cmd is not None:
            return cls._docker_compose_cmd

        # Try modern Docker Compose plugin first (docker compose)
        try:
            result = subprocess.run(
                ['docker', 'compose', 'version'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                cls._docker_compose_cmd = ['docker', 'compose']
                return cls._docker_compose_cmd
        except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.CalledProcessError):
            pass

        # Fall back to legacy docker-compose (Python version)
        if shutil.which('docker-compose'):
            cls._docker_compose_cmd = ['docker-compose']
            return cls._docker_compose_cmd

        # If neither is found, raise an error
        raise RuntimeError(
            "Docker Compose not found. Please install either:\n"
            "  - Docker Desktop (includes 'docker compose' plugin), or\n"
            "  - Legacy docker-compose (pip install docker-compose)"
        )

    def __init__(self, compose_file: Path, working_dir: Optional[Path] = None,
                 additional_compose_files: Optional[List[Path]] = None):
        """
        Initialize the wrapper.

        Args:
            compose_file: Path to primary docker-compose.yml file
            working_dir: Working directory for commands (defaults to compose file directory)
            additional_compose_files: List of additional compose files to layer on top
        """
        self.compose_file = Path(compose_file)
        self.additional_compose_files = [Path(f) for f in (additional_compose_files or [])]
        self.working_dir = working_dir or self.compose_file.parent

        # Detect docker compose command on initialization
        self.docker_compose_cmd = self._get_docker_compose_command()

        if not self.compose_file.exists():
            raise FileNotFoundError(f"Docker compose file not found: {self.compose_file}")

        # Validate additional compose files
        for compose_file in self.additional_compose_files:
            if not compose_file.exists():
                raise FileNotFoundError(f"Additional compose file not found: {compose_file}")
    
    def _build_command(self, command: List[str], profile: Optional[str] = None,
                       env_vars: Optional[Dict[str, str]] = None,
                       profiles: Optional[List[str]] = None) -> List[str]:
        """
        Build docker-compose command with proper arguments.

        Args:
            command: Docker compose subcommand and arguments
            profile: Docker compose profile to use (legacy, use profiles instead)
            env_vars: Environment variables to set
            profiles: List of Docker compose profiles to activate

        Returns:
            Complete command list ready for subprocess
        """
        # Start with detected docker compose command (e.g., ['docker', 'compose'] or ['docker-compose'])
        cmd = self.docker_compose_cmd.copy()

        # Add compose file
        cmd.extend(['-f', str(self.compose_file)])

        # Add additional compose files
        for compose_file in self.additional_compose_files:
            cmd.extend(['-f', str(compose_file)])

        # Add project name - will use the name field from compose file
        # No need to explicitly set -p as compose file now has 'name: tevv-airstack'

        # Add profiles (support both single profile and multiple profiles)
        active_profiles = []
        if profile:
            active_profiles.append(profile)
        if profiles:
            active_profiles.extend(profiles)

        for prof in active_profiles:
            cmd.extend(['--profile', prof])

        # Environment variables will be passed to subprocess.run via env parameter
        # Docker Compose doesn't use -e flags like docker run

        # Add the actual command
        cmd.extend(command)

        return cmd
    
    def execute(self, command: List[str], profile: Optional[str] = None,
                env_vars: Optional[Dict[str, str]] = None,
                capture_output: bool = False,
                check: bool = True,
                profiles: Optional[List[str]] = None) -> subprocess.CompletedProcess:
        """
        Execute docker-compose command.

        Args:
            command: Docker compose subcommand and arguments
            profile: Docker compose profile to use (legacy, use profiles instead)
            env_vars: Environment variables to set
            capture_output: Whether to capture stdout/stderr
            check: Whether to raise exception on non-zero exit
            profiles: List of Docker compose profiles to activate

        Returns:
            CompletedProcess result

        Raises:
            subprocess.CalledProcessError: If command fails and check=True
        """
        full_cmd = self._build_command(command, profile, env_vars, profiles)
        
        # Prepare environment - merge provided env_vars with system environment
        env = os.environ.copy()
        if env_vars:
            env.update(env_vars)
            # Debug: Show environment variables being set
            if any(key.startswith(('SWARM_', 'MAX_', 'DEBUG', 'ROS_')) for key in env_vars.keys()):
                print(f"Setting environment: {', '.join(f'{k}={v}' for k, v in env_vars.items())}")
        
        try:
            result = subprocess.run(
                full_cmd,
                cwd=self.working_dir,
                env=env,
                capture_output=capture_output,
                text=True,
                check=check
            )
            return result
            
        except subprocess.CalledProcessError as e:
            # Add more context to the error
            raise subprocess.CalledProcessError(
                e.returncode, 
                full_cmd, 
                output=e.output, 
                stderr=e.stderr
            ) from e
    
    def up(self, services: List[str], profile: str = 'integrated',
           env_vars: Optional[Dict[str, str]] = None,
           detached: bool = True, build: bool = False,
           remove_orphans: bool = True,
           profiles: Optional[List[str]] = None) -> subprocess.CompletedProcess:
        """
        Start services using docker-compose up.

        Args:
            services: List of service names to start
            profile: Docker compose profile (legacy, use profiles instead)
            env_vars: Environment variables
            detached: Run in detached mode (-d)
            build: Force build images (--build)
            remove_orphans: Remove orphaned containers (--remove-orphans)
            profiles: List of Docker compose profiles to activate

        Returns:
            Command result
        """
        cmd = ['up']

        if detached:
            cmd.append('-d')
        if build:
            cmd.append('--build')
        if remove_orphans:
            cmd.append('--remove-orphans')

        cmd.extend(services)

        return self.execute(cmd, profile=profile, env_vars=env_vars, profiles=profiles)
    
    def down(self, profile: str = 'integrated', 
             remove_volumes: bool = False, 
             remove_images: str = None,
             remove_orphans: bool = True) -> subprocess.CompletedProcess:
        """
        Stop and remove services using docker-compose down.
        
        Args:
            profile: Docker compose profile
            remove_volumes: Remove named volumes (-v)
            remove_images: Remove images ('all', 'local', or None)
            remove_orphans: Remove orphaned containers (--remove-orphans)
            
        Returns:
            Command result
        """
        cmd = ['down']
        
        if remove_volumes:
            cmd.append('-v')
        if remove_images:
            cmd.extend(['--rmi', remove_images])
        if remove_orphans:
            cmd.append('--remove-orphans')
        
        return self.execute(cmd, profile=profile)
    
    def ps(self, profile: str = 'integrated', 
           services: Optional[List[str]] = None) -> subprocess.CompletedProcess:
        """
        List containers using docker-compose ps.
        
        Args:
            profile: Docker compose profile
            services: Specific services to list (None for all)
            
        Returns:
            Command result with container information
        """
        cmd = ['ps', '--format', 'json']
        
        if services:
            cmd.extend(services)
        
        return self.execute(cmd, profile=profile, capture_output=True)
    
    def logs(self, services: Optional[List[str]] = None, 
             profile: str = 'integrated',
             follow: bool = False, tail: Optional[int] = None,
             timestamps: bool = False) -> subprocess.CompletedProcess:
        """
        Get logs using docker-compose logs.
        
        Args:
            services: Services to get logs from (None for all)
            profile: Docker compose profile
            follow: Follow log output (-f)
            tail: Number of lines from end of logs
            timestamps: Include timestamps (-t)
            
        Returns:
            Command result with logs
        """
        cmd = ['logs']
        
        if follow:
            cmd.append('-f')
        if tail is not None:
            cmd.extend(['--tail', str(tail)])
        if timestamps:
            cmd.append('-t')
        
        if services:
            cmd.extend(services)
        
        return self.execute(cmd, profile=profile, capture_output=not follow)
    
    def get_running_services(self, profile: str = 'integrated') -> List[Dict]:
        """
        Get list of currently running services with status.
        
        Args:
            profile: Docker compose profile
            
        Returns:
            List of service information dictionaries
        """
        try:
            result = self.ps(profile=profile)
            if result.stdout.strip():
                # Handle both single JSON object and multiple JSON objects on separate lines
                stdout = result.stdout.strip()
                if stdout.startswith('['):
                    # Already a JSON array
                    return json.loads(stdout)
                else:
                    # Multiple JSON objects, one per line
                    services = []
                    for line in stdout.split('\n'):
                        if line.strip():
                            services.append(json.loads(line.strip()))
                    return services
            return []
        except (subprocess.CalledProcessError, json.JSONDecodeError) as e:
            print(f"Debug: Error parsing docker-compose ps output: {e}")
            print(f"Debug: stdout was: {result.stdout if 'result' in locals() else 'No result'}")
            return []
    
    def validate_compose_file(self) -> Tuple[bool, Optional[str]]:
        """
        Validate the docker-compose file syntax.
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        try:
            result = self.execute(['config', '--quiet'], capture_output=True)
            return True, None
        except subprocess.CalledProcessError as e:
            return False, e.stderr or str(e)
    
    def get_available_profiles(self) -> List[str]:
        """
        Extract available profiles from the compose file.
        
        Returns:
            List of profile names found in compose file
        """
        try:
            with open(self.compose_file, 'r') as f:
                compose_data = yaml.safe_load(f)
            
            profiles = set()
            services = compose_data.get('services', {})
            
            for service_config in services.values():
                service_profiles = service_config.get('profiles', [])
                profiles.update(service_profiles)
            
            return sorted(list(profiles))
            
        except Exception:
            # Fallback to known profiles
            return ['integrated', 'px4-only', 'ros2-only', 'development']