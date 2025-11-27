"""
Service Selection Logic for MNSTEVV CLI

Maps user intent (drone count + profile) to specific Docker Compose services
based on the docker-compose-master.yml configuration.
"""
from typing import List, Dict, Any
from enum import Enum

class Profile(Enum):
    """Supported deployment profiles from docker-compose-master.yml"""
    INTEGRATED = "integrated"
    PX4_ONLY = "px4-only"
    ROS2_ONLY = "ros2-only"
    DEVELOPMENT = "development"
    STATS_ONLY = "stats-only"

class ServiceSelector:
    """Intelligent service selection based on user parameters"""
    
    MAX_DRONES = 6
    MIN_DRONES = 1
    
    # Core infrastructure services
    INFRASTRUCTURE_SERVICES = [
        # IP detection now handled by external PowerShell tools
    ]
    
    # PostgreSQL services for database functionality (backup disabled for development)
    POSTGRES_SERVICES = ['postgres-stats', 'pgadmin-stats']
    
    # Production PostgreSQL services (includes backup)  
    POSTGRES_SERVICES_PRODUCTION = ['postgres-stats', 'pgadmin-stats', 'postgres-backup']
    
    # Profile-specific additional services
    PROFILE_SERVICES = {
        Profile.INTEGRATED: ['ecosystem-monitor'],
        Profile.DEVELOPMENT: ['dev-helper'],
        Profile.PX4_ONLY: [],
        Profile.ROS2_ONLY: [],
        Profile.STATS_ONLY: []
    }
    
    @classmethod
    def get_services_for_config(cls, num_drones: int, profile: Profile, include_postgres: bool = False) -> List[str]:
        """
        Map user configuration to Docker Compose services.
        
        Args:
            num_drones: Number of drones to deploy (1-6)
            profile: Deployment profile
            include_postgres: Whether to include PostgreSQL services
            
        Returns:
            List of service names to pass to docker-compose
            
        Raises:
            ValueError: If drone count is invalid
        """
        # For stats-only profile, ignore drone count validation
        if profile != Profile.STATS_ONLY and not (cls.MIN_DRONES <= num_drones <= cls.MAX_DRONES):
            raise ValueError(f"Drone count must be between {cls.MIN_DRONES} and {cls.MAX_DRONES}, got {num_drones}")
        
        services = cls.INFRASTRUCTURE_SERVICES.copy()
        
        # Handle stats-only profile (PostgreSQL only)
        if profile == Profile.STATS_ONLY:
            services.extend(cls.POSTGRES_SERVICES)
            return services
        
        # Add ROS2 services for profiles that need them
        if profile in [Profile.INTEGRATED, Profile.ROS2_ONLY, Profile.DEVELOPMENT]:
            services.append('ros2-multi-node')
        
        # Add PX4 drone services for profiles that need them
        if profile in [Profile.INTEGRATED, Profile.PX4_ONLY, Profile.DEVELOPMENT]:
            drone_services = [f'px4-bridge-drone-{i}' for i in range(1, num_drones + 1)]
            services.extend(drone_services)
        
        # Add PostgreSQL services only if explicitly requested
        if include_postgres:
            services.extend(cls.POSTGRES_SERVICES)
        
        # Add profile-specific services
        if profile in cls.PROFILE_SERVICES:
            services.extend(cls.PROFILE_SERVICES[profile])
        
        return services
    
    @classmethod
    def get_px4_services_only(cls, num_drones: int) -> List[str]:
        """
        Get only PX4 drone services, excluding all other services.
        
        Args:
            num_drones: Number of PX4 drones (1-6)
            
        Returns:
            List of PX4 drone service names only
            
        Raises:
            ValueError: If drone count is invalid
        """
        if not (cls.MIN_DRONES <= num_drones <= cls.MAX_DRONES):
            raise ValueError(f"Drone count must be between {cls.MIN_DRONES} and {cls.MAX_DRONES}, got {num_drones}")
        
        return [f'px4-bridge-drone-{i}' for i in range(1, num_drones + 1)]
    
    @classmethod
    def get_postgres_services_only(cls) -> List[str]:
        """
        Get only PostgreSQL services, excluding all other services.
        
        Returns:
            List of PostgreSQL service names only
        """
        return cls.POSTGRES_SERVICES.copy()
    
    @classmethod
    def get_environment_variables(cls, num_drones: int, **kwargs) -> Dict[str, str]:
        """
        Generate environment variables for docker-compose based on configuration.
        
        Args:
            num_drones: Number of drones
            **kwargs: Additional environment overrides
            
        Returns:
            Dictionary of environment variables
        """
        env_vars = {
            'SWARM_SIZE': str(num_drones),
            'MAX_DRONES': str(num_drones),
        }
        
        # Add optional environment variables if provided
        optional_vars = {
            'debug': 'DEBUG',
            'ros_domain_id': 'ROS_DOMAIN_ID', 
            'airsim_host': 'AIRSIM_HOST_IP',
            'airsim_port': 'AIRSIM_HOST_PORT',
            'px4_sim_host': 'PX4_SIM_HOSTNAME',
            'launch_mode': 'LAUNCH_MODE',
            'enable_coordination': 'ENABLE_COORDINATION',
            'rpc_timeout': 'RPC_TIMEOUT'
        }
        
        for param, env_var in optional_vars.items():
            if param in kwargs and kwargs[param] is not None:
                if param == 'debug':
                    env_vars[env_var] = 'true' if kwargs[param] else 'false'
                elif param == 'enable_coordination':
                    env_vars[env_var] = 'true' if kwargs[param] else 'false'
                else:
                    env_vars[env_var] = str(kwargs[param])
        
        return env_vars
    
    @classmethod
    def validate_profile(cls, profile_str: str) -> Profile:
        """
        Validate and convert profile string to Profile enum.
        
        Args:
            profile_str: Profile name as string
            
        Returns:
            Profile enum value
            
        Raises:
            ValueError: If profile is invalid
        """
        try:
            return Profile(profile_str)
        except ValueError:
            valid_profiles = [p.value for p in Profile]
            raise ValueError(f"Invalid profile '{profile_str}'. Must be one of: {', '.join(valid_profiles)}")
    
    @classmethod
    def get_profile_description(cls, profile: Profile) -> str:
        """Get human-readable description of profile."""
        descriptions = {
            Profile.INTEGRATED: "Full ecosystem: ROS2 + PX4 drones + PostgreSQL + monitoring",
            Profile.PX4_ONLY: "PX4 SITL drones only (no ROS2)",
            Profile.ROS2_ONLY: "ROS2 multi-node only (no PX4 drones)",
            Profile.DEVELOPMENT: "Development environment with debug tools",
            Profile.STATS_ONLY: "PostgreSQL database services only (for drone statistics)"
        }
        return descriptions.get(profile, "Unknown profile")
    
    @classmethod
    def preview_services(cls, num_drones: int, profile: Profile) -> Dict[str, Any]:
        """
        Preview what services would be started without executing.
        
        Returns:
            Dictionary with services, environment variables, and metadata
        """
        services = cls.get_services_for_config(num_drones, profile)
        env_vars = cls.get_environment_variables(num_drones)
        
        return {
            'services': services,
            'environment': env_vars,
            'profile': profile.value,
            'profile_description': cls.get_profile_description(profile),
            'num_drones': num_drones,
            'total_services': len(services)
        }