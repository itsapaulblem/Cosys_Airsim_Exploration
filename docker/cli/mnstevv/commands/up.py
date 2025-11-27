"""
Up command implementation for MNSTEVV CLI

Handles service startup with intelligent service selection based on
--num_drones parameter and deployment profiles.
"""
import click
import sys
from pathlib import Path

# Add core modules to path
core_path = str(Path(__file__).parent.parent / 'core')
if core_path not in sys.path:
    sys.path.insert(0, core_path)

try:
    from service_selector import ServiceSelector, Profile
    from docker_compose import DockerComposeWrapper
except ImportError as e:
    click.echo(f"ERROR: Failed to import core modules: {e}")
    click.echo("Make sure the mnstevv package is properly installed.")
    sys.exit(1)

@click.command('up')
@click.option('--num_drones', default=3, type=click.IntRange(1, 6),
              help='Number of drones to deploy (1-6) [default: 3]')
@click.option('--profile', '-p', default='integrated',
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development', 'stats-only']),
              help='Deployment profile [default: integrated]')
@click.option('--debug', is_flag=True, default=False,
              help='Enable debug logging')
@click.option('--ros-domain-id', type=int, default=0,
              help='ROS domain ID [default: 0]')
@click.option('--airsim-host', default=None,
              help='AirSim host IP (auto-detected if not specified)')
@click.option('--airsim-port', type=int, default=41451,
              help='AirSim host port [default: 41451]')
@click.option('--px4-sim-host', default=None,
              help='PX4 simulation host IP (defaults to airsim-host if not specified)')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.option('--build', is_flag=True, default=False,
              help='Force rebuild of images')
@click.option('--follow-logs', is_flag=True, default=False,
              help='Follow logs after startup')
@click.option('--detached/--no-detached', default=True,
              help='Run in detached mode [default: detached]')
@click.option('--config', default=None,
              help='Use saved configuration profile')
@click.option('--include-postgres', is_flag=True, default=False,
              help='Include PostgreSQL services in deployment')
@click.pass_context
def up_command(ctx, num_drones, profile, debug, ros_domain_id, airsim_host,
               airsim_port, px4_sim_host, dry_run, build, follow_logs, detached, config, include_postgres):
    """
    Start drone swarm services with intelligent service selection.

    The up command automatically determines which services to start based on
    the number of drones and deployment profile. It handles all the complexity
    of docker-compose service selection and environment variable management.

    \b
    Examples:
        mnstevv up --num_drones 2                    # Start 2 drones (integrated)
        mnstevv up --num_drones 4 --profile px4-only # Start 4 PX4 drones only
        mnstevv --platform linux up                  # Linux + YOLO + QGC (auto-enabled)
        mnstevv --platform linux --qgc=false up      # Linux + YOLO, no QGC
        mnstevv --platform linux --yolo=false up     # Linux + QGC, no YOLO
        mnstevv --monitoring --platform linux up     # Full stack with monitoring
        mnstevv up --num_drones 1 --debug            # Single drone with debug
        mnstevv up --num_drones 3 --dry-run          # Preview without executing
        mnstevv up --px4-sim-host 172.24.144.1       # Use specific WSL2 IP for PX4
    """
    try:
        # Get global options from context
        monitoring = ctx.obj.get('monitoring', False)
        platform = ctx.obj.get('platform', None)
        yolo = ctx.obj.get('yolo', None)
        qgc = ctx.obj.get('qgc', None)

        # Auto-detect YOLO setting based on platform if not explicitly set
        if yolo is None:
            yolo = (platform == 'linux')  # Auto-enable YOLO for Linux platform

        # Auto-detect QGC setting based on platform if not explicitly set
        if qgc is None:
            qgc = (platform == 'linux')  # Auto-enable QGC for Linux platform

        # Load configuration profile if specified
        if config:
            from config import get_config_profile
            try:
                config_data = get_config_profile(config)
                # Override parameters with config values (command line takes precedence)
                num_drones = num_drones if ctx.get_parameter_source('num_drones').name != 'DEFAULT' else config_data.get('num_drones', num_drones)
                profile = profile if ctx.get_parameter_source('profile').name != 'DEFAULT' else config_data.get('profile', profile)
                debug = debug if ctx.get_parameter_source('debug').name != 'DEFAULT' else config_data.get('debug', debug)
                ros_domain_id = ros_domain_id if ctx.get_parameter_source('ros_domain_id').name != 'DEFAULT' else config_data.get('ros_domain_id', ros_domain_id)
                airsim_host = airsim_host if airsim_host is not None else config_data.get('airsim_host', airsim_host)
                
                click.echo(f"Using configuration profile: {config}")
            except ValueError as e:
                click.echo(f"ERROR: {e}", err=True)
                sys.exit(1)
        
        # Validate and convert profile
        profile_enum = ServiceSelector.validate_profile(profile)
        
        # If px4_sim_host not specified but airsim_host is, use the same value
        if px4_sim_host is None and airsim_host is not None:
            px4_sim_host = airsim_host
        
        # Get service configuration
        services = ServiceSelector.get_services_for_config(num_drones, profile_enum, include_postgres)
        env_vars = ServiceSelector.get_environment_variables(
            num_drones,
            debug=debug,
            ros_domain_id=ros_domain_id,
            airsim_host=airsim_host,
            airsim_port=airsim_port,
            px4_sim_host=px4_sim_host
        )

        # Add YOLO detection service if enabled
        # Note: YOLO requires ros2-x11-node which is only in linux-integrated profile
        if yolo:
            if platform != 'linux':
                click.echo("WARNING: YOLO requires --platform linux (needs ros2-x11-node). Disabling YOLO.", err=True)
                yolo = False
            elif 'yolov10-detection-service' not in services:
                services.append('yolov10-detection-service')
                # Ensure ros2-x11-node is included
                if 'ros2-x11-node' not in services:
                    services.append('ros2-x11-node')
        elif not yolo and 'yolov10-detection-service' in services:
            services.remove('yolov10-detection-service')

        # Add QGroundControl X11 if enabled
        # Note: QGC requires X11 forwarding which is only available with linux platform
        if qgc:
            if platform != 'linux':
                click.echo("WARNING: QGroundControl requires --platform linux (needs X11 forwarding). Disabling QGC.", err=True)
                qgc = False
            elif 'qgroundcontrol-x11' not in services:
                services.append('qgroundcontrol-x11')
        elif not qgc and 'qgroundcontrol-x11' in services:
            services.remove('qgroundcontrol-x11')

        # Build list of additional compose files
        additional_compose_files = []
        if platform:
            docker_dir = ctx.obj['docker_dir']
            hybrid_override = docker_dir / 'docker-compose-hybrid-override.yml'
            additional_compose_files.append(hybrid_override)

        # Build list of profiles
        active_profiles = [profile]
        if monitoring:
            active_profiles.append('monitoring')
        if platform:
            active_profiles.append(f'{platform}-integrated')

        # Display configuration
        click.echo(f"MNSTEVV - Starting {num_drones} drone{'s' if num_drones > 1 else ''}")
        click.echo(f"Profile: {profile} ({ServiceSelector.get_profile_description(profile_enum)})")
        if monitoring:
            click.echo(f"Monitoring: enabled")
        if platform:
            click.echo(f"Platform: {platform} (using hybrid-override)")
        click.echo(f"YOLO Detection: {'enabled' if yolo else 'disabled'}")
        click.echo(f"QGroundControl: {'enabled' if qgc else 'disabled'}")
        click.echo(f"Active Profiles: {', '.join(active_profiles)}")
        click.echo(f"Services: {', '.join(services)}")

        if debug or dry_run:
            click.echo(f"Environment Variables:")
            for key, value in env_vars.items():
                click.echo(f"   {key}={value}")

        # Initialize Docker Compose wrapper (needed for both dry-run and execution)
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file, additional_compose_files=additional_compose_files)

        if dry_run:
            click.echo(f"\nDry Run - Would execute:")

            # Build the actual command that would be run using the wrapper
            cmd_parts = docker_compose.docker_compose_cmd.copy()
            cmd_parts.extend(['-f', str(compose_file)])

            # Add additional compose files
            for additional_file in additional_compose_files:
                cmd_parts.extend(['-f', str(additional_file)])

            # Add all profiles
            for prof in active_profiles:
                cmd_parts.extend(['--profile', prof])

            # Add environment variables
            for key, value in env_vars.items():
                cmd_parts.extend(['-e', f'{key}={value}'])

            cmd_parts.append('up')
            if detached:
                cmd_parts.append('-d')
            if build:
                cmd_parts.append('--build')
            cmd_parts.append('--remove-orphans')
            cmd_parts.extend(services)

            click.echo(' '.join(cmd_parts))
            return
        
        # Validate compose file
        is_valid, error = docker_compose.validate_compose_file()
        if not is_valid:
            click.echo(f"Docker Compose file validation failed: {error}", err=True)
            sys.exit(1)
        
        # Start services
        click.echo(f"\nStarting services...")

        try:
            result = docker_compose.up(
                services=services,
                profile=None,  # Use profiles list instead
                profiles=active_profiles,
                env_vars=env_vars,
                detached=detached,
                build=build,
                remove_orphans=True
            )
            
            if result.returncode == 0:
                click.echo(f"Services started successfully!")

                # Show running services
                running_services = docker_compose.get_running_services(profile=active_profiles[0] if active_profiles else None)
                if running_services:
                    click.echo(f"\nRunning Services:")
                    for service in running_services:
                        name = service.get('Name', 'unknown')
                        status = service.get('State', 'unknown')
                        ports = service.get('Ports', '')
                        click.echo(f"   • {name}: {status} {ports}")

                # Follow logs if requested
                if follow_logs:
                    click.echo(f"\n📜 Following logs (Ctrl+C to stop)...")
                    try:
                        docker_compose.logs(
                            services=services,
                            profile=active_profiles[0] if active_profiles else None,
                            follow=True,
                            timestamps=True
                        )
                    except KeyboardInterrupt:
                        click.echo(f"\nStopped following logs")
                        
            else:
                click.echo(f"Service startup failed with exit code {result.returncode}", err=True)
                sys.exit(result.returncode)
                
        except Exception as e:
            click.echo(f"Error starting services: {e}", err=True)
            sys.exit(1)
    
    except ValueError as e:
        click.echo(f"Configuration error: {e}", err=True)
        sys.exit(1)
    except Exception as e:
        click.echo(f"Unexpected error: {e}", err=True)
        sys.exit(1)