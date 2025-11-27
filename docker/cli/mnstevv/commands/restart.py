"""
Restart command implementation for MNSTEVV CLI

Handles restarting services to apply configuration changes or recover from issues.
Support both traditional options and new subcommand syntax.
"""
import click
import sys
from pathlib import Path

# Add core modules to path
core_path = str(Path(__file__).parent.parent / 'core')
if core_path not in sys.path:
    sys.path.insert(0, core_path)

try:
    from docker_compose import DockerComposeWrapper
    from service_selector import ServiceSelector
except ImportError as e:
    click.echo(f"ERROR: Failed to import core modules: {e}")
    click.echo("Make sure the mnstevv package is properly installed.")
    sys.exit(1)

@click.group('restart', invoke_without_command=True)
@click.option('--num_drones', type=click.IntRange(1, 6), default=None,
              help='Number of drones to restart (implies profile-based restart)')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile [default: integrated]')
@click.option('--services', default=None,
              help='Comma-separated list of specific services to restart')
@click.option('--timeout', '-t', type=int, default=10,
              help='Timeout for stopping containers in seconds [default: 10]')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.pass_context
def restart_command(ctx, num_drones, profile, services, timeout, dry_run):
    """
    Restart services to apply configuration changes or recover from issues.
    
    This command supports both traditional option-based syntax and new subcommand syntax:
    
    Traditional Options:
        mnstevv restart                           # Restart all integrated services
        mnstevv restart --num_drones 2           # Restart with 2 drones
        mnstevv restart --services px4-drone-1   # Restart specific service
        mnstevv restart --profile px4-only       # Restart PX4-only services
        mnstevv restart --timeout 30             # Allow 30s for graceful shutdown
        
    Subcommands:
        mnstevv restart num_drones 3             # Restart first 3 PX4 drones
        mnstevv restart num_drones 2 --timeout 30  # With additional options
    """
    # If no subcommand was called, execute the traditional restart behavior
    if ctx.invoked_subcommand is None:
        _execute_restart(ctx, num_drones, profile, services, timeout, dry_run)

def _execute_restart(ctx, num_drones, profile, services, timeout, dry_run):
    """Execute the restart logic (shared between main command and subcommands)."""
    try:
        compose_file = ctx.obj['compose_file']
        
        click.echo(f"MNSTEVV - Restarting services")
        click.echo(f"Profile: {profile}")
        
        # Determine services to restart
        if services:
            service_list = [s.strip() for s in services.split(',')]
            click.echo(f"Services: {', '.join(service_list)}")
        elif num_drones:
            # Use service selector to get services for specific drone count
            try:
                profile_enum = ServiceSelector.validate_profile(profile)
                service_list = ServiceSelector.get_services_for_config(num_drones, profile_enum)
                click.echo(f"Drones: {num_drones}")
                click.echo(f"Services: {', '.join(service_list)}")
            except ValueError as e:
                click.echo(f"ERROR: {e}", err=True)
                sys.exit(1)
        else:
            # Get all services for the profile with default drone count
            try:
                profile_enum = ServiceSelector.validate_profile(profile)
                service_list = ServiceSelector.get_services_for_config(3, profile_enum)
                click.echo(f"Services (auto-detected): {', '.join(service_list)}")
            except ValueError:
                service_list = []
                click.echo("Services: All running services")
        
        if timeout != 10:
            click.echo(f"Timeout: {timeout} seconds")
        
        if dry_run:
            click.echo(f"\nDry Run - Would execute:")
            
            cmd_parts = [
                'docker-compose', '-f', str(compose_file),
                '--profile', profile, 'restart'
            ]
            
            if timeout != 10:
                cmd_parts.extend(['-t', str(timeout)])
            
            if service_list:
                cmd_parts.extend(service_list)
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Restart services
        click.echo(f"\nRestarting services...")
        
        # Build restart command
        cmd = ['restart']
        if timeout != 10:
            cmd.extend(['-t', str(timeout)])
        if service_list:
            cmd.extend(service_list)
        
        # Set environment variables if num_drones was specified
        env_vars = None
        if num_drones:
            env_vars = ServiceSelector.get_environment_variables(num_drones)
        
        result = docker_compose.execute(cmd, profile=profile, env_vars=env_vars)
        
        if result.returncode == 0:
            click.echo(f"Services restarted successfully!")
            
            # Show current status
            try:
                running_services = docker_compose.get_running_services(profile=profile)
                if running_services:
                    click.echo(f"\nRunning services: {len(running_services)}")
                    for service in running_services[:5]:  # Show first 5
                        name = service.get('Name', 'unknown')
                        status = service.get('State', 'unknown')
                        health = service.get('Health', '')
                        status_icon = "[RUNNING]" if status == "running" else f"[{status.upper()}]"
                        health_info = f" ({health})" if health else ""
                        click.echo(f"   {status_icon} {name}{health_info}")
                    if len(running_services) > 5:
                        click.echo(f"   ... and {len(running_services) - 5} more")
                else:
                    click.echo(f"\nNo services running")
            except Exception:
                pass  # Don't fail on status check
                
            click.echo(f"\nQuick Actions:")
            click.echo(f"   mnstevv status --detailed    # Check detailed status")
            click.echo(f"   mnstevv logs --follow        # View live logs")
            
        else:
            click.echo(f"ERROR: Service restart failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"ERROR: Error restarting services: {e}", err=True)
        sys.exit(1)

@restart_command.command('num_drones')
@click.argument('count', type=click.IntRange(1, 6))
@click.option('--profile', '-p', default='px4-only', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile [default: px4-only for PX4 drone restart]')
@click.option('--timeout', '-t', type=int, default=10,
              help='Timeout for stopping containers in seconds [default: 10]')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.pass_context  
def num_drones_command(ctx, count, profile, timeout, dry_run):
    """
    Restart specific number of PX4 drones.
    
    This command specifically targets PX4 drone containers for restart,
    restarting the first N PX4 drones (px4-bridge-drone-1, px4-bridge-drone-2, etc.).
    
    Arguments:
        COUNT: Number of PX4 drones to restart (1-6)
        
    Examples:
        mnstevv restart num_drones 3              # Restart first 3 PX4 drones  
        mnstevv restart num_drones 2 --timeout 30 # With custom timeout
        mnstevv restart num_drones 1 --dry-run    # Preview restart actions
    """
    
    click.echo(f"MNSTEVV - Restarting PX4 Drones")
    click.echo(f"Drone Count: {count}")
    click.echo(f"Profile: {profile}")
    
    # Use the shared restart logic but with PX4-specific service selection
    _execute_restart(
        ctx=ctx,
        num_drones=count,
        profile=profile, 
        services=None,  # Let service selector determine PX4 services
        timeout=timeout,
        dry_run=dry_run
    )

@restart_command.command('px4')
@click.option('--count', '-c', type=click.IntRange(1, 6), default=None,
              help='Number of PX4 drones to restart [default: all running PX4 drones]')
@click.option('--timeout', '-t', type=int, default=10,
              help='Timeout for stopping containers in seconds [default: 10]')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.pass_context
def px4_command(ctx, count, timeout, dry_run):
    """
    Restart PX4 drone containers only.
    
    This command exclusively targets PX4 drone containers, ignoring all other services
    like ROS2, monitoring, or infrastructure containers. Perfect for PX4-specific issues.
    
    Options:
        --count: Restart only first N PX4 drones (default: all running)
        
    Examples:
        mnstevv restart px4                       # Restart all PX4 drones
        mnstevv restart px4 --count 3             # Restart first 3 PX4 drones
        mnstevv restart px4 --timeout 30          # With custom timeout
        mnstevv restart px4 --dry-run             # Preview restart actions
    """
    
    try:
        compose_file = ctx.obj['compose_file']
        
        click.echo(f"MNSTEVV - Restarting PX4 Containers Only")
        
        # Determine PX4 services to restart
        if count:
            service_list = ServiceSelector.get_px4_services_only(count)
            click.echo(f"PX4 Drones: {count}")
        else:
            # Get all PX4 services (up to max drones)
            service_list = ServiceSelector.get_px4_services_only(ServiceSelector.MAX_DRONES)
            click.echo(f"PX4 Drones: All running (up to {ServiceSelector.MAX_DRONES})")
        
        click.echo(f"Services: {', '.join(service_list)}")
        
        if timeout != 10:
            click.echo(f"Timeout: {timeout} seconds")
        
        if dry_run:
            click.echo(f"\nDry Run - Would execute:")
            
            cmd_parts = [
                'docker-compose', '-f', str(compose_file),
                'restart'
            ]
            
            if timeout != 10:
                cmd_parts.extend(['-t', str(timeout)])
            
            cmd_parts.extend(service_list)
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Restart PX4 services only
        click.echo(f"\nRestarting PX4 containers...")
        
        cmd = ['restart']
        if timeout != 10:
            cmd.extend(['-t', str(timeout)])
        cmd.extend(service_list)
        
        result = docker_compose.execute(cmd)
        
        if result.returncode == 0:
            click.echo(f"PX4 containers restarted successfully!")
            
            # Show PX4-specific status
            try:
                running_services = docker_compose.get_running_services()
                px4_services = [s for s in running_services if s.get('Name', '').startswith('px4-bridge-drone-')]
                
                if px4_services:
                    click.echo(f"\nRunning PX4 services: {len(px4_services)}")
                    for service in px4_services:
                        name = service.get('Name', 'unknown')
                        status = service.get('State', 'unknown')
                        health = service.get('Health', '')
                        status_icon = "[RUNNING]" if status == "running" else f"[{status.upper()}]"
                        health_info = f" ({health})" if health else ""
                        click.echo(f"   {status_icon} {name}{health_info}")
                else:
                    click.echo(f"\nNo PX4 services running")
            except Exception:
                pass  # Don't fail on status check
                
            click.echo(f"\nQuick Actions:")
            click.echo(f"   mnstevv status --detailed    # Check detailed status")
            click.echo(f"   mnstevv logs px4-bridge-drone-1 --follow  # View PX4 logs")
            
        else:
            click.echo(f"ERROR: PX4 container restart failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"ERROR: Error restarting PX4 containers: {e}", err=True)
        sys.exit(1)