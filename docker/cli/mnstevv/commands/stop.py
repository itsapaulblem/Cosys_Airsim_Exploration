"""
Stop command implementation for MNSTEVV CLI

Handles graceful container stopping without removing containers (unlike down).
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

@click.command('stop')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile [default: integrated]')
@click.option('--services', default=None,
              help='Comma-separated list of specific services to stop')
@click.option('--timeout', '-t', type=int, default=10,
              help='Timeout for stopping containers in seconds [default: 10]')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.pass_context
def stop_command(ctx, profile, services, timeout, dry_run):
    """
    Stop running containers gracefully without removing them.
    
    Unlike the 'down' command, this stops containers but keeps them for quick restart.
    Useful when you want to temporarily stop services without losing configuration.
    
    Examples:
        mnstevv stop                              # Stop all integrated services
        mnstevv stop --profile px4-only          # Stop PX4-only services
        mnstevv stop --services px4-drone-1,px4-drone-2  # Stop specific services
        mnstevv stop --timeout 30                # Allow 30 seconds for graceful shutdown
    """
    try:
        compose_file = ctx.obj['compose_file']
        
        click.echo(f"MNSTEVV - Stopping services")
        click.echo(f"Profile: {profile}")
        
        # Determine services to stop
        if services:
            service_list = [s.strip() for s in services.split(',')]
            click.echo(f"Services: {', '.join(service_list)}")
        else:
            # Get all services for the profile
            try:
                profile_enum = ServiceSelector.validate_profile(profile)
                service_list = ServiceSelector.get_services_for_config(3, profile_enum)  # Use 3 as default
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
                '--profile', profile, 'stop'
            ]
            
            if timeout != 10:
                cmd_parts.extend(['-t', str(timeout)])
            
            if service_list:
                cmd_parts.extend(service_list)
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Stop services
        click.echo(f"\nStopping services...")
        
        # Build stop command
        cmd = ['stop']
        if timeout != 10:
            cmd.extend(['-t', str(timeout)])
        if service_list:
            cmd.extend(service_list)
        
        result = docker_compose.execute(cmd, profile=profile)
        
        if result.returncode == 0:
            click.echo(f"Services stopped successfully!")
            
            # Show current status
            try:
                remaining_services = docker_compose.get_running_services(profile=profile)
                if remaining_services:
                    click.echo(f"\nRemaining running services: {len(remaining_services)}")
                    for service in remaining_services[:3]:  # Show first 3
                        name = service.get('Name', 'unknown')
                        click.echo(f"   • {name}")
                    if len(remaining_services) > 3:
                        click.echo(f"   • ... and {len(remaining_services) - 3} more")
                else:
                    click.echo(f"\nAll services stopped")
            except Exception:
                pass  # Don't fail on status check
                
            click.echo(f"\nQuick Actions:")
            click.echo(f"   mnstevv up --num_drones 3    # Restart services")
            click.echo(f"   mnstevv status               # Check current status")
            
        else:
            click.echo(f"ERROR: Service stop failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"ERROR: Error stopping services: {e}", err=True)
        sys.exit(1)