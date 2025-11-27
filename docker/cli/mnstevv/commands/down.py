"""
Down command implementation for MNSTEVV CLI

Handles graceful service shutdown with cleanup options.
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
except ImportError as e:
    click.echo(f"ERROR: Failed to import core modules: {e}")
    click.echo("Make sure the mnstevv package is properly installed.")
    sys.exit(1)

@click.command('down')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development', 'stats-only']),
              help='Deployment profile [default: integrated]')
@click.option('--volumes', is_flag=True, default=False,
              help='Remove named volumes declared in volumes section')
@click.option('--images', type=click.Choice(['all', 'local']), default=None,
              help='Remove images (all: all images, local: only locally built)')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.option('--preserve-data', is_flag=True, default=True,
              help='Preserve PostgreSQL data when stopping database services [default: true]')
@click.pass_context
def down_command(ctx, profile, volumes, images, dry_run, preserve_data):
    """
    Stop and remove services gracefully.
    
    The down command stops all running services for the specified profile
    and cleans up containers, networks, and optionally volumes and images.
    
    \b
    Examples:
        mnstevv down                      # Stop integrated profile services
        mnstevv down --profile px4-only   # Stop PX4-only services
        mnstevv down --volumes            # Stop and remove volumes
        mnstevv down --images local       # Stop and remove local images
    """
    try:
        compose_file = ctx.obj['compose_file']
        
        click.echo(f"MNSTEVV - Stopping services")
        click.echo(f"Profile: {profile}")
        
        # Warning for postgres data when volumes flag is used
        if volumes and (profile in ['integrated', 'stats-only'] or 'postgres' in profile.lower()):
            if preserve_data:
                click.echo("WARNING: --volumes flag specified but --preserve-data is enabled")
                click.echo("PostgreSQL data will be preserved. Use --no-preserve-data to remove database volumes.")
                volumes = False  # Override volumes flag to preserve data
            else:
                click.echo("WARNING: PostgreSQL data will be PERMANENTLY DELETED!")
                if not click.confirm("Are you sure you want to continue?"):
                    click.echo("Operation cancelled")
                    return
        
        if volumes:
            click.echo("Will remove volumes")
        if images:
            click.echo(f"Will remove {images} images")
        
        if dry_run:
            click.echo(f"\nDry Run - Would execute:")
            
            cmd_parts = [
                'docker-compose',
                '-f', str(compose_file),
                '--profile', profile,
                'down'
            ]
            
            if volumes:
                cmd_parts.append('-v')
            if images:
                cmd_parts.extend(['--rmi', images])
            cmd_parts.append('--remove-orphans')
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Stop services
        click.echo(f"\nStopping services...")
        
        result = docker_compose.down(
            profile=profile,
            remove_volumes=volumes,
            remove_images=images,
            remove_orphans=True
        )
        
        if result.returncode == 0:
            click.echo(f"Services stopped successfully!")
        else:
            click.echo(f"ERROR: Service shutdown failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"ERROR: Error stopping services: {e}", err=True)
        sys.exit(1)