"""
Clean command implementation for MNSTEVV CLI

Provides comprehensive cleanup of containers, volumes, networks, and images.
"""
import click
import sys
import subprocess
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

@click.command('clean')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile to clean [default: integrated]')
@click.option('--volumes', is_flag=True, default=False,
              help='Remove named volumes')
@click.option('--images', is_flag=True, default=False,
              help='Remove built images')
@click.option('--networks', is_flag=True, default=False,
              help='Remove unused networks')
@click.option('--all', 'clean_all', is_flag=True, default=False,
              help='Remove everything (volumes, images, networks)')
@click.option('--docker-system', is_flag=True, default=False,
              help='Run docker system prune after cleanup')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show what would be cleaned without executing')
@click.option('--force', is_flag=True, default=False,
              help='Skip confirmation prompts')
@click.pass_context
def clean_command(ctx, profile, volumes, images, networks, clean_all, docker_system, dry_run, force):
    """
    Clean up Docker resources (containers, volumes, networks, images).
    
    The clean command provides comprehensive cleanup of all Docker resources
    created by MNSTEVV, with granular control over what to remove.
    
    \b
    Examples:
        mnstevv clean                     # Basic cleanup (containers only)
        mnstevv clean --volumes           # Also remove volumes
        mnstevv clean --all               # Remove everything
        mnstevv clean --docker-system     # Also run docker system prune
        mnstevv clean --dry-run           # Preview what would be cleaned
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Set cleanup flags
        if clean_all:
            volumes = images = networks = True
        
        click.echo(f"MNSTEVV Cleanup - Profile: {profile}")
        click.echo("=" * 50)
        
        cleanup_actions = []
        
        # Always stop and remove containers
        cleanup_actions.append("Stop and remove containers")
        
        if volumes:
            cleanup_actions.append("Remove named volumes")
        
        if images:
            cleanup_actions.append("Remove built images")
        
        if networks:
            cleanup_actions.append("Remove unused networks")
        
        if docker_system:
            cleanup_actions.append("Run docker system prune")
        
        click.echo("Cleanup actions:")
        for action in cleanup_actions:
            click.echo(f"   • {action}")
        
        if dry_run:
            click.echo(f"\nDry Run - Commands that would be executed:")
            
            # Show docker-compose down command
            cmd_parts = [
                'docker-compose', '-f', str(compose_file),
                '--profile', profile, 'down'
            ]
            if volumes:
                cmd_parts.append('-v')
            if images:
                cmd_parts.extend(['--rmi', 'local'])
            cmd_parts.append('--remove-orphans')
            
            click.echo(f"   {' '.join(cmd_parts)}")
            
            if networks:
                click.echo(f"   docker network prune -f")
            
            if docker_system:
                click.echo(f"   docker system prune -f")
            
            return
        
        # Confirmation prompt
        if not force:
            click.echo(f"\nWARNING: This will permanently remove Docker resources.")
            if not click.confirm("Continue with cleanup?"):
                click.echo("Cleanup cancelled")
                return
        
        # Execute cleanup
        click.echo(f"\nStarting cleanup...")
        
        # 1. Stop and remove containers with docker-compose down
        click.echo(f"Stopping and removing containers...")
        try:
            result = docker_compose.down(
                profile=profile,
                remove_volumes=volumes,
                remove_images='local' if images else None,
                remove_orphans=True
            )
            
            if result.returncode == 0:
                click.echo(f"Containers cleaned up")
            else:
                click.echo(f"WARNING: Container cleanup completed with warnings")
        except Exception as e:
            click.echo(f"ERROR: Error during container cleanup: {e}", err=True)
        
        # 2. Clean unused networks
        if networks:
            click.echo(f"Removing unused networks...")
            try:
                result = subprocess.run(
                    ['docker', 'network', 'prune', '-f'],
                    capture_output=True,
                    text=True,
                    check=True
                )
                click.echo(f"Networks cleaned up")
            except subprocess.CalledProcessError as e:
                click.echo(f"WARNING: Network cleanup: {e.stderr or str(e)}")
            except Exception as e:
                click.echo(f"ERROR: Error during network cleanup: {e}", err=True)
        
        # 3. Run docker system prune
        if docker_system:
            click.echo(f"Running docker system prune...")
            try:
                result = subprocess.run(
                    ['docker', 'system', 'prune', '-f'],
                    capture_output=True,
                    text=True,
                    check=True
                )
                
                if result.stdout:
                    # Extract space reclaimed from output
                    lines = result.stdout.strip().split('\n')
                    for line in lines:
                        if 'Total reclaimed space' in line:
                            click.echo(f"System cleanup: {line}")
                            break
                    else:
                        click.echo(f"System cleanup completed")
                else:
                    click.echo(f"System cleanup completed (no space reclaimed)")
                    
            except subprocess.CalledProcessError as e:
                click.echo(f"WARNING: System cleanup: {e.stderr or str(e)}")
            except Exception as e:
                click.echo(f"ERROR: Error during system cleanup: {e}", err=True)
        
        # Show final status
        click.echo(f"\nCleanup completed!")
        
        # Check if any containers are still running
        try:
            remaining_services = docker_compose.get_running_services(profile=profile)
            if remaining_services:
                click.echo(f"WARNING: {len(remaining_services)} service(s) still running:")
                for service in remaining_services:
                    name = service.get('Name', 'unknown')
                    click.echo(f"   • {name}")
            else:
                click.echo(f"All services stopped")
        except Exception:
            pass  # Don't fail on status check
        
        click.echo(f"\nQuick Actions:")
        click.echo(f"   mnstevv up --num_drones 3    # Start fresh environment")
        click.echo(f"   mnstevv status               # Check current status")
    
    except Exception as e:
        click.echo(f"ERROR: Error during cleanup: {e}", err=True)
        sys.exit(1)